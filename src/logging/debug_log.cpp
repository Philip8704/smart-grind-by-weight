#include "debug_log.h"

#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <LittleFS.h>
#include <stdarg.h>
#include <string.h>
#include "../config/build_info.h"

namespace {

// --- Persistent log staging, PSRAM. Written by any task, drained by the file IO task.
char* g_stage = nullptr;
size_t g_stage_write = 0;
size_t g_stage_filled = 0;
uint32_t g_stage_dropped = 0;           // Bytes overwritten before they reached flash
volatile bool g_grind_active = false;
uint32_t g_idle_hb_window_start = 0;
bool g_idle_hb_window_open = false;

// --- Persistent log files. Touched only by persist_begin (main task, before the file
// IO task exists) and then only by the file IO task - except the previous-boot index,
// which is fixed after begin and read by the report.
bool g_persist_ready = false;
uint32_t g_segment_current = 0;
uint32_t g_segment_count = 0;
int64_t g_previous_boot_segment = -1;
bool g_boot_marker_pending = true;
uint32_t g_last_flush_ms = 0;
uint32_t g_dropped_reported = 0;

// --- Boot log, PSRAM, cleared on every boot ---
char* g_boot_buffer = nullptr;
size_t g_boot_write = 0;
size_t g_boot_filled = 0;
bool g_boot_capture_open = true;

// --- Rolling crash ring, RTC slow memory ---
// RTC_NOINIT_ATTR survives a software reset, a panic and a watchdog reset. It is only
// indeterminate after a true power-on, which the magic value below detects.
RTC_NOINIT_ATTR static char g_rtc_ring[DEBUG_CRASH_LOG_BYTES];
RTC_NOINIT_ATTR static uint32_t g_rtc_write;
RTC_NOINIT_ATTR static uint32_t g_rtc_filled;
RTC_NOINIT_ATTR static uint32_t g_rtc_magic;

constexpr uint32_t kRtcMagic = 0x43524148;  // "CRAH"

// --- Preserved copy of the previous session's ring, only when it crashed ---
char* g_dump_buffer = nullptr;
size_t g_dump_size = 0;
const char* g_dump_reason = nullptr;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

void append_ring(char* ring, size_t capacity, size_t& write_index, size_t& filled,
                 const char* text, size_t length) {
    for (size_t i = 0; i < length; i++) {
        ring[write_index] = text[i];
        write_index = (write_index + 1) % capacity;
        if (filled < capacity) {
            filled++;
        }
    }
}

// Copies out of a ring, oldest first, from an offset. Shared by both readers.
size_t copy_from_ring(const char* ring, size_t capacity, size_t write_index, size_t filled,
                      size_t offset, char* out, size_t out_size) {
    if (!out || out_size == 0) {
        return 0;
    }
    out[0] = 0;
    if (!ring || offset >= filled) {
        return 0;
    }

    size_t start = (write_index + capacity - filled) % capacity;
    size_t remaining = filled - offset;
    size_t to_copy = (remaining < out_size - 1) ? remaining : out_size - 1;
    for (size_t i = 0; i < to_copy; i++) {
        out[i] = ring[(start + offset + i) % capacity];
    }
    out[to_copy] = 0;
    return to_copy;
}

// Staging ring: unlike the crash ring, bytes are CONSUMED by the flush, and a byte
// overwritten before it was flushed is counted as lost rather than silently gone.
// Caller holds g_mux.
void stage_append(const char* text, size_t length) {
    for (size_t i = 0; i < length; i++) {
        if (g_stage_filled == DEBUG_PERSIST_STAGING_BYTES) {
            g_stage_dropped++;
        } else {
            g_stage_filled++;
        }
        g_stage[g_stage_write] = text[i];
        g_stage_write = (g_stage_write + 1) % DEBUG_PERSIST_STAGING_BYTES;
    }
}

// Caller holds g_mux. Removes up to max bytes, oldest first.
size_t stage_take(char* out, size_t max) {
    size_t n = (g_stage_filled < max) ? g_stage_filled : max;
    size_t start = (g_stage_write + DEBUG_PERSIST_STAGING_BYTES - g_stage_filled) % DEBUG_PERSIST_STAGING_BYTES;
    for (size_t i = 0; i < n; i++) {
        out[i] = g_stage[(start + i) % DEBUG_PERSIST_STAGING_BYTES];
    }
    g_stage_filled -= n;
    return n;
}

void segment_path(uint32_t index, char* out, size_t out_size) {
    snprintf(out, out_size, "%s/log_%05lu.txt", DEBUG_PERSIST_DIR, (unsigned long)index);
}

bool parse_segment_index(const char* name, uint32_t* index_out) {
    const char* base = strrchr(name, '/');
    base = base ? base + 1 : name;
    unsigned long value = 0;
    if (sscanf(base, "log_%lu.txt", &value) != 1) {
        return false;
    }
    *index_out = (uint32_t)value;
    return true;
}

bool scan_segments(uint32_t* min_out, uint32_t* max_out, uint32_t* count_out) {
    *count_out = 0;
    File dir = LittleFS.open(DEBUG_PERSIST_DIR);
    if (!dir || !dir.isDirectory()) {
        return false;
    }
    File entry = dir.openNextFile();
    while (entry) {
        uint32_t index;
        if (!entry.isDirectory() && parse_segment_index(entry.name(), &index)) {
            if (*count_out == 0 || index < *min_out) *min_out = index;
            if (*count_out == 0 || index > *max_out) *max_out = index;
            (*count_out)++;
        }
        entry = dir.openNextFile();
    }
    return true;
}

size_t littlefs_free() {
    size_t total = LittleFS.totalBytes();
    size_t used = LittleFS.usedBytes();
    return (used < total) ? total - used : 0;
}

// Deletes the oldest segment, never the one being written. Returns false if there is
// nothing older left to delete.
bool delete_oldest_segment() {
    uint32_t min_index = 0, max_index = 0, count = 0;
    if (!scan_segments(&min_index, &max_index, &count) || count == 0 || min_index == g_segment_current) {
        return false;
    }
    char path[48];
    segment_path(min_index, path, sizeof(path));
    bool removed = LittleFS.remove(path);
    g_segment_count = removed ? count - 1 : count;
    return removed;
}

// File IO task only. Appends to the current segment, rolling over and making room
// first - oldest deleted before newest written.
void persist_write(const char* data, size_t length) {
    if (length == 0) {
        return;
    }
    char path[48];
    segment_path(g_segment_current, path, sizeof(path));
    File probe = LittleFS.open(path, "r");
    size_t current_size = probe ? probe.size() : 0;
    bool exists = (bool)probe;
    if (probe) probe.close();

    if (exists && current_size + length > DEBUG_PERSIST_SEGMENT_BYTES) {
        g_segment_current++;
        segment_path(g_segment_current, path, sizeof(path));
        exists = false;
    }
    if (!exists) {
        while (g_segment_count >= DEBUG_PERSIST_MAX_SEGMENTS) {
            if (!delete_oldest_segment()) break;
        }
    }
    while (littlefs_free() < DEBUG_PERSIST_MIN_FREE_BYTES + length) {
        if (delete_oldest_segment()) {
            continue;
        }
        // Only the current segment is left. Start a fresh one and delete the current,
        // so the log keeps its newest lines instead of freezing on its oldest.
        if (exists) {
            char old_path[48];
            segment_path(g_segment_current, old_path, sizeof(old_path));
            g_segment_current++;
            segment_path(g_segment_current, path, sizeof(path));
            if (LittleFS.remove(old_path) && g_segment_count > 0) {
                g_segment_count--;
            }
            exists = false;
            continue;
        }
        return;  // No log files left to free - drop rather than fill the partition
    }

    File file = LittleFS.open(path, "a");
    if (!file) {
        return;
    }
    size_t written = file.write(reinterpret_cast<const uint8_t*>(data), length);
    file.close();
    if (!exists && written > 0) {
        g_segment_count++;
    }
}

// Only these leave the previous session's log worth keeping. A clean restart - an OTA
// reboot, say - is not a crash and its tail is noise.
bool reset_reason_is_crash(esp_reset_reason_t reason, const char** name_out) {
    switch (reason) {
        case ESP_RST_PANIC:    *name_out = "PANIC (exception)";      return true;
        case ESP_RST_INT_WDT:  *name_out = "INT_WDT (interrupt)";    return true;
        case ESP_RST_TASK_WDT: *name_out = "TASK_WDT (task hang)";   return true;
        case ESP_RST_WDT:      *name_out = "WDT (other watchdog)";   return true;
        case ESP_RST_BROWNOUT: *name_out = "BROWNOUT (supply dip)";  return true;
        default:               *name_out = nullptr;                  return false;
    }
}

}  // namespace

void debug_log_init() {
    if (g_boot_buffer) {
        return;
    }

    // Decide the fate of whatever the previous session left in RTC memory, before any
    // new logging can overwrite it.
    const char* crash_name = nullptr;
    bool previous_crashed = reset_reason_is_crash(esp_reset_reason(), &crash_name);
    bool ring_valid = (g_rtc_magic == kRtcMagic) && (g_rtc_filled <= DEBUG_CRASH_LOG_BYTES) &&
                      (g_rtc_write < DEBUG_CRASH_LOG_BYTES);

    if (previous_crashed && ring_valid && g_rtc_filled > 0) {
        g_dump_buffer = static_cast<char*>(
            heap_caps_malloc_prefer(g_rtc_filled + 1, 2,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_8BIT));
        if (g_dump_buffer) {
            g_dump_size = copy_from_ring(g_rtc_ring, DEBUG_CRASH_LOG_BYTES, g_rtc_write,
                                         g_rtc_filled, 0, g_dump_buffer, g_rtc_filled + 1);
            g_dump_reason = crash_name;
        }
    }

    // Start the ring fresh for this session regardless of what happened
    g_rtc_write = 0;
    g_rtc_filled = 0;
    g_rtc_magic = kRtcMagic;

    g_boot_buffer = static_cast<char*>(
        heap_caps_malloc_prefer(DEBUG_BOOT_LOG_BYTES, 2,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_8BIT));
    if (!g_boot_buffer) {
        g_boot_capture_open = false;  // Not fatal - serial still works, just not retained
    }

    // Staging starts now so the persistent log includes startup, even though flash
    // cannot be written until LittleFS is mounted and the file IO task is running
    g_stage = static_cast<char*>(
        heap_caps_malloc_prefer(DEBUG_PERSIST_STAGING_BYTES, 2,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_8BIT));
}

void debug_log_printf(const char* format, ...) {
    char line[256];

    va_list args;
    va_start(args, format);
    int written = vsnprintf(line, sizeof(line), format, args);
    va_end(args);

    if (written < 0) {
        return;
    }
    // vsnprintf reports what it WOULD have written, so clamp to what fit
    size_t length = (static_cast<size_t>(written) < sizeof(line))
                        ? static_cast<size_t>(written)
                        : sizeof(line) - 1;

    Serial.print(line);

    // Decided outside the lock. Heartbeats are tagged "[<ms> <TASK>_HEARTBEAT]" or
    // "[<ms> TASK_HEARTBEAT_<name>]", so look for HEARTBEAT inside the leading tag
    // rather than for one exact suffix - matching "_HEARTBEAT]" missed the second form.
    bool is_heartbeat = false;
    if (line[0] == '[') {
        const char* tag_end = strchr(line, ']');
        const char* found = strstr(line, "HEARTBEAT");
        is_heartbeat = tag_end && found && found < tag_end;
    }

    portENTER_CRITICAL(&g_mux);

    if (g_stage) {
        bool persist = true;
        if (is_heartbeat && !g_grind_active) {
            // Idle: keep one full round of task heartbeats (each task logs once per
            // 10s) every DEBUG_PERSIST_IDLE_HEARTBEAT_INTERVAL_MS
            uint32_t now = millis();
            if (!g_idle_hb_window_open ||
                now - g_idle_hb_window_start >= DEBUG_PERSIST_IDLE_HEARTBEAT_INTERVAL_MS) {
                g_idle_hb_window_start = now;
                g_idle_hb_window_open = true;
            }
            persist = (now - g_idle_hb_window_start) < 10000;
        }
        if (persist) {
            stage_append(line, length);
        }
    }

    // Always roll the crash ring - this is the window that matters when it stops
    if (g_rtc_magic == kRtcMagic) {
        size_t write_index = g_rtc_write;
        size_t filled = g_rtc_filled;
        append_ring(g_rtc_ring, DEBUG_CRASH_LOG_BYTES, write_index, filled, line, length);
        g_rtc_write = write_index;
        g_rtc_filled = filled;
    }

    // Boot buffer only while its window is open
    if (g_boot_capture_open && g_boot_buffer) {
        if (millis() > DEBUG_BOOT_LOG_WINDOW_MS) {
            g_boot_capture_open = false;
        } else {
            append_ring(g_boot_buffer, DEBUG_BOOT_LOG_BYTES, g_boot_write, g_boot_filled,
                        line, length);
        }
    }

    portEXIT_CRITICAL(&g_mux);
}

size_t debug_log_boot_log_size() {
    if (!g_boot_buffer) {
        return 0;
    }
    portENTER_CRITICAL(&g_mux);
    size_t filled = g_boot_filled;
    portEXIT_CRITICAL(&g_mux);
    return filled;
}

size_t debug_log_copy_boot_log(size_t offset, char* out, size_t out_size) {
    if (!g_boot_buffer) {
        if (out && out_size > 0) out[0] = 0;
        return 0;
    }
    portENTER_CRITICAL(&g_mux);
    size_t write_index = g_boot_write;
    size_t filled = g_boot_filled;
    portEXIT_CRITICAL(&g_mux);

    return copy_from_ring(g_boot_buffer, DEBUG_BOOT_LOG_BYTES, write_index, filled,
                          offset, out, out_size);
}

bool debug_log_boot_capture_finished() {
    return !g_boot_capture_open || millis() > DEBUG_BOOT_LOG_WINDOW_MS;
}

bool debug_log_has_crash_dump() {
    return g_dump_buffer != nullptr && g_dump_size > 0;
}

const char* debug_log_crash_reason() {
    return g_dump_reason ? g_dump_reason : "none";
}

size_t debug_log_crash_dump_size() {
    return g_dump_size;
}

size_t debug_log_copy_crash_dump(size_t offset, char* out, size_t out_size) {
    if (!out || out_size == 0) {
        return 0;
    }
    out[0] = 0;
    if (!g_dump_buffer || offset >= g_dump_size) {
        return 0;
    }

    // Already a flat, oldest-first copy - no ring arithmetic needed
    size_t remaining = g_dump_size - offset;
    size_t to_copy = (remaining < out_size - 1) ? remaining : out_size - 1;
    memcpy(out, g_dump_buffer + offset, to_copy);
    out[to_copy] = 0;
    return to_copy;
}

//------------------------------------------------------------------------------
// Persistent log
//------------------------------------------------------------------------------

const char* debug_log_reset_reason_name() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "POWER-ON (power cycle)";
        case ESP_RST_EXT:       return "EXTERNAL (reset pin)";
        case ESP_RST_SW:        return "SOFTWARE (restart / update)";
        case ESP_RST_PANIC:     return "PANIC (exception)";
        case ESP_RST_INT_WDT:   return "INT_WDT (interrupt watchdog)";
        case ESP_RST_TASK_WDT:  return "TASK_WDT (task hang)";
        case ESP_RST_WDT:       return "WDT (other watchdog)";
        case ESP_RST_DEEPSLEEP: return "DEEP SLEEP wake";
        case ESP_RST_BROWNOUT:  return "BROWNOUT (supply dip)";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

void debug_log_persist_begin() {
    if (!g_stage) {
        return;
    }
    if (!LittleFS.exists(DEBUG_PERSIST_DIR)) {
        LittleFS.mkdir(DEBUG_PERSIST_DIR);
    }
    uint32_t min_index = 0, max_index = 0, count = 0;
    scan_segments(&min_index, &max_index, &count);
    if (count > 0) {
        // Every boot writes at least its marker, so the newest segment on disk is the
        // end of the previous boot - including one that ended in a manual restart
        g_previous_boot_segment = max_index;
        g_segment_current = max_index + 1;
    } else {
        g_previous_boot_segment = -1;
        g_segment_current = 0;
    }
    g_segment_count = count;
    g_boot_marker_pending = true;
    g_last_flush_ms = millis();
    g_persist_ready = true;
}

void debug_log_persist_service() {
    if (!g_persist_ready || !g_stage) {
        return;
    }

    portENTER_CRITICAL(&g_mux);
    size_t pending = g_stage_filled;
    uint32_t dropped = g_stage_dropped;
    portEXIT_CRITICAL(&g_mux);

    uint32_t now = millis();
    bool due = pending >= DEBUG_PERSIST_FLUSH_THRESHOLD_BYTES ||
               (pending > 0 && now - g_last_flush_ms >= DEBUG_PERSIST_FLUSH_INTERVAL_MS);
    if (!due) {
        return;
    }
    g_last_flush_ms = now;

    if (g_boot_marker_pending) {
        g_boot_marker_pending = false;
        char marker[160];
        int n = snprintf(marker, sizeof(marker),
                         "\n===== BOOT | reset: %s | build #%d | segment %lu =====\n",
                         debug_log_reset_reason_name(), BUILD_NUMBER,
                         (unsigned long)g_segment_current);
        if (n > 0) {
            persist_write(marker, (size_t)n < sizeof(marker) ? (size_t)n : sizeof(marker) - 1);
        }
    }

    if (dropped != g_dropped_reported) {
        char note[80];
        int n = snprintf(note, sizeof(note), "[log] %lu bytes were lost before reaching flash\n",
                         (unsigned long)(dropped - g_dropped_reported));
        g_dropped_reported = dropped;
        if (n > 0) {
            persist_write(note, (size_t)n < sizeof(note) ? (size_t)n : sizeof(note) - 1);
        }
    }

    // PSRAM, allocated once: file IO task only, too big for its stack, and internal RAM
    // is the heap that actually runs out on this board
    constexpr size_t kChunkBytes = 2048;
    static char* chunk = nullptr;
    if (!chunk) {
        chunk = static_cast<char*>(heap_caps_malloc_prefer(kChunkBytes, 2,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_8BIT));
        if (!chunk) return;
    }
    for (int round = 0; round < 8; round++) {
        portENTER_CRITICAL(&g_mux);
        size_t n = stage_take(chunk, kChunkBytes);
        portEXIT_CRITICAL(&g_mux);
        if (n == 0) {
            break;
        }
        persist_write(chunk, n);
    }
}

void debug_log_set_grind_active(bool active) {
    g_grind_active = active;
}

size_t debug_log_copy_previous_boot_tail(char* out, size_t out_size) {
    if (!out || out_size == 0) {
        return 0;
    }
    out[0] = 0;
    if (g_previous_boot_segment < 0 || out_size < 2) {
        return 0;
    }
    char path[48];
    segment_path((uint32_t)g_previous_boot_segment, path, sizeof(path));
    File file = LittleFS.open(path, "r");
    if (!file) {
        return 0;  // Rotated away since boot
    }
    size_t size = file.size();
    size_t want = out_size - 1;
    size_t start = (size > want) ? size - want : 0;
    file.seek(start);
    size_t got = file.read(reinterpret_cast<uint8_t*>(out), want);
    file.close();
    out[got] = 0;

    // Started mid-file: skip the partial first line so the tail reads cleanly
    if (start > 0) {
        char* newline = strchr(out, '\n');
        if (newline && newline[1]) {
            size_t skip = (size_t)(newline + 1 - out);
            memmove(out, newline + 1, got - skip + 1);
            got -= skip;
        }
    }
    return got;
}

uint32_t debug_log_persist_segment_count() {
    return g_segment_count;
}

uint32_t debug_log_persist_dropped_bytes() {
    portENTER_CRITICAL(&g_mux);
    uint32_t dropped = g_stage_dropped;
    portEXIT_CRITICAL(&g_mux);
    return dropped;
}
