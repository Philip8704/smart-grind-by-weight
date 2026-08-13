#include "debug_log.h"

#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <stdarg.h>
#include <string.h>

namespace {

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

    portENTER_CRITICAL(&g_mux);

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
