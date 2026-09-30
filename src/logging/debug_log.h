#pragma once

#include <Arduino.h>
#include <stddef.h>

/*
 * Log capture: boot log and crash dump
 *
 * Everything logged through LOG_BLE goes to the USB serial port and nowhere else,
 * so anything printed without a cable attached is lost. Two windows matter and
 * neither can be watched live over BLE:
 *
 *  - Startup, because nothing is connected yet. Kept in a PSRAM buffer for the
 *    first few seconds.
 *
 *  - The moments before a crash. Kept in a rolling ring in RTC memory, which is
 *    a separate region from the heap and PSRAM and, crucially, is NOT cleared by
 *    a panic or watchdog reset. Nothing is written anywhere while running: the
 *    ring simply rolls. On the next boot the reset reason is examined, and only
 *    if it was a crash is the surviving content kept and offered for download.
 *    A clean restart discards it.
 *
 * Both of those are lost on a manual restart or power cycle - which is exactly how a
 * hang gets cleared - so there is also a persistent log:
 *
 *  - Every line is staged in a PSRAM ring and the file IO task (Core 1) appends it to
 *    LittleFS every few seconds. Nothing touches flash from the caller's context.
 *    Files are fixed-size segments under /logs; the oldest segment is deleted before
 *    a new one is started, and before any write that would bring free space below a
 *    floor, so the log can never fill the partition or crowd out grind sessions.
 *    Each boot starts its own segment behind a marker line with the reset reason.
 *
 *  - Task heartbeats (one line per task every 10s) would otherwise be most of it and
 *    push everything useful out within an hour. They are kept in full while a grind is
 *    running - the grind heartbeat shows phase and weight, which is what explains a
 *    stall - and one set every DEBUG_PERSIST_IDLE_HEARTBEAT_INTERVAL_MS otherwise.
 */

// Persistent log (LittleFS)
#define DEBUG_PERSIST_DIR "/logs"
#define DEBUG_PERSIST_STAGING_BYTES 16384                  // PSRAM ring awaiting flush
#define DEBUG_PERSIST_SEGMENT_BYTES (32 * 1024)            // One file
#define DEBUG_PERSIST_MAX_SEGMENTS 8                       // 256KB total - ~2-3 days of normal use; sessions get the rest
#define DEBUG_PERSIST_FLUSH_INTERVAL_MS 5000               // Lines reach flash within this
#define DEBUG_PERSIST_FLUSH_THRESHOLD_BYTES 4096           // ...or sooner once this much is waiting
#define DEBUG_PERSIST_MIN_FREE_BYTES (64 * 1024)           // Never write the partition below this
#define DEBUG_PERSIST_IDLE_HEARTBEAT_INTERVAL_MS 600000    // Idle: one heartbeat set per 10 minutes
#define DEBUG_PREVIOUS_BOOT_TAIL_BYTES 3072                // Shown in the diagnostic report

// Boot window: how long to capture and how much to keep (PSRAM).
#define DEBUG_BOOT_LOG_WINDOW_MS 10000
#define DEBUG_BOOT_LOG_BYTES 6144

// Rolling crash ring (RTC slow memory, survives a panic reset but not power loss).
// Kept modest because this region is only ~8KB in total and is shared with the system.
#define DEBUG_CRASH_LOG_BYTES 3072

// Allocates buffers and decides whether the previous boot left a crash dump.
// Call before anything else logs.
void debug_log_init();

// Writes to Serial, to the boot buffer while that window is open, and always to the
// rolling crash ring. Safe from any task - both rings are guarded by a spinlock.
void debug_log_printf(const char* format, ...) __attribute__((format(printf, 1, 2)));

// --- Boot log (this session) ---
size_t debug_log_boot_log_size();
size_t debug_log_copy_boot_log(size_t offset, char* out, size_t out_size);
bool debug_log_boot_capture_finished();

// --- Crash dump (previous session, only present if that session ended badly) ---
bool debug_log_has_crash_dump();
const char* debug_log_crash_reason();      // Reset reason that produced the dump
size_t debug_log_crash_dump_size();
size_t debug_log_copy_crash_dump(size_t offset, char* out, size_t out_size);

// --- Persistent log ---
// Call once after LittleFS is mounted, before the file IO task starts: finds the
// previous boot's segments and picks this boot's.
void debug_log_persist_begin();

// File IO task only. Appends staged lines to flash when due; cheap when nothing is.
void debug_log_persist_service();

// Heartbeats are persisted in full while a grind is active (see above)
void debug_log_set_grind_active(bool active);

// Reset reason of THIS boot, for every reason including power-on and manual reset
const char* debug_log_reset_reason_name();

// Last bytes the previous boot wrote to the persistent log - what a manual restart
// after a hang would otherwise have erased. Returns bytes copied (NUL-terminated).
size_t debug_log_copy_previous_boot_tail(char* out, size_t out_size);
uint32_t debug_log_persist_segment_count();
uint32_t debug_log_persist_dropped_bytes();
