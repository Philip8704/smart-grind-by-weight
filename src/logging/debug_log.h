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
 * Nothing here writes to flash, so there is no wear and no storage to fill. Both
 * buffers are fixed size and overwrite oldest.
 */

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
