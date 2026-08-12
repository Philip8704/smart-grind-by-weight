#pragma once

#include <Arduino.h>
#include <stddef.h>

/*
 * Boot log capture
 *
 * Everything logged through LOG_BLE goes to the USB serial port and nowhere else,
 * so anything printed before a phone can connect - which is all of startup - was
 * simply lost. Startup is exactly where the interesting facts are: whether the
 * calibration and empty-cradle reference loaded, whether the load cell validated,
 * what sample rate was detected.
 *
 * This keeps a copy of the first few seconds in a ring buffer so the BLE
 * diagnostic report can hand it back later. Capture stops on its own once the
 * window closes, after which logging costs one comparison.
 */

// How long after boot to keep capturing, and how much to keep. The buffer lives in
// PSRAM when available; the window is short because startup is the only part that
// cannot be observed any other way.
#define DEBUG_BOOT_LOG_WINDOW_MS 10000
#define DEBUG_BOOT_LOG_BYTES 6144

// Allocates the buffer. Safe to call before anything else; logging works without it,
// it just is not retained.
void debug_log_init();

// Writes to Serial and, while the boot window is open, to the ring buffer.
// Safe to call from any task - the ring is guarded by a spinlock.
void debug_log_printf(const char* format, ...) __attribute__((format(printf, 1, 2)));

// Total bytes currently held.
size_t debug_log_boot_log_size();

// Copies a slice of the captured text, oldest first, NUL-terminated. Reading in slices
// lets the caller stream it out without allocating a copy of the whole buffer. Returns
// bytes written excluding the terminator; zero once offset passes the end.
size_t debug_log_copy_boot_log(size_t offset, char* out, size_t out_size);

// True once the capture window has closed.
bool debug_log_boot_capture_finished();
