#include "debug_log.h"

#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <stdarg.h>
#include <string.h>

namespace {

char* g_buffer = nullptr;
size_t g_write_index = 0;   // Next byte to write
size_t g_filled = 0;        // Bytes held, up to DEBUG_BOOT_LOG_BYTES
bool g_capture_open = true;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// Appends under the lock. Kept deliberately small: it runs from the control loop and
// from BLE callbacks, so it must not allocate, block, or take long.
void append_locked(const char* text, size_t length) {
    for (size_t i = 0; i < length; i++) {
        g_buffer[g_write_index] = text[i];
        g_write_index = (g_write_index + 1) % DEBUG_BOOT_LOG_BYTES;
        if (g_filled < DEBUG_BOOT_LOG_BYTES) {
            g_filled++;
        }
    }
}

}  // namespace

void debug_log_init() {
    if (g_buffer) {
        return;
    }
    g_buffer = static_cast<char*>(
        heap_caps_malloc_prefer(DEBUG_BOOT_LOG_BYTES, 2,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT,
                                MALLOC_CAP_8BIT));
    if (!g_buffer) {
        // Not fatal - logging still reaches the serial port, it is just not retained
        g_capture_open = false;
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
    // vsnprintf returns what it WOULD have written, so clamp to what fit
    size_t length = (static_cast<size_t>(written) < sizeof(line))
                        ? static_cast<size_t>(written)
                        : sizeof(line) - 1;

    Serial.print(line);

    if (!g_capture_open || !g_buffer) {
        return;
    }
    if (millis() > DEBUG_BOOT_LOG_WINDOW_MS) {
        g_capture_open = false;  // Window closed; from here this is a single compare
        return;
    }

    portENTER_CRITICAL(&g_mux);
    append_locked(line, length);
    portEXIT_CRITICAL(&g_mux);
}

size_t debug_log_boot_log_size() {
    if (!g_buffer) {
        return 0;
    }
    portENTER_CRITICAL(&g_mux);
    size_t filled = g_filled;
    portEXIT_CRITICAL(&g_mux);
    return filled;
}

size_t debug_log_copy_boot_log(size_t offset, char* out, size_t out_size) {
    if (!out || out_size == 0) {
        return 0;
    }
    out[0] = 0;

    if (!g_buffer) {
        return 0;
    }

    portENTER_CRITICAL(&g_mux);
    size_t filled = g_filled;
    size_t start = (g_write_index + DEBUG_BOOT_LOG_BYTES - filled) % DEBUG_BOOT_LOG_BYTES;
    portEXIT_CRITICAL(&g_mux);

    if (offset >= filled) {
        return 0;
    }

    size_t remaining = filled - offset;
    size_t to_copy = (remaining < out_size - 1) ? remaining : out_size - 1;
    for (size_t i = 0; i < to_copy; i++) {
        out[i] = g_buffer[(start + offset + i) % DEBUG_BOOT_LOG_BYTES];
    }
    out[to_copy] = 0;
    return to_copy;
}

bool debug_log_boot_capture_finished() {
    return !g_capture_open || millis() > DEBUG_BOOT_LOG_WINDOW_MS;
}
