#pragma once
#include <Arduino.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_encoder.h>
#include <functional>
#include "../config/constants.h"

// Forward declarations
struct GrindEventData;
enum class UIGrindEvent;

class Grinder {
private:
    int motor_pin;
    bool grinding;
    bool initialized;

    // RMT pulse control
    rmt_channel_handle_t rmt_channel;
    rmt_encoder_handle_t current_encoder;
    bool pulse_active;
    bool rmt_initialized;

    // Motor settling tracking
    unsigned long motor_start_time;

    // Motor fault visibility. Every actuation path is gated on `initialized` and
    // `rmt_initialized`, and both are set only if the RMT channel came up at boot. If
    // it did not, every start/stop/pulse in every mode returns silently - weight, time,
    // motor test and autotune alike - with no error anywhere. These record why, so a
    // dead motor shows up as one line in the diagnostic report rather than as a
    // grinder that simply does nothing for no stated reason.
    esp_err_t rmt_init_err;          // Result of rmt_new_tx_channel at boot
    esp_err_t last_transmit_err;     // Result of the most recent rmt_transmit
    uint32_t transmit_fail_count;    // Transmits that returned anything but ESP_OK
    uint32_t encoder_fail_count;     // Times an encoder could not be created

    // Pulse train (chute knock test). Symbols must outlive the transmission, so they
    // live here rather than on the caller's stack.
    static constexpr size_t TRAIN_SYMBOL_CAPACITY = 96;
    rmt_symbol_word_t train_symbols_[TRAIN_SYMBOL_CAPACITY];
    bool train_active_ = false;
    unsigned long train_end_ms_ = 0;     // When the train's last LOW period ends

    // Background indicator state (always compiled in)
    bool background_active;
    std::function<void(const GrindEventData&)> ui_event_callback;

    void emit_background_change(bool active);

public:
    void init(int pin);
    void start();
    void stop();
    
    // RMT-based precise pulse control
    void start_pulse_rmt(uint32_t duration_ms);
    bool is_pulse_complete();

    // `count` pulses of on_ms with off_ms between, timed entirely by the RMT in one
    // transmission so every period is exact. Core 0 only, like every motor call.
    // Returns false (motor untouched) if it could not be started.
    bool start_pulse_train(uint32_t on_ms, uint32_t off_ms, uint8_t count);
    // True once the train has finished. Time-based: the train's length is exact, and
    // polling the driver with no timeout logs an error on every call until it is done.
    bool is_pulse_train_done();
    
    bool is_grinding() const { return grinding; }
    bool is_initialized() const { return initialized; }
    bool is_motor_settled() const;

    // Fault accessors for the diagnostic report
    bool is_rmt_ready() const { return rmt_initialized; }
    esp_err_t get_rmt_init_error() const { return rmt_init_err; }
    esp_err_t get_last_transmit_error() const { return last_transmit_err; }
    uint32_t get_transmit_fail_count() const { return transmit_fail_count; }
    uint32_t get_encoder_fail_count() const { return encoder_fail_count; }
    int get_motor_pin() const { return motor_pin; }

    // Background indicator setup (always compiled in)
    void set_ui_event_callback(const std::function<void(const GrindEventData&)>& callback);
};
