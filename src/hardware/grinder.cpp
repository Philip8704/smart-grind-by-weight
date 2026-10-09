#include "grinder.h"
#include "../controllers/grind_events.h"
#include "../config/constants.h"
#include "pulse_train.h"
#if DEBUG_ENABLE_LOADCELL_MOCK
#include "mock_hx711_driver.h"
#endif

void Grinder::init(int pin) {
    motor_pin = pin;
    grinding = false;
    pulse_active = false;
    rmt_initialized = false;
    current_encoder = nullptr;
    motor_start_time = 0;

    rmt_init_err = ESP_OK;
    last_transmit_err = ESP_OK;
    transmit_fail_count = 0;
    encoder_fail_count = 0;

    // Initialize background indicator
    background_active = false;
    ui_event_callback = nullptr;

#if DEBUG_ENABLE_LOADCELL_MOCK
    initialized = true;
    return;
#endif
    
    // Initialize RMT for all motor control (both continuous and pulse)
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = (gpio_num_t)motor_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1MHz resolution = 1µs per tick
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    
    // Logged on both paths on purpose. A failure here disables the motor everywhere -
    // weight, time, motor test, autotune - because every actuation path returns early
    // on !rmt_initialized. Without a line at boot that condition is indistinguishable
    // from a wiring fault, and the report had no way to tell them apart.
    rmt_init_err = rmt_new_tx_channel(&tx_chan_config, &rmt_channel);
    if (rmt_init_err == ESP_OK) {
        rmt_enable(rmt_channel);
        rmt_initialized = true;
        initialized = true;
        LOG_BLE("[MOTOR] RMT ready on GPIO %d - motor control available\n", motor_pin);
    } else {
        LOG_BLE("[MOTOR] RMT channel FAILED on GPIO %d: %s - motor is dead in ALL modes\n",
                motor_pin, esp_err_to_name(rmt_init_err));
    }
}

void Grinder::start() {
#if DEBUG_ENABLE_LOADCELL_MOCK
    if (!initialized) return;
    MockHX711Driver::notify_grinder_start();
    pulse_active = false;
    grinding = true;
    motor_start_time = millis();
    emit_background_change(true);
    return;
#endif
    if (!initialized || !rmt_initialized) return;

    // Reset any active pulse state when using continuous mode
    pulse_active = false;
    motor_start_time = millis();
    
    // Clean up any existing encoder
    if (current_encoder) {
        rmt_del_encoder(current_encoder);
        current_encoder = nullptr;
    }
    
    // Create copy encoder for raw symbol data
    rmt_copy_encoder_config_t encoder_config = {};
    
    if (rmt_new_copy_encoder(&encoder_config, &current_encoder) != ESP_OK) {
        // Counted rather than logged every time: this sits on the grind control loop,
        // and a repeating fault would bury the 3KB crash ring in its own noise. The
        // count is reported, and callers that fire once (motor test) log it directly.
        encoder_fail_count++;
        last_transmit_err = ESP_ERR_NO_MEM;
        if (encoder_fail_count == 1) {
            LOG_BLE("[MOTOR] Encoder creation FAILED - motor will not actuate\n");
        }
        return;
    }
    
    // Use RMT infinite loop for continuous grinding
    rmt_symbol_word_t continuous_data[1];
    continuous_data[0].duration0 = 32767; // Maximum 15-bit duration per symbol (~32ms at 1MHz)
    continuous_data[0].level0 = 1; // HIGH
    continuous_data[0].duration1 = 0;
    continuous_data[0].level1 = 0;
    
    rmt_transmit_config_t tx_config = {
        .loop_count = -1, // Infinite loop
    };
    
    // The return value used to be discarded, so a failed transmit left the firmware
    // believing the motor was running. It is recorded now, but `grinding` is still set
    // exactly as before: clearing it would make PRIME re-issue start() every control
    // cycle, and this change is meant to reveal the fault, not alter what follows it.
    last_transmit_err = rmt_transmit(rmt_channel, current_encoder, continuous_data,
                                     sizeof(continuous_data), &tx_config);
    if (last_transmit_err != ESP_OK) {
        transmit_fail_count++;
        if (transmit_fail_count == 1) {
            LOG_BLE("[MOTOR] Continuous transmit FAILED: %s\n", esp_err_to_name(last_transmit_err));
        }
    }
    grinding = true;
    emit_background_change(true);
}

void Grinder::stop() {
#if DEBUG_ENABLE_LOADCELL_MOCK
    if (!initialized) return;
    MockHX711Driver::notify_grinder_stop();
    grinding = false;
    pulse_active = false;
    train_active_ = false;
    emit_background_change(false);
    return;
#endif
    if (!initialized || !rmt_initialized) return;
    
    // Stop RMT transmission (works for both infinite loop and finite pulses)
    rmt_disable(rmt_channel);
    rmt_enable(rmt_channel); // Re-enable for next operation
    
    // Clean up current encoder
    if (current_encoder) {
        rmt_del_encoder(current_encoder);
        current_encoder = nullptr;
    }
    
    grinding = false;
    pulse_active = false;
    train_active_ = false;
    emit_background_change(false);
}

bool Grinder::start_pulse_train(uint32_t on_ms, uint32_t off_ms, uint8_t count) {
    if (on_ms == 0 || count == 0) {
        return false;
    }
#if DEBUG_ENABLE_LOADCELL_MOCK
    if (!initialized) return false;
    train_end_ms_ = millis() + (unsigned long)(on_ms + off_ms) * count;
    train_active_ = true;
    pulse_active = true;
    grinding = true;
    motor_start_time = millis();
    emit_background_change(true);
    return true;
#endif
    if (!initialized || !rmt_initialized) return false;

    size_t symbols = build_pulse_train(train_symbols_, TRAIN_SYMBOL_CAPACITY,
                                       on_ms * 1000, off_ms * 1000, count);
    if (symbols == 0) {
        LOG_BLE("[MOTOR] Pulse train %lums x%u does not fit %u symbols\n",
                (unsigned long)on_ms, (unsigned)count, (unsigned)TRAIN_SYMBOL_CAPACITY);
        return false;
    }

    if (current_encoder) {
        rmt_del_encoder(current_encoder);
        current_encoder = nullptr;
    }
    rmt_copy_encoder_config_t encoder_config = {};
    if (rmt_new_copy_encoder(&encoder_config, &current_encoder) != ESP_OK) {
        encoder_fail_count++;
        last_transmit_err = ESP_ERR_NO_MEM;
        LOG_BLE("[MOTOR] Encoder creation FAILED - pulse train not started\n");
        return false;
    }

    // One transmission, no hardware loop: looping repeats the whole symbol, so it can
    // only make trains of identical symbols, and the timing is exact as built
    rmt_transmit_config_t tx_config = {.loop_count = 0};
    last_transmit_err = rmt_transmit(rmt_channel, current_encoder, train_symbols_,
                                     symbols * sizeof(rmt_symbol_word_t), &tx_config);
    if (last_transmit_err != ESP_OK) {
        transmit_fail_count++;
        LOG_BLE("[MOTOR] Pulse train transmit FAILED: %s\n", esp_err_to_name(last_transmit_err));
        return false;
    }

    unsigned long now = millis();
    train_end_ms_ = now + (unsigned long)(pulse_train_duration_us(on_ms * 1000, off_ms * 1000, count) / 1000) + 1;
    train_active_ = true;
    pulse_active = true;
    grinding = true;
    motor_start_time = now;
    emit_background_change(true);
    return true;
}

bool Grinder::is_pulse_train_done() {
    if (!train_active_) {
        return true;
    }
    if ((long)(millis() - train_end_ms_) < 0) {
        return false;
    }
#if !DEBUG_ENABLE_LOADCELL_MOCK
    // Due by the clock; confirm with the driver once, with a timeout, so a stuck
    // transmission is noticed instead of assumed finished
    if (rmt_initialized && rmt_tx_wait_all_done(rmt_channel, 50) != ESP_OK) {
        LOG_BLE("[MOTOR] Pulse train did not finish on time - stopping the motor\n");
        stop();
        return true;
    }
#endif
    train_active_ = false;
    pulse_active = false;
    grinding = false;
    emit_background_change(false);
    return true;
}

void Grinder::start_pulse_rmt(uint32_t duration_ms) {
#if DEBUG_ENABLE_LOADCELL_MOCK
    if (!initialized) return;
    MockHX711Driver::notify_pulse(duration_ms);
    pulse_active = true;
    grinding = true;
    motor_start_time = millis();
    emit_background_change(true);
    return;
#endif
    if (!initialized || !rmt_initialized) return;

    // A single pulse is a train of one with no gap. This used to build its own symbols,
    // and for anything over 65.5ms got the length wrong: it put a 32767us chunk plus
    // the remainder into one symbol and looped it duration/32767 - 1 times, but the
    // driver treats loop_count 0 and 1 alike as one transmission and repeats the whole
    // symbol, remainder included. A 72ms correction pulse went out as 39ms and a 190ms
    // one as 236ms. build_pulse_train() splits every period exactly, so the commanded
    // length is what is sent - which the pulse delivery model depends on.
    start_pulse_train(duration_ms, 0, 1);
}

bool Grinder::is_pulse_complete() {
#if DEBUG_ENABLE_LOADCELL_MOCK
    if (!pulse_active) return true;
    if (!MockHX711Driver::is_pulse_active()) {
        pulse_active = false;
        grinding = false;
        emit_background_change(false);
        return true;
    }
    return false;
#endif
    if (!pulse_active) return true;
    
    // For simplicity, we'll use a transmission done callback approach
    // Since RMT handles the pulse timing in hardware, we can check the GPIO state
    // as a simple completion indicator
    // Known to fire on the first call: the RMT drives this pin without its input buffer
    // enabled, so digitalRead() sees LOW while the pulse is still running. Left as is
    // for now because changing it moves when settling starts - see CLAUDE.md.
    if (digitalRead(motor_pin) == LOW) {
        pulse_active = false;
        grinding = false;
        train_active_ = false;  // Pulses are sent as one-pulse trains
        emit_background_change(false);
        return true;
    }
    
    return false;
}

bool Grinder::is_motor_settled() const {
    // Return true if sufficient time has passed since motor start
    if (motor_start_time == 0) {
        return false;  // Motor has never started
    }
    return (millis() - motor_start_time) >= HW_GRINDER_SETTLING_TIME_MS;
}

void Grinder::set_ui_event_callback(const std::function<void(const GrindEventData&)>& callback) {
    ui_event_callback = callback;
}

void Grinder::emit_background_change(bool active) {
    if (background_active == active) {
        return; // No change
    }
    
    background_active = active;
    
    if (ui_event_callback) {
        // Properly initialize all required fields to prevent null pointer crashes
        GrindEventData event_data = {};
        event_data.event = UIGrindEvent::BACKGROUND_CHANGE;
        event_data.phase = GrindPhase::IDLE;  // Safe default
        event_data.current_weight = 0.0f;
        event_data.progress_percent = 0;
        event_data.phase_display_text = "BACKGROUND";  // Safe string for logging
        event_data.show_taring_text = false;
        event_data.background_active = active;
        
        ui_event_callback(event_data);
        
        LOG_BLE("[Grinder] Background change: %s\n", active ? "ACTIVE" : "INACTIVE");
    }
}
