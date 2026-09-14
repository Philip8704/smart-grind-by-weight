#include "grinder.h"
#include "../controllers/grind_events.h"
#include "../config/constants.h"
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
    emit_background_change(false);
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
    
    // Create RMT symbols for HIGH pulse + LOW end
    rmt_symbol_word_t pulse_symbols[2];
    uint32_t duration_us = duration_ms * 1000;
    
    // Handle long durations by using maximum duration and remainder
    if (duration_us <= 32767) {
        // Single symbol for short durations
        pulse_symbols[0].level0 = 1;
        pulse_symbols[0].duration0 = duration_us;
        pulse_symbols[0].level1 = 0;
        pulse_symbols[0].duration1 = 1; // Minimal LOW to end pulse
        
        rmt_transmit_config_t tx_config = {.loop_count = 0};
        pulse_active = true;
        grinding = true;
        
        last_transmit_err = rmt_transmit(rmt_channel, current_encoder, pulse_symbols,
                                         sizeof(rmt_symbol_word_t), &tx_config);
        if (last_transmit_err != ESP_OK) {
            transmit_fail_count++;
        }
        emit_background_change(true);
    } else {
        // For longer durations, use loop_count to repeat
        uint32_t base_duration = 32767; // Max single symbol duration
        uint32_t loop_count = (duration_us / base_duration) - 1; // -1 because first isn't a loop
        uint32_t remainder = duration_us % base_duration;
        
        pulse_symbols[0].level0 = 1;
        pulse_symbols[0].duration0 = base_duration;
        pulse_symbols[0].level1 = 1;
        pulse_symbols[0].duration1 = remainder > 0 ? remainder : 1;
        
        pulse_symbols[1].level0 = 0;
        pulse_symbols[1].duration0 = 1; // Minimal LOW to end
        pulse_symbols[1].level1 = 0;
        pulse_symbols[1].duration1 = 0;
        
        rmt_transmit_config_t tx_config = {.loop_count = (int)loop_count};
        pulse_active = true;
        grinding = true;
        
        last_transmit_err = rmt_transmit(rmt_channel, current_encoder, pulse_symbols,
                                         sizeof(pulse_symbols), &tx_config);
        if (last_transmit_err != ESP_OK) {
            transmit_fail_count++;
        }
        emit_background_change(true);
    }
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
    if (digitalRead(motor_pin) == LOW) {
        pulse_active = false;
        grinding = false;
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
