#include "grind_controller.h"
#include "grind_events.h"
#include "../hardware/circular_buffer_math/circular_buffer_math.h"
#include "../config/constants.h"
#include "../system/diagnostics_controller.h"
#include "../system/statistics_manager.h"
#include <Arduino.h>
#include <cstdarg>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <esp_heap_caps.h>

#if defined(DEBUG_ENABLE_LOADCELL_MOCK) && (DEBUG_ENABLE_LOADCELL_MOCK != 0)
#include "../hardware/mock_hx711_driver.h"
#endif

// UI event queue size
#define UI_EVENT_QUEUE_SIZE 10

// Flash operation queue size
#define FLASH_OP_QUEUE_SIZE 5

static constexpr float NO_WEIGHT_DELIVERED_THRESHOLD_G = 0.2f;

//------------------------------------------------------------------------------
// Coast measurement window
//------------------------------------------------------------------------------
// A ring of the last few coast times for one profile, and the order statistic the
// predictive stop reads off it. See GRIND_COAST_* in grind_control.h for why the
// prediction is a quantile of real measurements rather than a running average.

void coast_window_reset(CoastWindow& window) {
    // memset rather than field-by-field so the trailing padding is zeroed too. The
    // struct is written to NVS as a raw blob, and leaving padding uninitialised would
    // make an unchanged window serialise differently each boot - which NVS cannot tell
    // from a real change, so it would rewrite the entry every grind for no reason.
    memset(&window, 0, sizeof(CoastWindow));
}

void coast_window_push(CoastWindow& window, float observed_s) {
    if (!isfinite(observed_s) ||
        observed_s < GRIND_COAST_TIME_MIN_S ||
        observed_s > GRIND_COAST_TIME_MAX_S) {
        return;
    }
    // Guard the index rather than trusting it: this struct round-trips through NVS,
    // and a stale or truncated blob must not be able to write outside the array.
    if (window.next >= CoastWindow::CAPACITY) {
        window.next = 0;
    }
    window.samples[window.next] = observed_s;
    window.next = (uint8_t)((window.next + 1) % CoastWindow::CAPACITY);
    if (window.count < CoastWindow::CAPACITY) {
        window.count++;
    }
}

int coast_window_sorted(const CoastWindow& window, float* out, int out_capacity) {
    if (!out || out_capacity <= 0) {
        return 0;
    }
    int n = window.count;
    if (n > CoastWindow::CAPACITY) n = CoastWindow::CAPACITY;
    if (n > out_capacity) n = out_capacity;

    for (int i = 0; i < n; i++) {
        out[i] = window.samples[i];
    }
    // Insertion sort ascending. At most eight elements, so the simplest thing that
    // works is also the fastest, and it runs once per grind rather than per cycle.
    for (int i = 1; i < n; i++) {
        float key = out[i];
        int j = i - 1;
        while (j >= 0 && out[j] > key) {
            out[j + 1] = out[j];
            j--;
        }
        out[j + 1] = key;
    }
    return n;
}

float coast_window_quantile_s(const CoastWindow& window) {
    float sorted[CoastWindow::CAPACITY];
    int n = coast_window_sorted(window, sorted, CoastWindow::CAPACITY);
    if (n <= 0) {
        return 0.0f;
    }
    // Rank counted from the top, clamped to what the window actually holds so the
    // first grinds - when only one or two measurements exist - still return the
    // highest of them rather than reading off the end of the array.
    int rank = GRIND_COAST_RANK_FROM_TOP;
    if (rank < 1) rank = 1;
    if (rank > n) rank = n;
    return sorted[n - rank];
}

float coast_window_prediction_s(const CoastWindow& window) {
    float quantile = coast_window_quantile_s(window);
    if (quantile <= 0.0f) {
        return 0.0f;  // Nothing learned yet - caller falls back to the latency seed
    }
    float predicted = quantile + GRIND_COAST_TAIL_PAD_S;
    // The pad can only push the prediction past the sanity ceiling, never below the
    // floor, but both ends are clamped so a corrupted window cannot reach the motor.
    if (predicted < GRIND_COAST_TIME_MIN_S) predicted = GRIND_COAST_TIME_MIN_S;
    if (predicted > GRIND_COAST_TIME_MAX_S) predicted = GRIND_COAST_TIME_MAX_S;
    return predicted;
}

bool coast_window_is_valid(const CoastWindow& window) {
    if (window.count > CoastWindow::CAPACITY || window.next >= CoastWindow::CAPACITY) {
        return false;
    }
    // Only the occupied slots are checked. Unused ones are left at zero by
    // coast_window_reset and would fail a range test that the ring never reads.
    for (int i = 0; i < window.count; i++) {
        float s = window.samples[i];
        if (!isfinite(s) || s < GRIND_COAST_TIME_MIN_S || s > GRIND_COAST_TIME_MAX_S) {
            return false;
        }
    }
    return true;
}

//------------------------------------------------------------------------------
// Pulse delivery window - see GRIND_PULSE_* in grind_control.h
//------------------------------------------------------------------------------

void pulse_window_reset(PulseWindow& window) {
    memset(&window, 0, sizeof(PulseWindow));  // Padding too: written to NVS as a blob
}

void pulse_window_push(PulseWindow& window, float extra_g) {
    if (!isfinite(extra_g) || extra_g < GRIND_PULSE_EXTRA_VALID_MIN_G || extra_g > GRIND_PULSE_EXTRA_VALID_MAX_G) {
        return;
    }
    if (window.next >= PulseWindow::CAPACITY) {
        window.next = 0;
    }
    window.extra_g[window.next] = extra_g;
    window.next = (uint8_t)((window.next + 1) % PulseWindow::CAPACITY);
    if (window.count < PulseWindow::CAPACITY) {
        window.count++;
    }
}

bool pulse_window_is_valid(const PulseWindow& window) {
    if (window.count > PulseWindow::CAPACITY || window.next >= PulseWindow::CAPACITY) {
        return false;
    }
    for (int i = 0; i < window.count; i++) {
        float v = window.extra_g[i];
        if (!isfinite(v) || v < GRIND_PULSE_EXTRA_VALID_MIN_G || v > GRIND_PULSE_EXTRA_VALID_MAX_G) {
            return false;
        }
    }
    return true;
}

int pulse_window_sorted(const PulseWindow& window, float* out, int out_capacity) {
    if (!out || out_capacity <= 0) {
        return 0;
    }
    int n = window.count;
    if (n > PulseWindow::CAPACITY) n = PulseWindow::CAPACITY;
    if (n > out_capacity) n = out_capacity;
    for (int i = 0; i < n; i++) {
        out[i] = window.extra_g[i];
    }
    for (int i = 1; i < n; i++) {
        float key = out[i];
        int j = i - 1;
        while (j >= 0 && out[j] > key) {
            out[j + 1] = out[j];
            j--;
        }
        out[j + 1] = key;
    }
    return n;
}

// Linearly interpolated quantile of the window, or the seed while it is empty. Never
// below zero: the model may only shorten a pulse relative to the flow-only plan.
static float pulse_window_quantile_g(const PulseWindow& window, float fraction) {
    float sorted[PulseWindow::CAPACITY];
    int n = pulse_window_sorted(window, sorted, PulseWindow::CAPACITY);
    if (n <= 0) {
        return GRIND_PULSE_EXTRA_SEED_G;
    }
    float position = fraction * (float)(n - 1);
    int lower = (int)position;
    int upper = (lower + 1 < n) ? lower + 1 : lower;
    float value = sorted[lower] + (sorted[upper] - sorted[lower]) * (position - (float)lower);
    return value > 0.0f ? value : 0.0f;
}

float pulse_window_expected_extra_g(const PulseWindow& window) {
    // Median: what a pulse typically brings, robust to the odd one that caught a clump
    // or delivered nothing. Used to judge whether even the shortest pulse would overshoot.
    return pulse_window_quantile_g(window, 0.5f);
}

float pulse_window_planning_extra_g(const PulseWindow& window) {
    // Upper quartile: pulses are sized as if they bring more than usual, so most come
    // out a little short and a second correction finishes the job - rather than one
    // too large. Simulated against this grinder's measured pulses, sizing from the
    // median overshot 6% of grinds and this 5%, for ~0.15 more pulses per grind.
    return pulse_window_quantile_g(window, GRIND_PULSE_PLANNING_QUANTILE);
}

float burst_window_expected_g(const PulseWindow& window) {
    // Median of the knock tests so far: what a burst typically shakes loose. The coast
    // model absorbs the rest of the tail, so the median is enough - an overestimate
    // only stops the main run earlier, an underestimate is caught by the coast window.
    if (window.count == 0) {
        return GRIND_BURST_SEED_G;
    }
    return pulse_window_quantile_g(window, 0.5f);
}

void GrindController::init(WeightSensor* lc, Grinder* gr, Preferences* prefs) {
    weight_sensor = lc;
    grinder = gr;
    preferences = prefs;
    phase = GrindPhase::IDLE;
    tolerance = GRIND_ACCURACY_TOLERANCE_G;
    current_profile_id = 0;
    force_measurement_log = false;
    target_time_ms = 0;
    time_grind_start_ms = 0;
    mode = GrindMode::WEIGHT;
    grinder_purge_mode_for_session = static_cast<GrinderPurgeMode>(GRIND_PURGE_MODE_DEFAULT);
    grinder_purge_amount_g_for_session = GRIND_PURGE_AMOUNT_DEFAULT_G;
    last_error_message[0] = '\0';
    last_session_result_ = GrindSessionResult::UNKNOWN;
    control_loop_paused_ = false;

    mechanical_anomaly_count_ = 0;
    last_mechanical_event_ms_ = 0;
    last_mechanical_weight_ = 0.0f;
    mechanical_monitor_initialized_ = false;

    // Initialize grind freshness tracking
    grinder_purged_since_boot = false;
    last_purge_runtime_ms = 0;
    coast_window_reset(coast_window_);
    coast_window_dirty_ = false;
    coast_predicted_s_ = 0.0f;
    pending_coast_time_s_ = 0.0f;
    coast_prediction_flow_gps = 0.0f;

    purge_settled_weight_g_ = 0.0f;
    retare_after_purge_ = false;
    purge_check_plausible_since_ms_ = 0;
    purge_missing_notice_shown_ = false;

    neg_weight_run_ = 0;
    neg_weight_last_seq_ = 0;
    neg_weight_last_g_ = 0.0f;

    pulse_window_reset(pulse_window_);
    pulse_window_dirty_ = false;
    pulse_anomaly_at_start_ = 0;

    pulse_window_reset(burst_window_);
    burst_window_dirty_ = false;
    burst_planned_ = false;
    burst_expected_g_ = 0.0f;
    burst_started_ = false;
    burst_fired_ = false;

    pending_vibration_test_ = false;
    vib_floor_.clear();
    vib_motor_.clear();
    vib_last_seq_ = 0;
    vib_last_motor_ = false;
    vib_state_change_ms_ = 0;
    vib_baseline_start_ms_ = 0;
    vib_run_started_ = false;
    vib_run_first_raw_ = 0;
    vib_run_last_raw_ = 0;
    memset(&vib_result_, 0, sizeof(vib_result_));
    anomaly_count_at_motor_stop_ = 0;
    coast_history_count_ = 0;
    coast_history_next_ = 0;
    pending_observation_ = nullptr;
    error_history_count_ = 0;
    error_history_next_ = 0;
    // Deliberately not restored from NVS. It records esp_timer_get_time(), which is
    // time since boot and restarts at zero on every power-up, so a value saved in an
    // earlier session says nothing about this one - and being larger than the current
    // clock it underflowed the elapsed-time subtraction, making every grind look
    // stale. Freshness is a within-session measure; the first grind after boot is
    // already treated as stale via grinder_purged_since_boot.

    // Set up grinder background indicator callback (if enabled)
    if (grinder) {
#if DEBUG_ENABLE_GRINDER_BACKGROUND_INDICATOR
        grinder->set_ui_event_callback([this](const GrindEventData& event_data) {
            // Forward grinder background events to our existing UI event queue
            if (event_data.event == UIGrindEvent::BACKGROUND_CHANGE) {
                this->emit_ui_event(event_data);
            }
        });
#else
        // Background indicator disabled - set empty callback
        grinder->set_ui_event_callback(nullptr);
#endif
    }
    
    // Initialize the grind logger
    if (!grind_logger.init(preferences)) {
        LOG_BLE("Warning: Grind logging disabled due to initialization failure\n");
    }
    
    // RealtimeController removed - functionality moved to FreeRTOS WeightSamplingTask and GrindControlTask
    
    // Initialize UI event system
    ui_event_callback = nullptr;
    ui_ready_for_setup = false;
    
    // Initialize thread-safe UI event queue
    ui_event_queue = xQueueCreate(UI_EVENT_QUEUE_SIZE, sizeof(GrindEventData));
    if (!ui_event_queue) {
        LOG_BLE("ERROR: Failed to create UI event queue\n");
    } else {
        LOG_BLE("UI event queue created successfully\n");
    }
    
    // Initialize thread-safe flash operation queue
    flash_op_queue = xQueueCreate(FLASH_OP_QUEUE_SIZE, sizeof(FlashOpRequest));
    if (!flash_op_queue) {
        LOG_BLE("ERROR: Failed to create flash operation queue\n");
    } else {
        LOG_BLE("Flash operation queue created successfully\n");
    }
    
    // Initialize thread-safe log message queue
    log_queue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogMessage));
    if (!log_queue) {
        LOG_BLE("ERROR: Failed to create log message queue\n");
    } else {
        LOG_BLE("Log message queue created successfully\n");
    }

    strategy_context.controller = this;
    active_strategy = nullptr;

    // Load motor response latency from preferences
    load_motor_latency();

    // After the counters above are zeroed, so restored entries are not wiped again.
    // main.cpp mounts LittleFS before calling init().
    load_persistent_history();
}

void GrindController::start_grind(float target, uint32_t time_ms, GrindMode grind_mode) {
    // Guard against a second start landing on a live session - a double tap, or the
    // auto-start trigger firing in the same UI cycle as a button press. Restarting
    // mid-grind would swap the strategy and reset the phase underneath Core 0.
    if (is_active()) {
        LOG_BLE("[%lums CONTROLLER] Ignoring start_grind() - a grind is already running (%s)\n",
                millis(), get_phase_name());
        return;
    }

    // An OTA suspends the control-loop task outright. Starting a grind into that would
    // freeze the state machine mid-phase with the motor energised and no loop left to
    // stop it.
    if (ota_active_ && ota_active_()) {
        LOG_BLE("[%lums CONTROLLER] Ignoring start_grind() - firmware update in progress\n", millis());
        return;
    }

    // Refuse a target the control loop could never satisfy, whatever produced it. A
    // non-finite weight makes `current_weight >= target - coast` false on every cycle,
    // so the predictive stop never fires and the grind only ends when the timeout
    // catches it. Better to not start than to run a grind that cannot stop itself.
    if (grind_mode == GrindMode::WEIGHT &&
        (!isfinite(target) || target < USER_MIN_TARGET_WEIGHT_G || target > USER_MAX_TARGET_WEIGHT_G)) {
        LOG_BLE("[%lums CONTROLLER] Refusing start_grind() - target %.2fg outside %.1f-%.1fg\n",
                millis(), target, USER_MIN_TARGET_WEIGHT_G, USER_MAX_TARGET_WEIGHT_G);
        return;
    }
    if (grind_mode == GrindMode::TIME && (time_ms == 0 || time_ms > (uint32_t)(USER_MAX_TARGET_TIME_S * 1000.0f))) {
        LOG_BLE("[%lums CONTROLLER] Refusing start_grind() - time %lums outside 1-%.0fms\n",
                millis(), (unsigned long)time_ms, USER_MAX_TARGET_TIME_S * 1000.0f);
        return;
    }

    target_weight = target;
    target_time_ms = time_ms;
    mode = grind_mode;
    LOG_BLE("[%lums CONTROLLER] start_grind() called with target=%.1fg, time=%lums, mode=%s\n",
            millis(), target, (unsigned long)time_ms, grind_mode == GrindMode::TIME ? "TIME" : "WEIGHT");
    if (!grinder) return;

    bool time_use_scale = false;
    if (mode == GrindMode::TIME && preferences) {
        time_use_scale = preferences->getBool(PREF_KEY_TIME_USE_SCALE, true);
    }
    // A vibration test is nothing but scale data, so it always tares and records,
    // whatever the time-mode display preference says
    const bool vibration_test = pending_vibration_test_ && mode == GrindMode::TIME;
    if (vibration_test) {
        time_use_scale = true;
    }
    const bool knock_test = pending_knock_test_ && mode == GrindMode::WEIGHT;

    if (mode == GrindMode::WEIGHT) {
        if (!weight_sensor) return;
        if (weight_sensor->has_hardware_fault()) {
            LOG_BLE("ERROR: Cannot start grind - load cell hardware fault detected (%d)\n",
                    static_cast<int>(weight_sensor->get_hardware_fault()));
            return;
        }
        // Read grinder purge settings from preferences (weight mode only)
        grinder_purge_mode_for_session = static_cast<GrinderPurgeMode>(GRIND_PURGE_MODE_DEFAULT);
        grinder_purge_amount_g_for_session = GRIND_PURGE_AMOUNT_DEFAULT_G;
        if (preferences) {
            int purge_mode_int = preferences->getInt(PREF_KEY_GRINDER_MODE, GRIND_PURGE_MODE_DEFAULT);
            grinder_purge_mode_for_session = static_cast<GrinderPurgeMode>(purge_mode_int);
            float configured_amount = preferences->getFloat(PREF_KEY_GRINDER_AMOUNT_G, GRIND_PURGE_AMOUNT_DEFAULT_G);
            configured_amount = std::clamp(configured_amount, GRIND_PURGE_AMOUNT_MIN_G, GRIND_PURGE_AMOUNT_MAX_G);
            grinder_purge_amount_g_for_session = configured_amount;
        }
    }

    start_time = millis();
    pulse_attempts = 0;
    timeout_phase = GrindPhase::IDLE; // Initialize timeout phase
    timeout_pause_start = 0;
    timeout_offset_ms = 0;
    last_session_result_ = GrindSessionResult::UNKNOWN;
    // Load cell now runs at constant high speed - no mode switching needed
    
    
    grind_latency_ms = 0;
    predictive_end_weight = 0;
    final_weight = 0;
    motor_stop_target_weight = GRIND_UNDERSHOOT_TARGET_G; // Start with a safe default

    time_grind_start_ms = 0;
    time_grind_elapsed_at_pause_ms = 0;

    flow_start_confirmed = false;

    // Initialize dynamic pulse algorithm variables
    pulse_flow_rate = 0.0f;
    
    // Initialize loop counters
    current_phase_loop_count = 0;
    
    // Initialize measurement state tracking for logger
    last_logged_weight = 0.0f;
    measurement_row_logged_ = false;  // First row of every session is always written
    last_logged_time = millis();
    force_measurement_log = false;

    // Reset UI acknowledgment flag for new grind
    ui_ready_for_setup = false;

    // Reset flash operation flag for new grind
    session_end_flash_queued = false;

    last_error_message[0] = '\0';
    last_notice_message[0] = '\0';
    notice_pending_ = false;

    // The scale is display-only in time mode. If it is unusable, drop to a
    // countdown-only grind and tell the user instead of refusing to start.
    if (time_use_scale && (!weight_sensor || weight_sensor->has_hardware_fault())) {
        LOG_BLE("[%lums CONTROLLER] Load cell unavailable - running time grind without weight display\n",
                millis());
        time_use_scale = false;
        set_notice_message("No scale");
    }

    session_descriptor.mode = mode;
    session_descriptor.target_weight = target_weight;
    session_descriptor.target_time_ms = target_time_ms;
    session_descriptor.tolerance = tolerance;
    session_descriptor.profile_id = current_profile_id;
    session_descriptor.time_use_scale = time_use_scale;
    session_descriptor.vibration_test = vibration_test && time_use_scale;
    session_descriptor.knock_test = knock_test;

    knock_stage_ = KnockStage::PENDING;
    knock_drop_start_ms_ = 0;
    memset(&knock_result_, 0, sizeof(knock_result_));

    // Start counting from the sample current now, so nothing left from before this
    // grind can count towards its failsafe
    neg_weight_run_ = 0;
    neg_weight_last_seq_ = weight_sensor ? weight_sensor->get_raw_sample_snapshot().seq : 0;
    neg_weight_last_g_ = 0.0f;

    purge_settled_weight_g_ = 0.0f;
    retare_after_purge_ = false;
    purge_check_plausible_since_ms_ = 0;
    purge_missing_notice_shown_ = false;

    vib_floor_.clear();
    vib_motor_.clear();
    vib_last_seq_ = weight_sensor ? weight_sensor->get_raw_sample_snapshot().seq : 0;
    vib_last_motor_ = false;
    vib_state_change_ms_ = millis();
    vib_baseline_start_ms_ = 0;
    vib_run_started_ = false;
    vib_run_first_raw_ = 0;
    vib_run_last_raw_ = 0;
    memset(&vib_result_, 0, sizeof(vib_result_));
    if (session_descriptor.vibration_test) {
        set_notice_message("Vibration test");
    } else if (session_descriptor.knock_test) {
        set_notice_message("Knock test");
    }

    // Drop any stale stop request so it cannot cancel the grind we are starting
    stop_requested_.store(false, std::memory_order_relaxed);

    // Only weight grinds use, or can measure, the coast model
    pending_coast_time_s_ = 0.0f;
    coast_observed_ = false;
    coast_prediction_flow_gps = 0.0f;
    coast_predicted_s_ = 0.0f;
    anomaly_count_at_motor_stop_ = 0;
    pending_observation_ = nullptr;
    burst_planned_ = false;
    burst_expected_g_ = 0.0f;
    burst_started_ = false;
    burst_fired_ = false;
    pulse_window_reset(burst_window_);
    burst_window_dirty_ = false;
    if (mode == GrindMode::WEIGHT) {
        load_coast_time(current_profile_id);
        load_pulse_window();
        if (preferences) {
            read_burst_window(*preferences, burst_window_);
        }
        // Every weight grind ends its main run with a burst - except the knock test,
        // which fires its burst at the knock point to measure it
        burst_planned_ = GRIND_BURST_ENABLED && !session_descriptor.knock_test;
        if (burst_planned_) {
            burst_expected_g_ = get_burst_expected_g();
        }
    } else {
        coast_window_reset(coast_window_);
        coast_window_dirty_ = false;
    }

    // Initialize pulse tracking
    additional_pulse_count = 0;
    pulse_duration_ms = GRIND_TIME_PULSE_DURATION_MS;
    control_loop_paused_ = false;

    reset_mechanical_anomaly_count();

    if (diagnostics_controller_) {
        diagnostics_controller_->reset_diagnostic(DiagnosticCode::MECHANICAL_INSTABILITY);
    }

    if (mode == GrindMode::WEIGHT) {
        active_strategy = static_cast<IGrindStrategy*>(&weight_strategy);
    } else if (mode == GrindMode::TIME) {
        active_strategy = static_cast<IGrindStrategy*>(&time_strategy);
    } else {
        active_strategy = nullptr;
    }
    
    // Start with INITIALIZING phase - this emits immediate UI event
    // Create minimal loop_data for initial phase transition
    GrindLoopData loop_data = {};
    loop_data.now = millis();
    loop_data.timestamp_ms = loop_data.now - start_time;
    loop_data.current_weight = weight_sensor ? weight_sensor->get_weight_low_latency() : 0.0f;

    if (active_strategy) {
        active_strategy->on_enter(session_descriptor, strategy_context, loop_data);
    }
    
    switch_phase(GrindPhase::INITIALIZING, loop_data);
}

void NoiseAccumulator::add(double t_ms, double value) {
    if (n < 2) {
        t[n] = t_ms;
        y[n] = value;
        n++;
        return;
    }
    // Deviation of the middle sample from the line joining its neighbours. With
    // interpolation weights a and b that residual has variance s^2 (1 + a^2 + b^2) for
    // white noise, so dividing that out makes the estimate read s directly - and
    // correctly even when the 10 SPS sample spacing jitters.
    double span = t_ms - t[0];
    if (span > 0.0f) {
        double a = (t_ms - t[1]) / span;
        double b = (t[1] - t[0]) / span;
        double residual = y[1] - (a * y[0] + b * value);
        sum_sq += (residual * residual) / (1.0 + a * a + b * b);
        count++;
    }
    t[0] = t[1]; y[0] = y[1];
    t[1] = t_ms; y[1] = value;
}

bool GrindController::start_vibration_test() {
    if (is_active()) {
        return false;
    }
    if (!weight_sensor || weight_sensor->has_hardware_fault()) {
        LOG_BLE("[%lums CONTROLLER] Vibration test needs a working load cell\n", millis());
        return false;
    }
    // start_grind() reads this to flag the session; it must not outlive the call, or
    // the next ordinary time grind would be recorded as a test
    pending_vibration_test_ = true;
    start_grind(0.0f, VIBRATION_TEST_RUN_MS, GrindMode::TIME);
    pending_vibration_test_ = false;
    return is_active();
}

void GrindController::update_vibration_test(const GrindLoopData& loop_data) {
    WeightSensor::RawSampleSnapshot sample = weight_sensor->get_raw_sample_snapshot();
    if (sample.seq == vib_last_seq_) {
        return;  // Five control cycles per HX711 sample - only act on a new one
    }
    vib_last_seq_ = sample.seq;

    // Worked in raw counts, converted once at the end: the tare moves the offset part
    // way through TARE_CONFIRM, and counts are immune to that where grams are not
    const bool motor_on = grinder && grinder->is_grinding();
    if (motor_on != vib_last_motor_) {
        vib_last_motor_ = motor_on;
        vib_state_change_ms_ = sample.timestamp_ms;
        vib_floor_.break_run();
        vib_motor_.break_run();
    }
    // The first moments after a start or stop are a torque step and spin-up, not the
    // steady vibration the filter has to live with
    // Signed: the sample current when the session started can predate it, and an
    // unsigned difference would wrap and wave it through
    if ((int32_t)(sample.timestamp_ms - vib_state_change_ms_) < (int32_t)VIBRATION_TEST_TRANSIENT_MS) {
        return;
    }

    const double t = (double)sample.timestamp_ms;
    const double raw = (double)sample.raw_adc;
    if (!motor_on && vib_baseline_start_ms_ != 0 &&
        (phase == GrindPhase::TARE_CONFIRM || phase == GrindPhase::FINAL_SETTLING)) {
        vib_floor_.add(t, raw);
    } else if (motor_on && phase == GrindPhase::TIME_GRINDING) {
        vib_motor_.add(t, raw);
        if (!vib_run_started_) {
            vib_run_started_ = true;
            vib_run_first_raw_ = sample.raw_adc;
        }
        vib_run_last_raw_ = sample.raw_adc;
    }
    (void)loop_data;
}

void GrindController::finish_vibration_test() {
    memset(&vib_result_, 0, sizeof(vib_result_));
    float cal = weight_sensor ? fabsf(weight_sensor->get_calibration_factor()) : 0.0f;
    if (cal <= 0.0f) {
        return;
    }
    vib_result_.floor_samples = (uint16_t)std::min<uint32_t>(vib_floor_.count, UINT16_MAX);
    vib_result_.vibration_samples = (uint16_t)std::min<uint32_t>(vib_motor_.count, UINT16_MAX);
    vib_result_.floor_sigma_g = vib_floor_.sigma() / cal;
    vib_result_.vibration_sigma_g = vib_motor_.sigma() / cal;
    if (vib_run_started_) {
        // Sign follows the calibration factor, which is negative on some wirings
        vib_result_.weight_rise_g = (float)(vib_run_last_raw_ - vib_run_first_raw_) /
                                    weight_sensor->get_calibration_factor();
        vib_result_.grounds_detected = vib_result_.weight_rise_g > VIBRATION_TEST_GROUNDS_DETECT_G;
    }
    vib_result_.valid = vib_result_.floor_samples >= 10 && vib_result_.vibration_samples >= 10;

    queue_log_message("[VIBRATION] floor %.4fg (%u samples), motor %.4fg (%u samples), rise %+.2fg%s\n",
                      vib_result_.floor_sigma_g, (unsigned)vib_result_.floor_samples,
                      vib_result_.vibration_sigma_g, (unsigned)vib_result_.vibration_samples,
                      vib_result_.weight_rise_g,
                      vib_result_.grounds_detected ? " - GROUNDS ARRIVED, hopper not empty" : "");
}

//------------------------------------------------------------------------------
// Chute knock test
//------------------------------------------------------------------------------

uint16_t GrindController::get_jolt_ms() const {
    // The motor latency learned by Tune Pulses: the start-up kick, without the grinding
    // that would follow it
    float on_ms = motor_response_latency_ms;
    if (!(on_ms >= GRIND_AUTOTUNE_LATENCY_MIN_MS)) on_ms = GRIND_AUTOTUNE_LATENCY_MIN_MS;  // Also catches NaN
    if (on_ms > GRIND_AUTOTUNE_LATENCY_MAX_MS) on_ms = GRIND_AUTOTUNE_LATENCY_MAX_MS;
    return (uint16_t)on_ms;
}

bool GrindController::start_knock_test() {
    if (is_active()) {
        return false;
    }
    if (!weight_sensor || weight_sensor->has_hardware_fault()) {
        LOG_BLE("[%lums CONTROLLER] Knock test needs a working load cell\n", millis());
        return false;
    }
    if (!grinder || !grinder->is_rmt_ready()) {
        LOG_BLE("[%lums CONTROLLER] Knock test needs the motor\n", millis());
        return false;
    }
    // start_grind() reads this to flag the session; it must not outlive the call, or
    // the next ordinary grind would stop short and knock too
    pending_knock_test_ = true;
    start_grind(KNOCK_TEST_TARGET_G, 0, GrindMode::WEIGHT);
    pending_knock_test_ = false;
    return is_active();
}

void GrindController::begin_knock(float settled_weight, const GrindLoopData& loop_data) {
    // Exactly the burst an ordinary grind fires after its main run, so this measures it
    knock_result_.on_ms = get_jolt_ms();
    knock_result_.pulses = GRIND_BURST_PULSES;
    knock_result_.before_g = settled_weight;

    if (!grinder->start_pulse_train(knock_result_.on_ms, GRIND_BURST_OFF_MS, GRIND_BURST_PULSES)) {
        // Staying in PULSE_DECISION finishes the dose without the knock, rather than
        // leaving it half a gram short
        knock_result_.aborted = true;
        knock_stage_ = KnockStage::DONE;
        queue_log_message("[KNOCK] %u jolts of %ums could not start - finishing without\n",
                          (unsigned)GRIND_BURST_PULSES, (unsigned)knock_result_.on_ms);
        return;
    }
    knock_stage_ = KnockStage::TRAIN;
    queue_log_message("[KNOCK] At %.2fg of %.1fg: %u jolts of %ums, %ums apart\n",
                      settled_weight, target_weight, (unsigned)GRIND_BURST_PULSES,
                      (unsigned)knock_result_.on_ms, (unsigned)GRIND_BURST_OFF_MS);
    set_notice_message("Knocking");
    switch_phase(GrindPhase::KNOCK_TEST, loop_data);
}

void GrindController::update_knock_test(const GrindLoopData& loop_data) {
    if (!weight_sensor) {
        return;
    }

    switch (knock_stage_) {
        case KnockStage::TRAIN:
            if (grinder->is_pulse_train_done()) {
                knock_stage_ = KnockStage::DROP;
                knock_drop_start_ms_ = loop_data.now;
            }
            break;

        case KnockStage::DROP: {
            if (loop_data.now - knock_drop_start_ms_ < KNOCK_TEST_DROP_WAIT_MS) {
                break;
            }
            // Read the way the pulse decision read the weight before the knock - stopped
            // rising, not merely quiet, with the same backstop - so the two compare
            float settled_weight;
            bool stable = weight_sensor->check_settling_complete(
                GRIND_SCALE_PRECISION_SETTLING_TIME_MS, &settled_weight, GRIND_SETTLING_DRIFT_MAX_GPS);
            if (!stable) {
                bool timed_out = (loop_data.now - knock_drop_start_ms_) >= GRIND_SETTLING_STABLE_TIMEOUT_MS;
                if (!(timed_out && weight_sensor->check_settling_complete(
                                       GRIND_SCALE_PRECISION_SETTLING_TIME_MS, &settled_weight))) {
                    break;
                }
            }

            knock_result_.after_g = settled_weight;
            knock_result_.released_g = settled_weight - knock_result_.before_g;
            knock_result_.knocked = true;
            knock_stage_ = KnockStage::DONE;
            queue_log_message("[KNOCK] %u x %ums released %+.3fg (%.2fg -> %.2fg)\n",
                              (unsigned)knock_result_.pulses, (unsigned)knock_result_.on_ms,
                              knock_result_.released_g, knock_result_.before_g, knock_result_.after_g);

            // What the test is for: the burst model learns it, unless no burst could
            // have produced it
            const float released = knock_result_.released_g;
            if (isfinite(released) && released >= GRIND_PULSE_DELIVERED_MIN_G &&
                released <= GRIND_PULSE_DELIVERED_MAX_G) {
                pulse_window_push(burst_window_, released);
                burst_window_dirty_ = true;
                knock_result_.learned = true;
            }
            knock_result_.burst_model_g = get_burst_expected_g();
            queue_log_message("[BURST] Knock test measured %+.3fg%s -> model now %+.3fg from %u test(s)\n",
                              released, knock_result_.learned ? "" : " - implausible, not learned",
                              knock_result_.burst_model_g, (unsigned)burst_window_.count);

            char notice[sizeof(last_notice_message)];
            snprintf(notice, sizeof(notice), "Knock %+.2fg", knock_result_.released_g);
            set_notice_message(notice);

            // The usual decision takes it from here and finishes the dose with pulses
            switch_phase(GrindPhase::PULSE_DECISION, loop_data);
            break;
        }

        default:
            // Nothing to wait for in this phase; hand back rather than stall
            switch_phase(GrindPhase::PULSE_DECISION, loop_data);
            break;
    }
}

void GrindController::user_tare_request() {
    // Tare request is now handled automatically when grinding starts
    // This function is kept for compatibility but does nothing
}

void GrindController::execute_return_to_idle() {
    // This is called by the UI to acknowledge a completed or timed-out grind
    // and return the controller to the IDLE state.
    if (phase == GrindPhase::COMPLETED || phase == GrindPhase::TIMEOUT) {
        LOG_BLE("[%lums CONTROLLER] UI acknowledged completion/timeout, returning to IDLE.\n", millis());
        time_grind_start_ms = 0;
        target_time_ms = 0;
        grinder_purge_mode_for_session = static_cast<GrinderPurgeMode>(GRIND_PURGE_MODE_DEFAULT);
        grinder_purge_amount_g_for_session = GRIND_PURGE_AMOUNT_DEFAULT_G;
        last_error_message[0] = '\0';
        if (active_strategy) {
            active_strategy->on_exit(session_descriptor, strategy_context);
            active_strategy = nullptr;
        }
        switch_phase(GrindPhase::IDLE);  // No loop_data needed for IDLE transition
    }
    // If already IDLE, do nothing. If in another active state, this method shouldn't be called.
}

//==============================================================================
// UI-facing entry points (Core 1). These only raise a request; the matching
// execute_* body runs on Core 0 from process_ui_requests(). Applied within one
// control cycle (20ms), which is imperceptible on a button press.
//==============================================================================

void GrindController::stop_grind() {
    stop_requested_.store(true, std::memory_order_release);
}

void GrindController::return_to_idle() {
    return_to_idle_requested_.store(true, std::memory_order_release);
}

void GrindController::continue_from_purge() {
    purge_continue_requested_.store(true, std::memory_order_release);
}

void GrindController::pause_time_grind() {
    pause_requested_.store(true, std::memory_order_release);
}

void GrindController::resume_time_grind() {
    resume_requested_.store(true, std::memory_order_release);
}

void GrindController::start_additional_pulse() {
    pulse_requested_.store(true, std::memory_order_release);
}

bool GrindController::process_ui_requests() {
    if (stop_requested_.exchange(false, std::memory_order_acquire)) {
        if (is_active()) {
            execute_stop();
        } else if (grinder) {
            grinder->stop();  // Safety net: never leave the motor on after a stop request
        }
        return true;
    }
    if (return_to_idle_requested_.exchange(false, std::memory_order_acquire)) {
        execute_return_to_idle();
        return true;
    }
    if (purge_continue_requested_.exchange(false, std::memory_order_acquire)) {
        execute_continue_from_purge();
        return true;
    }
    if (pause_requested_.exchange(false, std::memory_order_acquire)) {
        execute_pause_time_grind();
        return true;
    }
    if (resume_requested_.exchange(false, std::memory_order_acquire)) {
        execute_resume_time_grind();
        return true;
    }
    if (pulse_requested_.exchange(false, std::memory_order_acquire)) {
        execute_additional_pulse();
        return true;
    }
    return false;
}

void GrindController::execute_stop() {
    if (!grinder) return;

    grinder->stop();

    // Cancelled grinds just discard PSRAM data and go to IDLE. The discard is queued
    // rather than done here: the logger's buffers belong to the file IO task, which
    // may still be starting this very session.
    if (grind_logger.is_logging_active()) {
        FlashOpRequest discard = {};
        discard.operation_type = FlashOpRequest::DISCARD_GRIND_SESSION;
        queue_flash_operation(discard);
        session_end_flash_queued = true;
    }
    discard_coast_observation("stopped by user");
    
    LOG_BLE("--- GRIND STOPPED BY USER ---\n");

    time_grind_start_ms = 0;
    time_grind_elapsed_at_pause_ms = 0;
    target_time_ms = 0;
    grinder_purge_mode_for_session = static_cast<GrinderPurgeMode>(GRIND_PURGE_MODE_DEFAULT);
    grinder_purge_amount_g_for_session = GRIND_PURGE_AMOUNT_DEFAULT_G;
    last_error_message[0] = '\0';
    if (active_strategy) {
        active_strategy->on_exit(session_descriptor, strategy_context);
        active_strategy = nullptr;
    }
    switch_phase(GrindPhase::IDLE);  // No loop_data needed for IDLE transition
}

void GrindController::execute_continue_from_purge() {
    // Called by UI when user confirms purge completion
    if (phase != GrindPhase::PURGE_CONFIRM) {
        LOG_BLE("[%lums CONTROLLER] Warning: continue_from_purge() called in wrong phase: %s\n",
                millis(), get_phase_name());
        return;
    }

    LOG_BLE("[%lums CONTROLLER] User confirmed purge, checking whether grounds were kept\n", millis());

    // Add time spent in PURGE_CONFIRM to timeout offset (exclude from timeout calculation)
    if (timeout_pause_start > 0) {
        unsigned long pause_duration = millis() - timeout_pause_start;
        timeout_offset_ms += pause_duration;
        LOG_BLE("[%lums CONTROLLER] Excluding %lums pause time from timeout (total offset: %lums)\n",
                millis(), pause_duration, timeout_offset_ms);
        timeout_pause_start = 0;
    }

    // The motor stays off: PURGE_CHECK decides, once the portafilter has settled,
    // whether to re-tare or carry on - the tap on the check mark usually comes while
    // it is still being seated, and a 600g portafilter settling can swing past 0.5g.
    purge_check_plausible_since_ms_ = 0;
    purge_missing_notice_shown_ = false;

    // Supply loop_data so the event bookkeeping starts a fresh record for the next
    // phase. Without it switch_phase skips that step, and everything after gets logged
    // under PURGE_CONFIRM - which once made the session data read as though 16g had
    // been ground while waiting for the user to confirm.
    GrindLoopData loop_data = {};
    loop_data.now = millis();
    loop_data.timestamp_ms = loop_data.now - start_time;
    loop_data.current_weight = weight_sensor ? weight_sensor->get_weight_low_latency() : 0.0f;
    switch_phase(GrindPhase::PURGE_CHECK, loop_data);
}

void GrindController::execute_pause_time_grind() {
    if (!can_pause_time_grind() || !grinder) {
        return;
    }

    grinder->stop();

    unsigned long now = millis();
    time_grind_elapsed_at_pause_ms = (time_grind_start_ms > 0) ? (now - time_grind_start_ms) : 0;
    timeout_pause_start = now;  // Track pause start so paused time is excluded from timeout

    queue_log_message("[%lums CONTROLLER] Time grind paused at %lums of %lums\n",
                      now, time_grind_elapsed_at_pause_ms, (unsigned long)target_time_ms);

    switch_phase(GrindPhase::TIME_PAUSED);
}

void GrindController::execute_resume_time_grind() {
    if (phase != GrindPhase::TIME_PAUSED || !grinder) {
        return;
    }

    unsigned long now = millis();
    if (timeout_pause_start > 0) {
        timeout_offset_ms += now - timeout_pause_start;
        timeout_pause_start = 0;
    }

    // Rebase the start timestamp so already-elapsed grind time is preserved
    time_grind_start_ms = now - time_grind_elapsed_at_pause_ms;

    queue_log_message("[%lums CONTROLLER] Time grind resumed, %lums remaining\n",
                      now, (unsigned long)get_time_remaining_ms());

    grinder->start();
    switch_phase(GrindPhase::TIME_GRINDING);
}

bool GrindController::can_pause_time_grind() const {
    return mode == GrindMode::TIME && phase == GrindPhase::TIME_GRINDING;
}

uint32_t GrindController::get_time_remaining_ms() const {
    if (mode != GrindMode::TIME || target_time_ms == 0) {
        return 0;
    }
    if (phase == GrindPhase::COMPLETED || phase == GrindPhase::TIMEOUT ||
        phase == GrindPhase::IDLE) {
        return 0;
    }

    unsigned long elapsed;
    if (phase == GrindPhase::TIME_PAUSED) {
        elapsed = time_grind_elapsed_at_pause_ms;
    } else if (time_grind_start_ms > 0) {
        elapsed = millis() - time_grind_start_ms;
    } else {
        elapsed = 0;  // Not started grinding yet - full time remaining
    }

    return (elapsed >= target_time_ms) ? 0 : (uint32_t)(target_time_ms - elapsed);
}

void GrindController::update() {
    // Apply anything the UI asked for first, on this core, so that a phase which is
    // mid-cycle cannot re-assert the motor afterwards
    if (process_ui_requests()) return;

    if (!is_active()) return;

    unsigned long now = millis();
    
    // Calculate all measurement values once at the start - pass to methods to avoid redundant calculations
    GrindLoopData loop_data = {};
    loop_data.now = now;
    loop_data.timestamp_ms = now - start_time;  // Relative to session start
    loop_data.current_weight = weight_sensor ? weight_sensor->get_weight_low_latency() : 0.0f;
    loop_data.instant_weight = weight_sensor ? weight_sensor->get_instant_weight() : 0.0f;
    loop_data.display_weight = weight_sensor ? weight_sensor->get_display_weight() : 0.0f;
    loop_data.motor_is_on = grinder ? (grinder->is_grinding() ? 1 : 0) : 0;
    loop_data.phase_id = get_current_phase_id();
    loop_data.flow_rate = weight_sensor ? weight_sensor->get_flow_rate() : 0.0f;
    loop_data.weight_delta = loop_data.current_weight - last_logged_weight;

    if (control_loop_paused_) {
        emit_progress_update(loop_data);

        // Keep measurement baseline aligned for when logging resumes
        last_logged_weight = loop_data.current_weight;
        last_logged_time = loop_data.now;
        return;
    }
    
    // Increment loop counter for current phase performance tracking
    current_phase_loop_count = current_phase_loop_count + 1;

    monitor_mechanical_instability(loop_data);
    
    switch (phase) {
        case GrindPhase::INITIALIZING:
            // Wait for UI to acknowledge the phase transition before proceeding
            if (ui_ready_for_setup) {
                LOG_UI_DEBUG("UI acknowledged INITIALIZING phase, proceeding to SETUP\n");
                switch_phase(GrindPhase::SETUP, loop_data);
            }
            break;
            
        case GrindPhase::SETUP: {
            float pre_tare_weight = weight_sensor ? weight_sensor->get_weight_low_latency() : 0.0f;
            grind_logger.start_grind_session(session_descriptor, pre_tare_weight);
            if (mode == GrindMode::TIME && !session_descriptor.time_use_scale) {
                // Sensor-free time mode: skip taring, start motor immediately
                if (!grinder->is_grinding()) grinder->start();
                time_grind_start_ms = loop_data.now;
                switch_phase(GrindPhase::TIME_GRINDING, loop_data);
            } else {
                // Weight mode and scale-assisted time mode tare first
                switch_phase(GrindPhase::TARING, loop_data);
            }
            break;
        }
            
        case GrindPhase::TARING:
            if (weight_sensor && weight_sensor->start_nonblocking_tare()) {
                LOG_LOADCELL_DEBUG("Non-blocking tare started\n");
                switch_phase(GrindPhase::TARE_CONFIRM, loop_data);
            }
            break;

        case GrindPhase::TARE_CONFIRM: {
            // Tare is complete once the sampling finished and the reading settled
            bool tare_complete = weight_sensor &&
                                 !weight_sensor->is_tare_in_progress() &&
                                 weight_sensor->is_settled();

            // Time mode only tares for the display, so a failed tare must not
            // block the grind - warn the user and start grinding anyway
            if (!tare_complete && mode == GrindMode::TIME &&
                (loop_data.now - phase_start_time) >= GRIND_TIME_TARE_TIMEOUT_MS) {
                queue_log_message("[%lums CONTROLLER] Tare failed after %lums - grinding on time only\n",
                                  loop_data.now, (unsigned long)GRIND_TIME_TARE_TIMEOUT_MS);
                set_notice_message("Tare failed");
                tare_complete = true;
            }

            if (tare_complete) {
                // Snapshot the conversion the raw samples need. A re-tare after the purge
                // overwrites it, so it always describes the zero the dose was ground on;
                // the session is flagged when that happens.
                if (weight_sensor) {
                    grind_logger.record_scale_state(weight_sensor->get_zero_offset(),
                                                    weight_sensor->get_calibration_factor());
                }

                // Vibration test: hold the motor off a while longer with the scale
                // loaded, to measure the load cell's own floor before the run
                if (session_descriptor.vibration_test) {
                    if (vib_baseline_start_ms_ == 0) {
                        vib_baseline_start_ms_ = loop_data.now;
                    }
                    if ((loop_data.now - vib_baseline_start_ms_) < VIBRATION_TEST_BASELINE_MS) {
                        break;
                    }
                }

                // Re-zeroed after discarded purge grounds: the chute is already primed,
                // so go straight to the main grind instead of purging again
                if (retare_after_purge_) {
                    retare_after_purge_ = false;
                    grind_logger.add_session_flags(GRIND_SESSION_FLAG_RETARED_AFTER_PURGE);
                    queue_log_message("[PURGE] Re-tared - grinding full dose from zero\n");
                    grinder->start();
                    time_grind_start_ms = loop_data.now;
                    switch_phase(GrindPhase::PREDICTIVE, loop_data);
                    break;
                }

                if (!grinder->is_grinding()) {
                    grinder->start();  // Ensure motor is running
                }
                time_grind_start_ms = loop_data.now;
                if (mode == GrindMode::TIME) {
                    switch_phase(GrindPhase::TIME_GRINDING, loop_data);
                } else {
                    // Always run chute operation for weight mode
                    switch_phase(GrindPhase::PRIME, loop_data);
                }
            }
            break;
        }

        case GrindPhase::PRIME: {
            if (!grinder->is_grinding()) {
                grinder->start();
            }

            // Use configurable grinder purge amount
            bool reached_weight = loop_data.current_weight >= grinder_purge_amount_g_for_session;
            bool exceeded_duration = (loop_data.now - phase_start_time) >= GRIND_PRIME_MAX_DURATION_MS;
            if (reached_weight || exceeded_duration) {
                grinder->stop();
                if (exceeded_duration && !reached_weight) {
                    queue_log_message("[GRINDER] Max duration reached (%.2fg delivered)\n", loop_data.current_weight);
                }
                switch_phase(GrindPhase::PRIME_SETTLING, loop_data);
            }
            break;
        }

        case GrindPhase::PRIME_SETTLING: {
            if (!weight_sensor) {
                break;
            }

            float purge_settled_weight = loop_data.current_weight;
            bool settled = weight_sensor->check_settling_complete(GRIND_SCALE_PRECISION_SETTLING_TIME_MS,
                                                                  &purge_settled_weight);
            bool settling_timed_out = (loop_data.now - phase_start_time) >= GRIND_SCALE_SETTLING_TIMEOUT_MS;
            if (settled || settling_timed_out) {
                if (settling_timed_out && !settled) {
                    queue_log_message("[GRINDER] Settling timeout, resuming grind\n");
                }
                flow_start_confirmed = false;
                grind_latency_ms = 0;

                // Check if grounds are stale and purge confirmation should be shown
                bool should_show_purge_popup = false;
                if (!grinder_purged_since_boot) {
                    // First grind since boot - grounds are stale
                    should_show_purge_popup = true;
                } else {
                    // Check if enough time has elapsed since last grind. Both stamps
                    // come from this boot's clock, so the subtraction cannot go
                    // backwards - treat it as stale if it ever does rather than
                    // silently skipping the purge.
                    uint64_t current_ms = esp_timer_get_time() / 1000;
                    float freshness_hours = preferences ? preferences->getFloat(PREF_KEY_GRIND_FRESHNESS_HOURS, GRIND_FRESHNESS_DEFAULT_HOURS) : GRIND_FRESHNESS_DEFAULT_HOURS;
                    uint64_t threshold_ms = (uint64_t)(freshness_hours * 3600000.0f);

                    if (current_ms < last_purge_runtime_ms) {
                        should_show_purge_popup = true;
                    } else {
                        should_show_purge_popup = ((current_ms - last_purge_runtime_ms) > threshold_ms);
                    }
                }

                // Determine next phase based on mode AND staleness
                if (grinder_purge_mode_for_session == GrinderPurgeMode::PURGE && should_show_purge_popup) {
                    // Purge mode with stale grounds: wait for user confirmation before continuing.
                    // Remember what actually landed - the configured amount is only where
                    // PRIME stopped, and coast adds to it - so the check after confirm can
                    // tell kept grounds from discarded ones.
                    purge_settled_weight_g_ = purge_settled_weight;
                    queue_log_message("[PURGE] Delivered %.2fg, waiting for confirmation\n", purge_settled_weight);
                    timeout_pause_start = loop_data.now;  // Track when pause started for timeout offset
                    switch_phase(GrindPhase::PURGE_CONFIRM, loop_data);
                } else {
                    // Prime mode OR fresh grounds: continue immediately to grinding
                    grinder->start();
                    time_grind_start_ms = loop_data.now;
                    switch_phase(GrindPhase::PREDICTIVE, loop_data);
                }
            }
            break;
        }

        case GrindPhase::PURGE_CONFIRM: {
            // This phase waits for UI confirmation
            // The UI will call a method to acknowledge and continue to PREDICTIVE
            // For now, this case just holds the state
            break;
        }

        case GrindPhase::PURGE_CHECK: {
            if (!weight_sensor) {
                grinder->start();
                time_grind_start_ms = loop_data.now;
                switch_phase(GrindPhase::PREDICTIVE, loop_data);
                break;
            }

            const float reading = loop_data.current_weight;

            // Portafilter still off the scale. Taring now would zero the empty cradle
            // and grind onto it, so wait for it to come back; the grind timeout still
            // applies if it never does.
            if (reading < GRIND_PURGE_PORTAFILTER_MISSING_G) {
                purge_check_plausible_since_ms_ = 0;
                if (!purge_missing_notice_shown_) {
                    purge_missing_notice_shown_ = true;
                    set_notice_message("Place portafilter");  // Delivered by this cycle's progress event
                }
                break;
            }
            if (purge_check_plausible_since_ms_ == 0) {
                purge_check_plausible_since_ms_ = loop_data.now;
            }

            // Steady means the whole window lies after the portafilter came back, so a
            // placement still in progress cannot pass as settled
            const unsigned long on_scale_ms = loop_data.now - purge_check_plausible_since_ms_;
            const bool steady = on_scale_ms >= GRIND_PURGE_CHECK_STEADY_WINDOW_MS &&
                                !weight_sensor->weight_range_exceeds(GRIND_PURGE_CHECK_STEADY_WINDOW_MS,
                                                                    GRIND_PURGE_CHECK_STEADY_RANGE_G);
            if (!steady && on_scale_ms < GRIND_PURGE_CHECK_TIMEOUT_MS) {
                break;
            }

            const float deviation = fabsf(reading - purge_settled_weight_g_);
            if (deviation > GRIND_PURGE_RETARE_THRESHOLD_G) {
                // Grounds discarded: the basket was emptied and re-seated, so zero again
                // rather than let the seating difference end up in the dose
                queue_log_message("[PURGE] Reads %.2fg vs %.2fg purged (%.2fg apart%s) - re-taring\n",
                                  reading, purge_settled_weight_g_, deviation, steady ? "" : ", not steady");
                retare_after_purge_ = true;
                set_notice_message("Re-taring");
                switch_phase(GrindPhase::TARING, loop_data);
            } else {
                // Grounds kept: carry on counting them towards the dose, as before
                queue_log_message("[PURGE] Reads %.2fg vs %.2fg purged - grounds kept, continuing\n",
                                  reading, purge_settled_weight_g_);
                grinder->start();
                time_grind_start_ms = loop_data.now;
                switch_phase(GrindPhase::PREDICTIVE, loop_data);
            }
            break;
        }

        case GrindPhase::KNOCK_TEST:
            update_knock_test(loop_data);
            break;

        case GrindPhase::TIME_GRINDING:
            if (mode == GrindMode::TIME && active_strategy) {
                active_strategy->update(session_descriptor, strategy_context, loop_data);
            }
            break;

        case GrindPhase::PREDICTIVE:
            if (mode == GrindMode::WEIGHT && active_strategy) {
                active_strategy->update(session_descriptor, strategy_context, loop_data);
            }
            break;
            
        case GrindPhase::PULSE_DECISION:
            if (mode == GrindMode::WEIGHT && active_strategy) {
                active_strategy->update(session_descriptor, strategy_context, loop_data);
            }
            break;

        case GrindPhase::PULSE_EXECUTE:
            if (mode == GrindMode::WEIGHT && active_strategy) {
                active_strategy->update(session_descriptor, strategy_context, loop_data);
            }
            break;

        case GrindPhase::PULSE_SETTLING:
            if (mode == GrindMode::WEIGHT && active_strategy) {
                active_strategy->update(session_descriptor, strategy_context, loop_data);
            }
            break;

        case GrindPhase::BURST:
            if (mode == GrindMode::WEIGHT && active_strategy) {
                active_strategy->update(session_descriptor, strategy_context, loop_data);
            }
            break;

        case GrindPhase::FINAL_SETTLING: {
            // Require the reading to have stopped rising, not merely gone quiet, so the
            // final weight is not captured mid-trickle. Falls back to the variance-only
            // test after the backstop timeout so a slow grinder cannot stall here.
            bool settled = weight_sensor &&
                           weight_sensor->check_settling_complete(GRIND_SCALE_PRECISION_SETTLING_TIME_MS,
                                                                  nullptr, GRIND_SETTLING_DRIFT_MAX_GPS);
            if (!settled && weight_sensor && mode == GrindMode::WEIGHT &&
                (loop_data.now - phase_start_time) >= GRIND_SETTLING_STABLE_TIMEOUT_MS) {
                settled = weight_sensor->check_settling_complete(GRIND_SCALE_PRECISION_SETTLING_TIME_MS);
            }

            // Time mode reports whatever the scale happens to show; it never waits
            // on a scale that is absent, disabled or refusing to settle
            if (!settled && mode == GrindMode::TIME) {
                settled = !session_descriptor.time_use_scale ||
                          (loop_data.now - phase_start_time) >= GRIND_TIME_SETTLING_TIMEOUT_MS;
            }

            if (settled) {
                final_measurement(loop_data);
            }
            break;
        }
            
        case GrindPhase::TIME_ADDITIONAL_PULSE:
            // Check for additional pulse completion
            if (grinder && grinder->is_pulse_complete()) {
                LOG_BLE("[%lums CONTROLLER] Additional pulse #%d completed, weight: %.2fg\n", 
                        millis(), additional_pulse_count, weight_sensor ? weight_sensor->get_display_weight() : 0.0f);
                
                // Return to completed phase
                switch_phase(GrindPhase::COMPLETED, loop_data);
            }
            break;
            
        case GrindPhase::COMPLETED:
            if (grind_logger.is_logging_active() && !session_end_flash_queued) {
                float error = final_weight - target_weight;
                if (mode == GrindMode::TIME) {
                    error = 0.0f;
                }

                const char* result_string = "COMPLETE";
                switch (last_session_result_) {
                    case GrindSessionResult::OVERSHOOT:
                        result_string = "OVERSHOOT";
                        LOG_BLE("--- RESULT: OVERSHOOT (Error: %+.2fg) ---\n", error);
                        break;
                    case GrindSessionResult::MAX_PULSES:
                        result_string = "COMPLETE - MAX PULSES";
                        LOG_BLE("--- RESULT: COMPLETE - MAX PULSES (Error: %+.2fg) ---\n", error);
                        break;
                    default:
                        LOG_BLE("--- RESULT: COMPLETE (Error: %+.2fg) ---\n", error);
                        break;
                }

                // Queue flash operation for Core 1 processing - no blocking on Core 0
                FlashOpRequest request = {};
                request.operation_type = FlashOpRequest::END_GRIND_SESSION;
                strncpy(request.result_string, result_string, sizeof(request.result_string) - 1);
                request.final_weight = final_weight;
                request.pulse_count = pulse_attempts;

                // Piggyback the session's NVS writes so they land off the control loop
                request.persist_coast_window = coast_window_dirty_;
                request.coast_profile_id = session_descriptor.profile_id;
                request.coast_window = coast_window_;
                coast_window_dirty_ = false;
                request.persist_pulse_window = pulse_window_dirty_;
                request.pulse_window = pulse_window_;
                pulse_window_dirty_ = false;
                request.persist_burst_window = burst_window_dirty_;
                request.burst_window = burst_window_;
                burst_window_dirty_ = false;

                queue_flash_operation(request);
                
                // Mark flash operation as queued to prevent repeated calls
                session_end_flash_queued = true;
            }
            break;
            
        case GrindPhase::TIMEOUT:
            if (grind_logger.is_logging_active() && !session_end_flash_queued) {
                // Queue flash operation for Core 1 processing - no blocking on Core 0
                FlashOpRequest request = {};
                request.operation_type = FlashOpRequest::END_GRIND_SESSION;
                strncpy(request.result_string, "TIMEOUT", sizeof(request.result_string) - 1);
                request.final_weight = final_weight;
                request.pulse_count = pulse_attempts;
                // Pulses measured before the timeout were measured at a clean settle and
                // stay valid; the coast candidate does not, and is discarded elsewhere
                request.persist_pulse_window = pulse_window_dirty_;
                request.pulse_window = pulse_window_;
                pulse_window_dirty_ = false;
                request.persist_burst_window = burst_window_dirty_;
                request.burst_window = burst_window_;
                burst_window_dirty_ = false;
                queue_flash_operation(request);
                
                // Mark flash operation as queued to prevent repeated calls
                session_end_flash_queued = true;
            }
            break;
            
        default:
            break;
    }
    
    // Unified continuous logging for ALL active phases at the control loop rate
    if (session_descriptor.vibration_test && weight_sensor) {
        update_vibration_test(loop_data);
    }

    if (should_log_measurements()) {
        // The raw sample goes in every row alongside the filtered weight, so any filter
        // can be replayed offline against exactly what the controller saw, and noise can
        // be split by motor state and phase. Its age places it in time properly: rows
        // are written at 50Hz but a sample arrives only every 100ms.
        WeightSensor::RawSampleSnapshot raw = {0, 0, 0};
        if (weight_sensor) {
            raw = weight_sensor->get_raw_sample_snapshot();
        }
        uint32_t raw_age_ms = (raw.timestamp_ms != 0 && loop_data.now >= raw.timestamp_ms)
                                  ? (uint32_t)(loop_data.now - raw.timestamp_ms)
                                  : UINT16_MAX;
        if (raw_age_ms > UINT16_MAX) raw_age_ms = UINT16_MAX;

        // Skip rows identical to the last one in everything but the clock. At 50Hz
        // against a 10 SPS load cell most rows were exact repeats, which made a 30s grind
        // ~56KB and meant 50 grinds did not fit the filesystem. The test is exact, not a
        // tolerance: a row is written whenever ANY value the controller acts on differs -
        // a new sample, a sample ageing out of a filter window, a motor or phase edge, a
        // new stop target - so what the controller saw each cycle stays reconstructible.
        const bool row_changed =
            force_measurement_log || !measurement_row_logged_ ||
            raw.seq != last_logged_raw_seq_ ||
            loop_data.motor_is_on != last_logged_motor_on_ ||
            loop_data.phase_id != last_logged_phase_id_ ||
            loop_data.current_weight != last_logged_weight ||
            loop_data.flow_rate != last_logged_flow_rate_ ||
            motor_stop_target_weight != last_logged_stop_target_;

        if (row_changed) {
            grind_logger.log_continuous_measurement(loop_data.timestamp_ms, loop_data.current_weight, loop_data.weight_delta,
                                                   loop_data.flow_rate, loop_data.motor_is_on, loop_data.phase_id, motor_stop_target_weight,
                                                   raw.raw_adc, (uint16_t)raw.seq, (uint16_t)raw_age_ms);

            // Update tracking variables for next measurement
            last_logged_weight = loop_data.current_weight;
            last_logged_time = loop_data.now;
            last_logged_raw_seq_ = raw.seq;
            last_logged_motor_on_ = loop_data.motor_is_on;
            last_logged_phase_id_ = loop_data.phase_id;
            last_logged_flow_rate_ = loop_data.flow_rate;
            last_logged_stop_target_ = motor_stop_target_weight;
            measurement_row_logged_ = true;
            force_measurement_log = false;
        }
    }
    
    // Emit progress update events every cycle for responsive UI
    emit_progress_update(loop_data);

    // Check for negative weight failsafe after TARE_CONFIRM phase during active grinding
    // Only check after motor has settled to avoid false positives from startup transients.
    // Weight mode only - time mode is driven purely by the clock, so a drifting or
    // untared scale must never abort the grind.
    const bool neg_failsafe_armed =
        mode == GrindMode::WEIGHT &&
        phase != GrindPhase::COMPLETED && phase != GrindPhase::TIMEOUT &&
        phase != GrindPhase::IDLE && phase != GrindPhase::INITIALIZING &&
        phase != GrindPhase::SETUP && phase != GrindPhase::TARING &&
        phase != GrindPhase::TARE_CONFIRM && phase != GrindPhase::TIME_PAUSED &&
        phase != GrindPhase::PURGE_CHECK &&  // Handles a lifted portafilter itself, and ~-1g after dumping is what the re-tare fixes
        grinder->is_motor_settled();

    // Counted on raw HX711 samples, not on the filtered weight. The HX711 has no
    // checksum, and one corrupted read - a relay switching an inductive motor next to
    // its data lines is enough - used to end the grind with a -400g error on the spot.
    // Counting the filtered weight would not fix that: the least-squares fit drags a
    // single bad sample through about three consecutive filtered values. Requiring
    // GRIND_NEGATIVE_WEIGHT_CONFIRM_SAMPLES consecutive bad samples means a real lift or
    // a tipped cup (which persist) still stop the grind, ~0.4s later than before.
    if (!neg_failsafe_armed || !weight_sensor) {
        neg_weight_run_ = 0;
    } else {
        WeightSensor::RawSampleSnapshot sample = weight_sensor->get_raw_sample_snapshot();
        if (sample.seq != neg_weight_last_seq_) {
            neg_weight_last_seq_ = sample.seq;
            float cal = weight_sensor->get_calibration_factor();
            float sample_g = (cal != 0.0f)
                                 ? (float)(sample.raw_adc - weight_sensor->get_zero_offset()) / cal
                                 : 0.0f;
            if (sample_g < GRIND_NEGATIVE_WEIGHT_FAILSAFE_G) {
                if (neg_weight_run_ < UINT8_MAX) neg_weight_run_++;
                neg_weight_last_g_ = sample_g;
            } else {
                if (neg_weight_run_ > 0) {
                    // Worth recording: this is exactly the event that used to abort a grind
                    queue_log_message("[FAILSAFE] Ignored %u implausible sample(s), last %.1fg, in %s\n",
                                      (unsigned)neg_weight_run_, neg_weight_last_g_, get_phase_name());
                }
                neg_weight_run_ = 0;
            }
        }
    }

    if (neg_failsafe_armed && neg_weight_run_ >= GRIND_NEGATIVE_WEIGHT_CONFIRM_SAMPLES) {
        timeout_phase = phase;
        grinder->stop();
        last_session_result_ = GrindSessionResult::ERROR;

        // Kept under the 128-byte log message limit with the longest phase name
        queue_log_message("[FAILSAFE] Neg weight: %u samples < %.1fg, last %.1fg, filtered %.1fg, in %s\n",
                         (unsigned)neg_weight_run_, GRIND_NEGATIVE_WEIGHT_FAILSAFE_G, neg_weight_last_g_,
                         loop_data.current_weight, get_phase_name(timeout_phase));
        neg_weight_run_ = 0;
        set_error_message("Err: neg wt");
        switch_phase(GrindPhase::TIMEOUT, loop_data);
    }
    // Only check timeout during active grinding phases, not during completion states, user confirmation, or pause
    else if (phase != GrindPhase::COMPLETED && phase != GrindPhase::TIMEOUT && phase != GrindPhase::PURGE_CONFIRM &&
             phase != GrindPhase::TIME_PAUSED && check_timeout()) {
        timeout_phase = phase;
        grinder->stop();
        last_session_result_ = GrindSessionResult::TIMEOUT;
        
        queue_log_message("--- GRIND TIMEOUT in phase %s ---\n", get_phase_name(timeout_phase));
        char timeout_msg[32];
        const char* phase_name = get_phase_name(timeout_phase);
        if (phase_name && phase_name[0]) {
            char phase_short[5];
            strncpy(phase_short, phase_name, sizeof(phase_short) - 1);
            phase_short[sizeof(phase_short) - 1] = '\0';
            snprintf(timeout_msg, sizeof(timeout_msg), "Timeout:%s", phase_short);
        } else {
            snprintf(timeout_msg, sizeof(timeout_msg), "Timeout");
        }
        set_error_message(timeout_msg);
        switch_phase(GrindPhase::TIMEOUT, loop_data);
    }
}

// OLD predictive_grind method removed - logic now inline in update()

void GrindController::reset_mechanical_anomaly_count() {
    mechanical_anomaly_count_ = 0;
    last_mechanical_event_ms_ = 0;
    last_mechanical_weight_ = 0.0f;
    mechanical_monitor_initialized_ = false;
}

void GrindController::monitor_mechanical_instability(const GrindLoopData& loop_data) {
    // Only monitor during active grinding with motor running. Not during a burst or a
    // knock: shaking the grinder is the point, and it would raise the instability warning.
    bool grinding_active = grinder && grinder->is_grinding() &&
                           phase != GrindPhase::KNOCK_TEST && phase != GrindPhase::BURST;
    if (!grinding_active) {
        mechanical_monitor_initialized_ = false;
        return;
    }

    // Deliberately uses the unsmoothed reading. This looks for a sudden drop, and the
    // regression filter behind current_weight spreads any step across its whole window -
    // a 0.4g slip would arrive as ~0.027g per control cycle and never trip the threshold.
    if (!mechanical_monitor_initialized_) {
        last_mechanical_weight_ = loop_data.instant_weight;
        mechanical_monitor_initialized_ = true;
        return;
    }

    float delta = loop_data.instant_weight - last_mechanical_weight_;
    if (delta <= -GRIND_MECHANICAL_DROP_THRESHOLD_G) {
        if (loop_data.now - last_mechanical_event_ms_ >= GRIND_MECHANICAL_EVENT_COOLDOWN_MS) {
            mechanical_anomaly_count_++;
            last_mechanical_event_ms_ = loop_data.now;
        }
    }

    last_mechanical_weight_ = loop_data.instant_weight;
}

void GrindController::final_measurement(const GrindLoopData& loop_data) {
    final_weight = weight_sensor ? weight_sensor->get_weight_high_latency() : 0.0f;

    if (mode == GrindMode::WEIGHT && target_weight >= GRIND_MIN_TARGET_FOR_DELIVERY_CHECK_G &&
        final_weight < NO_WEIGHT_DELIVERED_THRESHOLD_G) {
        timeout_phase = GrindPhase::FINAL_SETTLING;
        set_error_message("Err: no wt");
        last_session_result_ = GrindSessionResult::ERROR;
        switch_phase(GrindPhase::TIMEOUT, loop_data);
        return;
    }

    // Switch to COMPLETED. The state machine will then transition to IDLE on the next tick.
    switch_phase(GrindPhase::COMPLETED, loop_data);
}


void GrindController::switch_phase(GrindPhase new_phase, const GrindLoopData& loop_data) {
    if (phase == new_phase) return;

    // Check if we have valid loop_data (non-zero timestamp indicates valid data)
    bool has_loop_data = (loop_data.now > 0);
    unsigned long now = has_loop_data ? loop_data.now : millis();

    // Always logged, and so always in the persistent log: the one thing a report taken
    // after a stall has to answer is which phase it stopped in, and with what reading.
    // A handful of lines per grind.
    queue_log_message("[PHASE] %s -> %s at %.1fs, %.2fg\n",
                      get_phase_name(), get_phase_name(new_phase),
                      (start_time > 0 && now >= start_time) ? (now - start_time) / 1000.0f : 0.0f,
                      has_loop_data ? loop_data.current_weight
                                    : (weight_sensor ? weight_sensor->get_weight_low_latency() : 0.0f));
    debug_log_set_grind_active(new_phase != GrindPhase::IDLE);

#if ENABLE_GRIND_DEBUG
    // DEBUG: Log phase transition with boot time and proper phase duration
    unsigned long phase_duration = (phase_start_time > 0) ? (now - phase_start_time) : 0;
    queue_log_message("[DEBUG %lums] PHASE_CHANGE: %s -> %s (phase duration: %lums)\n", 
                  now, get_phase_name(), get_phase_name(new_phase), phase_duration);
#endif
    
    // Finalize and log the event for the phase that just ENDED (only when we have loop_data)
    // Same hand-off rule as should_log_measurements(): the file IO task owns the
    // logger's buffers once the session end has been queued
    if (has_loop_data && !session_end_flash_queued && grind_logger.is_logging_active() &&
        phase != GrindPhase::IDLE) {
        event_in_progress.duration_ms = now - phase_start_time;
        event_in_progress.end_weight = loop_data.current_weight;  // Use pre-calculated weight
        
        // Populate context-specific data for the completed event
        if (phase == GrindPhase::PREDICTIVE) {
            event_in_progress.motor_stop_target_weight = motor_stop_target_weight;
            event_in_progress.grind_latency_ms = grind_latency_ms;
            event_in_progress.pulse_flow_rate = pulse_flow_rate;
            event_in_progress.loop_count = current_phase_loop_count;
        } else if (phase == GrindPhase::PULSE_EXECUTE) {
            event_in_progress.pulse_duration_ms = current_pulse_duration_ms;
            event_in_progress.pulse_attempt_number = pulse_attempts; // attempts is 1-based
            event_in_progress.pulse_flow_rate = pulse_flow_rate;
            event_in_progress.loop_count = current_phase_loop_count;
        } else if (phase == GrindPhase::PULSE_DECISION) {
            event_in_progress.pulse_flow_rate = pulse_flow_rate;
            event_in_progress.loop_count = current_phase_loop_count;
        } else if (phase == GrindPhase::PULSE_SETTLING || phase == GrindPhase::FINAL_SETTLING || phase == GrindPhase::PRIME_SETTLING) {
            event_in_progress.settling_duration_ms = event_in_progress.duration_ms;
            event_in_progress.pulse_flow_rate = pulse_flow_rate;
            event_in_progress.loop_count = current_phase_loop_count;
        }
        
        // For all other phases, just log the general loop count
        if (phase != GrindPhase::PREDICTIVE && phase != GrindPhase::PULSE_EXECUTE && 
            phase != GrindPhase::PULSE_DECISION && phase != GrindPhase::PULSE_SETTLING && 
            phase != GrindPhase::FINAL_SETTLING && phase != GrindPhase::PRIME_SETTLING) {
            event_in_progress.loop_count = current_phase_loop_count;
        }

        grind_logger.log_event(event_in_progress);
    }
    
    // Update phase state
    phase = new_phase;
    control_loop_paused_ = (phase == GrindPhase::PURGE_CONFIRM || phase == GrindPhase::TIME_PAUSED);
    phase_start_time = now;
    
    // Reset loop counter for new phase
    current_phase_loop_count = 0;

    // Set flag to force measurement logging in next update() cycle to capture exact motor state transition
    force_measurement_log = true;

    // Start a new event for the NEW phase (only when we have loop_data and not going to IDLE)
    if (has_loop_data && new_phase != GrindPhase::IDLE) {
        memset(&event_in_progress, 0, sizeof(GrindEvent));
        event_in_progress.phase_id = (uint8_t)new_phase;
        event_in_progress.timestamp_ms = loop_data.timestamp_ms;  // Use pre-calculated timestamp for perfect alignment
        event_in_progress.start_weight = loop_data.current_weight;  // Use pre-calculated weight

        if (session_descriptor.mode == GrindMode::TIME) {
            event_in_progress.event_flags |= GRIND_EVENT_FLAG_TIME_MODE;
        }

        switch (new_phase) {
            case GrindPhase::PRIME:
            case GrindPhase::PREDICTIVE:
            case GrindPhase::TIME_GRINDING:
            case GrindPhase::KNOCK_TEST:
            case GrindPhase::BURST:
                event_in_progress.event_flags |= GRIND_EVENT_FLAG_MOTOR_ACTIVE;
                break;
            case GrindPhase::PULSE_EXECUTE:
                event_in_progress.event_flags |= (GRIND_EVENT_FLAG_MOTOR_ACTIVE | GRIND_EVENT_FLAG_PULSE_PHASE);
                break;
            case GrindPhase::PULSE_SETTLING:
                event_in_progress.event_flags |= GRIND_EVENT_FLAG_PULSE_PHASE;
                break;
            case GrindPhase::PULSE_DECISION:
                // Motor state depends on strategy decision; leave flags unchanged
                break;
            default:
                break;
        }
    }
    
    // pulse_start_time no longer needed for RMT-based pulses
    
    // Emit UI event for phase change
    GrindEventData event_data = {};
    event_data.event = UIGrindEvent::PHASE_CHANGED;
    event_data.phase = new_phase;
    event_data.mode = session_descriptor.mode;
    event_data.current_weight = weight_sensor ? weight_sensor->get_display_weight() : 0.0f;
    event_data.progress_percent = get_progress_percent();
    event_data.phase_display_text = get_phase_name(new_phase);
    event_data.show_taring_text = show_taring_text();
    event_data.time_remaining_ms = get_time_remaining_ms();
    event_data.time_show_weight = (session_descriptor.mode == GrindMode::TIME && session_descriptor.time_use_scale);
    event_data.notice_message = take_pending_notice();

    // Special handling for completion and timeout events
    if (new_phase == GrindPhase::COMPLETED) {
        float error = final_weight - target_weight;
        if (mode == GrindMode::TIME) {
            error = 0.0f;
        }

        GrindSessionResult session_result = GrindSessionResult::SUCCESS;
        if (mode == GrindMode::WEIGHT) {
            if (error > tolerance) {
                session_result = GrindSessionResult::OVERSHOOT;
            } else if (pulse_attempts >= GRIND_MAX_PULSE_ATTEMPTS && fabsf(error) > tolerance) {
                session_result = GrindSessionResult::MAX_PULSES;
            }
        }
        last_session_result_ = session_result;

        // Freshness tracking is RAM-only - see the note in init() for why a
        // since-boot timestamp must not be persisted. A vibration test runs the motor
        // with an empty hopper and moves no coffee, so it must not make stale grounds
        // look fresh and let the next real grind skip its purge. The knock test grinds
        // a real dose and counts.
        if (session_descriptor.vibration_test) {
            finish_vibration_test();
        } else {
            grinder_purged_since_boot = true;
            last_purge_runtime_ms = esp_timer_get_time() / 1000;
        }
        if (session_descriptor.knock_test) {
            // Written before the event is queued, so the UI reads it complete
            knock_result_.final_g = final_weight;
            knock_result_.target_g = target_weight;
            knock_result_.corrections = (uint8_t)pulse_attempts;
        }

        // The grind outcome is settled by this point, so the held coast observation
        // can finally be judged
        commit_coast_observation();

        event_data.event = UIGrindEvent::COMPLETED;
        // Use final_weight if available (from final_measurement), otherwise use high latency weight
        event_data.final_weight = (final_weight > 0) ? final_weight : 
                                 (weight_sensor ? weight_sensor->get_weight_high_latency() : 0.0f);
        
        // For time mode, also indicate pulse availability
        if (mode == GrindMode::TIME) {
            event_data.can_pulse = !is_test_session();
            event_data.pulse_count = additional_pulse_count;
            event_data.pulse_duration_ms = pulse_duration_ms;
        }
    } else if (new_phase == GrindPhase::TIMEOUT) {
        if (last_session_result_ == GrindSessionResult::UNKNOWN) {
            last_session_result_ = GrindSessionResult::TIMEOUT;
        }
        discard_coast_observation("grind aborted");
        event_data.event = UIGrindEvent::TIMEOUT;
        if (last_error_message[0] == '\0') {
            set_error_message("Error");
        }
        event_data.error_message = last_error_message;
        // Use non-blocking high latency weight instead of precision settled weight
        event_data.error_weight = weight_sensor ? weight_sensor->get_weight_high_latency() : 0.0f;
        event_data.error_progress = get_progress_percent();
    } else if (new_phase == GrindPhase::IDLE) {
        event_data.event = UIGrindEvent::STOPPED;
    }
    
    emit_ui_event(event_data);
}


bool GrindController::check_timeout() const {
    // Calculate elapsed time excluding paused states (like PURGE_CONFIRM)
    unsigned long elapsed_ms = millis() - start_time;
    unsigned long active_time_ms = elapsed_ms - timeout_offset_ms;
    return active_time_ms >= (GRIND_TIMEOUT_SEC * 1000);
}

bool GrindController::is_active() const {
    return phase != GrindPhase::IDLE;
}

int GrindController::get_progress_percent() const {
    if (active_strategy && session_descriptor.mode == GrindMode::TIME) {
        return active_strategy->progress_percent(session_descriptor, *this);
    }

    if (target_weight <= 0) return 0;
    
    float ground = (phase == GrindPhase::COMPLETED || phase == GrindPhase::TIMEOUT) 
                   ? final_weight 
                   : (weight_sensor ? weight_sensor->get_display_weight() : 0.0f);
    if (ground < 0) ground = 0;
    int progress = (int)((ground / target_weight) * 100);
    return min(progress, 100);
}

float GrindController::get_grind_time() const {
    if (phase == GrindPhase::IDLE || start_time == 0) return 0.0f;
    return (millis() - start_time) / (float)SYS_MS_PER_SECOND;
}

const char* GrindController::get_phase_name(GrindPhase p) const {
    // Use provided phase, or current phase if not specified
    GrindPhase phase_to_check = (p == static_cast<GrindPhase>(-1)) ? phase : p;
    
    switch (phase_to_check) {
        case GrindPhase::IDLE: return "IDLE";
        case GrindPhase::INITIALIZING: return "INITIALIZING";
        case GrindPhase::SETUP: return "SETUP";
        case GrindPhase::TARING: return "TARING";
        case GrindPhase::TARE_CONFIRM: return "TARE_CONFIRM";
        case GrindPhase::PRIME: return "PRIME";
        case GrindPhase::PRIME_SETTLING: return "PRIME_SETTLING";
        case GrindPhase::PURGE_CONFIRM: return "PURGE_CONFIRM";
        case GrindPhase::PREDICTIVE: return "PREDICTIVE";
        case GrindPhase::PULSE_DECISION: return "PULSE_DECISION";
        case GrindPhase::PULSE_EXECUTE: return "PULSE_EXECUTE";
        case GrindPhase::PULSE_SETTLING: return "PULSE_SETTLING";
        case GrindPhase::FINAL_SETTLING: return "FINAL_SETTLING";
        case GrindPhase::TIME_GRINDING: return "TIME";
        case GrindPhase::TIME_PAUSED: return "PAUSED";
        case GrindPhase::PURGE_CHECK: return "PURGE_CHECK";
        case GrindPhase::KNOCK_TEST: return "KNOCK_TEST";
        case GrindPhase::BURST: return "BURST";
        case GrindPhase::TIME_ADDITIONAL_PULSE: return "PULSE";
        case GrindPhase::COMPLETED: return "COMPLETED";
        case GrindPhase::TIMEOUT: return "TIMEOUT";
        default: return "UNKNOWN";
    }
}



uint8_t GrindController::get_current_phase_id() const {
    return (uint8_t)phase;
}


void GrindController::send_measurements_data() {
    grind_logger.send_current_session_via_serial();
}

float GrindController::get_current_flow_rate() const {
    return weight_sensor->get_flow_rate(); 
}

void GrindController::set_ui_event_callback(void (*callback)(const GrindEventData&)) {
    ui_event_callback = callback;
}

void GrindController::ui_acknowledge_phase_transition() {
    if (phase == GrindPhase::INITIALIZING) {
        ui_ready_for_setup = true;
        LOG_UI_DEBUG("UI acknowledged INITIALIZING phase transition\n");
    }
}

void GrindController::emit_ui_event(const GrindEventData& data) {
    // Thread-safe Core 0 → Core 1 UI event emission using FreeRTOS queue
    if (ui_event_queue) {
        BaseType_t result = xQueueSend(ui_event_queue, &data, 0); // 0 = no wait (non-blocking)
        
        if (result != pdPASS && data.event != UIGrindEvent::PROGRESS_UPDATED) {
            // Progress updates are produced every control cycle and are safe to lose,
            // but a dropped COMPLETED or TIMEOUT strands the UI on the grinding screen
            // with no way back. Evict the oldest entry - almost always a progress
            // update that a newer one has already superseded - and retry once.
            GrindEventData discarded;
            if (xQueueReceive(ui_event_queue, &discarded, 0) == pdPASS) {
                result = xQueueSend(ui_event_queue, &data, 0);
            }
        }

        if (result != pdPASS) {
            // Queue full - drop event to prevent Core 0 blocking
            // Only log dropped significant events, not progress updates
            if (data.event != UIGrindEvent::PROGRESS_UPDATED) {
                LOG_BLE("WARNING: UI event queue full, dropped event type %d\n", (int)data.event);
            }
        } else {
            // Only log significant queued events, not every progress update
            if (data.event != UIGrindEvent::PROGRESS_UPDATED) {
                const char* event_name = "UNKNOWN";
                switch(data.event) {
                    case UIGrindEvent::PHASE_CHANGED: event_name = "PHASE_CHANGED"; break;
                    case UIGrindEvent::PROGRESS_UPDATED: event_name = "PROGRESS_UPDATED"; break;
                    case UIGrindEvent::COMPLETED: event_name = "COMPLETED"; break;
                    case UIGrindEvent::TIMEOUT: event_name = "TIMEOUT"; break;
                    case UIGrindEvent::STOPPED: event_name = "STOPPED"; break;
                    case UIGrindEvent::BACKGROUND_CHANGE: event_name = "BACKGROUND_CHANGE"; break;
                    case UIGrindEvent::PULSE_AVAILABLE: event_name = "PULSE_AVAILABLE"; break;
                    case UIGrindEvent::PULSE_STARTED: event_name = "PULSE_STARTED"; break;
                    case UIGrindEvent::PULSE_COMPLETED: event_name = "PULSE_COMPLETED"; break;
                }
                LOG_BLE("[%lums UI_EVENT] QUEUED %s: phase=%s, weight=%.2fg, progress=%d%%\n", 
                        millis(), event_name, data.phase_display_text, data.current_weight, data.progress_percent);
            }
        }
    }
}

void GrindController::emit_progress_update(const GrindLoopData& loop_data) {
    GrindEventData progress_event = {};
    progress_event.event = UIGrindEvent::PROGRESS_UPDATED;
    progress_event.phase = phase;
    progress_event.mode = session_descriptor.mode;
    progress_event.current_weight = (phase == GrindPhase::COMPLETED || phase == GrindPhase::TIMEOUT)
                                    ? final_weight
                                    : loop_data.display_weight;
    progress_event.progress_percent = get_progress_percent();
    progress_event.phase_display_text = get_phase_name();
    progress_event.show_taring_text = show_taring_text();
    progress_event.flow_rate = loop_data.flow_rate;
    progress_event.time_remaining_ms = get_time_remaining_ms();
    progress_event.time_show_weight = (session_descriptor.mode == GrindMode::TIME && session_descriptor.time_use_scale);
    progress_event.notice_message = take_pending_notice();
    emit_ui_event(progress_event);
}

//==============================================================================
// CORE 0 HELPER METHODS
//==============================================================================


bool GrindController::should_log_measurements() const {
    // Once the session has been handed to the file IO task, this core must not touch
    // the logger again - that task clears the shared PSRAM buffers while finishing the
    // session. COMPLETED and TIMEOUT are excluded below, but TIME_ADDITIONAL_PULSE is
    // reachable afterwards via the pulse button and would otherwise resume writing.
    return SYS_CONTINUOUS_LOGGING_ENABLED
        && !session_end_flash_queued
        && phase != GrindPhase::INITIALIZING
        && phase != GrindPhase::SETUP
        && phase != GrindPhase::COMPLETED
        && phase != GrindPhase::TIMEOUT
        && phase != GrindPhase::PURGE_CONFIRM   // Don't log while waiting for user to confirm purge
        && phase != GrindPhase::TIME_PAUSED;    // Don't log while time grind is paused
}

void GrindController::process_queued_ui_events() {
    GrindEventData event;
    
    // Process all queued events from Core 0
    while (xQueueReceive(ui_event_queue, &event, 0) == pdPASS) {
        if (ui_event_callback) {
            ui_event_callback(event); // Safe - runs on Core 1
        }
    }
}

void GrindController::queue_flash_operation(const FlashOpRequest& request) {
    // Thread-safe Core 0 → Core 1 flash operation queuing
    if (flash_op_queue) {
        BaseType_t result = xQueueSend(flash_op_queue, &request, 0); // 0 = no wait (non-blocking)
        
        if (result != pdPASS) {
            // Queue full - this shouldn't happen with reasonable queue size
            LOG_BLE("WARNING: Flash operation queue full, dropping request type %d\n", (int)request.operation_type);
        } else {
            const char* op_name = (request.operation_type == FlashOpRequest::END_GRIND_SESSION)
                                   ? "END_GRIND_SESSION" : "START_GRIND_SESSION";
            LOG_BLE("[%lums FLASH_OP] QUEUED %s operation for Core 1 processing\n", millis(), op_name);
        }
    }
}

void GrindController::process_queued_flash_operations() {
    FlashOpRequest request;
    
    // Process all queued flash operations from Core 0
    while (xQueueReceive(flash_op_queue, &request, 0) == pdPASS) {
        switch (request.operation_type) {
            case FlashOpRequest::START_GRIND_SESSION:
                // Perform the blocking flash operation on Core 1
                LOG_BLE("[%lums FLASH_OP] Processing START_GRIND_SESSION on Core 1: mode=%s, profile=%d\n", 
                        millis(),
                        request.descriptor.mode == GrindMode::TIME ? "TIME" : "WEIGHT",
                        request.descriptor.profile_id);
                grind_logger.start_grind_session(request.descriptor, request.start_weight);
                break;
                
            case FlashOpRequest::END_GRIND_SESSION:
                // Perform the blocking flash operation on Core 1
                LOG_BLE("[%lums FLASH_OP] Processing END_GRIND_SESSION on Core 1: %s, %.2fg, %d pulses\n", 
                        millis(), request.result_string, request.final_weight, request.pulse_count);
                grind_logger.end_grind_session(request.result_string, request.final_weight, request.pulse_count);

                // Session settings are persisted here rather than on the control loop,
                // where a blocking flash write would stall grind timing
                if (preferences) {
                    // Refuse to write a window that would fail validation on the way
                    // back in. Storing one would leave the profile permanently seeded
                    // from latency, since the load path resets anything it cannot trust.
                    if (request.persist_coast_window && coast_window_is_valid(request.coast_window)) {
                        char key[16];
                        snprintf(key, sizeof(key), "%s%u", PREF_KEY_COAST_WINDOW_PREFIX,
                                 (unsigned)request.coast_profile_id);
                        preferences->putBytes(key, &request.coast_window, sizeof(CoastWindow));
                    }
                    if (request.persist_pulse_window && pulse_window_is_valid(request.pulse_window)) {
                        preferences->putBytes(PREF_KEY_PULSE_WINDOW, &request.pulse_window, sizeof(PulseWindow));
                    }
                    if (request.persist_burst_window && pulse_window_is_valid(request.burst_window)) {
                        preferences->putBytes(PREF_KEY_BURST_WINDOW, &request.burst_window, sizeof(PulseWindow));
                    }
                }
                // Session outcome is final by now - coast entry judged, any error recorded
                persist_history();
                break;

            case FlashOpRequest::DISCARD_GRIND_SESSION:
                LOG_BLE("[%lums FLASH_OP] Processing DISCARD_GRIND_SESSION on Core 1\n", millis());
                grind_logger.discard_current_session();
                // A stopped grind can still have produced a coast entry, now marked discarded
                persist_history();
                break;

            default:
                LOG_BLE("WARNING: Unknown flash operation type %d\n", request.operation_type);
                break;
        }
    }
}

void GrindController::queue_log_message(const char* format, ...) {
    // Thread-safe Core 0 → Core 1 log message queuing
    if (log_queue) {
        LogMessage log_msg;
        
        // Format the message using va_list
        va_list args;
        va_start(args, format);
        vsnprintf(log_msg.message, sizeof(log_msg.message), format, args);
        va_end(args);
        
        // Ensure null termination
        log_msg.message[sizeof(log_msg.message) - 1] = '\0';
        
        BaseType_t result = xQueueSend(log_queue, &log_msg, 0); // 0 = no wait (non-blocking)
        
        if (result != pdPASS) {
            // Queue full - silently drop the message to avoid blocking Core 0
            // Don't log this error as it could cause recursion
        }
    }
}

void GrindController::process_queued_log_messages() {
    LogMessage log_msg;
    
    // Process all queued log messages from Core 0
    while (xQueueReceive(log_queue, &log_msg, 0) == pdPASS) {
        // Output the message using LOG_BLE on Core 1
        LOG_BLE("%s", log_msg.message);
    }
}

void GrindController::set_error_message(const char* message) {
    if (!message || !message[0]) {
        last_error_message[0] = '\0';
        return;
    }
    strncpy(last_error_message, message, sizeof(last_error_message) - 1);
    last_error_message[sizeof(last_error_message) - 1] = '\0';

    // Keep a short history so a fault that has since been acknowledged, and its
    // message overwritten, is still visible in the diagnostic report
    ErrorRecord record;
    memset(&record, 0, sizeof(record));
    record.uptime_s = (uint32_t)(millis() / 1000);
    record.boot_seq = boot_seq_;
    strncpy(record.message, last_error_message, sizeof(record.message) - 1);
    record.message[sizeof(record.message) - 1] = '\0';
    const char* phase_name = get_phase_name();
    strncpy(record.phase, phase_name ? phase_name : "?", sizeof(record.phase) - 1);
    record.phase[sizeof(record.phase) - 1] = '\0';

    portENTER_CRITICAL(&history_mux_);
    error_history_[error_history_next_] = record;
    error_history_next_ = (error_history_next_ + 1) % ERROR_HISTORY_SIZE;
    if (error_history_count_ < ERROR_HISTORY_SIZE) {
        error_history_count_++;
    }
    portEXIT_CRITICAL(&history_mux_);
}

bool GrindController::read_coast_window(Preferences& prefs, uint8_t profile_id, CoastWindow& out) {
    coast_window_reset(out);

    char key[16];
    snprintf(key, sizeof(key), "%s%u", PREF_KEY_COAST_WINDOW_PREFIX, (unsigned)profile_id);

    // getBytes into a scratch copy first: a short read leaves the destination partly
    // written, and validating a half-filled struct is not the same as rejecting it.
    CoastWindow scratch;
    coast_window_reset(scratch);
    // getBytes returns the STORED length when it differs from the buffer size, and
    // having already copied that many bytes in if it was the shorter of the two, so a
    // short blob leaves scratch part-written. Requiring an exact-size read rejects
    // both that and a missing key, and scratch is discarded either way.
    size_t read = prefs.getBytes(key, &scratch, sizeof(CoastWindow));
    if (read == sizeof(CoastWindow) && coast_window_is_valid(scratch) && scratch.count > 0) {
        out = scratch;
        return true;
    }

    // No usable window. A device updated from a build that stored a single averaged
    // value still has one, and throwing it away would make the next grind guess coast
    // from spin-up latency, which is a far worse starting point than a real
    // measurement. Seed the window with it and let real measurements displace it.
    char legacy_key[16];
    snprintf(legacy_key, sizeof(legacy_key), "%s%u", PREF_KEY_COAST_TIME_PREFIX, (unsigned)profile_id);
    float legacy = prefs.getFloat(legacy_key, 0.0f);
    if (isfinite(legacy) && legacy >= GRIND_COAST_TIME_MIN_S && legacy <= GRIND_COAST_TIME_MAX_S) {
        coast_window_push(out, legacy);
        return true;
    }
    return false;
}

bool GrindController::read_burst_window(Preferences& prefs, PulseWindow& out) {
    pulse_window_reset(out);
    PulseWindow scratch;
    pulse_window_reset(scratch);
    // Exact-size read into a scratch copy, as for the pulse window
    size_t read = prefs.getBytes(PREF_KEY_BURST_WINDOW, &scratch, sizeof(PulseWindow));
    if (read == sizeof(PulseWindow) && pulse_window_is_valid(scratch)) {
        out = scratch;
        return out.count > 0;
    }
    return false;
}

bool GrindController::read_pulse_window(Preferences& prefs, PulseWindow& out) {
    pulse_window_reset(out);
    PulseWindow scratch;
    pulse_window_reset(scratch);
    // Exact-size read into a scratch copy, as for the coast window: a short or
    // mismatched blob must be rejected, not half-applied
    size_t read = prefs.getBytes(PREF_KEY_PULSE_WINDOW, &scratch, sizeof(PulseWindow));
    if (read == sizeof(PulseWindow) && pulse_window_is_valid(scratch)) {
        out = scratch;
        return out.count > 0;
    }
    return false;
}

void GrindController::load_pulse_window() {
    pulse_window_reset(pulse_window_);
    pulse_window_dirty_ = false;
    if (preferences) {
        read_pulse_window(*preferences, pulse_window_);
    }
}

void GrindController::observe_pulse(int index, float settled_after_g) {
    if (index < 0 || index >= GRIND_MAX_PULSE_ATTEMPTS) {
        return;
    }
    PulseReport& pulse = pulse_history[index];
    pulse.end_weight = settled_after_g;

    const float delivered = settled_after_g - pulse.start_weight;
    const float grinding_ms = pulse.duration_ms - motor_response_latency_ms;
    const float from_flow = pulse.flow_gps * (grinding_ms > 0.0f ? grinding_ms : 0.0f) / 1000.0f;
    const float extra = delivered - from_flow;

    // Same gates as a coast measurement: a disturbed settle, or a delivery no pulse
    // could produce, says nothing about pulses
    const char* reject = nullptr;
    if (mechanical_anomaly_count_ != pulse_anomaly_at_start_) {
        reject = "scale disturbed";
    } else if (!isfinite(delivered) || delivered < GRIND_PULSE_DELIVERED_MIN_G ||
               delivered > GRIND_PULSE_DELIVERED_MAX_G) {
        reject = "implausible delivery";
    } else if (session_descriptor.knock_test) {
        reject = "knock test";  // The knock already shook loose what a pulse would
    }

    if (reject) {
        queue_log_message("[PULSE] #%d %.0fms: delivered %+.3fg - not learned (%s)\n",
                          index + 1, pulse.duration_ms, delivered, reject);
        return;
    }

    pulse_window_push(pulse_window_, extra);
    pulse_window_dirty_ = true;
    queue_log_message("[PULSE] #%d %.0fms: delivered %+.3fg, planned %+.3fg, extra %+.3fg -> model now %+.3fg\n",
                      index + 1, pulse.duration_ms, delivered, pulse.expected_g, extra,
                      pulse_window_expected_extra_g(pulse_window_));
}

void GrindController::load_coast_time(uint8_t profile_id) {
    coast_window_reset(coast_window_);
    coast_window_dirty_ = false;

    if (!preferences) {
        return;
    }

    if (read_coast_window(*preferences, profile_id, coast_window_)) {
        LOG_BLE("[CONTROLLER] Profile %u coast window: %u samples, predicting %.3fs\n",
                (unsigned)profile_id, (unsigned)coast_window_.count,
                coast_window_prediction_s(coast_window_));
    } else {
        LOG_BLE("[CONTROLLER] Profile %u has no coast history - seeding from latency\n",
                (unsigned)profile_id);
    }
}

void GrindController::log_coast_window(const char* prefix) {
    // One line listing every measurement the prediction was chosen from, so a report
    // read days later shows the spread rather than just the number that came out of it.
    float sorted[CoastWindow::CAPACITY];
    int n = coast_window_sorted(coast_window_, sorted, CoastWindow::CAPACITY);

    // Eight samples print as "0.318 " each, so 64 bytes holds the full window with room
    // to spare and still leaves the fixed text inside the 128-byte log message buffer.
    char list[64];
    int offset = 0;
    for (int i = 0; i < n; i++) {
        int written = snprintf(list + offset, sizeof(list) - offset, "%s%.3f",
                               (i > 0) ? " " : "", sorted[i]);
        // snprintf returns what it WOULD have written, so a truncated tail must stop
        // the loop rather than advance the offset past the end of the buffer
        if (written < 0 || (size_t)written >= sizeof(list) - offset) {
            break;
        }
        offset += written;
    }
    list[sizeof(list) - 1] = '\0';
    if (n == 0) {
        snprintf(list, sizeof(list), "empty");
    }

    queue_log_message("%s window [%s] n=%u -> %.3fs +%.3fs pad = %.3fs\n",
                      prefix, list, (unsigned)coast_window_.count,
                      coast_window_quantile_s(coast_window_), GRIND_COAST_TAIL_PAD_S,
                      coast_window_prediction_s(coast_window_));
}

void GrindController::record_coast_observation(float observed_s, float coast_weight_g,
                                               float flow_rate_gps, bool accepted,
                                               const char* reason) {
    CoastObservation entry;
    memset(&entry, 0, sizeof(entry));
    entry.uptime_s = (uint32_t)(millis() / 1000);
    entry.boot_seq = boot_seq_;
    entry.predicted_s = coast_predicted_s_;
    entry.predicted_weight_g = motor_stop_target_weight;
    entry.observed_s = observed_s;
    entry.coast_weight_g = coast_weight_g;
    entry.flow_rate_gps = flow_rate_gps;
    entry.target_weight_g = target_weight;
    // Filled in later if the grind gets far enough to be judged
    entry.final_weight_g = 0.0f;
    entry.error_g = 0.0f;
    entry.pulse_count = 0;
    entry.profile_id = session_descriptor.profile_id;
    entry.accepted = accepted;
    if (reason && reason[0]) {
        strncpy(entry.reason, reason, sizeof(entry.reason) - 1);
        entry.reason[sizeof(entry.reason) - 1] = '\0';
    } else {
        entry.reason[0] = '\0';
    }

    // Built above and published in one step, so the file IO task snapshotting the
    // history for flash never sees a half-written entry
    portENTER_CRITICAL(&history_mux_);
    CoastObservation& slot = coast_history_[coast_history_next_];
    slot = entry;
    // Only a still-undecided entry may be revisited later; a rejection is final
    pending_observation_ = accepted ? &slot : nullptr;
    coast_history_next_ = (coast_history_next_ + 1) % COAST_HISTORY_SIZE;
    if (coast_history_count_ < COAST_HISTORY_SIZE) {
        coast_history_count_++;
    }
    portEXIT_CRITICAL(&history_mux_);
}

const CoastObservation* GrindController::get_coast_history_entry(int index_from_newest) const {
    if (index_from_newest < 0 || index_from_newest >= coast_history_count_) {
        return nullptr;
    }
    int slot = (coast_history_next_ - 1 - index_from_newest + COAST_HISTORY_SIZE * 2) % COAST_HISTORY_SIZE;
    return &coast_history_[slot];
}

bool GrindController::get_error_history_entry(int index_from_newest, uint32_t* uptime_s_out,
                                              const char** message_out,
                                              const char** phase_out,
                                              uint16_t* boot_seq_out) const {
    if (index_from_newest < 0 || index_from_newest >= error_history_count_) {
        return false;
    }
    int slot = (error_history_next_ - 1 - index_from_newest + ERROR_HISTORY_SIZE * 2) % ERROR_HISTORY_SIZE;
    if (uptime_s_out) *uptime_s_out = error_history_[slot].uptime_s;
    if (message_out) *message_out = error_history_[slot].message;
    if (phase_out) *phase_out = error_history_[slot].phase;
    if (boot_seq_out) *boot_seq_out = error_history_[slot].boot_seq;
    return true;
}

//------------------------------------------------------------------------------
// Persistent coast and error history
//------------------------------------------------------------------------------
// Both histories used to live only in RAM, so a restart - including the manual kind
// after a hang - erased exactly the record that would have explained it. One small
// fixed-size file holds them; it is replaced whole, never grown, so it cannot fill
// the partition.

namespace {
constexpr uint32_t kHistoryMagic = 0x48495354;  // "HIST"
constexpr uint16_t kHistoryVersion = 1;

struct PersistentHistoryHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t boot_seq;            // Boot that last wrote the file
    uint16_t coast_record_size;   // sizeof(CoastObservation) when written
    uint16_t error_record_size;
    uint8_t coast_capacity;
    uint8_t coast_count;
    uint8_t coast_next;
    uint8_t error_capacity;
    uint8_t error_count;
    uint8_t error_next;
    uint8_t reserved[2];
};
}  // namespace

void GrindController::load_persistent_history() {
    boot_seq_ = 1;

    // A save interrupted by power loss can leave only the temporary copy behind
    const char* path = LittleFS.exists(GRIND_HISTORY_FILE) ? GRIND_HISTORY_FILE
                     : LittleFS.exists(GRIND_HISTORY_TEMP_FILE) ? GRIND_HISTORY_TEMP_FILE
                     : nullptr;
    if (!path) {
        LOG_BLE("[HISTORY] No saved history - starting at boot 1\n");
        return;
    }

    File file = LittleFS.open(path, "r");
    if (!file) {
        LOG_BLE("[HISTORY] Could not open %s\n", path);
        return;
    }

    PersistentHistoryHeader header;
    bool header_ok = file.read(reinterpret_cast<uint8_t*>(&header), sizeof(header)) == sizeof(header) &&
                     header.magic == kHistoryMagic;
    if (header_ok) {
        // The boot counter is worth keeping even if the records are unusable
        boot_seq_ = (uint16_t)(header.boot_seq + 1);
        if (boot_seq_ == 0) boot_seq_ = 1;
    }

    // Records are only restored if they match today's layout exactly. After a firmware
    // change to either struct they are dropped rather than misread.
    bool layout_ok = header_ok && header.version == kHistoryVersion &&
                     header.coast_record_size == sizeof(CoastObservation) &&
                     header.error_record_size == sizeof(ErrorRecord) &&
                     header.coast_capacity == COAST_HISTORY_SIZE &&
                     header.error_capacity == ERROR_HISTORY_SIZE &&
                     header.coast_count <= COAST_HISTORY_SIZE && header.coast_next < COAST_HISTORY_SIZE &&
                     header.error_count <= ERROR_HISTORY_SIZE && header.error_next < ERROR_HISTORY_SIZE;

    CoastObservation coast[COAST_HISTORY_SIZE];
    ErrorRecord errors[ERROR_HISTORY_SIZE];
    bool records_ok = layout_ok &&
                      file.read(reinterpret_cast<uint8_t*>(coast), sizeof(coast)) == sizeof(coast) &&
                      file.read(reinterpret_cast<uint8_t*>(errors), sizeof(errors)) == sizeof(errors);
    file.close();

    if (!records_ok) {
        LOG_BLE("[HISTORY] Saved history unusable (%s) - kept boot counter only, now boot %u\n",
                header_ok ? "layout changed" : "bad header", (unsigned)boot_seq_);
        return;
    }

    // Strings came from flash: terminate them regardless of what was stored
    for (int i = 0; i < COAST_HISTORY_SIZE; i++) {
        coast[i].reason[sizeof(coast[i].reason) - 1] = '\0';
    }
    for (int i = 0; i < ERROR_HISTORY_SIZE; i++) {
        errors[i].message[sizeof(errors[i].message) - 1] = '\0';
        errors[i].phase[sizeof(errors[i].phase) - 1] = '\0';
    }

    portENTER_CRITICAL(&history_mux_);
    memcpy(coast_history_, coast, sizeof(coast));
    coast_history_count_ = header.coast_count;
    coast_history_next_ = header.coast_next;
    memcpy(error_history_, errors, sizeof(errors));
    error_history_count_ = header.error_count;
    error_history_next_ = header.error_next;
    pending_observation_ = nullptr;
    portEXIT_CRITICAL(&history_mux_);

    LOG_BLE("[HISTORY] Restored %u grinds and %u errors - now boot %u\n",
            (unsigned)header.coast_count, (unsigned)header.error_count, (unsigned)boot_seq_);
}

void GrindController::persist_history() {
    // Core 1 only: snapshot under the lock, then write with the lock released
    PersistentHistoryHeader header;
    memset(&header, 0, sizeof(header));
    // Snapshot buffers in PSRAM, allocated once - file IO task is the only caller.
    // Kept off its stack and out of internal RAM, which is what runs out first.
    const size_t coast_bytes = sizeof(CoastObservation) * COAST_HISTORY_SIZE;
    const size_t error_bytes = sizeof(ErrorRecord) * ERROR_HISTORY_SIZE;
    static uint8_t* snapshot = nullptr;
    if (!snapshot) {
        snapshot = static_cast<uint8_t*>(heap_caps_malloc_prefer(coast_bytes + error_bytes, 2,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_8BIT));
        if (!snapshot) {
            LOG_BLE("[HISTORY] No memory for snapshot - not saved\n");
            return;
        }
    }
    CoastObservation* coast = reinterpret_cast<CoastObservation*>(snapshot);
    ErrorRecord* errors = reinterpret_cast<ErrorRecord*>(snapshot + coast_bytes);

    portENTER_CRITICAL(&history_mux_);
    memcpy(coast, coast_history_, coast_bytes);
    memcpy(errors, error_history_, error_bytes);
    header.coast_count = coast_history_count_;
    header.coast_next = coast_history_next_;
    header.error_count = error_history_count_;
    header.error_next = error_history_next_;
    portEXIT_CRITICAL(&history_mux_);

    header.magic = kHistoryMagic;
    header.version = kHistoryVersion;
    header.boot_seq = boot_seq_;
    header.coast_record_size = sizeof(CoastObservation);
    header.error_record_size = sizeof(ErrorRecord);
    header.coast_capacity = COAST_HISTORY_SIZE;
    header.error_capacity = ERROR_HISTORY_SIZE;

    // Write a complete copy, then swap it in. Power lost mid-write leaves the previous
    // file intact; lost between remove and rename leaves the temporary copy, which
    // load_persistent_history() falls back to.
    File file = LittleFS.open(GRIND_HISTORY_TEMP_FILE, "w");
    if (!file) {
        LOG_BLE("[HISTORY] Could not write %s\n", GRIND_HISTORY_TEMP_FILE);
        return;
    }
    size_t written = file.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
    written += file.write(snapshot, coast_bytes + error_bytes);
    file.close();

    const size_t expected = sizeof(header) + coast_bytes + error_bytes;
    if (written != expected) {
        LittleFS.remove(GRIND_HISTORY_TEMP_FILE);
        LOG_BLE("[HISTORY] Short write (%u of %u bytes) - previous history kept\n",
                (unsigned)written, (unsigned)expected);
        return;
    }
    LittleFS.remove(GRIND_HISTORY_FILE);
    if (!LittleFS.rename(GRIND_HISTORY_TEMP_FILE, GRIND_HISTORY_FILE)) {
        LOG_BLE("[HISTORY] Rename failed - history left in %s\n", GRIND_HISTORY_TEMP_FILE);
    }
}

void GrindController::mark_coast_window_start() {
    // Only instability during the settle can corrupt the coast reading; a bump earlier
    // in the grind is irrelevant, so the baseline is taken here rather than at start
    anomaly_count_at_motor_stop_ = mechanical_anomaly_count_;
}

void GrindController::observe_coast(float coast_weight_g) {
    // Once per grind: the first settle after the predictive stop. A knock test comes
    // back through the pulse decision after its knock, before any pulse has fired.
    if (coast_observed_) {
        return;
    }
    coast_observed_ = true;

    // A burst fired straight after the stop lands in this same settle. Take out what it
    // was expected to bring so the window keeps holding coast: the next prediction adds
    // the expected burst back, so the total it stops short by is still a tail this
    // grinder really produced, even when the burst figure is off.
    if (burst_fired_) {
        const float tail_g = coast_weight_g;
        coast_weight_g -= burst_expected_g_;
        queue_log_message("[BURST] Tail %+.3fg = coast %+.3fg + expected burst %+.3fg\n",
                          tail_g, coast_weight_g, burst_expected_g_);
    }

    // Convert the observed coast weight into a time using the SAME flow figure the
    // prediction was built on, so the multiply and divide cancel. Using pulse_flow_rate
    // here - a 95th percentile, and so higher than the mean the prediction uses - made
    // every learned coast come out short by that ratio, roughly 20-25%, which showed up
    // as a persistent overshoot no amount of headroom could cover.
    float flow_rate = (coast_prediction_flow_gps > 0.0f) ? coast_prediction_flow_gps
                                                         : pulse_flow_rate;
    if (!isfinite(coast_weight_g) || flow_rate < GRIND_FLOW_RATE_MIN_SANE_GPS) {
        // Recorded rather than dropped. This fires when the predictive phase never got
        // a usable flow reading, so the stop happened on the fallback undershoot target
        // instead of a real prediction. Returning quietly left the grind missing from
        // the history entirely, which reads as "the model learned nothing" without
        // saying why - the one question the report exists to answer.
        queue_log_message("[COAST] Rejected - no usable flow rate (%.2fg/s)\n", flow_rate);
        record_coast_observation(0.0f, coast_weight_g, flow_rate, false, "no flow rate");
        return;
    }

    // A single wobble during the settle is enough to discard. The instability
    // diagnostic needs three before it complains, but one bad coast reading skews
    // the average for the next several grinds, so this is deliberately stricter.
    if (mechanical_anomaly_count_ != anomaly_count_at_motor_stop_) {
        queue_log_message("[COAST] Rejected - scale disturbed during settle (%d events)\n",
                          mechanical_anomaly_count_ - anomaly_count_at_motor_stop_);
        record_coast_observation(0.0f, coast_weight_g, flow_rate, false, "scale disturbed");
        return;
    }

    float observed_s = coast_weight_g / flow_rate;
    if (observed_s < GRIND_COAST_TIME_MIN_S || observed_s > GRIND_COAST_TIME_MAX_S) {
        queue_log_message("[COAST] Rejected observation %.3fs (%.2fg at %.2fg/s)\n",
                          observed_s, coast_weight_g, flow_rate);
        record_coast_observation(observed_s, coast_weight_g, flow_rate, false, "out of range");
        return;
    }

    // The knock test measures a real coast, but test sessions are kept out of the
    // models ordinary grinds are planned from
    if (session_descriptor.knock_test) {
        queue_log_message("[COAST] Knock test: measured %.3fs (%.2fg at %.2fg/s) - not learned\n",
                          observed_s, coast_weight_g, flow_rate);
        record_coast_observation(observed_s, coast_weight_g, flow_rate, false, "knock test");
        return;
    }

    // Held, not applied: whether this grind actually hit its target is not known
    // until it finishes, and a grind that missed teaches the wrong coast
    pending_coast_time_s_ = observed_s;
    record_coast_observation(observed_s, coast_weight_g, flow_rate, true, nullptr);
    queue_log_message("[COAST] Predicted %.3fs (%.2fg), measured %.3fs (%.2fg at %.2fg/s)\n",
                      coast_predicted_s_, motor_stop_target_weight,
                      observed_s, coast_weight_g, flow_rate);
}

void GrindController::discard_coast_observation(const char* reason) {
    if (pending_coast_time_s_ > 0.0f) {
        queue_log_message("[COAST] Discarded candidate %.3fs - %s\n",
                          pending_coast_time_s_, reason ? reason : "grind did not complete");
        portENTER_CRITICAL(&history_mux_);
        if (pending_observation_) {
            pending_observation_->accepted = false;
            strncpy(pending_observation_->reason, reason ? reason : "not completed",
                    sizeof(pending_observation_->reason) - 1);
            pending_observation_->reason[sizeof(pending_observation_->reason) - 1] = '\0';
            pending_observation_ = nullptr;
        }
        portEXIT_CRITICAL(&history_mux_);
        pending_coast_time_s_ = 0.0f;
    }
}

void GrindController::commit_coast_observation() {
    if (pending_coast_time_s_ <= 0.0f) {
        return;
    }

    // Missing the target does NOT invalidate the measurement. Coast is measured at the
    // first settle after the motor stops, before any correction pulse runs, so it
    // records what was actually in flight regardless of where the grind finally landed.
    // Refusing to learn from a miss was self-defeating: a profile predicting coast
    // badly would overshoot, have its observation thrown away, and so never improve -
    // it could not climb out of a bad starting point. An overshoot is precisely the
    // evidence that coast is larger than believed, which is what the model needs.
    //
    // What genuinely does invalidate it is handled elsewhere: a disturbed scale or an
    // out-of-range value in observe_coast(), and an aborted grind, which never reaches
    // here at all.
    float error = final_weight - target_weight;

    float observed_s = pending_coast_time_s_;
    pending_coast_time_s_ = 0.0f;
    portENTER_CRITICAL(&history_mux_);
    if (pending_observation_) {
        pending_observation_->final_weight_g = final_weight;
        pending_observation_->error_g = error;
        pending_observation_->pulse_count = (uint8_t)pulse_attempts;
        pending_observation_ = nullptr;
    }
    portEXIT_CRITICAL(&history_mux_);

    // The measurement simply joins the window; nothing is averaged and no rate decides
    // how far the model moves. What the next grind predicts is whichever of the stored
    // measurements the rank selects, so a single grind can only change the prediction
    // by displacing the oldest sample - never by dragging an estimate toward itself.
    // That is what stops the model hunting: there is no accumulator to ratchet.
    coast_window_push(coast_window_, observed_s);
    coast_window_dirty_ = true;

    queue_log_message("[COAST] Accepted %.3fs (%s, %.2fg vs %.2fg target, %+.2fg, %d pulses)\n",
                      observed_s,
                      (fabsf(error) <= GRIND_ACCURACY_TOLERANCE_G) ? "on target"
                          : (error > 0.0f ? "OVERSHOT" : "undershot"),
                      final_weight, target_weight, error, pulse_attempts);
    log_coast_window("[COAST]");
}


void GrindController::set_notice_message(const char* message) {
    if (!message || !message[0]) {
        last_notice_message[0] = '\0';
        notice_pending_ = false;
        return;
    }
    strncpy(last_notice_message, message, sizeof(last_notice_message) - 1);
    last_notice_message[sizeof(last_notice_message) - 1] = '\0';
    notice_pending_ = true;
}

const char* GrindController::take_pending_notice() {
    if (!notice_pending_) {
        return nullptr;
    }
    notice_pending_ = false;
    return last_notice_message;
}

void GrindController::execute_additional_pulse() {
    if (!can_pulse()) {
        return;
    }
    
    if (!grinder) {
        LOG_BLE("ERROR: Cannot pulse - grinder not available\n");
        return;
    }
    
    additional_pulse_count++;

    // Update statistics for time mode pulse
    statistics_manager.update_time_pulse();

    // Reset timeout timer to prevent timeout during additional pulses
    start_time = millis();

    LOG_BLE("[%lums CONTROLLER] Starting additional pulse #%d (%lums) - timeout timer reset\n",
            millis(), additional_pulse_count, (unsigned long)pulse_duration_ms);

    // Transition to additional pulse phase (without loop_data since this is a manual action)
    GrindLoopData empty_loop_data = {};
    empty_loop_data.now = millis();
    switch_phase(GrindPhase::TIME_ADDITIONAL_PULSE, empty_loop_data);

    // Start the pulse
    grinder->start_pulse_rmt(pulse_duration_ms);
    
    // Notify mock driver for weight simulation (if mock is active)
#if defined(DEBUG_ENABLE_LOADCELL_MOCK) && (DEBUG_ENABLE_LOADCELL_MOCK != 0)
    MockHX711Driver::notify_pulse(pulse_duration_ms);
#endif
}

bool GrindController::can_pulse() const {
    // Only allow pulses in time mode when grind is completed and not in pulse phase
    return mode == GrindMode::TIME &&
           phase == GrindPhase::COMPLETED &&
           !is_test_session();
}

//==============================================================================
// Motor Response Latency Management
//==============================================================================

void GrindController::load_motor_latency() {
    if (!preferences) {
        motor_response_latency_ms = GRIND_MOTOR_RESPONSE_LATENCY_DEFAULT_MS;
        LOG_BLE("Motor latency: Using default %.1fms (no preferences)\n", motor_response_latency_ms);
        return;
    }

    motor_response_latency_ms = preferences->getFloat("motor_lat_ms", GRIND_MOTOR_RESPONSE_LATENCY_DEFAULT_MS);

    // Validate loaded value
    if (motor_response_latency_ms < GRIND_AUTOTUNE_LATENCY_MIN_MS ||
        motor_response_latency_ms > GRIND_AUTOTUNE_LATENCY_MAX_MS) {
        LOG_BLE("Warning: Invalid motor latency %.1fms in preferences, using default %.1fms\n",
                motor_response_latency_ms, GRIND_MOTOR_RESPONSE_LATENCY_DEFAULT_MS);
        motor_response_latency_ms = GRIND_MOTOR_RESPONSE_LATENCY_DEFAULT_MS;
    } else {
        LOG_BLE("Motor latency: Loaded %.1fms from preferences\n", motor_response_latency_ms);
    }
}

void GrindController::save_motor_latency(float value) {
    if (!preferences) {
        LOG_BLE("ERROR: Cannot save motor latency - no preferences available\n");
        return;
    }

    // Validate value
    if (value < GRIND_AUTOTUNE_LATENCY_MIN_MS || value > GRIND_AUTOTUNE_LATENCY_MAX_MS) {
        LOG_BLE("ERROR: Cannot save invalid motor latency %.1fms (range: %.1f-%.1fms)\n",
                value, GRIND_AUTOTUNE_LATENCY_MIN_MS, GRIND_AUTOTUNE_LATENCY_MAX_MS);
        return;
    }

    motor_response_latency_ms = value;
    size_t written = preferences->putFloat("motor_lat_ms", value);
    if (written == 0) {
        LOG_BLE("ERROR: Failed to save motor latency to NVS\n");
    } else {
        LOG_BLE("Motor latency: Saved %.1fms to preferences\n", value);
    }
}

void GrindController::set_motor_response_latency(float value) {
    // Validate value
    if (value < GRIND_AUTOTUNE_LATENCY_MIN_MS || value > GRIND_AUTOTUNE_LATENCY_MAX_MS) {
        LOG_BLE("ERROR: Cannot set invalid motor latency %.1fms (range: %.1f-%.1fms)\n",
                value, GRIND_AUTOTUNE_LATENCY_MIN_MS, GRIND_AUTOTUNE_LATENCY_MAX_MS);
        return;
    }

    motor_response_latency_ms = value;
    LOG_BLE("Motor latency: Set to %.1fms (not saved to NVS)\n", value);
}
