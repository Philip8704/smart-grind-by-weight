#include "weight_grind_strategy.h"

#include "grind_controller.h"
#include "../hardware/WeightSensor.h"
#include "../hardware/grinder.h"
#include "../logging/grind_logging.h"
#include "../config/constants.h"
#include <Arduino.h>

void WeightGrindStrategy::on_enter(const GrindSessionDescriptor&, GrindStrategyContext&, const GrindLoopData&) {
    // No additional setup required; controller handled initialization.
}

bool WeightGrindStrategy::update(const GrindSessionDescriptor&,
                                 GrindStrategyContext& context,
                                 const GrindLoopData& loop_data) {
    auto* controller = context.controller;
    if (!controller) {
        return false;
    }

    switch (controller->phase) {
        case GrindPhase::PREDICTIVE:
            run_predictive_phase(*controller, loop_data);
            return true;
        case GrindPhase::PULSE_DECISION:
            run_pulse_decision_phase(*controller, loop_data);
            return true;
        case GrindPhase::PULSE_EXECUTE:
            run_pulse_execute_phase(*controller, loop_data);
            return true;
        case GrindPhase::PULSE_SETTLING:
            run_pulse_settling_phase(*controller, loop_data);
            return true;
        default:
            return false;
    }
}

void WeightGrindStrategy::on_exit(const GrindSessionDescriptor&, GrindStrategyContext&) {
    // No teardown required for weight strategy yet.
}

int WeightGrindStrategy::progress_percent(const GrindSessionDescriptor&,
                                          const GrindController&) const {
    // Weight-based progress remains computed directly by the controller.
    return 0;
}

float WeightGrindStrategy::get_clamped_pulse_flow_rate(const GrindController& controller) const {
    float flow_rate = controller.pulse_flow_rate;

    if (flow_rate < GRIND_FLOW_RATE_MIN_SANE_GPS) {
        flow_rate = GRIND_PULSE_FLOW_RATE_FALLBACK_GPS;
    } else if (flow_rate > GRIND_FLOW_RATE_MAX_SANE_GPS) {
        flow_rate = GRIND_FLOW_RATE_MAX_SANE_GPS;
    }

    return flow_rate;
}

float WeightGrindStrategy::calculate_productive_pulse_ms(const GrindController& controller,
                                                         float error_grams) const {
    float clamped_flow_rate = get_clamped_pulse_flow_rate(controller);

    // Grinding time needed to close the error, excluding startup latency. The pulse
    // model's expected extra - grounds the pulse shakes loose on top of what it grinds,
    // learned from past pulses - is subtracted first, taken at the upper quartile so
    // the pulse tends to come out short. If that alone covers the error the productive
    // part is zero and the pulse is the minimum: the latency.
    float grams_to_grind = error_grams - controller.get_pulse_planning_extra_g();
    if (grams_to_grind <= 0.0f) {
        return 0.0f;
    }
    float productive_duration_ms = (grams_to_grind / clamped_flow_rate) * 1000.0f;

    return max(0.0f, min(productive_duration_ms, GRIND_MOTOR_MAX_PULSE_DURATION_MS));
}

void WeightGrindStrategy::run_predictive_phase(GrindController& controller,
                                               const GrindLoopData& loop_data) const {
    if (!controller.weight_sensor) {
        return;
    }

    if (!controller.flow_start_confirmed) {
        float current_flow_rate = controller.weight_sensor->get_flow_rate(GRIND_FLOW_DETECTION_WINDOW_MS);

        if (current_flow_rate >= GRIND_FLOW_DETECTION_THRESHOLD_GPS) {
            controller.grind_latency_ms = loop_data.now - controller.phase_start_time;
            controller.flow_start_confirmed = true;
            LOG_BLE("[PREDICTIVE] Flow start CONFIRMED! Latency: %.1fms, Flow: %.2fg/s\n",
                    controller.grind_latency_ms, current_flow_rate);
        }
    }

    if (controller.flow_start_confirmed) {
        if (loop_data.now > (controller.phase_start_time + controller.grind_latency_ms + GRIND_FLOW_RATE_CALC_WINDOW_MS)) {
            float current_flow_rate = controller.weight_sensor->get_flow_rate(GRIND_FLOW_RATE_CALC_WINDOW_MS);

            if (current_flow_rate > GRIND_FLOW_DETECTION_THRESHOLD_GPS) {
                // Coffee still in flight when the motor stops = coast time x flow rate.
                //
                // The coast time is an upper quantile of what this profile has actually
                // measured, not an average of it, because the two ways of being wrong
                // do not cost the same: predicting too little lands the grind heavy and
                // nothing can undo that, predicting too much lands it light and a pulse
                // fixes it. Only the very first grind on a profile falls back to
                // guessing from the spin-up latency, which measures chute fill rather
                // than burr spin-down and is a seed rather than a measurement.
                float predicted_coast_s = controller.get_coast_prediction_s();
                if (predicted_coast_s <= 0.0f) {
                    predicted_coast_s = (controller.grind_latency_ms * GRIND_LATENCY_TO_COAST_RATIO) /
                                        (float)SYS_MS_PER_SECOND +
                                        GRIND_COAST_TAIL_PAD_S;
                }
                controller.motor_stop_target_weight = predicted_coast_s * current_flow_rate;

                // Held so the observation that follows can be logged against what was
                // actually predicted. Recomputing it later would read a window that the
                // previous grind may already have moved.
                controller.coast_predicted_s_ = predicted_coast_s;

                // Remember which flow figure this prediction was built on. The coast
                // observation has to divide by the same one, or the units do not
                // cancel: dividing the observed coast weight by a different (higher)
                // flow yields a smaller coast time, which then gets multiplied back by
                // this lower one, and the model reads permanently short.
                controller.coast_prediction_flow_gps = current_flow_rate;
            }
        }
    }

    // Only allow motor stop decision after motor has settled to avoid startup transients
    if (controller.grinder->is_motor_settled() &&
        loop_data.current_weight >= (controller.target_weight - controller.motor_stop_target_weight)) {
        controller.grinder->stop();
        controller.predictive_end_weight = loop_data.current_weight;
        controller.pulse_flow_rate = controller.weight_sensor->get_flow_rate_95th_percentile(GRIND_PULSE_FLOW_RATE_WINDOW_MS);
        controller.mark_coast_window_start();
        controller.switch_phase(GrindPhase::PULSE_SETTLING, loop_data);
    }
}

void WeightGrindStrategy::run_pulse_decision_phase(GrindController& controller,
                                                   const GrindLoopData& loop_data) const {
    if (!controller.weight_sensor) {
        return;
    }

    // Wait for the reading to actually stop rising before deciding whether to pulse.
    // Reading mid-creep under-reports the weight and can fire an unnecessary pulse that
    // then overshoots. The backstop keeps a slow-trickling grinder from stalling here:
    // after the timeout, accept a variance-settled reading like the old behaviour.
    float settled_weight;
    bool stable = controller.weight_sensor->check_settling_complete(
        GRIND_SCALE_PRECISION_SETTLING_TIME_MS, &settled_weight, GRIND_SETTLING_DRIFT_MAX_GPS);
    if (!stable) {
        bool timed_out = (loop_data.now - controller.phase_start_time) >= GRIND_SETTLING_STABLE_TIMEOUT_MS;
        if (!(timed_out && controller.weight_sensor->check_settling_complete(
                               GRIND_SCALE_PRECISION_SETTLING_TIME_MS, &settled_weight))) {
            return;
        }
    }

    // The first settle after the predictive stop is the only clean look at coast:
    // everything delivered between motor-off and this reading was already in flight.
    // Later settles follow pulses, which have their own latency and are not coast.
    if (controller.pulse_attempts == 0) {
        controller.observe_coast(settled_weight - controller.predictive_end_weight);
    } else {
        // Every later settle follows a pulse: what that pulse actually delivered is what
        // the pulse model learns from
        controller.observe_pulse(controller.pulse_attempts - 1, settled_weight);
    }

    float conservative_target = controller.target_weight - GRIND_ACCURACY_TOLERANCE_G;
    float error = conservative_target - settled_weight;

    // coast_time_ms removed - was only used for logging pulse history

    // Stop pulsing when the remaining error is smaller than the smallest correction
    // worth making. Firing anyway would waste an attempt on pure motor latency that
    // delivers nothing measurable.
    //
    // The threshold is a weight, not a pulse duration. A fixed minimum duration means
    // a flow-dependent cutoff - at 3g/s a 20ms floor gives up while 0.06g is still
    // owed, twice the tolerance - whereas this bounds the worst-case undershoot at
    // tolerance + GRIND_PULSE_MIN_DELIVERY_G no matter how fast the grinder runs.
    if (controller.target_weight - settled_weight < GRIND_ACCURACY_TOLERANCE_G ||
        controller.pulse_attempts >= GRIND_MAX_PULSE_ATTEMPTS ||
        error < GRIND_PULSE_MIN_DELIVERY_G) {
        controller.switch_phase(GrindPhase::FINAL_SETTLING, loop_data);
        return;
    }

    // The shortest pulse allowed is the motor latency learned by Tune Pulses, and the
    // model expects even that one to bring `extra` grams. If that alone would carry the
    // grind past target + tolerance, finish where it is: a pulse cannot be made smaller,
    // and an undershoot can be topped up by hand where an overshoot cannot be undone.
    const float extra = controller.get_pulse_expected_extra_g();
    if (settled_weight + extra > controller.target_weight + GRIND_ACCURACY_TOLERANCE_G) {
        controller.queue_log_message("[PULSE] Shortest pulse (%.0fms) expected to add %.3fg - would pass %.2fg, finishing at %.2fg\n",
                                     controller.get_motor_response_latency(), extra,
                                     controller.target_weight + GRIND_ACCURACY_TOLERANCE_G, settled_weight);
        controller.switch_phase(GrindPhase::FINAL_SETTLING, loop_data);
        return;
    }

    PulseReport& pulse = controller.pulse_history[controller.pulse_attempts];
    const float flow = get_clamped_pulse_flow_rate(controller);
    const float productive_ms = calculate_productive_pulse_ms(controller, error);
    pulse.start_weight = settled_weight;
    pulse.end_weight = settled_weight;
    pulse.flow_gps = flow;
    pulse.expected_g = extra + flow * productive_ms / 1000.0f;

    // Never below the learned latency: productive_ms is >= 0, so the pulse is at least
    // it. Whole milliseconds are what get sent, so that is what the observation of this
    // pulse is computed from - the same figure on both sides.
    const uint32_t sent_ms = static_cast<uint32_t>(controller.get_motor_response_latency() + productive_ms);
    controller.current_pulse_duration_ms = (float)sent_ms;
    pulse.duration_ms = (float)sent_ms;
    controller.pulse_anomaly_at_start_ = controller.mechanical_anomaly_count_;

    controller.queue_log_message("[PULSE] #%d plan: need %.3fg -> %lums (latency %.0f + %.0f), expects %+.3fg (extra %.3f, sized %.3f)\n",
                                 controller.pulse_attempts + 1, error, (unsigned long)sent_ms,
                                 controller.get_motor_response_latency(), productive_ms,
                                 pulse.expected_g, extra, controller.get_pulse_planning_extra_g());

    controller.switch_phase(GrindPhase::PULSE_EXECUTE, loop_data);
    controller.grinder->start_pulse_rmt(sent_ms);

    controller.pulse_attempts++;
}

void WeightGrindStrategy::run_pulse_execute_phase(GrindController& controller,
                                                  const GrindLoopData& loop_data) const {
    if (controller.grinder && controller.grinder->is_pulse_complete()) {
        controller.switch_phase(GrindPhase::PULSE_SETTLING, loop_data);
    }
}

void WeightGrindStrategy::run_pulse_settling_phase(GrindController& controller,
                                                   const GrindLoopData& loop_data) const {
    if (!controller.weight_sensor) {
        return;
    }

    if (loop_data.now - controller.phase_start_time >= controller.grind_latency_ms + GRIND_MOTOR_SETTLING_TIME_MS) {
        if (controller.weight_sensor->check_settling_complete(GRIND_MOTOR_SETTLING_TIME_MS)) {
            controller.switch_phase(GrindPhase::PULSE_DECISION, loop_data);
        }
    }
}
