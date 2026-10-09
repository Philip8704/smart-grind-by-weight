#pragma once

//==============================================================================
// GRIND CONTROL CONFIGURATION
//==============================================================================
// This file contains all grind control related configuration constants
// including timing parameters, accuracy settings, flow detection, and
// pulse control algorithms.

//------------------------------------------------------------------------------
// GRINDER PURGE/PRIME
//------------------------------------------------------------------------------
// Grinder saturation modes
enum class GrinderPurgeMode {
    PRIME = 0,  // Saturate grinder, keep coffee, continue immediately
    PURGE = 1   // Saturate grinder, prompt user to discard stale grinds
};

// Grinder saturation defaults and ranges
#define GRIND_PURGE_MODE_DEFAULT static_cast<int>(GrinderPurgeMode::PURGE)
#define GRIND_PURGE_AMOUNT_DEFAULT_G 1.0f
#define GRIND_PURGE_AMOUNT_MIN_G 0.1f
#define GRIND_PURGE_AMOUNT_MAX_G 2.5f

// Grind freshness tracking
#define GRIND_FRESHNESS_DEFAULT_HOURS 8.0f

// After confirming a purge, the settled reading is compared with what the purge
// actually delivered. Still about the purge amount means the grounds were kept, so the
// grind carries on counting them as now. Far from it means they were discarded - the
// basket is empty and has been lifted and re-seated - so the scale is re-zeroed rather
// than letting the seating difference land in the dose.
#define GRIND_PURGE_RETARE_THRESHOLD_G 0.5f
#define GRIND_PURGE_CHECK_STEADY_WINDOW_MS 500                        // Reading must hold still this long...
#define GRIND_PURGE_CHECK_STEADY_RANGE_G 0.2f                         // ...within this peak-to-peak band. At 0.03g noise five samples span ~0.12g, so 0.1 would rarely pass; still well clear of the 0.5g decision
#define GRIND_PURGE_CHECK_TIMEOUT_MS 3000                             // Decide on the best available reading after this
#define GRIND_PURGE_PORTAFILTER_MISSING_G -20.0f                      // Below this the portafilter is off the scale - wait, never tare

//------------------------------------------------------------------------------
// VIBRATION TEST
//------------------------------------------------------------------------------
// Runs the motor with an empty hopper and the portafilter in place, recording raw
// samples, to measure how much vibration the load cell picks up with nothing landing.
// Grinds only offer the few samples between motor start and the first grounds; this
// gives dozens in one run. The quiet periods either side give the load-cell floor.
#define VIBRATION_TEST_BASELINE_MS 3000                               // Motor off, scale loaded, before the run
#define VIBRATION_TEST_RUN_MS 5000                                    // Motor on
#define VIBRATION_TEST_TRANSIENT_MS 500                               // Ignored after each motor start/stop
#define VIBRATION_TEST_GROUNDS_DETECT_G 0.3f                          // Rise during the run that means the hopper was not empty

//------------------------------------------------------------------------------
// CHUTE KNOCK TEST
//------------------------------------------------------------------------------
// Grounds retained in the chute trickle out late, which varies coast and slows
// settling. This test fires short motor pulses after a grind - hopper EMPTY, so
// nothing new can be ground - and weighs what falls, to find out whether start-up
// jolts shake retained grounds loose and which pulse length the grinder responds to.
// The ESP only drives the Eureka's control input, and its board may ignore or stretch
// short pulses, so on-times are swept rather than assumed.
#define KNOCK_TEST_STEPS 4
#define KNOCK_TEST_ON_MS_LIST {20, 40, 60, 80}                        // One train per entry, shortest first
#define KNOCK_TEST_OFF_MS 250                                         // Long enough for the rotor to slow, so each start is a fresh jolt
#define KNOCK_TEST_CYCLES 8                                           // Pulses per train
#define KNOCK_TEST_BASELINE_MS 3000                                   // Quiet after the tare, before the first train
#define KNOCK_TEST_DROP_WAIT_MS 2500                                  // After each train, for released grounds to land
#define KNOCK_TEST_MEASURE_WINDOW_MS 1000                             // Weight is an outlier-rejected average over this, at the end of each quiet period
#define KNOCK_TEST_RESPONSE_RATIO 1.5f                                // Scale noise during a train this many times the floor = motor reacted
#define KNOCK_TEST_GRINDING_SUSPECT_G 1.0f                            // More than this in total means beans were ground, not grounds released

//------------------------------------------------------------------------------
// GRIND CONTROL TUNING
//------------------------------------------------------------------------------
// Main accuracy and timeout settings
#define GRIND_ACCURACY_TOLERANCE_G 0.03f                                  // Final target accuracy tolerance
#define GRIND_TIMEOUT_SEC 60                                              // Maximum time for grind operation
#define GRIND_MAX_PULSE_ATTEMPTS 10                                       // Maximum pulse corrections before stopping

// Flow rate detection
#define GRIND_FLOW_DETECTION_THRESHOLD_GPS 0.5f                           // Minimum coffee flow rate to establish first grinds reachinig the cup = latency

// Undershoot strategy - determine when to stop grinding during the predictive phase
#define GRIND_UNDERSHOOT_TARGET_G 1.0f                                    // Default conservative undershoot target
#define GRIND_LATENCY_TO_COAST_RATIO 1.0f                                 // Seed only: coast time guessed from spin-up latency until a real coast time is learned

// Flow measurement windows used by the predictive phase
#define GRIND_FLOW_DETECTION_WINDOW_MS 500                                // Window for detecting that coffee has started falling
#define GRIND_FLOW_RATE_CALC_WINDOW_MS 1500                               // Window for the steady-state flow rate driving the stop prediction
#define GRIND_PULSE_FLOW_RATE_WINDOW_MS 2500                              // Window for the 95th-percentile flow rate captured at motor stop

// Failsafe thresholds
#define GRIND_NEGATIVE_WEIGHT_FAILSAFE_G -1.0f                            // Weight below this during a weight grind means the cup moved or the scale broke
#define GRIND_NEGATIVE_WEIGHT_CONFIRM_SAMPLES 4                            // Consecutive raw samples below it before the grind is stopped - one corrupted HX711 read must not abort a grind
#define GRIND_MIN_TARGET_FOR_DELIVERY_CHECK_G 1.0f                        // Only targets at or above this are checked for "no coffee delivered"

//------------------------------------------------------------------------------
// LEARNED COAST MODEL (per profile, persisted)
//------------------------------------------------------------------------------
// Coast = coffee still in flight after the motor stops. Measured every grind as
// (settled weight - weight at motor stop) / flow rate. Storing it as a TIME rather than
// a weight keeps it valid across dose sizes and grind settings, since the weight it
// turns into scales with the current flow rate.
//
// The model keeps the last few measurements and predicts with a HIGH QUANTILE of them
// rather than with an average. That choice follows directly from the cost being
// one-sided: coffee still in the chute can be topped up by a pulse, coffee already in
// the cup cannot be taken back. An average is the right answer when overshooting and
// undershooting cost the same, and the wrong one here - it is beaten by roughly half
// of all grinds by construction. The quantile that minimises a one-sided cost is the
// upper one, so that is what gets predicted.
//
// Why a window of real measurements instead of a running average plus a safety factor:
// a running average has to be nudged after every grind, and any scheme that nudges it
// harder in one direction than the other ratchets. The visible symptom is hunting -
// creep down over several grinds, one overshoot, a large correction down, creep back
// up - which never settles. A window cannot do that. The predicted value is always one
// of the measurements actually taken, so it can only ever move to another value the
// grinder has really produced, and it can never wander outside the range of observed
// behaviour no matter how the errors fall.
//
// The rank is counted from the top of the window: 1 = highest of the window, which
// estimates the N/(N+1) quantile (about the 89th percentile at N=8), 2 = second
// highest (about the 78th), which is immune to a single freak measurement at the cost
// of predicting lower. Rank 1 is the default because the failure it risks is an extra
// correction pulse, and the failure the alternative risks is an overshoot.
#define GRIND_COAST_WINDOW_SIZE 8                                         // Measurements kept per profile
#define GRIND_COAST_RANK_FROM_TOP 1                                       // Which order statistic to predict with (1 = highest)

// A window of eight can only resolve a quantile as fine as one part in nine, so it
// cannot see the tail beyond about the 89th percentile on its own. This pad reaches a
// little past what the samples can show, and also covers the very first grinds when
// the window holds one or two entries and says almost nothing about spread. At a
// typical 1.2 g/s it is worth well under a tenth of a gram - about one short pulse.
#define GRIND_COAST_TAIL_PAD_S 0.02f

#define GRIND_COAST_TIME_MIN_S 0.05f                                      // Reject observations below this as measurement noise
#define GRIND_COAST_TIME_MAX_S 1.50f                                      // Reject observations above this as a stalled or mis-settled grind

// Prime phase behavior
#define GRIND_PRIME_TARGET_WEIGHT_G 1.0f                                   // Amount of coffee delivered during chute priming
#define GRIND_PRIME_MAX_DURATION_MS 5000                                   // Safety timeout for chute priming run

//------------------------------------------------------------------------------
// SCALE CALIBRATION AND SETTLING
//------------------------------------------------------------------------------
// Tare and settling behavior  
#define GRIND_SCALE_SETTLING_TOLERANCE_G 0.010f                           // Maximum standard deviation for settled reading. Used to determine if scale is settled. Increase value if you have a noisy load cell.

// The std-dev test above measures how NOISY the reading is, not whether it has stopped
// RISING. A slow steady trickle of grounds into the cup has low variance but is still
// climbing - up to about 0.07 g/s reads as "settled" - so the weight can be measured
// mid-creep, then finish higher. That either mis-reports the final weight or provokes
// one extra correction pulse, and shows up as a grind landing 0.1-0.2g heavy at random.
// The grind measurement paths additionally require the reading to be climbing slower
// than this, using the least-squares slope. The backstop timeout below stops a grinder
// that trickles for a long time from stalling the grind.
#define GRIND_SETTLING_DRIFT_MAX_GPS 0.03f                                // Reading must be changing slower than this to count as settled
#define GRIND_SETTLING_STABLE_TIMEOUT_MS 3000                             // After this, accept a variance-settled reading even if still drifting

//------------------------------------------------------------------------------
// TIME MODE PULSE SETTINGS
//------------------------------------------------------------------------------
#define GRIND_TIME_PULSE_DURATION_MS 100                                        // Duration of additional pulses in time mode (milliseconds)

//------------------------------------------------------------------------------
// TIME MODE SCALE HANDLING
//------------------------------------------------------------------------------
// Time mode never depends on the scale: the load cell only feeds the display.
// These bounds keep a slow or dead scale from stalling a purely time-based grind.
#define GRIND_TIME_TARE_TIMEOUT_MS 4000                                         // Give up on taring and grind anyway (18 samples @ ~11.5 SPS + settling)
#define GRIND_TIME_SETTLING_TIMEOUT_MS 2000                                     // Max wait for the final weight reading before reporting whatever is shown
#define GRIND_NOTICE_DISPLAY_MS 3000                                            // How long a non-fatal notice ("Tare failed") replaces the target label



//------------------------------------------------------------------------------
// FLOW RATE PARAMETERS
//------------------------------------------------------------------------------
#define GRIND_FLOW_RATE_MIN_SANE_GPS 1.0f                                         // Minimum reasonable flow rate
#define GRIND_FLOW_RATE_MAX_SANE_GPS 3.0f                                         // Maximum reasonable flow rate
#define GRIND_PULSE_FLOW_RATE_FALLBACK_GPS 1.5f                                   // Fallback pulse flow rate when measured rate is invalid or too low

//------------------------------------------------------------------------------
// TIMING CONSTRAINTS (Hardware-dependent)
//------------------------------------------------------------------------------
// Motor response latency - runtime configurable via auto-tune
#define GRIND_MOTOR_RESPONSE_LATENCY_DEFAULT_MS 50.0f                             // Safe default motor response latency
#define GRIND_MOTOR_MAX_PULSE_DURATION_MS 250.0f                                  // Maximum pulse duration above latency (latency + GRIND_MOTOR_MAX_PULSE_DURATION_MS)
#define GRIND_PULSE_MIN_DELIVERY_G 0.02f                                          // Smallest correction worth firing a pulse for. Below this the pulse is mostly
                                                                                  // motor latency and delivers nothing measurable, so the grind is declared done
                                                                                  // instead of burning attempts. Expressed as a weight rather than a duration so
                                                                                  // the worst-case undershoot stays at tolerance + this, whatever the flow rate.

// Pulse delivery model. A correction pulse delivers more than its grinding time
// (length past the motor latency) times the flow rate: the start-up jolt also shakes
// retained grounds out of the chute. Measured on this grinder, pulses delivered
// 0.04-0.10g beyond that, and a pulse planned from flow alone overshoots.
//
// So each pulse is planned as   length = latency + max(0, needed - extra) / flow
// where `extra` comes from what recent pulses delivered beyond their grinding time.
// Three rules keep it on the safe side:
//  - `needed` is measured to (target - tolerance), as before, and pulses are sized from
//    the UPPER QUARTILE of the measured extra rather than the median, so most pulses
//    come out a little short. Two small corrections are preferred to one too large.
//  - `extra` is never taken below zero, so the model can only shorten a pulse relative
//    to the flow-only plan, never lengthen it.
//  - No pulse is ever shorter than the learned motor latency from Tune Pulses. If even
//    that shortest pulse is expected (median extra) to carry the grind past target +
//    tolerance, the grind finishes where it is rather than overshoot. The median, not
//    the upper quartile, so this does not stop grinds short more often than needed.
#define GRIND_PULSE_WINDOW_SIZE 8                                                 // Pulse observations kept (one window for the grinder)
#define GRIND_PULSE_PLANNING_QUANTILE 0.75f                                       // Extra assumed when sizing a pulse: upper quartile of the window
#define GRIND_PULSE_EXTRA_SEED_G 0.06f                                            // Used until a pulse has been measured: the median measured on this grinder
#define GRIND_PULSE_EXTRA_VALID_MIN_G -0.5f                                       // Observations outside this are measurement faults, not pulses
#define GRIND_PULSE_EXTRA_VALID_MAX_G 1.0f
#define GRIND_PULSE_DELIVERED_MIN_G -0.05f                                        // A pulse cannot remove coffee - more negative means the scale was disturbed
#define GRIND_PULSE_DELIVERED_MAX_G 1.0f                                          // More than this from one pulse means something else landed on the scale

// Motor timing
#define GRIND_MOTOR_SETTLING_TIME_MS 200                                          // Motor vibration settling time

// Mechanical instability detection
#define GRIND_MECHANICAL_DROP_THRESHOLD_G 0.4f                                    // Weight drop considered mechanical instability
#define GRIND_MECHANICAL_EVENT_COOLDOWN_MS 200                                    // Minimum time between detecting drops
#define GRIND_MECHANICAL_EVENT_REQUIRED_COUNT 3                                   // Events required to flag diagnostic

// Scale settling timing
#define GRIND_SCALE_PRECISION_SETTLING_TIME_MS 500                                // High-precision settling time
#define GRIND_SCALE_SETTLING_TIMEOUT_MS 10000                                     // Maximum time to wait for settling

// Tare and calibration timing (hardware sample rate dependent)
#define GRIND_TARE_SAMPLE_WINDOW_MS 500                                           // Time window for tare sampling
#define GRIND_TARE_TIMEOUT_MS 3000                                                // Maximum tare completion time
#define GRIND_CALIBRATION_SAMPLE_WINDOW_MS 800                                    // Time window for calibration sampling  
#define GRIND_CALIBRATION_TIMEOUT_MS 2000                                         // Maximum calibration completion time

// Calculated sample counts based on hardware rate
#define GRIND_TARE_SAMPLE_COUNT (GRIND_TARE_SAMPLE_WINDOW_MS / HW_LOADCELL_SAMPLE_INTERVAL_MS)
#define GRIND_CALIBRATION_SAMPLE_COUNT (GRIND_CALIBRATION_SAMPLE_WINDOW_MS / HW_LOADCELL_SAMPLE_INTERVAL_MS)

//------------------------------------------------------------------------------
// MOTOR RESPONSE AUTO-TUNE ALGORITHM
//------------------------------------------------------------------------------
#define GRIND_AUTOTUNE_LATENCY_MIN_MS 30.0f                                       // Lower search bound for latency
#define GRIND_AUTOTUNE_LATENCY_MAX_MS 300.0f                                      // Upper search bound for latency
#define GRIND_AUTOTUNE_PRIMING_PULSE_MS 1000                                      // Initial chute priming pulse
#define GRIND_AUTOTUNE_TARGET_ACCURACY_MS 5.0f                                    // Target resolution
#define GRIND_AUTOTUNE_SUCCESS_RATE 0.80f                                         // 80% success threshold (4/5 pulses)
#define GRIND_AUTOTUNE_VERIFICATION_PULSES 5                                      // Verification attempts per candidate
#define GRIND_AUTOTUNE_MAX_ITERATIONS 50                                          // Hard stop safety limit
#define GRIND_AUTOTUNE_COLLECTION_DELAY_MS 1500                                   // Minimum wait after pulse for grounds to drop
#define GRIND_AUTOTUNE_SETTLING_TIMEOUT_MS 5000                                   // Max wait per pulse for scale settling
#define GRIND_AUTOTUNE_WEIGHT_THRESHOLD_G GRIND_SCALE_SETTLING_TOLERANCE_G        // 0.010g detection threshold
