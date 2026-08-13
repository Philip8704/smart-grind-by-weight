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
#define GRIND_MIN_TARGET_FOR_DELIVERY_CHECK_G 1.0f                        // Only targets at or above this are checked for "no coffee delivered"

//------------------------------------------------------------------------------
// LEARNED COAST MODEL (per profile, persisted)
//------------------------------------------------------------------------------
// Coast = coffee still in flight after the motor stops. Measured every grind as
// (settled weight - weight at motor stop) / flow rate, then averaged across grinds.
// Storing it as a TIME rather than a weight keeps it valid across dose sizes and
// grind settings, since the weight it turns into scales with the current flow rate.
// Deliberate bias toward stopping early. The predicted in-flight coffee is inflated by
// this factor, so the motor cuts sooner than the model strictly says and the grind
// lands a little light - which the correction pulses then top up.
//
// The asymmetry is the whole point: coffee still in the chute can be added by a pulse,
// coffee already in the cup cannot be taken back. Aiming dead-on means overshooting
// roughly half the time coast runs above its average. Above 1.0 = stop earlier =
// undershoot; below 1.0 would stop later and overshoot, which is what we are avoiding.
#define GRIND_COAST_SAFETY_FACTOR 1.15f                                   // Predict 15% more in flight than measured

// EWMA weight of the newest observation. Effective memory is roughly 1/alpha, so this
// averages over about 8 grinds. Longer than it needs to be for a stable machine, but
// observations now include overshoots and undershoots as well as clean grinds, and a
// wider average keeps one unusual dose from moving the model much. The cost is that a
// genuine change - new beans, a grind setting adjustment - takes about 8 grinds to
// track rather than 4.
#define GRIND_COAST_LEARNING_ALPHA 0.125f
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
