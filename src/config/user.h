#pragma once

//==============================================================================
// USER CONFIGURATION PARAMETERS
//==============================================================================
// This file contains user-configurable parameters that affect coffee grinding
// behavior, UI responsiveness, and device operation. These are the primary
// settings that users might want to modify to customize their grinder.

//------------------------------------------------------------------------------
// COFFEE PROFILES
//------------------------------------------------------------------------------
#define USER_PROFILE_COUNT 3                                                   // Number of coffee profiles available
#define USER_PROFILE_NAME_MAX_LENGTH 8                                         // Maximum characters in profile name

// Default target weights for each profile
#define USER_SINGLE_ESPRESSO_WEIGHT_G 9.0f                                     // Single espresso default weight
#define USER_DOUBLE_ESPRESSO_WEIGHT_G 18.0f                                    // Double espresso default weight  
#define USER_CUSTOM_PROFILE_WEIGHT_G 21.5f                                     // Custom profile default weight

#define USER_SINGLE_ESPRESSO_TIME_S 5.0f                                       // Single espresso default grind time
#define USER_DOUBLE_ESPRESSO_TIME_S 10.0f                                      // Double espresso default grind time
#define USER_CUSTOM_PROFILE_TIME_S 12.0f                                       // Custom profile default grind time

// Weight limits
#define USER_MIN_TARGET_WEIGHT_G 5.0f                                          // Minimum allowed target weight
#define USER_MAX_TARGET_WEIGHT_G 1000.0f                                        // Maximum allowed target weight

#define USER_MIN_TARGET_TIME_S 0.5f                                            // Minimum allowed target time
#define USER_MAX_TARGET_TIME_S 25.0f                                           // Maximum allowed target time

//------------------------------------------------------------------------------
// WEIGHT/TIME ADJUSTMENTS
//------------------------------------------------------------------------------
#define USER_FINE_WEIGHT_ADJUSTMENT_G 0.1f                                     // Small weight increment for fine tuning
#define USER_FINE_TIME_ADJUSTMENT_S 0.1f                                       // Fine adjustment step for time editing

// USER_JOG parameters moved to system.h to be near SYS_JOG parameters

//------------------------------------------------------------------------------
// SCALE CALIBRATION
//------------------------------------------------------------------------------
#define USER_CALIBRATION_REFERENCE_WEIGHT_G 100.0f                             // Default reference weight for calibration
#define USER_DEFAULT_CALIBRATION_FACTOR -7050.0f                               // Default load cell calibration factor

//------------------------------------------------------------------------------
// SCREEN AUTO-DIMMING
//------------------------------------------------------------------------------
#define USER_SCREEN_AUTO_DIM_TIMEOUT_MS 300000                                 // Time before screen dims due to inactivity
#define USER_SCREEN_BRIGHTNESS_NORMAL 1.0f                                     // Normal screen brightness
#define USER_SCREEN_BRIGHTNESS_DIMMED 0.35f                                    // Dimmed screen brightness
#define USER_WEIGHT_ACTIVITY_THRESHOLD_G 1.0f                                  // Weight change threshold for screen timeout reset (grams)

//------------------------------------------------------------------------------
// AUTO ACTIONS
//------------------------------------------------------------------------------
#define USER_AUTO_GRIND_TRIGGER_SETTLING_MS 1000                                // Scale must stay settled this long before auto-start fires
#define USER_AUTO_GRIND_REARM_DELAY_MS 1500                                     // Minimum delay between auto actions (milliseconds)

// Minimum weight that must be sitting on the scale before auto-start will tare and grind.
// Set this to just under the portafilter weight so lighter objects - a dosing funnel, a
// cup, a hand resting on the scale - produce the trigger delta but never start a grind.
// The threshold IS the auto-start trigger: the grind begins once the scale comes to
// rest above it. That way you can seat the portafilter, work the slider, and let it
// fire when everything stops moving - no sudden placement needed, and nothing lighter
// than the portafilter ever reaches the threshold. Left at 0 there is no trigger at
// all and grinding only starts from the on-screen button.
#define USER_AUTO_GRIND_MIN_WEIGHT_DEFAULT_G 0                                  // 0 = auto-start disabled, manual button only
#define USER_AUTO_GRIND_MIN_WEIGHT_STEP_G 100                                   // Dropdown granularity
#define USER_AUTO_GRIND_MIN_WEIGHT_MAX_G 1000                                   // Highest selectable threshold

// Re-arming: after a grind the portafilter is still on the scale, settled and heavy, so
// the trigger stays disarmed until the scale is clearly unloaded again. Requiring both a
// large drop and a dwell time stops a lift-and-replace or a knock from re-arming it.
#define USER_AUTO_GRIND_REARM_DROP_G 100.0f                                     // Weight must fall this far below the threshold
#define USER_AUTO_GRIND_REARM_DWELL_MS 2000                                     // ...and stay there this long

// Grinding tares with the portafilter already sitting on the cradle, so once it is
// lifted the empty cradle reads roughly minus one portafilter. Left alone, no
// portafilter would ever reach the minimum weight again and auto-start would work
// exactly once. When the empty cradle settles this far below zero the zero point is
// stale, and it is re-taken while nothing is on the scale.
#define USER_AUTO_GRIND_REZERO_BELOW_G 50.0f
