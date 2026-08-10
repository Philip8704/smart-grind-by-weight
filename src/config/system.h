#pragma once

//==============================================================================
// SYSTEM CONFIGURATION CONSTANTS
//==============================================================================
// This file contains core system configuration values that control the
// operation of the ESP32 coffee grinder's internal algorithms and timing.
// These parameters affect system behavior, task scheduling, and algorithmic
// calculations.
//
// Categories:
// - Task Scheduling Intervals (RTOS task update frequencies)
// - Algorithm Parameters (error thresholds, calculation factors)
// - System Timeouts (operation limits, safety timeouts)
// - Display Processing (filtering, formatting, update rates)
// - Logging and Debug Configuration (output intervals, buffer sizes)
//
// All constants use the SYS_ prefix to indicate system-level parameters.

// Logging system is included separately where needed

//------------------------------------------------------------------------------
// FREERTOS TASK CONFIGURATION
//------------------------------------------------------------------------------
// Critical timing for FreeRTOS task architecture with 6 specialized tasks

// Task Intervals (milliseconds)
#define SYS_TASK_WEIGHT_SAMPLING_INTERVAL_MS 20                                // Weight sampling poll interval (50Hz poll; HX711 @10SPS) - Core 0
#define SYS_TASK_GRIND_CONTROL_INTERVAL_MS 20                                  // Grind controller update interval (50Hz) - Core 0
#define SYS_TASK_UI_INTERVAL_MS 16                                             // UI rendering frequency (60Hz) - Core 1  
#define SYS_TASK_BLUETOOTH_INTERVAL_MS 20                                      // Bluetooth handling frequency (50Hz) - Core 1
#define SYS_TASK_FILE_IO_INTERVAL_MS 100                                       // File I/O operations frequency (10Hz) - Core 1

// Task Stack Sizes (bytes) - Increased for BLE_LOG overhead and complex operations
#define SYS_TASK_WEIGHT_SAMPLING_STACK_SIZE 4096                               // 4KB stack for weight sampling (was 2KB, increased for BLE_LOG)
#define SYS_TASK_GRIND_CONTROL_STACK_SIZE 6144                                 // 6KB stack for grind control logic (was 4KB, increased for complex algorithms)
#define SYS_TASK_UI_STACK_SIZE 8192                                            // 8KB stack for LVGL rendering (unchanged)
#define SYS_TASK_BLUETOOTH_STACK_SIZE 4096                                     // 4KB stack for BLE operations (unchanged)
#define SYS_TASK_FILE_IO_STACK_SIZE 6144                                       // 6KB stack for LittleFS operations (was 4KB, increased for file operations)

// Task Priorities (higher number = higher priority)
#define SYS_TASK_PRIORITY_WEIGHT_SAMPLING 4                                    // Highest priority (real-time sampling)
#define SYS_TASK_PRIORITY_GRIND_CONTROL 3                                      // High priority (grind control)
#define SYS_TASK_PRIORITY_UI 2                                                 // Medium priority (UI updates)
// Raise BLE above UI to prevent starvation during transfers
#define SYS_TASK_PRIORITY_BLUETOOTH 3                                          // Higher priority (BLE operations)
#define SYS_TASK_PRIORITY_FILE_IO 1                                            // Low priority (file operations)

// Inter-Task Communication Queue Sizes
#define SYS_QUEUE_UI_TO_GRIND_SIZE 5                                           // UI events to grind controller
#define SYS_QUEUE_FILE_IO_SIZE 20                                              // File I/O operation requests

// Legacy task scheduler intervals (deprecated - kept for compatibility)
#define SYS_TASK_LOADCELL_INTERVAL_MS 20                                       // Load cell polling frequency (50Hz)


//------------------------------------------------------------------------------
// TIME CONVERSION CONSTANTS
//------------------------------------------------------------------------------
#define SYS_MS_PER_SECOND 1000                                                 // Milliseconds per second conversion


//------------------------------------------------------------------------------
// WEIGHT DISPLAY FILTER SETTINGS
//------------------------------------------------------------------------------
// Asymmetric display filter for smooth weight updates. The filter now advances once
// per load cell sample (~10/s), so alpha is a true per-sample weight on the newest
// reading: 0.3 gives a ~3 sample (~300ms) decay. Raising it back towards 1.0 makes
// downward steps instant again and brings the visible bouncing back.
#define SYS_DISPLAY_FILTER_ALPHA_DOWN 0.3f                                     // Decay rate when the weight decreases
#define SYS_DISPLAY_DEADBAND_G 0.01f                                           // Reading must move at least this much before the display updates

// Window for the least-squares estimator behind the control-path weight and the flow
// rate. Wider = quieter but needs more samples before it engages; at 10 SPS this holds
// ~3 samples, the minimum for a meaningful fit. The fit is reported at the newest
// sample, so widening this reduces noise without adding lag.
#define SYS_CONTROL_FIT_WINDOW_MS 300

//------------------------------------------------------------------------------
// MEMORY HEALTH MONITORING
//------------------------------------------------------------------------------
// LVGL lives in a PSRAM pool, but FreeRTOS, BLE and LittleFS all compete for the much
// smaller internal DRAM heap - that is the one that runs out. With LV_USE_ASSERT_MALLOC
// enabled a failed allocation aborts, so the point of these thresholds is to raise the
// warning icon while there is still enough headroom to see what caused it.
#define SYS_MEMORY_CHECK_INTERVAL_MS 5000                                      // How often to sample the heaps
#define SYS_MEMORY_INTERNAL_FREE_WARN_BYTES (40 * 1024)                        // Warn below this much free internal DRAM
#define SYS_MEMORY_INTERNAL_BLOCK_WARN_BYTES (16 * 1024)                       // Warn when fragmentation leaves no block this large
#define SYS_MEMORY_RECOVERY_HYSTERESIS_BYTES (8 * 1024)                        // Must recover this far above the threshold to clear

//------------------------------------------------------------------------------
// JOG ACCELERATION CONFIGURATION
//------------------------------------------------------------------------------
// Button jog/hold acceleration thresholds (when to transition between stages)
#define USER_JOG_STAGE_1_INTERVAL_MS 100                                       // Stage 1: slow jog interval
#define USER_JOG_STAGE_2_THRESHOLD_MS 2000                                     // Time to reach stage 2 acceleration
#define USER_JOG_STAGE_3_THRESHOLD_MS 4000                                     // Time to reach stage 3 acceleration
#define USER_JOG_STAGE_4_THRESHOLD_MS 6000                                     // Time to reach stage 4 acceleration

// JOG acceleration multipliers (using multipliers to overcome LVGL timer limits)
#define SYS_JOG_STAGE_1_MULTIPLIER 1                                           // Stage 1: single increment per timer tick
#define SYS_JOG_STAGE_2_INTERVAL_MS 64                                         // Stage 2: LVGL minimum timer interval
#define SYS_JOG_STAGE_2_MULTIPLIER 3                                           // Stage 2: 3 increments per 64ms = ~4.7g/s
#define SYS_JOG_STAGE_3_INTERVAL_MS 64                                         // Stage 3: LVGL minimum timer interval  
#define SYS_JOG_STAGE_3_MULTIPLIER 6                                           // Stage 3: 6 increments per 64ms = ~9.4g/s
#define SYS_JOG_STAGE_4_INTERVAL_MS 64                                         // Stage 4: LVGL minimum timer interval
#define SYS_JOG_STAGE_4_MULTIPLIER 13                                          // Stage 4: 13 increments per 64ms = ~20.3g/s

//------------------------------------------------------------------------------
// LOGGING CONFIGURATION
//------------------------------------------------------------------------------
#define SYS_LOG_EVERY_N_GRIND_LOOPS 1                                          // Log frequency for grind control loop
#define SYS_CONTINUOUS_LOGGING_ENABLED true                                    // Enable/disable continuous logging

//------------------------------------------------------------------------------
// DEBUG HEARTBEAT CONFIGURATION
//------------------------------------------------------------------------------
#define SYS_ENABLE_REALTIME_HEARTBEAT 1                                            // Enable Core 0/Core 1 heartbeat logging (0=disabled, 1=enabled)
#define SYS_REALTIME_HEARTBEAT_INTERVAL_MS 10000                                   // Heartbeat interval in milliseconds

//------------------------------------------------------------------------------
// PRINTF FORMAT STRINGS
//------------------------------------------------------------------------------
#define SYS_WEIGHT_DISPLAY_FORMAT "%.1fg"                                      // Weight display format string
#define SYS_RAW_VALUE_FORMAT "%ld"                                             // Raw load cell value format string

