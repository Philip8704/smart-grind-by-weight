#include "circular_buffer_math.h"
#include "../../config/constants.h"
#include <math.h>
#include <algorithm>
#include <atomic>

CircularBufferMath::CircularBufferMath() {
    write_index = 0;
    samples_count = 0;
    display_filtered_raw = 0;
    display_filter_initialized = false;
    flow_stable_since_ms = 0;
    flow_stability_initialized = false;
    
    // Initialize buffer
    for (uint16_t i = 0; i < MAX_BUFFER_SIZE; i++) {
        circular_buffer[i].raw_value = 0;
        circular_buffer[i].timestamp_ms = 0;
    }
}

void CircularBufferMath::add_sample(int32_t raw_adc_value, uint32_t timestamp_ms) {
    // Raw ADC values should be valid 24-bit signed integers
    // We don't validate range here as different ADCs have different ranges
    
    // Add raw value directly to circular buffer (no IIR filtering)
    circular_buffer[write_index].raw_value = raw_adc_value;
    circular_buffer[write_index].timestamp_ms = timestamp_ms;

    // Publish the sample before the index that exposes it. Readers on the other core
    // walk backwards from write_index, so without this fence they could observe the
    // advanced index while the two field stores are still in the store buffer, and
    // pair a fresh raw value with a stale timestamp.
    std::atomic_thread_fence(std::memory_order_release);

    // Advance write index (circular)
    write_index = (write_index + 1) % MAX_BUFFER_SIZE;

    // Track sample count (up to buffer size)
    if (samples_count < MAX_BUFFER_SIZE) {
        samples_count++;
    }
}

int32_t CircularBufferMath::get_instant_raw() const {
    if (samples_count == 0) return 0;
    
    // Return most recent sample
    return get_latest_sample();
}

int32_t CircularBufferMath::get_latest_sample() const {
    if (samples_count == 0) return 0;
    
    // Most recent sample is at (write_index - 1) % MAX_BUFFER_SIZE
    uint16_t latest_index = (write_index - 1 + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
    return circular_buffer[latest_index].raw_value;
}

// Unified smoothing method with outlier rejection on raw data
int32_t CircularBufferMath::get_smoothed_raw(uint32_t window_ms) const {
    if (samples_count == 0) return 0;
    
    // Calculate max samples needed for this window
    int max_samples = calculate_max_samples_for_window(window_ms);
    
    // Allocate temporary array on stack (reasonable size expected)
    int32_t* samples = (int32_t*)alloca(max_samples * sizeof(int32_t));
    
    // Get samples within time window
    int actual_samples = get_samples_in_window(window_ms, samples);
    
    if (actual_samples == 0) {
        return get_latest_sample(); // Fallback to latest sample
    }
    
    // Apply outlier rejection and return smoothed result
    return apply_outlier_rejection(samples, actual_samples);
}

int CircularBufferMath::get_samples_in_window(uint32_t window_ms, int32_t* samples_out) const {
    if (samples_count == 0) return 0;

    // Pairs with the release fence in add_sample(): everything the producer wrote
    // before advancing write_index is visible to us from here on
    std::atomic_thread_fence(std::memory_order_acquire);

    uint32_t current_time = millis();
    uint32_t window_start = current_time - window_ms;
    int collected_samples = 0;
    
    // Walk backwards from most recent sample
    for (int i = 0; i < samples_count; i++) {
        uint16_t index = (write_index - 1 - i + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
        
        // Check if sample is within time window
        if (circular_buffer[index].timestamp_ms >= window_start) {
            samples_out[collected_samples] = circular_buffer[index].raw_value;
            collected_samples++;
        } else {
            break; // Samples are time-ordered, so we can stop here
        }
    }
    
    return collected_samples;
}

int32_t CircularBufferMath::apply_outlier_rejection(const int32_t* samples, int count) const {
    if (count == 0) return 0;
    if (count == 1) return samples[0];
    if (count == 2) return (samples[0] + samples[1]) / 2;

    // Decide how many to reject from each side
    int reject_each_side = 1; // (HW_LOADCELL_SAMPLE_RATE_SPS == 80) ? 8 : 1;

    // Not enough samples left -> fallback to median
    if (count <= 2 * reject_each_side) {
        int32_t* sorted_samples = (int32_t*)alloca(count * sizeof(int32_t));
        memcpy(sorted_samples, samples, count * sizeof(int32_t));
        std::sort(sorted_samples, sorted_samples + count);
        return sorted_samples[count / 2];
    }

    // Copy and sort
    int32_t* sorted_samples = (int32_t*)alloca(count * sizeof(int32_t));
    memcpy(sorted_samples, samples, count * sizeof(int32_t));
    std::sort(sorted_samples, sorted_samples + count);

    // Average trimmed values
    int samples_to_average = count - 2 * reject_each_side;
    int64_t sum = 0;
    for (int i = reject_each_side; i < count - reject_each_side; i++) {
        sum += sorted_samples[i];
    }

    return static_cast<int32_t>(sum / samples_to_average);
}

int CircularBufferMath::calculate_max_samples_for_window(uint32_t window_ms) const {
    // Estimate max samples
    int estimated_samples = (window_ms * HW_LOADCELL_SAMPLE_RATE_SPS) / 1000 + 10; // +10 for safety margin
    
    // Cap at reasonable limits
    if (estimated_samples > (int)samples_count) {
        estimated_samples = samples_count;
    }
    if (estimated_samples > MAX_BUFFER_SIZE) {
        estimated_samples = MAX_BUFFER_SIZE;
    }
    
    return estimated_samples;
}

int32_t CircularBufferMath::get_raw_low_latency() const {
    // This is the reading the grind controller stops the motor on, so it has to be
    // both quiet and current. A 100ms window at 10 SPS holds a single sample, which
    // means no averaging and no outlier rejection at all - one noisy conversion could
    // stop the motor early or late. The regression uses a wider window for noise
    // reduction but reports the value at the newest sample, so it costs no latency.
    float fitted_value = 0.0f;
    if (get_linear_fit(SYS_CONTROL_FIT_WINDOW_MS, nullptr, &fitted_value, nullptr)) {
        return (int32_t)fitted_value;
    }

    return get_smoothed_raw(100); // Too few samples to fit (startup, or after a buffer clear)
}

int32_t CircularBufferMath::get_display_raw(int32_t raw_deadband) {
    // The smoothing and the newest-sample lookup are pure reads, so they stay outside
    // the lock - get_smoothed_raw() sorts its window, which is far too long to hold a
    // critical section for.
    int32_t current_raw = get_smoothed_raw(300); // 300ms base window
    uint32_t newest_sample_ms = newest_sample_timestamp_ms();

    if (raw_deadband < 0) {
        raw_deadband = 0;
    }

    // Updating the filter is a read-modify-write, and both the control loop and the UI
    // task reach it from different cores. Without this lock the two can interleave and
    // leave display_filtered_raw holding a value neither of them computed.
    portENTER_CRITICAL(&display_filter_mux);

    if (!display_filter_initialized) {
        display_filtered_raw = current_raw;
        display_filter_initialized = true;
        display_filter_last_sample_ms = newest_sample_ms;
    } else if (newest_sample_ms != display_filter_last_sample_ms) {
        // Only advance the filter when the load cell has actually produced a new
        // sample. The UI and the control loop both read this at ~50Hz while the
        // HX711 delivers ~10 samples/s, so stepping the IIR per call would make the
        // decay rate depend on how often someone happens to look at the weight.
        display_filter_last_sample_ms = newest_sample_ms;

        if (abs(current_raw - display_filtered_raw) >= raw_deadband) {
            if (current_raw > display_filtered_raw) {
                // Fast response for increases - coffee arriving shows up immediately
                display_filtered_raw = current_raw;
            } else {
                // Slow response for decreases, so a dip in the reading does not make
                // the displayed weight jump back and forth
                float alpha = SYS_DISPLAY_FILTER_ALPHA_DOWN; // From constants.h
                display_filtered_raw =
                    (int32_t)(alpha * current_raw + (1.0f - alpha) * display_filtered_raw);
            }
        }
    }

    int32_t result = display_filtered_raw;
    portEXIT_CRITICAL(&display_filter_mux);
    return result;
}

uint32_t CircularBufferMath::newest_sample_timestamp_ms() const {
    if (samples_count == 0) {
        return 0;
    }
    uint16_t newest_index = (write_index - 1 + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
    return circular_buffer[newest_index].timestamp_ms;
}

int32_t CircularBufferMath::get_raw_high_latency() const {
    return get_smoothed_raw(300); // 300ms window for final measurements
}

uint32_t CircularBufferMath::get_buffer_time_span_ms() const {
    if (samples_count < 2) return 0;
    
    // Time span from oldest to newest sample
    uint16_t oldest_index = (samples_count < MAX_BUFFER_SIZE) ? 0 : write_index;
    uint16_t newest_index = (write_index - 1 + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
    
    return circular_buffer[newest_index].timestamp_ms - circular_buffer[oldest_index].timestamp_ms;
}

bool CircularBufferMath::get_window_delta(uint32_t window_ms, int32_t* delta_out,
                                          uint32_t* span_ms_out, int* samples_out) const {
    if (!delta_out || samples_count < 2) {
        if (delta_out) {
            *delta_out = 0;
        }
        if (span_ms_out) {
            *span_ms_out = 0;
        }
        if (samples_out) {
            *samples_out = 0;
        }
        return false;
    }

    uint32_t current_time = millis();
    uint32_t window_start = current_time - window_ms;

    int collected = 0;
    int32_t newest_raw = 0;
    int32_t oldest_raw = 0;
    uint32_t newest_ts = 0;
    uint32_t oldest_ts = 0;

    for (int i = 0; i < samples_count; ++i) {
        uint16_t index = (write_index - 1 - i + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
        const AdcSample& sample = circular_buffer[index];

        if (sample.timestamp_ms < window_start) {
            break;
        }

        if (collected == 0) {
            newest_raw = sample.raw_value;
            newest_ts = sample.timestamp_ms;
        }

        oldest_raw = sample.raw_value;
        oldest_ts = sample.timestamp_ms;
        ++collected;
    }

    if (samples_out) {
        *samples_out = collected;
    }

    if (collected < 2) {
        *delta_out = 0;
        if (span_ms_out) {
            *span_ms_out = 0;
        }
        return false;
    }

    *delta_out = newest_raw - oldest_raw;

    if (span_ms_out) {
        if (newest_ts >= oldest_ts) {
            *span_ms_out = newest_ts - oldest_ts;
        } else {
            *span_ms_out = (UINT32_MAX - oldest_ts) + newest_ts + 1;
        }
    }

    return true;
}

bool CircularBufferMath::is_settled(uint32_t window_ms, int32_t threshold_raw_units) const {
    float std_dev = get_standard_deviation_raw(window_ms);
    bool settled = std_dev <= threshold_raw_units;
    
    // Debug output every 1s during settling checks
    static uint32_t last_debug_time = 0;
    if (millis() - last_debug_time > 1000) {
        // Get raw samples for display
        int max_samples = calculate_max_samples_for_window(window_ms);
        if (max_samples > 0) {
            int32_t* samples = (int32_t*)alloca(max_samples * sizeof(int32_t));
            int actual_samples = get_samples_in_window(window_ms, samples);
            
            // Format raw samples on one line (limit to first 10 samples to avoid spam)
            char sample_str[256] = {0};
            int offset = 0;
            int samples_to_show = std::min(actual_samples, 10);
            for (int i = 0; i < samples_to_show; i++) {
                offset += snprintf(sample_str + offset, sizeof(sample_str) - offset, 
                                 "%ld%s", (long)samples[i], (i < samples_to_show - 1) ? "," : "");
            }
            if (actual_samples > 10) {
                offset += snprintf(sample_str + offset, sizeof(sample_str) - offset, "...");
            }
            
            LOG_LOADCELL_DEBUG("[SETTLING] Window:%lums Samples:%d Raw:[%s] StdDev:%.2f Threshold:%ld Settled:%s\n",
                             window_ms, actual_samples, sample_str, std_dev, (long)threshold_raw_units, 
                             settled ? "YES" : "NO");
        }
        last_debug_time = millis();
    }
    
    return settled;
}

float CircularBufferMath::get_settling_confidence(uint32_t window_ms) const {
    // Calculate confidence based on standard deviation
    float std_dev = get_standard_deviation_raw(window_ms);
    
    // Confidence inversely related to standard deviation
    // This is a heuristic - may need tuning based on ADC characteristics
    float max_expected_std = 1000.0f; // Raw units
    float confidence = 1.0f - (std_dev / max_expected_std);
    
    return std::max(0.0f, std::min(1.0f, confidence));
}

float CircularBufferMath::get_standard_deviation_raw(uint32_t window_ms) const {
    // Calculate max samples needed
    int max_samples = calculate_max_samples_for_window(window_ms);
    if (max_samples == 0) return 0.0f;
    
    // Allocate temporary array
    int32_t* samples = (int32_t*)alloca(max_samples * sizeof(int32_t));
    
    // Get samples within time window
    int actual_samples = get_samples_in_window(window_ms, samples);
    
    return calculate_standard_deviation(samples, actual_samples);
}

float CircularBufferMath::calculate_standard_deviation(const int32_t* samples, int count) const {
    if (count <= 1) return 0.0f;
    
    // Calculate mean
    int64_t sum = 0;
    for (int i = 0; i < count; i++) {
        sum += samples[i];
    }
    float mean = (float)sum / count;
    
    // Calculate variance
    float variance_sum = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = samples[i] - mean;
        variance_sum += diff * diff;
    }
    
    float variance = variance_sum / (count - 1);
    return sqrt(variance);
}

int CircularBufferMath::get_samples_with_time_in_window(uint32_t window_ms, int max_samples,
                                                        int32_t* values_out, uint32_t* times_out) const {
    if (samples_count == 0) return 0;

    // See get_samples_in_window(): pairs with the producer's release fence
    std::atomic_thread_fence(std::memory_order_acquire);

    uint32_t window_start = millis() - window_ms;
    int collected = 0;

    // Walk backwards from most recent, so index 0 is always the newest sample
    for (int i = 0; i < (int)samples_count && collected < max_samples; i++) {
        uint16_t index = (write_index - 1 - i + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;

        if (circular_buffer[index].timestamp_ms >= window_start) {
            values_out[collected] = circular_buffer[index].raw_value;
            times_out[collected] = circular_buffer[index].timestamp_ms;
            collected++;
        } else {
            break; // Samples are time-ordered
        }
    }

    return collected;
}

bool CircularBufferMath::get_linear_fit(uint32_t window_ms, float* slope_raw_per_ms,
                                        float* value_at_newest_raw, int* samples_used) const {
    // Least-squares fit of raw value against time across the whole window.
    //
    // Two properties matter here. Using every sample instead of just the endpoints
    // drops the noise on both outputs by roughly sqrt(n). And because coffee arrives
    // at a near-constant rate, a straight line is the right model - so evaluating the
    // fit at the newest sample gives a de-noised reading with no lag, where a plain
    // moving average would sit half a window behind the truth while grinding.
    if (samples_used) *samples_used = 0;

    int max_samples = calculate_max_samples_for_window(window_ms);
    if (max_samples < MIN_SAMPLES_FOR_FIT) return false;

    int32_t* values = (int32_t*)alloca(max_samples * sizeof(int32_t));
    uint32_t* times = (uint32_t*)alloca(max_samples * sizeof(uint32_t));

    int n = get_samples_with_time_in_window(window_ms, max_samples, values, times);
    if (n < MIN_SAMPLES_FOR_FIT) return false;

    // Express time relative to the newest sample so x = 0 is "now" and the
    // magnitudes stay small enough for single precision
    float sum_x = 0.0f, sum_y = 0.0f, sum_xx = 0.0f, sum_xy = 0.0f;
    for (int i = 0; i < n; i++) {
        float x = (float)((int32_t)(times[i] - times[0]));  // <= 0
        float y = (float)values[i];
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }

    float denominator = (float)n * sum_xx - sum_x * sum_x;
    if (fabsf(denominator) < 1e-6f) return false;  // All samples share one timestamp

    float slope = ((float)n * sum_xy - sum_x * sum_y) / denominator;
    float intercept = (sum_y - slope * sum_x) / (float)n;  // Value at x = 0, the newest sample

    if (!isfinite(slope) || !isfinite(intercept)) return false;

    if (slope_raw_per_ms) *slope_raw_per_ms = slope;
    if (value_at_newest_raw) *value_at_newest_raw = intercept;
    if (samples_used) *samples_used = n;
    return true;
}

float CircularBufferMath::get_raw_flow_rate(uint32_t window_ms) const {
    float slope_per_ms = 0.0f;
    if (get_linear_fit(window_ms, &slope_per_ms, nullptr, nullptr)) {
        return slope_per_ms * 1000.0f;  // Raw units per second
    }

    // Not enough samples to fit - fall back to the endpoint difference
    int max_samples = calculate_max_samples_for_window(window_ms);
    if (max_samples < 2) return 0.0f;

    int32_t* samples = (int32_t*)alloca(max_samples * sizeof(int32_t));
    uint32_t* timestamps = (uint32_t*)alloca(max_samples * sizeof(uint32_t));

    int collected = get_samples_with_time_in_window(window_ms, max_samples, samples, timestamps);
    if (collected < 2) return 0.0f;

    int32_t raw_change = samples[0] - samples[collected - 1]; // Most recent - oldest
    uint32_t time_change = timestamps[0] - timestamps[collected - 1];

    if (time_change == 0) return 0.0f;

    return (float)raw_change * 1000.0f / time_change;
}

float CircularBufferMath::get_raw_flow_rate_95th_percentile(uint32_t window_ms) const {
    // Define parameters for the sub-window analysis
    const uint32_t MIN_SAMPLES_FOR_PERCENTILE = 10;
    const uint32_t SUB_WINDOW_MS = 300;
    const uint32_t STEP_MS = 100;
    const int MIN_SUB_WINDOWS = 4;
    const int MAX_SUB_WINDOWS = 32;
    const int MIN_SAMPLES_PER_SUB_WINDOW = 3;

    if (samples_count < MIN_SAMPLES_FOR_PERCENTILE) {
        return get_raw_flow_rate(window_ms); // Fallback for insufficient data
    }

    // Ensure the window is large enough to contain a minimum number of samples
    uint32_t min_window_for_samples = (MIN_SAMPLES_FOR_PERCENTILE * 1000) / HW_LOADCELL_SAMPLE_RATE_SPS;
    uint32_t effective_window_ms = std::max(window_ms, min_window_for_samples);

    // 1. Collect all relevant samples and timestamps in one go.
    int max_samples = calculate_max_samples_for_window(effective_window_ms);
    if (max_samples < MIN_SAMPLES_FOR_PERCENTILE) {
        return get_raw_flow_rate(effective_window_ms);
    }

    int32_t* sample_values = (int32_t*)alloca(max_samples * sizeof(int32_t));
    uint32_t* sample_times = (uint32_t*)alloca(max_samples * sizeof(uint32_t));
    int collected_samples = 0;
    uint32_t current_time = millis();
    uint32_t window_start_time = current_time - effective_window_ms;

    for (int i = 0; i < (int)samples_count && collected_samples < max_samples; ++i) {
        uint16_t index = (write_index - 1 - i + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
        if (circular_buffer[index].timestamp_ms >= window_start_time) {
            // Samples are collected from newest to oldest
            sample_values[collected_samples] = circular_buffer[index].raw_value;
            sample_times[collected_samples] = circular_buffer[index].timestamp_ms;
            collected_samples++;
        } else {
            break; // Samples are time-ordered
        }
    }

    if (collected_samples < MIN_SAMPLES_FOR_PERCENTILE) {
        return get_raw_flow_rate(effective_window_ms);
    }

    // 2. Calculate the number of sub-windows and allocate space for their flow rates.
    int num_sub_windows = (effective_window_ms > SUB_WINDOW_MS) ? 1 + (effective_window_ms - SUB_WINDOW_MS) / STEP_MS : 1;
    num_sub_windows = std::max(MIN_SUB_WINDOWS, std::min(MAX_SUB_WINDOWS, num_sub_windows));
    float* flow_rates = (float*)alloca(num_sub_windows * sizeof(float));
    int valid_flow_rates_count = 0;

    // 3. Iterate through sub-windows and calculate flow rate for each.
    for (int i = 0; i < num_sub_windows; ++i) {
        uint32_t sub_window_end_time = current_time - (i * STEP_MS);
        uint32_t sub_window_start_time = sub_window_end_time - SUB_WINDOW_MS;

        // Find the newest and oldest samples within this sub-window from our collected arrays
        int newest_idx = -1, oldest_idx = -1;
        for (int j = 0; j < collected_samples; ++j) {
            if (sample_times[j] <= sub_window_end_time) {
                if (newest_idx == -1) newest_idx = j;
                if (sample_times[j] >= sub_window_start_time) {
                    oldest_idx = j;
                } else {
                    break; // Past the start of the sub-window
                }
            }
        }

        if (newest_idx != -1 && oldest_idx != -1 && (oldest_idx - newest_idx + 1) >= MIN_SAMPLES_PER_SUB_WINDOW) {
            uint32_t time_delta = sample_times[newest_idx] - sample_times[oldest_idx];
            if (time_delta > 0) {
                int32_t raw_delta = sample_values[newest_idx] - sample_values[oldest_idx];
                flow_rates[valid_flow_rates_count++] = (float)raw_delta * 1000.0f / time_delta;
            }
        }
    }

    // 4. Calculate the 95th percentile from the collected flow rates.
    if (valid_flow_rates_count >= MIN_SAMPLES_PER_SUB_WINDOW) {
        std::sort(flow_rates, flow_rates + valid_flow_rates_count);
        int percentile_95_index = static_cast<int>(valid_flow_rates_count * 0.95f);
        percentile_95_index = std::min(percentile_95_index, valid_flow_rates_count - 1);
        return flow_rates[percentile_95_index];
    }

    // Fallback if we couldn't get enough valid sub-window rates
    return get_raw_flow_rate(effective_window_ms);
}

bool CircularBufferMath::raw_flowrate_is_stable(uint32_t window_ms) const {
    // Simple stability check - compare recent flow rates
    float current_flow = get_raw_flow_rate(window_ms);
    float recent_flow = get_raw_flow_rate(window_ms / 2); // Half window
    
    // Consider stable if flow rates are within 10% of each other
    float threshold = abs(current_flow) * 0.1f;
    return abs(current_flow - recent_flow) <= threshold;
}

int32_t CircularBufferMath::get_min_raw(uint32_t window_ms) const {
    int max_samples = calculate_max_samples_for_window(window_ms);
    if (max_samples == 0) return 0;
    
    int32_t* samples = (int32_t*)alloca(max_samples * sizeof(int32_t));
    int actual_samples = get_samples_in_window(window_ms, samples);
    
    if (actual_samples == 0) return 0;
    
    int32_t min_val = samples[0];
    for (int i = 1; i < actual_samples; i++) {
        if (samples[i] < min_val) {
            min_val = samples[i];
        }
    }
    return min_val;
}

int32_t CircularBufferMath::get_max_raw(uint32_t window_ms) const {
    int max_samples = calculate_max_samples_for_window(window_ms);
    if (max_samples == 0) return 0;
    
    int32_t* samples = (int32_t*)alloca(max_samples * sizeof(int32_t));
    int actual_samples = get_samples_in_window(window_ms, samples);
    
    if (actual_samples == 0) return 0;
    
    int32_t max_val = samples[0];
    for (int i = 1; i < actual_samples; i++) {
        if (samples[i] > max_val) {
            max_val = samples[i];
        }
    }
    return max_val;
}

void CircularBufferMath::reset_display_filter() {
    // Same state get_display_raw() guards - tare and calibration reset it from a
    // different core than the one that may be updating it
    portENTER_CRITICAL(&display_filter_mux);
    display_filter_initialized = false;
    display_filtered_raw = 0;
    display_filter_last_sample_ms = 0;
    portEXIT_CRITICAL(&display_filter_mux);
}

void CircularBufferMath::clear_all_samples() {
    write_index = 0;
    samples_count = 0;
    display_filter_initialized = false;
    flow_stability_initialized = false;
    
    // Clear buffer
    for (uint16_t i = 0; i < MAX_BUFFER_SIZE; i++) {
        circular_buffer[i].raw_value = 0;
        circular_buffer[i].timestamp_ms = 0;
    }
}
