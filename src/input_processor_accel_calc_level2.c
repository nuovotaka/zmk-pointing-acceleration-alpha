// input_processor_accel_calc_level2.c - Level 2 (Standard) calculation functions
// Advanced speed-based acceleration with comprehensive safety
//
// Copyright (c) 2024 The ZMK Contributors
// Modifications (c) 2025 NUOVOTAKA
// SPDX-License-Identifier: MIT

#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <stdlib.h>
#include "../include/drivers/input_processor_accel.h"

LOG_MODULE_DECLARE(input_processor_accel);

// =============================================================================
// LEVEL 2 CALCULATION FUNCTION
// =============================================================================

int32_t accel_standard_calculate(const struct accel_config *cfg, struct accel_data *data, 
                                int32_t input_value, uint16_t code) {
    if (!cfg) {
        LOG_ERR("Configuration pointer is NULL in standard calculation");
        return input_value; // Graceful degradation: return original value
    }
    if (!data) {
        LOG_ERR("Data pointer is NULL in standard calculation");
        return input_value; // Graceful degradation: return original value
    }

#if !defined(CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL_STANDARD)
    LOG_DBG("Standard level not enabled, fallback to simple calculation");
    return accel_simple_calculate(cfg, input_value, code);
#else
    // Enhanced safety: Input value validation for reasonable range with improved logic
    if (abs(input_value) > MAX_REASONABLE_INPUT) {
        if (abs(input_value) > MAX_EXTREME_INPUT) {
            // Extremely large values are likely sensor noise or malicious input
            LOG_WRN("Level2: Input value %d too extreme (>%d), rejecting for safety", 
                    input_value, MAX_EXTREME_INPUT);
            return 0;
        } else {
            // Large but reasonable values - clamp to limit with warning
            LOG_DBG("Level2: Input value %d clamped to %d for safety", 
                    input_value, MAX_REASONABLE_INPUT);
            input_value = (input_value > 0) ? MAX_REASONABLE_INPUT : -MAX_REASONABLE_INPUT;
        }
    }
    
    // Enhanced safety: Data structure validation (simplified)
    if (data->recent_speed > UINT16_MAX / 2) {
        LOG_WRN("Level2: Invalid recent_speed %u, resetting data", data->recent_speed);
        data->recent_speed = 0;
        data->last_time_ms = 0;
    }
    
    uint32_t speed = accel_calculate_simple_speed(data, input_value);
    
    // Enhanced safety: Type-safe speed validation with bounds checking
    uint32_t speed_threshold = (cfg->cfg.level2.speed_threshold > 0) ? 
        (uint32_t)cfg->cfg.level2.speed_threshold : DEFAULT_SPEED_THRESHOLD; // Safe default
    uint32_t speed_max = (cfg->cfg.level2.speed_max > speed_threshold) ? 
        (uint32_t)cfg->cfg.level2.speed_max : speed_threshold + DEFAULT_SPEED_MAX_OFFSET; // Safe default
    
    #if defined(CONFIG_INPUT_PROCESSOR_ACCEL_DEBUG_LOG)
    LOG_DBG("Level2: speed=%u, threshold=%u, max=%u", 
            speed, speed_threshold, speed_max);
    #endif
    
    // Enhanced safety: Speed validation with type-safe comparison
    if (speed > MAX_REASONABLE_SPEED) {
        LOG_ERR("Level2: Calculated speed %u exceeds maximum %u, using fallback", 
                speed, MAX_REASONABLE_SPEED);
        return accel_safe_fallback_calculate(input_value, cfg->cfg.level2.max_factor);
    }
    
    uint32_t dpi_adjusted_sensitivity = calculate_dpi_adjusted_sensitivity(cfg);
    
    // Enhanced safety: Sensitivity validation
    if (dpi_adjusted_sensitivity == 0 || dpi_adjusted_sensitivity > MAX_SAFE_SENSITIVITY) {
        LOG_ERR("Level2: Invalid DPI-adjusted sensitivity %u, using fallback", 
                dpi_adjusted_sensitivity);
        return accel_safe_fallback_calculate(input_value, cfg->cfg.level2.max_factor);
    }
    
    // CRITICAL FIX: Safe sensitivity application with comprehensive overflow protection
    int64_t result;
    
    // Enhanced safety: Use 64-bit safe comparison for overflow detection
    const int64_t max_safe_input = INT64_MAX / dpi_adjusted_sensitivity;
    if (abs(input_value) > max_safe_input) {
        LOG_WRN("Level2: Potential overflow detected, using safe calculation");
        result = safe_multiply_64((int64_t)input_value, (int64_t)dpi_adjusted_sensitivity, 
                                 (int64_t)INT32_MAX * SENSITIVITY_SCALE);
    } else {
        result = (int64_t)input_value * (int64_t)dpi_adjusted_sensitivity;
    }
    
    // Enhanced safety: Comprehensive intermediate result validation
    const int64_t max_intermediate = (int64_t)INT16_MAX * SENSITIVITY_SCALE;
    if (abs(result) > max_intermediate) {
        LOG_WRN("Level2: Intermediate result %lld exceeds safe limit, using fallback", result);
        return accel_safe_fallback_calculate(input_value, cfg->cfg.level2.max_factor);
    }
    
    // Apply sensitivity scaling with safety validation
    if (result != 0) {
        result = result / SENSITIVITY_SCALE;
        
        // Additional safety check after scaling
        if (abs(result) > INT16_MAX) {
            LOG_WRN("Level2: Scaled result %lld exceeds int16 range, using fallback", result);
            return accel_safe_fallback_calculate(input_value, cfg->cfg.level2.max_factor);
        }
    }
    
    // Enhanced safety: Speed-based acceleration with type-safe operations
    uint32_t factor = (uint32_t)cfg->cfg.level2.min_factor;
    
    // Enhanced safety: Type-safe speed threshold validation
    if (speed_threshold >= speed_max) {
        LOG_ERR("Level2: Invalid speed configuration (threshold=%u >= max=%u), using linear", 
                speed_threshold, speed_max);
        factor = (uint32_t)cfg->cfg.level2.min_factor;
    } else if (speed > speed_threshold && speed_max > speed_threshold) {
        if (speed >= speed_max) {
            factor = (uint32_t)cfg->cfg.level2.max_factor;
        } else {
            uint32_t speed_range = speed_max - speed_threshold;
            uint32_t speed_offset = speed - speed_threshold;
            
            // Enhanced safety: Validate speed range
            if (speed_range == 0) {
                LOG_ERR("Level2: Zero speed range, using min factor");
                factor = cfg->cfg.level2.min_factor;
            } else {
                // Safe calculation of normalized speed (0-1000)
                uint64_t t_temp = ((uint64_t)speed_offset * SPEED_NORMALIZATION) / speed_range;
                uint32_t t = (t_temp > SPEED_NORMALIZATION) ? SPEED_NORMALIZATION : (uint32_t)t_temp;
                
                // Enhanced safety: Validate acceleration exponent
                uint8_t safe_exponent = ACCEL_CLAMP(cfg->cfg.level2.acceleration_exponent, 1, 5);
                if (safe_exponent != cfg->cfg.level2.acceleration_exponent) {
                    LOG_WRN("Level2: Clamping acceleration exponent from %u to %u", 
                            cfg->cfg.level2.acceleration_exponent, safe_exponent);
                }
                
                uint32_t curve;
                // Enhanced safety: Try exponential curve with fallback
                #if defined(CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL_STANDARD)
                curve = calculate_exponential_curve(t, safe_exponent);
                
                // Enhanced safety: Validate curve result
                if (curve > SPEED_NORMALIZATION * 10) {
                    LOG_WRN("Level2: Exponential curve result %u too large, using linear", curve);
                    curve = t; // Linear fallback
                }
                #else
                curve = t; // Linear fallback if exponential not available
                #endif
                
                // Final clamp and factor calculation
                curve = ACCEL_CLAMP(curve, 0, SPEED_NORMALIZATION);
                
                // Enhanced safety: Type-safe factor calculation with overflow protection
                uint32_t max_factor = (uint32_t)cfg->cfg.level2.max_factor;
                uint32_t min_factor = (uint32_t)cfg->cfg.level2.min_factor;
                
                if (max_factor >= min_factor) {
                    uint64_t factor_range = (uint64_t)max_factor - min_factor;
                    
                    // Enhanced safety: Check for potential overflow with type safety
                    if (factor_range > UINT32_MAX / SPEED_NORMALIZATION) {
                        LOG_WRN("Level2: Factor range too large, using conservative calculation");
                        factor = min_factor + (uint32_t)((factor_range * curve) / (SPEED_NORMALIZATION * 2));
                    } else {
                        uint64_t factor_add = (factor_range * curve) / SPEED_NORMALIZATION;
                        factor = min_factor + (uint32_t)ACCEL_CLAMP(factor_add, 0, factor_range);
                    }
                } else {
                    LOG_WRN("Level2: max_factor < min_factor, using min_factor");
                    factor = min_factor;
                }
            }
        }
        
        // Enhanced safety: Type-safe final factor validation
        uint32_t min_factor = (uint32_t)cfg->cfg.level2.min_factor;
        uint32_t max_factor = (uint32_t)ACCEL_CLAMP(cfg->cfg.level2.max_factor, SENSITIVITY_SCALE, MAX_SAFE_FACTOR);
        factor = ACCEL_CLAMP(factor, min_factor, max_factor);
        
        // Debug: Log acceleration factor calculation
        LOG_DBG("Level2: factor=%u, min=%u, max=%u", 
                factor, cfg->cfg.level2.min_factor, cfg->cfg.level2.max_factor);
        
        // Enhanced safety: Apply acceleration with comprehensive overflow protection
        if (factor > SENSITIVITY_SCALE) {
            // Check if multiplication would overflow
            if (abs(result) > (int64_t)INT16_MAX * SENSITIVITY_SCALE / factor) {
                LOG_WRN("Level2: Acceleration would cause overflow, using fallback");
                return accel_safe_fallback_calculate(input_value, factor);
            }
            
            int64_t temp_result = safe_multiply_64(result, (int64_t)factor, 
                                                 (int64_t)INT16_MAX * SENSITIVITY_SCALE);
            result = temp_result / SENSITIVITY_SCALE;
            
            // Enhanced safety: Check result after acceleration
            if (abs(result) > INT16_MAX) {
                LOG_WRN("Level2: Accelerated result %lld exceeds int16 range, clamping", result);
                result = (result > 0) ? INT16_MAX : INT16_MIN;
            }
        }
    }
    
    // Enhanced safety: Y-axis boost with comprehensive overflow protection
    if (code == INPUT_REL_Y) {
        uint16_t y_boost = accel_decode_y_boost(cfg->y_boost_scaled);
        if (y_boost != SENSITIVITY_SCALE) {
            // Enhanced safety: Validate y_boost value
            uint32_t safe_y_boost = ACCEL_CLAMP(y_boost, 500, 3000);
            if (safe_y_boost != y_boost) {
                LOG_WRN("Level2: Clamping y_boost from %u to %u", y_boost, safe_y_boost);
            }
            
            // Enhanced safety: Check if Y-boost would cause overflow
            if (abs(result) > (int64_t)INT16_MAX * SENSITIVITY_SCALE / safe_y_boost) {
                LOG_WRN("Level2: Y-boost would cause overflow, using conservative boost");
                safe_y_boost = SENSITIVITY_SCALE + (safe_y_boost - SENSITIVITY_SCALE) / 2;
            }
            
            int64_t temp_result = safe_multiply_64(result, (int64_t)safe_y_boost, 
                                                 (int64_t)INT16_MAX * SENSITIVITY_SCALE);
            result = temp_result / SENSITIVITY_SCALE;
            
            // Enhanced safety: Check result after Y-boost
            if (abs(result) > INT16_MAX) {
                LOG_WRN("Level2: Y-boosted result %lld exceeds int16 range, clamping", result);
                result = (result > 0) ? INT16_MAX : INT16_MIN;
            }
        }
    }
    
    // Enhanced safety: Calculate final accelerated value with comprehensive validation
    int32_t accelerated_value = safe_int64_to_int32(result);
    
    // Enhanced safety: Multiple range checks for Level 2
    if (abs(accelerated_value) > INT16_MAX) {
        LOG_ERR("Level2: Accelerated value %d exceeds int16 range, clamping", accelerated_value);
        accelerated_value = (accelerated_value > 0) ? INT16_MAX : INT16_MIN;
    }
    
    // Enhanced safety: Sanity check for Level 2 - detect unreasonable results
    if (abs(input_value) <= 50 && abs(accelerated_value) > 2000) {
        LOG_WRN("Level2: Suspicious result %d for input %d, using conservative fallback", 
                accelerated_value, input_value);
        return accel_safe_fallback_calculate(input_value, cfg->cfg.level2.max_factor);
    }
    
    // Remainder processing removed for safety and simplicity
    // The precision loss (1/1000) is negligible for practical mouse usage
    
    // Enhanced safety: Improved minimum movement guarantee based on calculation result
    if (input_value != 0 && accelerated_value == 0) {
        // Calculate what the raw result would have been before final scaling
        int64_t raw_result = (int64_t)input_value * (int64_t)dpi_adjusted_sensitivity;
        
        // Only output movement if the raw calculation was >= 0.5 (half of SENSITIVITY_SCALE)
        if (abs(raw_result) >= SENSITIVITY_SCALE / CONSERVATIVE_FALLBACK_MULTIPLIER) {
            accelerated_value = (raw_result > 0) ? 1 : -1;
            LOG_DBG("Level2: Minimum movement applied - raw=%lld -> output=%d", raw_result, accelerated_value);
        } else {
            // Raw calculation was < 0.5, legitimately should be 0
            accelerated_value = 0;
            LOG_DBG("Level2: Micro movement ignored - raw=%lld (< 0.5 threshold)", raw_result);
        }
    }
    
    // Level 2 calculation complete
    
    // Enhanced safety: Final comprehensive validation
    int16_t final_result = safe_int32_to_int16(accelerated_value);
    
    // Enhanced safety: Ultimate bounds check
    if (abs(final_result) > INT16_MAX) {
        LOG_ERR("Level2: Final result %d exceeds int16 bounds, emergency clamp", final_result);
        final_result = (final_result > 0) ? INT16_MAX : INT16_MIN;
    }
    
    // Enhanced safety: Log suspicious Level 2 results
    static uint32_t level2_log_counter = 0;
    if ((level2_log_counter++ % LOG_COUNTER_INTERVAL) == 0 || abs(final_result) > abs(input_value) * SUSPICIOUS_RESULT_MULTIPLIER) {
        LOG_DBG("Level2: Input=%d, Speed=%u, Factor=%u, Final=%d", 
                input_value, speed, factor, final_result);
    }
    
    return final_result;
#endif
}