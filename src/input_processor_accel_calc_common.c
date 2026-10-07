// input_processor_accel_calc_common.c - Common calculation functions
// Shared between Level 1 and Level 2 implementations
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
// OVERFLOW-SAFE HELPER FUNCTIONS (SHARED)
// =============================================================================

int64_t safe_multiply_64(int64_t a, int64_t b, int64_t max_result) {
    if (a == 0 || b == 0) return 0;
    
    // CRITICAL SECURITY: Enhanced overflow detection with complete coverage
    // Handle all sign combinations systematically
    
    // Step 1: Convert to absolute values for overflow checking
    int64_t abs_a = (a < 0) ? -a : a;
    int64_t abs_b = (b < 0) ? -b : b;
    int64_t abs_max = (max_result < 0) ? -max_result : max_result;
    
    // Step 2: Check for overflow in absolute value multiplication
    if (abs_a > 0 && abs_b > abs_max / abs_a) {
        // Would overflow, return clamped result with proper sign
        bool result_negative = (a < 0) != (b < 0); // XOR for sign
        return result_negative ? -abs_max : abs_max;
    }
    
    // Step 3: Perform safe multiplication
    int64_t result = a * b;
    
    // Step 4: Final bounds checking
    if (result > max_result) return max_result;
    if (result < -max_result) return -max_result;
    
    return result;
}

int32_t safe_int64_to_int32(int64_t value) {
    if (value > INT32_MAX) return INT32_MAX;
    if (value < INT32_MIN) return INT32_MIN;
    return (int32_t)value;
}

int16_t safe_int32_to_int16(int32_t value) {
    if (value > INT16_MAX) return INT16_MAX;
    if (value < INT16_MIN) return INT16_MIN;
    return (int16_t)value;
}

// =============================================================================
// DPI ADJUSTMENT (SHARED)
// =============================================================================

uint32_t calculate_dpi_adjusted_sensitivity(const struct accel_config *cfg) {
    if (!cfg) {
        LOG_ERR("Configuration pointer is NULL in DPI adjustment");
        return SENSITIVITY_SCALE; // Graceful degradation: return neutral sensitivity
    }
    
    uint32_t dpi_adjusted_sensitivity;
    
    uint16_t sensor_dpi = accel_decode_sensor_dpi(cfg->sensor_dpi_class);
    if (sensor_dpi > 0 && sensor_dpi <= MAX_SENSOR_DPI) {
        // Get sensitivity based on level
        uint16_t sensitivity = (cfg->level == 1) ? cfg->cfg.level1.sensitivity : cfg->cfg.level2.sensitivity;
        
        // Improved DPI scaling using 64-bit arithmetic for precision
        if (sensor_dpi > STANDARD_DPI_REFERENCE) {
            // High DPI sensor: reduce sensitivity with precise calculation
            uint64_t temp = (uint64_t)sensitivity * STANDARD_DPI_REFERENCE;
            dpi_adjusted_sensitivity = (uint32_t)(temp / sensor_dpi);
            
            // Apply conservative limits to prevent extreme reduction
            uint32_t min_allowed = sensitivity / FALLBACK_MAX_REDUCTION; // Max reduction
            if (dpi_adjusted_sensitivity < min_allowed) {
                dpi_adjusted_sensitivity = min_allowed;
            }
            
            // Ensure minimum sensitivity
            if (dpi_adjusted_sensitivity < MIN_SAFE_SENSITIVITY) {
                dpi_adjusted_sensitivity = MIN_SAFE_SENSITIVITY;
            }
        } else if (sensor_dpi < STANDARD_DPI_REFERENCE) {
            // Low DPI sensor: increase sensitivity with precise calculation
            uint64_t temp = (uint64_t)sensitivity * STANDARD_DPI_REFERENCE;
            dpi_adjusted_sensitivity = (uint32_t)(temp / sensor_dpi);
            
            // Apply conservative limits to prevent extreme increase
            uint32_t max_allowed = sensitivity * FALLBACK_MAX_INCREASE; // Max increase
            if (dpi_adjusted_sensitivity > max_allowed) {
                dpi_adjusted_sensitivity = max_allowed;
            }
            
            // Ensure maximum sensitivity
            if (dpi_adjusted_sensitivity > MAX_SAFE_SENSITIVITY) {
                dpi_adjusted_sensitivity = MAX_SAFE_SENSITIVITY;
            }
        } else {
            // Standard DPI: use as-is
            dpi_adjusted_sensitivity = sensitivity;
        }
        
        LOG_DBG("DPI adjustment: %u DPI, sensitivity %u -> %u (precise calc)", 
                sensor_dpi, sensitivity, dpi_adjusted_sensitivity);
    } else {
        // Invalid or missing DPI: use original sensitivity
        uint16_t sensitivity = (cfg->level == 1) ? cfg->cfg.level1.sensitivity : cfg->cfg.level2.sensitivity;
        dpi_adjusted_sensitivity = sensitivity;
        LOG_WRN("Invalid sensor DPI %u, using original sensitivity %u", 
                sensor_dpi, sensitivity);
    }
    
    // Final safety clamp
    dpi_adjusted_sensitivity = ACCEL_CLAMP(dpi_adjusted_sensitivity, MIN_SAFE_SENSITIVITY, MAX_SAFE_SENSITIVITY);
    return dpi_adjusted_sensitivity;
}

// =============================================================================
// EXPONENTIAL CURVE CALCULATION (SHARED)
// =============================================================================

#if defined(CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL_STANDARD)
uint32_t calculate_exponential_curve(uint32_t t, uint8_t exponent) {
    // CRITICAL SECURITY: Enhanced input validation and overflow protection
    if (t > SPEED_NORMALIZATION) {
        LOG_WRN("Exponential curve: Input %u exceeds normalization %u, clamping", 
                t, SPEED_NORMALIZATION);
        t = SPEED_NORMALIZATION;
    }
    
    if (exponent > 5) {
        LOG_ERR("Exponential curve: Invalid exponent %u, using default 2", exponent);
        exponent = 2;
    }
    
    switch (exponent) {
        case 1: // Linear
            return t;
            
        case 2: // Mild exponential - Enhanced security
            {
                // Step 1: Check for square overflow
                if (t > 0 && t > UINT64_MAX / t) {
                    LOG_WRN("Exponential curve: t^2 would overflow, using linear");
                    return ACCEL_CLAMP(t * 2, 0, SPEED_NORMALIZATION * 2);
                }
                
                uint64_t t_sq = (uint64_t)t * t;
                
                // Step 2: Check division safety
                if (CURVE_MILD_DIVISOR == 0) {
                    LOG_ERR("Exponential curve: Zero divisor detected");
                    return t; // Linear fallback
                }
                
                uint32_t quad = (t_sq > CURVE_MILD_DIVISOR * UINT32_MAX) ? 
                    UINT32_MAX : (uint32_t)(t_sq / CURVE_MILD_DIVISOR);
                    
                // Step 3: Check final addition safety
                uint32_t result = (t > UINT32_MAX - quad) ? UINT32_MAX : t + quad;
                return ACCEL_CLAMP(result, 0, SPEED_NORMALIZATION * 2);
            }
            
        case 3: // Moderate exponential - Enhanced security
        case 4: // Strong exponential - Enhanced security  
        case 5: // Aggressive exponential - Enhanced security
            {
                // For higher exponents, use more conservative approach to prevent overflow
                // Step 1: Validate input for higher powers
                if (t > 100) { // Conservative limit for higher exponents
                    LOG_DBG("Exponential curve: Large input %u for exponent %u, using quadratic approximation", 
                            t, exponent);
                    // Use quadratic approximation for large inputs
                    uint64_t t_sq = (uint64_t)t * t;
                    uint32_t multiplier = (exponent == 3) ? 3 : (exponent == 4) ? 4 : 5;
                    uint32_t result = t + (uint32_t)(t_sq / (2000ULL / multiplier));
                    return ACCEL_CLAMP(result, 0, SPEED_NORMALIZATION * multiplier);
                }
                
                // For small inputs, use safer calculation
                uint64_t t_sq = (uint64_t)t * t;
                uint32_t quad_factor = (exponent == 3) ? 1000 : (exponent == 4) ? 800 : 600;
                uint32_t result = t + (uint32_t)(t_sq / quad_factor);
                return ACCEL_CLAMP(result, 0, SPEED_NORMALIZATION * exponent);
            }
            
        default: // Safe fallback
            {
                LOG_WRN("Exponential curve: Unknown exponent %u, using quadratic fallback", exponent);
                uint64_t t_sq = (uint64_t)t * t;
                uint32_t result = (t_sq > CURVE_DEFAULT_DIVISOR * UINT32_MAX) ? 
                    UINT32_MAX : (uint32_t)(t_sq / CURVE_DEFAULT_DIVISOR);
                return ACCEL_CLAMP(result, 0, SPEED_NORMALIZATION);
            }
    }
}
#endif // CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL_STANDARD