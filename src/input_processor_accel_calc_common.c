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
    
    // Enhanced safety: More robust overflow detection
    if (a > 0 && b > 0) {
        if (a > max_result / b) return max_result;
    } else if (a < 0 && b < 0) {
        // Both negative: result is positive
        if ((-a) > max_result / (-b)) return max_result;
    } else if (a < 0 && b > 0) {
        // a negative, b positive: result is negative
        if ((-a) > max_result / b) return -max_result;
    } else if (a > 0 && b < 0) {
        // a positive, b negative: result is negative
        if (a > max_result / (-b)) return -max_result;
    }
    
    return a * b;
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
    // Input validation
    if (t > SPEED_NORMALIZATION) {
        t = SPEED_NORMALIZATION;
    }
    
    switch (exponent) {
        case 1: // Linear
            return t;
            
        case 2: // Mild exponential
            {
                uint64_t t_sq = (uint64_t)t * t;
                uint32_t quad = (t_sq > CURVE_MILD_DIVISOR * UINT32_MAX) ? 
                    UINT32_MAX : (uint32_t)(t_sq / CURVE_MILD_DIVISOR);
                uint32_t result = (t > UINT32_MAX - quad) ? UINT32_MAX : t + quad;
                return ACCEL_CLAMP(result, 0, SPEED_NORMALIZATION * 2);
            }
            
        case 3: // Moderate exponential
            {
                uint64_t t_sq = (uint64_t)t * t;
                uint32_t quad = (t_sq > CURVE_MODERATE_QUAD_DIV * UINT32_MAX) ? 
                    UINT32_MAX : (uint32_t)(t_sq / CURVE_MODERATE_QUAD_DIV);
                uint64_t t_cb = (t_sq > UINT64_MAX / t) ? UINT64_MAX : t_sq * t;
                uint32_t cubic = (t_cb > CURVE_MODERATE_CUBIC_DIV * UINT32_MAX) ? 
                    UINT32_MAX : (uint32_t)(t_cb / CURVE_MODERATE_CUBIC_DIV);
                uint64_t temp_result = (uint64_t)t + quad + cubic;
                uint32_t result = (temp_result > UINT32_MAX) ? UINT32_MAX : (uint32_t)temp_result;
                return ACCEL_CLAMP(result, 0, SPEED_NORMALIZATION * 3);
            }
            
        case 4: // Strong exponential
            {
                uint64_t t_sq = (uint64_t)t * t;
                uint32_t quad = (t_sq > CURVE_STRONG_QUAD_DIV * UINT32_MAX) ? 
                    UINT32_MAX : (uint32_t)(t_sq / CURVE_STRONG_QUAD_DIV);
                uint64_t t_cb = (t_sq > UINT64_MAX / t) ? UINT64_MAX : t_sq * t;
                uint32_t cubic = (t_cb > CURVE_STRONG_CUBIC_DIV * UINT32_MAX) ? 
                    UINT32_MAX : (uint32_t)(t_cb / CURVE_STRONG_CUBIC_DIV);
                uint64_t temp_result = (uint64_t)t + quad + cubic;
                uint32_t result = (temp_result > UINT32_MAX) ? UINT32_MAX : (uint32_t)temp_result;
                return ACCEL_CLAMP(result, 0, SPEED_NORMALIZATION * 4);
            }
            
        case 5: // Aggressive exponential
            {
                uint64_t t_sq = (uint64_t)t * t;
                uint32_t quad = (t_sq > CURVE_AGGRESSIVE_QUAD_DIV * UINT32_MAX) ? 
                    UINT32_MAX : (uint32_t)(t_sq / CURVE_AGGRESSIVE_QUAD_DIV);
                uint64_t t_cb = (t_sq > UINT64_MAX / t) ? UINT64_MAX : t_sq * t;
                uint32_t cubic = (t_cb > CURVE_AGGRESSIVE_CUBIC_DIV * UINT32_MAX) ? 
                    UINT32_MAX : (uint32_t)(t_cb / CURVE_AGGRESSIVE_CUBIC_DIV);
                uint64_t temp_result = (uint64_t)t + quad + cubic;
                uint32_t result = (temp_result > UINT32_MAX) ? UINT32_MAX : (uint32_t)temp_result;
                return ACCEL_CLAMP(result, 0, SPEED_NORMALIZATION * 5);
            }
            
        default: // Fallback quadratic
            {
                uint64_t t_sq = (uint64_t)t * t;
                uint32_t result = (t_sq > CURVE_DEFAULT_DIVISOR * UINT32_MAX) ? 
                    UINT32_MAX : (uint32_t)(t_sq / CURVE_DEFAULT_DIVISOR);
                return ACCEL_CLAMP(result, 0, SPEED_NORMALIZATION);
            }
    }
}
#endif // CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL_STANDARD