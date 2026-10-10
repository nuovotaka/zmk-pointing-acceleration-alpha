// input_processor_accel_calc_level2.c - Level 2 calculation (embedded optimized)  
// Speed-based acceleration for keyboards with minimal overhead
//
// Copyright (c) 2024 The ZMK Contributors
// Modifications (c) 2025 NUOVOTAKA
// SPDX-License-Identifier: MIT

#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <stdlib.h>
#include "../include/drivers/input_processor_accel.h"

LOG_MODULE_DECLARE(input_processor_accel);

// Forward declarations
int32_t validate_and_clamp_input(int32_t input_value);
int32_t safe_multiply_embedded(int32_t a, int32_t b);
uint16_t calculate_speed_embedded(struct accel_data *data, int32_t input_value);

// =============================================================================
// LEVEL 2 CALCULATION (EMBEDDED OPTIMIZED)
// =============================================================================

int32_t accel_standard_calculate(const struct accel_config *cfg, struct accel_data *data, 
                                int32_t input_value, uint16_t code) {
    if (!cfg || !data) return input_value;

#if !defined(CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL_STANDARD)
    // Fallback to simple calculation
    return accel_simple_calculate(cfg, input_value, code);
#else
    // Validate and clamp input
    input_value = validate_and_clamp_input(input_value);
    if (input_value == 0) return 0;
    
    // Calculate speed (simplified for embedded)
    uint16_t speed = calculate_speed_embedded(data, input_value);
    
    // Get configuration values
    uint16_t speed_threshold = cfg->cfg.level2.speed_threshold;
    uint16_t speed_max = cfg->cfg.level2.speed_max;
    uint16_t min_factor = cfg->cfg.level2.min_factor;
    uint16_t max_factor = cfg->cfg.level2.max_factor;
    
    // Simple DPI adjustment
    uint32_t sensitivity = cfg->cfg.level2.sensitivity;
    uint16_t dpi = accel_decode_sensor_dpi(cfg->sensor_dpi_class);
    if (dpi != 800) {
        sensitivity = (sensitivity * 800) / dpi;
        if (sensitivity < 500) sensitivity = 500;
        if (sensitivity > 2000) sensitivity = 2000;
    }
    
    // Apply base sensitivity  
    int32_t result = safe_multiply_embedded(input_value, sensitivity) / 1000;
    
    // Speed-based acceleration
    uint16_t factor = min_factor;
    if (speed > speed_threshold && speed_max > speed_threshold) {
        if (speed >= speed_max) {
            factor = max_factor;
        } else {
            // Linear interpolation for embedded systems
            uint16_t progress = ((speed - speed_threshold) * 1000) / (speed_max - speed_threshold);
            factor = min_factor + ((max_factor - min_factor) * progress) / 1000;
        }
    }
    
    // Apply acceleration factor
    if (factor > 1000) {
        result = safe_multiply_embedded(result, factor) / 1000;
    }
    
    // Y-axis boost (simplified)
    if (code == INPUT_REL_Y && cfg->y_boost_scaled > 0) {
        uint16_t y_boost = 1000 + (cfg->y_boost_scaled * 10);
        result = safe_multiply_embedded(result, y_boost) / 1000;
    }
    
    // Ensure minimum movement
    if (input_value != 0 && result == 0) {
        result = (input_value > 0) ? 1 : -1;
    }
    
    // Final clamp to 16-bit range
    if (result > INT16_MAX) return INT16_MAX;
    if (result < INT16_MIN) return INT16_MIN;
    
    return result;
#endif
}