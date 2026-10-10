// input_processor_accel_calc_level1.c - Level 1 calculation (embedded optimized)
// Simplified for keyboard/pointing devices with minimal memory usage
//
// Copyright (c) 2024 The ZMK Contributors
// Modifications (c) 2025 NUOVOTAKA
// SPDX-License-Identifier: MIT

#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <stdlib.h>
#include "../include/drivers/input_processor_accel.h"

LOG_MODULE_DECLARE(input_processor_accel);

// Forward declaration of embedded helpers
int32_t validate_and_clamp_input(int32_t input_value);
int32_t safe_multiply_embedded(int32_t a, int32_t b);
uint16_t get_acceleration_factor(int32_t abs_input, uint8_t curve_type, uint16_t max_factor);

// =============================================================================
// LEVEL 1 CALCULATION (EMBEDDED OPTIMIZED)
// =============================================================================

int32_t accel_simple_calculate(const struct accel_config *cfg, int32_t input_value, uint16_t code) {
    if (!cfg) return input_value;

#if !defined(CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL_SIMPLE)
    // Simple fallback for disabled level
    int32_t clamped = validate_and_clamp_input(input_value);
    return (abs(clamped) > 5) ? safe_multiply_embedded(clamped, 1200) / 1000 : clamped;
#else
    // Validate and clamp input
    input_value = validate_and_clamp_input(input_value);
    if (input_value == 0) return 0;
    
    // Get DPI-adjusted sensitivity (simplified)
    uint32_t sensitivity = cfg->cfg.level1.sensitivity;
    uint16_t dpi = accel_decode_sensor_dpi(cfg->sensor_dpi_class);
    
    // Simple DPI adjustment
    if (dpi != 800) {
        sensitivity = (sensitivity * 800) / dpi;
        if (sensitivity < 500) sensitivity = 500;
        if (sensitivity > 2000) sensitivity = 2000;
    }
    
    // Apply base sensitivity
    int32_t result = safe_multiply_embedded(input_value, sensitivity) / 1000;
    
    // Apply acceleration curve for larger movements
    int32_t abs_input = abs(input_value);
    if (abs_input > ACCEL_THRESHOLD) {
        uint16_t factor = get_acceleration_factor(abs_input, cfg->cfg.level1.curve_type, 
                                                cfg->cfg.level1.max_factor);
        result = safe_multiply_embedded(result, factor) / 1000;
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