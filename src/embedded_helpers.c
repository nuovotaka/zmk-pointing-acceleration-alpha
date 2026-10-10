// embedded_helpers.c - Optimized helper functions for embedded keyboards
// Focused on minimal memory footprint and fast execution
//
// Copyright (c) 2024 The ZMK Contributors
// Modifications (c) 2025 NUOVOTAKA  
// SPDX-License-Identifier: MIT

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include "../include/drivers/input_processor_accel.h"

LOG_MODULE_DECLARE(input_processor_accel);

// =============================================================================
// EMBEDDED INPUT VALIDATION (SIMPLIFIED)
// =============================================================================

int32_t validate_and_clamp_input(int32_t input_value) {
    // Fast clamp without logging in interrupt context
    if (input_value > MAX_INPUT_VALUE) return MAX_INPUT_VALUE;
    if (input_value < -MAX_INPUT_VALUE) return -MAX_INPUT_VALUE;
    return input_value;
}

// =============================================================================
// SAFE ARITHMETIC FOR EMBEDDED (MINIMAL OVERHEAD)
// =============================================================================

int32_t safe_multiply_embedded(int32_t a, int32_t b) {
    // Simplified overflow check for embedded systems
    if (a == 0 || b == 0) return 0;
    
    // Quick overflow detection using bit shifts
    if ((a > 0 && b > 0 && a > INT16_MAX / b) ||
        (a < 0 && b < 0 && (-a) > INT16_MAX / (-b)) ||
        (a > 0 && b < 0 && a > INT16_MAX / (-b)) ||
        (a < 0 && b > 0 && (-a) > INT16_MAX / b)) {
        return (a > 0) == (b > 0) ? INT16_MAX : INT16_MIN;
    }
    
    return a * b;
}

// =============================================================================
// EMBEDDED SPEED CALCULATION (INTERRUPT-SAFE)  
// =============================================================================
// NOTE: Speed calculation is implemented in input_processor_accel_utils.c
// as accel_calculate_simple_speed() - using that implementation

// =============================================================================
// EMBEDDED ACCELERATION CURVES (LOOKUP TABLE)
// =============================================================================

static const uint16_t accel_curve_mild[16] = {
    1000, 1050, 1100, 1200, 1300, 1450, 1600, 1800,
    2000, 2250, 2500, 2750, 3000, 3000, 3000, 3000
};

static const uint16_t accel_curve_strong[16] = {
    1000, 1100, 1300, 1600, 2000, 2500, 3000, 3000,
    3000, 3000, 3000, 3000, 3000, 3000, 3000, 3000
};

uint16_t get_acceleration_factor(int32_t abs_input, uint8_t curve_type, uint16_t max_factor) {
    if (abs_input <= 1) return 1000; // No acceleration for minimal input
    
    // Map input (0-200) to table index (0-15)
    uint8_t index = (abs_input > 200) ? 15 : (abs_input * 15) / 200;
    
    uint16_t factor;
    switch (curve_type) {
        case 2: // Strong
            factor = accel_curve_strong[index];
            break;
        case 1: // Mild
        default:
            factor = accel_curve_mild[index];
            break;
    }
    
    // Clamp to max_factor
    return (factor > max_factor) ? max_factor : factor;
}