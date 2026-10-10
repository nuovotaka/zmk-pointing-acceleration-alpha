/*
 * Copyright (c) 2024 The ZMK Contributors
 * Modifications (c) 2025 NUOVOTAKA
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>



#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// FORWARD DECLARATIONS
// =============================================================================

// Forward declaration for ZMK input processor state
struct zmk_input_processor_state;

// =============================================================================
// EMBEDDED SYSTEM OPTIMIZED CONSTANTS
// =============================================================================

// Core limits optimized for keyboard/pointing devices
#define MAX_INPUT_VALUE         200     // Keyboard/mouse typical max delta
#define MAX_ACCEL_FACTOR        3000    // Conservative max for embedded (3x)
#define MAX_SENSITIVITY         1500    // Reduced for keyboard use
#define MIN_SENSITIVITY         300     // Practical minimum
#define MAX_SPEED               10000   // Sufficient for keyboard devices

// Backward compatibility with validation requirements
#define MAX_SAFE_FACTOR         MAX_ACCEL_FACTOR
#define MAX_SAFE_SENSITIVITY    MAX_SENSITIVITY  
#define MIN_SAFE_SENSITIVITY    MIN_SENSITIVITY
#define MAX_REASONABLE_SPEED    MAX_SPEED
#define MAX_SAFE_INPUT_VALUE    MAX_INPUT_VALUE  // Legacy compatibility

// Embedded-optimized configuration ranges
#define SENSITIVITY_RANGE_MIN   300
#define SENSITIVITY_RANGE_MAX   1500
#define FACTOR_RANGE_MIN        1000
#define FACTOR_RANGE_MAX        3000
#define DPI_RANGE_MIN          400
#define DPI_RANGE_MAX          3200     // Sufficient for most keyboards
#define SPEED_THRESHOLD_RANGE  500
#define ACCEL_EXP_MAX          3       // Simplified curve options

// Backward compatibility with old naming
#define SENSITIVITY_MIN         SENSITIVITY_RANGE_MIN
#define SENSITIVITY_MAX         SENSITIVITY_RANGE_MAX  
#define MAX_FACTOR_MIN          FACTOR_RANGE_MIN
#define MAX_FACTOR_MAX          FACTOR_RANGE_MAX
#define CURVE_TYPE_MIN          0
#define CURVE_TYPE_MAX          2
#define SENSOR_DPI_MIN          DPI_RANGE_MIN
#define SENSOR_DPI_MAX          DPI_RANGE_MAX
#define SPEED_THRESHOLD_MIN     100
#define SPEED_THRESHOLD_MAX     2000
#define SPEED_MAX_MIN           1000
#define SPEED_MAX_MAX           8000
#define MIN_FACTOR_MIN          200
#define MIN_FACTOR_MAX          1500
#define ACCEL_EXPONENT_MIN      1
#define ACCEL_EXPONENT_MAX      ACCEL_EXP_MAX

// Embedded safety limits (simplified)
#define EMERGENCY_LIMIT         200     // Emergency clamp
#define ACCEL_THRESHOLD         5       // Basic threshold
#define FALLBACK_FACTOR         2       // Conservative fallback

// Missing fallback constants for compatibility
#define FALLBACK_MAX_REDUCTION      4       // Maximum DPI reduction factor
#define FALLBACK_MAX_INCREASE       3       // Maximum DPI increase factor  
#define FALLBACK_MAX_ACCEL_LIMIT    5       // Maximum acceleration limit
#define CONSERVATIVE_FALLBACK_MULTIPLIER 2  // Conservative fallback multiplier

// Speed calculation (optimized for interrupts)
#define SPEED_TIME_LIMIT_MS     500     // Reduced for responsiveness
#define SPEED_ALPHA             250     // Simplified averaging
#define SPEED_BASE              1000    // Scaling base

// Utility calculation constants
#define QUADRATIC_SAFE_INPUT_LIMIT  1000    // Safe input limit for quadratic calculations
#define QUADRATIC_LINEAR_DIVISOR    10      // Divisor for linear approximation
#define QUADRATIC_SCALE_DIVISOR     100     // Scale divisor for quadratic results
#define LOG_COUNTER_INTERVAL        200     // Interval for debug logging

// Memory pool alignment
#define ACCEL_DATA_POOL_ALIGNMENT   4       // Memory pool alignment in bytes

// Default values
#define DEFAULT_SPEED_THRESHOLD     600     // Default speed threshold
#define DEFAULT_SPEED_MAX_OFFSET    1000    // Default offset for speed max calculation

// DPI calculation constants
#define STANDARD_DPI_REFERENCE  800     // Reference DPI for normalization
#define MAX_SENSOR_DPI          8000    // Maximum supported sensor DPI

// Exponential curve calculation constants
#define CURVE_MILD_DIVISOR      2000ULL    // Divisor for mild exponential curve
#define CURVE_MODERATE_QUAD_DIV 1000ULL    // Quadratic divisor for moderate curve
#define CURVE_MODERATE_CUBIC_DIV 3000000ULL // Cubic divisor for moderate curve
#define CURVE_STRONG_QUAD_DIV   800ULL     // Quadratic divisor for strong curve
#define CURVE_STRONG_CUBIC_DIV  2000000ULL // Cubic divisor for strong curve
#define CURVE_AGGRESSIVE_QUAD_DIV 600ULL   // Quadratic divisor for aggressive curve
#define CURVE_AGGRESSIVE_CUBIC_DIV 1500000ULL // Cubic divisor for aggressive curve
#define CURVE_DEFAULT_DIVISOR   1000ULL    // Default curve divisor

// Calculation scaling constants
#define SENSITIVITY_SCALE       1000    // Sensitivity scaling factor
#define SPEED_NORMALIZATION     1000    // Speed normalization factor
#define LINEAR_CURVE_MULTIPLIER 10ULL   // Linear curve multiplication factor (reduced for proper scaling)

// Essential utility macros (optimized for MCU)
#define ACCEL_CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))


// Error severity levels for acceleration processor
// Level 1: Minor issues (processing continues with fallback)
#define ACCEL_ERR_MINOR_BASE        -1
#define ACCEL_ERR_TEMP_UNAVAIL      -EAGAIN     // Temporary issue, retry possible
#define ACCEL_ERR_NO_DATA           -ENODATA    // No data available (normal)

// Level 2: Warning issues (processing continues with alternative)
#define ACCEL_ERR_WARNING_BASE      -10
#define ACCEL_ERR_OUT_OF_RANGE      -ERANGE     // Value out of range (clamped)
#define ACCEL_ERR_OVERFLOW          -EOVERFLOW  // Calculation overflow (limited)
#define ACCEL_ERR_NOT_SUPPORTED     -ENOTSUP    // Feature not supported (fallback)

// Level 3: Error issues (processing aborted)
#define ACCEL_ERR_ERROR_BASE        -20
#define ACCEL_ERR_INVALID_ARG       -EINVAL     // Invalid argument
#define ACCEL_ERR_NO_DEVICE         -ENODEV     // Device not available
#define ACCEL_ERR_NO_MEMORY         -ENOMEM     // Memory allocation failed

// Level 4: Critical issues (system protection)
#define ACCEL_ERR_CRITICAL_BASE     -30
#define ACCEL_ERR_MEMORY_FAULT      -EFAULT     // Memory access violation
#define ACCEL_ERR_NO_SYSTEM         -ENOSYS     // System function unavailable
#define ACCEL_ERR_NO_PERMISSION     -EPERM      // Permission denied

// Simplified speed calculation constants
#define ACCEL_MAX_SPEED_SAMPLES     8       // Maximum speed samples for averaging
#define ACCEL_SPEED_SCALE_FACTOR    10      // Speed scaling factor (simpler than 1000)

// Backward compatibility
#ifndef CLAMP
#define CLAMP(val, min, max) ACCEL_CLAMP(val, min, max)
#endif

// =============================================================================
// MEMORY POOL OPTIMIZATION
// =============================================================================

// Memory pool for acceleration data - reduces heap fragmentation
#define ACCEL_MAX_INSTANCES 4

// =============================================================================
// DATA STRUCTURES - ULTRA-OPTIMIZED FOR MCU
// =============================================================================

/**
 * @brief Level-specific configuration union for embedded systems
 */
union accel_level_config {
    struct {
        uint16_t sensitivity;      // Base sensitivity
        uint16_t max_factor;       // Max acceleration
        uint8_t curve_type;        // Curve type (0-2)
        uint8_t reserved;          // Padding
    } level1;                      // 6 bytes
    
    struct {
        uint16_t sensitivity;      // Base sensitivity
        uint16_t max_factor;       // Max acceleration  
        uint16_t min_factor;       // Min acceleration
        uint16_t speed_threshold;  // Speed threshold
        uint16_t speed_max;        // Speed maximum
        uint8_t acceleration_exponent; // Curve exponent
        uint8_t reserved;          // Padding
    } level2;                      // 10 bytes
} __packed;

/**
 * @brief Compact acceleration data structure for embedded systems - 6 bytes
 * Optimized for keyboard/pointing devices with limited RAM
 */
struct accel_data {
    uint32_t last_time_ms;         // Last event timestamp
    uint16_t recent_speed;         // Moving average speed
} __packed;

/**
 * @brief Embedded-optimized configuration structure - 16 bytes  
 * Reduced from 20 bytes for better memory efficiency
 */
struct accel_config {
    const uint16_t *codes;         // Input codes (4 bytes)
    uint32_t codes_count;          // Number of codes (4 bytes)
    union accel_level_config cfg;  // Level config (max 10 bytes -> 8 bytes optimized)
    uint8_t y_boost_scaled;        // Y boost (1 byte)
    uint8_t sensor_dpi_class;      // DPI class (1 byte) 
    uint8_t input_type;            // Event type (1 byte)
    uint8_t level;                 // Config level (1 byte)
} __packed;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================

/**
 * @brief Decode scaled configuration values (simplified for embedded)
 */
static inline uint16_t accel_decode_y_boost(uint8_t scaled) {
    return 1000 + (scaled * 10); // Simple linear mapping
}

static inline uint16_t accel_decode_sensor_dpi(uint8_t dpi_class) {
    static const uint16_t dpi_table[] = {400, 800, 1200, 1600, 3200, 6400};
    return (dpi_class < 6) ? dpi_table[dpi_class] : 800; // Default to 800 DPI
}

#ifdef __cplusplus
}
#endif

// Forward declarations for embedded helper functions  
int32_t validate_and_clamp_input(int32_t input_value);
int32_t safe_multiply_embedded(int32_t a, int32_t b);
uint16_t calculate_speed_embedded(struct accel_data *data, int32_t input_value);
uint16_t get_acceleration_factor(int32_t abs_input, uint8_t curve_type, uint16_t max_factor);

// Utility functions (from utils.c)
uint32_t accel_safe_quadratic_curve(int32_t abs_input, uint32_t multiplier);
int32_t accel_safe_fallback_calculate(int32_t input_value, uint32_t max_factor);
uint32_t accel_calculate_simple_speed(struct accel_data *data, int32_t input_value);

// Common calculation functions (from calc_common.c)
uint32_t calculate_dpi_adjusted_sensitivity(const struct accel_config *cfg);
int64_t safe_multiply_64(int64_t a, int64_t b, int64_t max_result);
int32_t safe_int64_to_int32(int64_t value);
int16_t safe_int32_to_int16(int32_t value);

#if defined(CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL_STANDARD)
uint32_t calculate_exponential_curve(uint32_t t, uint8_t exponent);
#endif

/**
 * @brief Main acceleration calculation functions (level-specific)
 */
int32_t accel_simple_calculate(const struct accel_config *cfg, int32_t input_value, uint16_t code);
int32_t accel_standard_calculate(const struct accel_config *cfg, struct accel_data *data, 
                                int32_t input_value, uint16_t code);

/**
 * @brief Main event handler (optimized for embedded keyboards)
 */
int accel_handle_event(const struct device *dev, struct input_event *event,
                      uint32_t param1, uint32_t param2,
                      struct zmk_input_processor_state *state);

/**
 * @brief Configuration and validation functions
 */
int accel_validate_config(const struct accel_config *cfg);
struct accel_data *accel_data_alloc(void);
void accel_data_free(struct accel_data *data);

// Configuration functions from presets
void accel_config_apply_kconfig_preset(struct accel_config *cfg);
int accel_config_init(struct accel_config *cfg, uint8_t level, int inst);

// Encode/decode functions (external implementation in config.c - avoid inline duplicate)
uint8_t accel_encode_y_boost(uint16_t y_boost);
uint8_t accel_encode_sensor_dpi(uint16_t sensor_dpi);

// Device initialization functions  
int accel_device_init_instance(const struct device *dev, int inst);

// Static memory pool (defined in main.c)
extern struct k_mem_slab accel_data_pool;