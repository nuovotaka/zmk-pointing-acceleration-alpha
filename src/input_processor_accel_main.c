// input_processor_accel_main.c - ZMK Input Processor for Pointing Device Acceleration
// Main event handler and device registration
// 
// Copyright (c) 2024 The ZMK Contributors
// Modifications (c) 2025 NUOVOTAKA
// SPDX-License-Identifier: MIT

#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/input_processor.h>
#include "../include/drivers/input_processor_accel.h"
#include "config/accel_config.h"
#include "config/accel_device_init.h"

LOG_MODULE_REGISTER(input_processor_accel, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_input_processor_acceleration

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

// =============================================================================
// MEMORY POOL DEFINITION
// =============================================================================

// Define the memory pool for acceleration data (only once in main.c)
K_MEM_SLAB_DEFINE(accel_data_pool, sizeof(struct accel_data), ACCEL_MAX_INSTANCES, ACCEL_DATA_POOL_ALIGNMENT);

// =============================================================================
// DEVICE INITIALIZATION
// =============================================================================

static int accel_init_device(const struct device *dev) {
    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;

    // Validate final configuration
    int ret = accel_validate_config(cfg);
    if (ret < 0) {
        LOG_ERR("Device %s: Configuration validation failed: %d", dev->name, ret);
        return ret; // Pass through validation error code
    }
    
    // Enhanced NULL pointer validation and initialization
    if (!data) {
        LOG_ERR("Device %s: Data structure is NULL", dev->name ? dev->name : "unknown");
        return ACCEL_ERR_NO_MEMORY;
    }
    
    // Initialize runtime data structures - ensure proper memory initialization
    memset(data, 0, sizeof(struct accel_data));
    // Initialize timing data to prevent division by zero
    data->last_time_ms = k_uptime_get_32();
    data->recent_speed = 0;
    
    LOG_INF("Device %s: Acceleration processor ready (Level %d)", dev->name, cfg->level);
    return 0;
}

// =============================================================================
// DEVICE INSTANCE CREATION USING DT_INST_FOREACH_STATUS_OKAY
// =============================================================================

// Macro to create device instance initialization function
#define ACCEL_INIT_FUNC(inst)                                                                     \
    static int accel_init_##inst(const struct device *dev) {                                     \
        /* Initialize device instance configuration */                                           \
        int ret = accel_device_init_instance(dev, inst);                                        \
        if (ret < 0) {                                                                           \
            return ret;                                                                          \
        }                                                                                        \
                                                                                                  \
        /* Apply DT custom properties if enabled (must be done in macro context) */            \
        struct accel_config *cfg = (struct accel_config *)dev->config;                          \
        bool use_custom_config = IS_ENABLED(CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_CUSTOM);       \
        if (use_custom_config) {                                                                  \
            /* Apply common DTS properties for both levels */                                   \
            if (cfg->level == 1) { \
                cfg->cfg.level1.sensitivity = ACCEL_CLAMP(DT_INST_PROP_OR(inst, sensitivity, cfg->cfg.level1.sensitivity), SENSITIVITY_MIN, SENSITIVITY_MAX); \
                cfg->cfg.level1.max_factor = ACCEL_CLAMP(DT_INST_PROP_OR(inst, max_factor, cfg->cfg.level1.max_factor), MAX_FACTOR_MIN, MAX_FACTOR_MAX); \
                cfg->cfg.level1.curve_type = ACCEL_CLAMP(DT_INST_PROP_OR(inst, curve_type, cfg->cfg.level1.curve_type), CURVE_TYPE_MIN, CURVE_TYPE_MAX); \
            } else { \
                cfg->cfg.level2.sensitivity = ACCEL_CLAMP(DT_INST_PROP_OR(inst, sensitivity, cfg->cfg.level2.sensitivity), SENSITIVITY_MIN, SENSITIVITY_MAX); \
                cfg->cfg.level2.max_factor = ACCEL_CLAMP(DT_INST_PROP_OR(inst, max_factor, cfg->cfg.level2.max_factor), MAX_FACTOR_MIN, MAX_FACTOR_MAX); \
            } \
            cfg->y_boost_scaled = ACCEL_CLAMP((DT_INST_PROP_OR(inst, y_boost, SENSITIVITY_SCALE) - SENSITIVITY_SCALE) / 10, 0, 200); \
            uint16_t dpi = ACCEL_CLAMP(DT_INST_PROP_OR(inst, sensor_dpi, STANDARD_DPI_REFERENCE), SENSOR_DPI_MIN, SENSOR_DPI_MAX); \
            cfg->sensor_dpi_class = (dpi <= 400) ? 0 : (dpi <= 800) ? 1 : (dpi <= 1200) ? 2 : (dpi <= 1600) ? 3 : (dpi <= 3200) ? 4 : (dpi <= 6400) ? 5 : 6; \
                                                                                                  \
            /* Apply Level 2 specific DTS properties only for Standard level */                \
            if (cfg->level == 2) {                                                              \
                cfg->cfg.level2.speed_threshold = ACCEL_CLAMP(DT_INST_PROP_OR(inst, speed_threshold, cfg->cfg.level2.speed_threshold), SPEED_THRESHOLD_MIN, SPEED_THRESHOLD_MAX); \
                cfg->cfg.level2.speed_max = ACCEL_CLAMP(DT_INST_PROP_OR(inst, speed_max, cfg->cfg.level2.speed_max), SPEED_MAX_MIN, SPEED_MAX_MAX); \
                cfg->cfg.level2.min_factor = ACCEL_CLAMP(DT_INST_PROP_OR(inst, min_factor, cfg->cfg.level2.min_factor), MIN_FACTOR_MIN, MIN_FACTOR_MAX); \
                cfg->cfg.level2.acceleration_exponent = ACCEL_CLAMP(DT_INST_PROP_OR(inst, acceleration_exponent, cfg->cfg.level2.acceleration_exponent), ACCEL_EXPONENT_MIN, ACCEL_EXPONENT_MAX); \
            }                                                                                    \
        }                                                                                        \
                                                                                                  \
        /* Final device initialization and validation */                                        \
        return accel_init_device(dev);                                                          \
    }

// Macro to create device instance data and config structures
#define ACCEL_DEVICE_DEFINE(inst)                                                               \
    static struct accel_data accel_data_##inst = {0};                                          \
    static struct accel_config accel_config_##inst = {0};                                      \
    ACCEL_INIT_FUNC(inst)                                                                      \
    DEVICE_DT_INST_DEFINE(inst,                                                                \
                          accel_init_##inst,                                                    \
                          NULL,                                                                 \
                          &accel_data_##inst,                                                   \
                          &accel_config_##inst,                                                 \
                          POST_KERNEL,                                                          \
                          CONFIG_INPUT_PROCESSOR_ACCELERATION_INIT_PRIORITY,                    \
                          &(const struct zmk_input_processor_driver_api){                      \
                              .handle_event = accel_handle_event                                \
                          });

// Create device instances for all enabled DT nodes
DT_INST_FOREACH_STATUS_OKAY(ACCEL_DEVICE_DEFINE)



// =============================================================================
// MAIN EVENT HANDLER
// =============================================================================

// =============================================================================
// OPTIMIZED FAST-PATH EVENT HANDLER
// =============================================================================



int accel_handle_event(const struct device *dev, struct input_event *event,
                      uint32_t param1, uint32_t param2,
                      struct zmk_input_processor_state *state) {
    // CRITICAL: Minimize interrupt processing time
    // Enhanced NULL pointer validation with proper error reporting
    if (!dev) {
        LOG_ERR("Device pointer is NULL in event handler");
        return ACCEL_ERR_INVALID_ARG;
    }
    if (!event) {
        LOG_ERR("Event pointer is NULL in event handler");
        return ACCEL_ERR_INVALID_ARG;
    }
    if (!dev->config) {
        LOG_ERR("Device config is NULL for device %s", dev->name ? dev->name : "unknown");
        return ACCEL_ERR_NO_DEVICE;
    }
    if (!dev->data) {
        LOG_ERR("Device data is NULL for device %s", dev->name ? dev->name : "unknown");
        return ACCEL_ERR_NO_DEVICE;
    }
    
    const struct accel_config *cfg = dev->config;
    struct accel_data *data = dev->data;
    
    // Fast path checks - optimized for common cases with clear logic
    // Check event type first
    if (event->type != cfg->input_type) {
        return ZMK_INPUT_PROC_CONTINUE; // Wrong event type, continue processing
    }
    
    // Check for supported axis codes (movement + scroll)
    if (event->code != INPUT_REL_X && 
        event->code != INPUT_REL_Y && 
        event->code != INPUT_REL_WHEEL && 
        event->code != INPUT_REL_HWHEEL) {
        return ZMK_INPUT_PROC_CONTINUE; // Unsupported axis, continue processing
    }
    
    // Check for zero movement (no acceleration needed)
    if (event->value == 0) {
        return ZMK_INPUT_PROC_CONTINUE; // No movement to accelerate, continue processing
    }
    
    // Skip expensive validation in interrupt context
    // (validation done at initialization time)
    
    // Fast input clamping
    int32_t input_value = accel_clamp_input_value(event->value);

    // OPTIMIZED: Fast-path processing with minimal overhead
    int32_t accelerated_value;
    
    // Ultra-fast calculation dispatch - branch prediction optimized
    if (cfg->level == 1) {
        // Level 1: Use simple calculation from dedicated file
        accelerated_value = accel_simple_calculate(cfg, input_value, event->code);
    } else {
        // Level 2: Use standard calculation from dedicated file
        accelerated_value = accel_standard_calculate(cfg, data, input_value, event->code);
    }
    
    // Minimal safety check - emergency brake only
    if (__builtin_expect(abs(accelerated_value) > EMERGENCY_BRAKE_THRESHOLD, 0)) {
        // Unlikely path - extreme values
        accelerated_value = (accelerated_value > 0) ? EMERGENCY_BRAKE_LIMIT : -EMERGENCY_BRAKE_LIMIT;
    }
    
    // Minimum movement guarantee - optimized
    if (__builtin_expect(input_value != 0 && accelerated_value == 0, 0)) {
        accelerated_value = (input_value > 0) ? 1 : -1;
    }
    
    // Final safety validation before updating event (no logging in interrupt context)
    if (__builtin_expect(abs(accelerated_value) > INT16_MAX, 0)) {
        // Emergency clamp without logging to maintain real-time performance
        accelerated_value = (accelerated_value > 0) ? INT16_MAX : INT16_MIN;
    }
    
    // Update event value - single assignment with final validation
    event->value = accelerated_value;
    
    return ZMK_INPUT_PROC_CONTINUE;
}

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)