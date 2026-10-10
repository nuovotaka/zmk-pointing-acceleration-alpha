// accel_config.c - Unified configuration management with memory optimization
// Centralizes all configuration logic and reduces complexity
//
// Copyright (c) 2024 The ZMK Contributors
// Modifications (c) 2025 NUOVOTAKA
// SPDX-License-Identifier: MIT

#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include "accel_config.h"

LOG_MODULE_DECLARE(input_processor_accel);

// =============================================================================
// MEMORY POOL MANAGEMENT
// =============================================================================

struct accel_data *accel_data_alloc(void) {
    struct accel_data *data;
    int ret = k_mem_slab_alloc(&accel_data_pool, (void **)&data, K_NO_WAIT);
    if (ret == 0 && data != NULL) {
        // Enhanced safety: Validate allocated pointer before use
        memset(data, 0, sizeof(struct accel_data));
        // Initialize with safe default values to prevent issues
        data->last_time_ms = k_uptime_get_32();
        data->recent_speed = 0;
        LOG_DBG("Allocated accel_data from pool: %p", data);
        return data;
    }
    
    // Enhanced error handling with acceleration-specific error codes
    if (ret == -ENOMEM) {
        LOG_ERR("Memory pool exhausted - no available accel_data blocks");
        // Note: Keep original -ENOMEM for k_mem_slab compatibility, but could map to ACCEL_ERR_NO_MEMORY
    } else if (ret == -EAGAIN) {
        LOG_ERR("Memory allocation would block - called from interrupt context");
        // Note: Keep original -EAGAIN for k_mem_slab compatibility, but could map to ACCEL_ERR_TEMP_UNAVAIL
    } else {
        LOG_ERR("Failed to allocate accel_data from pool: %d (ptr=%p)", ret, data);
    }
    
    return NULL;
}

void accel_data_free(struct accel_data *data) {
    if (data) {
        k_mem_slab_free(&accel_data_pool, (void *)data);
        LOG_DBG("Freed accel_data to pool: %p", data);
    }
}

// =============================================================================
// CONFIGURATION ENCODING/DECODING HELPERS
// =============================================================================

uint8_t accel_encode_y_boost(uint16_t y_boost) {
    // Encode 1000-3000 range to 0-200
    if (y_boost < 1000) return 0;
    if (y_boost > 3000) return 200;
    return (uint8_t)((y_boost - 1000) / 10);
}

uint8_t accel_encode_sensor_dpi(uint16_t sensor_dpi) {
    // Encode common DPI values to classes
    if (sensor_dpi <= 400) return 0;
    if (sensor_dpi <= 800) return 1;
    if (sensor_dpi <= 1200) return 2;
    if (sensor_dpi <= 1600) return 3;
    if (sensor_dpi <= 3200) return 4;
    if (sensor_dpi <= 6400) return 5;
    if (sensor_dpi <= 8000) return 6;
    return 1; // Default to 800 DPI class
}

// =============================================================================
// DEFAULT CONFIGURATIONS
// =============================================================================

static const struct accel_config level1_defaults = {
    .level = 1,
    .input_type = INPUT_EV_REL,
    .y_boost_scaled = 0,       // 1.0x (no Y-axis boost)
    .sensor_dpi_class = 1,     // 800 DPI class
    .cfg.level1 = {
        .sensitivity = 1000,   // 1.0x (neutral base sensitivity)
        .max_factor = 2500,    // 2.5x (more noticeable acceleration)
        .curve_type = 1,       // Mild (smooth acceleration curve)
        .reserved = 0
    }
};

static const struct accel_config level2_defaults = {
    .level = 2,
    .input_type = INPUT_EV_REL,
    .y_boost_scaled = 0,       // 1.0x (no Y-axis boost by default)
    .sensor_dpi_class = 1,     // 800 DPI class
    .cfg.level2 = {
        .sensitivity = 1000,   // 1.0x (neutral base sensitivity)
        .max_factor = 2500,    // 2.5x (moderate acceleration for standard level)
        .min_factor = 900,     // 0.9x (slight precision boost for slow movements)
        .speed_threshold = 600, // Lower threshold for more responsive acceleration
        .speed_max = 3500,     // Higher maximum speed for standard level
        .acceleration_exponent = 2, // Mild exponential curve
        .reserved = 0
    }
};



// =============================================================================
// CONFIGURATION FUNCTIONS
// =============================================================================

const struct accel_config *accel_config_get_defaults(uint8_t level) {
    switch (level) {
        case 1: 
            LOG_DBG("Using Level 1 (Simple) defaults");
            return &level1_defaults;
        case 2: 
            LOG_DBG("Using Level 2 (Standard) defaults");
            return &level2_defaults;
        default:
            LOG_ERR("Invalid configuration level: %u, falling back to Level 1", level);
            return &level1_defaults;
    }
}

int accel_config_init(struct accel_config *cfg, uint8_t level, int inst) {
    if (!cfg) {
        LOG_ERR("Configuration pointer is NULL");
        return ACCEL_ERR_INVALID_ARG;
    }

    if (level < 1 || level > 2) {
        LOG_ERR("Invalid configuration level: %u (must be 1 or 2)", level);
        return ACCEL_ERR_INVALID_ARG;
    }

    LOG_INF("Initializing acceleration config: level=%u, instance=%d", level, inst);
    
    // Clear configuration structure first
    memset(cfg, 0, sizeof(struct accel_config));
    
    // Start with defaults for the specified level
    const struct accel_config *defaults = accel_config_get_defaults(level);
    if (!defaults) {
        LOG_ERR("Failed to get default configuration for level %u", level);
        return ACCEL_ERR_NO_DATA;
    }
    
    memcpy(cfg, defaults, sizeof(struct accel_config));
    
    // Ensure level is correctly set
    cfg->level = level;
    
    // Validate basic configuration based on level
    uint16_t sensitivity, max_factor;
    if (level == 1) {
        sensitivity = cfg->cfg.level1.sensitivity;
        max_factor = cfg->cfg.level1.max_factor;
    } else {
        sensitivity = cfg->cfg.level2.sensitivity;
        max_factor = cfg->cfg.level2.max_factor;
    }
    
    if (sensitivity == 0 || max_factor == 0) {
        LOG_ERR("Invalid default configuration: sensitivity=%u, max_factor=%u", 
                sensitivity, max_factor);
        return ACCEL_ERR_INVALID_ARG;
    }
    
    LOG_INF("Base configuration initialized: level=%u, max_factor=%u, sensitivity=%u", 
            cfg->level, max_factor, sensitivity);
    
    return 0;
}

// Configuration validation is handled in src/validation/accel_validation.c
// to avoid duplicate symbol errors