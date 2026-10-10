// accel_validation.c - Minimal validation for MCU efficiency
// Only essential checks to prevent system crashes

#include <zephyr/logging/log.h>
#include "../../include/drivers/input_processor_accel.h"

LOG_MODULE_DECLARE(input_processor_accel);

// =============================================================================
// MINIMAL VALIDATION FUNCTIONS
// =============================================================================

int accel_validate_config(const struct accel_config *cfg) {

    if (!cfg) {
        LOG_ERR("Configuration pointer is NULL");
        return ACCEL_ERR_INVALID_ARG;
    }

    // Validate configuration level
    if (cfg->level < 1 || cfg->level > 2) {
        LOG_ERR("Invalid configuration level: %u (must be 1 or 2)", cfg->level);
        return ACCEL_ERR_INVALID_ARG;
    }

    // Validate DPI class with enhanced bounds checking
    if (cfg->sensor_dpi_class > 7) { // Allow up to 7 (8 total entries in table)
        LOG_ERR("Invalid sensor DPI class: %u (must be 0-7)", cfg->sensor_dpi_class);
        return ACCEL_ERR_INVALID_ARG;
    }
    
    uint16_t sensor_dpi = accel_decode_sensor_dpi(cfg->sensor_dpi_class);
    if (sensor_dpi < 400 || sensor_dpi > MAX_SENSOR_DPI) {
        LOG_ERR("Sensor DPI %u out of reasonable range (400-%u)", sensor_dpi, MAX_SENSOR_DPI);
        return ACCEL_ERR_OUT_OF_RANGE;
    }
    
    // Prevent extreme values that could cause overflow
    uint16_t max_factor = (cfg->level == 1) ? cfg->cfg.level1.max_factor : cfg->cfg.level2.max_factor;
    if (max_factor > MAX_SAFE_FACTOR) {
        LOG_ERR("Max factor %u exceeds safe limit %u", max_factor, MAX_SAFE_FACTOR);
        return ACCEL_ERR_OVERFLOW;
    }
    if (max_factor < 1000) {
        LOG_ERR("Max factor %u below minimum 1000 (1.0x)", max_factor);
        return ACCEL_ERR_OUT_OF_RANGE;
    }
    
    // Validate sensitivity range
    uint16_t sensitivity = (cfg->level == 1) ? cfg->cfg.level1.sensitivity : cfg->cfg.level2.sensitivity;
    if (sensitivity > MAX_SAFE_SENSITIVITY) {
        LOG_ERR("Sensitivity %u exceeds safe limit %u", sensitivity, MAX_SAFE_SENSITIVITY);
        return ACCEL_ERR_OVERFLOW;
    }
    if (sensitivity < MIN_SAFE_SENSITIVITY) {
        LOG_ERR("Sensitivity %u below minimum %u", sensitivity, MIN_SAFE_SENSITIVITY);
        return ACCEL_ERR_OUT_OF_RANGE;
    }
    
    // Validate curve type (Level 1 only)
    if (cfg->level == 1 && cfg->cfg.level1.curve_type > 2) {
        LOG_ERR("Invalid curve type: %u (must be 0-2)", cfg->cfg.level1.curve_type);
        return ACCEL_ERR_INVALID_ARG;
    }
    
    // Validate Y-axis boost (common to both levels)
    uint16_t y_boost = accel_decode_y_boost(cfg->y_boost_scaled);
    if (y_boost < 500 || y_boost > 3000) {
        LOG_ERR("Y-axis boost %u out of reasonable range (500-3000)", y_boost);
        return ACCEL_ERR_OUT_OF_RANGE;
    }
    
    // Level-specific validation
    if (cfg->level == 1) {
        LOG_DBG("Level 1 (Simple) validation: basic checks only");
        // Level 1 uses basic validation only
        // speed_threshold, speed_max, min_factor, acceleration_exponent are not used
        
    } else if (cfg->level == 2) {
        LOG_DBG("Level 2 (Standard) validation: comprehensive checks");
        
        // Prevent division by zero in speed calculation
        if (cfg->cfg.level2.speed_max <= cfg->cfg.level2.speed_threshold) {
            LOG_ERR("Speed max (%u) must be greater than speed threshold (%u)", 
                    cfg->cfg.level2.speed_max, cfg->cfg.level2.speed_threshold);
            return ACCEL_ERR_INVALID_ARG;
        }
        
        // Validate speed ranges
        if (cfg->cfg.level2.speed_threshold > MAX_REASONABLE_SPEED) {
            LOG_ERR("Speed threshold %u exceeds reasonable limit %u", 
                    cfg->cfg.level2.speed_threshold, MAX_REASONABLE_SPEED);
            return ACCEL_ERR_OUT_OF_RANGE;
        }
        if (cfg->cfg.level2.speed_max > MAX_REASONABLE_SPEED) {
            LOG_ERR("Speed max %u exceeds reasonable limit %u", 
                    cfg->cfg.level2.speed_max, MAX_REASONABLE_SPEED);
            return ACCEL_ERR_OUT_OF_RANGE;
        }
        
        // Prevent invalid factor relationship
        if (cfg->cfg.level2.min_factor > cfg->cfg.level2.max_factor) {
            LOG_ERR("Min factor (%u) cannot be greater than max factor (%u)", 
                    cfg->cfg.level2.min_factor, cfg->cfg.level2.max_factor);
            return ACCEL_ERR_INVALID_ARG;
        }
        
        // Validate factor ranges
        if (cfg->cfg.level2.min_factor < 200 || cfg->cfg.level2.min_factor > 2000) {
            LOG_ERR("Min factor %u out of reasonable range (200-2000)", cfg->cfg.level2.min_factor);
            return ACCEL_ERR_OUT_OF_RANGE;
        }
        
        // Validate acceleration exponent
        if (cfg->cfg.level2.acceleration_exponent < 1 || cfg->cfg.level2.acceleration_exponent > 5) {
            LOG_ERR("Acceleration exponent %u out of valid range (1-5)", cfg->cfg.level2.acceleration_exponent);
            return ACCEL_ERR_INVALID_ARG;
        }
    }
    
    // Logical consistency checks
    if (max_factor <= sensitivity) {
        LOG_WRN("Max factor (%u) should typically be greater than sensitivity (%u)", 
                max_factor, sensitivity);
    }
    
    // Enhanced safety: Check for extreme value combinations that could cause overflow
    // Note: sensor_dpi already declared above, reuse it
    uint64_t overflow_check = (uint64_t)sensitivity * max_factor * sensor_dpi;
    if (overflow_check > (UINT64_MAX / 4000)) { // Conservative limit
        LOG_WRN("Extreme configuration detected: sensitivity=%u, max_factor=%u, dpi=%u", 
                sensitivity, max_factor, sensor_dpi);
        LOG_WRN("This combination may cause calculation overflow in extreme cases");
    }
    
    LOG_DBG("Configuration validation passed for level %u", cfg->level);
    return 0;
}