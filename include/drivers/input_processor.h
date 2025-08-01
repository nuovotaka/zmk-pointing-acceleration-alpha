/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

/**
 * @brief ZMK Input Processor State
 * 
 * State structure passed to input processor event handlers
 */
struct zmk_input_processor_state {
    /* Add state fields as needed */
    void *user_data;
};

/**
 * @brief Input Processor Event Handler
 * 
 * @param dev The input processor device
 * @param event The input event to process
 * @param param1 First parameter (usage depends on processor)
 * @param param2 Second parameter (usage depends on processor)
 * @param state Processor state
 * @return 0 on success, negative errno on failure
 */
typedef int (*zmk_input_processor_event_handler_t)(const struct device *dev,
                                                   struct input_event *event,
                                                   uint32_t param1,
                                                   uint32_t param2,
                                                   struct zmk_input_processor_state *state);

/**
 * @brief ZMK Input Processor Driver API
 */
struct zmk_input_processor_driver_api {
    int (*handle_event)(const struct device *dev,
                        struct input_event *event,
                        uint32_t param1,
                        uint32_t param2,
                        struct zmk_input_processor_state *state);
};

#ifdef __cplusplus
}
#endif
