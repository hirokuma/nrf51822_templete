/*
 * Copyright (c) 2012-2014, hiro99ma
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *         this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *         this list of conditions and the following disclaimer
 *         in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

/** @file
 *
 * @defgroup ble_sdk_srv_bas Battery Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Battery Service module.
 *
 * @details This module implements the Battery Service with the Battery Level characteristic.
 *          During initialization it adds the Battery Service and Battery Level characteristic
 *          to the BLE stack database. Optionally it can also add a Report Reference descriptor
 *          to the Battery Level characteristic (used when including the Battery Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the Battery Level characteristic
 *          through the ble_ios_battery_level_update() function.
 *          If an event handler is supplied by the application, the Battery Service will
 *          generate Battery Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Battery Service module by calling
 *       ble_ios_on_ble_evt() from the from the @ref ble_stack_handler callback.
 *
 * @note Attention!
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_IOS_H__
#define BLE_IOS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

//87C9xxxx-CBA0-7D7D-F1B5-E1635787F177
//                                                                                  xxxxxxxxx
#define IOS_UUID_BASE { 0x77,0xf1,0x87,0x57,0x63,0xe1,0xb5,0xf1,0x7d,0x7d,0xa0,0xcb,0x00,0x00,0xc9,0x87 }
#define IOS_UUID_SERVICE        (0x0001)
#define IOS_UUID_CHAR_INPUT     (0x0002)
#define IOS_UUID_CHAR_OUTPUT    (0x0003)

/**@brief I/O Service event type. */
typedef enum {
    BLE_IOS_EVT_NOTIFICATION_ENABLED,   /**< notification enabled event. */
    BLE_IOS_EVT_NOTIFICATION_DISABLED   /**< notification disabled event. */
} ble_ios_evt_type_t;

/**@brief I/O Service event. */
typedef struct {
    ble_ios_evt_type_t evt_type;                                /**< Type of event. */
} ble_ios_evt_t;

// Forward declaration of the ble_ios_t type.
typedef struct ble_ios_s ble_ios_t;

/**@brief I/O Service event handler type. */
typedef void (*ble_ios_evt_handler_t) (ble_ios_t *p_ios, uint8_t value);

/**@brief I/O Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct {
    ble_ios_evt_handler_t           evt_handler_in;             /**< Event handler to be called for handling events in the I/O Service. */
} ble_ios_init_t;

/**@brief I/O Service structure. This contains various status information for the service. */
typedef struct ble_ios_s {
    uint16_t                        service_handle;             /**< Handle of I/O Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t        char_handle_in;             /**< Handles related to the Input characteristic. */
    ble_gatts_char_handles_t        char_handle_out;            /**< Handles related to the Output characteristic. */
    uint16_t                        conn_handle;                /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                         uuid_type;
    //
    ble_ios_evt_handler_t           evt_handler_in;             /**< Event handler to be called for handling events in the I/O Service. */
} ble_ios_t;

/**@brief Function for initializing the I/O Service.
 *
 * @param[out]  p_ios       I/O Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_ios_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_ios_init(ble_ios_t *p_ios, const ble_ios_init_t *p_ios_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the I/O Service.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       ble_ios_battery_level_update() must be called upon reconnection if the
 *       battery level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_ios      I/O Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_ios_on_ble_evt(ble_ios_t *p_ios, ble_evt_t *p_ble_evt);


uint32_t ble_ios_on_output(ble_ios_t *p_ios, uint8_t value);

#endif // BLE_IOS_H__

/** @} */
