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

/**
 * @file    ble_ios.h
 *
 * Input/Outputサービス
 */
#ifndef BLE_IOS_H__
#define BLE_IOS_H__

/**************************************************************************
 * include
 **************************************************************************/

#include "ble.h"


/**************************************************************************
 * macro
 **************************************************************************/

//87C9xxxx-CBA0-7D7D-F1B5-E1635787F177
//                                                                                  xxxxxxxxx
#define IOS_UUID_BASE { 0x77,0xf1,0x87,0x57,0x63,0xe1,0xb5,0xf1,0x7d,0x7d,0xa0,0xcb,0x00,0x00,0xc9,0x87 }
#define IOS_UUID_SERVICE        (0x0001)
#define IOS_UUID_CHAR_INPUT     (0x0002)
#define IOS_UUID_CHAR_OUTPUT    (0x0003)


/**************************************************************************
 * definition
 **************************************************************************/

// Forward declaration of the ble_ios_t type.
typedef struct ble_ios_s ble_ios_t;


/**
 * @brief サービスイベントハンドラ
 *
 * @param[in]   p_ios   I/Oサービス構造体
 * @param[in]   p_value 受信バッファ
 * @param[in]   length  受信データ長
 */
typedef void (*ble_ios_evt_handler_t) (ble_ios_t *p_ios, const uint8_t *p_value, uint16_t length);


/**@brief サービス初期化構造体 */
typedef struct {
    ble_ios_evt_handler_t           evt_handler_in;             /**< イベントハンドラ : Input Notify発生 */
    uint16_t                        len_in;                     /**< Inputデータ長 */
    uint16_t                        len_out;                    /**< Outputデータ長 */
} ble_ios_init_t;


/**@brief サービス構造体 */
typedef struct ble_ios_s {
    uint16_t                        service_handle;             /**< Handle of I/O Service (as provided by the BLE stack). */
    uint16_t                        conn_handle;                /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                         uuid_type;
    //
    ble_gatts_char_handles_t        char_handle_in;             /**< Handles related to the Input characteristic. */
    ble_ios_evt_handler_t           evt_handler_in;             /**< Event handler to be called for handling events in the I/O Service. */
    //
    ble_gatts_char_handles_t        char_handle_out;            /**< Handles related to the Output characteristic. */
} ble_ios_t;


/**************************************************************************
 * prototype
 **************************************************************************/

/**@brief サービス初期化
 *
 * @param[in]   p_ios       サービス構造体
 * @param[in]   p_ios_init  サービス初期化構造体
 * @retval      NRF_SUCCESS 成功
 */
uint32_t ble_ios_init(ble_ios_t *p_ios, const ble_ios_init_t *p_ios_init);


/**@brief BLEイベントハンドラ
 * アプリ層のBLEイベントハンドラから呼び出されることを想定している.
 *
 * @param[in]   p_ios       サービス構造体
 * @param[in]   p_ble_evt   イベント構造体
 */
void ble_ios_on_ble_evt(ble_ios_t *p_ios, ble_evt_t *p_ble_evt);


/**@brief Notify送信
 *
 * @param[in]   p_ios       サービス構造体
 * @param[in]   p_value     送信データバッファ
 * @param[in]   length      送信データサイズ
 * @retval      NRF_SUCCESS 成功
 */
uint32_t ble_ios_on_output(ble_ios_t *p_ios, const uint8_t *p_value, uint16_t length);

#endif // BLE_IOS_H__

