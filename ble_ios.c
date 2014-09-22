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

/* Attention!
*  To maintain compliance with Nordic Semiconductor ASA痴 Bluetooth profile
*  qualification listings, this section of source code must not be modified.
*/

#include "ble_ios.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_ios       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_ios_t *p_ios, ble_evt_t *p_ble_evt)
{
    p_ios->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_ios       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_ios_t *p_ios, ble_evt_t *p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_ios->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ios       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_ios_t *p_ios, ble_evt_t *p_ble_evt)
{
    ble_gatts_evt_write_t *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_ios->char_handle_in.value_handle) &&
      (p_evt_write->len == 1) &&
      (p_ios->evt_handler_in != NULL)) {
        p_ios->evt_handler_in(p_ios, p_evt_write->data[0]);
    }
}


void ble_ios_on_ble_evt(ble_ios_t *p_ios, ble_evt_t *p_ble_evt)
{
    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        on_connect(p_ios, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        on_disconnect(p_ios, p_ble_evt);
        break;

    case BLE_GATTS_EVT_WRITE:
        on_write(p_ios, p_ble_evt);
        break;

    default:
        // No implementation needed.
        break;
    }
}


uint32_t ble_ios_on_output(ble_ios_t *p_ios, uint8_t value)
{
    //Notify
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(value);

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_ios->char_handle_out.value_handle;
    params.p_data = &value;
    params.p_len = &len;

    return sd_ble_gatts_hvx(p_ios->conn_handle, &params);
}


/*
 * write
 */
static uint32_t char_add_input(ble_ios_t *p_ios)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    ///////////////////////
    // キャラクタリスティックメタデータの設定

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write  = 1;
//    char_md.p_char_user_desc  = NULL;
//    char_md.p_char_pf         = NULL;
//    char_md.p_user_desc_md    = NULL;
//    char_md.p_cccd_md         = NULL;
//    char_md.p_sccd_md         = NULL;


    ///////////////////////
    // UUIDの設定

    ble_uuid.type = p_ios->uuid_type;
    ble_uuid.uuid = IOS_UUID_CHAR_INPUT;


    ///////////////////////
    // Attributeの設定

    memset(&attr_md, 0, sizeof(attr_md));

    //Write Only
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
//    attr_md.rd_auth    = 0;
//    attr_md.wr_auth    = 0;
//    attr_md.vlen       = 0;


    ///////////////////////
    // Attributeの設定

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;


    ///////////////////////
    // キャラクタリスティックの登録

    return sd_ble_gatts_characteristic_add(p_ios->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_ios->char_handle_in);
}


/*
 * read & notify
 *
 * CCCDは、NotifyかIndicateをサポートする場合必要
 */
static uint32_t char_add_output(ble_ios_t *p_ios)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    ///////////////////////
    // CCCDの設定(Notify用)

    memset(&cccd_md, 0, sizeof(cccd_md));

    //require no protection, open link.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;


    ///////////////////////
    // キャラクタリスティックメタデータの設定

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
//    char_md.p_char_user_desc  = NULL;
//    char_md.p_char_pf         = NULL;
//    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
//    char_md.p_sccd_md         = NULL;


    ///////////////////////
    // UUIDの設定

    ble_uuid.type = p_ios->uuid_type;
    ble_uuid.uuid = IOS_UUID_CHAR_OUTPUT;


    ///////////////////////
    // Attributeの設定

    memset(&attr_md, 0, sizeof(attr_md));

    //Read Only
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
//    attr_md.rd_auth    = 0;
//    attr_md.wr_auth    = 0;
//    attr_md.vlen       = 0;


    ///////////////////////
    // Attributeの設定

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;


    ///////////////////////
    // キャラクタリスティックの登録

    return sd_ble_gatts_characteristic_add(p_ios->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_ios->char_handle_out);
}


uint32_t ble_ios_init(ble_ios_t *p_ios, const ble_ios_init_t *p_ios_init)
{
    uint32_t   err_code;

    //ハンドラ
    p_ios->evt_handler_in   = p_ios_init->evt_handler_in;
    p_ios->conn_handle      = BLE_CONN_HANDLE_INVALID;

    //Base UUIDを登録し、UUID typeを取得
    ble_uuid128_t   base_uuid = { IOS_UUID_BASE };
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_ios->uuid_type);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    //サービス登録
    ble_uuid_t ble_uuid;
    ble_uuid.uuid = IOS_UUID_SERVICE;
    ble_uuid.type = p_ios->uuid_type;
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid, &p_ios->service_handle);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    //キャラクタリスティック登録
    err_code = char_add_input(p_ios);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }
    err_code = char_add_output(p_ios);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

    return NRF_SUCCESS;
}

