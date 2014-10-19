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

/**************************************************************************/
/* include                                                                */
/**************************************************************************/

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "app_scheduler.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"

#include "ble_ios.h"


/**************************************************************************/
/* macro                                                                  */
/**************************************************************************/

#define IS_SRVC_CHANGED_CHARACT_PRESENT (0)                                         /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define BUTTON_PIN_NO_APP               (19)                                        /**< Button used to wake up the application. */

#define LED_PIN_NO_ADVERTISING          (21)                                        /**< Is on when device is advertising. */
#define LED_PIN_NO_CONNECTED            (26)                                        /**< Is on when device has connected. */
#define LED_PIN_NO_ASSERT               (27)                                        /**< Is on when application has asserted. */
#define LED_PIN_NO_APP                  (16)

#define DEVICE_NAME                     "hiro99ma_Template"                         /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                (64)                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      (180)                                       /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             (0)                                         /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            (2)                                         /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         (4)                                         /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   (0)                                         /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    (3)                                         /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            (1)                                         /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               (30)                                        /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  (1)                                         /**< Perform bonding. */
#define SEC_PARAM_MITM                  (0)                                         /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   (0)                                         /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          (7)                                         /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          (16)                                        /**< Maximum encryption key size. */

#define DEAD_BEEF                       (0xDEADBEEF)                                /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                (10)                                        /**< Maximum number of events in the scheduler queue. */

#define ARRAY_SIZE(array)               (sizeof(array) / sizeof(array[0]))


/**************************************************************************/
/* declaration                                                            */
/**************************************************************************/

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_ios_t                        m_ios;


/**************************************************************************/
/* prototype                                                              */
/**************************************************************************/

static void sys_evt_dispatch(uint32_t sys_evt);

static void leds_init(void);
__attribute__( ( always_inline ) ) __STATIC_INLINE void led_on(int pin);
__attribute__( ( always_inline ) ) __STATIC_INLINE void led_off(int pin);

static void timers_init(void);
static void timers_start(void);

static void scheduler_init(void);

static void gpiote_init(void);
static void buttons_init(void);
static void button_event_handler(uint8_t pin_no, uint8_t button_event);

static void gap_params_init(void);

static void advertising_init(void);
static void advertising_start(void);

static void services_init(void);
static void services_ios_handler_in(ble_ios_t *p_ios, uint8_t value);

static void sec_params_init(void);

static void conn_params_init(void);
static void conn_params_evt_handler(ble_conn_params_evt_t * p_evt);
static void conn_params_error_handler(uint32_t nrf_error);

static void ble_evt_handler(ble_evt_t * p_ble_evt);
static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
static void ble_stack_init(void);


/**************************************************************************/
/* main entry                                                             */
/**************************************************************************/

/**@brief main
 */
int main(void)
{
    // 初期化
    leds_init();
    timers_init();

    gpiote_init();
    buttons_init();
    ble_stack_init();
    scheduler_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();

    // 処理開始
    //timers_start();
    advertising_start();
    while (1) {
        //スケジュール済みイベントの実行(mainloop内で呼び出す)
        app_sched_execute();
        uint32_t err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}


/**************************************************************************/
/* public function                                                        */
/**************************************************************************/

/**@brief エラーハンドラ
 *
 * APP_ERROR_HANDLER()やAPP_ERROR_CHECK()から呼び出される(app_error.h)。
 * 引数は見ず、ASSERT LEDを点灯させて処理を止める。
 *
 * @param[in] error_code  エラーコード
 * @param[in] line_num    エラー発生行(__LINE__など)
 * @param[in] p_file_name エラー発生ファイル(__FILE__など)
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name)
{
    //ASSERT LED点灯
    led_on(LED_PIN_NO_ASSERT);

#if 0
    //リセットによる再起動
    NVIC_SystemReset();
#else
    //留まる
    while(1) {
        __WFI();
    }
#endif
}


/**@brief SoftDeviceのASSERTハンドラ
 *
 * ASSERTした場合に呼び出される(nrf_assert.h)。
 * 条件は、DEBUG_NRFかDEBUG_NRF_USERが定義されている場合。
 * 定義されていない場合でもプロトタイプ宣言だけは残っている。
 *
 * と思ったら、softdevice_assertion_handler()が直接呼び出している(softdevice_handler.c)。
 * よって、この定義はなくすわけにはいかない。
 *
 * @param[in] line_num    エラー発生行(__LINE__など)
 * @param[in] p_file_name エラー発生ファイル(__FILE__など)
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**************************************************************************/
/* private function                                                       */
/**************************************************************************/

/**@brief システムイベント発生
 *
 * SoCでイベントが発生した場合にコールバックされる。
 *
 *
 * @param[in]   sys_evt   enum NRF_SOC_EVTS型(NRF_EVT_xxx). nrf_soc.hに定義がある.
 *      - NRF_EVT_HFCLKSTARTED
 *      - NRF_EVT_POWER_FAILURE_WARNING
 *      - NRF_EVT_FLASH_OPERATION_SUCCESS
 *      - NRF_EVT_FLASH_OPERATION_ERROR
 *      - NRF_EVT_RADIO_BLOCKED
 *      - NRF_EVT_RADIO_CANCELED
 *      - NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN
 *      - NRF_EVT_RADIO_SESSION_IDLE
 *      - NRF_EVT_RADIO_SESSION_CLOSED
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    //ここでは異常発生のように扱っているが、必ずそうではないようなので、
    //実装する際には注意しよう。
    app_error_handler(DEAD_BEEF, 0, NULL);
}


/**********************************************/
/* LED                                        */
/**********************************************/

/**@brief LED初期化
 *
 * 全部消灯させる。
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(LED_PIN_NO_ADVERTISING);
    nrf_gpio_cfg_output(LED_PIN_NO_CONNECTED);
    nrf_gpio_cfg_output(LED_PIN_NO_ASSERT);
    nrf_gpio_cfg_output(LED_PIN_NO_APP);

    led_off(LED_PIN_NO_ADVERTISING);
    led_off(LED_PIN_NO_CONNECTED);
    led_off(LED_PIN_NO_ASSERT);
    led_off(LED_PIN_NO_APP);
}


/**@brief LED点灯
 *
 * @param[in]   pin     対象PIN番号
 */
__attribute__( ( always_inline ) ) __STATIC_INLINE void led_on(int pin)
{
    /* アクティブLOW */
    nrf_gpio_pin_clear(pin);
}


/**@brief LED消灯
 *
 * @param[in]   pin     対象PIN番号
 */
__attribute__( ( always_inline ) ) __STATIC_INLINE void led_off(int pin)
{
    /* アクティブLOW */
    nrf_gpio_pin_set(pin);
}


/**********************************************/
/* タイマ                                     */
/**********************************************/

/**@brief タイマ初期化
 *
 * タイマ機能を初期化する。
 * 調べた範囲では、以下の機能を使用する場合には、その初期化よりも前に実行しておく必要がある。
 *  - ボタン
 *  - BLE
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);

#if 0
    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
     */
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code);
#endif
}


/**@brief タイマ開始
*/
static void timers_start(void)
{
#if 0
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer. */
    uint32_t err_code;

    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
#endif
}


/**********************************************/
/* Scheduler                                  */
/**********************************************/

/**@brief スケジューラ初期化
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**********************************************/
/* GPIOTE & Button                            */
/**********************************************/

/**@brief GPIOTE初期化
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief ボタン初期化
 *
 * @note    初期化しただけではボタンを使用できない。
 *          使用する場合はapp_button_enable()を呼び出すこと。
 */
static void buttons_init(void)
{
    // Note: SoftDeviceにアドレスが保持されるので、staticにしておくこと
    static app_button_cfg_t buttons[] = {
        { BUTTON_PIN_NO_APP, APP_BUTTON_ACTIVE_HIGH, NRF_GPIO_PIN_PULLDOWN, button_event_handler },
    };

    APP_BUTTON_INIT(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY, true);
}


/**@brief ボタンイベントハンドラ
 *
 * @param[in]   pin_no         イベントが発生したピン番号
 * @param[in]   button_event   APP_BUTTON_PUSH or APP_BUTTON_RELEASE.
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_event)
{
    uint32_t err_code = NRF_SUCCESS;
    uint8_t value;

    /*
     * APPボタンを押下すると0x80を、離すと0x00を送信する。
     */
    switch (pin_no) {
    case BUTTON_PIN_NO_APP:
        if (button_event == APP_BUTTON_PUSH) {
            //push
            value = 0x80;
        }
        else {
            //release
            value = 0x00;
        }
        //Notify送信
        err_code = ble_ios_on_output(&m_ios, value);
        if ((err_code != NRF_SUCCESS) &&
          (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
          (err_code != NRF_ERROR_INVALID_STATE)) {
            APP_ERROR_CHECK(err_code);
        }
        break;

    default:
        APP_ERROR_HANDLER(pin_no);
        break;
    }

}


/**********************************************/
/* BLE : GAP                                  */
/**********************************************/

/**@brief GAP初期化
 *
 *  - デバイス名設定(必須)
 *  - Appearance設定(optional:設定無し)
 *  - PPCP設定(optional:設定有り)
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    /* デバイス名設定 */
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

#if 0
    /* YOUR_JOB: Use an appearance value matching the application's use case. */
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
    APP_ERROR_CHECK(err_code);
#endif

    /*
     * Peripheral Preferred Connection Parameters(PPCP)
     * ここで設定しておくと、Connection Parameter Update Reqを送信せずに済むらしい。
     *
     * パラメータの意味はCore_v4.1 p.2537 "4.5 CONNECTION STATE"を参照
     *  connInterval : Connectionイベントの送信間隔(7.5msec～4sec)
     *  connSlaveLatency : SlaveがConnectionイベントを連続して無視できる回数
     *  connSupervisionTimeout :
     */
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**********************************************/
/* BLE : Advertising                          */
/**********************************************/

/**@brief Advertising初期化
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    // サービスのUUID設定
    ble_uuid_t adv_uuids[] = {
        { IOS_UUID_SERVICE, m_ios.uuid_type }
    };

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = ARRAY_SIZE(adv_uuids);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    led_on(LED_PIN_NO_ADVERTISING);
}


/**********************************************/
/* BLE : Services                             */
/**********************************************/

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ble_ios_init_t ios_init;
    ios_init.evt_handler_in = services_ios_handler_in;
    uint32_t err_code = ble_ios_init(&m_ios, &ios_init);
    APP_ERROR_CHECK(err_code);
}


//void (*ble_ios_evt_handler_t) (ble_ios_t *p_ios, uint8_t value)
static void services_ios_handler_in(ble_ios_t *p_ios, uint8_t value)
{
    if (value == 0x80) {
        //
        led_on(LED_PIN_NO_APP);
    }
    else {
        //
        led_off(LED_PIN_NO_APP);
    }
}


/**********************************************/
/* BLE : Security                             */
/**********************************************/

/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**********************************************/
/* BLE : Connect                              */
/**********************************************/

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = conn_params_evt_handler;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void conn_params_evt_handler(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**********************************************/
/* BLE                                        */
/**********************************************/

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_handler(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        led_on(LED_PIN_NO_CONNECTED);
        led_off(LED_PIN_NO_ADVERTISING);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

        //接続されている間だけボタンは使用できる
        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        led_off(LED_PIN_NO_CONNECTED);
        m_conn_handle = BLE_CONN_HANDLE_INVALID;

        //接続されている間だけボタンは使用できる
        err_code = app_button_disable();
        APP_ERROR_CHECK(err_code);

        advertising_start();
        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                               BLE_GAP_SEC_STATUS_SUCCESS,
                                               &m_sec_params);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_AUTH_STATUS:
        m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
        break;

    case BLE_GAP_EVT_SEC_INFO_REQUEST:
        p_enc_info = &m_auth_status.periph_keys.enc_info;
        if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div) {
            err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
            APP_ERROR_CHECK(err_code);
        }
        else {
            // No keys found for this device
            err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
            APP_ERROR_CHECK(err_code);
        }
        break;

    case BLE_GAP_EVT_TIMEOUT:
        if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT) {
            led_off(LED_PIN_NO_ADVERTISING);

            // Go to system-off mode (this function will not return; wakeup will cause a reset)
            err_code = sd_power_system_off();
            APP_ERROR_CHECK(err_code);
        }
        break;

    default:
        // No implementation needed.
        break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);

    //I/O Service
    ble_ios_on_ble_evt(&m_ios, p_ble_evt);
}


/**@brief BLEスタック初期化
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, true);

    // Register with the SoftDevice handler module for System events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

