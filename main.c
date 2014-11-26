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

/**************************************************************************
 * include
 **************************************************************************/

#include <string.h>

#include "sd_common/softdevice_handler.h"
#include "app_common/app_timer.h"
#include "app_common/app_gpiote.h"
#include "app_common/app_button.h"
#include "ble/ble_advdata.h"
#include "ble/ble_conn_params.h"
#include "s110/ble_hci.h"

#include "ble_ios.h"

//from Makefile
#ifdef USE_UART_LOG
#include "simple_uart.h"
#endif  /* USE_UART_LOG */


/**************************************************************************
 * macro
 **************************************************************************/

/**
 * Include or not the service_changed characteristic.
 * if not enabled, the server's database cannot be changed for the lifetime of the device
 */
#define IS_SRVC_CHANGED_CHARACT_PRESENT (0)

/** Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define DEAD_BEEF                       (0xDEADBEEF)

#define ARRAY_SIZE(array)               (sizeof(array) / sizeof(array[0]))

/*
 * ピン番号
 */
/** Button : アプリ用 */
#define BUTTON_PIN_NO_APP               (19)

/** LED : Advertising中 */
#define LED_PIN_NO_ADVERTISING          (21)

/** LED : Connect中 */
#define LED_PIN_NO_CONNECTED            (26)

/** LED : Assert発生 */
#define LED_PIN_NO_ASSERT               (27)

/** LED : アプリ用 */
#define LED_PIN_NO_APP                  (16)

#ifdef USE_UART_LOG
#define UART_PIN_NO_RX                  (9)
#define UART_PIN_NO_TX                  (8)
#endif  /* USE_UART_LOG */


/*
 * Timer
 */
/** Value of the RTC1 PRESCALER register. */
#define APP_TIMER_PRESCALER             (0)

/** Maximum number of simultaneously created timers. */
#define APP_TIMER_MAX_TIMERS            (2)

/** Size of timer operation queues. */
#define APP_TIMER_OP_QUEUE_SIZE         (4)

/*
 * GPIOTEおよびButton
 */
/** Maximum number of users of the GPIOTE handler. */
#define APP_GPIOTE_MAX_USERS            (1)

/** Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */
#define BUTTON_DETECTION_DELAY          (50)

/*
 * Scheduler
 */

/**
 * Maximum size of scheduler events.
 * Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler.
 */
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)

/** Maximum number of events in the scheduler queue. */
#define SCHED_QUEUE_SIZE                (10)


/*
 * デバイス名
 *   UTF-8かつ、\0を含まずに20文字以内(20byte?)
 */
#define GAP_DEVICE_NAME                 "hiro99ma_Template"

/*
 * Appearance設定
 *  コメントアウト時はAppearance無しにするが、おそらくSoftDeviceがUnknownにしてくれる。
 *
 * Bluetooth Core Specification Supplement, Part A, Section 1.12
 * Bluetooth Core Specification 4.0 (Vol. 3), Part C, Section 12.2
 * https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
 * https://devzone.nordicsemi.com/documentation/nrf51/4.3.0/html/group___b_l_e___a_p_p_e_a_r_a_n_c_e_s.html
 */

/*
 * BLE : Advertising
 */
/* Advertising間隔[msec単位] */
#define APP_ADV_INTERVAL                (100)

/* Advertisingタイムアウト時間[sec単位] */
#define APP_ADV_TIMEOUT_IN_SECONDS      (180)

/*
 * Peripheral Preferred Connection Parameters(PPCP)
 *   パラメータの意味はCore_v4.1 p.2537 "4.5 CONNECTION STATE"を参照
 *
 * connInterval : Connectionイベントの送信間隔(7.5msec～4sec)
 * connSlaveLatency : SlaveがConnectionイベントを連続して無視できる回数
 * connSupervisionTimeout : Connection時に相手がいなくなったとみなす時間(100msec～32sec)
 *                          (1 + connSlaveLatency) * connInterval * 2以上
 */
/* 最小時間[msec単位] */
#define CONN_MIN_INTERVAL               (500)

/* 最大時間[msec単位] */
#define CONN_MAX_INTERVAL               (1000)

/* slave latency */
#define CONN_SLAVE_LATENCY              (0)

/* connSupervisionTimeout[msec単位] */
#define CONN_SUP_TIMEOUT                (4000)

/** sd_ble_gap_conn_param_update()を実行してから初回の接続イベントを通知するまでの時間[msec単位] */
/*
 * sd_ble_gap_conn_param_update()
 *   Central role時:
 *      Link Layer接続パラメータ更新手続きを初期化する。
 *
 *   Peripheral role時:
 *      L2CAPへ連絡要求(corresponding L2CAP request)を送信し、Centralからの手続きを待つ。
 *      接続パラメータとしてNULLが指定でき、そのときはPPCPキャラクタリスティックが使われる。
 *      ということは、Peripheralだったらsd_ble_gap_conn_param_update()は呼ばずに
 *      sd_ble_gap_ppcp_set()を呼ぶということでもよいということか？
 *
 * note:
 *      connSupervisionTimeout * 8 >= connIntervalMax * (connSlaveLatency + 1)
 */
#define CONN_FIRST_PARAMS_UPDATE_DELAY  (5000)

/** sd_ble_gap_conn_param_update()を呼び出す間隔[msec単位] */
#define CONN_NEXT_PARAMS_UPDATE_DELAY   (30000)

/** Number of attempts before giving up the connection parameter negotiation. */
#define CONN_MAX_PARAMS_UPDATE_COUNT    (3)

/*
 * BLE : Security
 */
/** ペアリング要求からのタイムアウト時間[sec] */
#define SEC_PARAM_TIMEOUT               (30)

/** 1:Bondingあり 0:なし */
#define SEC_PARAM_BOND                  (1)

/** 1:ペアリング時の認証あり 0:なし */
#define SEC_PARAM_MITM                  (0)

/** IO能力
 * - BLE_GAP_IO_CAPS_DISPLAY_ONLY     : input=無し/output=画面
 * - BLE_GAP_IO_CAPS_DISPLAY_YESNO    : input=Yes/No程度/output=画面
 * - BLE_GAP_IO_CAPS_KEYBOARD_ONLY    : input=キーボード/output=無し
 * - BLE_GAP_IO_CAPS_NONE             : input/outputなし。あるいはMITM無し
 * - BLE_GAP_IO_CAPS_KEYBOARD_DISPLAY : input=キーボード/output=画面
 */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE

/** 1:OOB認証有り/0:なし */
#define SEC_PARAM_OOB                   (0)

/** 符号化鍵サイズ:最小byte(7～) */
#define SEC_PARAM_MIN_KEY_SIZE          (7)

/** 符号化鍵サイズ:最大byte(min～16) */
#define SEC_PARAM_MAX_KEY_SIZE          (16)


/*
 * UARTデバッグログ出力
 *   使用する場合は、MakefileでUSE_UART_LOGを有効にすること。
 */
#ifdef USE_UART_LOG
#define DBG_OUT(str)    simple_uart_putstring((const uint8_t *)str)
#else
#define DBG_OUT(str)    /* nothing */
#endif  /* USE_UART_LOG */


//設定値のチェック
#if (APP_ADV_INTERVAL < 20)
#error connInterval(Advertising) too small.
#elif (APP_ADV_INTERVAL < 100)
#warning connInterval(Advertisin) is too small in non-connectable mode
#elif (10240 < APP_ADV_INTERVAL)
#error connInterval(Advertising) too large.
#endif  //APP_ADV_INTERVAL

#if (BLE_GAP_ADV_TIMEOUT_LIMITED_MAX < APP_ADV_TIMEOUT_IN_SECONDS)
#warning Advertising Timeout is too large in limited discoverable mode
#endif  //APP_ADV_TIMEOUT_IN_SECONDS

#if (CONN_MIN_INTERVAL * 1000 < 7500)
#error connInterval_Min(Connection) too small.
#elif (4000 < CONN_MIN_INTERVAL)
#error connInterval_Min(Connection) too large.
#endif  //CONN_MIN_INTERVAL

#if (CONN_MAX_INTERVAL * 1000 < 7500)
#error connInterval_Max(Connection) too small.
#elif (4000 < CONN_MAX_INTERVAL)
#error connInterval_Max(Connection) too large.
#endif  //CONN_MAX_INTERVAL

#if (CONN_MAX_INTERVAL < CONN_MIN_INTERVAL)
#error connInterval_Max < connInterval_Min
#endif  //connInterval Max < Min

#if (BLE_GAP_CP_SLAVE_LATENCY_MAX < CONN_SLAVE_LATENCY)
#error connSlaveLatency too large.
#endif  //CONN_SLAVE_LATENCY

#if (CONN_SUP_TIMEOUT < 100)
#error connSupervisionTimeout too small.
#elif (32000 < CONN_SUP_TIMEOUT)
#error connSupervisionTimeout too large.
#endif  //CONN_SUP_TIMEOUT

#if (CONN_SUP_TIMEOUT * 8 < CONN_MAX_INTERVAL * (CONN_SLAVE_LATENCY + 1))
#error connSupervisionTimeout too small in manner.
#endif  //


/**************************************************************************
 * declaration
 **************************************************************************/

/** Handle of the current connection. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;
static ble_ios_t                        m_ios;


/**************************************************************************
 * prototype
 **************************************************************************/

static void sys_evt_dispatch(uint32_t sys_evt);

/* GPIO */
static void gpio_init(void);

/* LED */
__attribute__( ( always_inline ) ) __STATIC_INLINE void led_on(int pin);
__attribute__( ( always_inline ) ) __STATIC_INLINE void led_off(int pin);

/* Timer */
static void timers_init(void);
//static void timers_start(void);

/* Scheduler */
static void scheduler_init(void);

/* GPIOTEおよびButton */
static void gpiote_init(void);
static void buttons_init(void);
static void button_event_handler(uint8_t pin_no, uint8_t button_event);

static void advertising_start(void);

static void services_ios_handler_in(ble_ios_t *p_ios, uint8_t value);

static void conn_params_evt_handler(ble_conn_params_evt_t * p_evt);
static void conn_params_error_handler(uint32_t nrf_error);

static void ble_evt_handler(ble_evt_t * p_ble_evt);
static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
static void ble_stack_init(void);


/**************************************************************************
 * main entry
 **************************************************************************/

/**@brief main
 */
int main(void)
{
    // 初期化
    gpio_init();
    timers_init();		//buttons_init()やble_stack_init()よりも前に呼ぶこと!

    gpiote_init();
    buttons_init();
    scheduler_init();

#ifdef USE_UART_LOG
    /*
     * Baud: 38400, data bits: 8 , Stop Bits: 1
     * HW flow制御 : なし
     */
    simple_uart_config(0, UART_PIN_NO_TX, 0, UART_PIN_NO_TX, false);
#endif  /* USE_UART_LOG */

    ble_stack_init();

    // 処理開始
    //timers_start();
    advertising_start();

    // メインループ
    while (1) {
        //スケジュール済みイベントの実行(mainloop内で呼び出す)
        app_sched_execute();
        uint32_t err_code = sd_app_evt_wait();
        APP_ERROR_CHECK(err_code);
    }
}


/**************************************************************************
 * public function
 **************************************************************************/

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


/**************************************************************************
 * private function
 **************************************************************************/

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


/**********************************************
 * GPIO
 **********************************************/

/**
 * @brief GPIO初期化
 */
static void gpio_init(void)
{
    nrf_gpio_cfg_output(LED_PIN_NO_ADVERTISING);
    nrf_gpio_cfg_output(LED_PIN_NO_CONNECTED);
    nrf_gpio_cfg_output(LED_PIN_NO_ASSERT);
    nrf_gpio_cfg_output(LED_PIN_NO_APP);

    /* LED消灯 */
    led_off(LED_PIN_NO_ADVERTISING);
    led_off(LED_PIN_NO_CONNECTED);
    led_off(LED_PIN_NO_ASSERT);
    led_off(LED_PIN_NO_APP);
}


/**********************************************
 * LED
 **********************************************/

/**
 * @brief LED点灯
 *
 * @param[in]   pin     対象PIN番号
 */
__attribute__( ( always_inline ) ) __STATIC_INLINE void led_on(int pin)
{
    /* アクティブLOW */
    nrf_gpio_pin_clear(pin);
}


/**
 * @brief LED消灯
 *
 * @param[in]   pin     対象PIN番号
 */
__attribute__( ( always_inline ) ) __STATIC_INLINE void led_off(int pin)
{
    /* アクティブLOW */
    nrf_gpio_pin_set(pin);
}


/**********************************************
 * タイマ
 **********************************************/

/**
 * @brief タイマ初期化
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

#if 0
/**
 * @brief タイマ開始
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
#endif

/**********************************************
 * Scheduler
 **********************************************/

/**
 * @brief スケジューラ初期化
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**********************************************
 * GPIOTE
 **********************************************/

/**
 * @brief GPIOTE初期化
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**********************************************
 * Button
 **********************************************/

/**
 * @brief ボタン初期化
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

    APP_BUTTON_INIT(buttons, ARRAY_SIZE(buttons),
                APP_TIMER_TICKS(BUTTON_DETECTION_DELAY, APP_TIMER_PRESCALER), true);
}


/**
 * @brief ボタンイベントハンドラ
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


/**********************************************
 * BLE : Advertising
 **********************************************/

/**
 * @brief Advertising開始
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
    adv_params.interval    = MSEC_TO_UNITS(APP_ADV_INTERVAL, UNIT_0_625_MS);
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    led_on(LED_PIN_NO_ADVERTISING);

    DBG_OUT("advertising start\r\n");
}


/**********************************************
 * BLE : Services
 **********************************************/

/**
 * @brief I/Oサービスイベントハンドラ
 *
 * @param[in]   p_ios   I/Oサービス構造体
 * @param[in]   value   受信した値
 */
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


/**********************************************
 * BLE : Connection
 **********************************************/

/**
 * @brief Connectionパラメータモジュールイベントハンドラ
 *
 * @details Connectionパラメータモジュールでアプリに通知するイベントが発生した場合に呼ばれる。
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Connectionパラメータモジュールから受信したイベント
 */
static void conn_params_evt_handler(ble_conn_params_evt_t *p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**
 * @brief Connectionパラメータエラーハンドラ
 *
 * @param[in]   nrf_error   エラーコード
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**********************************************
 * BLE stack
 **********************************************/

/**
 * @brief BLEスタックイベントハンドラ
 *
 * @param[in]   p_ble_evt   BLEスタックイベント
 */
static void ble_evt_handler(ble_evt_t *p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;

    switch (p_ble_evt->header.evt_id) {
    /*************
     * GAP event
     *************/

    //接続が成立したとき
    case BLE_GAP_EVT_CONNECTED:
        DBG_OUT("BLE_GAP_EVT_CONNECTED\r\n");
        led_on(LED_PIN_NO_CONNECTED);
        led_off(LED_PIN_NO_ADVERTISING);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

        //ボタンの使用を許可する
        err_code = app_button_enable();
        APP_ERROR_CHECK(err_code);
        break;

    //相手から切断されたとき
    //必要があればsd_ble_gatts_sys_attr_get()でSystem Attributeを取得し、保持しておく。
    //保持したSystem Attributeは、EVT_SYS_ATTR_MISSINGで返すことになる。
    case BLE_GAP_EVT_DISCONNECTED:
        DBG_OUT("BLE_GAP_EVT_DISCONNECTED\r\n");
        led_off(LED_PIN_NO_CONNECTED);
        m_conn_handle = BLE_CONN_HANDLE_INVALID;

        //ボタンの使用を禁止する
        err_code = app_button_disable();
        APP_ERROR_CHECK(err_code);

        advertising_start();
        break;

    //SMP Paring要求を受信したとき
    //sd_ble_gap_sec_params_reply()で値を返したあと、SMP Paring Phase 2に状態遷移する
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        DBG_OUT("BLE_GAP_EVT_SEC_PARAMS_REQUEST\r\n");
        {
            ble_gap_sec_params_t sec_param;
            sec_param.timeout      = SEC_PARAM_TIMEOUT;
            sec_param.bond         = SEC_PARAM_BOND;
            sec_param.mitm         = SEC_PARAM_MITM;
            sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
            sec_param.oob          = SEC_PARAM_OOB;
            sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
            sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                               BLE_GAP_SEC_STATUS_SUCCESS,
                                               &sec_param);
            APP_ERROR_CHECK(err_code);
        }
        break;

    //Just Works(Bonding有り)の場合、SMP Paring Phase 3のあとでPeripheral Keyが渡される。
    //ここではPeripheral Keyを保存だけしておき、次のBLE_GAP_EVT_SEC_INFO_REQUESTで処理する。
    case BLE_GAP_EVT_AUTH_STATUS:
        DBG_OUT("BLE_GAP_EVT_AUTH_STATUS\r\n");
        m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
        break;

    //SMP Paringが終わったとき？
    case BLE_GAP_EVT_SEC_INFO_REQUEST:
        DBG_OUT("BLE_GAP_EVT_SEC_INFO_REQUEST\r\n");
        p_enc_info = &m_auth_status.periph_keys.enc_info;
        if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div) {
            //Peripheral Keyが有る
            err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
        }
        else {
            //Peripheral Keyが無い
            err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
        }
        APP_ERROR_CHECK(err_code);
        break;

    //Advertisingか認証のタイムアウト発生
    case BLE_GAP_EVT_TIMEOUT:
        DBG_OUT("BLE_GAP_EVT_TIMEOUT\r\n");
        switch (p_ble_evt->evt.gap_evt.params.timeout.src) {
        case BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT: //Advertisingのタイムアウト
            /* Advertising LEDを消灯 */
            led_off(LED_PIN_NO_ADVERTISING);

            /* System-OFFにする(もう戻ってこない) */
            err_code = sd_power_system_off();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_TIMEOUT_SRC_SECURITY_REQUEST:  //Security requestのタイムアウト
            break;

        default:
            break;
        }
        break;

    /*********************
     * GATT Server event
     *********************/

    //接続後、Bondingした相手からSystem Attribute要求を受信したとき
    //System Attributeは、EVT_DISCONNECTEDで保持するが、今回は保持しないのでNULLを返す。
    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        DBG_OUT("BLE_GATTS_EVT_SYS_ATTR_MISSING\r\n");
        err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
        APP_ERROR_CHECK(err_code);
        break;

    default:
        // No implementation needed.
        break;
    }
}


/**
 * @brief BLEイベントハンドラ
 *
 * @details BLEスタックイベント受信後、メインループのスケジューラから呼ばれる。
 *
 * @param[in]   p_ble_evt   BLEスタックイベント
 */
static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
    ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);

    //I/O Service
    ble_ios_on_ble_evt(&m_ios, p_ble_evt);
}


/**
 * @brief BLEスタック初期化
 *
 * @detail BLE関連の初期化を行う。
 *      -# SoftDeviceハンドラ初期化
 *      -# システムイベントハンドラ初期化
 *      -# BLEスタック有効化
 *      -# BLEイベントハンドラ設定
 *      -# デバイス名設定
 *      -# Appearance設定(GAP_USE_APPEARANCE定義時)
 *      -# PPCP設定
 *      -# Service初期化
 *      -# Advertising初期化
 *      -# Connection初期化
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    /*
     * SoftDeviceの初期化
     * スケジューラ使用はよいとして、HANDLERは何を指しているのか？
     */
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, true);

    /* システムイベントハンドラの設定 */
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    /* BLEスタックの有効化 */
    {
        ble_enable_params_t ble_enable_params;

        memset(&ble_enable_params, 0, sizeof(ble_enable_params));
        ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
        err_code = sd_ble_enable(&ble_enable_params);
        APP_ERROR_CHECK(err_code);
    }

    /* BLEイベントハンドラの設定 */
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    /* デバイス名設定 */
    {
        //デバイス名へのWrite Permission(no protection, open link)
        ble_gap_conn_sec_mode_t sec_mode;

        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
        err_code = sd_ble_gap_device_name_set(&sec_mode,
                                            (const uint8_t *)GAP_DEVICE_NAME,
                                            strlen(GAP_DEVICE_NAME));
        APP_ERROR_CHECK(err_code);
    }

#ifdef GAP_USE_APPEARANCE
    /* Appearance設定 */
    err_code = sd_ble_gap_appearance_set(GAP_USE_APPEARANCE);
    APP_ERROR_CHECK(err_code);
#endif  //GAP_USE_APPEARANCE

    /*
     * Peripheral Preferred Connection Parameters(PPCP)
     * ここで設定しておくと、Connection Parameter Update Reqを送信せずに済むらしい。
     */
    {
        ble_gap_conn_params_t   gap_conn_params = {0};

        gap_conn_params.min_conn_interval = MSEC_TO_UNITS(CONN_MIN_INTERVAL, UNIT_1_25_MS);
        gap_conn_params.max_conn_interval = MSEC_TO_UNITS(CONN_MAX_INTERVAL, UNIT_1_25_MS);
        gap_conn_params.slave_latency     = CONN_SLAVE_LATENCY;
        gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(CONN_SUP_TIMEOUT, UNIT_10_MS);

        err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
        APP_ERROR_CHECK(err_code);
    }

    /////////////////////////////
    // Service初期化
    {
        ble_ios_init_t ios_init;

        ios_init.evt_handler_in = services_ios_handler_in;
        err_code = ble_ios_init(&m_ios, &ios_init);
        APP_ERROR_CHECK(err_code);
    }

    /////////////////////////////
    // Advertising初期化
    {
        ble_uuid_t adv_uuids[] = { { IOS_UUID_SERVICE, m_ios.uuid_type } };
        ble_advdata_t advdata = {0};
        ble_advdata_t scanrsp = {0};
        uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

        /*
         * ble_advdata_name_type_t (ble_advdata.h)
         *
         * BLE_ADVDATA_NO_NAME    : デバイス名無し
         * BLE_ADVDATA_SHORT_NAME : デバイス名あり «Shortened Local Name»
         * BLE_ADVDATA_FULL_NAME  : デバイス名あり «Complete Local Name»
         *
         * https://www.bluetooth.org/en-us/specification/assigned-numbers/generic-access-profile
         */
        advdata.name_type          = BLE_ADVDATA_FULL_NAME;

        /*
         * Appearanceが含まれるかどうか
         */
#ifdef GAP_USE_APPEARANCE
        advdata.include_appearance = true;
#else   //GAP_USE_APPEARANCE
        advdata.include_appearance = false;
#endif  //GAP_USE_APPEARANCE
        /*
         * Advertisingフラグの設定
         * CSS_v4 : Part A  1.3 FLAGS
         * https://devzone.nordicsemi.com/documentation/nrf51/4.3.0/html/group___b_l_e___g_a_p___a_d_v___f_l_a_g_s.html
         *
         * BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE = BLE_GAP_ADV_FLAG_LE_LIMITED_DISC_MODE | BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED
         *      BLE_GAP_ADV_FLAG_LE_LIMITED_DISC_MODE : LE Limited Discoverable Mode
         *      BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED : BR/EDR not supported
         */
        advdata.flags.size         = sizeof(flags);
        advdata.flags.p_data       = &flags;

        /* SCAN_RSPデータ設定 */
        scanrsp.uuids_complete.uuid_cnt = ARRAY_SIZE(adv_uuids);
        scanrsp.uuids_complete.p_uuids  = adv_uuids;

        err_code = ble_advdata_set(&advdata, &scanrsp);
        APP_ERROR_CHECK(err_code);
    }

    /////////////////////////////
    // Connection初期化
    {
        ble_conn_params_init_t cp_init = {0};

        cp_init.p_conn_params                  = NULL;
        cp_init.first_conn_params_update_delay = APP_TIMER_TICKS(CONN_FIRST_PARAMS_UPDATE_DELAY, APP_TIMER_PRESCALER);
        cp_init.next_conn_params_update_delay  = APP_TIMER_TICKS(CONN_NEXT_PARAMS_UPDATE_DELAY, APP_TIMER_PRESCALER);
        cp_init.max_conn_params_update_count   = CONN_MAX_PARAMS_UPDATE_COUNT;
        cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
        cp_init.disconnect_on_fail             = false;
        cp_init.evt_handler                    = conn_params_evt_handler;
        cp_init.error_handler                  = conn_params_error_handler;

        err_code = ble_conn_params_init(&cp_init);
        APP_ERROR_CHECK(err_code);
    }
}

