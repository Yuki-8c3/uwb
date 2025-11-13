/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * Experiment 5: 同步频率（锚点后台功耗）
 * A组：sync.period_s=1s（基线双源 PPS+PTP）
 * B组：sync.period_s=2s（放宽）
 */

#include "sdk_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "boards.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_spi.h"
#include "nrf_uart.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "app_error.h"
#include <string.h>
#include "port_platform.h"
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"
#include "uart.h"
#include "ss_resp_main.h"

//-----------------dw1000----------------------------

/* Baseline configuration */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PRF_64M,      /* Pulse repetition frequency: 64MHz */
    DWT_PLEN_64,      /* Preamble length: 64 symbols */
    DWT_PAC8,         /* Preamble acquisition chunk size: 8 */
    10,               /* TX preamble code. */
    10,               /* RX preamble code. */
    0,                /* 0 = standard SFD (8 symbols) */
    DWT_BR_6M8,       /* Data rate: 6.8 Mbps */
    DWT_PHRMODE_STD,  /* PHY header mode: standard */
    (64 + 8 - 8)      /* SFD timeout */
};

/* Sync frequency modes */
typedef enum {
    SYNC_MODE_A = 0,  /* A组：sync.period_s=1s（基线） */
    SYNC_MODE_B = 1   /* B组：sync.period_s=2s（放宽） */
} sync_mode_t;

/* Sync periods in seconds */
#define SYNC_PERIOD_A_S  1   /* 1 second */
#define SYNC_PERIOD_B_S  2   /* 2 seconds */

/*Should be accurately calculated during calibration*/
#define TX_ANT_DLY 16300
#define RX_ANT_DLY 16456

/* Experiment timing parameters */
#define EXPERIMENT_DURATION_MS (150 * 1000)
#define MODE_SWITCH_DELAY_MS (10 * 1000)

//--------------dw1000---end---------------

#define TASK_DELAY        200
#define TIMER_PERIOD      2000

#ifdef USE_FREERTOS

TaskHandle_t  ss_responder_task_handle;
extern void ss_responder_task_function (void * pvParameter);
TaskHandle_t  led_toggle_task_handle;
TimerHandle_t led_toggle_timer_handle;
#endif

#ifdef USE_FREERTOS

static void led_toggle_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  while (true)
  {
    LEDS_INVERT(BSP_LED_0_MASK);
    vTaskDelay(TASK_DELAY);
  }
}

static void led_toggle_timer_callback (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  LEDS_INVERT(BSP_LED_1_MASK);
}
#else

  extern int ss_resp_run(void);

#endif

/* Forward declaration */
static void sync_mode_switch_monitor_task(void *pvParameter);

int main(void)
{
  LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK);
  LEDS_ON(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK );

  #ifdef USE_FREERTOS
    UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task_handle));
    led_toggle_timer_handle = xTimerCreate( "LED1", TIMER_PERIOD, pdTRUE, NULL, led_toggle_timer_callback);
    UNUSED_VARIABLE(xTimerStart(led_toggle_timer_handle, 0));
    UNUSED_VARIABLE(xTaskCreate(ss_responder_task_function, "SSTWR_RESP", configMINIMAL_STACK_SIZE + 200, NULL, 2, &ss_responder_task_handle));
    TaskHandle_t sync_mode_switch_task_handle;
    UNUSED_VARIABLE(xTaskCreate(sync_mode_switch_monitor_task, "SYNC_SWITCH", configMINIMAL_STACK_SIZE + 200, NULL, 1, &sync_mode_switch_task_handle));
  #endif
  
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL);
  boUART_Init ();
  printf("========================================\r\n");
  printf("Experiment 5: Sync Frequency\r\n");
  printf("Anchor Background Sync Power Test\r\n");
  printf("========================================\r\n");
  
  reset_DW1000(); 
  port_set_dw1000_slowrate();
  
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
  {
    printf("ERROR: DW1000 initialization failed!\r\n");
    while (1) {};
  }

  port_set_dw1000_fastrate();
  dwt_configure(&config);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setrxtimeout(0);

  /* Start with SYNC_MODE_A (1s baseline) */
  set_current_sync_mode(SYNC_MODE_A);
  
  #ifdef USE_FREERTOS		
    vTaskStartScheduler();	
    while(1) {};
  #else
    while (1)
    {
      ss_resp_run();
    }
  #endif
}

static void sync_mode_switch_monitor_task(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    
    while (1)
    {
        sync_mode_t pending = get_pending_sync_mode_switch();
        sync_mode_t current = get_current_sync_mode();
        
        if (pending != current)
        {
            printf("Main: Detected sync mode switch request from %d to %d\r\n", current, pending);
            clear_sync_mode_switch_pending();
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. Experiment 5: 同步频率（锚点后台功耗）
 *    - A组：sync.period_s=1s（基线双源 PPS+PTP）
 *    - B组：sync.period_s=2s（放宽）
 *    - 记录同步相关的功耗和定位精度指标
 *
 ****************************************************************************************************************************************************/


