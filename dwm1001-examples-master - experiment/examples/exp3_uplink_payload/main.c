/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * Experiment 3: 上报内容大小（回传侧功耗/带宽）
 * A组：uplink.payload=TS（仅时间戳）
 * B组：uplink.payload=TS+CIR_s, cir_s_bytes=96（含CIR摘要）
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

/* Baseline configuration (same as experiment 1 baseline) */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PRF_64M,      /* Pulse repetition frequency: 64MHz */
    DWT_PLEN_64,      /* Preamble length: 64 symbols */
    DWT_PAC8,         /* Preamble acquisition chunk size: 8 (for PL=64) */
    10,               /* TX preamble code. */
    10,               /* RX preamble code. */
    0,                /* 0 = standard SFD (8 symbols) */
    DWT_BR_6M8,       /* Data rate: 6.8 Mbps */
    DWT_PHRMODE_STD,  /* PHY header mode: standard */
    (64 + 8 - 8)      /* SFD timeout (preamble length + 1 + SFD length - PAC size) */
};

/* Uplink payload modes */
typedef enum {
    UPLINK_MODE_A = 0,  /* A组：仅时间戳（TS） */
    UPLINK_MODE_B = 1   /* B组：时间戳+CIR摘要（TS+CIR_s, 96 bytes） */
} uplink_mode_t;

/* A组：仅时间戳大小（假设8字节时间戳） */
#define UPLINK_A_PAYLOAD_SIZE  8   /* TS only */

/* B组：时间戳+CIR摘要大小 */
#define UPLINK_B_PAYLOAD_SIZE  (8 + 96)  /* TS + CIR_s (96 bytes) */

/* Preamble timeout */
#define PRE_TIMEOUT 1000

/*Should be accurately calculated during calibration*/
#define TX_ANT_DLY 16300
#define RX_ANT_DLY 16456

/* Experiment timing parameters */
#define EXPERIMENT_DURATION_MS (150 * 1000)  /* 150 seconds per mode */
#define MODE_SWITCH_DELAY_MS (10 * 1000)      /* 10 seconds for mode switching */

//--------------dw1000---end---------------

#define TASK_DELAY        200           /**< Task delay. Delays a LED0 task for 200 ms */
#define TIMER_PERIOD      2000          /**< Timer period. LED1 timer will expire after 1000 ms */

#ifdef USE_FREERTOS

TaskHandle_t  ss_responder_task_handle;   /**< Reference to SS TWR Responder FreeRTOS task. */
extern void ss_responder_task_function (void * pvParameter);
TaskHandle_t  led_toggle_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */
#endif

#ifdef USE_FREERTOS

/**@brief LED0 task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void led_toggle_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  while (true)
  {
    LEDS_INVERT(BSP_LED_0_MASK);
    /* Delay a task for a given number of ticks */
    vTaskDelay(TASK_DELAY);
    /* Tasks must be implemented to never return... */
  }
}

/**@brief The function to call when the LED1 FreeRTOS timer expires.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.
 */
static void led_toggle_timer_callback (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  LEDS_INVERT(BSP_LED_1_MASK);
}
#else

  extern int ss_resp_run(void);

#endif   // #ifdef USE_FREERTOS

/* Forward declaration */
static void uplink_mode_switch_monitor_task(void *pvParameter);

int main(void)
{
  /* Setup some LEDs for debug Green and Blue on DWM1001-DEV */
  LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK);
  LEDS_ON(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK );

  #ifdef USE_FREERTOS
    /* Create task for LED0 blinking with priority set to 2 */
    UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task_handle));

    /* Start timer for LED1 blinking */
    led_toggle_timer_handle = xTimerCreate( "LED1", TIMER_PERIOD, pdTRUE, NULL, led_toggle_timer_callback);
    UNUSED_VARIABLE(xTimerStart(led_toggle_timer_handle, 0));

    /* Create task for SS TWR Responder set to 2 */
    UNUSED_VARIABLE(xTaskCreate(ss_responder_task_function, "SSTWR_RESP", configMINIMAL_STACK_SIZE + 200, NULL, 2, &ss_responder_task_handle));
    
    /* Create a task to monitor and handle uplink mode switches */
    TaskHandle_t uplink_mode_switch_task_handle;
    UNUSED_VARIABLE(xTaskCreate(uplink_mode_switch_monitor_task, "UPLINK_SWITCH", configMINIMAL_STACK_SIZE + 200, NULL, 1, &uplink_mode_switch_task_handle));
  #endif // #ifdef USE_FREERTOS
  
  //-------------dw1000  ini------------------------------------	

  /* Setup DW1000 IRQ pin */  
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); 		//irq
  
  /*Initialization UART*/
  boUART_Init ();
  printf("========================================\r\n");
  printf("Experiment 3: Uplink Payload Size\r\n");
  printf("Anchor Uplink Power/Bandwidth Test\r\n");
  printf("========================================\r\n");
  
  /* Reset DW1000 */
  reset_DW1000(); 

  /* Set SPI clock to 2MHz */
  port_set_dw1000_slowrate();			
  
  /* Init the DW1000 */
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
  {
    //Init of DW1000 Failed
    printf("ERROR: DW1000 initialization failed!\r\n");
    while (1) {};
  }

  // Set SPI to 8MHz clock
  port_set_dw1000_fastrate();

  /* Configure DW1000. */
  dwt_configure(&config);

  /* Apply default antenna delay value. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set preamble timeout */
  dwt_setrxtimeout(0);  // No receive timeout for continuous listening

  /* Start with Uplink mode A (TS only) */
  set_current_uplink_mode(UPLINK_MODE_A);
  printf("Starting with Uplink mode A (TS only, %d bytes)\r\n", UPLINK_A_PAYLOAD_SIZE);
  
  //-------------dw1000  ini------end---------------------------	
  // IF WE GET HERE THEN THE LEDS WILL BLINK

  #ifdef USE_FREERTOS		
    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();	

    while(1) 
    {};
  #else

    // No RTOS task here so just call the main loop here.
    // Loop forever responding to ranging requests.
    while (1)
    {
      ss_resp_run();
    }

  #endif
}

/* Uplink mode switch monitor task */
static void uplink_mode_switch_monitor_task(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    
    while (1)
    {
        uplink_mode_t pending = get_pending_uplink_mode_switch();
        uplink_mode_t current = get_current_uplink_mode();
        
        if (pending != current)
        {
            printf("Main: Detected uplink mode switch request from %d to %d\r\n", current, pending);
            printf("Main: Uplink mode switched to mode %d\r\n", pending);
            clear_uplink_mode_switch_pending();
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  /* Check every second */
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. Experiment 3: 上报内容大小（回传侧功耗/带宽）
 *    - A组：uplink.payload=TS（仅时间戳，8字节）
 *    - B组：uplink.payload=TS+CIR_s（时间戳+CIR摘要，104字节，其中CIR_s=96字节）
 *    - 每个模式运行150秒，中间有10秒切换时间
 *    - 记录指标：uplink_bytes, bandwidth, tx_count（用于评估功耗和带宽）
 *
 ****************************************************************************************************************************************************/


