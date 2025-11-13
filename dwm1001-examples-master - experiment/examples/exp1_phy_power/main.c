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
 * Experiment 1: PHY档位对功耗（快 vs 稳）
 * A组（快）：6.8Mbps / PRF 64MHz / PL=64 / SFD=standard(8)
 * B组（稳）：850kbps / PRF 16MHz / PL=1024 / SFD=DW(16)
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
#include "ss_init_main.h"

/* Forward declaration */
static void mode_switch_monitor_task(void *pvParameter);

//-----------------dw1000----------------------------

/* A组（快）配置：基线配置 */
static dwt_config_t config_fast = {
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

/* B组（稳）配置 */
static dwt_config_t config_stable = {
    5,                /* Channel number. */
    DWT_PRF_16M,      /* Pulse repetition frequency: 16MHz */
    DWT_PLEN_1024,    /* Preamble length: 1024 symbols */
    DWT_PAC64,        /* Preamble acquisition chunk size: 64 (for PL=1024) */
    1,                /* TX preamble code (1-8 for 16MHz PRF) */
    1,                /* RX preamble code (1-8 for 16MHz PRF) */
    1,                /* 1 = non-standard SFD (DW, 16 symbols) */
    DWT_BR_850K,      /* Data rate: 850 kbps */
    DWT_PHRMODE_STD,  /* PHY header mode: standard */
    (1024 + 16 - 64)  /* SFD timeout (preamble length + 1 + SFD length - PAC size) */
};

/* Preamble timeout, in multiple of PAC size. */
#define PRE_TIMEOUT 1000

/* Delay between frames, in UWB microseconds. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 100 

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

TaskHandle_t  ss_initiator_task_handle;   /**< Reference to SS TWR Initiator FreeRTOS task. */
extern void ss_initiator_task_function (void * pvParameter);
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

  extern int ss_init_run(void);

#endif   // #ifdef USE_FREERTOS

/* Mode switch monitor task */
static void mode_switch_monitor_task(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    
    while (1)
    {
        phy_mode_t pending = get_pending_mode_switch();
        phy_mode_t current = get_current_phy_mode();
        
        if (pending != current)
        {
            printf("Main: Detected mode switch request from %d to %d\r\n", current, pending);
            switch_phy_mode(pending);
            clear_mode_switch_pending();
            printf("Main: PHY reconfigured to mode %d\r\n", pending);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  /* Check every second */
    }
}

/* Function to switch PHY configuration */
static void switch_phy_mode(phy_mode_t mode)
{
    dwt_config_t *config;
    const char *mode_name;
    
    if (mode == MODE_FAST) {
        config = &config_fast;
        mode_name = "FAST (6.8Mbps/64MHz/PL64)";
    } else {
        config = &config_stable;
        mode_name = "STABLE (850kbps/16MHz/PL1024)";
    }
    
    printf("\r\n========================================\r\n");
    printf("Switching to mode: %s\r\n", mode_name);
    printf("========================================\r\n");
    
    /* Configure DW1000 with new parameters */
    dwt_configure(config);
    
    /* Apply antenna delay values */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    
    /* Set preamble timeout */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(65000); // Maximum value timeout with DW1000 is 65ms
    
    printf("PHY mode switched successfully\r\n");
    printf("Config: DR=%d, PRF=%d, PL=%d, SFD=%d\r\n", 
           config->dataRate, config->prf, config->txPreambLength, config->nsSFD);
}

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

    /* Create task for SS TWR Initiator set to 2 */
    UNUSED_VARIABLE(xTaskCreate(ss_initiator_task_function, "SSTWR_INIT", configMINIMAL_STACK_SIZE + 200, NULL, 2, &ss_initiator_task_handle));
  #endif // #ifdef USE_FREERTOS
  
  //-------------dw1000  ini------------------------------------	

  /* Setup DW1000 IRQ pin */  
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); 		//irq
  
  /*Initialization UART*/
  boUART_Init ();
  printf("========================================\r\n");
  printf("Experiment 1: PHY Power Consumption\r\n");
  printf("Fast vs Stable Mode Comparison\r\n");
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

  /* Start with FAST mode (A组) */
  switch_phy_mode(MODE_FAST);
  set_current_phy_mode(MODE_FAST);
  
  //-------------dw1000  ini------end---------------------------	
  // IF WE GET HERE THEN THE LEDS WILL BLINK

  #ifdef USE_FREERTOS
    /* Create a task to monitor and handle mode switches */
    TaskHandle_t mode_switch_task_handle;
    UNUSED_VARIABLE(xTaskCreate(mode_switch_monitor_task, "MODE_SWITCH", configMINIMAL_STACK_SIZE + 200, NULL, 1, &mode_switch_task_handle));
		
    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();	

    while(1) 
    {};
  #else

    // No RTOS task here so just call the main loop here.
    // Loop forever responding to ranging requests.
    while (1)
    {
      ss_init_run();
    }

  #endif
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. Experiment 1: PHY档位对功耗（快 vs 稳）
 *    - A组（快）：6.8Mbps / PRF 64MHz / PL=64 / SFD=standard(8) - 基线配置
 *    - B组（稳）：850kbps / PRF 16MHz / PL=1024 / SFD=DW(16)
 *    - 每个模式运行150秒，中间有10秒切换时间
 *    - 记录指标：rssi, rxpacc, snr, loss, uplink bytes, 功耗
 *
 ****************************************************************************************************************************************************/

