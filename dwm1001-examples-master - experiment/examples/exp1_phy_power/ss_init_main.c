/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.c
*  @brief   Single-sided two-way ranging (SS TWR) initiator example code
*
*           Experiment 1: PHY档位对功耗（快 vs 稳）
*           This code implements SS TWR with two PHY modes switching and data recording
*
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "ss_init_main.h"

#define APP_NAME "SS TWR INIT Exp1 v1.0"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 100

/* Experiment timing parameters */
#define EXPERIMENT_DURATION_MS (150 * 1000)  /* 150 seconds per mode */
#define MODE_SWITCH_DELAY_MS (10 * 1000)     /* 10 seconds for mode switching */
#define RECORD_INTERVAL_MS 1000              /* Record data every 1 second */

/* Frames used in the ranging process. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor. */
#define UUS_TO_DWT_TIME 65536

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance */
static double tof;
static double distance;

/* Current PHY mode */
static volatile phy_mode_t current_mode = MODE_FAST;
static volatile phy_mode_t pending_mode = MODE_FAST;
static volatile uint8_t mode_switch_pending = 0;

/* Experiment timing */
static uint32_t experiment_start_time_ms = 0;
static uint32_t mode_start_time_ms = 0;
static uint32_t last_record_time_ms = 0;

/* Transaction Counters */
static volatile uint32_t tx_count = 0;
static volatile uint32_t rx_count = 0;
static volatile uint32_t loss_count = 0;
static volatile uint32_t uplink_bytes_total = 0;

/* Statistics for current mode */
static struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t loss_count;
    uint32_t uplink_bytes;
    double distance_sum;
    double distance_count;
    int32_t rssi_sum;
    uint32_t rssi_count;
    int16_t snr_sum;
    uint32_t snr_count;
} mode_stats[2] = {0};

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);
static uint32_t get_system_time_ms(void);
static void check_mode_switch(void);
static void reset_mode_statistics(phy_mode_t mode);
static void update_statistics(phy_mode_t mode, int32_t rssi, int16_t snr, double distance, uint32_t bytes);

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_system_time_ms()
*
* @brief Get system time in milliseconds (simplified version using FreeRTOS tick count)
*
* @return system time in milliseconds
*/
static uint32_t get_system_time_ms(void)
{
#ifdef USE_FREERTOS
    return (xTaskGetTickCount() * portTICK_PERIOD_MS);
#else
    // For non-RTOS, you would need to implement a timer-based counter
    static uint32_t counter = 0;
    counter += RNG_DELAY_MS;  // Approximate increment
    return counter;
#endif
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn check_mode_switch()
*
* @brief Check if it's time to switch PHY mode
*/
static void check_mode_switch(void)
{
    uint32_t current_time = get_system_time_ms();
    uint32_t elapsed = current_time - mode_start_time_ms;
    
    if (elapsed >= EXPERIMENT_DURATION_MS)
    {
        /* Time to switch mode */
        phy_mode_t next_mode = (current_mode == MODE_FAST) ? MODE_STABLE : MODE_FAST;
        
        printf("\r\n========================================\r\n");
        printf("Mode switch triggered after %lu ms\r\n", elapsed);
        printf("Switching from mode %d to mode %d\r\n", current_mode, next_mode);
        printf("Waiting %d seconds for mode transition...\r\n", MODE_SWITCH_DELAY_MS / 1000);
        printf("========================================\r\n");
        
        /* Print statistics for current mode */
        print_statistics();
        
        /* Reset statistics for next mode */
        reset_mode_statistics(next_mode);
        
        /* Set pending mode switch flag - main.c will handle actual PHY reconfiguration */
        pending_mode = next_mode;
        mode_switch_pending = 1;
        
        printf("Mode switch requested: %d (pending PHY reconfiguration)\r\n", next_mode);
        
        /* Wait a short time for main.c monitor task to detect and reconfigure PHY */
        vTaskDelay(pdMS_TO_TICKS(500));  /* Give 500ms for PHY reconfiguration */
        
        /* Wait for remaining mode switch delay period */
        vTaskDelay(pdMS_TO_TICKS(MODE_SWITCH_DELAY_MS - 500));
        
        /* After delay, update current mode (PHY should already be reconfigured by main.c) */
        current_mode = next_mode;
        mode_start_time_ms = get_system_time_ms();
        
        /* Clear pending flag if not already cleared by main.c */
        if (mode_switch_pending) {
            mode_switch_pending = 0;
        }
        
        printf("Mode switched to: %d\r\n", current_mode);
        printf("Starting new measurement period...\r\n");
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn reset_mode_statistics()
*
* @brief Reset statistics for a given mode
*/
static void reset_mode_statistics(phy_mode_t mode)
{
    if (mode < 2) {
        memset(&mode_stats[mode], 0, sizeof(mode_stats[mode]));
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn update_statistics()
*
* @brief Update statistics for current mode
*/
static void update_statistics(phy_mode_t mode, int32_t rssi, int16_t snr, double distance, uint32_t bytes)
{
    if (mode >= 2) return;
    
    mode_stats[mode].tx_count = tx_count;
    mode_stats[mode].rx_count = rx_count;
    mode_stats[mode].loss_count = loss_count;
    mode_stats[mode].uplink_bytes += bytes;
    
    if (distance > 0) {
        mode_stats[mode].distance_sum += distance;
        mode_stats[mode].distance_count++;
    }
    
    if (rssi != 0) {
        mode_stats[mode].rssi_sum += rssi;
        mode_stats[mode].rssi_count++;
    }
    
    if (snr != 0) {
        mode_stats[mode].snr_sum += snr;
        mode_stats[mode].snr_count++;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn set_current_phy_mode()
*
* @brief Set current PHY mode (called from main.c after reconfiguration)
*/
void set_current_phy_mode(phy_mode_t mode)
{
    current_mode = mode;
    mode_start_time_ms = get_system_time_ms();
    reset_mode_statistics(mode);
    printf("PHY mode set to: %d at time %lu ms\r\n", mode, mode_start_time_ms);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_current_phy_mode()
*
* @brief Get current PHY mode
*/
phy_mode_t get_current_phy_mode(void)
{
    return current_mode;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_pending_mode_switch()
*
* @brief Get pending mode switch (if any)
*/
phy_mode_t get_pending_mode_switch(void)
{
    if (mode_switch_pending) {
        return pending_mode;
    }
    return current_mode;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn clear_mode_switch_pending()
*
* @brief Clear the pending mode switch flag
*/
void clear_mode_switch_pending(void)
{
    mode_switch_pending = 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn print_statistics()
*
* @brief Print statistics for current mode
*/
void print_statistics(void)
{
    phy_mode_t mode = current_mode;
    const char *mode_name = (mode == MODE_FAST) ? "FAST" : "STABLE";
    
    printf("\r\n========================================\r\n");
    printf("Statistics for %s mode:\r\n", mode_name);
    printf("========================================\r\n");
    printf("TX Count: %lu\r\n", mode_stats[mode].tx_count);
    printf("RX Count: %lu\r\n", mode_stats[mode].rx_count);
    printf("Loss Count: %lu\r\n", mode_stats[mode].loss_count);
    printf("Loss Rate: %.2f%%\r\n", 
           mode_stats[mode].tx_count > 0 ? 
           (100.0f * mode_stats[mode].loss_count / mode_stats[mode].tx_count) : 0.0f);
    printf("Uplink Bytes: %lu\r\n", mode_stats[mode].uplink_bytes);
    
    if (mode_stats[mode].distance_count > 0) {
        printf("Avg Distance: %.3f m\r\n", 
               mode_stats[mode].distance_sum / mode_stats[mode].distance_count);
    }
    
    if (mode_stats[mode].rssi_count > 0) {
        printf("Avg RSSI: %ld dBm\r\n", 
               mode_stats[mode].rssi_sum / mode_stats[mode].rssi_count);
    }
    
    if (mode_stats[mode].snr_count > 0) {
        printf("Avg SNR: %d dB\r\n", 
               mode_stats[mode].snr_sum / mode_stats[mode].snr_count);
    }
    printf("========================================\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn record_measurement()
*
* @brief Record a measurement (can be extended to write to file or send via UART)
*/
void record_measurement(const measurement_record_t *record)
{
    const char *mode_name = (record->mode == MODE_FAST) ? "FAST" : "STABLE";
    
    printf("[%lu ms] %s: TX=%lu RX=%lu Loss=%lu RSSI=%ld SNR=%d Dist=%.3f m Bytes=%lu\r\n",
           record->timestamp_ms,
           mode_name,
           record->tx_count,
           record->rx_count,
           record->loss_count,
           record->rssi,
           record->snr,
           record->distance,
           record->uplink_bytes);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_get_ts()
*
* @brief Read a given timestamp value from the response message
*/
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    *ts += ts_field[i] << (i * 8);
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn ss_init_run()
*
* @brief Application entry point for SS TWR ranging exchange
*
* @return none
*/
int ss_init_run(void)
{
    uint32_t current_time = get_system_time_ms();
    
    /* Initialize timing on first call */
    if (experiment_start_time_ms == 0) {
        experiment_start_time_ms = current_time;
        mode_start_time_ms = current_time;
        last_record_time_ms = current_time;
    }
    
    /* Check if mode switch is needed */
    check_mode_switch();
    
    /* Record data periodically */
    if (current_time - last_record_time_ms >= RECORD_INTERVAL_MS) {
        measurement_record_t record = {0};
        record.timestamp_ms = current_time;
        record.mode = current_mode;
        record.tx_count = tx_count;
        record.rx_count = rx_count;
        record.loss_count = loss_count;
        record.uplink_bytes = uplink_bytes_total;
        record.valid = 0;
        
        record_measurement(&record);
        last_record_time_ms = current_time;
    }

    /* Write frame data to DW1000 and prepare transmission. */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

    /* Start transmission */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    tx_count++;
    uplink_bytes_total += sizeof(tx_poll_msg);

    /* Poll for reception of a frame or error/timeout. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {};

    /* Increment frame sequence number after transmission */
    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG)
    {
        uint32 frame_len;
        int32 rssi = 0;
        int16 snr = 0;
        uint16 rxpacc = 0;

        /* Clear good RX frame event */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
   
        if (frame_len <= RX_BUF_LEN)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        /* Read RX quality metrics */
        rssi = (int32_t)dwt_readrssi();
        rxpacc = dwt_read16bitreg(RX_FQUAL_ID) & RX_FQUAL_RXPACC_MASK;
        snr = (int16_t)((dwt_read16bitreg(RX_FQUAL_ID) >> 8) & 0xFF);

        /* Check that the frame is the expected response */
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            rx_count++;
            uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
            int32 rtd_init, rtd_resp;
            float clockOffsetRatio;

            /* Retrieve poll transmission and response reception timestamps */
            poll_tx_ts = dwt_readtxtimestamplo32();
            resp_rx_ts = dwt_readrxtimestamplo32();

            /* Read carrier integrator value and calculate clock offset ratio */
            clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);

            /* Get timestamps embedded in response message */
            resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
            resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

            /* Compute time of flight and distance */
            rtd_init = resp_rx_ts - poll_tx_ts;
            rtd_resp = resp_tx_ts - poll_rx_ts;

            tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS;
            distance = tof * SPEED_OF_LIGHT;
            
            /* Update statistics */
            update_statistics(current_mode, rssi, snr, distance, frame_len);
            
            /* Print detailed info periodically */
            if (rx_count % 10 == 0) {
                printf("[%lu] RX#%lu: Dist=%.3fm RSSI=%ld SNR=%d RXPACC=%d\r\n",
                       current_time, rx_count, distance, rssi, snr, rxpacc);
            }
        }
    }
    else
    {
        /* RX error or timeout */
        loss_count++;
        
        /* Clear RX error/timeout events */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        /* Reset RX to properly reinitialise LDE operation */
        dwt_rxreset();
    }

    return 1;
}

/**@brief SS TWR Initiator task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_initiator_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    dwt_setleds(DWT_LEDS_ENABLE);

    printf("SS TWR Initiator task started\r\n");
    printf("Experiment will run for %d seconds per mode\r\n", EXPERIMENT_DURATION_MS / 1000);
    printf("Mode switch delay: %d seconds\r\n", MODE_SWITCH_DELAY_MS / 1000);

    while (true)
    {
        ss_init_run();
        /* Delay a task for a given number of ticks */
        vTaskDelay(RNG_DELAY_MS);
    }
}

/*****************************************************************************************************************************************************
* NOTES:
*
* 1. Experiment 1: PHY档位对功耗（快 vs 稳）
*    - Records: rssi, rxpacc, snr, loss, uplink bytes, distance
*    - Each mode runs for 150 seconds
*    - 10 second delay between mode switches
*    - Statistics printed at mode switch
*
****************************************************************************************************************************************************/

