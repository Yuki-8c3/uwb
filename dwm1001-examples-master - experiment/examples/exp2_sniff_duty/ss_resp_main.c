/*! ----------------------------------------------------------------------------
*  @file    ss_resp_main.c
*  @brief   Single-sided two-way ranging (SS TWR) responder example code
*
*           Experiment 2: 嗅探占空比（锚点省电开关）
*           This code implements SS TWR responder with two sniff modes switching and data recording
*
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include "sdk_config.h" 
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "ss_resp_main.h"

#define APP_NAME "SS TWR RESP Exp2 v1.0"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 80

/* Experiment timing parameters */
#define EXPERIMENT_DURATION_MS (150 * 1000)  /* 150 seconds per mode */
#define MODE_SWITCH_DELAY_MS (10 * 1000)     /* 10 seconds for mode switching */
#define RECORD_INTERVAL_MS 1000              /* Record data every 1 second */

/* Frames used in the ranging process. */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of the common part of the message */
#define ALL_MSG_COMMON_LEN 10

/* Index to access some of the fields in the frames */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

/* Frame sequence number */
static uint8 frame_seq_nb = 0;

/* Buffer to store received poll message */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu) conversion factor */
#define UUS_TO_DWT_TIME 65536

/* Delay from poll RX to response TX */
#define POLL_RX_TO_RESP_TX_DLY_UUS  1100

/* Timestamps */
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* Current sniff mode */
static volatile sniff_mode_t current_mode = SNIFF_MODE_A;
static volatile sniff_mode_t pending_mode = SNIFF_MODE_A;
static volatile uint8_t mode_switch_pending = 0;

/* Experiment timing */
static uint32_t experiment_start_time_ms = 0;
static uint32_t mode_start_time_ms = 0;
static uint32_t last_record_time_ms = 0;

/* Transaction Counters */
static volatile uint32_t rx_count = 0;
static volatile uint32_t tx_count = 0;
static volatile uint32_t timeout_count = 0;
static volatile uint32_t error_count = 0;
static volatile uint32_t downlink_bytes_total = 0;

/* Statistics for current mode */
static struct {
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t timeout_count;
    uint32_t error_count;
    uint32_t downlink_bytes;
    int32_t rssi_sum;
    uint32_t rssi_count;
    uint16_t rxpacc_sum;
    uint32_t rxpacc_count;
    int16_t snr_sum;
    uint32_t snr_count;
} mode_stats[2] = {0};

/* Declaration of static functions */
static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);
static uint32_t get_system_time_ms(void);
static void check_mode_switch(void);
static void reset_mode_statistics(sniff_mode_t mode);
static void update_statistics(sniff_mode_t mode, int32_t rssi, uint16_t rxpacc, int16_t snr, uint32_t bytes);

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_system_time_ms()
*
* @brief Get system time in milliseconds
*/
static uint32_t get_system_time_ms(void)
{
#ifdef USE_FREERTOS
    return (xTaskGetTickCount() * portTICK_PERIOD_MS);
#else
    static uint32_t counter = 0;
    counter += RNG_DELAY_MS;
    return counter;
#endif
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn check_mode_switch()
*
* @brief Check if it's time to switch sniff mode
*/
static void check_mode_switch(void)
{
    uint32_t current_time = get_system_time_ms();
    uint32_t elapsed = current_time - mode_start_time_ms;
    
    if (elapsed >= EXPERIMENT_DURATION_MS)
    {
        /* Time to switch mode */
        sniff_mode_t next_mode = (current_mode == SNIFF_MODE_A) ? SNIFF_MODE_B : SNIFF_MODE_A;
        
        printf("\r\n========================================\r\n");
        printf("Mode switch triggered after %lu ms\r\n", elapsed);
        printf("Switching from mode %d to mode %d\r\n", current_mode, next_mode);
        printf("Waiting %d seconds for mode transition...\r\n", MODE_SWITCH_DELAY_MS / 1000);
        printf("========================================\r\n");
        
        /* Print statistics for current mode */
        print_statistics();
        
        /* Reset statistics for next mode */
        reset_mode_statistics(next_mode);
        
        /* Set pending mode switch flag */
        pending_mode = next_mode;
        mode_switch_pending = 1;
        
        printf("Sniff mode switch requested: %d (pending PHY reconfiguration)\r\n", next_mode);
        
        /* Wait a short time for main.c monitor task to detect and reconfigure */
        vTaskDelay(pdMS_TO_TICKS(500));
        
        /* Wait for remaining mode switch delay period */
        vTaskDelay(pdMS_TO_TICKS(MODE_SWITCH_DELAY_MS - 500));
        
        /* After delay, update current mode */
        current_mode = next_mode;
        mode_start_time_ms = get_system_time_ms();
        
        /* Clear pending flag if not already cleared by main.c */
        if (mode_switch_pending) {
            mode_switch_pending = 0;
        }
        
        printf("Sniff mode switched to: %d\r\n", current_mode);
        printf("Starting new measurement period...\r\n");
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn reset_mode_statistics()
*
* @brief Reset statistics for a given mode
*/
static void reset_mode_statistics(sniff_mode_t mode)
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
static void update_statistics(sniff_mode_t mode, int32_t rssi, uint16_t rxpacc, int16_t snr, uint32_t bytes)
{
    if (mode >= 2) return;
    
    mode_stats[mode].rx_count = rx_count;
    mode_stats[mode].tx_count = tx_count;
    mode_stats[mode].timeout_count = timeout_count;
    mode_stats[mode].error_count = error_count;
    mode_stats[mode].downlink_bytes += bytes;
    
    if (rssi != 0) {
        mode_stats[mode].rssi_sum += rssi;
        mode_stats[mode].rssi_count++;
    }
    
    if (rxpacc != 0) {
        mode_stats[mode].rxpacc_sum += rxpacc;
        mode_stats[mode].rxpacc_count++;
    }
    
    if (snr != 0) {
        mode_stats[mode].snr_sum += snr;
        mode_stats[mode].snr_count++;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn set_current_sniff_mode()
*
* @brief Set current sniff mode
*/
void set_current_sniff_mode(sniff_mode_t mode)
{
    current_mode = mode;
    mode_start_time_ms = get_system_time_ms();
    reset_mode_statistics(mode);
    printf("Sniff mode set to: %d at time %lu ms\r\n", mode, mode_start_time_ms);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_current_sniff_mode()
*
* @brief Get current sniff mode
*/
sniff_mode_t get_current_sniff_mode(void)
{
    return current_mode;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_pending_sniff_mode_switch()
*
* @brief Get pending mode switch (if any)
*/
sniff_mode_t get_pending_sniff_mode_switch(void)
{
    if (mode_switch_pending) {
        return pending_mode;
    }
    return current_mode;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn clear_sniff_mode_switch_pending()
*
* @brief Clear the pending mode switch flag
*/
void clear_sniff_mode_switch_pending(void)
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
    sniff_mode_t mode = current_mode;
    const char *mode_name = (mode == SNIFF_MODE_A) ? "A (ont_pac=3, offt_us=25)" : "B (ont_pac=3, offt_us=50)";
    
    printf("\r\n========================================\r\n");
    printf("Statistics for SNIFF mode %s:\r\n", mode_name);
    printf("========================================\r\n");
    printf("RX Count: %lu\r\n", mode_stats[mode].rx_count);
    printf("TX Count: %lu\r\n", mode_stats[mode].tx_count);
    printf("Timeout Count: %lu\r\n", mode_stats[mode].timeout_count);
    printf("Error Count: %lu\r\n", mode_stats[mode].error_count);
    printf("Total Missed: %lu (timeouts + errors)\r\n", 
           mode_stats[mode].timeout_count + mode_stats[mode].error_count);
    printf("Downlink Bytes: %lu\r\n", mode_stats[mode].downlink_bytes);
    
    if (mode_stats[mode].rssi_count > 0) {
        printf("Avg RSSI: %ld dBm\r\n", 
               mode_stats[mode].rssi_sum / mode_stats[mode].rssi_count);
    }
    
    if (mode_stats[mode].rxpacc_count > 0) {
        printf("Avg RXPACC: %d\r\n", 
               mode_stats[mode].rxpacc_sum / mode_stats[mode].rxpacc_count);
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
* @brief Record a measurement
*/
void record_measurement(const measurement_record_t *record)
{
    const char *mode_name = (record->mode == SNIFF_MODE_A) ? "A" : "B";
    
    printf("[%lu ms] SNIFF_%s: RX=%lu TX=%lu TO=%lu ERR=%lu RSSI=%ld RXPACC=%d SNR=%d Bytes=%lu\r\n",
           record->timestamp_ms,
           mode_name,
           record->rx_count,
           record->tx_count,
           record->timeout_count,
           record->error_count,
           record->rssi,
           record->rxpacc,
           record->snr,
           record->downlink_bytes);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_rx_timestamp_u64()
*
* @brief Get the RX time-stamp in a 64-bit variable
*/
static uint64 get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_set_ts()
*
* @brief Fill a given timestamp field in the response message
*/
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
  int i;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn ss_resp_run()
*
* @brief Application entry point for SS TWR responder
*
* @return none
*/
int ss_resp_run(void)
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
        record.rx_count = rx_count;
        record.tx_count = tx_count;
        record.timeout_count = timeout_count;
        record.error_count = error_count;
        record.downlink_bytes = downlink_bytes_total;
        record.valid = 0;
        
        record_measurement(&record);
        last_record_time_ms = current_time;
    }

    /* Activate reception immediately (SNIFF mode is already configured) */
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* Poll for reception of a frame or error/timeout */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {};

    if (status_reg & SYS_STATUS_RXFCG)
    {
        uint32 frame_len;
        int32 rssi = 0;
        int16 snr = 0;
        uint16 rxpacc = 0;

        /* Clear good RX frame event */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        /* A frame has been received, read it into the local buffer */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
   
        if (frame_len <= RX_BUF_LEN)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        /* Read RX quality metrics */
        rssi = (int32_t)dwt_readrssi();
        rxpacc = dwt_read16bitreg(RX_FQUAL_ID) & RX_FQUAL_RXPACC_MASK;
        snr = (int16_t)((dwt_read16bitreg(RX_FQUAL_ID) >> 8) & 0xFF);

        /* Check that the frame is a poll sent by initiator */
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            uint32 resp_tx_time;
            int ret;

            rx_count++;

            /* Retrieve poll reception timestamp */
            poll_rx_ts = get_rx_timestamp_u64();

            /* Compute response message transmission time */
            resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_tx_time);

            /* Response TX timestamp is the transmission time we programmed plus the antenna delay */
            resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            /* Write all timestamps in the response message */
            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

            /* Write and send the response message */
            tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
            ret = dwt_starttx(DWT_START_TX_DELAYED);

            if (ret == DWT_SUCCESS)
            {
                /* Poll DW1000 until TX frame sent event set */
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                {};

                /* Clear TXFRS event */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                tx_count++;
                downlink_bytes_total += sizeof(tx_resp_msg);
                
                /* Update statistics */
                update_statistics(current_mode, rssi, rxpacc, snr, sizeof(tx_resp_msg));

                /* Increment frame sequence number */
                frame_seq_nb++;
                
                /* Print detailed info periodically */
                if (rx_count % 10 == 0) {
                    printf("[%lu] RX#%lu: RSSI=%ld RXPACC=%d SNR=%d\r\n",
                           current_time, rx_count, rssi, rxpacc, snr);
                }
            }
            else
            {
                /* TX failed, reset RX */
                dwt_rxreset();
            }
        }
    }
    else
    {
        /* RX error or timeout */
        if (status_reg & SYS_STATUS_ALL_RX_TO) {
            timeout_count++;
        }
        if (status_reg & SYS_STATUS_ALL_RX_ERR) {
            error_count++;
        }
        
        /* Clear RX error/timeout events */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        /* Reset RX to properly reinitialise LDE operation */
        dwt_rxreset();
    }

    return 1;
}

/**@brief SS TWR Responder task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_responder_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    dwt_setleds(DWT_LEDS_ENABLE);

    printf("SS TWR Responder task started\r\n");
    printf("Experiment will run for %d seconds per sniff mode\r\n", EXPERIMENT_DURATION_MS / 1000);
    printf("Mode switch delay: %d seconds\r\n", MODE_SWITCH_DELAY_MS / 1000);
    printf("SNIFF mode enabled - anchor will use duty-cycled RX\r\n");

    while (true)
    {
        ss_resp_run();
        /* Delay a task for a given number of ticks */
        vTaskDelay(RNG_DELAY_MS);
    }
}

/*****************************************************************************************************************************************************
* NOTES:
*
* 1. Experiment 2: 嗅探占空比（锚点省电开关）
*    - Records: rssi, rxpacc, snr, timeout_count, error_count, downlink_bytes
*    - Each mode runs for 150 seconds
*    - 10 second delay between mode switches
*    - Statistics printed at mode switch
*    - Timeout and error counts indicate missed packets (漏检)
*
****************************************************************************************************************************************************/


