/*! ----------------------------------------------------------------------------
*  @file    ss_resp_main.c
*  @brief   Single-sided two-way ranging (SS TWR) responder example code
*
*           Experiment 5: 同步频率（锚点后台功耗）
*           This code implements SS TWR responder with two sync frequencies and power tracking
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
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "ss_resp_main.h"

#define APP_NAME "SS TWR RESP Exp5 v1.0"

/* Sync periods in seconds */
#define SYNC_PERIOD_A_S  1
#define SYNC_PERIOD_B_S  2

/* Corresponding periods in milliseconds */
#define SYNC_PERIOD_A_MS  (SYNC_PERIOD_A_S * 1000)   /* 1000 ms */
#define SYNC_PERIOD_B_MS  (SYNC_PERIOD_B_S * 1000)   /* 2000 ms */

/* Inter-ranging delay period */
#define RNG_DELAY_MS 80

/* Experiment timing parameters */
#define EXPERIMENT_DURATION_MS (150 * 1000)
#define MODE_SWITCH_DELAY_MS (10 * 1000)
#define RECORD_INTERVAL_MS 1000

/* Frames used in the ranging process */
static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

static uint8 frame_seq_nb = 0;
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];
static uint32 status_reg = 0;

#define UUS_TO_DWT_TIME 65536

#define POLL_RX_TO_RESP_TX_DLY_UUS  1100

typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* Current sync mode */
static volatile sync_mode_t current_mode = SYNC_MODE_A;
static volatile sync_mode_t pending_mode = SYNC_MODE_A;
static volatile uint8_t mode_switch_pending = 0;

/* Experiment timing */
static uint32_t experiment_start_time_ms = 0;
static uint32_t mode_start_time_ms = 0;
static uint32_t last_record_time_ms = 0;
static uint32_t last_sync_time_ms = 0;

/* Transaction Counters */
static volatile uint32_t rx_count = 0;
static volatile uint32_t tx_count = 0;
static volatile uint32_t sync_count = 0;

/* Sync timing simulation (for PTP offset and PPS jitter) */
static int32_t ptp_offset_ns = 0;
static uint32_t pps_jitter_ns = 0;

/* Statistics for current mode */
static struct {
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t sync_count;
    uint32_t total_time_ms;
    int32_t ptp_offset_sum_ns;
    uint32_t ptp_offset_count;
    uint32_t pps_jitter_sum_ns;
    uint32_t pps_jitter_count;
    uint32_t sync_energy_units;
    int32_t rssi_sum;
    uint32_t rssi_count;
    int16_t snr_sum;
    uint32_t snr_count;
} mode_stats[2] = {0};

/* Declaration of static functions */
static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);
static uint32_t get_system_time_ms(void);
static void check_mode_switch(void);
static void reset_mode_statistics(sync_mode_t mode);
static void update_statistics(sync_mode_t mode, int32_t rssi, int16_t snr);
static void simulate_sync_event(sync_mode_t mode);

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_system_time_ms()
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
* @fn get_sync_period_ms()
*/
uint32_t get_sync_period_ms(sync_mode_t mode)
{
    if (mode == SYNC_MODE_A) {
        return SYNC_PERIOD_A_MS;
    } else {
        return SYNC_PERIOD_B_MS;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn simulate_sync_event()
*
* @brief Simulate PPS+PTP sync event (in real system, this would be actual hardware sync)
*/
static void simulate_sync_event(sync_mode_t mode)
{
    sync_count++;
    
    /* Simulate PTP offset: random variation around 0, typical range ±100ns */
    /* In real system, this would be measured from actual PTP sync */
    ptp_offset_ns = (int32_t)((rand() % 200) - 100);  /* -100 to +100 ns */
    
    /* Simulate PPS jitter: random variation, typical range 0-50ns */
    /* In real system, this would be measured from actual PPS signal */
    pps_jitter_ns = (uint32_t)(rand() % 50);  /* 0 to 50 ns */
    
    /* Estimate sync energy: each sync event consumes energy */
    if (mode < 2) {
        mode_stats[mode].sync_energy_units += 1;  /* Simplified: 1 unit per sync */
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn check_mode_switch()
*/
static void check_mode_switch(void)
{
    uint32_t current_time = get_system_time_ms();
    uint32_t elapsed = current_time - mode_start_time_ms;
    
    if (elapsed >= EXPERIMENT_DURATION_MS)
    {
        sync_mode_t next_mode = (current_mode == SYNC_MODE_A) ? SYNC_MODE_B : SYNC_MODE_A;
        
        printf("\r\n========================================\r\n");
        printf("Mode switch triggered after %lu ms\r\n", elapsed);
        printf("Switching from mode %d to mode %d\r\n", current_mode, next_mode);
        printf("Waiting %d seconds for mode transition...\r\n", MODE_SWITCH_DELAY_MS / 1000);
        printf("========================================\r\n");
        
        print_statistics();
        reset_mode_statistics(next_mode);
        
        pending_mode = next_mode;
        mode_switch_pending = 1;
        
        printf("Sync mode switch requested: %d (period: %d s)\r\n", 
               next_mode, (next_mode == SYNC_MODE_A) ? SYNC_PERIOD_A_S : SYNC_PERIOD_B_S);
        
        vTaskDelay(pdMS_TO_TICKS(500));
        vTaskDelay(pdMS_TO_TICKS(MODE_SWITCH_DELAY_MS - 500));
        
        current_mode = next_mode;
        mode_start_time_ms = get_system_time_ms();
        last_sync_time_ms = get_system_time_ms();
        
        if (mode_switch_pending) {
            mode_switch_pending = 0;
        }
        
        printf("Sync mode switched to: %d\r\n", current_mode);
        printf("Starting new measurement period...\r\n");
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn reset_mode_statistics()
*/
static void reset_mode_statistics(sync_mode_t mode)
{
    if (mode < 2) {
        memset(&mode_stats[mode], 0, sizeof(mode_stats[mode]));
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn update_statistics()
*/
static void update_statistics(sync_mode_t mode, int32_t rssi, int16_t snr)
{
    if (mode >= 2) return;
    
    mode_stats[mode].rx_count = rx_count;
    mode_stats[mode].tx_count = tx_count;
    mode_stats[mode].sync_count = sync_count;
    
    if (rssi != 0) {
        mode_stats[mode].rssi_sum += rssi;
        mode_stats[mode].rssi_count++;
    }
    
    if (snr != 0) {
        mode_stats[mode].snr_sum += snr;
        mode_stats[mode].snr_count++;
    }
    
    /* Update PTP offset and PPS jitter statistics */
    if (ptp_offset_ns != 0) {
        mode_stats[mode].ptp_offset_sum_ns += ptp_offset_ns;
        mode_stats[mode].ptp_offset_count++;
    }
    
    if (pps_jitter_ns != 0) {
        mode_stats[mode].pps_jitter_sum_ns += pps_jitter_ns;
        mode_stats[mode].pps_jitter_count++;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn set_current_sync_mode()
*/
void set_current_sync_mode(sync_mode_t mode)
{
    current_mode = mode;
    mode_start_time_ms = get_system_time_ms();
    last_sync_time_ms = get_system_time_ms();
    reset_mode_statistics(mode);
    printf("Sync mode set to: %d (period: %d s) at time %lu ms\r\n", 
           mode, (mode == SYNC_MODE_A) ? SYNC_PERIOD_A_S : SYNC_PERIOD_B_S, mode_start_time_ms);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_current_sync_mode()
*/
sync_mode_t get_current_sync_mode(void)
{
    return current_mode;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_pending_sync_mode_switch()
*/
sync_mode_t get_pending_sync_mode_switch(void)
{
    if (mode_switch_pending) {
        return pending_mode;
    }
    return current_mode;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn clear_sync_mode_switch_pending()
*/
void clear_sync_mode_switch_pending(void)
{
    mode_switch_pending = 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn print_statistics()
*/
void print_statistics(void)
{
    sync_mode_t mode = current_mode;
    const char *mode_name = (mode == SYNC_MODE_A) ? "A (1s)" : "B (2s)";
    uint32_t period_s = (mode == SYNC_MODE_A) ? SYNC_PERIOD_A_S : SYNC_PERIOD_B_S;
    
    printf("\r\n========================================\r\n");
    printf("Statistics for Sync mode %s:\r\n", mode_name);
    printf("Sync period: %d s\r\n", period_s);
    printf("========================================\r\n");
    printf("RX Count: %lu\r\n", mode_stats[mode].rx_count);
    printf("TX Count: %lu\r\n", mode_stats[mode].tx_count);
    printf("Sync Count: %lu\r\n", mode_stats[mode].sync_count);
    
    if (mode_stats[mode].sync_count > 0) {
        uint32_t elapsed = get_system_time_ms() - mode_start_time_ms;
        double sync_rate = (double)mode_stats[mode].sync_count / (elapsed / 1000.0);
        double sync_power_units = (double)mode_stats[mode].sync_energy_units / (elapsed / 1000.0);
        
        printf("Actual Sync Rate: %.2f Hz\r\n", sync_rate);
        printf("Sync Power (relative units): %.2f units/sec\r\n", sync_power_units);
    }
    
    if (mode_stats[mode].ptp_offset_count > 0) {
        int32_t avg_ptp_offset = mode_stats[mode].ptp_offset_sum_ns / mode_stats[mode].ptp_offset_count;
        printf("Avg PTP Offset: %ld ns\r\n", avg_ptp_offset);
    }
    
    if (mode_stats[mode].pps_jitter_count > 0) {
        uint32_t avg_pps_jitter = mode_stats[mode].pps_jitter_sum_ns / mode_stats[mode].pps_jitter_count;
        printf("Avg PPS Jitter: %lu ns\r\n", avg_pps_jitter);
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
*/
void record_measurement(const measurement_record_t *record)
{
    const char *mode_name = (record->mode == SYNC_MODE_A) ? "A(1s)" : "B(2s)";
    
    printf("[%lu ms] SYNC_%s: RX=%lu TX=%lu Sync=%lu PTP_off=%ld ns PPS_jit=%lu ns Energy=%lu\r\n",
           record->timestamp_ms,
           mode_name,
           record->rx_count,
           record->tx_count,
           record->sync_count,
           record->ptp_offset_ns,
           record->pps_jitter_ns,
           record->sync_energy_units);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_rx_timestamp_u64()
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
*/
int ss_resp_run(void)
{
    uint32_t current_time = get_system_time_ms();
    
    if (experiment_start_time_ms == 0) {
        experiment_start_time_ms = current_time;
        mode_start_time_ms = current_time;
        last_record_time_ms = current_time;
        last_sync_time_ms = current_time;
    }
    
    check_mode_switch();
    
    /* Check if it's time for sync event */
    uint32_t sync_period_ms = get_sync_period_ms(current_mode);
    if (current_time - last_sync_time_ms >= sync_period_ms) {
        simulate_sync_event(current_mode);
        last_sync_time_ms = current_time;
    }
    
    if (current_time - last_record_time_ms >= RECORD_INTERVAL_MS) {
        measurement_record_t record = {0};
        record.timestamp_ms = current_time;
        record.mode = current_mode;
        record.rx_count = rx_count;
        record.tx_count = tx_count;
        record.sync_count = sync_count;
        record.ptp_offset_ns = ptp_offset_ns;
        record.pps_jitter_ns = pps_jitter_ns;
        record.sync_energy_units = mode_stats[current_mode].sync_energy_units;
        record.valid = 0;
        
        record_measurement(&record);
        last_record_time_ms = current_time;
    }

    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {};

    if (status_reg & SYS_STATUS_RXFCG)
    {
        uint32 frame_len;
        int32 rssi = 0;
        int16 snr = 0;

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
   
        if (frame_len <= RX_BUF_LEN)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        rssi = (int32_t)dwt_readrssi();
        snr = (int16_t)((dwt_read16bitreg(RX_FQUAL_ID) >> 8) & 0xFF);

        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            uint32 resp_tx_time;
            int ret;

            rx_count++;

            poll_rx_ts = get_rx_timestamp_u64();

            resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_tx_time);

            resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
            resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

            tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
            dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
            dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
            ret = dwt_starttx(DWT_START_TX_DELAYED);

            if (ret == DWT_SUCCESS)
            {
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                {};

                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                tx_count++;
                update_statistics(current_mode, rssi, snr);

                frame_seq_nb++;
                
                if (tx_count % 10 == 0) {
                    printf("[%lu] TX#%lu: Sync#%lu PTP_off=%ld ns PPS_jit=%lu ns\r\n",
                           current_time, tx_count, sync_count, ptp_offset_ns, pps_jitter_ns);
                }
            }
            else
            {
                dwt_rxreset();
            }
        }
    }
    else
    {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        dwt_rxreset();
    }

    return 1;
}

/**@brief SS TWR Responder task entry function */
void ss_responder_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    dwt_setleds(DWT_LEDS_ENABLE);

    printf("SS TWR Responder task started\r\n");
    printf("Experiment will run for %d seconds per sync mode\r\n", EXPERIMENT_DURATION_MS / 1000);
    printf("Mode switch delay: %d seconds\r\n", MODE_SWITCH_DELAY_MS / 1000);
    printf("Sync mode A: %d s period\r\n", SYNC_PERIOD_A_S);
    printf("Sync mode B: %d s period\r\n", SYNC_PERIOD_B_S);
    printf("Note: PPS+PTP sync events are simulated for this experiment\r\n");

    while (true)
    {
        ss_resp_run();
        vTaskDelay(RNG_DELAY_MS);
    }
}

/*****************************************************************************************************************************************************
* NOTES:
*
* 1. Experiment 5: 同步频率（锚点后台功耗）
*    - Records: sync_count, ptp_offset_ns, pps_jitter_ns, sync_energy_units
*    - Each mode runs for 150 seconds
*    - 10 second delay between mode switches
*    - Statistics printed at mode switch
*    - A组：sync.period_s=1s（基线双源 PPS+PTP）
*    - B组：sync.period_s=2s（放宽）
*    - Note: PPS and PTP are simulated in this code. In real system, these would be actual hardware sync signals.
*
****************************************************************************************************************************************************/

