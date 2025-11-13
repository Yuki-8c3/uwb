/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.c
*  @brief   Single-sided two-way ranging (SS TWR) initiator example code
*
*           Experiment 4: 标签上报频度（能量/次定位）
*           This code implements SS TWR with two tag reporting rates and energy per localization tracking
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

#define APP_NAME "SS TWR INIT Exp4 v1.0"

/* Tag reporting rates (Hz) */
#define TAG_RATE_A_HZ  2
#define TAG_RATE_B_HZ  10

/* Corresponding delays in milliseconds */
#define TAG_RATE_A_DELAY_MS  (1000 / TAG_RATE_A_HZ)   /* 500 ms */
#define TAG_RATE_B_DELAY_MS  (1000 / TAG_RATE_B_HZ)   /* 100 ms */

/* Experiment timing parameters */
#define EXPERIMENT_DURATION_MS (150 * 1000)
#define MODE_SWITCH_DELAY_MS (10 * 1000)
#define RECORD_INTERVAL_MS 1000

/* Frames used in the ranging process */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

static uint8 frame_seq_nb = 0;
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];
static uint32 status_reg = 0;

#define UUS_TO_DWT_TIME 65536
#define SPEED_OF_LIGHT 299702547

static double tof;
static double distance;

/* Current tag rate mode */
static volatile tag_rate_mode_t current_mode = TAG_RATE_MODE_B;
static volatile tag_rate_mode_t pending_mode = TAG_RATE_MODE_B;
static volatile uint8_t mode_switch_pending = 0;

/* Experiment timing */
static uint32_t experiment_start_time_ms = 0;
static uint32_t mode_start_time_ms = 0;
static uint32_t last_record_time_ms = 0;
static uint32_t last_localization_time_ms = 0;

/* Transaction Counters */
static volatile uint32_t tx_count = 0;
static volatile uint32_t rx_count = 0;
static volatile uint32_t localization_count = 0;

/* Statistics for current mode */
static struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t localization_count;
    uint32_t total_time_ms;
    double distance_sum;
    uint32_t distance_count;
    int32_t rssi_sum;
    uint32_t rssi_count;
    int16_t snr_sum;
    uint32_t snr_count;
    /* Energy estimation (simplified: based on TX/RX counts and time) */
    uint32_t estimated_energy_units;  /* Relative energy units */
} mode_stats[2] = {0};

/* Declaration of static functions */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);
static uint32_t get_system_time_ms(void);
static void check_mode_switch(void);
static void reset_mode_statistics(tag_rate_mode_t mode);
static void update_statistics(tag_rate_mode_t mode, int32_t rssi, int16_t snr, double distance);

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_system_time_ms()
*/
static uint32_t get_system_time_ms(void)
{
#ifdef USE_FREERTOS
    return (xTaskGetTickCount() * portTICK_PERIOD_MS);
#else
    static uint32_t counter = 0;
    counter += TAG_RATE_B_DELAY_MS;
    return counter;
#endif
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_tag_rate_delay_ms()
*/
uint32_t get_tag_rate_delay_ms(tag_rate_mode_t mode)
{
    if (mode == TAG_RATE_MODE_A) {
        return TAG_RATE_A_DELAY_MS;
    } else {
        return TAG_RATE_B_DELAY_MS;
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
        tag_rate_mode_t next_mode = (current_mode == TAG_RATE_MODE_A) ? TAG_RATE_MODE_B : TAG_RATE_MODE_A;
        
        printf("\r\n========================================\r\n");
        printf("Mode switch triggered after %lu ms\r\n", elapsed);
        printf("Switching from mode %d to mode %d\r\n", current_mode, next_mode);
        printf("Waiting %d seconds for mode transition...\r\n", MODE_SWITCH_DELAY_MS / 1000);
        printf("========================================\r\n");
        
        print_statistics();
        reset_mode_statistics(next_mode);
        
        pending_mode = next_mode;
        mode_switch_pending = 1;
        
        printf("Tag rate mode switch requested: %d (%d Hz)\r\n", 
               next_mode, (next_mode == TAG_RATE_MODE_A) ? TAG_RATE_A_HZ : TAG_RATE_B_HZ);
        
        vTaskDelay(pdMS_TO_TICKS(500));
        vTaskDelay(pdMS_TO_TICKS(MODE_SWITCH_DELAY_MS - 500));
        
        current_mode = next_mode;
        mode_start_time_ms = get_system_time_ms();
        last_localization_time_ms = get_system_time_ms();
        
        if (mode_switch_pending) {
            mode_switch_pending = 0;
        }
        
        printf("Tag rate mode switched to: %d\r\n", current_mode);
        printf("Starting new measurement period...\r\n");
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn reset_mode_statistics()
*/
static void reset_mode_statistics(tag_rate_mode_t mode)
{
    if (mode < 2) {
        memset(&mode_stats[mode], 0, sizeof(mode_stats[mode]));
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn update_statistics()
*/
static void update_statistics(tag_rate_mode_t mode, int32_t rssi, int16_t snr, double distance)
{
    if (mode >= 2) return;
    
    mode_stats[mode].tx_count = tx_count;
    mode_stats[mode].rx_count = rx_count;
    mode_stats[mode].localization_count = localization_count;
    
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
    
    /* Estimate energy: simplified model based on TX/RX operations */
    /* Each TX/RX cycle consumes energy, accumulate over time */
    mode_stats[mode].estimated_energy_units += 1;  /* Simplified: 1 unit per successful localization */
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn set_current_tag_rate_mode()
*/
void set_current_tag_rate_mode(tag_rate_mode_t mode)
{
    current_mode = mode;
    mode_start_time_ms = get_system_time_ms();
    last_localization_time_ms = get_system_time_ms();
    reset_mode_statistics(mode);
    printf("Tag rate mode set to: %d (%d Hz) at time %lu ms\r\n", 
           mode, (mode == TAG_RATE_MODE_A) ? TAG_RATE_A_HZ : TAG_RATE_B_HZ, mode_start_time_ms);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_current_tag_rate_mode()
*/
tag_rate_mode_t get_current_tag_rate_mode(void)
{
    return current_mode;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_pending_tag_rate_mode_switch()
*/
tag_rate_mode_t get_pending_tag_rate_mode_switch(void)
{
    if (mode_switch_pending) {
        return pending_mode;
    }
    return current_mode;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn clear_tag_rate_mode_switch_pending()
*/
void clear_tag_rate_mode_switch_pending(void)
{
    mode_switch_pending = 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn print_statistics()
*/
void print_statistics(void)
{
    tag_rate_mode_t mode = current_mode;
    const char *mode_name = (mode == TAG_RATE_MODE_A) ? "A (2Hz)" : "B (10Hz)";
    uint32_t rate_hz = (mode == TAG_RATE_MODE_A) ? TAG_RATE_A_HZ : TAG_RATE_B_HZ;
    
    printf("\r\n========================================\r\n");
    printf("Statistics for Tag Rate mode %s:\r\n", mode_name);
    printf("Reporting rate: %d Hz\r\n", rate_hz);
    printf("========================================\r\n");
    printf("TX Count: %lu\r\n", mode_stats[mode].tx_count);
    printf("RX Count: %lu\r\n", mode_stats[mode].rx_count);
    printf("Localization Count: %lu\r\n", mode_stats[mode].localization_count);
    
    if (mode_stats[mode].localization_count > 0) {
        uint32_t elapsed = get_system_time_ms() - mode_start_time_ms;
        double avg_power_units = (double)mode_stats[mode].estimated_energy_units / (elapsed / 1000.0);
        double energy_per_localization = (double)mode_stats[mode].estimated_energy_units / mode_stats[mode].localization_count;
        
        printf("Avg Power (relative units): %.2f units/sec\r\n", avg_power_units);
        printf("Energy per Localization: %.2f units\r\n", energy_per_localization);
        printf("Actual Rate: %.2f Hz\r\n", (double)mode_stats[mode].localization_count / (elapsed / 1000.0));
    }
    
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
*/
void record_measurement(const measurement_record_t *record)
{
    const char *mode_name = (record->mode == TAG_RATE_MODE_A) ? "A(2Hz)" : "B(10Hz)";
    
    printf("[%lu ms] TAG_%s: TX=%lu RX=%lu Loc=%lu Dist=%.3f m Energy/Loc=%lu\r\n",
           record->timestamp_ms,
           mode_name,
           record->tx_count,
           record->rx_count,
           record->localization_count,
           record->distance,
           record->energy_per_localization);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_get_ts()
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
*/
int ss_init_run(void)
{
    uint32_t current_time = get_system_time_ms();
    
    if (experiment_start_time_ms == 0) {
        experiment_start_time_ms = current_time;
        mode_start_time_ms = current_time;
        last_record_time_ms = current_time;
        last_localization_time_ms = current_time;
    }
    
    check_mode_switch();
    
    /* Check if enough time has passed for next localization based on current rate */
    uint32_t delay_ms = get_tag_rate_delay_ms(current_mode);
    if (current_time - last_localization_time_ms < delay_ms) {
        /* Not time yet, skip this cycle */
        return 1;
    }
    
    if (current_time - last_record_time_ms >= RECORD_INTERVAL_MS) {
        measurement_record_t record = {0};
        record.timestamp_ms = current_time;
        record.mode = current_mode;
        record.tx_count = tx_count;
        record.rx_count = rx_count;
        record.localization_count = localization_count;
        record.distance = distance;
        if (localization_count > 0) {
            record.energy_per_localization = mode_stats[current_mode].estimated_energy_units / localization_count;
        }
        record.valid = 0;
        
        record_measurement(&record);
        last_record_time_ms = current_time;
    }

    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    tx_count++;

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {};

    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG)
    {
        uint32 frame_len;
        int32 rssi = 0;
        int16 snr = 0;
        uint16 rxpacc = 0;

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
   
        if (frame_len <= RX_BUF_LEN)
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        rssi = (int32_t)dwt_readrssi();
        rxpacc = dwt_read16bitreg(RX_FQUAL_ID) & RX_FQUAL_RXPACC_MASK;
        snr = (int16_t)((dwt_read16bitreg(RX_FQUAL_ID) >> 8) & 0xFF);

        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            rx_count++;
            localization_count++;
            last_localization_time_ms = current_time;
            
            uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
            int32 rtd_init, rtd_resp;
            float clockOffsetRatio;

            poll_tx_ts = dwt_readtxtimestamplo32();
            resp_rx_ts = dwt_readrxtimestamplo32();

            clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);

            resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
            resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

            rtd_init = resp_rx_ts - poll_tx_ts;
            rtd_resp = resp_tx_ts - poll_rx_ts;

            tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS;
            distance = tof * SPEED_OF_LIGHT;
            
            update_statistics(current_mode, rssi, snr, distance);
            
            if (localization_count % 10 == 0) {
                printf("[%lu] Loc#%lu: Dist=%.3fm RSSI=%ld SNR=%d Rate=%dHz\r\n",
                       current_time, localization_count, distance, rssi, snr,
                       (current_mode == TAG_RATE_MODE_A) ? TAG_RATE_A_HZ : TAG_RATE_B_HZ);
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

/**@brief SS TWR Initiator task entry function */
void ss_initiator_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);

    dwt_setleds(DWT_LEDS_ENABLE);

    printf("SS TWR Initiator task started\r\n");
    printf("Experiment will run for %d seconds per tag rate mode\r\n", EXPERIMENT_DURATION_MS / 1000);
    printf("Mode switch delay: %d seconds\r\n", MODE_SWITCH_DELAY_MS / 1000);
    printf("Tag rate mode A: %d Hz (delay %d ms)\r\n", TAG_RATE_A_HZ, TAG_RATE_A_DELAY_MS);
    printf("Tag rate mode B: %d Hz (delay %d ms)\r\n", TAG_RATE_B_HZ, TAG_RATE_B_DELAY_MS);

    while (true)
    {
        ss_init_run();
        /* Use dynamic delay based on current mode */
        vTaskDelay(get_tag_rate_delay_ms(get_current_tag_rate_mode()));
    }
}

/*****************************************************************************************************************************************************
* NOTES:
*
* 1. Experiment 4: 标签上报频度（能量/次定位）
*    - Records: localization_count, energy_per_localization, avg_power
*    - Each mode runs for 150 seconds
*    - 10 second delay between mode switches
*    - Statistics printed at mode switch
*    - A组：r_tag_hz=2Hz
*    - B组：r_tag_hz=10Hz（基线）
*
****************************************************************************************************************************************************/


