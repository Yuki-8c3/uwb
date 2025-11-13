/*! ----------------------------------------------------------------------------
*  @file    ss_resp_main.h
*  @brief   Single-sided two-way ranging (SS TWR) responder example code header
*
*           Experiment 5: 同步频率（锚点后台功耗）
*
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/

#ifndef SS_RESP_MAIN_H
#define SS_RESP_MAIN_H

#include "deca_types.h"

/* Sync frequency mode type */
typedef enum {
    SYNC_MODE_A = 0,  /* A组：sync.period_s=1s（基线） */
    SYNC_MODE_B = 1   /* B组：sync.period_s=2s（放宽） */
} sync_mode_t;

/* Data recording structure */
typedef struct {
    uint32_t timestamp_ms;      /* Timestamp in milliseconds */
    sync_mode_t mode;            /* Current sync mode */
    int32_t rssi;               /* Received signal strength indicator */
    uint16_t rxpacc;            /* RX preamble accumulation counter */
    int16_t snr;                /* Signal to noise ratio */
    uint32_t rx_count;          /* Reception count */
    uint32_t tx_count;          /* Transmission count */
    uint32_t sync_count;        /* Sync event count (PPS+PTP) */
    int32_t ptp_offset_ns;      /* PTP offset in nanoseconds (simulated) */
    uint32_t pps_jitter_ns;     /* PPS jitter in nanoseconds (simulated) */
    uint32_t sync_energy_units; /* Sync-related energy (estimated) */
    uint8_t valid;              /* Valid measurement flag */
} measurement_record_t;

/* Function declarations */
int ss_resp_run(void);
void ss_responder_task_function (void * pvParameter);
void set_current_sync_mode(sync_mode_t mode);
sync_mode_t get_current_sync_mode(void);
sync_mode_t get_pending_sync_mode_switch(void);
void clear_sync_mode_switch_pending(void);
void record_measurement(const measurement_record_t *record);
void print_statistics(void);
uint32_t get_sync_period_ms(sync_mode_t mode);

#endif /* SS_RESP_MAIN_H */


