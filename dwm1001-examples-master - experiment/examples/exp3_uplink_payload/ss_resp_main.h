/*! ----------------------------------------------------------------------------
*  @file    ss_resp_main.h
*  @brief   Single-sided two-way ranging (SS TWR) responder example code header
*
*           Experiment 3: 上报内容大小（回传侧功耗/带宽）
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

/* Uplink payload mode type */
typedef enum {
    UPLINK_MODE_A = 0,  /* A组：仅时间戳（TS） */
    UPLINK_MODE_B = 1   /* B组：时间戳+CIR摘要（TS+CIR_s） */
} uplink_mode_t;

/* Data recording structure */
typedef struct {
    uint32_t timestamp_ms;      /* Timestamp in milliseconds */
    uplink_mode_t mode;          /* Current uplink mode */
    int32_t rssi;               /* Received signal strength indicator */
    uint16_t rxpacc;            /* RX preamble accumulation counter */
    int16_t snr;                /* Signal to noise ratio */
    uint32_t rx_count;          /* Reception count */
    uint32_t tx_count;          /* Transmission count (response frames) */
    uint32_t uplink_count;      /* Uplink transmission count */
    uint32_t uplink_bytes;      /* Total uplink bytes transmitted */
    uint32_t uplink_bytes_per_sec;  /* Uplink bytes per second (bandwidth) */
    uint8_t valid;              /* Valid measurement flag */
} measurement_record_t;

/* Function declarations */
int ss_resp_run(void);
void ss_responder_task_function (void * pvParameter);
void set_current_uplink_mode(uplink_mode_t mode);
uplink_mode_t get_current_uplink_mode(void);
uplink_mode_t get_pending_uplink_mode_switch(void);  /* Returns next mode if switch is pending */
void clear_uplink_mode_switch_pending(void);         /* Clear the pending switch flag */
void record_measurement(const measurement_record_t *record);
void print_statistics(void);
uint32_t get_uplink_payload_size(uplink_mode_t mode);

#endif /* SS_RESP_MAIN_H */


