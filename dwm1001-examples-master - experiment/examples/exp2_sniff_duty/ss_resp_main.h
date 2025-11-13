/*! ----------------------------------------------------------------------------
*  @file    ss_resp_main.h
*  @brief   Single-sided two-way ranging (SS TWR) responder example code header
*
*           Experiment 2: 嗅探占空比（锚点省电开关）
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

/* Sniff mode type */
typedef enum {
    SNIFF_MODE_A = 0,  /* A组：ont_pac=3, offt_us=25 */
    SNIFF_MODE_B = 1   /* B组：ont_pac=3, offt_us=50 */
} sniff_mode_t;

/* Data recording structure */
typedef struct {
    uint32_t timestamp_ms;      /* Timestamp in milliseconds */
    sniff_mode_t mode;          /* Current sniff mode */
    int32_t rssi;               /* Received signal strength indicator */
    uint16_t rxpacc;            /* RX preamble accumulation counter */
    int16_t snr;                /* Signal to noise ratio */
    uint32_t rx_count;          /* Reception count */
    uint32_t tx_count;          /* Transmission count */
    uint32_t timeout_count;     /* RX timeout count (indicates missed packets) */
    uint32_t error_count;       /* RX error count */
    uint32_t downlink_bytes;    /* Downlink bytes transmitted */
    uint8_t valid;              /* Valid measurement flag */
} measurement_record_t;

/* Function declarations */
int ss_resp_run(void);
void ss_responder_task_function (void * pvParameter);
void set_current_sniff_mode(sniff_mode_t mode);
sniff_mode_t get_current_sniff_mode(void);
sniff_mode_t get_pending_sniff_mode_switch(void);  /* Returns next mode if switch is pending */
void clear_sniff_mode_switch_pending(void);         /* Clear the pending switch flag */
void record_measurement(const measurement_record_t *record);
void print_statistics(void);

#endif /* SS_RESP_MAIN_H */


