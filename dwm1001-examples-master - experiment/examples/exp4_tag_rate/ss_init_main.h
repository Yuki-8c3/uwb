/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.h
*  @brief   Single-sided two-way ranging (SS TWR) initiator example code header
*
*           Experiment 4: 标签上报频度（能量/次定位）
*
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/

#ifndef SS_INIT_MAIN_H
#define SS_INIT_MAIN_H

#include "deca_types.h"

/* Tag reporting rate mode type */
typedef enum {
    TAG_RATE_MODE_A = 0,  /* A组：r_tag_hz=2Hz */
    TAG_RATE_MODE_B = 1   /* B组：r_tag_hz=10Hz（基线） */
} tag_rate_mode_t;

/* Data recording structure */
typedef struct {
    uint32_t timestamp_ms;      /* Timestamp in milliseconds */
    tag_rate_mode_t mode;        /* Current tag rate mode */
    int32_t rssi;               /* Received signal strength indicator */
    uint16_t rxpacc;            /* RX preamble accumulation counter */
    int16_t snr;                /* Signal to noise ratio */
    uint32_t tx_count;          /* Transmission count */
    uint32_t rx_count;          /* Reception count */
    uint32_t localization_count; /* Successful localization count */
    double distance;            /* Calculated distance */
    uint32_t energy_per_localization; /* Energy per localization (estimated) */
    uint8_t valid;              /* Valid measurement flag */
} measurement_record_t;

/* Function declarations */
int ss_init_run(void);
void ss_initiator_task_function (void * pvParameter);
void set_current_tag_rate_mode(tag_rate_mode_t mode);
tag_rate_mode_t get_current_tag_rate_mode(void);
tag_rate_mode_t get_pending_tag_rate_mode_switch(void);
void clear_tag_rate_mode_switch_pending(void);
void record_measurement(const measurement_record_t *record);
void print_statistics(void);
uint32_t get_tag_rate_delay_ms(tag_rate_mode_t mode);

#endif /* SS_INIT_MAIN_H */


