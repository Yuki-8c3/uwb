/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.h
*  @brief   Single-sided two-way ranging (SS TWR) initiator example code header
*
*           Experiment 1: PHY档位对功耗（快 vs 稳）
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

/* PHY mode type */
typedef enum {
    MODE_FAST = 0,      // A组（快）
    MODE_STABLE = 1     // B组（稳）
} phy_mode_t;

/* Data recording structure */
typedef struct {
    uint32_t timestamp_ms;      /* Timestamp in milliseconds */
    phy_mode_t mode;            /* Current PHY mode */
    int32_t rssi;               /* Received signal strength indicator */
    uint16_t rxpacc;            /* RX preamble accumulation counter */
    int16_t snr;                /* Signal to noise ratio */
    uint32_t tx_count;          /* Transmission count */
    uint32_t rx_count;          /* Reception count */
    uint32_t loss_count;        /* Packet loss count */
    uint32_t uplink_bytes;      /* Uplink bytes transmitted */
    double distance;            /* Calculated distance */
    uint8_t valid;              /* Valid measurement flag */
} measurement_record_t;

/* Function declarations */
int ss_init_run(void);
void ss_initiator_task_function (void * pvParameter);
void set_current_phy_mode(phy_mode_t mode);
phy_mode_t get_current_phy_mode(void);
phy_mode_t get_pending_mode_switch(void);  /* Returns next mode if switch is pending, otherwise returns current mode */
void clear_mode_switch_pending(void);      /* Clear the pending switch flag */
void record_measurement(const measurement_record_t *record);
void print_statistics(void);

#endif /* SS_INIT_MAIN_H */

