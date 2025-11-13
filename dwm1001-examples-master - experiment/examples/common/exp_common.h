/*! ----------------------------------------------------------------------------
*  @file    exp_common.h
*  @brief   Common experiment framework for all experiments
*
*           Provides baseline calibration, steady-state waiting, ABBA sequence, etc.
*
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/

#ifndef EXP_COMMON_H
#define EXP_COMMON_H

#include "deca_types.h"

/* Experiment phases */
typedef enum {
    EXP_PHASE_BASELINE = 0,    /* Baseline calibration: 60s idle */
    EXP_PHASE_STEADY = 1,      /* Steady-state wait: 30s */
    EXP_PHASE_MEASURE = 2,     /* Measurement: 120-180s */
    EXP_PHASE_COMPLETE = 3     /* Experiment complete */
} exp_phase_t;

/* ABBA sequence modes */
typedef enum {
    SEQ_MODE_A = 0,
    SEQ_MODE_B = 1
} seq_mode_t;

/* ABBA sequence order */
typedef enum {
    SEQ_ORDER_ABBA = 0,  /* A -> B -> B -> A */
    SEQ_ORDER_BAAB = 1   /* B -> A -> A -> B */
} seq_order_t;

/* Power measurement structure */
typedef struct {
    float i_avg_ma;          /* Average current in mA (from external measurement) */
    float voltage_v;          /* Voltage in V */
    float power_mw;           /* Power in mW (calculated: I_avg × V) */
    float energy_per_op;      /* Energy per operation (calculated: P / rate) */
    uint32_t measurement_time_ms;  /* Measurement duration */
} power_measurement_t;

/* Baseline calibration data */
typedef struct {
    float baseline_i_ma;      /* Baseline static current in mA */
    float baseline_v;         /* Baseline voltage in V */
    float baseline_power_mw;  /* Baseline power in mW */
    uint32_t calibration_time_ms;  /* Calibration duration (60s) */
} baseline_data_t;

/* Function declarations */
void exp_common_init(void);
exp_phase_t exp_get_current_phase(void);
void exp_set_phase(exp_phase_t phase);
uint32_t exp_get_phase_start_time_ms(void);
void exp_reset_phase_timer(void);

/* Baseline calibration */
void exp_start_baseline_calibration(void);
baseline_data_t* exp_get_baseline_data(void);
uint8_t exp_is_baseline_complete(void);

/* Steady-state wait */
void exp_start_steady_state_wait(void);
uint8_t exp_is_steady_state_complete(void);

/* Measurement phase */
void exp_start_measurement_phase(void);
uint8_t exp_is_measurement_complete(void);

/* ABBA sequence management */
void exp_init_abba_sequence(seq_order_t order);
seq_mode_t exp_get_next_mode(seq_mode_t current_mode, uint8_t iteration);
uint8_t exp_get_sequence_iteration(void);
void exp_reset_sequence(void);

/* Power calculation helpers */
void exp_calculate_power(power_measurement_t *measurement);
float exp_calculate_energy_per_tag(float power_mw, float tag_rate_hz);
float exp_calculate_energy_per_anchor(float power_mw, float effective_tag_rate_hz);

/* Voltage reading (from DW1000) */
float exp_read_voltage(void);

/* Current reading placeholder (actual measurement from external hardware) */
/* In real system, this would interface with current measurement hardware */
float exp_read_current_ma(void);  /* Returns 0 if not available, actual value from external measurement */

#endif /* EXP_COMMON_H */


