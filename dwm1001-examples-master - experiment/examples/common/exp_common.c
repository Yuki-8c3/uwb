/*! ----------------------------------------------------------------------------
*  @file    exp_common.c
*  @brief   Common experiment framework implementation
*
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/

#include "exp_common.h"
#include "deca_device_api.h"
#include <stdio.h>
#include <string.h>
#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

/* Timing constants */
#define BASELINE_DURATION_MS (60 * 1000)    /* 60 seconds */
#define STEADY_STATE_DURATION_MS (30 * 1000) /* 30 seconds */
#define MEASUREMENT_DURATION_MS (150 * 1000) /* 150 seconds (120-180s range) */

/* Current experiment state */
static exp_phase_t current_phase = EXP_PHASE_BASELINE;
static uint32_t phase_start_time_ms = 0;
static baseline_data_t baseline_data = {0};
static seq_order_t abba_order = SEQ_ORDER_ABBA;
static uint8_t sequence_iteration = 0;

/* Forward declarations */
static uint32_t get_system_time_ms(void);

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_system_time_ms()
*/
static uint32_t get_system_time_ms(void)
{
#ifdef USE_FREERTOS
    extern TickType_t xTaskGetTickCount(void);
    return (xTaskGetTickCount() * portTICK_PERIOD_MS);
#else
    static uint32_t counter = 0;
    counter += 100;
    return counter;
#endif
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_common_init()
*/
void exp_common_init(void)
{
    current_phase = EXP_PHASE_BASELINE;
    phase_start_time_ms = get_system_time_ms();
    memset(&baseline_data, 0, sizeof(baseline_data));
    sequence_iteration = 0;
    printf("Experiment framework initialized\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_get_current_phase()
*/
exp_phase_t exp_get_current_phase(void)
{
    return current_phase;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_set_phase()
*/
void exp_set_phase(exp_phase_t phase)
{
    current_phase = phase;
    phase_start_time_ms = get_system_time_ms();
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_get_phase_start_time_ms()
*/
uint32_t exp_get_phase_start_time_ms(void)
{
    return phase_start_time_ms;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_reset_phase_timer()
*/
void exp_reset_phase_timer(void)
{
    phase_start_time_ms = get_system_time_ms();
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_start_baseline_calibration()
*/
void exp_start_baseline_calibration(void)
{
    exp_set_phase(EXP_PHASE_BASELINE);
    printf("\r\n========================================\r\n");
    printf("BASELINE CALIBRATION PHASE\r\n");
    printf("System will be idle (no TX/RX) for 60 seconds\r\n");
    printf("Please measure static current (I_avg) during this period\r\n");
    printf("========================================\r\n");
    
    /* Read initial voltage */
    baseline_data.baseline_v = exp_read_voltage();
    baseline_data.calibration_time_ms = BASELINE_DURATION_MS;
    
    printf("Baseline voltage: %.3f V\r\n", baseline_data.baseline_v);
    printf("Waiting for baseline calibration to complete...\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_get_baseline_data()
*/
baseline_data_t* exp_get_baseline_data(void)
{
    return &baseline_data;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_is_baseline_complete()
*/
uint8_t exp_is_baseline_complete(void)
{
    uint32_t elapsed = get_system_time_ms() - phase_start_time_ms;
    
    if (elapsed >= BASELINE_DURATION_MS) {
        /* Read final voltage and calculate baseline power */
        baseline_data.baseline_v = exp_read_voltage();
        
        /* NOTE: baseline_i_ma should be set by external measurement */
        /* For now, we'll use a placeholder - in real system, this comes from current meter */
        baseline_data.baseline_i_ma = exp_read_current_ma();
        
        if (baseline_data.baseline_i_ma > 0) {
            baseline_data.baseline_power_mw = baseline_data.baseline_i_ma * baseline_data.baseline_v;
        }
        
        printf("\r\n========================================\r\n");
        printf("BASELINE CALIBRATION COMPLETE\r\n");
        printf("Baseline I_avg: %.3f mA\r\n", baseline_data.baseline_i_ma);
        printf("Baseline V: %.3f V\r\n", baseline_data.baseline_v);
        printf("Baseline P: %.3f mW\r\n", baseline_data.baseline_power_mw);
        printf("========================================\r\n");
        
        return 1;
    }
    
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_start_steady_state_wait()
*/
void exp_start_steady_state_wait(void)
{
    exp_set_phase(EXP_PHASE_STEADY);
    printf("\r\n========================================\r\n");
    printf("STEADY-STATE WAIT PHASE\r\n");
    printf("Waiting 30 seconds for system to reach steady state...\r\n");
    printf("========================================\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_is_steady_state_complete()
*/
uint8_t exp_is_steady_state_complete(void)
{
    uint32_t elapsed = get_system_time_ms() - phase_start_time_ms;
    
    if (elapsed >= STEADY_STATE_DURATION_MS) {
        printf("Steady-state wait complete. Starting measurement...\r\n");
        return 1;
    }
    
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_start_measurement_phase()
*/
void exp_start_measurement_phase(void)
{
    exp_set_phase(EXP_PHASE_MEASURE);
    printf("\r\n========================================\r\n");
    printf("MEASUREMENT PHASE STARTED\r\n");
    printf("Recording data for 120-180 seconds...\r\n");
    printf("========================================\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_is_measurement_complete()
*/
uint8_t exp_is_measurement_complete(void)
{
    uint32_t elapsed = get_system_time_ms() - phase_start_time_ms;
    
    if (elapsed >= MEASUREMENT_DURATION_MS) {
        return 1;
    }
    
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_init_abba_sequence()
*/
void exp_init_abba_sequence(seq_order_t order)
{
    abba_order = order;
    sequence_iteration = 0;
    printf("ABBA sequence initialized: %s\r\n", 
           (order == SEQ_ORDER_ABBA) ? "A->B->B->A" : "B->A->A->B");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_get_next_mode()
*/
seq_mode_t exp_get_next_mode(seq_mode_t current_mode, uint8_t iteration)
{
    sequence_iteration = iteration;
    
    if (abba_order == SEQ_ORDER_ABBA) {
        /* A -> B -> B -> A */
        switch (iteration % 4) {
            case 0: return SEQ_MODE_A;
            case 1: return SEQ_MODE_B;
            case 2: return SEQ_MODE_B;
            case 3: return SEQ_MODE_A;
            default: return SEQ_MODE_A;
        }
    } else {
        /* B -> A -> A -> B */
        switch (iteration % 4) {
            case 0: return SEQ_MODE_B;
            case 1: return SEQ_MODE_A;
            case 2: return SEQ_MODE_A;
            case 3: return SEQ_MODE_B;
            default: return SEQ_MODE_B;
        }
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_get_sequence_iteration()
*/
uint8_t exp_get_sequence_iteration(void)
{
    return sequence_iteration;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_reset_sequence()
*/
void exp_reset_sequence(void)
{
    sequence_iteration = 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_calculate_power()
*/
void exp_calculate_power(power_measurement_t *measurement)
{
    if (measurement->i_avg_ma > 0 && measurement->voltage_v > 0) {
        measurement->power_mw = measurement->i_avg_ma * measurement->voltage_v;
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_calculate_energy_per_tag()
*/
float exp_calculate_energy_per_tag(float power_mw, float tag_rate_hz)
{
    if (tag_rate_hz > 0) {
        return power_mw / tag_rate_hz;  /* Energy per localization in mJ */
    }
    return 0.0f;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_calculate_energy_per_anchor()
*/
float exp_calculate_energy_per_anchor(float power_mw, float effective_tag_rate_hz)
{
    if (effective_tag_rate_hz > 0) {
        return power_mw / effective_tag_rate_hz;  /* Energy per anchor operation in mJ */
    }
    return 0.0f;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_read_voltage()
*/
float exp_read_voltage(void)
{
    /* Read voltage from DW1000 */
    uint16 temp_vbat = dwt_readtempvbat(1);  /* fastSPI = 1 */
    uint8 vbat_raw = temp_vbat & 0xFF;
    
    /* Convert to voltage: 0.0057 * reading + 2.3 */
    float voltage = 0.0057f * vbat_raw + 2.3f;
    
    return voltage;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn exp_read_current_ma()
*
* @brief Placeholder for current measurement
*        In real system, this would interface with external current measurement hardware
*        For now, returns 0 to indicate measurement not available
*        User should manually input current values or interface with measurement hardware
*/
float exp_read_current_ma(void)
{
    /* TODO: Interface with external current measurement hardware */
    /* For now, return 0 to indicate measurement not available */
    /* In real system, this would read from ADC or external current meter */
    return 0.0f;  /* Placeholder - actual value from external measurement */
}

