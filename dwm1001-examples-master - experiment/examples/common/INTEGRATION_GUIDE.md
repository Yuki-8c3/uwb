# 实验框架集成指南

## 概述

所有实验需要遵循统一的测试流程：
1. **基线校准**：系统空闲（不发不收）各 60 s，量测标签/锚点"静态电流"
2. 切换到对应实验配置，等待 30 s 稳态后开始计；采 120–180 s
3. 记录：`I_avg(mA), V, r_tag_hz, rxpacc, loss(%), uplink bytes/s`
4. 计算：`P = I_avg × V`，`E_tag = P_tag / r_tag`
5. **顺序随机化**：用 ABBA 交叉顺序，减少环境漂移影响

## 集成步骤

### 1. 包含通用框架

在 `main.c` 中添加：
```c
#include "exp_common.h"
```

### 2. 初始化实验框架

在 `main()` 函数中，初始化DW1000后：
```c
/* Initialize experiment framework */
exp_common_init();

/* Start baseline calibration */
exp_start_baseline_calibration();
```

### 3. 在任务循环中检查阶段

在 `ss_init_run()` 或 `ss_resp_run()` 中：
```c
/* Check experiment phase */
exp_phase_t phase = exp_get_current_phase();

if (phase == EXP_PHASE_BASELINE) {
    /* Baseline: no TX/RX, just idle */
    if (exp_is_baseline_complete()) {
        exp_start_steady_state_wait();
    }
    return 1;  /* Skip TX/RX operations */
}

if (phase == EXP_PHASE_STEADY) {
    /* Steady-state wait: normal operation but not recording */
    if (exp_is_steady_state_complete()) {
        exp_start_measurement_phase();
        /* Switch to first mode in ABBA sequence */
        seq_mode_t first_mode = exp_get_next_mode(SEQ_MODE_A, 0);
        /* Apply mode configuration */
    }
    /* Continue normal operation but don't record statistics yet */
}

if (phase == EXP_PHASE_MEASURE) {
    /* Normal measurement phase */
    /* Check if measurement complete, then switch to next mode in ABBA sequence */
}
```

### 4. 实现ABBA顺序

在模式切换逻辑中：
```c
/* Initialize ABBA sequence (randomize order) */
seq_order_t order = (rand() % 2) ? SEQ_ORDER_ABBA : SEQ_ORDER_BAAB;
exp_init_abba_sequence(order);

/* Get next mode based on iteration */
uint8_t iteration = exp_get_sequence_iteration();
seq_mode_t next_mode = exp_get_next_mode(current_mode, iteration);
```

### 5. 记录功率指标

在统计记录中：
```c
power_measurement_t measurement = {0};
measurement.i_avg_ma = exp_read_current_ma();  /* From external measurement */
measurement.voltage_v = exp_read_voltage();    /* From DW1000 */
measurement.measurement_time_ms = elapsed_time;

exp_calculate_power(&measurement);

/* Calculate energy per operation */
float tag_rate_hz = get_current_tag_rate();  /* Get from your experiment */
measurement.energy_per_op = exp_calculate_energy_per_tag(
    measurement.power_mw, tag_rate_hz);
```

### 6. 输出记录格式

按照要求输出：
```
I_avg(mA), V, r_tag_hz, rxpacc, loss(%), uplink bytes/s
```

## 注意事项

1. **电流测量**：`exp_read_current_ma()` 目前返回0，需要：
   - 接口外部电流表硬件，或
   - 手动输入测量值，或
   - 使用nRF52的SAADC进行电流测量（如果硬件支持）

2. **电压测量**：已实现通过DW1000读取电压

3. **ABBA顺序**：每次实验开始时随机选择ABBA或BAAB顺序

4. **基线校准**：在基线阶段，确保不进行任何TX/RX操作

5. **稳态等待**：在稳态阶段，正常操作但不记录统计数据

## 示例

参考 `examples/exp1_phy_power/main.c` 的更新版本，查看完整集成示例。


