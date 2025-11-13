# 测试流程检查清单

## 当前代码状态

### ✅ 已实现的功能

1. **模式切换逻辑**
   - ✅ 所有实验都实现了两个模式的自动切换
   - ✅ 每个模式运行150秒（在120-180秒范围内）
   - ✅ 模式切换延迟10秒

2. **数据记录**
   - ✅ RSSI, RXPACC, SNR
   - ✅ TX/RX计数
   - ✅ 丢包计数和丢包率
   - ✅ 上行字节数（实验3）
   - ✅ 距离测量
   - ✅ 统计报告输出

3. **通用框架（新增）**
   - ✅ `examples/common/exp_common.h` 和 `.c` 提供：
     - 基线校准阶段管理
     - 稳态等待阶段管理
     - ABBA顺序管理
     - 功率计算辅助函数
     - 电压读取（通过DW1000）

### ⚠️ 需要手动集成/完成的功能

1. **基线校准（60秒空闲）**
   - ⚠️ 框架已提供，但需要集成到每个实验的main.c中
   - ⚠️ 需要在基线阶段禁用TX/RX操作
   - ⚠️ 需要外部电流表测量静态电流

2. **稳态等待（30秒）**
   - ⚠️ 框架已提供，但需要集成到每个实验
   - ⚠️ 需要在稳态阶段正常操作但不记录统计

3. **电流测量 (I_avg)**
   - ⚠️ `exp_read_current_ma()` 目前是占位符，返回0
   - ⚠️ 需要：
     - 接口外部电流表硬件，或
     - 手动输入测量值，或
     - 使用nRF52 SAADC进行测量（如果硬件支持）

4. **功率计算**
   - ✅ 框架提供了 `exp_calculate_power()` 函数
   - ⚠️ 需要在统计输出中调用并显示结果
   - ⚠️ 需要计算 `E_tag = P_tag / r_tag`

5. **ABBA顺序**
   - ✅ 框架提供了ABBA顺序管理
   - ⚠️ 需要集成到每个实验的模式切换逻辑中
   - ⚠️ 需要在实验开始时随机选择ABBA或BAAB

6. **记录格式**
   - ⚠️ 需要按照要求输出：`I_avg(mA), V, r_tag_hz, rxpacc, loss(%), uplink bytes/s`
   - ⚠️ 当前输出格式需要调整以匹配要求

## 集成步骤

### 对于每个实验（exp1-exp5）：

1. **在main.c中添加**：
```c
#include "exp_common.h"

/* 在main()函数中，初始化DW1000后 */
exp_common_init();
exp_start_baseline_calibration();

/* 初始化ABBA顺序（随机选择） */
seq_order_t order = (rand() % 2) ? SEQ_ORDER_ABBA : SEQ_ORDER_BAAB;
exp_init_abba_sequence(order);
```

2. **在ss_init_run()或ss_resp_run()中添加阶段检查**：
```c
exp_phase_t phase = exp_get_current_phase();

if (phase == EXP_PHASE_BASELINE) {
    if (exp_is_baseline_complete()) {
        exp_start_steady_state_wait();
    }
    return 1;  /* 跳过TX/RX操作 */
}

if (phase == EXP_PHASE_STEADY) {
    if (exp_is_steady_state_complete()) {
        exp_start_measurement_phase();
        /* 切换到ABBA序列的第一个模式 */
    }
    /* 正常操作但不记录统计 */
}

if (phase == EXP_PHASE_MEASURE) {
    /* 正常测量阶段 */
}
```

3. **在模式切换中使用ABBA顺序**：
```c
uint8_t iteration = exp_get_sequence_iteration();
seq_mode_t next_mode = exp_get_next_mode(current_mode, iteration);
exp_get_sequence_iteration()++;  /* 递增迭代计数 */
```

4. **在统计输出中添加功率信息**：
```c
power_measurement_t pwr = {0};
pwr.i_avg_ma = exp_read_current_ma();  /* 需要外部测量 */
pwr.voltage_v = exp_read_voltage();
exp_calculate_power(&pwr);

printf("I_avg: %.3f mA, V: %.3f V, P: %.3f mW\r\n", 
       pwr.i_avg_ma, pwr.voltage_v, pwr.power_mw);
```

## 当前缺失的关键功能

1. **电流测量接口**：需要硬件支持或手动输入
2. **基线阶段的TX/RX禁用**：需要在每个实验的run函数中实现
3. **ABBA顺序集成**：需要更新模式切换逻辑
4. **标准输出格式**：需要调整printf格式以匹配要求

## 建议

1. **电流测量**：由于需要外部硬件，建议：
   - 在代码中提供占位符，用户手动输入测量值
   - 或提供接口函数，用户可以连接自己的电流测量硬件

2. **分阶段实现**：
   - 第一阶段：集成基线校准和稳态等待
   - 第二阶段：集成ABBA顺序
   - 第三阶段：完善功率计算和输出格式

3. **测试验证**：
   - 先在一个实验（如exp1）中完整实现
   - 验证无误后，复制到其他实验

## 参考文档

- `examples/common/INTEGRATION_GUIDE.md`: 详细集成指南
- `examples/common/exp_common.h`: 框架API文档


