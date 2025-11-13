# 通用实验框架

## 功能

提供所有实验通用的功能：
- 基线校准（60秒空闲）
- 稳态等待（30秒）
- ABBA顺序管理
- 功率计算辅助函数
- 电压读取

## 文件

- `exp_common.h`: 头文件定义
- `exp_common.c`: 实现文件
- `INTEGRATION_GUIDE.md`: 集成指南

## 使用方法

1. 在实验的 `main.c` 中包含 `exp_common.h`
2. 在初始化时调用 `exp_common_init()`
3. 按照 `INTEGRATION_GUIDE.md` 的步骤集成到实验代码中

## 注意事项

- 电流测量需要外部硬件支持
- 电压可以通过DW1000读取
- ABBA顺序在每次实验开始时随机化


