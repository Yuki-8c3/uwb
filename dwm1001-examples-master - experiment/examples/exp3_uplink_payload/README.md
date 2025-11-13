# 实验3：上报内容大小（回传侧功耗/带宽）

## 实验目的
改**锚点→服务器回传负载**，看锚点侧功耗与带宽。

## 实验配置

### A组（仅时间戳）
- **Payload**: TS only（时间戳）
- **Payload大小**: 8 bytes
- **特点**: 更省电、带宽更低

### B组（时间戳+CIR摘要）
- **Payload**: TS + CIR_s（时间戳 + CIR摘要）
- **Payload大小**: 8 + 96 = 104 bytes
- **CIR_s大小**: 96 bytes
- **特点**: 功耗略高但更易做定位质量诊断

## 实验参数

- **每个模式运行时间**: 150秒
- **模式切换延迟**: 10秒
- **记录间隔**: 1秒
- **基线PHY配置**: 6.8Mbps / PRF 64MHz / PL=64 / SFD=standard(8)

## 记录指标

每个测量周期记录以下数据：
- **Uplink Count**: 上行传输计数
- **Uplink Bytes**: 总上行字节数
- **Uplink Bytes/sec**: 上行带宽（字节/秒）
- **Uplink Bandwidth**: 上行带宽（kbps）
- **Avg Uplink Bytes per TX**: 每次传输的平均字节数
- **RSSI**: 接收信号强度指示器
- **RXPACC**: RX前导累积计数器
- **SNR**: 信噪比

## 预期结果

- **A组**: 上行更省电、带宽更低（8 bytes/packet）
- **B组**: 功耗略高但更易做定位质量诊断（104 bytes/packet，包含CIR摘要）

## 文件结构

```
exp3_uplink_payload/
├── main.c              # 主程序，负责初始化和模式切换监控
├── ss_resp_main.c      # SS TWR responder逻辑和数据记录
├── ss_resp_main.h      # 头文件定义
├── UART/               # UART驱动
│   ├── UART.c
│   └── UART.h
└── config/             # SDK配置文件
    └── sdk_config.h
```

## 使用方法

1. 编译项目（使用Keil或SES）
2. 烧录到DWM1001设备（作为锚点/responder）
3. 通过UART观察输出数据
4. 实验将自动在Uplink模式A和B之间切换
5. 每个模式运行150秒后自动切换到下一个模式

## 输出格式

实验会输出以下信息：
- 模式切换通知和payload大小信息
- 每10次成功传输的详细信息（包括uplink payload大小）
- 每秒的统计摘要（包括带宽信息）
- 模式切换时的完整统计报告（包括带宽对比）

## 注意事项

- 此实验针对**锚点（responder）**，需要配合initiator使用
- 实验开始前确保系统已稳定
- 模式切换期间会有10秒延迟
- 所有统计数据会在模式切换时打印
- **关键指标**：uplink_bytes_per_sec用于评估带宽差异
- 在实际系统中，uplink传输会通过WiFi/LoRa/其他接口，本实验仅模拟数据大小


