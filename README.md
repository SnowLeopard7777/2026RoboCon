# 新R1对R2抬升【28.4kg版】

本分支是新 R1 对 R2 平台抬升测试工程，目标板为 STM32H743VIT6，IDE 使用 Keil MDK。当前程序为两个 DJI M3508 + C620 的底层 CAN 直发控制版本，用于 28.4 kg 左右负载的平台升降测试。

## 打开方式

使用 Keil 打开：

```text
MDK-ARM/VIT6.uvprojx
```

工程已按 Keil 编译通过的版本提交，生成文件目录 `MDK-ARM/VIT6/` 不需要提交，重新 Build 会自动生成。

## 电机与 CAN

- CAN 通道：CAN1 / FDCAN1
- 控制帧 ID：`0x200`
- 电机反馈 ID：`0x201`、`0x203`
- 电机 ID：
  - ID1：CAN1 ID 1
  - ID3：CAN1 ID 3

从电机轴端往尾端看：

- ID3 顺时针为抬升
- ID1 逆时针为抬升

因此当前控制方向为：

- ID1 上升：负电流
- ID3 上升：正电流
- 下降保护：下降阶段仍给上升方向的小电流，用于带负载制动，避免重物快速下落

## 当前 28.4 kg 版本参数

参数位置：

```text
Core/Src/main.c
```

搜索：

```c
RAW_CAN_TEST
```

当前参数：

```c
#define RAW_CAN_TEST_UP_TIME_MS             2000U
#define RAW_CAN_TEST_DOWN_TIME_MS           5000U
#define RAW_CAN_TEST_ID1_UP_CURRENT        -8100
#define RAW_CAN_TEST_ID1_DOWN_CURRENT      -2000
#define RAW_CAN_TEST_ID3_UP_CURRENT         8100
#define RAW_CAN_TEST_ID3_DOWN_CURRENT       2000
#define RAW_CAN_TEST_SEND_PERIOD_MS          10U
```

注意：下降电流方向与上升同向，是为了在重载下降时提供制动，不是主动向下拉。

## 调参记录

空载：

```text
上下 1 s
上电流 2140
下电流 -800
```

载重 17.5 kg：

```text
仅上无下
上电流 5000
下电流 -5000
```

载重 28.5 kg：

```text
R2 车身 + KFS * 2 约为 28.4 kg
上电流 8100
下电流 2000
注意：下电流与上电流同向，用于制动下降
```

## Watch 建议

Keil Debug 时可观察：

```c
g_main_loop_tick
g_main_hal_tick
g_raw_can_started
g_raw_can_phase
g_raw_can_motor1_current
g_raw_can_motor3_current
g_raw_can_tx_ok_count
g_raw_can_tx_fail_count
g_raw_can_last_hal_status
g_raw_can_last_rx_id
g_raw_can_rx_unmatched_count
g_raw_can_tx_free_level
g_raw_can_tx_error_count
g_raw_can_rx_error_count
```

判断：

- `g_raw_can_started = 1`：FDCAN 启动成功
- `g_raw_can_tx_ok_count` 增长：控制帧正在发送
- `g_raw_can_last_rx_id = 0x201/0x203`：收到 C620 反馈
- `g_raw_can_tx_error_count` 持续增长：优先检查 CANH/CANL、电调供电、终端电阻、CAN 口接线

## 安全提示

本工程当前是死代码循环测试：上升一段时间后下降一段时间。带重物测试时先确保平台周围无人、机械限位可靠，并准备断电。若下降过快，优先增大下降同向制动电流绝对值，或延长 `RAW_CAN_TEST_DOWN_TIME_MS`。
