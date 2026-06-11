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
Lift motion and protection parameters
```

当前参数：

```c
#define LIFT_UP_TIME_MS                     2000U
#define LIFT_DOWN_TIME_MS                   5000U
#define LIFT_SEND_PERIOD_MS                   10U

#define LIFT_ID1_UP_CURRENT                -8100
#define LIFT_ID1_DOWN_CURRENT              -2000
#define LIFT_ID3_UP_CURRENT                 8100
#define LIFT_ID3_DOWN_CURRENT               2000

#define LIFT_RAMP_TIME_MS                    400U
#define LIFT_SYNC_ENABLE                       1U
#define LIFT_SYNC_KP_NUM                       1
#define LIFT_SYNC_KP_DEN                       8
#define LIFT_SYNC_MAX_CORRECTION            1800
#define LIFT_LEVEL_PROTECT_ERROR_ECD        2200
#define LIFT_FEEDBACK_TIMEOUT_MS             100U
#define LIFT_MOTOR_CURRENT_LIMIT           12000
```

注意：下降电流方向与上升同向，是为了在重载下降时提供制动，不是主动向下拉。

## 偏载同步保护

当前程序已加入两侧 M3508 编码器同步补偿：

- ID1 反馈：`0x201`
- ID3 反馈：`0x203`
- 将 ID1/ID3 编码器方向统一成“平台上升为正”
- 根据两侧位置差 `g_lift_level_error` 自动修正两边电流
- 位置差超过 `LIFT_LEVEL_PROTECT_ERROR_ECD` 会停止输出
- 反馈超时超过 `LIFT_FEEDBACK_TIMEOUT_MS` 会停止输出

同步补偿调参建议：

- 偏载时仍明显倾斜：适当增大 `LIFT_SYNC_KP_NUM` 或减小 `LIFT_SYNC_KP_DEN`
- 抖动或两边来回抢：适当减小 `LIFT_SYNC_KP_NUM` 或增大 `LIFT_SYNC_KP_DEN`
- 补偿太弱：增大 `LIFT_SYNC_MAX_CORRECTION`
- 保护太容易触发：适当增大 `LIFT_LEVEL_PROTECT_ERROR_ECD`
- 保护触发太晚：减小 `LIFT_LEVEL_PROTECT_ERROR_ECD`

`LIFT_LEVEL_PROTECT_ERROR_ECD` 单位是 M3508 编码器累计计数，不是毫米。第一次带重物测试时建议在 Keil Watch 里观察 `g_lift_level_error`，根据实际最大偏差再调保护阈值。

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
g_lift_feedback_ready
g_lift_protect_active
g_lift_protect_reason
g_lift_motion_tick_ms
g_lift_id1_lift_pos
g_lift_id3_lift_pos
g_lift_level_error
g_lift_sync_correction
g_lift_id1_speed_rpm
g_lift_id3_speed_rpm
g_lift_id1_base_current
g_lift_id3_base_current
g_lift_id1_cmd_current
g_lift_id3_cmd_current
g_lift_id1_rx_count
g_lift_id3_rx_count
```

判断：

- `g_raw_can_started = 1`：FDCAN 启动成功
- `g_raw_can_tx_ok_count` 增长：控制帧正在发送
- `g_raw_can_last_rx_id = 0x201/0x203`：收到 C620 反馈
- `g_raw_can_tx_error_count` 持续增长：优先检查 CANH/CANL、电调供电、终端电阻、CAN 口接线
- `g_lift_feedback_ready = 1`：两边电机反馈都已收到
- `g_lift_protect_active = 1`：保护已触发，电机输出为 0
- `g_lift_protect_reason = 1`：两边高度差过大
- `g_lift_protect_reason = 2`：电机反馈超时
- `g_lift_level_error > 0`：ID1 侧相对更高
- `g_lift_level_error < 0`：ID3 侧相对更高

## 安全提示

本工程当前是死代码循环测试：上升一段时间后下降一段时间。带重物测试时先确保平台周围无人、机械限位可靠，并准备断电。若下降过快，优先增大下降同向制动电流绝对值，或延长 `LIFT_DOWN_TIME_MS`。
