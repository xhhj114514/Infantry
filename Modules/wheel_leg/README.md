# Wheel-leg chassis port

本模块保留从 `D:\TM32\esp32-wheeleg` 移植的五连杆运动学、LQR 增益调度、VMC、
串级 PID、离地检测和姿态保护算法。当前 `application/chassis/chassis.c` 用于
`WheelLegPosition()` 正运动学验证：读取电机反馈，解算左右腿长度与角度，六个电机始终保持停止。

## 电机参数和硬件映射

电气参数、PID 参数、CAN 接口、电机 ID、方向和机械 offset 集中在
`Modules/motor/motor_def.h`。六个电机均注册在 CAN2，反馈帧为 `0x100 + motor_id`：

| 逻辑电机 | 数组索引 / 在线位 | ID | offset rad | 方向 | 最大电压 | 扭矩/电压 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 左关节 0 | 0 | 1 | 0 | +1 | 9.69 V | 0.0333 Nm/V |
| 左关节 1 | 1 | 2 | 0 | +1 | 9.69 V | 0.0333 Nm/V |
| 左轮 | 2 | 3 | 0 | -1 | 4.5 V | 0.0100 Nm/V |
| 右关节 0 | 3 | 5 | 0 | -1 | 9.69 V | 0.0333 Nm/V |
| 右关节 1 | 4 | 6 | 0 | -1 | 9.69 V | 0.0333 Nm/V |
| 右轮 | 5 | 7 | 0 | -1 | 4.5 V | 0.0100 Nm/V |

`WHEEL_LEG_MOTOR_OUTPUT_RATIO` 为 `0.60`。关节上限由
`SMOTOR_MAX_VOLTAGE_MV = 9690.0f` 换算为 V，轮电机上限由
`WHEEL_LEG_WHEEL_MAX_VOLTAGE_V` 指定。当前验证任务不施加这些电压或扭矩输出。

`WHEEL_LEG_MOTOR_OFFSETS_RAD` 全部为 `0.0f` 占位值，后续按实际机械位置手动对齐。
索引顺序与上表一致，运动学角度使用以下换算：

```c
motor_angle_rad = (measure.total_angle_rad - offset_angle_rad) * direction;
```

这里使用原始累计编码器角度 `total_angle_rad`；Smotor 上电自动校零得到的
`position_rad` 不作为连杆机械零位。给定目标机械角度 `phi`，对应 offset 为
`total_angle_rad - phi / direction`。

## 当前任务行为

- `ChassisTask()` 随机器人核心任务以 4 ms（250 Hz）更新左右腿位置，Smotor 报文仍以 1 kHz 发送。
- 每个任务周期读取六电机在线状态与角度，并保持六电机停止；发布的 `chassis_feed.power_flag` 为 `0`。
- 左右腿分别只依赖本腿两个关节在线。轮电机离线不影响腿位置有效性，任务不依赖 IMU 就绪状态。
- 调用顺序为 `WheelLegPosition(JOINT_1, JOINT_0, position)`：关节 1 对应 `phi1`，关节 0 对应 `phi4`。
- 当前任务不调用 `WheelLegControlStep()`，因此不运行 LQR、VMC、速度解算或姿态保护控制流程；这些算法仍保留在模块中。
- `ChassisSetCommand(speed_mps, yaw_speed_rad_s, leg_length_m)` 和 `ChassisEnable(enabled)` 保留兼容接口，参数单位仍为 m/s、rad/s、m。它们可更新保留控制器的命令或使能状态，当前验证任务始终保持六电机停止。

## 调试观察与手动对齐

在调试器中观察 `chassis.c` 的静态变量 `leg_position`，或调用
`ChassisGetLegPosition()` 取得 `const ChassisLegPosition_s *`：

| 字段 | 含义 |
| --- | --- |
| `motor_angle_rad[6]` | 按上表索引排列的 offset 和方向修正后的电机角度，单位 rad；离线电机为 `NAN`。 |
| `motor_online_mask` | bit 0 至 bit 5 分别对应上表六个电机，位为 1 表示该电机在线；全部在线为 `0x3F`。 |
| `left_leg.length_m` / `right_leg.length_m` | 五连杆腿长，单位 m；以两个固定基座的中点为原点计算。 |
| `left_leg.angle_rad` / `right_leg.angle_rad` | 腿向量相对连杆局部 `+X` 轴的角度，单位 rad；局部 `+Y` 方向为 `π/2`。 |
| `left_leg.valid` / `right_leg.valid` | 本腿两个关节在线、解算结果有限且腿长大于 0 时为 1，否则为 0。无效腿的长度与角度为 `NAN`。 |

在线位只表示电机反馈在线，不保证角度或运动学结果有效；解算状态应以每腿的
`valid` 为准。纯位置验证不套用平衡控制器的目标腿长范围。

手动移动关节时，先核对 `motor_angle_rad` 的变化方向，再修改
`motor_def.h` 中对应的 offset，对照实测腿长和腿角观察左右腿结果。

## 构建和算法测试

主工程构建：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel
```

底盘位置验证集成测试编译真实 `chassis.c` 和 `leg_position.c`，以主机 stub 隔离硬件，
覆盖关节顺序、方向、独立几何参考、反馈离线/恢复和持续零输出：

```powershell
New-Item -ItemType Directory -Force build\codex-verify | Out-Null
gcc -std=c11 -Wall -Wextra -Werror -Itests\chassis_stubs `
  -Iapplication\chassis -IModules\motor -IModules\motor\Smotor -IModules\wheel_leg `
  tests\chassis_leg_position_test.c application\chassis\chassis.c `
  Modules\wheel_leg\leg_position.c -lm -o build\codex-verify\chassis_leg_position_test.exe
& .\build\codex-verify\chassis_leg_position_test.exe
```

再次编译时追加 `-include tests/chassis_stubs/motor_offsets_override.h` 可使用非零 offset
测试夹具，单独验证手动标定值的换算；该头文件只用于主机测试，不参与固件构建。

现有 `tests/wheel_leg_control_test.c` 验证保留的轮腿控制器状态机，覆盖未就绪、首次对齐、
输出和姿态保护等算法行为；它不代表当前底盘验证任务会驱动电机。

```powershell
New-Item -ItemType Directory -Force build\codex-verify | Out-Null
gcc -std=c11 -O2 -IModules\wheel_leg -IModules `
  Modules\wheel_leg\wheel_leg_control.c `
  Modules\wheel_leg\leg_position.c Modules\wheel_leg\leg_speed.c `
  Modules\wheel_leg\leg_vmc.c Modules\wheel_leg\lqr_gain.c `
  tests\wheel_leg_control_test.c -lm -o build\codex-verify\wheel_leg_control_test.exe
& .\build\codex-verify\wheel_leg_control_test.exe
```
