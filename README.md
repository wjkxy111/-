# MSPM0G3507 双轮小车工程说明

本文档是本工程提供给开发者和 GPT/Codex 的首要上下文。修改代码前，先阅读“当前有效架构”“方向与左右约定”“模块状态”和“已标定参数”。

## 1. 工程概况

本工程基于：

- MCU：TI MSPM0G3507，LQFP-64
- SDK：MSPM0 SDK 2.08.00.03
- 配置：TI SysConfig，源文件为 `empty.syscfg`
- 编译器：TI Arm Clang
- 运行模式：NoRTOS，主循环 + 中断
- 应用入口：`empty.c`
- 中断入口：`interrupt.c`
- 实车参数：`Driver/vehicle_tuning.c`
- CCS 调试配置：`targetConfigs/MSPM0G3507.ccxml`，当前为 XDS110

`empty.syscfg` 是引脚、外设、时钟和中断配置的唯一来源。不要手动修改 `Debug/ti_msp_dl_config.c`、`Debug/ti_msp_dl_config.h`、生成的 makefile、`.o`、`.map` 或 `.out`。

## 2. 当前有效架构

当前固件是四按键 OLED 实车标定与 PID 调试菜单，不是旧版 `peripheral_test.c` 综合菜单。

启动流程：

1. `SYSCFG_DL_init()` 初始化 SysConfig 外设。
2. 初始化 LED、蜂鸣器、继电器、光电传感器、OLED、UART2、UART3、电机和编码器。
3. 电机立即停止，配置 1 ms SysTick。
4. `TaskManager_Init()` 显示任务主页。
5. 主循环持续运行任务管理器、OLED、蜂鸣器和串口非阻塞服务。
6. 编码器 GPIO ISR 负责计数；TIMA1 ISR 每 2 ms 进入编码器定时处理，并每 10 ms更新轮速和电机控制器。

主页按键：

| 按键 | 功能 |
| --- | --- |
| K1 | 下一个任务 |
| K2 | 上一个任务 |
| K3 | 进入/执行当前任务 |
| K4 | 停止任务并返回主页；电机任务中视为急停 |

当前任务：

| 任务 | 用途 |
| --- | --- |
| `MOTOR MAP / DIR` | 识别逻辑左右电机、编码器通道和计数方向 |
| `ENCODER LIVE VIEW` | 实时查看 E1/E2 计数与 CPS |
| `LEFT CPR 10 TURNS` | 左轮手动旋转 10 圈测 CPR |
| `RIGHT CPR 10 TURNS` | 右轮手动旋转 10 圈测 CPR |
| `SPEED PID VERIFY` | 用正式速度 PID 验证 15%、25%、35% 目标；K1/K2/K3 选档 |
| `SPEED PID TUNE` | UART3 在线速度 PID 调参，使用临时浮点控制器 |
| `YAW PID TUNE` | UART3 在线 BMI088 yaw 角度 PID 调参 |

## 3. 左右、正负与编码器约定

所有上层代码必须遵守以下物理语义：

- `MOTOR_LEFT`：物理左轮。
- `MOTOR_RIGHT`：物理右轮。
- 正速度：小车前进方向。
- 负速度：小车后退方向。
- 正 yaw 修正：左轮目标增加、右轮目标减小。
- 编码器反馈符号必须与对应轮目标符号一致。

当前实车确认值：

```c
MOTOR_LOGICAL_SIDE_SWAP = 0;
MOTOR_OUTPUT_SIGN = 1;

Encoder_SetReversed(ENCODER_1, false);
Encoder_SetReversed(ENCODER_2, true);
```

当前对应关系：

| 逻辑对象 | 编码器 | 软件方向修正 |
| --- | --- | --- |
| 左轮 | ENCODER_1 | 不反转 |
| 右轮 | ENCODER_2 | 反转 |

原地 yaw 短测已验证：目标 `左 +3 / 右 -3` 时，反馈也为 `左正 / 右负`。不要仅根据历史接线注释再次翻转方向；应以 UART/OLED 的“目标与反馈同号”和实车前进方向为准。

## 4. 三环控制链

正式控制实现位于 `Driver/Hardware/motor.c`。工程所称“三环”实际结构为两个外环并行生成左右轮目标，再进入左右速度内环：

```text
位置 PID ──> 基础目标速度 ─┐
                           ├─> 左右轮目标速度 ─> 左/右速度 PID ─> PWM
yaw PID ──> 左右差速修正 ─┘
```

主要接口：

```c
Motor_SetSpeeds(left, right);                    // 仅左右速度闭环
Motor_DriveHeading(speed, target_yaw_x10);       // 速度 + yaw 航向保持
Motor_DriveDistanceHeading(speed, counts, yaw);  // 位置 + yaw + 速度
Motor_MovePositionYaw(counts, yaw);              // 默认速度的兼容封装
Motor_Off();                                     // 停止并关闭 TB6612
```

控制周期和保护：

- 编码器与速度 PID：10 ms。
- yaw 数据超过 `MOTOR_IMU_TIMEOUT_MS` 未更新时，航向/位置控制停车。
- 速度目标跨过零点换向时，清除对应速度 PID 历史状态，防止旧方向积分导致饱和。
- 正目标的速度 PID 不允许输出负 PWM，负目标不允许输出正 PWM；超速时撤掉驱动而不是反向制动。
- yaw 最大差速当前限制为 10。

不要在新代码中调用旧 `MotionControl_*`。`Driver/Control/motion_control.c` 和旧 `balance_control` 路径已从构建中排除，不使用本次实车标定结果。

## 5. 已标定参数

所有正式参数集中在 `Driver/vehicle_tuning.c`。

### 5.1 编码器

左右轮手动旋转 10 圈均测得 7845 个有效计数：

```c
MOTOR_ENCODER_LEFT_CPR  = 785;
MOTOR_ENCODER_RIGHT_CPR = 785;
```

`7845 / 10 = 784.5`，整数 CPR 四舍五入为 785。

`ENCODER_SPEED_100_PERCENT_CPS = 4000` 目前仍是速度百分比换算基准，不是最终满速实测值；若完成满速 CPS 测试，应更新此参数并重新验证速度环。

### 5.2 正式速度 PID

架空 15%、25%、35% 测试得到临时控制器参数 `Kp=2.8, Ki=1.0, Kd=0`。正式定点 PID 每 10 ms 直接累加误差，积分标度相差 100 倍，因此正式参数为：

```c
MOTOR_SPEED_PID_KP_X1000 = 2800;
MOTOR_SPEED_PID_KI_X1000 = 10;
MOTOR_SPEED_PID_KD_X1000 = 0;
```

不要把正式 `Ki` 改成 1000；这会使积分迅速饱和并导致车轮抽动。

### 5.3 正式 yaw PID

原地目标 `+10°`、`-10°`、`+30°` 实测后确定：

```c
MOTOR_YAW_PID_KP_X1000 = 120;
MOTOR_YAW_PID_KI_X1000 = 0;
MOTOR_YAW_PID_KD_X1000 = 300;
MOTOR_YAW_SPEED_LIMIT   = 10;
```

即 `Kp=0.12, Ki=0, Kd=0.30`。实测结果：

- +10°：约 +11.1°，超调约 1.1°，无持续振荡。
- -10°：约 -10.1°，超调约 0.2°。
- +30°：约 +28.8°，受电机死区影响残差约 1.2°。
- 不加 Ki，避免静摩擦下积分累积后突然转动。

`YAW PID TUNE` 为隔离角度外环，会临时把速度内环设为 `Kp=2.8, Ki=0, Kd=0`；退出任务时恢复正式速度 PID。

### 5.4 位置 PID

当前位置参数仍是初始值，尚未完成实车距离调试：

```c
MOTOR_POSITION_PID_KP_X1000 = 200;
MOTOR_POSITION_PID_KI_X1000 = 0;
MOTOR_POSITION_PID_KD_X1000 = 120;
```

调试顺序必须是：编码器与方向 → 速度环 → yaw 环 → 最后位置环。

## 6. UART3 调参协议

UART3 用于速度和 yaw 调参：

- MCU RX：PB3，应连接 USB 串口 TX。
- MCU TX：PB2，应连接 USB 串口 RX。
- GND 必须共地。
- 波特率：9600。
- 当前电脑测试端口曾使用 COM14，端口号不是固件常量。
- UART3 驱动同时使用 RX 中断和主循环 FIFO 轮询兜底。

### 6.1 速度调参

进入 `SPEED PID TUNE` 后支持：

```text
GET
PID,2800,1000,0
TARGET,15
TARGET,0
STOP
```

返回：

```text
DATA,target,left,right,left_pwm,right_pwm
```

此任务的 Ki 标度属于临时浮点控制器，`1000` 表示 `Ki=1.0`，不能直接复制成正式速度 PID 的 Ki。

### 6.2 yaw 调参

进入 `YAW PID TUNE` 后，小车必须静止完成 BMI088 零偏校准。支持：

```text
GET
ZERO
PID,120,0,300
TARGET,100
TARGET,-100
SPEED,0
RUN
STOP
```

角度单位为 0.1°，所以 `TARGET,100` 表示 +10.0°。返回：

```text
DATA,target_yaw,yaw,yaw_rate,
     left_target,right_target,
     left_feedback,right_feedback,
     left_pwm,right_pwm
```

原地角度调试使用 `SPEED,0`。调直线航向保持时再使用正基础速度，例如 `SPEED,15`。

## 7. 目录与模块作用

```text
.
├─ empty.c                         当前应用与任务表
├─ empty.syscfg                    SysConfig 唯一配置源
├─ interrupt.c/.h                  全工程 IRQ 入口
├─ Driver/
│  ├─ vehicle_tuning.c             全车实测参数唯一集中入口
│  ├─ Hardware/                    可复用硬件驱动和正式电机控制
│  ├─ PeripheralTest/              传感器、标定和调参任务
│  └─ Control/                     通用 PID、任务管理和扩展算法
├─ targetConfigs/                  CCS 调试探针配置
├─ speed_pid_tuner.py              UART3 自动速度 PID 搜索工具
└─ Debug/                          CCS/SysConfig 生成目录，不手改
```

### 7.1 `Driver/Hardware`

| 模块 | 作用 | 当前状态 |
| --- | --- | --- |
| `motor.c/.h` | 正式左右速度、位置、yaw 控制与 TB6612 PWM/方向 | 核心，在用 |
| `encoder.c/.h` | E1/E2 正交计数、10 ms 测速、调用电机控制更新 | 核心，在用 |
| `oled.c/.h` | 128×64 OLED 帧缓冲和非阻塞分片发送 | 在用；主循环必须调用 `OLED_Service()` |
| `UART3_OPENMV/OPENMV.c/.h` | UART3 环形缓冲；当前也用于 PID 调参 | 在用 |
| `bluetooth.c/.h` | UART2 蓝牙环形缓冲 | 已构建，当前调参不使用 |
| `UART0_openmv/openmv.c/.h` | 第二路视觉串口 UART0 | 已构建，当前菜单不使用 |
| `buzzer.c/.h` | 非阻塞蜂鸣器 | 在用 |
| `led.c/.h` | 红/黄/蓝 LED | 在用 |
| `relay.c/.h` | 继电器控制 | 初始化，当前任务少量使用 |
| `photoelectric_sensor.c/.h` | 光电输入封装 | 初始化，当前任务未使用 |
| `servo.c/.h` | 双路舵机角度与脉宽换算 | 已构建，当前菜单不使用 |
| `mmc5983ma.c/.h` | 磁力计读取、标定和航向 | 已构建，当前 yaw 调参未融合 |

### 7.2 `Driver/PeripheralTest`

| 模块 | 作用 | 当前状态 |
| --- | --- | --- |
| `button.c/.h` | 四按键消抖和按下边沿 | 在用 |
| `bsp_i2c.c/.h` | BMI088 等设备的 I2C 超时访问与恢复 | 在用 |
| `gyro_bmi088.c/.h` | BMI088 初始化、零偏校准、姿态与 yaw 积分 | yaw 任务和正式航向控制使用 |
| `motor_encoder_check.c/.h` | 电机、编码器对应关系和方向检查 | 在用 |
| `speed_pid_tune.c/.h` | UART3 临时速度 PID 调参任务 | 在用 |
| `yaw_pid_tune.c/.h` | UART3 BMI088 yaw PID 调参任务 | 在用 |
| `motor_test.c/.h` | 基础电机动作封装 | 已构建，当前任务表不直接使用 |
| `line_sensor.c/.h` | 八路数字循迹输入 | 已构建，当前任务表不使用 |
| `peripheral_test.c/.h` | 旧综合测试菜单 | 从构建排除，不要与当前入口混用 |

### 7.3 `Driver/Control`

| 模块 | 作用 | 当前状态 |
| --- | --- | --- |
| `pid.c/.h` | 通用定点 PID | 核心，在用 |
| `task_manager.c/.h` | OLED 四按键任务状态机 | 核心，在用 |
| `bluetooth_hmi.c/.h` | 参数滑条/波形协议 | 已构建，当前任务未初始化该协议 |
| `line_follow.c/.h` | 数字循迹状态机 | 已构建，当前任务未调用 |
| `motion_control.c/.h` | 旧运动兼容层 | 从构建排除 |
| `k230_line_follow.c/.h` | K230 循迹适配 | 从构建排除 |
| `openmv_line_follow.c/.h` | OpenMV 循迹适配 | 从构建排除 |
| `vision_line_follow.c/.h` | 通用视觉循迹 | 从构建排除 |
| `balance_control.c` | 旧控制路径引用 | 从构建排除/当前目录不存在，不要依赖 |

根目录的 `Driver/motor.c`、`Driver/Servo.c` 和 `oled.c` 是旧兼容实现，均已从构建排除。新代码使用 `Driver/Hardware` 下的实现。

## 8. 主要外设与引脚

以下内容来自 `empty.syscfg`；修改接线时应修改 SysConfig 并重新生成。

| 功能 | 外设/引脚 | 说明 |
| --- | --- | --- |
| BMI088 | I2C0：PA10 SDA、PA11 SCL | yaw 数据源 |
| OLED | I2C1：PA16 SDA、PA15 SCL | 默认地址 0x3C |
| 电机 PWM | TIMG8：PA26、PB22 | PWM period 3200 |
| 电机方向/STBY | PB23、PB21、PA22、PB19、PB20 | TB6612 |
| 编码器 1 | PA24 A、PB24 B | A 相双边沿中断 |
| 编码器 2 | PA13 A、PA14 B | A 相双边沿中断 |
| UART3 调参 | PB3 RX、PB2 TX | 9600 baud |
| UART2 蓝牙 | PB18 RX、PB17 TX | 115200 baud |
| UART0 视觉 | PB1 RX、PB0 TX | 9600 baud |
| 按键 K1~K4 | PA12、PB15、PB14、PB13 | 输入上拉 |
| LED | PA7、PB4、PB5 | 红、黄、蓝 |
| 蜂鸣器 | PB27 | 有效电平由调参变量定义 |
| 舵机 1/2 | TIMA0：PA8、PA9 | 当前 100 Hz |
| 八路循迹 | PA25、PB25、PB26、PA27~PA31 | LINE_1~LINE_8 |
| 控制定时器 | TIMA1，2 ms | 编码器内部每 10 ms 更新控制 |

## 9. 中断与主循环边界

`interrupt.c` 中的入口：

- `SysTick_Handler()`：只增加 1 ms 时间基准 `g_ms`。
- `GROUP1_IRQHandler()`：处理两路编码器 A 相边沿。
- `TIMER_0_INST_IRQHandler()`：编码器测速和正式电机控制更新。
- `UART_Bluetooth_INST_IRQHandler()`：UART2 环形缓冲搬运。
- `UART_OPENMV_INST_IRQHandler()`：UART3 环形缓冲搬运。

ISR 中不要刷新 OLED、阻塞等待 I2C、解析文本协议或打印大量数据。OLED、BMI088 服务、串口协议和任务状态机放在主循环。

## 10. 构建与验证

在 CCS Theia 中：

1. 保存 `empty.syscfg` 和源文件。
2. 执行 **Project > Clean**。
3. 执行 **Build Project**。
4. 确认编译日志包含新增 `.c` 文件。
5. 烧录后执行 System Reset，再运行。

静态检查：

```powershell
python C:\Users\34542\.codex\skills\mspm0-ccs\scripts\check_syscfg.py .
```

如果修改源文件后仍运行旧逻辑，先检查 `.cproject` source exclusions，再 Clean/Build。不要通过手改 `Debug` 目录的 makefile 加入源文件。

## 11. 待完成实车标定

以下项目仍未最终确认：

- `ENCODER_SPEED_100_PERCENT_CPS` 的左右轮满速实测值。
- 正式速度 PI 在落地负载下的 15%、25%、35% 响应。
- `Motor_DriveHeading(15, 0)` 的直线偏差和 yaw 修正效果。
- 位置 PID 和 0.5 m / 1 m 距离误差。
- 轮径、轮距及编码器计数到实际距离的换算。
- BMI088 长距离 yaw 漂移；纯陀螺仪没有绝对航向参考。
- 磁力计安装方向、硬铁/软铁标定和 yaw 融合。
- 循迹、视觉、舵机端点及其他比赛任务参数。

## 12. 给 GPT/Codex 的修改规则

1. 先读本 README、`empty.c`、`Driver/vehicle_tuning.c` 和目标模块头文件。
2. 引脚、外设、时钟和 IRQ 以 `empty.syscfg` 为准。
3. 正式运动控制只修改/调用 `Driver/Hardware/motor.c/.h`，不要恢复旧 `motion_control` 路径。
4. 左右和正负方向以第 3 节的当前实车结果为准。
5. 正式速度 PID 与临时速度调参器的 Ki 标度不同，禁止直接复制 1000。
6. 调参结果只有写入 `Driver/vehicle_tuning.c` 才会掉电保存。
7. 保留 K4 急停；任何自动电机测试都要限时、限速，并先说明架空或地面条件。
8. 不修改 SysConfig 生成文件和 Debug 构建产物。
9. 新增源文件后要求用户 Clean/Build，并确认该文件未被 `.cproject` 排除。
10. 未经实车验证的参数必须标为“待验证”，不要描述成最终事实。
