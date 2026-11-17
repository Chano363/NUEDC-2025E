# 仓库指南

## 项目结构与模块组织

本仓库包含 2025 年全国大学生电子设计竞赛 E 题（自瞄装置）的三个主要子系统。当前实际代码结构以 `src/car/empty/`、`src/Tripod/` 和 `src/k230/` 为准：

```
src/
├── car/
│   └── empty/             # MSPM0G3507 — 循迹小车 Keil/SysConfig 工程
│       ├── App/           #   应用层（循迹、速度控制、里程计、蓝牙命令、瞄准协同）
│       ├── Bsp/           #   板级支持（编码器、灰度、电机驱动、蓝牙、板间通信、OLED）
│       ├── Drv/           #   底层驱动（GPIO中断、I2C、PWM、定时器、UART）
│       ├── Lib/           #   通用库（PID、环形缓冲、CRC16、通信协议、协议解析器）
│       ├── empty.c        #   小车主入口
│       └── empty.syscfg   #   小车 MSPM0 SysConfig 配置
├── Tripod/                # STM32G474 — 云台瞄准模块工程
│   ├── App/               #   应用层（云台状态机、瞄准入口）
│   ├── Bsp/               #   板级支持（步进电机、云台控制、K230 UART、陀螺仪、激光、OLED、按键）
│   ├── Drv/               #   底层驱动（UART、I2C）
│   ├── Lib/               #   通用库（PID、通信协议、环形缓冲、数学工具）
│   ├── Core/              #   STM32CubeMX 生成的 main/gpio/usart/i2c/中断文件
│   ├── Drivers/           #   STM32G4 HAL/CMSIS 驱动
│   ├── MDK-ARM/           #   Keil MDK 工程与构建日志
│   ├── EWARM/             #   IAR 工程
│   └── Tripod.ioc         #   STM32CubeMX 配置
├── k230/                  # K230 视觉处理器（CanMV/MicroPython）
│   └── CV_lite混合.py     #   当前主脚本：矩形靶面检测 + 紫/蓝紫色光斑检测 + UART 发帧
└── 删除目标文件(用于打包备份).bat

docs/                      # 题目、控制方案、踩坑记录、引脚分配文档
```

小车和云台当前不是同一 MCU/同一工程模板：小车为 MSPM0G3507，云台为 STM32G474。两边各自维护 `App/`、`Bsp/`、`Drv/`、`Lib/` 分层，但并未通过同一个共享目录树自动复用源码。K230 模块运行 CanMV/MicroPython，独立烧录。

## 构建、测试与开发命令

- **小车固件（Keil MDK / MSPM0）**：打开 `src/car/empty/empty_LP_MSPM0G3507_nortos_keil.uvprojx`（若本地工程文件未入库，以 `src/car/empty/empty.c` 与 `empty.syscfg` 所在工程为准），项目 → 编译目标（F7）
- **小车 SysConfig**：在 TI SysConfig GUI 中打开 `src/car/empty/empty.syscfg`，重新生成 `ti_msp_dl_config.c/h`
- **云台固件（Keil MDK / STM32G474）**：打开 `src/Tripod/MDK-ARM/Tripod.uvprojx`，项目 → 编译目标（F7）
- **云台 CubeMX**：在 STM32CubeMX 中打开 `src/Tripod/Tripod.ioc`，重新生成 `Core/` 初始化代码
- **K230 Python**：将 `src/k230/CV_lite混合.py` 作为当前主脚本烧录/运行到 K230（CanMV 环境），无需本地构建
- **清理产物**：运行 `src/删除目标文件(用于打包备份).bat` 清除 Keil 输出文件后打包

## 编码风格与命名规范

| 语言 | 规范 |
|------|------|
| C（MSPM0/STM32） | 4 空格缩进。类型：`Type_Prefix_CamelCase`（如 `GimbalApp_StateTypeDef`）。函数：`Module_FunctionName`（如 `Tracking_Update`）。全局变量：`g_` 前缀。头文件保护：`_HEADER_H` |
| Python（K230） | 4 空格缩进。函数/变量 `snake_case`，常量 `UPPER_CASE`。标准 MicroPython |

小车 C 项目使用 TI DriverLib 和 MSPM0G3507 SDK；云台 C 项目使用 STM32G4 HAL/CMSIS。未配置自动格式化工具，保持与周围文件风格一致。

## 当前功能边界

- **小车**：以 `src/car/empty/` 为主，已具备灰度循迹、编码器测速、速度 PID、蓝牙命令、板间通信和三段式丢线搜索逻辑。
- **云台**：以 `src/Tripod/` 为主，已具备 K230 串口解析、陀螺仪 UART、步进电机角度控制、激光控制、按键/OLED 调试和状态机。当前瞄准控制重点是视觉辅助的俯仰跟踪，水平轴完整二维视觉闭环仍需继续完善。
- **K230**：当前只有 `CV_lite混合.py` 一个主脚本，混合使用 cv_lite 矩形检测和 LAB 色块检测。脚本会发送靶面矩形中心 `0xF2`，在测试/比赛模式下发送紫/蓝紫光斑坐标 `0xF1`，圆形点集发送代码目前保留但默认注释。
- **题目第二问状态**：仓库已有“定点瞄准”的通信和控制基础，但要稳定满足 2s 内 D1≤2cm，建议补齐“激光光斑坐标相对靶心坐标”的二维闭环，而不是只让靶心靠近画面中心。

## 踩坑记录

### MSPM0 PWM 占空比（CC=0 ≠ 0% 占空比）

**MSPM0 边沿对齐 PWM 模式下，CC 值与占空比的关系是反直觉的**：

| CC 值 | 实际占空比 | 说明 |
|-------|-----------|------|
| CC = 0 | **≈ 100%** | 计数器从 0 开始立刻匹配 CC0 → 输出翻转 → 直到 period 才复位，近乎满占空比 |
| CC = period（200） | **0%** | CC 匹配和 period 匹配同时发生 → 输出不翻转 → 真正的 0% 占空比 |
| CC = 100 | **50%** | 正常中间值 |

**这导致了上电自动跑车的 bug**：`MotorDrv_SetSpeed(0, 0)` 把 duty 设为 0 → `Drv_PWM_SetDuty(timer, 0, 0)` → CC=0 → 满占空比 → 方向引脚设为正转 → 电机全速跑！

**修复方案**（已在 `motor_drv.c` 中实施）：
- `MotorDrv_SetSpeed(0, 0)` → 直接调用 `MotorDrv_Stop()` 并 return（最安全的停车路径）
- 单侧 duty=0 时，用 `MOTOR_PWM_MAX`（200）代替 0（CC=200 = period = 真 0% 占空比）
- `MotorDrv_Stop()` 中 duty 值从 0 改为 `MOTOR_PWM_MAX`（200）

**关键规则**：
- **在 MSPM0 上设 PWM 占空比时，0 永远不能表示"停转"，必须用 period 值**
- **SysConfig 生成的 PWM 初始 CC=200（=period）是正确的 0% 占空比，不要手动改成 0**
- **TB6612 等 H 桥芯片：IN 引脚全低 = coast（滑行），IN 引脚全高 = brake（制动）**

### MSPM0 中断配置

SysConfig 生成的 `ti_msp_dl_config.c` **只做两件事**：
1. 配置 GPIO/UART/Timer 外设寄存器
2. 调用 `NVIC_SetPriority()` 设置中断优先级

**SysConfig 不会做的事**（必须手动补）：

| 缺失项 | 必须手动添加 | 添加位置 |
|--------|-------------|---------|
| `NVIC_EnableIRQ()` | 使能 NVIC，否则中断信号到不了 CPU | 在各 `Drv_xxx_Init()` 函数里 |
| `GROUP1_IRQHandler` | MSPM0 的 GPIOA 中断向量，SysConfig 生成的是 `ENC_ISR`（普通函数），不是真正的 IRQ Handler | `Drv/drv_gpio_int.c` |

**中断向量归属表**：

| 硬件中断向量 | 实际文件 | 服务对象 |
|-------------|---------|---------|
| `GROUP1_IRQHandler` | `Drv/drv_gpio_int.c` | GPIOA 中断（编码器 ENC + 按键 KEY） |
| `UART1_IRQHandler` | `Drv/drv_uart.c` | UART_GYRO（陀螺仪串口） |
| `UART2_IRQHandler` | `Drv/drv_uart.c` | UART_BT（蓝牙串口） |
| `TIMER_MAIN_ISR` | `Drv/drv_timer.c` | 系统定时器 |
| `SysTick_Handler` | `Bsp/bsp_init.c` | 1ms SysTick 计时 |

**关键规则**：
- **中断向量必须写在 `Drv/` 层**，不要写在 `empty.c`（主文件），否则换测试代码时会丢失
- **每个 `Drv_xxx_Init()` 必须包含对应的 `NVIC_EnableIRQ()`**，参见 `drv_gpio_int.c` 和 `drv_uart.c` 的做法
- 灰度传感器极性：**GPIO HIGH = 白地（不在线上）**，`grayscale.c` 已做取反（`? 0x00 : 0x01`），bit=1 表示黑线

### edge_yaw 旧字段清理

`tracking.c` 中 `Tracking_ConfigDef` 曾包含 `edge_yaw` 字段，用于旧版"大半径旋转"算法。当前循迹采用三段式丢线逻辑（正常循迹 → 直线行驶 → 原地旋转搜索），`edge_yaw` 已无作用。2026-07-15 已清理：

- **删除了** `edge_yaw` 字段、`g_corner_dir` / `g_corner_hold` 变量、`CORNER_HOLD_TICKS` / `CORNER_EDGE_THRESHOLD` 宏定义
- **搜索方向**现在仅依赖 `g_recent_deviation` → `g_last_seen_deviation` → `g_last_turn_dir` 回退链

## 小车控制方案（2025 E 题）

**方案选择：无陀螺仪，三段式丢线循迹**

小车端 MSPM0G3507 **没有陀螺仪**（陀螺仪在云台端），无法做角度闭环。当前循迹采用灰度传感器 + PD 控制器 + 三段式丢线处理：

```
循迹架构：
  灰度偏差 → PD 控制器（比例+微分）→ yaw 偏移
  base_speed ± yaw → 目标左/右轮速度
  编码器实测速度 → 速度 PID（增量式）→ PWM（0 ~ 200）

丢线三段式状态机：
  STATE_NORMAL        — 正常循迹，灰度读到线 → PD 控制
  STATE_LOST_STRAIGHT — 丢线后直线行驶 LOST_STRAIGHT_TICKS 个周期（约 300ms）
  STATE_LOST_SEARCH   — 直线阶段结束仍未找到线 → 原地中心旋转搜索

搜索方向优先级：
  最后时刻偏差方向（g_recent_deviation）
  → 最后看到线时的偏差方向（g_last_seen_deviation）
  → 上次转向方向（g_last_turn_dir）

蓝牙命令：
  N=1D / N=2D / ... → 设定圈数并启动
  STOP → 紧急停车
```

**PID 参数需硬件校准**：
- `base_speed`：基准速度（直线前进），单位 counts/20ms，需根据编码器精度实测
- `kp` / `kd`：PD 控制器参数，灰度偏差 → yaw 偏移量
- `search_speed`：原地旋转速度，一正一反产生中心旋转
- 速度环 `speed_kp` / `ki` / `kd`：目标速度 → PWM，增量式 PID

## 调试流程

嵌入式项目无自动化测试框架，采用硬件在环调试。推荐三层渐进式调试流程：

**第一层 — 绕过应用层，直接调用底层驱动**：
- 直接读 GPIO 引脚状态，验证硬件接线
- 检查中断计数（`enc L/R`）是否变化
- 如果引脚变但计数不变 → 检查 NVIC_EnableIRQ / IRQ Handler

**第二层 — 解耦模块，输出调试信息**：
- 单模块独立运行，蓝牙输出中间数据
- 例如：只测编码器速度 PID，不开循迹

**第三层 — 完整应用层，增加调试输出**：
- 全功能运行，蓝牙周期性输出状态报告
- 字段含义：`[灰度可视化] dev=偏差 edge=边 lap=圈 vt=目标速度 va=实际速度 pwm=输出 enc=累计计数`

各子系统调试方式：
- **小车**：放在 100×100 cm 轨道上，通过蓝牙状态报告观察
- **云台**：验证步进电机角度范围、激光开关、陀螺仪帧和 K230 UART 数据帧；可通过 OLED 查看部分 TX/RX 与状态
- **K230**：使用 K230 IDE（CanMV）帧缓冲区检查矩形、圆心和紫/蓝紫光斑叠加效果，串口确认 `0xF1`、`0xF2` 等帧

先验证原始外设信号，再进行上层融合。

## 提交与 PR 规范

提交信息使用**中文描述**概括变更的子系统：

```
增加云台控制代码
绘制完成小车底板
```

每个提交应保持原子性，只涉及一个子系统（car / Tripod / k230 / docs）。PR 描述应说明变更内容和原因，并在适当时引用 `docs/2025题目.md` 中的相关要求。

## 通信协议参考

小车与云台之间通过 UART 进行基于帧的通信：

```
[0xAA] [length] [cmd] [payload...] [checksum] [0x55]
```

- 小车 → 云台：`CAR_STATUS`、`CAR_POSITION`、`CAR_EMERGENCY`、`CAR_START_AIMING`
- 云台 → 小车：`GIMBAL_STATUS`、`GIMBAL_HIT_CONFIRM`、`GIMBAL_LASER_ON/OFF`
- 双方：`HEARTBEAT`（1 秒间隔）

K230 → 云台当前使用 `src/Tripod/Bsp/k230_uart.c` 中定义的帧格式：

```
[0xAA] [0xFF] [func_code] [length] [payload...] [sc] [ac]
```

常用功能码：
- `0xF1`：紫/蓝紫光斑坐标
- `0xF2`：靶面/矩形中心坐标
- `0xF3`：圆形点集（K230 侧发送默认注释）
- `0xF4`：偏差
- `0xF5`：命中确认

云台 → K230 当前使用：

```
[0xAA] [0xFE] [cmd] [length] [payload...] [sc] [ac]
```

命令码参见 `src/Tripod/Bsp/k230_uart.h`。

## 引脚分配参考

引脚映射表位于 `docs/pin_allocation.md`（小车 MSPM0）和 `docs/pin_allocation_stm32g474.md`（云台 STM32G474）。修改小车引脚分配时同步更新 SysConfig/用户标签与文档；修改云台引脚分配时同步更新 `src/Tripod/Tripod.ioc`、`Core/Inc`/`Core/Src` 生成文件与文档。
