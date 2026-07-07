# STM32H723 触觉夹爪控制工程

## 1. 当前能力

工程基于 STM32H723VGH6、FreeRTOS 和 ST MCSDK 6.4.1，当前已接入：

- 三相无刷电机有感 FOC；
- TIM1 互补 PWM 与 ADC1/ADC2 三电阻采样；
- KTH7812 绝对编码器角度、速度和多圈位置；
- 电流环、速度环和夹爪位置环；
- 夹爪开端/闭端自动回零与 0～1000 行程映射；
- UART1 夹爪调试命令与运行数据输出；
- UART2 端子保留，当前不作为调试控制台；
- WS2812E-1313 单灯运行状态指示；
- 编码器、MCSDK、母线电压、软件过流和回零异常保护。

触觉数据、自适应力控、CAN FD 和 RS485 尚未接入，但应用层接口已经与
MCSDK 解耦，后续通讯模块只需要提交夹爪命令，不应直接调用电机底层接口。

## 2. 软件分层

```text
Core/
  STM32CubeMX 生成的时钟、GPIO、ADC、TIM、SPI、UART、DMA 和启动代码

MotorControl/
  Inc/motor_control_service.h       电机执行器公共接口
  Src/motor_control_service.c       MCSDK 调度、启停、反馈和底层保护
  Inc/kth7812_speed_pos_fdbk.h      KTH7812 编码器接口
  Src/kth7812_speed_pos_fdbk.c      编码器采集与速度/多圈计算
  Inc|Src/...                       MCSDK 配置及驱动

Application/Gripper/
  Inc/gripper_config.h              夹爪机械与控制参数
  Inc/gripper_controller.h          纯夹爪状态机接口
  Src/gripper_controller.c          回零、位置映射和位置外环
  Inc/gripper_service.h             面向 UART/CAN/RS485/触觉层的统一 API
  Src/gripper_service.c             FreeRTOS 任务、命令队列和电机适配

Application/StatusIndicator/
  Inc/status_indicator.h            状态指示任务接口
  Src/status_indicator.c            夹爪状态到颜色/动画的映射策略

Platform/
  Inc|Src/debug_uart_transport.*    调试串口发送互斥与统一出口
  Inc|Src/ws2812_led.*              SPI3+DMA的WS2812底层驱动

MDK-ARM/
  Keil 工程文件
```

依赖方向固定为：

```text
UART / CAN / RS485 / 触觉策略
             ↓
       GripperService
             ↓
     GripperController
             ↓
   MotorControlService
             ↓
     MCSDK + KTH7812
```

`gripper_controller.c` 不依赖 HAL、FreeRTOS 或 MCSDK，便于后续在 PC 上做状态机
单元测试。`motor_control_service.c` 不包含夹爪开闭、回零和触觉概念，只提供电机
执行能力。

## 3. 控制链路

正常夹爪位置控制为三级串联：

```text
夹爪目标 0~1000
      ↓
位置环 200 Hz：位置误差 → 电机速度目标 rpm
      ↓
速度环 1 kHz：速度误差 → Iq 目标 A
      ↓
电流环 16 kHz：Iq/Id 误差 → SVPWM
```

- 位置环在 `Application/Gripper/Src/gripper_controller.c`；
- 速度环和电流环由 MCSDK 执行；
- KTH7812 位于电机轴，位置计数不会自动除以 10:1 减速比；
- 夹爪位置 `0` 表示全开，`1000` 表示全闭；
- 保持状态仍运行位置外环，受到外力偏移后会回到目标位置。

## 4. 自动回零流程

默认 `GRIPPER_AUTO_HOME_ON_BOOT=1`，上电后自动执行：

1. 等待编码器、母线电压和电机服务完成预检查；
2. 启动 MCSDK，由其完成电角度对齐；
3. 以受限速度和 0.3 A 限流寻找开端；
4. 检测“低速 + 持续转矩 300 ms”后记录开端；
5. 反向寻找闭端并记录总行程；
6. 检查最小行程和最大 20 电机圈限制；
7. 回到开端 5% 的安全位置；
8. 进入 `READY`，接受 0～1000 位置命令。

回零期间不要用手持续阻挡夹爪。堵转判定依赖实际机构负载，首次装机应准备急停，
并从较低电源限流开始验证开闭方向。

## 5. 关键参数

夹爪参数统一位于 `Application/Gripper/Inc/gripper_config.h`：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `GRIPPER_OPEN_DIRECTION` | -1 | 负转速寻找开端，机构方向相反时改为 1 |
| `GRIPPER_HOME_SPEED_RPM` | 1000 rpm | 回零电机转速 |
| `GRIPPER_HOME_CURRENT_LIMIT_A` | 2.00 A | 回零速度环 Iq 限幅，需低于电源和驱动安全限流 |
| `GRIPPER_HOME_STALL_CURRENT_A` | 0.90 A | 堵转电流门限，使用 Iq/Iq_ref 判断 |
| `GRIPPER_HOME_STALL_SPEED_RPM` | 30 rpm | 堵转速度门限 |
| `GRIPPER_HOME_STALL_IGNORE_MS` | 1200 ms | 每次开始寻找端点后的起步忽略时间，用于避开静摩擦、蜗杆齿隙和结构弹性形变 |
| `GRIPPER_HOME_STALL_TIME_MS` | 500 ms | 起步忽略结束后，满足低速和高 Iq 电流所需的连续确认时间 |
| `GRIPPER_HOME_TIMEOUT_MS` | 10000 ms | 单端搜索超时 |
| `GRIPPER_HOME_MIN_TRAVEL_COUNTS` | 8192 count | 开端到闭端的最小有效行程，低于该值判定回零行程异常 |
| `GRIPPER_POSITION_KP_RPM_PER_TURN` | 350 | 位置 P 环增益 |
| `GRIPPER_POSITION_MAX_SPEED_RPM` | 600 rpm | 夹爪位置模式最大电机速度 |
| `GRIPPER_POSITION_DEADBAND_COUNTS` | 64 | 到位死区 |
| `GRIPPER_OPERATION_CURRENT_LIMIT_A` | 0.60 A | 正常位置控制 Iq 限幅 |

调参顺序必须保持为电流环 → 速度环 → 位置环 → 回零判定 → 触觉力控。外环尚未稳定时
不应通过提高内层限流来掩盖振荡。

## 6. 应用 API

公共接口位于 `Application/Gripper/Inc/gripper_service.h`：

```c
bool GripperService_Home(void);
bool GripperService_SetPosition(int16_t position_permille);
bool GripperService_Stop(void);
bool GripperService_ClearFaults(void);
void GripperService_GetStatus(GripperStatus_t *status);
```

CAN、RS485 和触觉策略层应调用上述接口。命令通过 FreeRTOS 消息队列进入夹爪任务，
避免通讯中断或其他任务直接修改控制状态。

示例：

```c
GripperStatus_t status;

GripperService_GetStatus(&status);
if (status.homed && (status.faults == GRIPPER_FAULT_NONE))
{
  (void)GripperService_SetPosition(500); /* 移动到行程中点 */
}
```

## 7. RS485 调试命令

当前调试命令从 RS485 调试口输出和输入，接口为 USART10：PE3 TX、PE2 RX、PC13 EN，
参数为 460800-8-N-1。USART1/USART2 已经分配给左右触觉传感器，不再承担调试输出。

| 命令 | 作用 |
| --- | --- |
| `H` 或 `P` | 重新执行夹爪回零 |
| `G0` | 移动到全开位置 |
| `G500` | 移动到行程中点 |
| `G1000` | 移动到全闭位置 |
| `S` | 停止 PWM，夹爪进入 STOPPED |
| `C` | 清除电机与夹爪锁存故障 |
| `Q` | 输出一次夹爪状态快照 |

原纯电机命令 `T...` 和 `Z` 在夹爪模式中被禁用，避免绕过回零后的行程限制。

`Q` 输出：

```text
gripper: state,fault,homed,target_permille,pos_permille,target_count,pos_count,open_count,close_count
```

回零过程中周期日志使用带字段名的诊断格式：

```text
homing: state=HOMING_OPEN,ms=1250,stall=80,pos=12345,open=0,close=0,travel=12345,travel_mt=188,speedr=-800,speed=-4,iqr=260,iq=255,enc=1,fault=0x00000000,motor=0x00000000,mc=0x0000
```

回零完成后恢复纯数值位置控制日志：

```text
position: iqr,idr,iq,id,speedr,speed,posr,pos
```

电流单位 mA，速度单位 rpm，位置单位为电机编码器多圈 count。`travel_mt`为电机
毫圈，1000表示1电机圈。`homing:`里的`ms`是当前状态持续时间，`stall`是堵转
判定已累计时间。当前堵转只根据低速和 Iq 电流高两个条件累计，不使用位置窗口参与判断。
`homing:`默认约100 ms输出一次，`stall`累计超过200 ms后临时加密到约20 ms输出一次，
状态切换和故障会立即输出。

## 8. WS2812状态指示灯

状态灯为5V供电的WS2812E-1313，数据连接PC12/SPI3_MOSI。SPI3使用48MHz内核时钟
16分频得到3MHz，每个WS2812数据位编码为4个SPI位：`0=1000`、`1=1100`。
DMA1 Stream2发送12字节GRB数据，不屏蔽电机FOC中断。

WS2812发送缓存固定放在AXI SRAM起始地址`0x24000000`，工程中IRAM2从`0x24000100`
开始，前256字节保留给DMA外设缓存。后续调整Keil内存分布、scatter文件或D-Cache
策略时，需要同步检查这段缓存。

| 夹爪状态 | 指示效果 |
| --- | --- |
| PRECHECK / STARTING / HOMING_OPEN / HOMING_CLOSE / MOVING_SAFE | 紫色常亮 |
| 其他无故障状态 | 绿色常亮 |
| 任意故障 | 红色常亮 |

为减少眩光和5V电源扰动，各颜色通道限制在约10%亮度。指示灯属于非安全功能，驱动
失败不会阻止电机保护和夹爪控制。

硬件注意：手册规定VDD=5V时DIN高电平最小为0.7VDD，即3.5V；当前PCB由PC12直接
输出3.3V，低于手册保证值。硬件已经定型，因此软件按现板直连实现，但首板需要验证
不同供电电压和温度下是否出现不亮、颜色错误或偶发闪烁。

## 9. 硬件资源

| 功能 | STM32H723 资源 |
| --- | --- |
| U/V/W 上桥 PWM | PE9 / PE11 / PE13，TIM1 CH1/CH2/CH3 |
| U/V/W 下桥 PWM | PE8 / PE10 / PE12，TIM1 CH1N/CH2N/CH3N |
| A/B/C 相电流 | PA1 / PA2 / PA5 |
| 母线电流 | PA6 ADC2_IN3 |
| 母线电压 | PA7 ADC2_IN7 |
| 外接 NTC | PA3 ADC1_IN15 |
| KTH7812 | SPI1：PB3 SCK、PB4 MISO、PB5 MOSI、PA15 硬件 NSS |
| RS485 调试口 | USART10：PE3 TX、PE2 RX，PC13 EN，460800 baud |
| 右侧触觉传感器 | USART1：PA9 TX、PA10 RX，460800 baud |
| 左侧触觉传感器 | USART2：PD5 TX、PD6 RX，460800 baud |
| 状态指示灯 | WS2812E-1313：PC12 SPI3_MOSI，DMA1 Stream2 |

主要硬件参数：24 V 母线、5 mΩ 分流、20 倍电流放大、1/21 母线分压、14 极对、
电机轴侧 KTH7812、10:1 减速器。

## 10. 故障处理

底层电机故障位于 `MotorControlFault_t`，夹爪业务故障位于 `GripperFault_t`。任一锁存
故障都会请求关闭 PWM。主要保护包括：

- 编码器不可靠；
- MCSDK 故障；
- 母线低于 10 V 或高于 30 V；
- dq 软件电流超过 1.5 A；
- 电机启动超时；
- 回零超时、行程过短或超过 20 电机圈。

清故障后不会直接恢复动作，需要重新发送 `H` 完成回零。外接 NTC 尚未标定，当前
没有启用温度换算与过温停机，获得 NTC 参数后应在电机服务保护层补充。

## 11. 工程使用说明

Keil 工程：

```text
MDK-ARM/TactileGripper_H723VGH6.uvprojx
```

修改 CubeMX `.ioc` 并重新生成代码时，需要保留：

- `Core/Inc/FreeRTOSConfig.h` 中 2000 Hz tick；
- `Core/Src/main.c` 中电机服务和夹爪服务任务创建；
- Keil 中 `Application/Gripper/Inc` 包含路径和应用源文件；
- SPI3、PC12和DMA1 Stream2状态灯配置；
- MCSDK、KTH7812 与板级 ADC/TIM 配置。

本次架构调整未执行 Keil 编译，需在本机 Keil 中完成全量构建后再烧录。

## 12. 首次机构联调顺序

1. 抬起夹爪，确认机构全行程无硬性卡死；
2. 使用限流电源，准备随时断电；
3. 上电先观察 `homing:` 日志，回零完成后确认恢复 `position:` 日志；
4. 若首次寻找方向不是开端，立即断电并修改 `GRIPPER_OPEN_DIRECTION`；
5. 验证开端堵转后能自动反向寻找闭端；
6. 输入 `Q`，确认 `homed=1` 且开闭计数间距合理；
7. 依次测试 `G100`、`G500`、`G900`，暂不直接命令端点；
8. 确认位置环无振荡后再测试 `G0` 和 `G1000`；
9. 最后验证 `S`、编码器断线和故障清除流程。
## 当前通信配置更新

当前调试阶段使用 RS485 作为调试口，触觉传感器独占 USART1/USART2：

| 功能 | STM32H723 资源 |
| --- | --- |
| 右侧触觉传感器 | USART1：PA9 TX、PA10 RX，460800 baud |
| 左侧触觉传感器 | USART2：PD5 TX、PD6 RX，460800 baud |
| RS485 调试口 | USART10：PE3 TX、PE2 RX，PC13 EN，460800 baud |
| CAN | 默认关闭：`GRIPPER_CAN_ENABLE=0` |

触觉帧率输出格式：`touchfps: ru,rl,lu,ll`。触觉力摘要输出格式：`touch: sample,mask,ru_n,ru_t,rl_n,rl_t,lu_n,lu_t,ll_n,ll_t`。
