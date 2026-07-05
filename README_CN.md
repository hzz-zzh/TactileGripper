# STM32H723 触觉夹爪电机控制工程

## 1. 工程简介

本工程基于 STM32H723VGH6、FreeRTOS 和 ST MCSDK 6.4.1，实现三相无刷电机的有感 FOC 控制、KTH7812 绝对磁编码器反馈、自动双端回零以及夹爪位置控制。

当前版本专注于电机控制。触觉数据采集、自适应抓取、CAN FD 和 RS485 尚未接入。

主要能力：

- TIM1 产生 16 kHz 中心对齐六路互补 PWM；
- ADC1/ADC2 同步三电阻电流采样；
- KTH7812 提供角度、转速和多圈位置反馈；
- 支持电流环、速度环和夹爪位置环；
- 上电自动电角度对齐和双机械限位回零；
- 支持 Motor Pilot/ASPEP；
- 编码器、母线电压、过流、堵转和回零异常均会锁存故障并关闭 PWM。

## 2. 目录说明

| 路径 | 说明 |
| --- | --- |
| `Core` | H723 时钟、GPIO、ADC、TIM、SPI、UART、DMA、中断和 FreeRTOS 启动代码 |
| `MotorControl` | MCSDK 配置、KTH7812 驱动、夹爪服务和应用 API |
| `MDK-ARM` | Keil ARMCC 5 工程和编译产物 |
| `../Reference` | H745 MCSDK 参考工程及通用 MCSDK 库源码 |
| `../Tests` | 编码器16位跨零、多圈位置和回零判定单元测试 |

## 3. 硬件资源

| 功能 | STM32H723 资源 |
| --- | --- |
| U/V/W 上桥 PWM | PE9 / PE11 / PE13，TIM1 CH1/CH2/CH3 |
| U/V/W 下桥 PWM | PE8 / PE10 / PE12，TIM1 CH1N/CH2N/CH3N |
| A/B/C 相电流 | PA1 ADC1_IN17 / PA2 ADC1、ADC2_IN14 / PA5 ADC2_IN19 |
| 母线电流 | PA6 ADC2_IN3 |
| 母线电压 | PA7 ADC2_IN7 |
| 外接 NTC | PA3 ADC1_IN15 |
| KTH7812 | SPI1：PB3 SCK、PB4 MISO、PB5 MOSI、PA15 硬件 NSS（SPI1 自动片选） |
| Motor Pilot | USART1：PA9 TX、PA10 RX，1843200 baud，DMA1 Stream0/1 |
| 调试串口 | USART2：PD5 TX、PD6 RX，921600-8-N-1 |

SPI1 引脚以实际 PCB 为准，优先级高于旧版原理图中的编码器网络标注。

## 4. 时钟和控制参数

- 外部晶振：25 MHz；
- CPU：550 MHz；
- TIM1：275 MHz；
- ADC：24 MHz；
- SPI1：48 MHz 外设时钟、8 分频，实际 SCK 为 6 MHz，Mode 3，16 bit；
- PWM：16 kHz 中心对齐，MCU 死区约 500 ns；
- FreeRTOS tick：2000 Hz；
- HAL 时基：TIM6；
- 速度环：1 kHz；
- 安全检查：2 kHz；
- 夹爪位置环：200 Hz。

电机与功率级参数：

- 14 极对；
- 相电阻 0.555 Ω；
- 相电感约 0.142 mH；
- 最大转子速度 7790 rpm；
- 电机输出连接10:1减速器，KTH7812位于电机轴侧；电机角度和转速换算到减速器输出端时需除以10；
- 分流电阻 5 mΩ，电流放大倍数 20；
- 母线分压比 1/21；
- 标称母线 24 V；
- 软件运行限流 1 A；
- ADC2 模拟看门狗硬限制约 ±2 A；
- 欠压 10 V，过压 30 V。

## 5. KTH7812 编码器

编码器驱动位于：

- `MotorControl/Inc/kth7812_speed_pos_fdbk.h`
- `MotorControl/Src/kth7812_speed_pos_fdbk.c`

驱动按 KTH7812-N 型号处理完整16位角度数据（0～65535，无CRC），并实现：

- SPI1 Mode 3 读取；
- SPI超时和连续通信错误计数；
- 角度跨零处理；
- 正反向多圈累计；
- 机械速度估算；
- 机械角到电角度转换；
- 电角度对齐偏置；
- SPI 超时或连续通信错误时关闭 PWM。

## 6. 上电与自动回零

FreeRTOS 启动后，夹爪服务依次执行：

1. 检查编码器、母线电压和 MCSDK 状态；
2. 使用 0.3 A d 轴电流完成转子电角度对齐；
3. 以低速寻找开端机械限位并记录 `open_count`；
4. 寻找闭端机械限位并记录 `close_count`；
5. 退回到总行程的 5%；
6. 进入 `GRIPPER_MOTOR_READY` 状态。

堵转判定条件为低速并持续产生转矩 300 ms。单端回零超时为 10 s，允许的最大搜索行程为 20 电机圈。

归一化位置定义：

- `0`：全开；
- `1000`：全闭；
- 回零完成前拒绝位置命令。

## 7. 应用层 API

接口声明位于 `MotorControl/Inc/gripper_motor_service.h`。

```c
bool GripperMotor_SetPosition(int16_t position_permille);
bool GripperMotor_SetSpeed(float motor_rpm);
bool GripperMotor_SetCurrent(float iq_a);
bool GripperMotor_Rehome(void);
void GripperMotor_Stop(void);
bool GripperMotor_ClearFaults(void);
void GripperMotor_GetStatus(GripperMotorStatus_t *status);
```

位置控制示例：

```c
GripperMotorStatus_t status;

GripperMotor_GetStatus(&status);
if (status.homed && status.faults == GRIPPER_FAULT_NONE)
{
  (void)GripperMotor_SetPosition(500); /* 移动到行程中点 */
}
```

`GripperMotorStatus_t` 可读取当前状态、故障、原始多圈计数、开闭端点、归一化位置、目标位置、速度、Iq、母线电流、电压和编码器错误计数。

## 8. 故障与安全机制

以下故障会锁存并强制关闭六路 PWM：

- KTH7812 SPI 连续错误；
- MCSDK 电流环或速度反馈故障；
- ADC2 母线电流模拟看门狗越界；
- 母线欠压或过压；
- 回零超时；
- 回零行程超过 20 圈；
- 机械堵转异常。

外接 NTC 的 PA3 ADC 采样已经配置，但原理图只给出了板端 10 kΩ 上拉，没有给出所接 NTC 的 R25 和 Beta 参数。当前未启用过温关断，获得 NTC 型号后必须标定温度曲线和阈值。

## 9. 编译和烧录

### UART2 调试输出

调试串口使用板载 UART2 接口：PD5 为 MCU TX，PD6 为 MCU RX，参数为 921600 baud、8 数据位、无校验、1 停止位。

当前默认源码为`FOC32`的“纯Iq电流环0.5A持续运行”测试：`CURRENT_ADC_CALIBRATION_MODE=0`、`PWM_INSPECTION_MODE=0`。FOC29已确认编码器方向需要`-1`，FOC31已验证速度环可以跑到3000rpm；本版回到纯电流环，只给`Iq=0.5A、Id=0A`，用于继续观察电流环稳定性和电机响应。

示例：

```text
RST,raw=0x00E00000,bor=1,pin=1,por=1,soft=0,iwdg=0,wwdg=0,lpwr=0,cpu=0
PWM,f=16000,duty=10,arr=8592,ccr=859,dtg=137,ccer=0x00000555,bdtr=0x00008C89,cnt=1234,err=0x00
```

### 当前纯电流环配置（FOC32）

- PWM频率：10kHz中心对齐互补PWM。
- TIM1 CKD：DIV1；计数器预分频为2，避免MCSDK的16位SVPWM周期常量溢出；`DTG=137`，MCU实际死区约500ns。
- 串口2发送字符`P`启动，发送`S`或`s`停止。
- 启动前用`Id=300mA`在500ms内平滑完成电角度对齐，确保转子确实拉到已知位置；进入RUN后`Id`参考回到0。
- KTH7812方向保持为`-1`，机械角、速度反馈和FOC电角度方向已由FOC29验证。
- 对齐完成后不再下速度目标，而是通过`MC_SetCurrentReferenceMotor1_F()`持续写入`Iq=0.5A、Id=0A`。
- Iq目标用800ms斜坡从0A升到0.5A，之后一直保持，直到串口发送`S`或`s`停止。
- 软件dq电流保护阈值900mA，电机轴超过3500rpm仍会立即关闭PWM，防止纯电流环空载意外拉高转速。
- 当前NTC尚未标定，持续堵转测试时需要人工监控电机、MOS和分流电阻温升。
- 看到`FOC32,READY`后只发送一次字符`P`。
- 运行时日志使用固定三环格式：`current/speed/position: iqr,idr,iq,id,speedr,speed,posr,pos`。
- 本版保留MCSDK `R3_2`驱动原有的`Iab`符号，重点观察`Iq=500mA、Id=0`时三相电流、实际FOC角度和编码器角度是否稳定。
- `RAW`日志打印`sec/j1/j2/ofs/ccr`，用于确认MCSDK当前扇区实际读取了哪两个ADC通道。
- 启动前检查A/B/C三相零偏，任一零偏无效都会输出`phase_offset_invalid`并拒绝PWM。
- `FOC32,RUN`表示已经进入纯电流环；正常日志里`iqr`最终应接近500mA，`idr`为0。
- 冒号后只打印数值，不带字段名；电流单位为mA，速度单位为rpm，位置单位为编码器多圈计数。
- 发送`S`后输出`FOC32,STOP`和`FOC32,COMPLETE,current_loop_test=1,pwm=off`。
- 模式切换位置：`MotorControl/Src/gripper_motor_service.c`顶部的`COMMISSIONING_CONTROL_MODE`，改成`COMMISSIONING_CONTROL_MODE_SPEED`即可回到速度外环测试；同时`MotorControl/Inc/drive_parameters.h`里的`DEFAULT_CONTROL_MODE`当前设为`MCM_TORQUE_MODE`，若长期跑速度模式可以改回`MCM_SPEED_MODE`。

### 当前启用的安全诊断（PWM6）

- `PWM_INSPECTION_MODE=1`时，MCSDK与FOC任务不会启动，FOC32测试会被禁用。
- 串口看到`PWM6,READY`后只发送一次字符`P`。
- 输出一个`U正、V/W负`的极小固定电压矢量，持续10ms后强制关闭六路PWM。
- 同步采集U、W两路原始ADC，并根据三相电流和为零估算V相变化量。
- 若仍出现BOR/POR复位，优先检查12V/5V/3.3V供电和功率地回流；若不复位，则问题位于FOC闭环或角度/相序配置。

### PWM7正反矢量与三相极性测试

- 当前主程序调用PWM7，FOC和编码器对齐均不运行。
- 共四步：正向矢量U/W、正向矢量U/V、反向矢量U/W、反向矢量U/V。
- 每看到一次`PWM7,READY`后发送一次`P`，每步仅输出5ms并立即关闭PWM。
- V相通过ADC2通道14直接采集，不再只依靠三相和为零进行估算。
- 正反矢量的U/V/W变化量应整体反号；任一路连续三次超过1000计数立即中止。
- 本档仍保留500mA软件保护；出现复位、异常电流或电源限流时不要连续重试。

### 当前启用的安全诊断（PWM8）

- 当前主程序调用PWM8，FOC和编码器对齐均不运行。
- 共四步：正向矢量U/W、正向矢量U/V、反向矢量U/W、反向矢量U/V。
- 每看到一次`PWM8,READY`后发送一次`P`，每步内部先执行`BASE`再执行`VEC`，每段8ms，段间自动关断。
- `BASE`为三相50%占空；`VEC`为U相±20计数、V/W相反向各∓10计数的小矢量。
- 重点看`PWM8,DONE`里的`dU`、`dV`或`dW`，它们是微矢量平均值减去同条件基线平均值后的结果，可用于排除零点漂移和开关共模噪声。
- 任一路连续三次超过1000计数立即中止；出现BOR/POR复位、电源限流或明显异响时停止测试，不要连续重试。

字段说明：

- `RST`打印上次复位原因；重点关注`bor`、`iwdg`和`wwdg`；
- `PWM`每500 ms打印一次TIM1配置；`err=0x00`表示六路输出均成功启动；
- PE9/PE8为U高/低，PE11/PE10为V高/低，PE13/PE12为W高/低；
- 每对高低侧必须互补且无重叠，MCU死区目标约500 ns；
- 检查完成后将`PWM_INSPECTION_MODE`改为0，才能恢复电流环联调流程。

推荐用 USB 转 TTL 模块连接：板卡 UART2_TX 接转换器 RX，UART2_RX 接转换器 TX，并共地。电平必须为 3.3 V TTL。

使用 Keil MDK ARMCC 5.06 打开：

`MDK-ARM/TactileGripper_H723VGH6.uvprojx`

当前工程已通过全量构建：

- 0 Error；
- 0 Warning；
- 生成 AXF 和 HEX 文件。

HEX 默认输出位置：

`MDK-ARM/TactileGripper_H723VGH6/TactileGripper_H723VGH6.hex`

单元测试在仓库外层目录执行：

```powershell
python -m unittest discover -s Tests -p "test_*.py" -v
```

## 10. CubeMX `.ioc` 使用注意事项

`TactileGripper_H723VGH6.ioc` 已同步当前硬件资源、时钟和外设配置，包括 ADC1/2、TIM1、SPI1、USART1 DMA、NVIC、TIM6 HAL 时基和 FreeRTOS。

仍需注意：

1. CubeMX 6.17 的 FreeRTOS 配置界面把 tick 输入上限限制为 1000 Hz，本工程使用 2000 Hz。重新生成后必须确认 `Core/Inc/FreeRTOSConfig.h` 中 `configTICK_RATE_HZ` 仍为 2000；
2. MCSDK 板级配置、KTH7812 驱动和夹爪任务并非 CubeMX 自动生成内容；
3. 重新生成前应备份工程，并对比 `main.c`、`stm32h7xx_hal_msp.c`、`stm32h7xx_it.c`、`FreeRTOSConfig.h` 和 Keil 工程文件；
4. 不要用旧 H745 工程的 TIM2 编码器、STDRIVE102BP 或双核配置覆盖当前 H723 配置。

## 11. 首次上板顺序

首次测试建议使用 24 V 限流电源，并先拆除机械负载：

1. 不接电机，检查 PE8～PE13 六路 PWM、互补关系和死区；
2. 检查三相电流和母线电流零偏是否约为 1.65 V；
3. 检查 VBUS 换算；
4. 手动转动电机，确认16位编码器角度、跨零和方向；
5. 以 0.3 A 验证对齐方向；
6. 验证相序和低速电流环；
7. 再验证速度环、双端回零和 0～1000 位置控制；
8. 最后逐项触发编码器、过流、欠压、过压和堵转故障，确认 PWM 能立即关闭。
