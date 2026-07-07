# MotorControl

本目录只负责电机执行层，不包含夹爪开闭端、行程映射或触觉策略。

- `motor_control_service.*`：MCSDK 任务调度、电机启停、速度/电流指令、反馈快照和底层保护；
- `kth7812_speed_pos_fdbk.*`：编码器角度、速度与多圈位置；
- 其余文件：MCSDK 配置、FOC、PWM 和电流采样。当前 USART1 分配给右侧触觉传感器，
  Motor Pilot/ASPEP 暂时停用，调试输出统一走 RS485。

当前电机执行层允许的 Iq 指令上限为 2.0 A，软件过流保护阈值为 2.3 A。
MCSDK 的 `IQMAX_A` 同步为 2.0 A，避免速度环输出仍被内部参数限制在 1 A。

夹爪业务状态机位于 `../Application/Gripper`。应用代码、CAN、RS485 和触觉策略不应
直接包含 `mc_api.h`，统一通过 `gripper_service.h` 提交命令。
