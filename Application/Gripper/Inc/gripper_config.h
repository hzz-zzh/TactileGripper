#ifndef GRIPPER_CONFIG_H
#define GRIPPER_CONFIG_H

/* 夹爪控制周期为5 ms，对应200 Hz位置与回零状态机。 */
#define GRIPPER_CONTROL_PERIOD_TICKS          10U
#define GRIPPER_CONTROL_PERIOD_MS             5U

#define GRIPPER_AUTO_HOME_ON_BOOT             1U
#define GRIPPER_OPEN_DIRECTION                (-1)
#define GRIPPER_MOTOR_COUNTS_PER_TURN         65536L
#define GRIPPER_GEAR_RATIO                    10.0f

/* 机械限位采用低速搜索，避免减速器和蜗轮蜗杆在端点产生高速回弹。 */
#define GRIPPER_HOME_SPEED_RPM                500.0f
#define GRIPPER_HOME_CURRENT_LIMIT_A          1.00f
#define GRIPPER_HOME_STALL_CURRENT_A          0.60f
#define GRIPPER_HOME_STALL_SPEED_RPM          25.0f
#define GRIPPER_HOME_STALL_IGNORE_MS          500U
#define GRIPPER_HOME_STALL_TIME_MS            300U
#define GRIPPER_HOME_TIMEOUT_MS               20000U
#define GRIPPER_HOME_MAX_TRAVEL_TURNS         200L
#define GRIPPER_HOME_MIN_TRAVEL_COUNTS        8192L

#define GRIPPER_SAFE_OPEN_POSITION_PERMILLE   50
#define GRIPPER_POSITION_KP_RPM_PER_TURN      350.0f
#define GRIPPER_POSITION_MAX_SPEED_RPM        600.0f
#define GRIPPER_POSITION_DEADBAND_COUNTS      64L
#define GRIPPER_OPERATION_CURRENT_LIMIT_A     0.60f
/* 回零使用短斜坡抑制速度阶跃冲击，位置环目标由外环连续生成，不再叠加MCSDK斜坡。 */
#define GRIPPER_HOME_SPEED_RAMP_MS             50U
#define GRIPPER_POSITION_SPEED_RAMP_MS         0U

#endif
