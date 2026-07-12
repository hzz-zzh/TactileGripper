#ifndef GRIPPER_CAN_CONFIG_H
#define GRIPPER_CAN_CONFIG_H

#ifndef GRIPPER_CAN_ENABLE
/*
 * 默认关闭CAN运行时接入，避免未连接总线、无ACK或终端电阻异常时影响电机调试。
 * 需要联调CAN时改为1，再用CAN分析仪确认心跳和状态帧。
 */
#define GRIPPER_CAN_ENABLE             0U
#endif

#define GRIPPER_CAN_NODE_ID              8U
#define GRIPPER_CAN_MASTER_NODE_ID       24U
#define GRIPPER_CAN_BROADCAST_NODE_ID    255U

#define GRIPPER_CAN_STATUS_PERIOD_MS     20U
#define GRIPPER_CAN_DIAG_PERIOD_MS       100U
#define GRIPPER_CAN_HEARTBEAT_PERIOD_MS  500U
#define GRIPPER_CAN_CONTROL_TIMEOUT_MS   500U
#define GRIPPER_CAN_TASK_PERIOD_MS       5U

#endif
