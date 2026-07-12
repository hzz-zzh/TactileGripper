#ifndef TACTILE_DATA_H
#define TACTILE_DATA_H

#include <stdint.h>

/* 二指夹爪：左指、右指 */
#define TACTILE_FINGER_COUNT              2U

/* 单个触觉模块由上下两个触觉单元拼接组成 */
#define TACTILE_UNIT_COUNT_PER_MODULE     2U

/* 单个触觉单元为 5 × 5 阵列 */
#define TACTILE_UNIT_ROW_COUNT            5U
#define TACTILE_UNIT_COL_COUNT            5U
#define TACTILE_TAXEL_COUNT_PER_UNIT      \
    (TACTILE_UNIT_ROW_COUNT * TACTILE_UNIT_COL_COUNT)

/* 切向力方向无效值；有效方向为 0~359 deg */
#define TACTILE_DIRECTION_INVALID_DEG     UINT16_MAX





/**
 * @brief 夹爪手指编号。
 */
typedef enum
{
    TACTILE_FINGER_LEFT = 0,     /* 左指 */
    TACTILE_FINGER_RIGHT = 1,    /* 右指 */
} tactile_finger_id_t;






/**
 * @brief 单个触觉模块中的触觉单元编号。
 */
typedef enum
{
    TACTILE_UNIT_UPPER = 0,      /* 上触觉单元 */
    TACTILE_UNIT_LOWER = 1,      /* 下触觉单元 */

} tactile_unit_id_t;





/**
 * @brief 单个触觉单元的数据有效标志。
 *
 * 该字段由数据采集层填写。
 * 上层算法只判断数据是否可用，不关心 IIC、寄存器、厂商状态字等细节。
 */
typedef enum
{
    TACTILE_UNIT_DATA_VALID_TAXEL     = (1U << 0),  /* 触点差值有效 */
    TACTILE_UNIT_DATA_VALID_PROXIMITY = (1U << 1),  /* 接近觉有效 */
    TACTILE_UNIT_DATA_VALID_FORCE     = (1U << 2),  /* 法向力/切向力有效 */

} tactile_unit_data_valid_mask_t;





/**
 * @brief 单个 5x5 触觉单元的一帧动态数据。
 *
 * 只保存每帧变化的数据，不保存：
 * - IIC 地址；
 * - 寄存器地址；
 * - 厂商原始状态字；
 * - 触点二维坐标；
 * - 机械安装方向。
 *
 * taxel_delta 下标约定：
 *
 * index = row * TACTILE_UNIT_COL_COUNT + col
 *
 * row: 0~4，按采集层/厂商数据顺序；
 * col: 0~4，按采集层/厂商数据顺序。
 *
 * 注意：
 * 左右触觉模块存在镜像安装，但镜像关系不在本结构中处理，
 * 而是在 tactile_geometry.h 中通过几何映射处理。
 */
typedef struct
{
    /**
     * @brief 25 个触点的差值数据。
     *
     * 信号正负方向不在本结构中假定，应由标定或算法配置处理。
     */
    int32_t taxel_delta[TACTILE_TAXEL_COUNT_PER_UNIT];

    /**
     * @brief 自电容接近觉原始值。
     *
     * 用于接近趋势观察、标定和调试。
     */
    uint32_t proximity_raw;

    /**
     * @brief 自电容接近觉差值。
     *
     * 用于判断物体接近，但不能单独作为接触成立依据。
     */
    int32_t proximity_delta;

    /**
     * @brief 厂商解算的区域法向力，单位 N。
     */
    float normal_force_n;

    /**
     * @brief 厂商解算的区域切向力大小，单位 N。
     */
    float tangential_force_n;

    /**
     * @brief 厂商解算的切向力方向，单位 deg。
     *
     * 0~359 表示方向有效；
     * TACTILE_DIRECTION_INVALID_DEG 表示方向无效。
     *
     * 方向无效时，tangential_force_n 仍可作为切向力标量使用。
     */
    uint16_t tangential_direction_deg;

    /**
     * @brief 当前触觉单元数据有效标志。
     *
     * 使用 tactile_unit_data_valid_mask_t 的位组合。
     * 数据无效不等于“无接触”，算法层需要单独处理。
     */
    uint16_t valid_mask;

} tactile_unit_data_t;





/**
 * @brief 单个手指上的完整触觉模块数据。
 *
 * 一个完整触觉模块包含两个 5x5 触觉单元：
 * - unit[TACTILE_UNIT_UPPER]：上触觉单元；
 * - unit[TACTILE_UNIT_LOWER]：下触觉单元。
 */
typedef struct
{
    tactile_unit_data_t unit[TACTILE_UNIT_COUNT_PER_MODULE];

} tactile_module_data_t;





/**
 * @brief 二指夹爪的一帧完整触觉数据。
 *
 * 这是触觉状态评估 Task 的动态输入。
 */
typedef struct
{
    /**
     * @brief 左右两指触觉数据。
     *
     * finger[TACTILE_FINGER_LEFT]  ：左指；
     * finger[TACTILE_FINGER_RIGHT] ：右指。
     */
    tactile_module_data_t finger[TACTILE_FINGER_COUNT];

    /**
     * @brief 触觉帧序号
     * 每生成一帧完整快照后递增
     * 用于判断新帧、丢帧和避免重复处理
     */
    uint32_t sequence;

    /**
     * @brief 触觉帧时间戳，单位 ms。
     *
     * 用于计算 dt、力变化率、接触中心速度等。
     */
    uint32_t timestamp_ms;

} gripper_tactile_data_t;

#endif
