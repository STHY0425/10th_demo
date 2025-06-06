/**
 * @file robot_def.h
 * @author Keten (2863861004@qq.com)
 * @brief 机器人的参数配置文件，类似于 yaml文件
 * @version 0.1
 * @date 2024-09-14
 * 
 * @copyright Copyright (c) 2024
 * 
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once 

/* 是否启用多板通讯 */
#define ONE_BOARD 1                     // 单板控制整车

/* 底盘类型选择 */
#define USE_FOUR_SWERVE_CHASSIS 0           // 使用四舵轮底盘
#define USE_THREE_SWERVE_CHASSIS 0          // 使用三舵轮底盘
#define USE_FOUR_OMNI_CHASSIS 0             // 使用四全向轮
#define USE_THREE_OMNI_CHASSIS 1            // 使用三全向轮


/* 机器人重要参数定义 */

/* imu接口 */
#define USE_ACTION_FOR_IMU  1               // 使用action作为imu接口

#ifdef USE_OMNI_CHASSIS
/* 这里以八期r1为示例底盘 */
#define WHEEL_R                     0.076f  // 轮子半径（单位：m）
#define CHASSIS_R                   0.23f   // 底盘半径（单位：m）
#define MOTOR_REDUCTION_RATIO       19.0f                    // 底盘驱动轮系电机的减速比

#endif


#ifdef USE_SWERVE_CHASSIS
/* 九期扣篮车参数 */
#define WHEEL_R                     0.076f  // 轮子半径 （单位：m）
#define CHASSIS_R                   0.50f   // 底盘半径 （单位：m）
#define MOTOR_REDUCTION_RATIO       21.0f   // 轮向电机减速比


#endif




