/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _USER_TOOL_H
#define _USER_TOOL_H
#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"
#include "ccmram.h"

enum
{
    CHASSIS_DEBUG = 1,
    GIMBAL_DEBUG,
    INS_DEBUG,
    RC_DEBUG,
    IMU_HEAT_DEBUG,
    SHOOT_DEBUG,
    AIMASSIST_DEBUG,
};

extern uint8_t GlobalDebugMode;

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* math relevant */
/* radian coefficient */
#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT(val,min,max) \
    do { \
        while ((val) < (min)) (val) += (max) - (min); \
        while ((val) > (max)) (val) -= (max) - (min); \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

// 绝对值计算
#define ABS(x)      ((x)>0? (x):(-(x)))

typedef struct
{
    float input;        // 输入数据
    float out;          // 输出数据
    float min_value;    // 限幅最小值
    float max_value;    // 限幅最大值
    float frame_period; // 时间间隔
} ramp_function_source_t;

typedef struct PACKED
{
    uint16_t Order;// 最小二乘法的阶数
    uint32_t Count;// 已经存储的样本数

    float *x;// 存储样本时间间隔的数组
    float *y;// 存储样本信号值的数组

    float k;// 表示拟合直线的斜率
    float b;// 拟合直线的截距

    float StandardDeviation;// 拟合直线与样本点之间的标准差，用于评估拟合效果

    float t[4];
} Ordinary_Least_Squares_t;

typedef struct {
    float start, target;
    float speed_start, max_speed, speed_end;
    float max_acc, max_dec;
    float acc_ratio, dec_ratio;
} TrapezoidParams;

// 快速开方
float Sqrt(float x);

// 斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
// 斜波函数计算
float ramp_calc(ramp_function_source_t *ramp_source_type, float input);

// 绝对限制
float abs_limit(float num, float Limit);
// 判断符号位
float sign(float value);
// 浮点死区
float float_deadband(float Value, float minValue, float maxValue);
// int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
// 限幅函数
float float_constrain(float Value, float minValue, float maxValue);
// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue);
// 角度 °限幅 180 ~ -180
float theta_format(float Ang);

int float_rounding(float raw);

// 弧度格式化为-PI~PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);

unsigned char serial_get_crc8_value(unsigned char *data, unsigned char len);


#endif
