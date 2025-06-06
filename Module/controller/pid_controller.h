/**
  ******************************************************************************
  * @file	 controller.h
  * @author  Wang Hongxi
  * @author  Zhang Hongyu (fuzzy pid)
  * @version V1.1.3
  * @date    2021/7/3
  * @brief   
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */

#ifndef PID_CONTROLLER_H 
#define PID_CONTROLLER_H 

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "user_tool.h"
#include "arm_math.h"
#include "ccmram.h"
#include <math.h>


#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/******************************** FUZZY PID **********************************/
#define NB -3
#define NM -2
#define NS -1
#define ZE 0
#define PS 1
#define PM 2
#define PB 3
// 在这里添加注释

typedef struct PACKED
{
    float KpFuzzy;
    float KiFuzzy;
    float KdFuzzy;

    float (*FuzzyRuleKp)[7];
    float (*FuzzyRuleKi)[7];
    float (*FuzzyRuleKd)[7];

    float KpRatio;
    float KiRatio;
    float KdRatio;

    float eStep;
    float ecStep;

    float e;
    float ec;
    float eLast;

    uint32_t DWT_CNT;
    float dt;
} FuzzyRule_t;

void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule, float (*fuzzyRuleKp)[7], float (*fuzzyRuleKi)[7], float (*fuzzyRuleKd)[7],
                     float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep);
void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref);

/******************************* PID CONTROL *********************************/
typedef enum pid_Improvement_e
{
    NONE = 0X00,                        //0000 0000 不使用任何优化环节
    Integral_Limit = 0x01,              //0000 0001 使用积分限幅
    Derivative_On_Measurement = 0x02,   //0000 0010 微分先行
    Trapezoid_Intergral = 0x04,         //0000 0100 使用梯形积分
    Proportional_On_Measurement = 0x08, //0000 1000 比例先行
    OutputFilter = 0x10,                //0001 0000 输出滤波（LR）
    ChangingIntegrationRate = 0x20,     //0010 0000 变速积分
    DerivativeFilter = 0x40,            //0100 0000 微分滤波（LR）
    ErrorHandle = 0x80,                 //1000 0000 错误处理（电机堵转）
    IMCREATEMENT_OF_OUT = 0x100,        //0001 0000 0000 启用增量式输出
    Feedforward_CONTROLL = 0x200,       //0010 0000 0000 前馈控制
} PID_Improvement_e;


/**
 * @brief pid计算错误情况
 *  Motor_Blocked 电机堵转
 */
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;


/**
 * @brief 
 * 
 */
typedef struct PACKED
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;


/**
 * @brief 
 * 
 */
typedef struct PACKED PID_T 
{
    float Ref;
    float Kp;
    float Ki;
    float Kd;
    float FFJ;
    float FFB;

    float Measure;
    float Last_Measure;// 上次的测量值
    float Eriler_Measure;// 上上次的测量值
    float Err;
    float Last_Err;// 上一次的error
    float Eriler_Err;// 上上次的error
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;
    float FFout;

    float Output;
    float Last_Output;
    float Last_Dout;// 上一次的输出

    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    float ControlPeriod;
    float CoefA;         //For Changing Integral
    float CoefB;         //ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;

    uint16_t OLS_Order;
    Ordinary_Least_Squares_t OLS;

    uint32_t DWT_CNT;
    float dt;

    FuzzyRule_t *FuzzyRule;

    uint16_t Improve;// 配置pid优化环节，使用 | 运算符连接，详细见枚举 PID_Improvement_e

    PID_ErrorHandler_t ERRORHandler;

    void (*User_Func1_f)(struct PID_T *pid);
    void (*User_Func2_f)(struct PID_T *pid);
} PID_t;

void PID_Init(PID_t *pid);


float PID_Calculate(PID_t *pid, float measure, float ref);

void PID_Reset(PID_t *pid);


/*************************** FEEDFORWARD CONTROL *****************************/

/**
 * @brief 前馈控制器
 * 
 */
typedef struct PACKED
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

    float Ref;
    float Last_Ref;

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC; // RC = 1/omegac 低通滤波器的时间常数

    float Ref_dot;// 参考信号的一阶导数
    float Ref_ddot;// 参考信号的二阶导数
    float Last_Ref_dot;// 上一次计算的一阶导数

    uint16_t Ref_dot_OLS_Order;
    Ordinary_Least_Squares_t Ref_dot_OLS;
    uint16_t Ref_ddot_OLS_Order;
    Ordinary_Least_Squares_t Ref_ddot_OLS;

    float Output;
    float MaxOut;

} Feedforward_t;

void Feedforward_Init(
    Feedforward_t *ffc,
    float max_out,
    float *c,
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order);

float Feedforward_Calculate(Feedforward_t *ffc, float ref);

/************************* LINEAR DISTURBANCE OBSERVER *************************/

/**
 * @brief 线性扰动观测器
 * 
 */
typedef struct PACKED
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

    float Measure;
    float Last_Measure;

    float u; // system input

    float DeadBand;

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC; // RC = 1/omegac

    float Measure_dot;
    float Measure_ddot;
    float Last_Measure_dot;

    uint16_t Measure_dot_OLS_Order;
    Ordinary_Least_Squares_t Measure_dot_OLS;
    uint16_t Measure_ddot_OLS_Order;
    Ordinary_Least_Squares_t Measure_ddot_OLS;

    float Disturbance;
    float Output;
    float Last_Disturbance;
    float Max_Disturbance;
} LDOB_t;

void LDOB_Init(
    LDOB_t *ldob,
    float max_d,
    float deadband,
    float *c,
    float lpf_rc,
    uint16_t measure_dot_ols_order,
    uint16_t measure_ddot_ols_order);

float LDOB_Calculate(LDOB_t *ldob, float measure, float u);

/*************************** Tracking Differentiator ***************************/

/**
 * @brief 跟踪微分器
 * 
 */
typedef struct PACKED
{
    float Input;

    float h0;
    float r;

    float x;
    float dx;
    float ddx;

    float last_dx;
    float last_ddx;

    uint32_t DWT_CNT;
    float dt;
} TD_t;

void TD_Init(TD_t *td, float r, float h0);
float TD_Calculate(TD_t *td, float input);


#ifdef __cplusplus
}
#endif

#endif

