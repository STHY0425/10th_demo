/**
 * @file vofa_debug.c
 * @author Reelog
 * @version 0.1
 * @date 2025-04-10
 */
#ifndef _VOFA_DEBUG_H_
#define _VOFA_DEBUG_H_
#ifdef __cplusplus
extern "C"{
#endif
/* -------------------- 头文件 -------------------- */
#include "stm32f4xx_hal.h"
#include "bsp_usart.h"
#include "rtos_interface.h"
#include "data_type.h"
#include "topics.h"
#include "bsp_log.h"
/* -------------------- 宏定义 -------------------- */
#define VOFA_RX_HEAD   0xAE    // 帧头
#define VOFA_RX_TAIL   0xAD    // 帧尾
#define NUM_PARAM_MAX  16     // 可调参数的最大数量
/* ------------------ 自定义类型 ------------------ */
/* 可调参数的结构体 */
typedef struct
{
    float *addr;        // 可调参数的地址
    float value;        // 可调参数的值
} Vofa_Param_Select_t;
/* VOFA 实例 */
typedef struct
{
    Uart_Instance_t *uart_instance; // 串口实例
    rtos_for_module_t *rtos_for_vofa;   // rtos实例
    Vofa_Param_Select_t vofa_param_selected[NUM_PARAM_MAX]; // 可调参数

    uint8_t (*vofa_get_data)(uint8_t *data, Vofa_Param_Select_t *vofa_param_selected);
    uint8_t (*vofa_task)(void *vofa_instance);
    uint8_t (*vofa_send_data)(void *instance, float *data, uint8_t len);
} Vofa_Instance_t;

/* 用户接口函数 */
Vofa_Instance_t* Vofa_Init(Uart_Instance_t *vofa_uart, uint32_t queue_length);
uint8_t Vofa_GetData(uint8_t *data, Vofa_Param_Select_t *vofa_param_selected);
uint8_t Vofa_Task(void *vofa_instance);
uint8_t Vofa_Send_Justfloat(void *instance, float *data, uint8_t len);
uint8_t Vofa_RxCallback_Fun(void *vofa_uart_instance, uint16_t data_len);
uint8_t Vofa_SendData(void *instance, uint8_t who, float x, float y);

/* 底层函数 */
static uint8_t Vofa_Rtos_Init(Vofa_Instance_t* vofa_instance, uint32_t queue_length);
static void float_to_hex(float data, uint8_t *hex);

#ifdef __cplusplus
}
#endif
#endif
