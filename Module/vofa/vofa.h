/**
 * @file vofa.h
 * @author Keten (2863861004@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-10-14
 * 
 * @copyright Copyright (c) 2024
 * 
 * @attention :
 * @note :
 * @versioninfo :
 */
#ifndef VOFA_H 
#define VOFA_H 

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
/* freertos接口，提供堆管理以及队列处理 */
#include "rtos_interface.h"

/* bsp层接口 */
#include "bsp_usart.h"
#include "bsp_log.h"

/* module层接口 */
#include "topics.h"
#include "data_type.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/
typedef struct
{
    Uart_Instance_t *vofa_uart_instance;
    rtos_for_module_t *rtos_for_vofa;
    Publisher *vofa_pub;
    pub_vofa_pid pub_data;
    uint8_t (*vofa_task)(void* vofa_instance);
    uint8_t (*get_data)(uint8_t*,pub_vofa_pid*);
    uint8_t (*vofa_deinit)(void* vofa_instance);
}VOFA_Instance_t;


/*----------------------------------function----------------------------------*/

/**
 * @brief 
 * 
 * @param vofa_uart_instance 
 * @param queue_length 
 * @return VOFA_Instance_t* 
 */
VOFA_Instance_t* VOFA_init(Uart_Instance_t *vofa_uart_instance,uint32_t queue_length);


/**
 * @brief 
 * 
 * @param vofa_instance 
 * @return uint8_t 
 */
uint8_t VOFA_Task(void* vofa_instance);


/**
 * @brief 
 * 
 * @param data 
 * @param vofa_pid 
 * @return uint8_t 
 */
uint8_t VOFA_Get_Data(uint8_t *data,pub_vofa_pid *vofa_pid);


/**
 * @brief 
 * 
 * @param uart_instance 
 * @param data_len 
 * @return uint8_t 
 */
uint8_t VOFA_Uart_Rx_Callback(void *uart_instance,uint16_t data_len);

/**
 * @brief 
 * 
 * @param uart_instance 
 * @param float_data 
 * @return uint8_t 
 */
uint8_t VOFA_SendFloat(VOFA_Instance_t *vofa_instance,float data);
uint8_t VOFA_SendEND(VOFA_Instance_t *vofa_instance);
/**
 * @brief 
 * 
 * @param vofa_instance 
 * @return uint8_t 
 */
uint8_t VOFA_DeInit(void *vofa_instance);

#ifdef __cplusplus
}
#endif

#endif	/* VOFA_H */
