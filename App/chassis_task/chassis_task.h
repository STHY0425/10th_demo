/**
 * @file chassis_task.h
 * @author Keten (2863861004@qq.com)
 * @brief ����������
 *        ʵ������ע������õĵ��������Ϊÿһ��������ؿ��Ź�����߳�
 *                 �ḻ�������ͣ�Ŀǰ�������ȫ����ϵ�е��̡����ֵ���
 *                 ����������Ҫ��Ͽ������̣߳�ֻ�п������߳�׼�����ˣ������̲߳Żᷢ�Ϳ�����Ϣ�����
 *                 
 * @version 0.1
 * @date 2024-10-04
 * 
 * @copyright Copyright (c) 2024
 * 
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/

#include <stdint.h>
/* freertos�ӿ� */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* bsp��ӿ� */
#include "bsp_can.h"
/* module��ӿ� */
#include "soft_iwdg.h"
#include "topics.h"
#include "ccmram.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

/**
 * @brief 
 * 
 * @return uint8_t 
 */
uint8_t Chassis_Init();


void Chassis_Task(void *argument);


/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif
  
#ifdef __cplusplus
#include "rm_motor.h"
#include "Omni/Omni_Chassis.h"


/**
 * @brief Chassis�����˶�����
 *        ��ɹ��ܣ�
 *          1.������̬����
 *          2.����ģʽswitch-case
 *          3.�ٶ��������õ��������ӵ��ٶ�
 *          4.�������ٶȷ��͵����Ե�� 
 * @param user_chassis 
 * @return uint8_t 
 */
uint8_t Chassis();
extern float g_trace[8]; 
extern Motor_Control_Setting_t g_trace_pid[3];                       



#endif




