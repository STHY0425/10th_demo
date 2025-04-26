#ifndef _ATTITUDE_COM_H_
#define _ATTITUDE_COM_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "vofa_debug.h"

#define ATTITUDE_COM_SWITCH 1
#define VOFA_PACKAGE_LENGTH 20
#define MAX_WAITING_MESSAGES 10

extern osThreadId_t AttitudeCommunication_TaskHandle;
extern uint8_t vofaRX_buffer[VOFA_PACKAGE_LENGTH];
extern Uart_Instance_t *AttCom_uart_instance;
extern Vofa_Instance_t *AttCom_instance;
extern float xPos_Teammate;
extern float yPos_Teammate;
extern float yaw_Teammate ;

__attribute__((noreturn))  void Attitude_Communication_Task(void *argument);

#ifdef __cplusplus
}
#endif
#endif
