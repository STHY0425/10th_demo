/**
 * @file debug.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-10-07
 *
 * @copyright Copyright (c) 2024
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "debug_task.h"
#include "air_joy.h"
#include "arm_math.h"
#include "bsp_bitband.h"
#include "bsp_usart.h"
#include "data_type.h"
#include "topics.h"
#include "vofa.h"
#include "xbox.h"

#ifdef CHASSIS_TO_DEBUG
#include "pid_controller.h"
#endif
// extern Motor_C610 m2006;

osThreadId_t Debug_TaskHandle;

uint8_t debug_buffer[9];

Subscriber *sub_debug;
pub_Control_Data debug_pid;

Subscriber *xbox_data;
pub_Xbox_Data xbox_data_pub;

VOFA_Instance_t *vofa_instance = NULL;
Uart_Instance_t *vofa_uart_instance = NULL;
extern uart_package_t VOFA_uart_package;

#ifdef TEST_VESC

int count = 0;
CAN_Rx_Instance_t VESC_rx_instance1 = {
    .can_handle = &hcan2,
    .RxHeader = {0},
    .rx_len = 8,
    .can_rx_buff = {0},
};

CAN_Tx_Instance_t VESC_tx_instance1 = {
    .can_handle = &hcan2,
    .isExTid = 1,
    .tx_mailbox = 0,
    .tx_len = 8,
    .can_tx_buff = {0},
};

CAN_Rx_Instance_t VESC_rx_instance2 = {
    .can_handle = &hcan2,
    .RxHeader = {0},
    .rx_len = 8,
    .can_rx_buff = {0},
};

CAN_Tx_Instance_t VESC_tx_instance2 = {
    .can_handle = &hcan2,
    .isExTid = 1,
    .tx_mailbox = 0,
    .tx_len = 8,
    .can_tx_buff = {0},
};

CAN_Rx_Instance_t VESC_rx_instance3 = {
    .can_handle = &hcan2,
    .RxHeader = {0},
    .rx_len = 8,
    .can_rx_buff = {0},
};

CAN_Tx_Instance_t VESC_tx_instance3 = {
    .can_handle = &hcan2,
    .isExTid = 1,
    .tx_mailbox = 0,
    .tx_len = 8,
    .can_tx_buff = {0},
};

Motor_Control_Setting_t VESC_motor_ctrl = {0};

VESC vesc[3] = {
    VESC(1, VESC_rx_instance1, VESC_tx_instance1, VESC_motor_ctrl, 0, 1),
    VESC(2, VESC_rx_instance2, VESC_tx_instance2, VESC_motor_ctrl, 0, 1),
    VESC(3, VESC_rx_instance3, VESC_tx_instance3, VESC_motor_ctrl, 0, 1)};

extern Motor_C620 chassis_motor[4];
#endif

#ifdef TEST_DM
CAN_Rx_Instance_t dm_rx_instance = {
    .can_handle = &hcan1,
    .RxHeader = {0},
    .rx_len = 6,
    .can_rx_buff = {0},
};

CAN_Tx_Instance_t dm_tx_instance = {
    .can_handle = &hcan1,
    .isExTid = 0,
    .tx_mailbox = 0,
    .tx_id = 0x08,
    .tx_len = 8,
    .can_tx_buff = {0},
};

Motor_Control_Setting_t DM_motor_ctrl = {0};

DM_motor dm[1] = {
    DM_motor(8, dm_rx_instance, dm_tx_instance, DM_motor_ctrl,_POS_with_SPEED_CONTROL,0, 10)
};

float dm_pos = 0;
float dm_speed = 10;
#endif

#ifdef TEST_SYSTEM_TURNER
extern Motor_C620 chassis_motor[4];
extern Motor_C610 m2006[1];
extern Motor_GM6020 gm6020[1];

#endif

float wheel_v = 0;
float ref = 0;

#ifdef DEBUG_GO1_MOTOR
extern uint8_t have_start;

CAN_Rx_Instance_t go1_rx_instance = {
    .can_handle = &hcan2,
    .RxHeader = {0},
    .rx_len = 8,
    .can_rx_buff = {0},
};

CAN_Tx_Instance_t go1_tx_instance = {
    .can_handle = &hcan2,
    .isExTid = 1,
    .tx_mailbox = 0,
    .tx_len = 8,
    .can_tx_buff = {0},
};

Motor_Control_Setting_t go1_motor_ctrl = {0};
// 较为特殊的go1电机，有些选项不需要配置！
GO_M8010 go1_motor[1] = {
    GO_M8010(0, go1_rx_instance, go1_tx_instance, go1_motor_ctrl, 0, -1, 3)};
#endif

#ifdef TEST_SYSTEM_TURNER
uint16_t sample_rate = 5000;
uint16_t frequency = 1;
uint16_t num_samples = 100;
float32_t sine = 0;
float32_t phase_increment = 2 * PI * frequency / sample_rate;
float32_t phase = 0.0f;
float ref_temp = 0;
float speed_aps = 0;
#endif

#ifdef DEBUG_GO1_MOTOR
float debug_pos = 0.5;
float debug_kp = 0.15;
float debug_kd = 0.02;
float debug_spe = 0;
float go1_cur_pos = 0;
float go1_cur_spe = 0;
#endif

__attribute((noreturn)) void Debug_Task(void *argument) {
  // portTickType currentTime;
  // currentTime = xTaskGetTickCount();

#ifdef TEST_SYSTEM_TURNER
  int count = 0;
#endif

#ifdef VOFA_TO_DEBUG
  /* vofa设备创建 */
  vofa_uart_instance = Uart_Register(&VOFA_uart_package);
  if (vofa_uart_instance == NULL) {
    LOGERROR("vofa uart register failed!");
    vTaskDelete(NULL);
  }
  vofa_instance = VOFA_init(vofa_uart_instance, 10);
  if (vofa_instance == NULL) {
    LOGERROR("vofa init failed!");
    vTaskDelete(NULL);
  }
#endif

#ifdef DEBUG_GO1_MOTOR
  go1_motor[0].GO_Motor_No_Tarque_Ctrl();
  if (!have_start)
    vTaskDelay(5);
  go1_cur_pos = go1_motor[0].real_cur_data.Pos;
  go1_cur_spe = go1_motor[0].real_cur_data.W;
  debug_pos = go1_cur_pos;
  debug_spe = go1_cur_spe;
#endif

  publish_data xbox_;
  xbox_data = register_sub("xbox", 1);
  uint16_t ratio = 11000 / 1024;
  uint16_t ratio1 = 5000 / 1024;

  for (;;) {
    xbox_ = xbox_data->getdata(xbox_data);
    if (xbox_.len != -1) {
      xbox_data_pub = *(pub_Xbox_Data *)xbox_.data;
    }
#ifdef TEST_VESC
    count++;

    if (xbox_data_pub.btnY)
      chassis_motor[1].Motor_Ctrl(-3000);
    else if (xbox_data_pub.btnA)
      chassis_motor[1].Motor_Ctrl(3000);
    else
      chassis_motor[1].Motor_Ctrl(0);
    Motor_SendMsgs(chassis_motor);

    vesc[0].Rpm_Control(ratio * xbox_data_pub.trigLT);
    vesc[1].Rpm_Control(ratio * xbox_data_pub.trigLT);
    vesc[2].Rpm_Control(ratio * xbox_data_pub.trigLT);
    COMMON_Motor_SendMsgs(vesc);

#endif

#ifdef TEST_DM
    if (xbox_data_pub.btnDirLeft)
    {
      dm_pos += 0.05;
      dm[0].POS_with_SPEED_CONTROL(dm_pos,dm_speed);
    } else if (xbox_data_pub.btnDirRight) {
      dm_pos -= 0.05;
      dm[0].POS_with_SPEED_CONTROL(dm_pos,dm_speed);
    } else if (xbox_data_pub.btnB) {
      dm[0].DM_MOTOR_ZERO_POSITION();
    } else if (xbox_data_pub.btnY) {
      dm[0].DM_MOTOR_ENABLE();
    }
    COMMON_Motor_SendMsgs(dm);
#endif

#ifdef VOFA_TO_DEBUG
    vofa_instance->vofa_task(vofa_instance);
    LOGINFO("debug task is running!");
#endif

#ifdef TEST_SYSTEM_TURNER
    float32_t sine_value = arm_sin_f32(phase);
    sine = sine_value * 2000;
    /* 更新相位 */
    phase += phase_increment;
    if (phase >= 2 * PI) {
      phase -= 2 * PI;
    }
#ifdef TEST_SYSTEM_M3508
    count++;
    if (count <= 3000) {
      chassis_motor[3].Motor_Ctrl(0);
      Motor_SendMsgs(chassis_motor);
    } else if (count > 3000 && count <= 6000) {
      chassis_motor[3].Motor_Ctrl(2000);
      Motor_SendMsgs(chassis_motor);
    } else if (count > 6000 && count <= 9000) {
      chassis_motor[3].Motor_Ctrl(0);
      Motor_SendMsgs(chassis_motor);
    } else {
      count = 0;
    }
    speed_aps = chassis_motor[3].speed_aps;

#endif

#ifdef TEST_SYSTEM_M2006
    m2006[0].Motor_Ctrl(sine);
    Motor_SendMsgs(m2006);
#endif

#ifdef TEST_SYSTEM_GM6020
    count++;
    if (count <= 3000) {
      ref = 0;
      gm6020[0].Motor_Ctrl(0);
      Motor_SendMsgs(gm6020);
    } else if (count > 3000 && count <= 6000) {
      ref = 180;
      gm6020[0].Motor_Ctrl(180);
      Motor_SendMsgs(gm6020);
    } else if (count > 6000 && count <= 9000) {
      ref = 0;
      gm6020[0].Motor_Ctrl(0);
      Motor_SendMsgs(gm6020);
    } else {
      count = 0;
    }

#endif

#endif

#ifdef DEBUG_GO1_MOTOR
    if (xbox_data_pub.btnDirUp) {
      debug_pos += 0.005;
      go1_motor[0].GO_Motor_Pos_Ctrl(debug_pos, debug_kp, debug_kd);
    } else if (xbox_data_pub.btnDirDown) {
      debug_pos -= 0.005;
      go1_motor[0].GO_Motor_Pos_Ctrl(debug_pos, debug_kp, debug_kd);
    } else if (xbox_data_pub.btnB) {
      go1_motor->stop_the_motor();
    }

    // if (debug <= 5000 || debug >= 10000) {
    // go1_motor[0].GO_Motor_Speed_Ctrl(5, 0.05);
    //   go1_motor[0].GO_Motor_Pos_Ctrl(debug_pos, debug_kp, debug_kd);
    //   // go1_motor[0].GO_Motor_No_Tarque_Ctrl();
    //   debug++;
    // } else {
    //   go1_motor[0].GO_Motor_No_Tarque_Ctrl();
    //   debug++;
    // }
#endif
    // vTaskDelayUntil(&currentTime, 1);
    vTaskDelay(5);
  }
}
