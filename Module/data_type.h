/**
 * @file data_type.h
 * @author Keten (2863861004@qq.com)
 * @brief
 * 模块所依赖的数据类型结构体，有些模块会依赖这些数据类型结构体进行数据传输，所以移植module层都
 *        必须携带这个包
 * @version 0.1
 * @date 2024-10-03
 *
 * @copyright Copyright (c) 2024
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
#include "can.h"
#include "usart.h"
#include <stdbool.h>

#pragma pack(1)

/**
 * @brief 串口数据传输结构体封装
 *
 */
typedef struct {
  UART_HandleTypeDef *huart; // 串口句柄
  uint16_t len;              // 数据长度
  void *data_addr; // 数据地址，使用时把地址赋值给这个指针，数值强转为uint8_t
} UART_TxMsg;

typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float ref;
  float i_limit;
} pub_vofa_pid;

typedef struct {
  float data1;
  float data2;
  float data3;
  float data4;
  float data5;
  float data6;
} pub_ros_package;
typedef struct {
  bool btnY;
  bool btnA;
  bool btnLB;
  uint16_t trigLT;
  bool btnDirUp;
  bool btnDirDown;
  bool btnDirLeft;
  bool btnDirRight;
  bool btnB;
  uint16_t joyLHori;
  uint16_t joyLVert;
  uint16_t joyRHori;
  uint16_t joyRVert;
  // 这里填写你需要传输的Xbox按键摇杆等数据
  // bool btnY;
  // bool btnY_last;
  //......

} pub_Xbox_Data;

#pragma pack()