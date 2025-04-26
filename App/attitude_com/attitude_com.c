#include "attitude_com.h"

/* 任务相关 */
osThreadId_t AttitudeCommunication_TaskHandle;

/* 设备相关 */
uint8_t vofaRX_buffer[VOFA_PACKAGE_LENGTH];
Uart_Instance_t *AttCom_uart_instance;
Vofa_Instance_t *AttCom_instance;

/* 得到的坐标 */
float xPos_Teammate = 0.0f;
float yPos_Teammate = 0.0f;
float yaw_Teammate = 0.0f;

__attribute__((noreturn))  void Attitude_Communication_Task(void *argument)
{
    /* 任务创建成功 */
    LOGINFO("Attitude Communication Task Create Success!");
    /* 任务延时 */
    vTaskDelay(1000);
    /* 任务开始时间 */
    portTickType startTime = xTaskGetTickCount();
    /* 任务运行时间 */
    portTickType runTime = 0;
    /* 任务运行时间间隔 */
    portTickType runTimeInterval = 0;

    portTickType currentTime;
    currentTime = xTaskGetTickCount();
    // uart_package_t VOFA_uart_package = {
    //     .uart_handle = &huart1,
    //     .rx_buffer = vofa_rx_buffer,
    //     .rx_buffer_size = 9,
    //     .uart_callback = VOFA_Uart_Rx_Callback,
    // };
    /* 串口实例注册 */
    uart_package_t vofa_package2 = 
    {
        .uart_handle = &huart1, 
        .rx_buffer = vofaRX_buffer,
        .rx_buffer_size = VOFA_PACKAGE_LENGTH,
        .uart_callback = Vofa_RxCallback_Fun,
    };
    AttCom_uart_instance = Uart_Register(&vofa_package2);
    if(AttCom_uart_instance == NULL)
    {
        /* 如果vofa设备创建失败，就删除本task，防止占cpu */
        LOGWARNING("uart register failed!");
        vTaskDelete(NULL);
    }

    /* vofa设备注册 */
    AttCom_instance = Vofa_Init(AttCom_uart_instance, MAX_WAITING_MESSAGES);
    if(AttCom_instance == NULL)
    {
        /* 如果action设备创建失败，就删除本task，防止占cpu */
        LOGWARNING("vofa device init failed!");
        vTaskDelete(NULL);
    }

    /* 输入参数地址 */
    AttCom_instance->vofa_param_selected[0].addr = &xPos_Teammate;
    AttCom_instance->vofa_param_selected[1].addr = &yPos_Teammate;
    AttCom_instance->vofa_param_selected[2].addr = &yaw_Teammate;

    for(;;)
    {
        /* 主循环用于不断处理接收到的数据 */
        LOGINFO("Attitude Communicate Success!");
        AttCom_instance->vofa_task(AttCom_instance);
    }
}

