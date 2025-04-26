/**
 * @file vofa_debug.c
 * @author Reelog
 * @version 0.1
 * @date 2025-04-10
 */
#include "vofa_debug.h"
#include "attitude_com.h"
extern Vofa_Instance_t * Vofa_instance;
/* vofa 的 justfloat 通信格式的包尾 */
const uint8_t justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7F};

float xPos_teammate;
float yPos_teammate;

/**
 * @brief 初始化 vofa 调试模块
 * @param vofa_uart 串口实例
 * @param queue_length 消息队列长度
 */
Vofa_Instance_t* Vofa_Init(Uart_Instance_t *vofa_uart, uint32_t queue_length)
{
    if(vofa_uart == NULL)
    {
        LOGERROR("vofa init failed!");
        return NULL;
    }
    /* 创建临时实例 */
    Vofa_Instance_t *temp_Vofa_instance = (Vofa_Instance_t*)pvPortMalloc(sizeof(Vofa_Instance_t));
    if(temp_Vofa_instance  == NULL)
    {
        LOGERROR("vofa instance malloc failed!");
        return NULL;
    }
    memset(temp_Vofa_instance,0,sizeof(Vofa_Instance_t));
    temp_Vofa_instance->uart_instance = vofa_uart;
    /* 挂载 rtos_interface */
    if(Vofa_Rtos_Init(temp_Vofa_instance, queue_length) != 0)
    {
        LOGERROR("rtos for uart init wrong!");
        vPortFree(temp_Vofa_instance);
        temp_Vofa_instance = NULL;
        return NULL;
    }
   
    /* 关闭DMA半中断 */
    // __HAL_DMA_DISABLE_IT(vofa_uart->uart_package.uart_handle->hdmarx, DMA_IT_HT);

    

    /* 清空可调变量的结构体数组 */
    memset(temp_Vofa_instance->vofa_param_selected, 0, sizeof(Vofa_Param_Select_t) * NUM_PARAM_MAX);
    
    
    temp_Vofa_instance->uart_instance->uart_package.uart_callback = Vofa_RxCallback_Fun;

    /* 挂载函数指针 */
    temp_Vofa_instance->vofa_get_data = Vofa_GetData;
    temp_Vofa_instance->vofa_task = Vofa_Task;
    // temp_Vofa_instance->vofa_send_data = Vofa_SendData;
    temp_Vofa_instance->uart_instance->device = temp_Vofa_instance;

    return temp_Vofa_instance;
}

/**
 * @brief 挂载 rtos_interface 到 vofa 上
 */
static uint8_t Vofa_Rtos_Init(Vofa_Instance_t* Vofa_instance, uint32_t queue_length)
{
    /* 分配 rtos_interface 实例的内存, 并清空内置参数 */
    rtos_for_module_t *rtos_interface = (rtos_for_module_t *)pvPortMalloc(sizeof(rtos_for_module_t));
    assert_param(rtos_interface != NULL);
    if(rtos_interface == NULL)
    {
        LOGERROR("vofa rtos_interface pvPortMalloc failed!");
        return 1;
    }
    memset(rtos_interface,0,sizeof(rtos_for_module_t));

    /* 注册获取一个 Queue 句柄, 并赋给 rtos_interface 实例 */
    rtos_interface->queue_receive = xQueueReceive;
    rtos_interface->queue_send = queue_send_wrapper;
    QueueHandle_t queue = xQueueCreate(queue_length,sizeof(UART_TxMsg));
    if(queue == NULL)
    {
        LOGERROR("vofa queue for uart create failed!");
        vPortFree(rtos_interface);
        rtos_interface = NULL;
        return 1;
    }
    else
    {
        rtos_interface->xQueue = queue;
    }

    /* 对 vofa 实例挂载 rtos_interface */
    
    Vofa_instance->rtos_for_vofa = rtos_interface;
    LOGINFO("vofa uart rtos init success!");

    return 0;
}

/**
 * @brief 处理接收到的数据
 * @param data 接收到的数据
 * @param vofa_param_selected 存储可调数据的地址和值的结构体数组
 * @retval 0 成功
 *         1 帧头帧尾错误
 *         2 所选参数地址为空
 */
uint8_t Vofa_GetData(uint8_t *data, Vofa_Param_Select_t *vofa_param_selected)
{
    uint8_t i;
    union
    {
        uint8_t array[4];
        float value;
    } rxdata;

    /* 格式检验 */
    if(data[0] != 0xFC || data[1] != 0xFB || data[12] != 0xFD || data[13] != 0xFE)
    {
        return 1;
    }
    else
    {
        for(i = 0; i < 4; i++)
        {
            rxdata.array[i] = data[i + 4];
            xPos_teammate = rxdata.value;
        }
        for(i = 0; i < 4; i++)
        {
            rxdata.array[i] = data[i + 8];
            yPos_teammate = rxdata.value;
        }
        return 0;
    }
}

/**
 * @brief 串口空闲中断回调函数
 * @param vofa_uart_instance vofa串口实例
 * @param data_len 接收的数据长度
 * @retval 0 接收成功
 *         1 队列不存在/串口实例为空
 */
uint8_t Vofa_RxCallback_Fun(void *vofa_uart_instance, uint16_t data_len)
{
    UART_TxMsg Msg;

    Uart_Instance_t *temp_uart_instance = (Uart_Instance_t*)vofa_uart_instance;
    Vofa_Instance_t *Vofa_instance = (Vofa_Instance_t *)temp_uart_instance->device;

    if(Vofa_instance->rtos_for_vofa->xQueue !=NULL &&Vofa_instance->uart_instance != NULL)
    {
        Msg.data_addr = Vofa_instance->uart_instance->uart_package.rx_buffer;
        Msg.len = data_len;
        Msg.huart = Vofa_instance->uart_instance->uart_package.uart_handle;
        if(Msg.data_addr != NULL)
        {
            Vofa_instance->rtos_for_vofa->queue_send(Vofa_instance->rtos_for_vofa->xQueue,&Msg,NULL);
            return 0;
        }
    }
    return 1;
}

/**
 * @brief Vofa任务
 * @param Vofa_instance vofa实例
 */
uint8_t Vofa_Task(void *Vofa_instance)
{
    UART_TxMsg Msg;
    Vofa_Instance_t *Vofa_instance_temp = (Vofa_Instance_t *)Vofa_instance;

    /* 等待数据接收 */
    if (Vofa_instance_temp->rtos_for_vofa->queue_receive(Vofa_instance_temp->rtos_for_vofa->xQueue, &Msg, portMAX_DELAY) == pdTRUE)
    {
        /* 处理数据 */
        uint8_t result = Vofa_instance_temp->vofa_get_data(Msg.data_addr, Vofa_instance_temp->vofa_param_selected);
        if (result != 0)
        {
            LOGERROR("Data processing failed!");
            return 1;
        }
    }
    return 0;
}

/**
 * @brief 发送浮点数据
 * @param data 浮点数据
 * @retval 0 成功
 *         1 发送失败
 */
uint8_t Vofa_Send_Justfloat(void *instance, float *data, uint8_t len)
{
    Vofa_Instance_t *Vofa_instance_temp = (Vofa_Instance_t *)instance;
    uint8_t hex[4];
    uint8_t index;

    /* 发送float数据 */
    for(index = 0; index < len; index ++)
    {
        float_to_hex(data[index], hex);
        if(HAL_UART_Transmit_DMA(Vofa_instance_temp->uart_instance->uart_package.uart_handle, hex, 4) != HAL_OK)
        {
            return 1;
        }
    }

    /* 发送尾包 */
    HAL_UART_Transmit_DMA(Vofa_instance_temp->uart_instance->uart_package.uart_handle, justfloat_tail, 4);

    return 0;
}

uint8_t Vofa_SendData(void *instance, uint8_t who, float x, float y)
{
    uint8_t i;
    uint8_t hex[4];
    uint8_t Tx[7];

    Vofa_Instance_t *Vofa_instance_temp = (Vofa_Instance_t *)instance;
    
    Tx[0] = 0xFC;
    Tx[1] = 0xFB;
    Tx[2] = 0x66;
    Tx[3] = 8;

    float_to_hex(x, hex);
    for(i = 0; i < 4; i ++)
    {
        Tx[4 + i] = hex[i];
    }

    float_to_hex(y, hex);
    for(i = 0; i < 4; i ++)
    {
        Tx[4 + 4 + i] = hex[i];
    }

    Tx[12] = 0xFD;
    Tx[13] = 0xFE;

    HAL_UART_Transmit_DMA(Vofa_instance_temp->uart_instance->uart_package.uart_handle, Tx, 14);

    return 0;
}

/**
 * @brief 将浮点数转换为字节数组
 * @param data 浮点数
 * @param hex 字节数组
 */
static void float_to_hex(float data, uint8_t *hex)
{
    union   // 使用联合体实现浮点数到字节数组的转换
    {
        float f;
        uint8_t bytes[4];
    } converter;
    converter.f = data;
    hex[3] = converter.bytes[3]; 
    hex[2] = converter.bytes[2];
    hex[1] = converter.bytes[1];
    hex[0] = converter.bytes[0]; // 最高有效字节
}
