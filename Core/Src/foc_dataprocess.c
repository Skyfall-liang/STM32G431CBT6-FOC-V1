#include "foc_dataprocess.h"

uint8_t canbuf[20] = {0};
float motor_target = 0;
uint8_t data_feedback_flag = 0;
uint8_t controlloop_mode_flag = 3;
uint8_t send_data_flag = 0;

uint8_t motor_id = 0x02;
uint8_t motor_init_flag = 0;
uint16_t motor_cmd = 0;
int16_t temp1 = 0;

float zero_angle = 0;

/**
 * @brief       FIFO0回调函数
 * @param       hfdcan:FDCAN句柄
 * @param       RxFifo0ITs:接收FIFO
 * @retval      无;
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    //uint8_t i = 0;

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)    /* FIFO1新数据中断 */ 
    {
        /* 提取FIFO0中接收到的数据 */
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &g_fdcanx_rxheade, canbuf);

        if(g_fdcanx_rxheade.IdType != FDCAN_STANDARD_ID)
        {
            Error_Handler();
        }

        foc_data_process();
        
        
        // printf("id:%#x\r\n", g_fdcanx_rxheade.Identifier);
        // printf("len:%#x\r\n", g_fdcanx_rxheade.DataLength);
        // for (i = 0; i < 20; i++)
        // {
        //     printf("rxdata[%d]:%#x\r\n", i, canbuf[i]);
        // }
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}


uint32_t FDCAN_GetTxFifoFillLevel(const FDCAN_HandleTypeDef *hfdcan)
{
  uint32_t FreeLevel;

  FreeLevel = hfdcan->Instance->TXFQS & FDCAN_TXFQS_TFFL;

  /* Return Tx FIFO free level */
  return FreeLevel;
}


/**
 * @brief       FDCAN 发送一组数据
 * @note        发送格式固定为: 标准ID, 数据帧
 * @param       len     :数据长度,取值范围：FDCAN_DLC_BYTES_0 ~ FDCAN_DLC_BYTES_64
 * @param       msg     :数据指针,最大为64个字节
 * @retval      发送状态 0, 成功; 1, 失败;
 */
uint8_t fdcan_send_msg(uint8_t *msg, uint32_t len)
{
    g_fdcanx_txheade.Identifier = 0x10 + motor_id;                              /* 32位ID */
    g_fdcanx_txheade.IdType = FDCAN_STANDARD_ID;                     /* 标准ID */
    g_fdcanx_txheade.TxFrameType = FDCAN_DATA_FRAME;                 /* 使用标准帧 */
    g_fdcanx_txheade.DataLength = len;                               /* 数据长度 */
    g_fdcanx_txheade.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    g_fdcanx_txheade.BitRateSwitch = FDCAN_BRS_ON;                   /* 开启速率切换 */
    g_fdcanx_txheade.FDFormat = FDCAN_FD_CAN;                        /* FDCAN模式发送 */
    g_fdcanx_txheade.TxEventFifoControl = FDCAN_NO_TX_EVENTS;        /* 无发送事件 */
    g_fdcanx_txheade.MessageMarker = 0;

    while(FDCAN_GetTxFifoFillLevel(&hfdcan1) >= (4U));
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &g_fdcanx_txheade, msg) != HAL_OK) /* 发送消息 */
    {
        FDCAN_ErrorCountersTypeDef err_counters;
        HAL_FDCAN_GetErrorCounters(&hfdcan1, &err_counters);
        return 1;
    }
    while(FDCAN_GetTxFifoFillLevel(&hfdcan1) >= (4U));

    return 0;
}


void data_feedback(void)
{
    uint8_t data_buf[12] = {0};
    memcpy(data_buf, &motor_id, 2);
    memcpy(&data_buf[2], &(int32_t){(angle_now - zero_angle) * 1000}, 4);
    memcpy(&data_buf[6], &(int32_t){velocity_now * 100}, 4);
    memcpy(&data_buf[10], &(int16_t){current_now * 100}, 2);

    fdcan_send_msg(data_buf, FDCAN_DLC_BYTES_12);
}


void foc_data_process(void)
{
    //命令类型判断
    memcpy(&motor_cmd, &canbuf[0], 2);
    switch (motor_cmd)
    {
        //角度环
    case 0x00:
        controlloop_mode_flag = 0;
        break;

        //速度环
    case 0x01:
        controlloop_mode_flag = 1;
        break;

        //电流环
    case 0x02:
        controlloop_mode_flag = 2;
        break;

        //速度电流环
    case 0x03:
        controlloop_mode_flag = 3;
        memcpy(&temp1, &canbuf[6], 2);
        pid_VC_velocity.output_limit = (float)temp1 / 100.0f;
        //temp1 = 0;
        break;

        //三环合一
    case 0x04:
        controlloop_mode_flag = 4;
        memcpy(&temp1, &canbuf[6], 4);
        pid_AVC_angle.output_limit = (float)temp1 / 100.0f;
        memcpy(&temp1, &canbuf[10], 2);
        pid_AVC_velocity.output_limit = (float)temp1 / 100.0f;
        break;

        //数据反馈
    case 0x05:
        data_feedback_flag = 1;
        break;
    
        //电机初始化是否完成
    case 0x06:
        if(motor_init_flag == 1)
        {
            data_feedback_flag = 2;
        }
        break;
        
    case 0x07:
        control_data_updata();
        zero_angle = angle_now;
        data_feedback_flag = 3;
        break;

    default:
        break;
    }

    if(data_feedback_flag == 0)
    {
        int32_t rec_target = 0;
        memcpy(&rec_target, &canbuf[2], 4);
        motor_target = (float)rec_target / 1000.0f; 
        if((motor_cmd == 0x00)||(motor_cmd == 0x04))
        {
            motor_target = motor_target + zero_angle;
        }
    }
    else if(data_feedback_flag == 1)
    {
        //数据发送
        data_feedback();
        data_feedback_flag = 0;
    }
    else if(data_feedback_flag == 2)
    {
        uint8_t temp_data_buf[3] = {0};
        memcpy(temp_data_buf, &motor_id, 1);
        temp_data_buf[1] = 0x06;
        temp_data_buf[2] = motor_init_flag;
        fdcan_send_msg(temp_data_buf, FDCAN_DLC_BYTES_3);
        data_feedback_flag = 0;
    }
    
    data_feedback_flag = 0;
    
}


void Taskloop_select(void)
{
    switch (controlloop_mode_flag)
    {
    case 0:
        foc_set_angle_control(motor_target);
        break;

    case 1:
        foc_set_velocity_control(motor_target);
        break;

    case 2:
        foc_set_current_control(motor_target);
        break;

    case 3:
        foc_set_velocity_current_control(motor_target);
        break;

    case 4:
        foc_set_angle_velocity_current_control(motor_target);
        break;

    default:
        break;
    }

}

