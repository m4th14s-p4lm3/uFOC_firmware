#include "communication.h"

#include <string.h>

static CAN_HandleTypeDef *s_hcan = NULL;

static can_message_t s_last_rx_msg;
static volatile uint8_t s_msg_available = 0;

static HAL_StatusTypeDef communication_filter_init(void)
{
    CAN_FilterTypeDef filter;

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    // přijmi vše
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;

    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;

    #if defined(CAN_FILTER_FIFO1)
    filter.SlaveStartFilterBank = 14;
    #endif

    return HAL_CAN_ConfigFilter(s_hcan, &filter);
}

void communication_init(CAN_HandleTypeDef *hcan)
{
    s_hcan = hcan;
    memset(&s_last_rx_msg, 0, sizeof(s_last_rx_msg));
    s_msg_available = 0;
}

HAL_StatusTypeDef communication_start(void)
{
    if (s_hcan == NULL)
    {
        return HAL_ERROR;
    }

    if (communication_filter_init() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_Start(s_hcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_CAN_ActivateNotification(s_hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef communication_send(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    if (s_hcan == NULL || data == NULL || len > 8)
    {
        return HAL_ERROR;
    }

    tx_header.StdId = std_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(s_hcan, &tx_header, (uint8_t *)data, &tx_mailbox);
}

HAL_StatusTypeDef communication_send_test(void)
{
    uint8_t data[2] = {1, 2};
    return communication_send(0x123, data, 2);
}

bool communication_message_available(void)
{
    return (s_msg_available != 0);
}

bool communication_read(can_message_t *msg)
{
    if (msg == NULL || s_msg_available == 0)
    {
        return false;
    }

    __disable_irq();
    memcpy(msg, &s_last_rx_msg, sizeof(can_message_t));
    s_msg_available = 0;
    __enable_irq();

    return true;
}

void communication_rx_fifo0_pending_callback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }

    memset(&s_last_rx_msg, 0, sizeof(s_last_rx_msg));

    if (rx_header.IDE == CAN_ID_STD)
    {
        s_last_rx_msg.id = rx_header.StdId;
        s_last_rx_msg.is_extended = 0;
    }
    else
    {
        s_last_rx_msg.id = rx_header.ExtId;
        s_last_rx_msg.is_extended = 1;
    }

    s_last_rx_msg.dlc = rx_header.DLC;
    s_last_rx_msg.is_remote = (rx_header.RTR == CAN_RTR_REMOTE) ? 1 : 0;

    if (rx_header.DLC > 8)
    {
        s_last_rx_msg.dlc = 8;
    }

    memcpy(s_last_rx_msg.data, rx_data, s_last_rx_msg.dlc);
    s_msg_available = 1;
}