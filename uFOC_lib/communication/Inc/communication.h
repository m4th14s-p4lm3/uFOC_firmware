#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SET_P 0x100
#define SET_I 0x101
#define SET_D 0x107

#define SET_MIN 0x102
#define SET_MAX 0x103

#define SET_TARGET_VELOCITY 0x104

#define SET_ID_REF 0x105
#define SET_IQ_REF 0x106


typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint8_t is_extended;
    uint8_t is_remote;
} can_message_t;

void communication_init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef communication_start(void);

HAL_StatusTypeDef communication_send(uint32_t std_id, const uint8_t *data, uint8_t len);
HAL_StatusTypeDef communication_send_test(void);

bool communication_message_available(void);
bool communication_read(can_message_t *msg);

void communication_rx_fifo0_pending_callback(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif