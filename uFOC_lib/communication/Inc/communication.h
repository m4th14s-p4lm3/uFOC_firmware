#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"


#ifdef __cplusplus
extern "C" {
#endif



typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint8_t is_extended;
    uint8_t is_remote;
} can_message_t;



// Data 
// byte 0 - device ID 
// byte 1 - command
// byte 2 - 7 - data based on data type 


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



