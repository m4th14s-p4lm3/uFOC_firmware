#include "commands.h"
#include "communication.h"
#include "debug_utils.h"
#include <stdio.h>
#include "config.h"
#include "encoder.h"
#include <string.h>


float get_float_value_from_message(can_message_t* msg){
    float value;
    memcpy(&value, &msg->data[2], sizeof(float));
    return value;
}

uint8_t data[8];
void assemble_data_with_float_value(uint8_t command, float value){ // data is expected as a pointer to uint8_t data[6]
    data[0] = CAN_COMMUNICATION_DEVICE_ID;
    data[1] = command;
    memcpy(&data[2], &value, sizeof(float));
}

void handle_communication(config_t* config){
    can_message_t msg;
    if(!communication_read(&msg)) return;
    uint8_t target_device_id = (uint8_t)msg.data[0];
    if (target_device_id != CAN_COMMUNICATION_DEVICE_ID) return;
    uint8_t command = (uint8_t)msg.data[1];

    char buffer[128];

    switch(command){
        case SET_CONTROL_STATE:
            enum ControlState new_control_state = (enum ControlState)msg.data[2];
            set_control_state(config, new_control_state);
            break;

        case SET_TORQUE_CURRENT_TARGET:
            float new_torque_current_target = get_float_value_from_message(&msg);
            set_torque_current_target(config, new_torque_current_target);
            break;

        case SET_TORQUE_CURRENT_SOFT_LIMIT:
            float new_torque_current_soft_limit = get_float_value_from_message(&msg);
            set_torque_current_soft_limit(config, new_torque_current_soft_limit);
            break;

        case GET_TORQUE_CURRENT_SOFT_LIMIT:
            float torque_current_soft_limit = get_torque_current_soft_limit(config);
            assemble_data_with_float_value(command, torque_current_soft_limit);
            communication_send(CAN_COMMUNICATION_STD_ID, data, 6);
            break;
        
        case SET_CURRENT_KP:
            float new_kp_c = get_float_value_from_message(&msg);
            set_current_kp(config, new_kp_c);
            break;
        case SET_CURRENT_KI:
            float new_ki_c = get_float_value_from_message(&msg);
            set_current_ki(config, new_ki_c);
            break;


        case SET_ANGULAR_VELOCITY_TARGET:
            float new_angular_velocity_target = get_float_value_from_message(&msg);
            set_angular_velocity_target(config, new_angular_velocity_target);
            break;

        case SET_ANGULAR_VELOCITY_SOFT_LIMIT:
            float new_angular_velocity_soft_limit = get_float_value_from_message(&msg);
            set_angular_velocity_soft_limit(config, new_angular_velocity_soft_limit);
            break;

        case GET_ANGULAR_VELOCITY_SOFT_LIMIT:
            float angular_velocity_soft_limit = get_angular_velocity_soft_limit(config);

            break;
        case GET_ANGULAR_VELOCITY:
            float angular_velocity = get_angular_velocity(&config->encoder);
            assemble_data_with_float_value(command, angular_velocity);
            communication_send(CAN_COMMUNICATION_STD_ID, data, 6);
            break;


        case SET_ANGULAR_VELOCITY_KP:
            float new_kp_av = get_float_value_from_message(&msg);
            set_angular_velocity_kp(config, new_kp_av); 
            break;

        case SET_ANGULAR_VELOCITY_KI:
            float new_ki_av = get_float_value_from_message(&msg);
            set_angular_velocity_ki(config, new_ki_av); 
            break;

        case SET_POSITION_TARGET:
            float new_position_target = get_float_value_from_message(&msg);
            set_position_target(config, new_position_target);
            sprintf(buffer, "Position target %f\r\n", new_position_target);
            print(buffer);
            break;

        case GET_POSITION_TARGET:
            float position_target = get_position_target(config);
            assemble_data_with_float_value(command, position_target);
            communication_send(CAN_COMMUNICATION_STD_ID, data, 6);
            break;

        case GET_POSITION:
            float position = get_position(config);
            assemble_data_with_float_value(command, position);
            communication_send(CAN_COMMUNICATION_STD_ID, data, 6);
            break;

        case GET_RELATIVE_POSITION:
            float relative_position = 0; // UPDATE THIS SHIT!!!
            assemble_data_with_float_value(command, relative_position);
            communication_send(CAN_COMMUNICATION_STD_ID, data, 6);
            break;

        
        case SET_POSITION_KP:
            float new_kp_pos = get_float_value_from_message(&msg);
            set_position_kp(config, new_kp_pos);
            break;
        case SET_POSITION_KI:
            float new_ki_pos = get_float_value_from_message(&msg);
            set_position_ki(config, new_ki_pos);
            break;
        case SET_POSITION_KD:
            float new_kd_pos = get_float_value_from_message(&msg);
            set_position_kd(config, new_kd_pos);
            break;

        case GET_ELECTRICAL_OFFSET:
            float electrical_offset = config->encoder.electrical_offset;
            assemble_data_with_float_value(GET_POSITION, electrical_offset);
            communication_send(CAN_COMMUNICATION_STD_ID, data, 6);
            break;

        case SET_ELECTRICAL_OFFSET:
            float new_electrical_offset = get_float_value_from_message(&msg);
            update_electrical_offset(&config->encoder, new_electrical_offset);
            break;
        
        case CALIBRATE_ELECTRICAL_OFFSET:
            
            break;
    }


}