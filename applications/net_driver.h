
#ifndef __NET_DRIVER_H__
#define __NET_DRIVER_H__ 

#include "main.h"

//ATÃüÁî½á¹¹Ìå
typedef struct
{
    char*       cmd_str;
    char*       replay_str;
    uint32_t     cmd_timeout;
}AT_COMMAND;

int Wifi_Config(int ignore_flag);

uint8_t Wifi_Send_Data(uint8_t *data, uint8_t *ack, uint16_t waittime);
uint8_t Wifi_Quit_Trans(void);

#endif





