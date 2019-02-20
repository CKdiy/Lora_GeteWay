/*
 * nvram.h
 */
#ifndef __NVRAM_H__
#define __NVRAM_H__

#include <stdint.h>

#define BLK_MAX_SIZE                FLASH_PAGE_SIZE

#define WIFI_SSID_MAXLEN            64
#define WIFI_KEY_MAXLEN             64
#define SERVER_IP_DOMAIN_MAXLEN     64
#define BLK_NAME_SIZE               8

/**
 *  lora union
 */
typedef union 
{
	struct
	{
		uint8_t    power      :8;  // Bit15~Bit8 Lora Power
		uint8_t    channel    :5;  // Bit4~Bit0  channel
		uint8_t    rate       :3;  // Bit7~Bit5  rate
	}bit_t;
	
	uint16_t    loraPara;
}lora_para_n;

typedef struct
{
    uint32_t    crc32;
    char name[BLK_NAME_SIZE];
    
    char ssid[WIFI_SSID_MAXLEN];
    char key[WIFI_KEY_MAXLEN];
    char sip[SERVER_IP_DOMAIN_MAXLEN];      //server ip or server domain
    uint16_t    port;  
    uint8_t     net_mode; 
    lora_para_n lora_1para;
    lora_para_n lora_2para;
}nv_user_param_t;

#define UID_CRYPTO_LEN              32
#define HWVER_LEN                   2
typedef struct
{
    uint32_t    crc32;
    char name[BLK_NAME_SIZE];
    
    char crypto[UID_CRYPTO_LEN];
    char HWver[HWVER_LEN];

}nv_sys_param_t;


typedef struct
{
    char *name;
    uint16_t offset;
    uint16_t size;
} block_info_t;

int Nvram_Init(void);

int Nvram_Block_Write(char *name, void *buf, uint16_t size);

int Nvram_Block_Read(char *name, void *buf, uint16_t size);

int Nvram_Block_Check(char *name);

int Nvram_Block_Init(char *name);

int Nvram_Get_BlkOffset_By_Name(char *name, uint16_t *offset);


#endif
