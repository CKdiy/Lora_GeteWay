
#ifndef  __main_h__
#define  __main_h__

/* ------------------------Includes -----------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "atomic.h"
#include "endian.h"
#include "dbg.h" 

#include "systick.h"
#include "Led_Eval.h"
#include "Uart_Eval.h"
#include "Timer_Eval.h"
#include "Flash_Eval.h"
#include "Eval.h"
#include "crc.h"
#include "image.h"
#include "layout.h"
#include "MD5.h"
#include "nvram.h"
#include "net_driver.h"
#include "sx1278_lora.h"

/*-----------------------------define----------------------------------------*/


/*----------------------------Typedefs---------------------------------------*/
enum
{    
    TYPE_TAG_INFO           = 0x01,//上传TAG信息
    
    TYPE_WIFI_INFO          = 0x02,//上传WIFI信息
};
enum
{    
    TYPE_UPLOAD             = 0x01,//网关上传数据帧
    TYPE_UPLOADRESP         = 0x81,//网关回复上传数据帧
    
    TYPE_IMGPUSH            = 0x02,//服务器推送新版本
    TYPE_IMGPUSHRESP        = 0x82,
    
    TYPE_IMGREQ             = 0x03,//网关请求固件内容
    TYPE_IMGREQRESP         = 0x83,
    
    TYPE_SYNCHROREQ         = 0x04,//网关同步时间命令
    TYPE_SYNCHROREQRESP     = 0x84,
};
typedef __packed struct
{
    uint8_t   MAJOR;
    uint8_t   MINOR;
    uint16_t  PATCH;
}image_version_t;

typedef __packed struct
{
    uint8_t prefix[4];
    uint8_t version;
    uint8_t devid[8];
    uint8_t cmd_type;
    uint16_t len;
    //uint8_t payload[0];
}gateway_pkt_hdr_t;

typedef __packed struct
{
    uint32_t now_time;
}gateway_uploadResp_t;

typedef __packed struct
{
    image_version_t newimage_ver;
    uint32_t newimage_size;
}gateway_publishVer_t;

typedef __packed struct
{
    gateway_pkt_hdr_t newimage_header;
    uint8_t result;
    uint8_t crc;
}gateway_rsppublishVer_t;

typedef __packed struct
{
    gateway_pkt_hdr_t newimage_header;
    image_version_t newimage_ver;
    uint32_t newimage_offset;
    uint16_t newimage_reqsize;
    uint8_t crc;
}gateway_reqNewVer_t;

typedef __packed struct
{
    uint8_t result;
    image_version_t newimage_ver;
    uint32_t newimage_offset;
    uint16_t data_size;
    //uint8_t payload[0];
}gateway_rspNewVer_t;

typedef __packed struct
{
    gateway_pkt_hdr_t newimage_header;
    image_version_t newimage_ver;
    uint32_t now_time;
    uint8_t tag_counter;
    //uint8_t payload[0];
}gateway_sendtag_t;

typedef __packed struct
{
    uint32_t  synchro_time;
}gateway_synchroTime_t;

typedef __packed struct
{
    gateway_pkt_hdr_t synchro_header;
    uint8_t result;
    uint32_t now_time;
    uint8_t crc;
}gateway_rspsynchroTime_t;


typedef __packed struct
{
    uint8_t prefix[2];
    uint8_t cmd_type;
    uint8_t len;
    //uint8_t payload[0];
}bletg_pkt_hdr_t;

typedef __packed struct
{
    uint8_t status;
    uint8_t rssi;
    uint16_t major;
    uint16_t minor;
}bletg_info_t;

typedef __packed struct
{
    bletg_info_t tag_info;
    uint32_t lst_recv_time;
    uint8_t TAG_HAVING;
}bletg_record_t;

typedef __packed struct
{
	uint8_t *rxbuf;
	uint8_t *txbuf;
	uint8_t rxsize;
	uint8_t txsize;
}lorainf_mgr_t;
/* ---------------------Private variables -----------------------------------*/
/********************variables*************************/
extern image_version_t Gateway_Version;

extern uint8_t btgw_preFix[];
/******************************************************/

/********************Function**************************/

/******************************************************/


#endif

