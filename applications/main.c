/*****************************************************************
*                                                                *
*              文件名:   main.c                                  *
*              内容简述: STM32 xxx蓝牙网关                       *
*              作者：    wanchenchen                             *
*              版本：    V0.0.0                                  *
*              记录：                                            *
*                                                                *
******************************************************************/

/* Includes -----------------------------------------------------*/
#include "main.h"

/*=========================版本号================================*/
image_version_t Gateway_Version = {0 , 0 , 6};
uint8_t btgw_DeviceID[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//test buf
//const uint8_t testbuf[24] = {0xfe,0xbe,0x01,0x11,0x99,0x88,0x77,0x66,0x55,0x44,0x00,0x11,0x22,0x01,0x89,0x98,0x12,0x55,0x66,0x66,0x55,0x86,0x0d,0x0a};

	/* Define -------------------------------------------------------*/
#define BEELINKER_PROTOCOL_VER      (0x01)
#define OVER_OF_REQNEWIMAGE_TIME    (SYS_TICK_PER_SECOND*3)
#define FEED_IWDG_TIME              (SYS_TICK_PER_SECOND/2)
#define LED_FLUSH_TIME              (SYS_TICK_PER_SECOND/10)
#define OVER_UPDATA_TIME            (SYS_TICK_PER_SECOND*300)
#define DELETE_TAG_TIME             (SYS_TICK_PER_SECOND*2)
#define REQUEST_NEWIMAGE_DATA_LENS  (256)
#define MAX_LORATAG_COUNTER         (10)

#define UPLOAD1_NO_ACK_OVER_TIME    (SYS_TICK_PER_SECOND*30)
#define UPLOAD2_NO_ACK_OVER_TIME    (SYS_TICK_PER_SECOND*60)
#define NO_ACK_MAX_COUNTER          (3)
#define NO_LORATAG_CHECK_COUNTER    (30)

/* Variable -----------------------------------------------------*/
uint8_t btgw_preFix[] = {'L', 'R', 'G', 'W'};

uint8_t bltag_preFix[] = {0xFE, 0xBE};
uint8_t bltag_sufFix[] = {0x0D, 0x0A};

uint32_t feed_iwdg_time;
uint32_t lst_led_time;
uint32_t lst_updata_time;
uint8_t lora_rx_check_time;
int wifi_config_counter = 5;

uint32_t not_detect_ack_time;
uint8_t select_detect_mode = 0;
uint32_t not_detect_over_time = UPLOAD1_NO_ACK_OVER_TIME;
uint8_t detect_over_counter = 0;
uint8_t rst_net_module_counter = 0;

gateway_publishVer_t NewImage_Version;
uint8_t StartUpdataVersion = 0;
uint8_t StartReqNextPkt;
uint32_t lst_reqpkt_time;
uint32_t req_next_addr;

uint32_t lst_sendtag_time;
uint8_t  loratag_counter = 0;
loratg_record_t loratag_record[MAX_LORATAG_COUNTER];
bleinf_record_t bleinf_record[MAX_LORATAG_COUNTER];   
uint8_t lorataglist[6*MAX_LORATAG_COUNTER];
uint8_t  sendloratag_buf[300];
static lorainf_mgr_t  lora1inf_mgr;
static lorainf_mgr_t  lora2inf_mgr;

/* Function -----------------------------------------------------*/
static void Uart2AppTask(void);
static void ServerUploadResponseCmd(uint8_t cmd, uint8_t*payload_buf, uint16_t payload_len);
static void ServerPublishVersionCmd(uint8_t cmd, uint8_t*payload_buf, uint16_t payload_len);
static void ServerRepNewimageCmd(uint8_t cmd, uint8_t*payload_buf, uint16_t payload_len);
static void ServerSynchroTimeCmd(uint8_t cmd, uint8_t*payload_buf, uint16_t payload_len);

static void LoraAppTask(void);
/*************************************************
函数: int main(void)
功能: main主函数
参数: 无
返回: 无
*************************************************/
int main(void)
{
    uint16_t j = 0, inf_offset = 0;
    uint32_t temp_syn_time;
    uint16_t dstblkOffset;
    nv_user_param_t *user_param;
    
    //MCU时钟初始化
	RCC_Configuration();
    //中断向量初始化
	NVIC_Configuration();
    //设置调试器模式为SWD
	JTAG_Set(SWD);
    //定时器初始化
    Timer4_Configuration();
    //延时初始化
	delay_init();
    //LED引脚初始化
	LED_Init();
    //串口初始化
	UART_Work_Init();
    //Flash参数初始化
    Nvram_Init();
	//Lora模块初始化
    sx1278_1Init();
    sx1278_2Init();
//    lora1inf_mgr.rxbuf = (uint8_t *)testbuf;
//	lora1inf_mgr.rxsize = sizeof(testbuf);
	
    LED_Control(LED0,Off);
    LED_Control(LED1,Off);
    LED_Control(LED2,Off);	
    LED_Control(LED3,Off);
    LED_Control(LED4,Off);
    delay_ms(500);
    LED_Control(LED0,On);
    LED_Control(LED1,On);
    LED_Control(LED2,On);	
    LED_Control(LED3,On);
    LED_Control(LED4,On);
    delay_ms(500);
    LED_Control(LED0,Off);
    LED_Control(LED1,Off);
    LED_Control(LED2,Off);	
    LED_Control(LED3,Off);
    LED_Control(LED4,Off);
    
#ifdef IWDG_ENABLE
    //打开看门狗
    IWDG_Configuration();
#endif    
    
#ifdef MD5_ENABLE
    int i;
    
    //get device id
    for(i = 0; i < sizeof(btgw_DeviceID); i ++)
    {
        if(btgw_DeviceID[i] != 0xFF)
            break;
    }
    if(i == sizeof(btgw_DeviceID))
        GetDeviceId(btgw_DeviceID, sizeof(btgw_DeviceID));
#endif

	INTERRUPTS_ON();
    LED_Control(LED0,On);
	
    Nvram_Get_BlkOffset_By_Name("user", &dstblkOffset);
    user_param = (nv_user_param_t *)(CFG_NVRAM_ADDR + dstblkOffset);
    if(user_param->net_mode == 0)
    {
        LED_Control(STN_RST,Off);

WIFI_CONFIG_START:
        LED_Control(CHIPEN,Off);
        delay_ms(100);
        LED_Control(CHIPEN,On);
        delay_ms(2000);
        
        //AT配置wifi模块
        if(Wifi_Config(wifi_config_counter) == -1)
        {
            if(wifi_config_counter > 0)
            {
                wifi_config_counter--;
                goto WIFI_CONFIG_START;
            }
        }
        
        LED_Control(CHIPEN,Off);
        delay_ms(100);
        LED_Control(CHIPEN,On);
        
        ATOMIC
        (
            Uart2_RxCnt = 0;
            Uart2_RxBuf_Read = 0;
            Uart2_RxBuf_Write = 0;
            memset(Uart2_RxBuf, 0, sizeof(Uart2_RxBuf));
        )
    }
    else
    {
        LED_Control(CHIPEN,Off);
        LED_Control(STN_RST,Off);
        delay_ms(100);
        LED_Control(STN_RST,On);
        delay_ms(500);
        
        //直接启动串口转以太网模块，开始运行
    }
    
    //初始化检测变量
    select_detect_mode = 0;
    not_detect_ack_time = get_current_tick();
    not_detect_over_time = UPLOAD1_NO_ACK_OVER_TIME;
    detect_over_counter = 0;
    lora_rx_check_time  = 0;
	
    while(1)
    {
#ifdef IWDG_ENABLE
        //清看门狗
        if(get_current_tick() - feed_iwdg_time > FEED_IWDG_TIME)
        {
            IWDG_ReloadCounter();
            feed_iwdg_time = get_current_tick();
        }
#endif 
        
        //LED灯控制
        if(get_current_tick() - lst_led_time > LED_FLUSH_TIME)
        {
            if(user_param->net_mode == 0)
            {
                LED_Control(LED1,Off);
                LED_Control(LED3,Off);	
            }
            else
            {                
                LED_Control(LED2,Off);	 
                LED_Control(LED4,Off);
            }

            lst_led_time = get_current_tick();            
        }
        
        //与服务器通讯函数
        Uart2AppTask();
        
        //接收到新版本的固件推送,发送请求获取固件
        if(StartUpdataVersion == 1)
        {
            if( (StartReqNextPkt == 1) || (get_current_tick()-lst_reqpkt_time > OVER_OF_REQNEWIMAGE_TIME) )
            {
                gateway_reqNewVer_t req_newversion_pkt;
                
                memcpy(&(req_newversion_pkt.newimage_header.prefix), btgw_preFix, sizeof(btgw_preFix));
                req_newversion_pkt.newimage_header.version = BEELINKER_PROTOCOL_VER;
                memcpy(&(req_newversion_pkt.newimage_header.devid), btgw_DeviceID, sizeof(btgw_DeviceID));
                req_newversion_pkt.newimage_header.cmd_type = TYPE_IMGREQ;
                req_newversion_pkt.newimage_header.len = sizeof(gateway_reqNewVer_t) - sizeof(gateway_pkt_hdr_t) - sizeof(uint8_t);
                req_newversion_pkt.newimage_header.len = ntohs(req_newversion_pkt.newimage_header.len);
                
                memcpy(&(req_newversion_pkt.newimage_ver), &(NewImage_Version.newimage_ver), sizeof(image_version_t));
                req_newversion_pkt.newimage_ver.PATCH = ntohs(req_newversion_pkt.newimage_ver.PATCH);
                req_newversion_pkt.newimage_offset = req_next_addr;
                req_newversion_pkt.newimage_offset = ntohl(req_newversion_pkt.newimage_offset);
                req_newversion_pkt.newimage_reqsize = REQUEST_NEWIMAGE_DATA_LENS;
                req_newversion_pkt.newimage_reqsize = ntohs(req_newversion_pkt.newimage_reqsize);
                req_newversion_pkt.crc = checksum_8(0, (uint8_t*)&req_newversion_pkt, sizeof(gateway_reqNewVer_t) - sizeof(uint8_t));
                
                USART2_SendData((uint8_t*)&req_newversion_pkt, sizeof(gateway_reqNewVer_t));
                StartReqNextPkt = 0;
                lst_reqpkt_time = get_current_tick();
            }  
            
            if(get_current_tick()-lst_updata_time > OVER_UPDATA_TIME)
            {
                req_next_addr = 0;
                StartReqNextPkt = 0;
                lst_reqpkt_time = get_current_tick();
                StartUpdataVersion = 0;
                
                lst_updata_time = get_current_tick();
            }
        }
		
        //Lora功能处理函数
        LoraAppTask();
        
        //定时发送LoraTag信息
        temp_syn_time = get_synchro_time();
        if(lst_sendtag_time != temp_syn_time)
        {
            gateway_sendloratag_t*sendtag_pkt = (gateway_sendloratag_t*)sendloratag_buf;
            
            memcpy(&(sendtag_pkt->newimage_header.prefix), btgw_preFix, sizeof(btgw_preFix));
            sendtag_pkt->newimage_header.version = BEELINKER_PROTOCOL_VER;
            memcpy(&(sendtag_pkt->newimage_header.devid), btgw_DeviceID, sizeof(btgw_DeviceID));
            sendtag_pkt->newimage_header.cmd_type = TYPE_UPLOAD;
            
            memcpy(&(sendtag_pkt->newimage_ver), &(Gateway_Version), sizeof(image_version_t));
            sendtag_pkt->newimage_ver.PATCH = ntohs(sendtag_pkt->newimage_ver.PATCH);
            sendtag_pkt->now_time = htonl(temp_syn_time);
            sendtag_pkt->loratag_counter = loratag_counter;

            inf_offset += (uint16_t)((uint8_t*)&sendtag_pkt->loratag_counter - (uint8_t*)sendtag_pkt) + 1;
            for(j = 0; j < loratag_counter; j++)
            {
                memcpy((uint8_t*)sendtag_pkt + inf_offset, &loratag_record[j].macaddress[0], 10);
                inf_offset += 10;
                memcpy((uint8_t*)sendtag_pkt + inf_offset, loratag_record[j].blebuf_ptr, sizeof(bleinf_record_t)*loratag_record[j].blenum);
                inf_offset += sizeof(bleinf_record_t)*loratag_record[j].blenum;     
            }
            

            if(user_param->net_mode == 0)
            {
                LED_Control(LED1,On);
                LED_Control(LED3,On);	 
            }
            else
            {
                LED_Control(LED2,On);	 
                LED_Control(LED4,On);
            }
                
            sendtag_pkt->newimage_header.len = inf_offset-((uint16_t)((uint8_t*)&sendtag_pkt->newimage_header.len  - (uint8_t*)sendtag_pkt)) - sizeof(uint16_t);
            sendtag_pkt->newimage_header.len = ntohs(sendtag_pkt->newimage_header.len);
            *((uint8_t*)sendtag_pkt + inf_offset ) = checksum_8(0, (uint8_t*)sendtag_pkt, inf_offset);
                
            USART2_SendData((uint8_t*)sendtag_pkt, inf_offset + sizeof(uint8_t));
            
            lst_led_time = get_current_tick();
            detect_over_counter++;
        
            lst_sendtag_time = temp_syn_time;
			
			//删除LORA_TAG信息
			inf_offset = 0;
			
			if(loratag_counter == 0)
				lora_rx_check_time ++;			
			else
			{
				loratag_counter    = 0;
				lora_rx_check_time = 0;
			}
        }
        
        //检测上传回复命令
        if(select_detect_mode == 0)
        {
            if(get_current_tick() - not_detect_ack_time > not_detect_over_time)
            {
                select_detect_mode = 1;
                detect_over_counter = 0;
            }
        }
        else
        {
            if(detect_over_counter > NO_ACK_MAX_COUNTER)
            {
                if(user_param->net_mode == 0)
                {
                    LED_Control(CHIPEN,Off);
                    delay_ms(100);
                    LED_Control(CHIPEN,On);
                    delay_ms(2000);
                }
                else
                {
                    LED_Control(STN_RST,Off);
                    delay_ms(100);
                    LED_Control(STN_RST,On);
                    delay_ms(500);
                }
                
                select_detect_mode = 0;
                not_detect_ack_time = get_current_tick();
                rst_net_module_counter++;
                if(rst_net_module_counter == 1)
                {
                    not_detect_over_time = UPLOAD1_NO_ACK_OVER_TIME;
                }
                else if(rst_net_module_counter == 2)
                {
                    not_detect_over_time = UPLOAD2_NO_ACK_OVER_TIME;
                }
                else if(rst_net_module_counter >= 3)
                {
                    Sys_Soft_Reset();
                }
            }
        }
    }
}

/*************************************************
函数: static void Uart2AppTask(void)
功能: 处理与服务器通讯函数
参数: 无
返回: 无
*************************************************/
#define MAX_DATA_LEN        512
uint8_t usart2_tmpBuf[MAX_DATA_LEN];
static void Uart2AppTask(void)
{
    uint16_t tmpCnt, tmpRead, count;
    uint8_t crc;
    uint8_t cmd;
    uint16_t len;
    uint16_t i;   
    gateway_pkt_hdr_t*pkt_hdr = (gateway_pkt_hdr_t*)usart2_tmpBuf;
    
    ATOMIC
    (
        tmpCnt = Uart2_RxCnt;
        tmpRead = Uart2_RxBuf_Read;
    )
    
    if (tmpCnt < sizeof(gateway_pkt_hdr_t))
    {
        return;
    }
    
    memset(usart2_tmpBuf,0,MAX_DATA_LEN);
    count = 0;
    for(i = 0; i < sizeof(gateway_pkt_hdr_t); i ++)
    {
        usart2_tmpBuf[i] = Uart2_RxBuf[tmpRead++];
        if(tmpRead >= UART2_RXBUF_SIZE)         
            tmpRead = 0;
        count ++;
    }
    
    for(i = 0; i < sizeof(btgw_preFix); i ++)
    {
        if (usart2_tmpBuf[i] != btgw_preFix[i])
            break;
    }
    
    if (i != sizeof(btgw_preFix)) // not a valid preFix
        goto INVALID_PACKET;
    
    len = ntohs(pkt_hdr->len);      
    cmd = pkt_hdr->cmd_type;
            
    if (len > (MAX_DATA_LEN - sizeof(gateway_pkt_hdr_t) -sizeof(crc)))
        goto INVALID_PACKET;

    if (tmpCnt < sizeof(gateway_pkt_hdr_t) + len + sizeof(crc))
    {
        // packet has not been received completely.
        return;
    }

    for (i = 0; i < (len+sizeof(crc)); i ++)
    {
        usart2_tmpBuf[sizeof(gateway_pkt_hdr_t)+i] = Uart2_RxBuf[tmpRead ++];
        if(tmpRead >= UART2_RXBUF_SIZE)         
            tmpRead = 0;
        count ++;
    }
    
    crc = checksum_8(0, usart2_tmpBuf, sizeof(gateway_pkt_hdr_t) + len);
    if (crc != usart2_tmpBuf[sizeof(gateway_pkt_hdr_t) + len]) // not a valid preFix
        goto INVALID_PACKET;
         
    /* a valid packet is received */
    ATOMIC
    (
        Uart2_RxBuf_Read = tmpRead;
        Uart2_RxCnt -= count;
    )

    if( 0 == memcmp(pkt_hdr->devid, btgw_DeviceID, sizeof(btgw_DeviceID)) )
    {
        switch (cmd)
        {
            case TYPE_UPLOADRESP:   //0x81
                ServerUploadResponseCmd(cmd, (uint8_t*)(++pkt_hdr), len);
                break;
            
            case TYPE_IMGPUSH:   //0x02
                ServerPublishVersionCmd(cmd, (uint8_t*)(++pkt_hdr), len);
                break;
            
            case TYPE_IMGREQRESP:  //0x83
                ServerRepNewimageCmd(cmd, (uint8_t*)(++pkt_hdr), len);
                break;

            case TYPE_SYNCHROREQ:  //0x04
                ServerSynchroTimeCmd(cmd, (uint8_t*)(++pkt_hdr), len);
                break;
            
            default:
                break;
        }
    }
            
    /* end */
    return;

INVALID_PACKET:
    ATOMIC
    (
        Uart2_RxBuf_Read++;
        if(Uart2_RxBuf_Read >= UART2_RXBUF_SIZE)         
            Uart2_RxBuf_Read = 0;
        Uart2_RxCnt --;
    )
}

static void ServerUploadResponseCmd(uint8_t cmd, uint8_t*payload_buf, uint16_t payload_len)
{
    gateway_uploadResp_t*pkt = (gateway_uploadResp_t*)payload_buf;
    
    if(payload_len != sizeof(gateway_uploadResp_t))
        return;
    
    detect_over_counter = 0;
    rst_net_module_counter = 0;
}

static void ServerPublishVersionCmd(uint8_t cmd, uint8_t*payload_buf, uint16_t payload_len)
{
    gateway_rsppublishVer_t rsp_publishVer_pkt;
    gateway_publishVer_t*pkt = (gateway_publishVer_t*)payload_buf;
    
    if(payload_len != sizeof(gateway_publishVer_t))
        return;
    
    pkt->newimage_ver.PATCH = ntohs(pkt->newimage_ver.PATCH); 
    if(0 == memcmp(&Gateway_Version, &(pkt->newimage_ver), sizeof(image_version_t)))
        goto RSP_PACKET;
    
    if(0 == memcmp(&(NewImage_Version.newimage_ver), &(pkt->newimage_ver), sizeof(image_version_t)))
        goto RSP_PACKET;

    pkt->newimage_size = ntohl(pkt->newimage_size); 
    memcpy(&NewImage_Version, pkt, sizeof(gateway_publishVer_t));
    
    while(1)
    {
        if(0 == Flash_Area_Erase(CFG_BKP_ADDR, CFG_BKP_SIZE))
            break;
    }
    
    req_next_addr = 0;
    StartReqNextPkt = 1;
    lst_reqpkt_time = get_current_tick();
    StartUpdataVersion = 1;
    lst_updata_time = get_current_tick();
    
RSP_PACKET:           
    memcpy(&(rsp_publishVer_pkt.newimage_header.prefix), btgw_preFix, sizeof(btgw_preFix));
    rsp_publishVer_pkt.newimage_header.version = BEELINKER_PROTOCOL_VER;
    memcpy(&(rsp_publishVer_pkt.newimage_header.devid), btgw_DeviceID, sizeof(btgw_DeviceID));
    rsp_publishVer_pkt.newimage_header.cmd_type = TYPE_IMGPUSHRESP;
    rsp_publishVer_pkt.newimage_header.len = sizeof(gateway_rsppublishVer_t) - sizeof(gateway_pkt_hdr_t) - sizeof(uint8_t);
    rsp_publishVer_pkt.newimage_header.len = ntohs(rsp_publishVer_pkt.newimage_header.len);
    
    rsp_publishVer_pkt.result = 0x00;
    rsp_publishVer_pkt.crc = checksum_8(0, (uint8_t*)&rsp_publishVer_pkt, sizeof(gateway_rsppublishVer_t) - sizeof(uint8_t));
    
    USART2_SendData((uint8_t*)&rsp_publishVer_pkt, sizeof(gateway_rsppublishVer_t));
}

static void ServerRepNewimageCmd(uint8_t cmd, uint8_t*payload_buf, uint16_t payload_len)
{
    gateway_rspNewVer_t*pkt = (gateway_rspNewVer_t*)payload_buf;
    uint8_t*ptr_data = (uint8_t*)(payload_buf + sizeof(gateway_rspNewVer_t));
    uint16_t data_lens = payload_len-sizeof(gateway_rspNewVer_t);
    
    if(pkt->result == 0x01)
        goto FAILED_RSP;
    
    pkt->newimage_ver.PATCH = ntohs(pkt->newimage_ver.PATCH); 
    if(0 != memcmp(&(NewImage_Version.newimage_ver), &(pkt->newimage_ver), sizeof(image_version_t)))
        goto FAILED_RSP;
    
    pkt->newimage_offset = ntohl(pkt->newimage_offset); 
    if(pkt->newimage_offset != req_next_addr)
        goto FAILED_RSP;
    
    if(data_lens != REQUEST_NEWIMAGE_DATA_LENS)
    {
        if( (pkt->newimage_offset + data_lens) < NewImage_Version.newimage_size )
            goto FAILED_RSP;
    }
    
    if(0 != Flash_Area_Prog(CFG_BKP_ADDR+pkt->newimage_offset, ptr_data, data_lens))
    {
        //编程出错,待处理
        uint32_t erase_size = ((pkt->newimage_offset%FLASH_PAGE_SIZE) + data_lens + FLASH_PAGE_SIZE - 1)&FLASH_PAGE_SIZE;
        while(1)
        {
            if(0 == Flash_Area_Erase((CFG_BKP_ADDR+pkt->newimage_offset)&FLASH_PAGE_SIZE, erase_size))
                break;
        }
        req_next_addr = (CFG_BKP_ADDR+pkt->newimage_offset)&FLASH_PAGE_SIZE;
        goto FAILED_RSP;
    }
    
    if( (pkt->newimage_offset + data_lens) >= NewImage_Version.newimage_size )
    {
        if (0 == imageCheck(CFG_BKP_ADDR))
            Sys_Soft_Reset();
        
        memset(&NewImage_Version, 0, sizeof(gateway_publishVer_t));
        StartUpdataVersion = 0;
        StartReqNextPkt = 0;
        lst_reqpkt_time = get_current_tick();
        return;
    }
    
    req_next_addr += data_lens;
FAILED_RSP:
    StartReqNextPkt = 1;
    lst_reqpkt_time = get_current_tick();
}

static void ServerSynchroTimeCmd(uint8_t cmd, uint8_t*payload_buf, uint16_t payload_len)
{
    gateway_rspsynchroTime_t rsp_synchroTime_pkt;
    gateway_synchroTime_t*pkt = (gateway_synchroTime_t*)payload_buf;
    
    if(payload_len != sizeof(gateway_synchroTime_t))
        return;
    
    set_synchro_time(ntohl(pkt->synchro_time));
    
    memcpy(&(rsp_synchroTime_pkt.synchro_header.prefix), btgw_preFix, sizeof(btgw_preFix));
    rsp_synchroTime_pkt.synchro_header.version = BEELINKER_PROTOCOL_VER;
    memcpy(&(rsp_synchroTime_pkt.synchro_header.devid), btgw_DeviceID, sizeof(btgw_DeviceID));
    rsp_synchroTime_pkt.synchro_header.cmd_type = TYPE_SYNCHROREQRESP;
    rsp_synchroTime_pkt.synchro_header.len = sizeof(rsp_synchroTime_pkt) - sizeof(gateway_pkt_hdr_t) - sizeof(uint8_t);
    rsp_synchroTime_pkt.synchro_header.len = ntohs(rsp_synchroTime_pkt.synchro_header.len);
    
    rsp_synchroTime_pkt.result = 0x00;
    rsp_synchroTime_pkt.now_time = ntohl(get_synchro_time());
    rsp_synchroTime_pkt.crc = checksum_8(0, (uint8_t*)&rsp_synchroTime_pkt, sizeof(gateway_rspsynchroTime_t) - sizeof(uint8_t));
    
    USART2_SendData((uint8_t*)&rsp_synchroTime_pkt, sizeof(rsp_synchroTime_pkt));
}

/***********************Lora功能处理函数*************************/
#define BL_MAX_RXDATA_LEN        45
#define BL_MAX_TXDATA_LEN        25
uint8_t lora_RFRXbuf[BL_MAX_RXDATA_LEN];
uint8_t lora1_RFTXbuf[BL_MAX_TXDATA_LEN];
uint8_t lora2_RFTXbuf[BL_MAX_TXDATA_LEN];
/*************************************************
函数: static void Loratag_Addlist(uint8_t *macaddress , uint8_t length)
功能: 添加新的设备并过滤重复设备
参数: 设备MAC地址指针，地址长度
返回: true or false
*************************************************/
#define LORATAT_MACADDRE_LEN    6
static bool Loratag_Addlist(uint8_t *macaddress , uint8_t length)
{
	uint8_t i;
	
	if(loratag_counter >= MAX_LORATAG_COUNTER)
		return false;
	
	for(i=0; i<loratag_counter; i++)
	{
		if(!memcmp(macaddress, &lorataglist[i*LORATAT_MACADDRE_LEN], LORATAT_MACADDRE_LEN))
			break;
	}
	
	if(i < loratag_counter)
		return false;
	else	
		memcpy(&lorataglist[loratag_counter*LORATAT_MACADDRE_LEN], macaddress, LORATAT_MACADDRE_LEN);
		
	return true;
}
/*************************************************
函数: static void LoraTagInf_Resp(uint8_t *txbuf , uint8_t *txlen)
功能: LoraTag上传信息回复帧
参数: 发送buf地址，发送buf长度指针
返回: 无
*************************************************/
static uint8_t LoraTagInf_Resp(uint8_t *txbuf)
{
	uint8_t *ptr;
	uint8_t count;
	
	if(!txbuf)
		return 0;
	
	ptr = txbuf;
	
	memcpy(ptr, bltag_preFix, sizeof(bltag_preFix));
	ptr += sizeof(bltag_preFix);
	
	*ptr++ = TYPE_LORATAUPRESP; 
	
	*ptr++ = LORATAT_MACADDRE_LEN;
	
	memcpy(ptr, &lorataglist[loratag_counter-1], LORATAT_MACADDRE_LEN);
	ptr += LORATAT_MACADDRE_LEN;
    
    *ptr++ = checksum_8(0, txbuf+sizeof(bltag_preFix), LORATAT_MACADDRE_LEN+sizeof(uint16_t));	
    
	memcpy(ptr, bltag_sufFix, sizeof(bltag_sufFix));
	ptr += sizeof(bltag_sufFix);
	
	count = ptr - txbuf;
	
	return count;
}

/*************************************************
函数: static void LoraTagInf_Cmd(uint8_t *buf, uint8_t size)
功能: 解析LoraTag身份卡信息上传帧
参数: 负载 ，负载大小
返回: 无
*************************************************/
static uint8_t LoraTagInf_Cmd(uint8_t *payload, uint8_t size)
{
	uint8_t count = 0;
	uint8_t offset;
	
	memcpy(&loratag_record[loratag_counter].macaddress[0], &payload[count] , sizeof(loratg_record_t) - 1);	
	if(!Loratag_Addlist(&loratag_record[loratag_counter].macaddress[0], LORATAT_MACADDRE_LEN))
		return 0;
	
	count += 10;//sizeof(loratg_record_t) - 1;
	
	if(loratag_counter > 0)
	{
		offset = loratag_record[loratag_counter - 1].blenum * sizeof(bleinf_record_t);
		loratag_record[loratag_counter].blebuf_ptr = (uint8_t *)(&bleinf_record[loratag_counter -1].index[0] + offset);
	}
	else 
		loratag_record[loratag_counter].blebuf_ptr = (uint8_t *)bleinf_record;
	
	memcpy(loratag_record[loratag_counter].blebuf_ptr, &payload[count], loratag_record[loratag_counter].blenum*sizeof(bleinf_record_t));
	
	loratag_counter ++;
		
	return 1;
}

/*************************************************
函数: static void LoraRxData_Handle(void)
功能: 解析过滤Lora接收的数据
参数: 数据Buf ，buf大小
返回: 无
*************************************************/
static uint8_t LoraRxData_Handle(lorainf_mgr_t *mgr)
{
	uint16_t tmpCnt;
	uint8_t crc;
	uint8_t cmd;
	uint8_t len;
	uint8_t res;
	uint8_t *buf;
	uint8_t size;
	uint8_t i = 0;
	
	//loratg_record_t *loratag_hdr = &loratag_record[loratag_counter];
	
	if(loratag_counter >= MAX_LORATAG_COUNTER)
		return 0;	
	
	buf  = mgr->rxbuf;
	size = mgr->rxsize;
	res  = 0;
	
	while(i < size)
	{
		if(!memcmp(&buf[i], bltag_preFix, sizeof(bltag_preFix)))
			break;
		i ++;		
	}
	
	if ((size - i) < sizeof(bltag_preFix) + sizeof(cmd) + sizeof(len) + sizeof(loratg_record_t) + sizeof(crc) + sizeof(bltag_sufFix) - sizeof(uint8_t)) // PreFix + LEN + CMD + CRC + SufFix
	{
		return 0;
	}
	
	tmpCnt = i + sizeof(bltag_preFix);
	
	cmd = buf[tmpCnt];
	
	len = buf[tmpCnt + 1];
	
	if(len + sizeof(cmd) + sizeof(bltag_preFix) + sizeof(len) + sizeof(crc) + sizeof(bltag_sufFix) >  size-i)
		return 0;
	
	crc = buf[tmpCnt + sizeof(uint16_t) + len];
	
	if(crc != checksum_8(0, &buf[tmpCnt], len + sizeof(uint16_t)))
		return 0;
	
	if(memcmp(&buf[tmpCnt + len + sizeof(uint16_t) + sizeof(uint8_t)], bltag_sufFix, sizeof(bltag_preFix)))
		return 0;
	
	memset(lora_RFRXbuf, 0, BL_MAX_RXDATA_LEN);
	memcpy(lora_RFRXbuf, &buf[tmpCnt+sizeof(uint16_t)], len);
	
	switch(cmd)
	{
		case TYPE_LORATAGUP:
			res = LoraTagInf_Cmd(lora_RFRXbuf, len);
			if(res)
			{
				mgr->txsize = LoraTagInf_Resp(mgr->txbuf);
			}
			break;
		default:
			break;
	}
	
	return res;
}

/*************************************************
函数: static void LoraAppTask(void)
功能: 处理与Lora相关的进程
参数: 无
返回: 无
*************************************************/
static void LoraAppTask(void)
{		
	switch(sx1278Lora_1Process())
	{
		case RFLR_STATE_RX_DONE:	
			lora1inf_mgr.rxbuf = (uint8_t *)sx1278Lora_1GetRxData(&lora1inf_mgr.rxsize );
			break;
		
		case RFLR_STATE_TX_RUNNING:
			if(lora1inf_mgr.txflg == false)
			{
				sx1278Lora_1RFSendBuf(lora1inf_mgr.txbuf, lora1inf_mgr.txsize);
				lora1inf_mgr.txsize = 0;
				lora1inf_mgr.txflg = true;
			}
			break;
			
		case RFLR_STATE_TX_DONE:
			lora1inf_mgr.txflg = false;
			sx1278Lora_1SetRFStatus(RFLR_STATE_RX_INIT);
			break;
		
		default:
			break;		
	}
		
	switch(sx1278Lora_2Process())
	{	
		case RFLR_STATE_RX_DONE:
			lora2inf_mgr.rxbuf = (uint8_t *)sx1278Lora_2GetRxData(&lora2inf_mgr.rxsize );	
			break;
			
		case RFLR_STATE_TX_RUNNING:
			if(lora2inf_mgr.txflg == false)
			{
				sx1278Lora_2RFSendBuf(lora2inf_mgr.txbuf, lora2inf_mgr.txsize);
				lora2inf_mgr.txsize = 0;
				lora2inf_mgr.txflg = true;
			}
			break;
			
		case RFLR_STATE_TX_DONE:
			lora2inf_mgr.txflg = false;
			sx1278Lora_2SetRFStatus(RFLR_STATE_RX_INIT);
			break;
		
		default:
			break;			
	}
	
	if(lora1inf_mgr.rxsize != 0)
	{
		if(lora1inf_mgr.rxbuf != NULL)
		{	
			lora1inf_mgr.txbuf = lora1_RFTXbuf; 	
			LoraRxData_Handle(&lora1inf_mgr);
			
			if(lora1inf_mgr.txsize !=0)	
				sx1278Lora_1SetRFStatus(RFLR_STATE_TX_INIT);			
		}	
		lora1inf_mgr.rxsize = 0;
	}
	
	if(lora2inf_mgr.rxsize != 0)
	{
		if(lora2inf_mgr.rxbuf != NULL)
		{
			lora2inf_mgr.txbuf = lora2_RFTXbuf; 
			LoraRxData_Handle(&lora2inf_mgr);
		
			if(lora2inf_mgr.txsize !=0)	
				sx1278Lora_2SetRFStatus(RFLR_STATE_TX_INIT);
		}	
		lora2inf_mgr.rxsize = 0;
	}
	
	if(lora_rx_check_time > NO_LORATAG_CHECK_COUNTER)
	{
		lora_rx_check_time = 0;
		sx1278Lora_1SetRFStatus(RFLR_STATE_RX_INIT);
		sx1278Lora_2SetRFStatus(RFLR_STATE_RX_INIT);
	}
}
