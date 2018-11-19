
#include "net_driver.h"

const AT_COMMAND At_Command[]=
{
    {"AT+RESTORE\r\n","",2},
    {"AT+CWMODE=1\r\n","OK",2},
    {"AT+CWDHCP=1,1\r\n","OK",2},
    {"AT+CWJAP","OK",20},
    {"AT+CWAUTOCONN=1\r\n","OK",2},
    {"AT+SAVETRANSLINK","OK",2},
};

//向Wifi模块发送指定数据
//data:发送的数据(不需要添加回车了)
//ack:期待的应答结果,如果为空,则表示不需要等待应答
//waittime:等待时间(单位:10ms)
//返回值:0,发送成功(得到了期待的应答结果)
uint8_t Wifi_Send_Data(uint8_t *data, uint8_t *ack, uint16_t waittime)
{
	uint8_t res=1;
    uint32_t wifi_at_time;
    
    ATOMIC
    (
        Uart2_RxCnt = 0;
        Uart2_RxBuf_Read = 0;
        Uart2_RxBuf_Write = 0;
        Uart_Full = 0;
        memset(Uart2_RxBuf, 0, sizeof(Uart2_RxBuf));
    )
    USART2_SendData(data, strlen((char*)data));//发送命令
    wifi_at_time = get_current_tick();
    
	if(ack&&waittime)		//需要等待应答
	{
		while(get_current_tick() - wifi_at_time < waittime*SYS_TICK_PER_SECOND)	//等待倒计时
		{
            IWDG_ReloadCounter();
			delay_ms(10);
			if(strstr((char*)Uart2_RxBuf, (char*)ack) != NULL)
            {
                res = 0;
                break;
            }
		}
	}
	return res;
}

//wifi模块退出透传模式
//返回值:0,退出成功;
//       1,退出失败
uint8_t Wifi_Quit_Trans(void)
{
	USART2_SendData((uint8_t*)"+++", 3);
	delay_ms(500);
    Wifi_Send_Data((uint8_t*)"\r\n",(uint8_t*)"",0);
	return Wifi_Send_Data((uint8_t*)"AT\r\n",(uint8_t*)"OK",1);//退出透传判断.
}

/*******************************************************************************
* Function Name  : Wifi_Config
* Description    : 配置wifi模块
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int Wifi_Config(int ignore_flag)
{
    uint8_t connect_str[100];
    uint16_t dstblkOffset;
    nv_user_param_t *user_param;
    
    IWDG_ReloadCounter();
    Nvram_Get_BlkOffset_By_Name("user", &dstblkOffset);
    user_param = (nv_user_param_t *)(CFG_NVRAM_ADDR + dstblkOffset);
    IWDG_ReloadCounter();
    
    //1、退出透传模式
    if(Wifi_Quit_Trans() != 0)
        goto AT_CONFIG_ERR;
    
    //2、恢复模块出厂设置
    if(Wifi_Send_Data((uint8_t*)At_Command[0].cmd_str,(uint8_t*)At_Command[0].replay_str,At_Command[0].cmd_timeout) != 0)
        goto AT_CONFIG_ERR;
    
    delay_ms(1000);
    
    //3、设置STA模式
    if(Wifi_Send_Data((uint8_t*)At_Command[1].cmd_str,(uint8_t*)At_Command[1].replay_str,At_Command[1].cmd_timeout) != 0)
        goto AT_CONFIG_ERR;
    
    //4、设置动态分配IP地址
    if(Wifi_Send_Data((uint8_t*)At_Command[2].cmd_str,(uint8_t*)At_Command[2].replay_str,At_Command[2].cmd_timeout) != 0)
        goto AT_CONFIG_ERR;
    
    //5、设置连入的WIFI
    memset(connect_str, 0 ,sizeof(connect_str));
    sprintf((char*)connect_str, "%s=\"%s\",\"%s\"\r\n", At_Command[3].cmd_str, user_param->ssid, user_param->key);
    if(Wifi_Send_Data(connect_str,(uint8_t*)At_Command[3].replay_str,At_Command[3].cmd_timeout) != 0)
    {
        if(ignore_flag > 1)
            goto AT_CONFIG_ERR;
    }
    
    //6、设置开机自动连接WIFI
    if(Wifi_Send_Data((uint8_t*)At_Command[4].cmd_str,(uint8_t*)At_Command[4].replay_str,At_Command[4].cmd_timeout) != 0)
        goto AT_CONFIG_ERR;
    
    //7、设置连接的服务器
    memset(connect_str, 0 ,sizeof(connect_str));
    sprintf((char*)connect_str, "%s=1,\"%s\",%d,\"TCP\",500\r\n", At_Command[5].cmd_str, user_param->sip, user_param->port);
    if(Wifi_Send_Data(connect_str,(uint8_t*)At_Command[5].replay_str,At_Command[5].cmd_timeout) != 0)
        goto AT_CONFIG_ERR;

    return 0;
AT_CONFIG_ERR:
    return -1;
}











