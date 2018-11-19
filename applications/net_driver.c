
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

//��Wifiģ�鷢��ָ������
//data:���͵�����(����Ҫ��ӻس���)
//ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
//waittime:�ȴ�ʱ��(��λ:10ms)
//����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)
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
    USART2_SendData(data, strlen((char*)data));//��������
    wifi_at_time = get_current_tick();
    
	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(get_current_tick() - wifi_at_time < waittime*SYS_TICK_PER_SECOND)	//�ȴ�����ʱ
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

//wifiģ���˳�͸��ģʽ
//����ֵ:0,�˳��ɹ�;
//       1,�˳�ʧ��
uint8_t Wifi_Quit_Trans(void)
{
	USART2_SendData((uint8_t*)"+++", 3);
	delay_ms(500);
    Wifi_Send_Data((uint8_t*)"\r\n",(uint8_t*)"",0);
	return Wifi_Send_Data((uint8_t*)"AT\r\n",(uint8_t*)"OK",1);//�˳�͸���ж�.
}

/*******************************************************************************
* Function Name  : Wifi_Config
* Description    : ����wifiģ��
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
    
    //1���˳�͸��ģʽ
    if(Wifi_Quit_Trans() != 0)
        goto AT_CONFIG_ERR;
    
    //2���ָ�ģ���������
    if(Wifi_Send_Data((uint8_t*)At_Command[0].cmd_str,(uint8_t*)At_Command[0].replay_str,At_Command[0].cmd_timeout) != 0)
        goto AT_CONFIG_ERR;
    
    delay_ms(1000);
    
    //3������STAģʽ
    if(Wifi_Send_Data((uint8_t*)At_Command[1].cmd_str,(uint8_t*)At_Command[1].replay_str,At_Command[1].cmd_timeout) != 0)
        goto AT_CONFIG_ERR;
    
    //4�����ö�̬����IP��ַ
    if(Wifi_Send_Data((uint8_t*)At_Command[2].cmd_str,(uint8_t*)At_Command[2].replay_str,At_Command[2].cmd_timeout) != 0)
        goto AT_CONFIG_ERR;
    
    //5�����������WIFI
    memset(connect_str, 0 ,sizeof(connect_str));
    sprintf((char*)connect_str, "%s=\"%s\",\"%s\"\r\n", At_Command[3].cmd_str, user_param->ssid, user_param->key);
    if(Wifi_Send_Data(connect_str,(uint8_t*)At_Command[3].replay_str,At_Command[3].cmd_timeout) != 0)
    {
        if(ignore_flag > 1)
            goto AT_CONFIG_ERR;
    }
    
    //6�����ÿ����Զ�����WIFI
    if(Wifi_Send_Data((uint8_t*)At_Command[4].cmd_str,(uint8_t*)At_Command[4].replay_str,At_Command[4].cmd_timeout) != 0)
        goto AT_CONFIG_ERR;
    
    //7���������ӵķ�����
    memset(connect_str, 0 ,sizeof(connect_str));
    sprintf((char*)connect_str, "%s=1,\"%s\",%d,\"TCP\",500\r\n", At_Command[5].cmd_str, user_param->sip, user_param->port);
    if(Wifi_Send_Data(connect_str,(uint8_t*)At_Command[5].replay_str,At_Command[5].cmd_timeout) != 0)
        goto AT_CONFIG_ERR;

    return 0;
AT_CONFIG_ERR:
    return -1;
}











