/*
  Filename:       sx1278_lora.c
  Description:    Transplant official BSP
  Revised:        $Date: 2018-10-27  $
  Revision:       $Revision: ck $
 */

#include <string.h>
#include "sx1278_lora.h"
#include "Spi_Eval.h"

#define SX12781_DIO0_PORT    GPIOC
#define SX12781_DIO0_PIN     GPIO_Pin_6
#define SX12781_RESET_PORT   GPIOC
#define SX12781_RESET_PIN    GPIO_Pin_7

uint8_t Lora1RxBuffer[RF_DEFAULT_RXPACKET_LEN];    

// Default settings
const LoRaSettings_t LoRa1Settings =
{
    499300000,        // RFFrequency
	7,                // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                      // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
    20,               // Power
    7,                // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
	false,            // LowDatarateOptimize
    2,                // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
	8,                //PreambleLen
	0,                //FixLen
	64,               // PayloadLength (used for implicit header mode)
    true,             // CrcOn [0: OFF, 1: ON]
    0,                // FreqHopOn [0: OFF, 1: ON]
	0,                // HopPeriod Hops every frequency hopping period symbols
	true,             // IqInverted
    true,             // RxSingleOn [0: Single , 1 Continuous]
};

//SX12781 LoRa registers variable 
SX1276_t    SX12781;

/*******************************************************************/
// sx1278 LORA Functions 
void sx1278_1WriteData(uint8_t addr, uint8_t data);
void sx1278_1WriteBuf(uint8_t addr, uint8_t *buf, uint8_t size);
void sx1278_1ReadData(uint8_t addr, uint8_t *data);
void sx1278_1ReadBuf(uint8_t addr, uint8_t *buf, uint8_t size);
void sx1278_1PinInit(void);
//******************************************************************************
/* sx1278Lora Initialize/configuration/control functions */

void sx1278_1DelayMs(void)
{
	uint16_t i,j;
	
	for(i=0;i<50;i++)
	{
		for(j=0;j<50;j++);
	}
}
/* sx1278 Init */
uint8_t sx1278_1Init(void)
{
  	uint8_t i,def_val;
	
	RadioRegisters_t RadioRegsInit[] = RADIO_INIT_REGISTERS_VALUE;
	
	sx1278_1PinInit();
	
	Spi2_Configuration();
	
	// REMARK: See SX1276 datasheet for modified default values(0x12).
   	sx1278_1ReadData( REG_LR_VERSION, &def_val );
	if(REGVERSION_DEFAULT == def_val)
	{
		sx1278_1SetOpMode( RFLR_OPMODE_SLEEP );
		for( i = 0; i < sizeof( RadioRegsInit ) / sizeof( RadioRegisters_t ); i++ )
		{
			sx1278_1SetModem( RadioRegsInit[i].Modem );
			sx1278_1WriteData( RadioRegsInit[i].Addr, RadioRegsInit[i].value );
		}
		
		sx1278_1SetModem( MODEM_LORA );
		SX12781.Modem = MODEM_LORA;
		
		return true;
	}
	
    return false;		
}

/* Set Rf Parameters In Lora Mode */
void sx1278_1SetLoraPara(LoRaSettings_t *ptr)
{
	uint8_t paConfig = 0;
	uint8_t paDac = 0,Sf;
	uint8_t power;
	uint8_t bandwidth;
	uint8_t res;
	
    if(ptr == NULL)
		ptr = (LoRaSettings_t *)&LoRa1Settings;  
	
	sx1278_1SetRFChannel( ptr->Channel );
	sx1278_1SetModem( SX12781.Modem );
	
    sx1278_1ReadData( REG_PACONFIG, &paConfig); 
    sx1278_1ReadData( REG_PADAC, &paDac); 
    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | RF_PACONFIG_PASELECT_PABOOST;
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;
	
	power = ptr->Power;
	if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
		if( power > 17 )
        {
        	paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
		
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
	
	sx1278_1WriteData( REG_PACONFIG, paConfig );
	sx1278_1WriteData( REG_PADAC, paDac );
	SX12781.LoRa.Power = power;
	
	if(ptr->Sf < 6)	
	{
		ptr->Sf = 6;  	
	}
	else if(ptr->Sf >12)
	{
		ptr->Sf = 12;
	}
	
	Sf = ptr->Sf;
	
	SX12781.LoRa.Bandwidth 		= ptr->Bandwidth;
	SX12781.LoRa.Sf        		= ptr->Sf;
	SX12781.LoRa.Coderate  		= ptr->Coderate;
	SX12781.LoRa.PreambleLen 	= ptr->PreambleLen;
	SX12781.LoRa.FixLen      	= ptr->FixLen;
	SX12781.LoRa.FreqHopOn   	= ptr->FreqHopOn;
	SX12781.LoRa.HopPeriod   	= ptr->HopPeriod;
	SX12781.LoRa.CrcOn       	= ptr->CrcOn;
	SX12781.LoRa.IqInverted  	= ptr->IqInverted;
	SX12781.LoRa.RxContinuous 	= ptr->RxContinuous;
	
	bandwidth = SX12781.LoRa.Bandwidth;
	if( ( ( bandwidth == 7 ) && ( ( Sf == 11 ) || ( Sf == 12 ) ) ) ||
		( ( bandwidth == 8 ) && ( Sf == 12 ) ) )
	{
		SX12781.LoRa.LowDatarateOptimize = 0x01;
	}
	else
	{
		SX12781.LoRa.LowDatarateOptimize = 0x00;
	}
	
	if( SX12781.LoRa.FreqHopOn == true )
	{
	    sx1278_1ReadData( REG_LR_PLLHOP, &res);
		sx1278_1WriteData( REG_LR_PLLHOP, ( res & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
		sx1278_1WriteData( REG_LR_HOPPERIOD, SX12781.LoRa.HopPeriod );
	}
	
	sx1278_1ReadData( REG_LR_MODEMCONFIG1, &res);
	sx1278_1WriteData( REG_LR_MODEMCONFIG1, ( res & RFLR_MODEMCONFIG1_BW_MASK & RFLR_MODEMCONFIG1_CODINGRATE_MASK &
				   		RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( SX12781.LoRa.Bandwidth << 4 ) | ( SX12781.LoRa.Coderate << 1 ) | SX12781.LoRa.FixLen );
	
	sx1278_1ReadData( REG_LR_MODEMCONFIG2, &res);
	sx1278_1WriteData( REG_LR_MODEMCONFIG2, ( res & RFLR_MODEMCONFIG2_SF_MASK & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) |
					 	( SX12781.LoRa.Sf << 4 ) | ( SX12781.LoRa.CrcOn  << 2 ) );	
	
	sx1278_1ReadData( REG_LR_MODEMCONFIG3, &res);
	sx1278_1WriteData( REG_LR_MODEMCONFIG3,( res & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
				 	  	( SX12781.LoRa.LowDatarateOptimize << 3 ) );	
	
	sx1278_1WriteData( REG_LR_PREAMBLEMSB, ( SX12781.LoRa.PreambleLen >> 8 ) & 0x00FF );
	sx1278_1WriteData( REG_LR_PREAMBLELSB, SX12781.LoRa.PreambleLen & 0xFF );

	if( ( bandwidth == 9 ) && ( SX12781.LoRa.Channel > RF_MID_BAND_THRESH ))
	{
		sx1278_1WriteData( REG_LR_TEST36, 0x02 );
		sx1278_1WriteData( REG_LR_TEST3A, 0x64 );
	}
	else if( bandwidth == 9 )
	{
		sx1278_1WriteData( REG_LR_TEST36, 0x02 );
		sx1278_1WriteData( REG_LR_TEST3A, 0x7F );
	}
	else
	{
		sx1278_1WriteData( REG_LR_TEST36, 0x03 );
	}
	
	if( Sf == 6 )
	{
	  	sx1278_1ReadData( REG_LR_DETECTOPTIMIZE, &res);
		sx1278_1WriteData( REG_LR_DETECTOPTIMIZE,( res & RFLR_DETECTIONOPTIMIZE_MASK ) | RFLR_DETECTIONOPTIMIZE_SF6 );
		
		sx1278_1WriteData( REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF6 );
	}
	else
	{
	  	sx1278_1ReadData( REG_LR_DETECTOPTIMIZE, &res);
		sx1278_1WriteData( REG_LR_DETECTOPTIMIZE,( res & RFLR_DETECTIONOPTIMIZE_MASK ) | RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
		sx1278_1WriteData( REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
	}
}

/* RF TX configuration */
void sx1278_1SendBuf(uint8_t *pkt, uint8_t pkt_size)
{
	uint8_t res;   
  
	if( NULL == pkt )
	  return;
	
	if( SX12781.LoRa.IqInverted == true )
	{
	  	sx1278_1ReadData( REG_LR_INVERTIQ, &res);
		sx1278_1WriteData( REG_LR_INVERTIQ, ( ( res & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
		sx1278_1WriteData( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
	}
	else
	{
	  	sx1278_1ReadData( REG_LR_INVERTIQ, &res);
		sx1278_1WriteData( REG_LR_INVERTIQ, ( ( res & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
		sx1278_1WriteData( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
	}

	sx1278_1WriteData( REG_LR_PAYLOADLENGTH, pkt_size );
	sx1278_1WriteData( REG_LR_FIFOTXBASEADDR, 0 );
	sx1278_1WriteData( REG_LR_FIFOADDRPTR, 0 );
	
	sx1278_1ReadData(REG_OPMODE, &res);
	if( ( res & ~RF_OPMODE_MASK ) == RF_OPMODE_SLEEP )
	{
		sx1278_1SetOpMode( RFLR_OPMODE_STANDBY );
		sx1278_1DelayMs();
	}
	
	sx1278_1WriteBuf(0, pkt, pkt_size);
	
	sx1278_1WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_ALL);
	sx1278_1WriteData( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT | RFLR_IRQFLAGS_RXDONE |
						RFLR_IRQFLAGS_PAYLOADCRCERROR | RFLR_IRQFLAGS_VALIDHEADER | 
						RFLR_IRQFLAGS_CADDONE |RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | RFLR_IRQFLAGS_CADDETECTED );
	
	sx1278_1ReadData( REG_DIOMAPPING1, &res);
	sx1278_1WriteData( REG_DIOMAPPING1, ( res & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
	sx1278_1ReadData( REG_DIOMAPPING2, &res);
	sx1278_1WriteData( REG_DIOMAPPING2, ( res & RFLR_DIOMAPPING2_DIO4_MASK ) | RFLR_DIOMAPPING2_DIO4_00 );
	
    sx1278_1SetOpMode( RF_OPMODE_TRANSMITTER );   
	
	sx1278_1ReadData(REG_LR_IRQFLAGS, &res); 
	SX12781.State = RF_TX_RUNNING;
}

/* RF RX configuration */
void sx1278_1EnterRx(void)
{
    uint8_t rxContinuous = false;
	uint8_t res;
	
	if( SX12781.LoRa.IqInverted == true )
	{
	    sx1278_1ReadData( REG_LR_INVERTIQ, &res);
		sx1278_1WriteData( REG_LR_INVERTIQ, ( ( res & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF ) );
		sx1278_1WriteData( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
	}
	else
	{
	    sx1278_1ReadData( REG_LR_INVERTIQ, &res);
		sx1278_1WriteData( REG_LR_INVERTIQ, ( ( res & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
		sx1278_1WriteData( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
	}
	
	if( SX12781.LoRa.Bandwidth < 9 )
	{
	  	sx1278_1ReadData( REG_LR_DETECTOPTIMIZE, &res);
		sx1278_1WriteData( REG_LR_DETECTOPTIMIZE, res & 0x7F );
		sx1278_1WriteData( REG_LR_TEST30, 0x00 );
		
		switch( SX12781.LoRa.Bandwidth )
		{
			case 0:                     sx1278_1WriteData( REG_LR_TEST2F, 0x48 );
				sx1278_1SetRFChannel(SX12781.LoRa.Channel + (uint32_t)(7.81e3 ));
			break;
			
			case 1:                     sx1278_1WriteData( REG_LR_TEST2F, 0x44 );
				sx1278_1SetRFChannel(SX12781.LoRa.Channel + (uint32_t)(10.42e3 ));
			break;
			
			case 2:                     sx1278_1WriteData( REG_LR_TEST2F, 0x44 );
				sx1278_1SetRFChannel(SX12781.LoRa.Channel + (uint32_t)(15.62e3 ));
			break;
			
			case 3:                     sx1278_1WriteData( REG_LR_TEST2F, 0x44 );
				sx1278_1SetRFChannel(SX12781.LoRa.Channel + (uint32_t)(20.83e3 ));
			break;
			
			case 4:                     sx1278_1WriteData( REG_LR_TEST2F, 0x44 );
				sx1278_1SetRFChannel(SX12781.LoRa.Channel + (uint32_t)(31.25e3 ));
			break;
			
			case 5:                     sx1278_1WriteData( REG_LR_TEST2F, 0x44 );
				sx1278_1SetRFChannel(SX12781.LoRa.Channel + (uint32_t)(41.67e3 ));
			break;
			
			case 6:                     sx1278_1WriteData( REG_LR_TEST2F, 0x40 );
				break;
				
			case 7:                     sx1278_1WriteData( REG_LR_TEST2F, 0x40 );
				break;
				
			case 8:                     sx1278_1WriteData( REG_LR_TEST2F, 0x40 );
				break;
		}
	}
	else
	{
	  	sx1278_1ReadData( REG_LR_DETECTOPTIMIZE, &res);
		sx1278_1WriteData( REG_LR_DETECTOPTIMIZE, res | 0x80 );
	}
	
	sx1278_1WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_ALL);
	rxContinuous = SX12781.LoRa.RxContinuous;
	if( SX12781.LoRa.FreqHopOn == true )
	{
		sx1278_1WriteData( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |  RFLR_IRQFLAGS_VALIDHEADER |
						  RFLR_IRQFLAGS_TXDONE | RFLR_IRQFLAGS_CADDONE | RFLR_IRQFLAGS_CADDETECTED );
		
		sx1278_1ReadData( REG_DIOMAPPING1, &res);
		sx1278_1WriteData( REG_DIOMAPPING1, ( res & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00);
		
		sx1278_1ReadData( REG_DIOMAPPING2, &res);
		sx1278_1WriteData( REG_DIOMAPPING2, ( res & RFLR_DIOMAPPING2_DIO4_MASK ) | RFLR_DIOMAPPING2_DIO4_10 );
	}
	else
	{
		sx1278_1WriteData( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |  RFLR_IRQFLAGS_VALIDHEADER | 
						 RFLR_IRQFLAGS_TXDONE | RFLR_IRQFLAGS_CADDONE | RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL | RFLR_IRQFLAGS_CADDETECTED );
		
		sx1278_1ReadData( REG_DIOMAPPING1, &res);
		sx1278_1WriteData( REG_DIOMAPPING1, ( res & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO3_10);
		
		sx1278_1ReadData( REG_DIOMAPPING2, &res);
		sx1278_1WriteData( REG_DIOMAPPING2, ( res & RFLR_DIOMAPPING2_DIO4_MASK ) | RFLR_DIOMAPPING2_DIO4_10 );
	}
	
	sx1278_1WriteData( REG_LR_FIFORXBASEADDR, 0 );
	sx1278_1WriteData( REG_LR_FIFOADDRPTR, 0 );

    if( rxContinuous == true )
    {
        sx1278_1SetOpMode( RFLR_OPMODE_RECEIVER );
    }
    else
    {
        sx1278_1SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
    }
	
	sx1278_1ReadData(REG_LR_IRQFLAGS, &res); 
	SX12781.State = RF_RX_RUNNING;
	SX12781.Size  = 0;
}

/* Set The SX1278 Modulation Mode */
void sx1278_1SetModem( RadioModems_t modem )
{
    uint8_t 	res;
	
    SX12781.Modem = modem;
	sx1278_1SetOpMode( RF_OPMODE_SLEEP );
	sx1278_1ReadData( REG_OPMODE, &res); 
	sx1278_1WriteData( REG_OPMODE, ( res & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON );
	sx1278_1WriteData( REG_DIOMAPPING1, 0x00 );        
	sx1278_1WriteData( REG_DIOMAPPING2, 0x00 );
}

/* Set SX1278 Mode */
void sx1278_1SetOpMode(uint8_t opMode)
{
    uint8_t res;
	
	sx1278_1ReadData( REG_OPMODE, &res);
		
    sx1278_1WriteData(REG_LR_OPMODE, (res & RF_OPMODE_MASK) | opMode);    
}

/* Set RF Channel */
void sx1278_1SetRFChannel( uint32_t freq )
{	
    SX12781.LoRa.Channel = freq;
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
	sx1278_1WriteData( REG_LR_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ));
	sx1278_1WriteData( REG_LR_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ));
	sx1278_1WriteData( REG_LR_FRFLSB, ( uint8_t )( freq & 0xFF ));
}

/* Check If The Channel Is Free */
uint8_t sx1278_1IsChannelFree( RadioModems_t modem, uint32_t freq, int16_t rssiThresh )
{
    int16_t rssi = 0;
    uint8_t i;
	
    sx1278_1SetModem( modem );
    sx1278_1SetRFChannel( freq );
    sx1278_1SetOpMode( RF_OPMODE_RECEIVER );
	
    for( i=0; i<10; i++ )
    {
        sx1278_1DelayMs();
		
        rssi += sx1278_1ReadRssi( modem );
    }
	   
    sx1278_1SetOpMode( RFLR_OPMODE_SLEEP );
	
    rssi = rssi/10;
    if( rssi > rssiThresh )
    {
        return false;
    }
    return true;
}

/* Read SX1278's Rssi */
int16_t sx1278_1ReadRssi( RadioModems_t modem )
{
    uint8_t res;
    int16_t rssi = 0;
	
	sx1278_1ReadData( REG_LR_RSSIVALUE, &res);	
	rssi = (int16_t)(RSSI_OFFSET_LF + res);
	
    return rssi;
}

/* Set Payload Length */
void sx1278_1SetMaxPayloadLength( RadioModems_t modem, uint8_t max )
{
    sx1278_1SetModem( modem );
	
    sx1278_1WriteData( REG_LR_PAYLOADMAXLENGTH, max );
}

/* Read SX1278 Fifo Data And Resolve Rssi,SNR */
void sx1278_1ReadRxPkt(void)
{
	uint8_t irqFlags;
	int16_t rssi,snr;
	uint8_t	payload_len;
	
	SX12781.Size = 0;
	sx1278_1WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );
	sx1278_1ReadData( REG_LR_IRQFLAGS, &irqFlags);

	if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
	{
		sx1278_1WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );
		sx1278_1WriteData(REG_LR_FIFOADDRPTR, 0);
		SX12781.Size = 0;
		return;
	}

	sx1278_1ReadData( REG_LR_PKTSNRVALUE, (uint8_t *)&SX12781.SnrValue);
	if( SX12781.SnrValue & 0x80 ) 	
	{
		snr = ( ( ~SX12781.SnrValue + 1 ) & 0xFF ) >> 2;
		snr = -snr;
	}
	else
	{
		snr = ( SX12781.SnrValue & 0xFF ) >> 2;
	}
	SX12781.SnrValue = snr;
	
	sx1278_1ReadData( REG_LR_PKTRSSIVALUE, (uint8_t *)&rssi);
	if( snr < 0 )
	{
		SX12781.RssiValue = (int16_t)(RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) + snr);
	}
	else
	{
		SX12781.RssiValue = (int16_t)(RSSI_OFFSET_LF + rssi + ( rssi >> 4 ));
	}

	sx1278_1ReadData( REG_LR_RXNBBYTES, &payload_len);
	
	if(RF_DEFAULT_RXPACKET_LEN < payload_len)
	{
		sx1278_1WriteData(REG_LR_FIFOADDRPTR, 0);
		payload_len = 0;
		SX12781.Size = 0;
		return;
	}
	
	sx1278_1ReadBuf(0, Lora1RxBuffer, payload_len);
	
	sx1278_1SetSleep();
	delay_ms(5);
	sx1278_1EnterRx();
	
	SX12781.Size = payload_len;
}

/* Clear TX_IRQ */
void sx1278_1TxDoneCallback(void)
{
	sx1278_1WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );
}

void sx1278_1SetSleep(void)
{
	sx1278_1SetOpMode( RFLR_OPMODE_SLEEP );
	SX12781.State = RF_IDLE;
}

void sx1278_1SetStby(void)
{
	sx1278_1SetOpMode( RFLR_OPMODE_STANDBY );
	SX12781.State = RF_IDLE;
}

/* Get RX_Data */
void *sx1278_1GetRxData(uint8_t *size)
{
    if(size == NULL)
	  return NULL;
	
	if(RF_RX_RUNNING == SX12781.State)
	{
	 	if(SX12781.Size != 0)
		{
			*size = SX12781.Size;	
			SX12781.Size = 0;
		    return (void *)Lora1RxBuffer;
		}	   
	}
	
	return NULL;
}

void sx1278_1PinInit(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = SX12781_DIO0_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  		
	GPIO_Init(SX12781_DIO0_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SX12781_RESET_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SX12781_RESET_PORT, &GPIO_InitStructure);	
	
	GPIO_ResetBits(SX12781_RESET_PORT, SX12781_RESET_PIN);
	delay_ms(50);
	GPIO_SetBits(SX12781_RESET_PORT, SX12781_RESET_PIN);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);
	
	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		
}

uint8_t Read_sx1278_1Dio0_Pin(void)
{
	return GPIO_ReadInputDataBit(SX12781_DIO0_PORT, SX12781_DIO0_PIN);
}

//******************************************************************************
/* sx1278 Read and write register functions */
void  sx1278_1WriteData(uint8_t addr, uint8_t data)
{	
	uint8_t wAddres;
	
	wAddres = addr | 0x80;
	
	Spi2_AssertCSN();
	Spi2_ReadWriteByte(wAddres);
 	Spi2_ReadWriteByte(data);
	Spi2_DeAssertCSN();	
}

void  sx1278_1WriteBuf(uint8_t addr, uint8_t *buf, uint8_t size)
{	 
	uint8_t wAddres;		
	
	if(size > RF_DEFAULT_RXPACKET_LEN || size <= 0)
		return;
	
	wAddres = addr | 0x80;
			
	Spi2_AssertCSN();
	Spi2_ReadWriteByte(wAddres);
	while(size --)
	{
		Spi2_ReadWriteByte(*buf);
		buf ++;
	}
	Spi2_DeAssertCSN();		
}

void  sx1278_1ReadData(uint8_t addr, uint8_t *data)
{   
	uint8_t rAddres;
	
	rAddres = addr & 0x7F;  
	
	Spi2_AssertCSN();
	Spi2_ReadWriteByte(rAddres);
 	*data = Spi2_ReadWriteByte(0xFF);
	Spi2_DeAssertCSN();
}

void  sx1278_1ReadBuf(uint8_t addr, uint8_t *buf, uint8_t size)
{
	uint8_t rAddres;
	
	if(size > RF_DEFAULT_RXPACKET_LEN || size <= 0)
		return;
	
   	rAddres = addr & 0x7F;  
	
	Spi2_AssertCSN();
	Spi2_ReadWriteByte(rAddres);
	while(size --)
	{
		*buf = Spi2_ReadWriteByte(0xFF);
		buf ++;
	}
	Spi2_DeAssertCSN();
}
