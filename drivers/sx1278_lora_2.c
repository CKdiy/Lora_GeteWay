/*
  Filename:       sx1278_lora.c
  Description:    Transplant official BSP
  Revised:        $Date: 2018-10-27  $
  Revision:       $Revision: ck $
 */

#include <string.h>
#include "sx1278_lora.h"
#include "Spi_Eval.h"
  
/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF_2                              -155.0
#define RSSI_OFFSET_HF_2                              -150.0

#define NOISE_ABSOLUTE_ZERO_2                         -174.0

#define NOISE_FIGURE_LF_2                                4.0
#define NOISE_FIGURE_HF_2                                6.0 
/*!
 * Precomputed signal bandwidth log values
 * Used to compute the Packet RSSI value.
 */
extern const double SignalBwLog[];

extern const double RssiOffsetLF[];

extern const double RssiOffsetHF[];

#define REGVERSION_DEFAULT_2   0x12 
#define RF_PACKET_LEN_2         1<<6

#define SX12782_DIO0_PORT    GPIOB
#define SX12782_DIO0_PIN     GPIO_Pin_10
#define SX12782_RESET_PORT   GPIOB
#define SX12782_RESET_PIN    GPIO_Pin_11

static uint8_t RF2Buffer[RF_PACKET_LEN_2];

//tRadioDriver *sx1278Radio = NULL;

static bool LoRaOn = false;
static bool LoRaOnState = false;
static uint8_t RFLRState = RFLR_STATE_IDLE;
//static int8_t RxPacketSnrEstimate;
//static double RxPacketRssiValue;
static uint16_t RxPacketSize = 0;

//SX1278 registers variable 
static uint8_t SX1278Regs[0x70];

//Frequency hopping frequencies table
extern const int32_t HoppingFrequencies[];

// Default settings
tLoRaSettings LoRa2Settings =
{
    434000000,        // RFFrequency
    20,                // Power
    7,                // SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,
                      // 5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
    10,                // SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    2,                // ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    true,             // CrcOn [0: OFF, 1: ON]
    false,            // ImplicitHeaderOn [0: OFF, 1: ON]
    0,                // RxSingleOn [0: Continuous, 1 Single]
    0,                // FreqHopOn [0: OFF, 1: ON]
    4,                // HopPeriod Hops every frequency hopping period symbols
    100,              // TxPacketTimeout
    100,              // RxPacketTimeout
    64,              // PayloadLength (used for implicit header mode)
};

//SX1278 LoRa registers variable 
static tSX1276LR* SX1278LR_2;

/*******************************************************************/
// sx1278 LORA Functions 
void sx1278_2SetLoRaOn(bool enable);
bool sx1278_2GetLoRaOn(void);
void sx1278_2WriteData(uint8_t addr, uint8_t data);
void sx1278_2WriteBuf(uint8_t addr, uint8_t *buf, uint8_t size);
void sx1278_2ReadData(uint8_t addr, uint8_t *data);
void sx1278_2ReadBuf(uint8_t addr, uint8_t *buf, uint8_t size);

void sx1278Lora_2SetRFFrequency( uint32_t freq );
void sx1278Lora_2SetOpMode(uint8_t opMode);
void sx1278Lora_2SetSpreadingFactor( uint8_t factor );
void sx1278Lora_2SetErrorCoding( uint8_t value );
void sx1278Lora_2SetPacketCrcOn( bool enable );
void sx1278Lora_2SetSignalBandwidth( uint8_t bw );
void sx1278Lora_2SetImplicitHeaderOn( bool enable );
void sx1278Lora_2SetSymbTimeout( uint16_t value );
void sx1278Lora_2SetPayloadLength( uint8_t value );
void sx1278Lora_2SetLowDatarateOptimize( bool enable );
void sx1278Lora_2SetPAOutput( uint8_t outputPin );
void sx1278Lora_2SetPa20dBm( bool enale );
void sx1278Lora_2SetRFPower( int8_t power );
void sx1278Lora_2SetRFMode(bool mode);
uint8_t sx1278Lora_2GetOpMode( void );
bool sx1278Lora_2RFSendBuf( uint8_t *txBuf, size_t txLen);
tRFLRStates  sx1278Lora_2Process(void);
static void sx1278Lora_2SetParameters(void);
void sx1278_2PinInit(void);
uint8_t Read_sx1278_2Dio0_Pin(void);
//******************************************************************************
/* sx1278Lora Initialize/configuration/control functions */
void sx1278_2Init(void)
{
	SX1278LR_2 = (tSX1276LR *)SX1278Regs;
	
	Spi1_Configuration();
	sx1278_2PinInit();
	
	LoRaOn = true;
	sx1278_2SetLoRaOn(LoRaOn);  
}

void sx1278_2SetLoRaOn(bool enable)
{
	if(LoRaOnState == enable)
    	return;
    
    LoRaOnState = enable;
    LoRaOn      = enable;
	
	if( LoRaOn == true )
    {
	  	
	    // REMARK: See SX1276 datasheet for modified default values(0x12).
   	 	sx1278_2ReadData( REG_LR_VERSION, &SX1278LR_2->RegVersion );
	
		//Just for testing SPI
   		if(SX1278LR_2->RegVersion != REGVERSION_DEFAULT_2)
			return; 
		
    	sx1278Lora_2SetOpMode(RFLR_OPMODE_SLEEP);
        
		delay_ms(10);
		
		SX1278LR_2->RegTcxo = 0x09;   //USE TCXO
		
		sx1278_2WriteData(REG_LR_TCXO, SX1278LR_2->RegTcxo);
		
        SX1278LR_2->RegOpMode = ( SX1278LR_2->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_LOWFREQUENCYMODEON;
		
		//select lora mode
		sx1278_2WriteData(REG_LR_OPMODE, SX1278LR_2->RegOpMode);
        
		sx1278Lora_2SetRFFrequency(LoRa2Settings.RFFrequency);
		
		if(LoRa2Settings.RFFrequency > 200000000)
		{
			sx1278Lora_2SetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST );
    		sx1278Lora_2SetPa20dBm( true );
			LoRa2Settings.Power = 20;
    		sx1278Lora_2SetRFPower( LoRa2Settings.Power );
		}
		else
		{
			sx1278Lora_2SetPAOutput( RFLR_PACONFIG_PASELECT_RFO );
    		sx1278Lora_2SetPa20dBm( false );
			LoRa2Settings.Power = 14;
    		sx1278Lora_2SetRFPower( LoRa2Settings.Power );		
		}
		
		//RegOcp,Close Ocp		
		SX1278LR_2->RegOcp = 0x0B;
		sx1278_2WriteData(REG_LR_OCP, SX1278LR_2->RegOcp);
		
		//RegLNA,High & LNA Enable
		SX1278LR_2->RegLna = 0x23;
		sx1278_2WriteData(REG_LR_LNA, SX1278LR_2->RegLna);
		
		sx1278Lora_2SetParameters();
		
		sx1278Lora_2SetSymbTimeout( 0x3FF );	
		
		SX1278LR_2->RegPreambleMsb = 0;
		SX1278LR_2->RegPreambleLsb = 16;
		sx1278_2WriteData(REG_LR_PREAMBLEMSB, SX1278LR_2->RegPreambleMsb);
		sx1278_2WriteData(REG_LR_PREAMBLELSB, SX1278LR_2->RegPreambleLsb); 				
		
		//RegDioMapping2 DIO5=00, DIO4=01
		SX1278LR_2->RegDioMapping2 = 0x01;
		sx1278_2WriteData(REG_LR_DIOMAPPING2, SX1278LR_2->RegDioMapping2);
		sx1278Lora_2SetOpMode(RFLR_OPMODE_STANDBY);
    }
    else
    {
        sx1278Lora_2SetOpMode( RFLR_OPMODE_SLEEP );
        
        SX1278LR_2->RegOpMode = ( SX1278LR_2->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF;
        sx1278_2WriteData(REG_LR_OPMODE, SX1278LR_2->RegOpMode);
        
        sx1278Lora_2SetOpMode( RFLR_OPMODE_STANDBY );  
    }
	
	RFLRState = RFLR_STATE_RX_INIT;
}

static void sx1278Lora_2SetParameters(void)
{
	// SF6 only operates in implicit header mode.
	sx1278Lora_2SetSpreadingFactor(LoRa2Settings.SpreadingFactor); 
	
	sx1278Lora_2SetErrorCoding( LoRa2Settings.ErrorCoding );	
	sx1278Lora_2SetPacketCrcOn( LoRa2Settings.CrcOn );
	sx1278Lora_2SetSignalBandwidth( LoRa2Settings.SignalBw );
	sx1278Lora_2SetImplicitHeaderOn( LoRa2Settings.ImplicitHeaderOn );
    sx1278Lora_2SetPayloadLength( LoRa2Settings.PayloadLength );
    sx1278Lora_2SetLowDatarateOptimize( true );	 
}

void sx1278Lora_2EntryTx(void)
{	  
	sx1278Lora_2SetOpMode( RFLR_OPMODE_STANDBY );
	if( LoRa2Settings.FreqHopOn == true )
    {
    	SX1278LR_2->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    //RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1278LR_2->RegHopPeriod = LoRa2Settings.HopPeriod;

        sx1278_2ReadData( REG_LR_HOPCHANNEL, &SX1278LR_2->RegHopChannel );
        sx1278Lora_2SetRFFrequency( HoppingFrequencies[SX1278LR_2->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
    }
    else
    {
        SX1278LR_2->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    //RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1278LR_2->RegHopPeriod = 0;
    }
	
	sx1278_2WriteData( REG_LR_HOPPERIOD, SX1278LR_2->RegHopPeriod );
	
	                                    // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
    SX1278LR_2->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
    SX1278LR_2->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
    sx1278_2WriteBuf( REG_LR_DIOMAPPING1, &SX1278LR_2->RegDioMapping1, 2 );
	
	//Clear all irq
	sx1278_2WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_ALL);
    sx1278_2WriteData( REG_LR_IRQFLAGSMASK, SX1278LR_2->RegIrqFlagsMask );
	
	// Full buffer used for Tx
	SX1278LR_2->RegFifoTxBaseAddr = 0x00; 
    sx1278_2WriteData( REG_LR_FIFOTXBASEADDR, SX1278LR_2->RegFifoTxBaseAddr );
	
	SX1278LR_2->RegFifoAddrPtr = SX1278LR_2->RegFifoTxBaseAddr;
    sx1278_2WriteData( REG_LR_FIFOADDRPTR, SX1278LR_2->RegFifoAddrPtr );	
}

void sx1278Lora_2EntryRx(void)
{
	sx1278Lora_2SetOpMode( RFLR_OPMODE_STANDBY );

	SX1278LR_2->RegIrqFlagsMask = //RFLR_IRQFLAGS_RXTIMEOUT |
                                //RFLR_IRQFLAGS_RXDONE |
                                RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                RFLR_IRQFLAGS_VALIDHEADER |
                                RFLR_IRQFLAGS_TXDONE |
                                RFLR_IRQFLAGS_CADDONE |
                                RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                RFLR_IRQFLAGS_CADDETECTED;
	
	sx1278_2WriteData( REG_LR_IRQFLAGSMASK, SX1278LR_2->RegIrqFlagsMask );

	if( LoRa2Settings.FreqHopOn == true )
    {
		SX1278LR_2->RegHopPeriod = LoRa2Settings.HopPeriod;

		sx1278_2ReadData( REG_LR_HOPCHANNEL, &SX1278LR_2->RegHopChannel );
		sx1278Lora_2SetRFFrequency( HoppingFrequencies[SX1278LR_2->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
	}
	else
    {
		SX1278LR_2->RegHopPeriod = 255;
	}
        
    sx1278_2WriteData( REG_LR_HOPPERIOD, SX1278LR_2->RegHopPeriod );
                
                                   // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
    SX1278LR_2->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                   // CadDetected               ModeReady
    SX1278LR_2->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
    sx1278_2WriteBuf( REG_LR_DIOMAPPING1, &SX1278LR_2->RegDioMapping1, 2 );
    
    if( LoRa2Settings.RxSingleOn == true ) // Rx single mode
    {
        sx1278Lora_2SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
    }
    else // Rx continuous mode
    {
		// Full buffer used for Rx
		SX1278LR_2->RegFifoRxBaseAddr = 0x00;
        SX1278LR_2->RegFifoAddrPtr = SX1278LR_2->RegFifoRxBaseAddr;
        sx1278_2WriteData( REG_LR_FIFOADDRPTR, SX1278LR_2->RegFifoAddrPtr );
            
        sx1278Lora_2SetOpMode( RFLR_OPMODE_RECEIVER );
    }
	
    sx1278_2ReadData(REG_LR_IRQFLAGS, &SX1278LR_2->RegIrqFlags); 
    
    memset( RF2Buffer, 0, ( size_t )RF_PACKET_LEN_2 );
}


void sx1278Lora_2SetOpMode(uint8_t opMode)
{
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = true;
    bool antennaSwitchTxOn = false;

    opModePrev = SX1278LR_2->RegOpMode & ~RFLR_OPMODE_MASK;

    if(opMode != opModePrev)
    {
        if((opMode & 0x03) == RFLR_OPMODE_TRANSMITTER)
        {
            antennaSwitchTxOn = true;
        }
        else
        {
            antennaSwitchTxOn = false;
        }
        if(antennaSwitchTxOn != antennaSwitchTxOnPrev)
        {
            antennaSwitchTxOnPrev = antennaSwitchTxOn;
			
			// Antenna switch control
			if(antennaSwitchTxOn)
			{
			  //RX/TX 
			  sx1278Lora_2SetRFMode(antennaSwitchTxOn);
			}
        }
        SX1278LR_2->RegOpMode = ( SX1278LR_2->RegOpMode & RFLR_OPMODE_MASK ) | opMode;
		
        sx1278_2WriteData(REG_LR_OPMODE, SX1278LR_2->RegOpMode); 
    }
}

uint8_t sx1278Lora_2GetOpMode( void )
{
    sx1278_2ReadData( REG_LR_OPMODE, &SX1278LR_2->RegOpMode );
    
    return SX1278LR_2->RegOpMode & ~RFLR_OPMODE_MASK;
}

void sx1278Lora_2SetRFFrequency( uint32_t freq )
{
    LoRa2Settings.RFFrequency = freq;

    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1278LR_2->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1278LR_2->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1278LR_2->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    sx1278_2WriteBuf( REG_LR_FRFMSB, &SX1278LR_2->RegFrfMsb, 3 );
}

void sx1278Lora_2SetSpreadingFactor( uint8_t factor )
{
    if( factor > 12 )
    {
        factor = 12;
    }
    else if( factor < 6 )
    {
        factor = 6;
    }

    sx1278_2ReadData( REG_LR_MODEMCONFIG2, &SX1278LR_2->RegModemConfig2 );    
    SX1278LR_2->RegModemConfig2 = ( SX1278LR_2->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
    sx1278_2WriteData( REG_LR_MODEMCONFIG2, SX1278LR_2->RegModemConfig2 );  
    LoRa2Settings.SpreadingFactor = factor;
}

void sx1278Lora_2SetErrorCoding( uint8_t value )
{
    sx1278_2ReadData( REG_LR_MODEMCONFIG1, &SX1278LR_2->RegModemConfig1 );
    SX1278LR_2->RegModemConfig1 = ( SX1278LR_2->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
    sx1278_2WriteData( REG_LR_MODEMCONFIG1, SX1278LR_2->RegModemConfig1 );
    LoRa2Settings.ErrorCoding = value;
}

void  sx1278Lora_2SetPacketCrcOn( bool enable )
{
    sx1278_2ReadData( REG_LR_MODEMCONFIG2, &SX1278LR_2->RegModemConfig2 );
    SX1278LR_2->RegModemConfig2 = ( SX1278LR_2->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
    sx1278_2WriteData( REG_LR_MODEMCONFIG2, SX1278LR_2->RegModemConfig2 );
    LoRa2Settings.CrcOn = enable;
}

void sx1278Lora_2SetSignalBandwidth( uint8_t bw )
{
    sx1278_2ReadData( REG_LR_MODEMCONFIG1, &SX1278LR_2->RegModemConfig1 );
    SX1278LR_2->RegModemConfig1 = ( SX1278LR_2->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
    sx1278_2WriteData( REG_LR_MODEMCONFIG1, SX1278LR_2->RegModemConfig1 );
    LoRa2Settings.SignalBw = bw;
}

void sx1278Lora_2SetImplicitHeaderOn( bool enable )
{
    sx1278_2ReadData( REG_LR_MODEMCONFIG1, &SX1278LR_2->RegModemConfig1 );
    SX1278LR_2->RegModemConfig1 = ( SX1278LR_2->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
    sx1278_2WriteData( REG_LR_MODEMCONFIG1, SX1278LR_2->RegModemConfig1 );
    LoRa2Settings.ImplicitHeaderOn = enable;
}

void sx1278Lora_2SetSymbTimeout( uint16_t value )
{
    sx1278_2ReadBuf( REG_LR_MODEMCONFIG2, &SX1278LR_2->RegModemConfig2, 2 );

    SX1278LR_2->RegModemConfig2 = ( SX1278LR_2->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    SX1278LR_2->RegSymbTimeoutLsb = value & 0xFF;
    sx1278_2WriteBuf( REG_LR_MODEMCONFIG2, &SX1278LR_2->RegModemConfig2, 2 );
}

void sx1278Lora_2SetPayloadLength( uint8_t value )
{
    SX1278LR_2->RegPayloadLength = value;
    sx1278_2WriteData( REG_LR_PAYLOADLENGTH, SX1278LR_2->RegPayloadLength );
    LoRa2Settings.PayloadLength = value;
}

void sx1278Lora_2SetLowDatarateOptimize( bool enable )
{
    sx1278_2ReadData( REG_LR_MODEMCONFIG3, &SX1278LR_2->RegModemConfig3 );
    SX1278LR_2->RegModemConfig3 = ( SX1278LR_2->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) | ( enable << 3 );
    sx1278_2WriteData( REG_LR_MODEMCONFIG3, SX1278LR_2->RegModemConfig3 );
}

void sx1278Lora_2SetPAOutput( uint8_t outputPin )
{
    sx1278_2ReadData( REG_LR_PACONFIG, &SX1278LR_2->RegPaConfig );
    SX1278LR_2->RegPaConfig = (SX1278LR_2->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | outputPin;
    sx1278_2WriteData( REG_LR_PACONFIG, SX1278LR_2->RegPaConfig );
}

void sx1278Lora_2SetPa20dBm( bool enale )
{
    sx1278_2ReadData( REG_LR_PADAC, &SX1278LR_2->RegPaDac );
    sx1278_2ReadData( REG_LR_PACONFIG, &SX1278LR_2->RegPaConfig );

    if( ( SX1278LR_2->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {    
        if( enale == true )
        {
            SX1278LR_2->RegPaDac = 0x87;
        }
    }
    else
    {
        SX1278LR_2->RegPaDac = 0x84;
    }
    sx1278_2WriteData( REG_LR_PADAC, SX1278LR_2->RegPaDac );
}

void sx1278Lora_2SetRFPower( int8_t power )
{
    sx1278_2ReadData( REG_LR_PACONFIG, &SX1278LR_2->RegPaConfig );
    sx1278_2ReadData( REG_LR_PADAC, &SX1278LR_2->RegPaDac );
    
    if( ( SX1278LR_2->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1278LR_2->RegPaDac & 0x87 ) == 0x87 )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            SX1278LR_2->RegPaConfig = ( SX1278LR_2->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1278LR_2->RegPaConfig = ( SX1278LR_2->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
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
            SX1278LR_2->RegPaConfig = ( SX1278LR_2->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1278LR_2->RegPaConfig = ( SX1278LR_2->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        SX1278LR_2->RegPaConfig = ( SX1278LR_2->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
        SX1278LR_2->RegPaConfig = ( SX1278LR_2->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    sx1278_2WriteData( REG_LR_PACONFIG, SX1278LR_2->RegPaConfig );
    LoRa2Settings.Power = power;
}

bool sx1278Lora_2RFSendBuf( uint8_t *txBuf, size_t txLen)
{
    if(txLen > SX1278LR_2->RegPayloadLength)
		return false;
	
	SX1278LR_2->RegPayloadLength = txLen;
		
    // Initializes the payload size
    SX1278LR_2->RegPayloadLength = txLen;
    sx1278_2WriteData( REG_LR_PAYLOADLENGTH, SX1278LR_2->RegPayloadLength);
	
	// Write payload buffer to LORA modem
    sx1278_2WriteBuf(0, txBuf, SX1278LR_2->RegPayloadLength);
   
    sx1278Lora_2SetOpMode( RFLR_OPMODE_TRANSMITTER | RFLR_OPMODE_LOWFREQUENCYMODEON );
	
	sx1278_2ReadData(REG_LR_IRQFLAGS, &SX1278LR_2->RegIrqFlags);
	
    RFLRState = RFLR_STATE_TX_RUNNING;
	
	return true;
}

tRFLRStates  sx1278Lora_2Process(void)
{
    tRFLRStates result;
	//uint8_t rxSnrEstimate;
	
	switch(RFLRState)
	{
		case RFLR_STATE_IDLE:
			break;
		
		case RFLR_STATE_TX_INIT:
			sx1278Lora_2EntryTx();
			RFLRState = RFLR_STATE_TX_RUNNING;
		    break;
		
	  	case RFLR_STATE_TX_RUNNING:
		 	// TxDone
		    if(Read_sx1278_2Dio0_Pin()) 
			{
			 	sx1278_2ReadData(REG_LR_IRQFLAGS, &SX1278LR_2->RegIrqFlags);
				
				if((SX1278LR_2->RegIrqFlags & RFLR_IRQFLAGS_TXDONE) == RFLR_IRQFLAGS_TXDONE)
				{				  			
					// Clear Irq
					sx1278_2WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
					RFLRState = RFLR_STATE_TX_DONE; 
				}
				else
					result = RFLR_STATE_TX_RUNNING;
			}
			break;
		
		case RFLR_STATE_TX_DONE:
		    // optimize the power consumption by switching off the transmitter as soon as the packet has been sent
        	sx1278Lora_2SetOpMode( RFLR_OPMODE_STANDBY );
        	RFLRState = RFLR_STATE_IDLE;
			result =  RFLR_STATE_TX_DONE;
		  	break;
		
		case RFLR_STATE_RX_INIT:
			sx1278Lora_2EntryRx();
			RFLRState = RFLR_STATE_RX_RUNNING;
			break;
		
		case RFLR_STATE_RX_RUNNING:
			// RxDone
			if( Read_sx1278_2Dio0_Pin()) 
			{
				sx1278_2ReadData(REG_LR_IRQFLAGS, &SX1278LR_2->RegIrqFlags);
				
				if((SX1278LR_2->RegIrqFlags & RFLR_IRQFLAGS_RXDONE) == RFLR_IRQFLAGS_RXDONE)
				{									
					// Clear Irq
					sx1278_2WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );
					RFLRState = RFLR_STATE_RX_DONE;
				}
				else
					result = RFLR_STATE_RX_RUNNING;	
			}
			break;
			
		case RFLR_STATE_RX_DONE:
			sx1278_2ReadData( REG_LR_IRQFLAGS, &SX1278LR_2->RegIrqFlags );
			if( ( SX1278LR_2->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
			{
				// Clear Irq
				sx1278_2WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );
            
				if( LoRa2Settings.RxSingleOn == true ) // Rx single mode
				{
					RFLRState = RFLR_STATE_RX_INIT;
				}
				else
				{
					RFLRState = RFLR_STATE_RX_RUNNING;
				}
				break;
			}
        
//			{
//				sx1278_2ReadData( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
//				if( rxSnrEstimate & 0x80 ) // The SNR sign bit is 1
//				{
//					// Invert and divide by 4
//					RxPacketSnrEstimate = ( ( ~rxSnrEstimate + 1 ) & 0xFF ) >> 2;
//					RxPacketSnrEstimate = -RxPacketSnrEstimate;
//				}
//				else
//				{
//					// Divide by 4
//					RxPacketSnrEstimate = ( rxSnrEstimate & 0xFF ) >> 2;
//				}
//			}
        
//        	if( LoRa2Settings.RFFrequency < 860000000 )  // LF
//        	{    
//            	if( RxPacketSnrEstimate < 0 )
//            	{
//               	 RxPacketRssiValue = NOISE_ABSOLUTE_ZERO_2 + 10.0 * SignalBwLog[LoRa2Settings.SignalBw] + NOISE_FIGURE_LF_2 + ( double )RxPacketSnrEstimate;
//           	 }
//            	else
//            	{    
//                	sx1278_2ReadData( REG_LR_PKTRSSIVALUE, &SX1278LR_2->RegPktRssiValue );
//               	RxPacketRssiValue = RssiOffsetLF[LoRa2Settings.SignalBw] + ( double )SX1278LR_2->RegPktRssiValue;
//            	}
//       	 }
//       	 else                                        // HF
//       	 {    
//            	if( RxPacketSnrEstimate < 0 )
//           	 {
//               	 RxPacketRssiValue = NOISE_ABSOLUTE_ZERO_2 + 10.0 * SignalBwLog[LoRa2Settings.SignalBw] + NOISE_FIGURE_HF_2 + ( double )RxPacketSnrEstimate;
//            	}
//           	 else
//            	{    
//                	sx1278_2ReadData( REG_LR_PKTRSSIVALUE, &SX1278LR_2->RegPktRssiValue );
//                	RxPacketRssiValue = RssiOffsetHF[LoRa2Settings.SignalBw] + ( double )SX1278LR_2->RegPktRssiValue;
//            	}
//        	}

			if( LoRa2Settings.RxSingleOn == true ) // Rx single mode
			{
				SX1278LR_2->RegFifoAddrPtr = SX1278LR_2->RegFifoRxBaseAddr;
				sx1278_2WriteData( REG_LR_FIFOADDRPTR, SX1278LR_2->RegFifoAddrPtr );

				if( LoRa2Settings.ImplicitHeaderOn == true )
				{
					RxPacketSize = SX1278LR_2->RegPayloadLength;
					sx1278_2ReadBuf(0, RF2Buffer, RxPacketSize );
				}
				else
				{
					sx1278_2ReadData( REG_LR_NBRXBYTES, &SX1278LR_2->RegNbRxBytes );
					RxPacketSize = SX1278LR_2->RegNbRxBytes;
					sx1278_2ReadBuf(0, RF2Buffer, RxPacketSize );
				}
			}
			else // Rx continuous mode
			{
				sx1278_2ReadData( REG_LR_FIFORXCURRENTADDR, &SX1278LR_2->RegFifoRxCurrentAddr );

				if( LoRa2Settings.ImplicitHeaderOn == true )
				{
					RxPacketSize = SX1278LR_2->RegPayloadLength;
					SX1278LR_2->RegFifoAddrPtr = SX1278LR_2->RegFifoRxCurrentAddr;
					sx1278_2WriteData( REG_LR_FIFOADDRPTR, SX1278LR_2->RegFifoAddrPtr );
					sx1278_2ReadBuf(0, RF2Buffer, RxPacketSize );
				}
				else
				{
					sx1278_2ReadData( REG_LR_NBRXBYTES, &SX1278LR_2->RegNbRxBytes );
					RxPacketSize = SX1278LR_2->RegNbRxBytes;
					SX1278LR_2->RegFifoAddrPtr = SX1278LR_2->RegFifoRxCurrentAddr;
					sx1278_2WriteData( REG_LR_FIFOADDRPTR, SX1278LR_2->RegFifoAddrPtr );
					sx1278_2ReadBuf(0, RF2Buffer, RxPacketSize );
				}
			}
        
			if( LoRa2Settings.RxSingleOn == true ) // Rx single mode
			{
				RFLRState = RFLR_STATE_RX_INIT;
			}
			else // Rx continuous mode
			{
				RFLRState = RFLR_STATE_RX_RUNNING;
			}
			result = RFLR_STATE_RX_DONE;			
			break;
		
		default:
		  	break;		  		  			  
	}
	
	return  result;
}

void sx1278Lora_2SetRFStatus(tRFLRStates st)
{
	RFLRState = st;
}

void *sx1278Lora_2GetRxData(uint8_t *size)
{
	*size = RxPacketSize;
	RxPacketSize = 0;
	
	return (void *)RF2Buffer;
}

void sx1278_2PinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = SX12782_DIO0_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SX12782_DIO0_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SX12782_RESET_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SX12782_RESET_PORT, &GPIO_InitStructure);	
	
	GPIO_ResetBits(SX12782_RESET_PORT, SX12782_RESET_PIN);
	delay_ms(50);
	GPIO_SetBits(SX12782_RESET_PORT, SX12782_RESET_PIN);
}

uint8_t Read_sx1278_2Dio0_Pin(void)
{
	return GPIO_ReadInputDataBit(SX12782_DIO0_PORT, SX12782_DIO0_PIN);
}

void sx1278Lora_2SetRFMode(bool mode)
{
	;
}

bool sx1278_GetLoRaOn(void)
{
    return LoRaOn;
}
//******************************************************************************
/* sx1278 Read and write register functions */
void  sx1278_2WriteData(uint8_t addr, uint8_t data)
{	
	uint8_t wAddres;
	
	wAddres = addr | 0x80;
	
	Spi1_AssertCSN();
	Spi1_ReadWriteByte(wAddres);
 	Spi1_ReadWriteByte(data);
	Spi1_DeAssertCSN();	
}

void  sx1278_2WriteBuf(uint8_t addr, uint8_t *buf, uint8_t size)
{	 
	uint8_t wAddres;		
	
	if(size > RF_PACKET_LEN_2 || size <= 0)
		return;
	
	wAddres = addr | 0x80;
			
	Spi1_AssertCSN();
	Spi1_ReadWriteByte(wAddres);
	while(size --)
	{
		Spi1_ReadWriteByte(*buf);
		buf ++;
	}
	Spi1_DeAssertCSN();		
}

void  sx1278_2ReadData(uint8_t addr, uint8_t *data)
{   
	uint8_t rAddres;
	
	rAddres = addr & 0x7F;  
	
	Spi1_AssertCSN();
	Spi1_ReadWriteByte(rAddres);
 	*data = Spi1_ReadWriteByte(0xFF);
	Spi1_DeAssertCSN();
}

void  sx1278_2ReadBuf(uint8_t addr, uint8_t *buf, uint8_t size)
{
	uint8_t rAddres;
	
	if(size > RF_PACKET_LEN_2 || size <= 0)
		return;
	
   	rAddres = addr & 0x7F;  
	
	Spi1_AssertCSN();
	Spi1_ReadWriteByte(rAddres);
	while(size --)
	{
		*buf = Spi1_ReadWriteByte(0xFF);
		buf ++;
	}
	Spi1_DeAssertCSN();
}
