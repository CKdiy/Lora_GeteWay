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
#define RSSI_OFFSET_LF                              -155.0
#define RSSI_OFFSET_HF                              -150.0

#define NOISE_ABSOLUTE_ZERO                         -174.0

#define NOISE_FIGURE_LF                                4.0
#define NOISE_FIGURE_HF                                6.0 

/*!
 * Precomputed signal bandwidth log values
 * Used to compute the Packet RSSI value.
 */
const double SignalBwLog[] =
{
    3.8927900303521316335038277369285,  // 7.8 kHz
    4.0177301567005500940384239336392,  // 10.4 kHz
    4.193820026016112828717566631653,   // 15.6 kHz
    4.31875866931372901183597627752391, // 20.8 kHz
    4.4948500216800940239313055263775,  // 31.2 kHz
    4.6197891057238405255051280399961,  // 41.6 kHz
    4.795880017344075219145044421102,   // 62.5 kHz
    5.0969100130080564143587833158265,  // 125 kHz
    5.397940008672037609572522210551,   // 250 kHz
    5.6989700043360188047862611052755   // 500 kHz
};

const double RssiOffsetLF[] =
{   // These values need to be specify in the Lab
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
    -155.0,
};

const double RssiOffsetHF[] =
{   // These values need to be specify in the Lab
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
    -150.0,
};

#define REGVERSION_DEFAULT_1   0x12 
#define RF_PACKET_LEN_1        1<<6

#define SX12781_DIO0_PORT    GPIOC
#define SX12781_DIO0_PIN     GPIO_Pin_6
#define SX12781_RESET_PORT   GPIOC
#define SX12781_RESET_PIN    GPIO_Pin_7

uint8_t RF1Buffer[RF_PACKET_LEN_1];

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
const int32_t HoppingFrequencies[] =
{
    916500000,
    923500000,
    906500000,
    917500000,
    917500000,
    909000000,
    903000000,
    916000000,
    912500000,
    926000000,
    925000000,
    909500000,
    913000000,
    918500000,
    918500000,
    902500000,
    911500000,
    926500000,
    902500000,
    922000000,
    924000000,
    903500000,
    913000000,
    922000000,
    926000000,
    910000000,
    920000000,
    922500000,
    911000000,
    922000000,
    909500000,
    926000000,
    922000000,
    918000000,
    925500000,
    908000000,
    917500000,
    926500000,
    908500000,
    916000000,
    905500000,
    916000000,
    903000000,
    905000000,
    915000000,
    913000000,
    907000000,
    910000000,
    926500000,
    925500000,
    911000000,
};

// Default settings
tLoRaSettings LoRaSettings =
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
static tSX1276LR* SX1278LR_1;

/*******************************************************************/
// sx1278 LORA Functions 
void sx1278_1SetLoRaOn(bool enable);
bool sx1278_1GetLoRaOn(void);
void sx1278_1WriteData(uint8_t addr, uint8_t data);
void sx1278_1WriteBuf(uint8_t addr, uint8_t *buf, uint8_t size);
void sx1278_1ReadData(uint8_t addr, uint8_t *data);
void sx1278_1ReadBuf(uint8_t addr, uint8_t *buf, uint8_t size);

void sx1278Lora_1SetRFFrequency( uint32_t freq );
void sx1278Lora_1SetOpMode(uint8_t opMode);
void sx1278Lora_1SetSpreadingFactor( uint8_t factor );
void sx1278Lora_1SetErrorCoding( uint8_t value );
void sx1278Lora_1SetPacketCrcOn( bool enable );
void sx1278Lora_1SetSignalBandwidth( uint8_t bw );
void sx1278Lora_1SetImplicitHeaderOn( bool enable );
void sx1278Lora_1SetSymbTimeout( uint16_t value );
void sx1278Lora_1SetPayloadLength( uint8_t value );
void sx1278Lora_1SetLowDatarateOptimize( bool enable );
void sx1278Lora_1SetPAOutput( uint8_t outputPin );
void sx1278Lora_1SetPa20dBm( bool enale );
void sx1278Lora_1SetRFPower( int8_t power );
void sx1278Lora_1SetRFMode(bool mode);
uint8_t sx1278Lora_1GetOpMode( void );
bool sx1278Lora_1RFSendBuf( uint8_t *txBuf, size_t txLen);
tRFLRStates  sx1278Lora_1Process(void);
static void sx1278Lora_1SetParameters(void);
void sx1278_1PinInit(void);
uint8_t Read_sx1278_1Dio0_Pin(void);
//******************************************************************************
/* sx1278Lora Initialize/configuration/control functions */
void sx1278_1Init(void)
{
	SX1278LR_1 = (tSX1276LR *)SX1278Regs;
	
	sx1278_1PinInit();
	Spi2_Configuration();
	
	LoRaOn = true;
	sx1278_1SetLoRaOn(LoRaOn);  
}

void sx1278_1SetLoRaOn(bool enable)
{
	if(LoRaOnState == enable)
    	return;
    
    LoRaOnState = enable;
    LoRaOn      = enable;
	
	if( LoRaOn == true )
    {  	
	    // REMARK: See SX1276 datasheet for modified default values(0x12).
   	 	sx1278_1ReadData( REG_LR_VERSION, &SX1278LR_1->RegVersion );
	
		//Just for testing SPI
   		if(SX1278LR_1->RegVersion != REGVERSION_DEFAULT_1)
			return; 
		
    	sx1278Lora_1SetOpMode(RFLR_OPMODE_SLEEP);
        
		delay_ms(10);
		
		SX1278LR_1->RegTcxo = 0x09;   //USE TCXO
		
		sx1278_1WriteData(REG_LR_TCXO, SX1278LR_1->RegTcxo);
		
        SX1278LR_1->RegOpMode = ( SX1278LR_1->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_ON | RFLR_OPMODE_LOWFREQUENCYMODEON;
		
		//select lora mode
		sx1278_1WriteData(REG_LR_OPMODE, SX1278LR_1->RegOpMode);
        
		sx1278Lora_1SetRFFrequency(LoRaSettings.RFFrequency);
		
		if(LoRaSettings.RFFrequency > 200000000)
		{
			sx1278Lora_1SetPAOutput( RFLR_PACONFIG_PASELECT_PABOOST );
    		sx1278Lora_1SetPa20dBm( true );
			LoRaSettings.Power = 20;
    		sx1278Lora_1SetRFPower( LoRaSettings.Power );
		}
		else
		{
			sx1278Lora_1SetPAOutput( RFLR_PACONFIG_PASELECT_RFO );
    		sx1278Lora_1SetPa20dBm( false );
			LoRaSettings.Power = 14;
    		sx1278Lora_1SetRFPower( LoRaSettings.Power );		
		}
		
		//RegOcp,Close Ocp		
		SX1278LR_1->RegOcp = 0x0B;
		sx1278_1WriteData(REG_LR_OCP, SX1278LR_1->RegOcp);
		
		//RegLNA,High & LNA Enable
		SX1278LR_1->RegLna = 0x23;
		sx1278_1WriteData(REG_LR_LNA, SX1278LR_1->RegLna);
		
		sx1278Lora_1SetParameters();
		
		sx1278Lora_1SetSymbTimeout( 0x3FF );	
		
		SX1278LR_1->RegPreambleMsb = 0;
		SX1278LR_1->RegPreambleLsb = 16;
		sx1278_1WriteData(REG_LR_PREAMBLEMSB, SX1278LR_1->RegPreambleMsb);
		sx1278_1WriteData(REG_LR_PREAMBLELSB, SX1278LR_1->RegPreambleLsb); 				
		
		//RegDioMapping2 DIO5=00, DIO4=01
		SX1278LR_1->RegDioMapping2 = 0x01;
		sx1278_1WriteData(REG_LR_DIOMAPPING2, SX1278LR_1->RegDioMapping2);
		sx1278Lora_1SetOpMode(RFLR_OPMODE_STANDBY);
    }
    else
    {
        sx1278Lora_1SetOpMode( RFLR_OPMODE_SLEEP );
        
        SX1278LR_1->RegOpMode = ( SX1278LR_1->RegOpMode & RFLR_OPMODE_LONGRANGEMODE_MASK ) | RFLR_OPMODE_LONGRANGEMODE_OFF;
        sx1278_1WriteData(REG_LR_OPMODE, SX1278LR_1->RegOpMode);
        
        sx1278Lora_1SetOpMode( RFLR_OPMODE_STANDBY );  
    }
	
	RFLRState = RFLR_STATE_RX_INIT;
}

static void sx1278Lora_1SetParameters(void)
{
	// SF6 only operates in implicit header mode.
	sx1278Lora_1SetSpreadingFactor(LoRaSettings.SpreadingFactor); 
	
	sx1278Lora_1SetErrorCoding( LoRaSettings.ErrorCoding );	
	sx1278Lora_1SetPacketCrcOn( LoRaSettings.CrcOn );
	sx1278Lora_1SetSignalBandwidth( LoRaSettings.SignalBw );
	sx1278Lora_1SetImplicitHeaderOn( LoRaSettings.ImplicitHeaderOn );
    sx1278Lora_1SetPayloadLength( LoRaSettings.PayloadLength );
    sx1278Lora_1SetLowDatarateOptimize( true );	 
}

void sx1278Lora_1EntryTx(void)
{	  
	sx1278Lora_1SetOpMode( RFLR_OPMODE_STANDBY );
	if( LoRaSettings.FreqHopOn == true )
    {
    	SX1278LR_1->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    //RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1278LR_1->RegHopPeriod = LoRaSettings.HopPeriod;

        sx1278_1ReadData( REG_LR_HOPCHANNEL, &SX1278LR_1->RegHopChannel );
        sx1278Lora_1SetRFFrequency( HoppingFrequencies[SX1278LR_1->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
    }
    else
    {
        SX1278LR_1->RegIrqFlagsMask = RFLR_IRQFLAGS_RXTIMEOUT |
                                    RFLR_IRQFLAGS_RXDONE |
                                    RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                    RFLR_IRQFLAGS_VALIDHEADER |
                                    //RFLR_IRQFLAGS_TXDONE |
                                    RFLR_IRQFLAGS_CADDONE |
                                    RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                    RFLR_IRQFLAGS_CADDETECTED;
        SX1278LR_1->RegHopPeriod = 0;
    }
	
	sx1278_1WriteData( REG_LR_HOPPERIOD, SX1278LR_1->RegHopPeriod );
	
	                                    // TxDone               RxTimeout                   FhssChangeChannel          ValidHeader         
    SX1278LR_1->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                        // PllLock              Mode Ready
    SX1278LR_1->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
    sx1278_1WriteBuf( REG_LR_DIOMAPPING1, &SX1278LR_1->RegDioMapping1, 2 );
	
	//Clear all irq
	sx1278_1WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_ALL);
    sx1278_1WriteData( REG_LR_IRQFLAGSMASK, SX1278LR_1->RegIrqFlagsMask );
	
	// Full buffer used for Tx
	SX1278LR_1->RegFifoTxBaseAddr = 0x00; 
    sx1278_1WriteData( REG_LR_FIFOTXBASEADDR, SX1278LR_1->RegFifoTxBaseAddr );
	
	SX1278LR_1->RegFifoAddrPtr = SX1278LR_1->RegFifoTxBaseAddr;
    sx1278_1WriteData( REG_LR_FIFOADDRPTR, SX1278LR_1->RegFifoAddrPtr );	
}

void sx1278Lora_1EntryRx(void)
{
	sx1278Lora_1SetOpMode( RFLR_OPMODE_STANDBY );

	SX1278LR_1->RegIrqFlagsMask = //RFLR_IRQFLAGS_RXTIMEOUT |
                                //RFLR_IRQFLAGS_RXDONE |
                                RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                RFLR_IRQFLAGS_VALIDHEADER |
                                RFLR_IRQFLAGS_TXDONE |
                                RFLR_IRQFLAGS_CADDONE |
                                RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                RFLR_IRQFLAGS_CADDETECTED;
	
	sx1278_1WriteData( REG_LR_IRQFLAGSMASK, SX1278LR_1->RegIrqFlagsMask );

	if( LoRaSettings.FreqHopOn == true )
    {
		SX1278LR_1->RegHopPeriod = LoRaSettings.HopPeriod;

		sx1278_1ReadData( REG_LR_HOPCHANNEL, &SX1278LR_1->RegHopChannel );
		sx1278Lora_1SetRFFrequency( HoppingFrequencies[SX1278LR_1->RegHopChannel & RFLR_HOPCHANNEL_CHANNEL_MASK] );
	}
	else
    {
		SX1278LR_1->RegHopPeriod = 255;
	}
        
    sx1278_1WriteData( REG_LR_HOPPERIOD, SX1278LR_1->RegHopPeriod );
                
                                   // RxDone                    RxTimeout                   FhssChangeChannel           CadDone
    SX1278LR_1->RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00 | RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
                                   // CadDetected               ModeReady
    SX1278LR_1->RegDioMapping2 = RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
    sx1278_1WriteBuf( REG_LR_DIOMAPPING1, &SX1278LR_1->RegDioMapping1, 2 );
    
    if( LoRaSettings.RxSingleOn == true ) // Rx single mode
    {
        sx1278Lora_1SetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
    }
    else // Rx continuous mode
    {
		// Full buffer used for Rx
		SX1278LR_1->RegFifoRxBaseAddr = 0x00;
        SX1278LR_1->RegFifoAddrPtr = SX1278LR_1->RegFifoRxBaseAddr;
        sx1278_1WriteData( REG_LR_FIFOADDRPTR, SX1278LR_1->RegFifoAddrPtr );
            
        sx1278Lora_1SetOpMode( RFLR_OPMODE_RECEIVER );
    }
    
	sx1278_1ReadData(REG_LR_IRQFLAGS, &SX1278LR_1->RegIrqFlags);  
	
    memset( RF1Buffer, 0, ( size_t )RF_PACKET_LEN_1 );
}

void sx1278Lora_1SetOpMode(uint8_t opMode)
{
    static uint8_t opModePrev = RFLR_OPMODE_STANDBY;
    static bool antennaSwitchTxOnPrev = true;
    bool antennaSwitchTxOn = false;

    opModePrev = SX1278LR_1->RegOpMode & ~RFLR_OPMODE_MASK;

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
			  sx1278Lora_1SetRFMode(antennaSwitchTxOn);
			}
        }
        SX1278LR_1->RegOpMode = ( SX1278LR_1->RegOpMode & RFLR_OPMODE_MASK ) | opMode;
		
        sx1278_1WriteData(REG_LR_OPMODE, SX1278LR_1->RegOpMode); 
    }
}

uint8_t sx1278Lora_1GetOpMode( void )
{
    sx1278_1ReadData( REG_LR_OPMODE, &SX1278LR_1->RegOpMode );
    
    return SX1278LR_1->RegOpMode & ~RFLR_OPMODE_MASK;
}

void sx1278Lora_1SetRFFrequency( uint32_t freq )
{
    LoRaSettings.RFFrequency = freq;

    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    SX1278LR_1->RegFrfMsb = ( uint8_t )( ( freq >> 16 ) & 0xFF );
    SX1278LR_1->RegFrfMid = ( uint8_t )( ( freq >> 8 ) & 0xFF );
    SX1278LR_1->RegFrfLsb = ( uint8_t )( freq & 0xFF );
    sx1278_1WriteBuf( REG_LR_FRFMSB, &SX1278LR_1->RegFrfMsb, 3 );
}

void sx1278Lora_1SetSpreadingFactor( uint8_t factor )
{
    if( factor > 12 )
    {
        factor = 12;
    }
    else if( factor < 6 )
    {
        factor = 6;
    }

    sx1278_1ReadData( REG_LR_MODEMCONFIG2, &SX1278LR_1->RegModemConfig2 );    
    SX1278LR_1->RegModemConfig2 = ( SX1278LR_1->RegModemConfig2 & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
    sx1278_1WriteData( REG_LR_MODEMCONFIG2, SX1278LR_1->RegModemConfig2 );  
    LoRaSettings.SpreadingFactor = factor;
}

void sx1278Lora_1SetErrorCoding( uint8_t value )
{
    sx1278_1ReadData( REG_LR_MODEMCONFIG1, &SX1278LR_1->RegModemConfig1 );
    SX1278LR_1->RegModemConfig1 = ( SX1278LR_1->RegModemConfig1 & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );
    sx1278_1WriteData( REG_LR_MODEMCONFIG1, SX1278LR_1->RegModemConfig1 );
    LoRaSettings.ErrorCoding = value;
}

void  sx1278Lora_1SetPacketCrcOn( bool enable )
{
    sx1278_1ReadData( REG_LR_MODEMCONFIG2, &SX1278LR_1->RegModemConfig2 );
    SX1278LR_1->RegModemConfig2 = ( SX1278LR_1->RegModemConfig2 & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
    sx1278_1WriteData( REG_LR_MODEMCONFIG2, SX1278LR_1->RegModemConfig2 );
    LoRaSettings.CrcOn = enable;
}

void sx1278Lora_1SetSignalBandwidth( uint8_t bw )
{
    sx1278_1ReadData( REG_LR_MODEMCONFIG1, &SX1278LR_1->RegModemConfig1 );
    SX1278LR_1->RegModemConfig1 = ( SX1278LR_1->RegModemConfig1 & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
    sx1278_1WriteData( REG_LR_MODEMCONFIG1, SX1278LR_1->RegModemConfig1 );
    LoRaSettings.SignalBw = bw;
}

void sx1278Lora_1SetImplicitHeaderOn( bool enable )
{
    sx1278_1ReadData( REG_LR_MODEMCONFIG1, &SX1278LR_1->RegModemConfig1 );
    SX1278LR_1->RegModemConfig1 = ( SX1278LR_1->RegModemConfig1 & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
    sx1278_1WriteData( REG_LR_MODEMCONFIG1, SX1278LR_1->RegModemConfig1 );
    LoRaSettings.ImplicitHeaderOn = enable;
}

void sx1278Lora_1SetSymbTimeout( uint16_t value )
{
    sx1278_1ReadBuf( REG_LR_MODEMCONFIG2, &SX1278LR_1->RegModemConfig2, 2 );

    SX1278LR_1->RegModemConfig2 = ( SX1278LR_1->RegModemConfig2 & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
    SX1278LR_1->RegSymbTimeoutLsb = value & 0xFF;
    sx1278_1WriteBuf( REG_LR_MODEMCONFIG2, &SX1278LR_1->RegModemConfig2, 2 );
}

void sx1278Lora_1SetPayloadLength( uint8_t value )
{
    SX1278LR_1->RegPayloadLength = value;
    sx1278_1WriteData( REG_LR_PAYLOADLENGTH, SX1278LR_1->RegPayloadLength );
    LoRaSettings.PayloadLength = value;
}

void sx1278Lora_1SetLowDatarateOptimize( bool enable )
{
    sx1278_1ReadData( REG_LR_MODEMCONFIG3, &SX1278LR_1->RegModemConfig3 );
    SX1278LR_1->RegModemConfig3 = ( SX1278LR_1->RegModemConfig3 & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) | ( enable << 3 );
    sx1278_1WriteData( REG_LR_MODEMCONFIG3, SX1278LR_1->RegModemConfig3 );
}

void sx1278Lora_1SetPAOutput( uint8_t outputPin )
{
    sx1278_1ReadData( REG_LR_PACONFIG, &SX1278LR_1->RegPaConfig );
    SX1278LR_1->RegPaConfig = (SX1278LR_1->RegPaConfig & RFLR_PACONFIG_PASELECT_MASK ) | outputPin;
    sx1278_1WriteData( REG_LR_PACONFIG, SX1278LR_1->RegPaConfig );
}

void sx1278Lora_1SetPa20dBm( bool enale )
{
    sx1278_1ReadData( REG_LR_PADAC, &SX1278LR_1->RegPaDac );
    sx1278_1ReadData( REG_LR_PACONFIG, &SX1278LR_1->RegPaConfig );

    if( ( SX1278LR_1->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {    
        if( enale == true )
        {
            SX1278LR_1->RegPaDac = 0x87;
        }
    }
    else
    {
        SX1278LR_1->RegPaDac = 0x84;
    }
    sx1278_1WriteData( REG_LR_PADAC, SX1278LR_1->RegPaDac );
}

void sx1278Lora_1SetRFPower( int8_t power )
{
    sx1278_1ReadData( REG_LR_PACONFIG, &SX1278LR_1->RegPaConfig );
    sx1278_1ReadData( REG_LR_PADAC, &SX1278LR_1->RegPaDac );
    
    if( ( SX1278LR_1->RegPaConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
    {
        if( ( SX1278LR_1->RegPaDac & 0x87 ) == 0x87 )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            SX1278LR_1->RegPaConfig = ( SX1278LR_1->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1278LR_1->RegPaConfig = ( SX1278LR_1->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
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
            SX1278LR_1->RegPaConfig = ( SX1278LR_1->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
            SX1278LR_1->RegPaConfig = ( SX1278LR_1->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
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
        SX1278LR_1->RegPaConfig = ( SX1278LR_1->RegPaConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;
        SX1278LR_1->RegPaConfig = ( SX1278LR_1->RegPaConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    sx1278_1WriteData( REG_LR_PACONFIG, SX1278LR_1->RegPaConfig );
    LoRaSettings.Power = power;
}

bool sx1278Lora_1RFSendBuf( uint8_t *txBuf, size_t txLen)
{
    if(txLen > SX1278LR_1->RegPayloadLength)
		return false;
	
	SX1278LR_1->RegPayloadLength = txLen;
		
    // Initializes the payload size
    SX1278LR_1->RegPayloadLength = txLen;
    sx1278_1WriteData( REG_LR_PAYLOADLENGTH, SX1278LR_1->RegPayloadLength);
	
	// Write payload buffer to LORA modem
    sx1278_1WriteBuf(0, txBuf, SX1278LR_1->RegPayloadLength);
   
    sx1278Lora_1SetOpMode( RFLR_OPMODE_TRANSMITTER | RFLR_OPMODE_LOWFREQUENCYMODEON );
	
	sx1278_1ReadData(REG_LR_IRQFLAGS, &SX1278LR_1->RegIrqFlags);
	
    RFLRState = RFLR_STATE_TX_RUNNING;
	
	return true;
}

tRFLRStates  sx1278Lora_1Process(void)
{
    tRFLRStates result;
	//uint8_t rxSnrEstimate;
	
	switch(RFLRState)
	{
		case RFLR_STATE_IDLE:
			break;
		
		case RFLR_STATE_TX_INIT:
			sx1278Lora_1EntryTx();
			RFLRState = RFLR_STATE_TX_RUNNING;
		    result    = RFLR_STATE_TX_RUNNING;
		    break;
		
	  	case RFLR_STATE_TX_RUNNING:
			if(Read_sx1278_1Dio0_Pin())
			{			
				sx1278_1ReadData(REG_LR_IRQFLAGS, &SX1278LR_1->RegIrqFlags);
						
				if((SX1278LR_1->RegIrqFlags & RFLR_IRQFLAGS_TXDONE) == RFLR_IRQFLAGS_TXDONE)
				{ 	
					// Clear Irq
					sx1278_1WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
					RFLRState = RFLR_STATE_TX_DONE; 
				}
				else
					result = RFLR_STATE_TX_RUNNING; 
			}
			break;
		
		case RFLR_STATE_TX_DONE:
		    // optimize the power consumption by switching off the transmitter as soon as the packet has been sent
        	sx1278Lora_1SetOpMode( RFLR_OPMODE_STANDBY );
        	RFLRState = RFLR_STATE_IDLE;
			result =  RFLR_STATE_TX_DONE;
		  	break;
		
		case RFLR_STATE_RX_INIT:
			sx1278Lora_1EntryRx();
			RFLRState = RFLR_STATE_RX_RUNNING;
			break;
		
		case RFLR_STATE_RX_RUNNING:
			// RxDone
			if( Read_sx1278_1Dio0_Pin())
			{				
				sx1278_1ReadData(REG_LR_IRQFLAGS, &SX1278LR_1->RegIrqFlags);
							
				if((SX1278LR_1->RegIrqFlags & RFLR_IRQFLAGS_RXDONE) == RFLR_IRQFLAGS_RXDONE)
				{			
					// Clear Irq
					sx1278_1WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );
					RFLRState = RFLR_STATE_RX_DONE;
				}		
				else
					result = RFLR_STATE_RX_RUNNING;
			}
			break;
			
		case RFLR_STATE_RX_DONE:
			sx1278_1ReadData( REG_LR_IRQFLAGS, &SX1278LR_1->RegIrqFlags );
			if( ( SX1278LR_1->RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
			{
				// Clear Irq
				sx1278_1WriteData( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR  );
            
				if( LoRaSettings.RxSingleOn == true ) // Rx single mode
				{
					RFLRState = RFLR_STATE_RX_INIT;
				}
				else
				{
					RFLRState = RFLR_STATE_RX_RUNNING;
				}
				break;
			}
        
			//RF�������ʱ�ò���
//			{
//				sx1278_1ReadData( REG_LR_PKTSNRVALUE, &rxSnrEstimate );
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
        
//        	if( LoRaSettings.RFFrequency < 860000000 )  // LF
//        	{    
//            	if( RxPacketSnrEstimate < 0 )
//            	{
//                	RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_LF + ( double )RxPacketSnrEstimate;
//            	}
//            	else
//            	{    
//                	sx1278_1ReadData( REG_LR_PKTRSSIVALUE, &SX1278LR_1->RegPktRssiValue );
//                	RxPacketRssiValue = RssiOffsetLF[LoRaSettings.SignalBw] + ( double )SX1278LR_1->RegPktRssiValue;
//            	}
//        	}
//        	else                                        // HF
//        	{    
//            	if( RxPacketSnrEstimate < 0 )
//            	{
//                	RxPacketRssiValue = NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[LoRaSettings.SignalBw] + NOISE_FIGURE_HF + ( double )RxPacketSnrEstimate;
//           	 }
//            	else
//            	{    
//                	sx1278_1ReadData( REG_LR_PKTRSSIVALUE, &SX1278LR_1->RegPktRssiValue );
//               	 RxPacketRssiValue = RssiOffsetHF[LoRaSettings.SignalBw] + ( double )SX1278LR_1->RegPktRssiValue;
//            	}
//        	}

        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
        {
            SX1278LR_1->RegFifoAddrPtr = SX1278LR_1->RegFifoRxBaseAddr;
            sx1278_1WriteData( REG_LR_FIFOADDRPTR, SX1278LR_1->RegFifoAddrPtr );

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1278LR_1->RegPayloadLength;
                sx1278_1ReadBuf(0, RF1Buffer, RxPacketSize );
            }
            else
            {
                sx1278_1ReadData( REG_LR_NBRXBYTES, &SX1278LR_1->RegNbRxBytes );
                RxPacketSize = SX1278LR_1->RegNbRxBytes;
                sx1278_1ReadBuf(0, RF1Buffer, RxPacketSize );
            }
        }
        else // Rx continuous mode
        {
            sx1278_1ReadData( REG_LR_FIFORXCURRENTADDR, &SX1278LR_1->RegFifoRxCurrentAddr );

            if( LoRaSettings.ImplicitHeaderOn == true )
            {
                RxPacketSize = SX1278LR_1->RegPayloadLength;
                SX1278LR_1->RegFifoAddrPtr = SX1278LR_1->RegFifoRxCurrentAddr;
                sx1278_1WriteData( REG_LR_FIFOADDRPTR, SX1278LR_1->RegFifoAddrPtr );
                sx1278_1ReadBuf(0, RF1Buffer, RxPacketSize );
            }
            else
            {
                sx1278_1ReadData( REG_LR_NBRXBYTES, &SX1278LR_1->RegNbRxBytes );
                RxPacketSize = SX1278LR_1->RegNbRxBytes;
                SX1278LR_1->RegFifoAddrPtr = SX1278LR_1->RegFifoRxCurrentAddr;
                sx1278_1WriteData( REG_LR_FIFOADDRPTR, SX1278LR_1->RegFifoAddrPtr );
                sx1278_1ReadBuf(0, RF1Buffer, RxPacketSize );
            }
        }
        
        if( LoRaSettings.RxSingleOn == true ) // Rx single mode
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

void sx1278Lora_1SetRFStatus(tRFLRStates st)
{
	RFLRState = st;
}

void *sx1278Lora_1GetRxData(uint8_t *size)
{
	*size = RxPacketSize;
	RxPacketSize = 0;
	
	return (void *)RF1Buffer;
}

void sx1278_1PinInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = SX12781_DIO0_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SX12781_DIO0_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SX12781_RESET_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SX12781_RESET_PORT, &GPIO_InitStructure);	
	
	GPIO_ResetBits(SX12781_RESET_PORT, SX12781_RESET_PIN);
	delay_ms(50);
	GPIO_SetBits(SX12781_RESET_PORT, SX12781_RESET_PIN);
}

uint8_t Read_sx1278_1Dio0_Pin(void)
{
	return GPIO_ReadInputDataBit(SX12781_DIO0_PORT, SX12781_DIO0_PIN);
}

void sx1278Lora_1SetRFMode(bool mode)
{
	;
}

bool sx1278_1GetLoRaOn(void)
{
    return LoRaOn;
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
	
	if(size > RF_PACKET_LEN_1 || size <= 0)
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
	
	if(size > RF_PACKET_LEN_1 || size <= 0)
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
