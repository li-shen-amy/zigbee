/****************************************************************************
 *
 * MODULE:			   Router
 *
 * COMPONENT:          $RCSfile: WSN_Router.c,v $
 *
 * VERSION:            $Name:  $
 *
 * REVISION:           $Revision: $
 *
 * DATED:              $Date: 2009/9/25  $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             Li
 *
 * DESCRIPTION:
 *
 * Implements a Wireless Sensor Network Router using the Jennic Zigbee stack.
 * Reads temperature, humidity and battery voltage and transmits these to
 * network coordinator. Assumes code is running on a evaluation kit sensor
 * board.
 *
 * LAST MODIFIED BY:   $Author: Li $
 *                     $Modtime: $
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <ALSdriver.h>
#include <HTSdriver.h>
#include <LedControl.h>
#include <AppHardwareApi.h>
#include <JZ_Api.h>

#include "WSN_Profile.h"
//#include "serial.h"
#include "uart.h"

#include "data1.h"
#include "data2.h"
#include "data3.h"
#include "data4.h"
#include "data5.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Timing values */
#define APP_TICK_PERIOD_ms		  100
#define APP_TICK_PERIOD     	  (APP_TICK_PERIOD_ms * 32)

#define APP_DATA_SEND_PERIOD_ms	  1000
#define APP_DATA_SEND_PERIOD	  (APP_DATA_SEND_PERIOD_ms / APP_TICK_PERIOD_ms)

#define UART                    E_AHI_UART_0
#define UART_BAUD_RATE          115200//19200

#if UART == E_AHI_UART_0
    #define UART_START_ADR   0x30000000UL
#else
    #define UART_START_ADR   0x40000000UL
#endif

#define UART_LCR_OFFSET  0x0C
#define UART_DLM_OFFSET  0x04
#define MAX_COMMAND 0x08

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/* Battery reading state definitions */
typedef enum
{
	E_STATE_READ_BATT_VOLT_IDLE,
	E_STATE_READ_BATT_VOLTS_ADC_CONVERTING,
	E_STATE_READ_BATT_VOLTS_COMPLETE,
        E_STATE_READ_BATT_VOLTS_READY
}teStateReadBattVolt;

/* Temperature/Humidity Sensor - reading state definitions */
typedef enum
{
	E_STATE_READ_TEMP_HUMID_IDLE,
	E_STATE_READ_HUMID_RUNNING,
	E_STATE_READ_TEMP_HUMID_COMPLETE,
	E_STATE_READ_TEMP_START,
	E_STATE_READ_TEMP_HUMID_RUNNING,
	E_STATE_READ_TEMP_COMPLETE,
	E_STATE_READ_TEMP_HUMID_READY
}teStateReadTempHumidity;

/* Battery measurement data */
typedef struct
{
	uint16 					u16Reading;
	teStateReadBattVolt 	eState;
}tsBattSensor;

/* Temp/Humidity measurement data */
typedef struct
{
	uint16 					u16TempReading;
	uint16 					u16HumidReading;
	teStateReadTempHumidity eState;
}tsTempHumiditySensor;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vInit(void);
PRIVATE void vSendData(void);
PRIVATE void vInitSensors(void);
PRIVATE void vReadTempHumidity(void);
PRIVATE void vReadBatteryVoltage(void);
PRIVATE void vAppTick(void *pvMsg, uint8 u8Param);
PRIVATE void vUART_Init(void);
PRIVATE void vUART_SetBuadRate(uint32 u32BaudRate);
//PRIVATE void vProcessEventQueues(void);
PRIVATE void vTimer0Config(void);
PRIVATE void vDio2_Output(uint8 Rec_Char);
PRIVATE void data_copy(uint8 *DstData,uint8 *SrcData);
//PRIVATE void vTimer1ISR(uint32 u32DeviceId, uint32 u32ItemBitmap);
PRIVATE void Init_DIO2(void);
PRIVATE void vTickTimerConfig(void);
PRIVATE void vTickTimerISR(uint32 u32DeviceId, uint32 u32ItemBitmap);

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE uint8 u8AppTicks = 0;
PRIVATE tsBattSensor sBattSensor;
PRIVATE tsTempHumiditySensor sTempHumiditySensor;
PRIVATE bool_t bAppTimerStarted = FALSE;
PRIVATE bool_t bNwkJoined = FALSE;
PRIVATE char cCharIn=0;
PRIVATE uint8 Rec_Char=0;
PRIVATE uint8 repeat_count=128;
PRIVATE uint16 count=4;
PRIVATE bool_t time_last_sign=0;
PRIVATE uint8 time_count=0;
PRIVATE uint8 data[512];
PRIVATE bool_t output=0;
PRIVATE uint32 time_rise;
//PRIVATE uint16 time_rise;
//PRIVATE uint16 time_down;
PRIVATE uint16 num_count=0;
PRIVATE uint16 u16Hi;   /* stores the tmr0 count when input goes high */
PRIVATE uint16 u16Lo;   /* stores the tmr0 count when input goes low */
PRIVATE int32 i32Data; /* stores the difference between u16Hi and u16Lo */


/****************************************************************************
 *
 * NAME: AppColdStart
 *
 * DESCRIPTION:
 * Entry point for application. Initialises system, starts scan then
 * processes interrupts.
 *
 * RETURNS:
 * void, never returns
 *
 ****************************************************************************/
PUBLIC void AppColdStart(void)
{
    /* General initialisation: reset hardware */
    JZS_sConfig.u32Channel 	= WSN_CHANNEL;
    JZS_sConfig.u16PanId 	= WSN_PAN_ID;

    /* General initialisation: reset hardware */
    vInit();

    /* No return from the above function call */
}

/****************************************************************************
 *
 * NAME: AppWarmStart
 *
 * DESCRIPTION:
 * Entry point for application from boot loader. Simply jumps to AppColdStart
 * as, in this instance, application will never warm start.
 *
 * RETURNS:
 * Never returns.
 *
 ****************************************************************************/
PUBLIC void AppWarmStart(void)
{
    AppColdStart();
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: vInit
 *
 * DESCRIPTION:
 * Initialises Zigbee stack and hardware. Final action is to start BOS, from
 * which there is no return. Subsequent application actions occur in the
 * functions defined above.
 *
 * RETURNS:
 * No return from this function
 *
 ****************************************************************************/
PRIVATE void vInit(void)
{
    /* Initialise Zigbee stack */
    JZS_u32InitSystem(TRUE);

    /* Set DIO for LEDs */
    vLedInitRfd();
    vLedControl(0,0);
    vLedControl(1,0);

    /* Set sensors */
    vInitSensors();

    /* set uart */
    vUART_Init();

    /* Intialise serial comms */
    //vSerial_Init();

    /* Start BOS */
    (void)bBosRun(TRUE);

    /* No return from the above function call */
}

/****************************************************************************
 *
 * NAME: vInitSensors
 *
 * DESCRIPTION:
 * Initialise the temperature/humidity sensor and set the ADC to measure the
 * supply voltage.
 *
 ****************************************************************************/
PRIVATE void vInitSensors(void)
{
    /* Initialise temp/humidity sensor interface */
    vHTSreset();
    sTempHumiditySensor.eState = E_STATE_READ_TEMP_HUMID_IDLE;

    /* Initialise ADC for internal battery voltage measurement */
	sBattSensor.eState = E_STATE_READ_BATT_VOLT_IDLE;

	vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE,
	                 E_AHI_AP_INT_DISABLE,
	                 E_AHI_AP_SAMPLE_2,
	                 E_AHI_AP_CLOCKDIV_2MHZ,
	                 E_AHI_AP_INTREF);

    /* Wait until the analogue peripheral regulator has come up before setting
       the ADC. */
    while(!bAHI_APRegulatorEnabled());

    vAHI_AdcEnable(E_AHI_ADC_CONVERT_DISABLE,
                   E_AHI_AP_INPUT_RANGE_2,
                   E_AHI_ADC_SRC_VOLT);
}

/****************************************************************************
 *
 * NAME: vAppTick
 *
 * DESCRIPTION:
 *
 * Called by a BOS timer expiry. Reads sensor data and if complete transmits
 * to coordinator.
 *
 ****************************************************************************/
/*PRIVATE void vAppTick(void *pvMsg, uint8 u8Param)
{
    uint8 u8Msg;
    uint8 u8TimerId;
    static bool_t bToggle;

	/* Read sensor data */
	/*vReadTempHumidity();
	vReadBatteryVoltage();

	if (u8AppTicks++ > APP_DATA_SEND_PERIOD)
	{
	    /* If sensor reads are compete */
	 /*   if ((sBattSensor.eState         == E_STATE_READ_BATT_VOLTS_READY) &&
            (sTempHumiditySensor.eState == E_STATE_READ_TEMP_HUMID_READY))
        {
		    /* Toggle LED1 to show we are alive */
		/*    if (bToggle)
		    {
		    	vLedControl(0,0);
		    }
		    else
		    {
		    	vLedControl(0,1);
		    }
		    bToggle = !bToggle;

		    u8AppTicks = 0;

            /* Transmit data to coordinator */
		   // vSendData();

         /*  	sBattSensor.eState         = E_STATE_READ_BATT_VOLT_IDLE;
            sTempHumiditySensor.eState = E_STATE_READ_TEMP_HUMID_IDLE;
		}
	}
    (void)bBosCreateTimer(vAppTick, &u8Msg, 0, (APP_TICK_PERIOD_ms / 10), &u8TimerId);
}*/
PRIVATE void vAppTick(void *pvMsg, uint8 u8Param)
{
     uint8 u8Msg;
     uint8 u8TimerId;
    (void)bBosCreateTimer(vAppTick, &u8Msg, 0, (APP_TICK_PERIOD_ms / 10), &u8TimerId);
}
/****************************************************************************
 *
 * NAME: vReadBatteryVoltage
 *
 * DESCRIPTION:
 *
 * Uses ADC to read supply voltage. Measurement is performed using a state
 * machine to ensure that it never blocks.
 *
 ****************************************************************************/
PRIVATE void vReadBatteryVoltage(void)
{
    uint16 u16AdcReading;

	switch(sBattSensor.eState)
	{
		case E_STATE_READ_BATT_VOLT_IDLE:
	    	vAHI_AdcStartSample();
	    	sBattSensor.eState = E_STATE_READ_BATT_VOLTS_ADC_CONVERTING;
			break;

		case E_STATE_READ_BATT_VOLTS_ADC_CONVERTING:
	    	if (!bAHI_AdcPoll())
	    	{
	    	    sBattSensor.eState = E_STATE_READ_BATT_VOLTS_COMPLETE;
	    	}
			break;

		case E_STATE_READ_BATT_VOLTS_COMPLETE:

		    u16AdcReading = u16AHI_AdcRead();

		    /* Input range is 0 to 2.4V. ADC has full scale range of 12 bits.
		       Therefore a 1 bit change represents a voltage of approx 586uV */
		    sBattSensor.u16Reading = ((uint32)((uint32)(u16AdcReading * 586) +
                                     ((uint32)(u16AdcReading * 586) >> 1)))  /
                                     1000;

	    	sBattSensor.eState = E_STATE_READ_BATT_VOLTS_READY;
			break;

		case E_STATE_READ_BATT_VOLTS_READY:
			break;

		default:
			break;
	}
}

/****************************************************************************
 *
 * NAME: vReadTempHumidity
 *
 * DESCRIPTION:
 *
 * Read temperature/humidity sensor. Reading is performed using a state machine
 * to ensure that it never blocks.
 *
 ****************************************************************************/
PRIVATE void vReadTempHumidity(void)
{
    switch(sTempHumiditySensor.eState)
	{
		case E_STATE_READ_TEMP_HUMID_IDLE:
		    vHTSstartReadHumidity();
			sTempHumiditySensor.eState = E_STATE_READ_HUMID_RUNNING;
		break;

		case E_STATE_READ_HUMID_RUNNING:
			if ((u32AHI_DioReadInput() & HTS_DATA_DIO_BIT_MASK) == 0)
			{
				sTempHumiditySensor.eState = E_STATE_READ_TEMP_HUMID_COMPLETE;
			}
			break;

		case E_STATE_READ_TEMP_HUMID_COMPLETE:
			sTempHumiditySensor.u16HumidReading = u16HTSreadHumidityResult();
			sTempHumiditySensor.eState     = E_STATE_READ_TEMP_START;
			break;

		case E_STATE_READ_TEMP_START:
		    vHTSstartReadTemp();
			sTempHumiditySensor.eState = E_STATE_READ_TEMP_HUMID_RUNNING;
			break;

		case E_STATE_READ_TEMP_HUMID_RUNNING:
			if ((u32AHI_DioReadInput() & HTS_DATA_DIO_BIT_MASK) == 0)
			{
				sTempHumiditySensor.eState = E_STATE_READ_TEMP_COMPLETE;
			}
			break;

		case E_STATE_READ_TEMP_COMPLETE:
			sTempHumiditySensor.u16TempReading = u16HTSreadTempResult();
			sTempHumiditySensor.eState     = E_STATE_READ_TEMP_HUMID_READY;
			break;

		case E_STATE_READ_TEMP_HUMID_READY:
			break;

		default:
			break;
	}
}

/****************************************************************************
* name: vUART_Init
*
* Description:
* Initiate UART
*
* Parameter:
* void
*
* Return
* void
*
****************************************************************************/


PRIVATE void vUART_Init(void)
{
    /* Enable UART 0 */
    vAHI_UartEnable(UART);

    vAHI_UartReset(UART, TRUE, TRUE);
    vAHI_UartReset(UART, FALSE, FALSE);

    /* Set the clock divisor register to give required buad, this has to be done
       directly as the normal routines (in ROM) do not support all baud rates */
    vUART_SetBuadRate(UART_BAUD_RATE);

    vAHI_UartSetControl(UART, FALSE, FALSE, E_AHI_UART_WORD_LEN_8, TRUE, FALSE);
    vAHI_UartSetInterrupt(UART, FALSE, FALSE, TRUE, TRUE, E_AHI_UART_FIFO_LEVEL_1);
}


/****************************************************************************
* name: vUART_SetBuadRate
*
* Description:
* Set Buad Rate
*
* Parameter:
* name        RW USAGE
* u32BaudRate R  Buad Rate
*
* Return
* void
*
****************************************************************************/


PRIVATE void vUART_SetBuadRate(uint32 u32BaudRate)
{
    uint8 *pu8Reg;
    uint8  u8TempLcr;
    uint16 u16Divisor;
    uint32 u32Remainder;

    /* Put UART into clock divisor setting mode */
    pu8Reg    = (uint8 *)(UART_START_ADR + UART_LCR_OFFSET);
    u8TempLcr = *pu8Reg;
    *pu8Reg   = u8TempLcr | 0x80;

    /* Write to divisor registers:
       Divisor register = 16MHz / (16 x baud rate) */
    u16Divisor = (uint16)(16000000UL / (16UL * u32BaudRate));

    /* Correct for rounding errors */
    u32Remainder = (uint32)(16000000UL % (16UL * u32BaudRate));

    if (u32Remainder >= ((16UL * u32BaudRate) / 2))
    {
        u16Divisor += 1;
    }

    pu8Reg  = (uint8 *)UART_START_ADR;
    *pu8Reg = (uint8)(u16Divisor & 0xFF);
    pu8Reg  = (uint8 *)(UART_START_ADR + UART_DLM_OFFSET);
    *pu8Reg = (uint8)(u16Divisor >> 8);

    /* Put back into normal mode */
    pu8Reg    = (uint8 *)(UART_START_ADR + UART_LCR_OFFSET);
    u8TempLcr = *pu8Reg;
    *pu8Reg   = u8TempLcr & 0x7F;
}

/****************************************************************************
 *
 * NAME: vSendData
 *
 * DESCRIPTION:
 *
 * Transmit sensor data to coordinator.
 *
 ****************************************************************************/
PRIVATE void vSendData(void)
{
    AFDE_DATA_REQ_INFO  asAfdeDataReq[1];
    AF_ADDRTYPE         hDstAddr;
   // uint8               au8Afdu[6];
    uint8               au8Afdu[1];

    hDstAddr.hAddrMode  = DEV_16BIT_ADDR;
    hDstAddr.u16Address = 0x0000;
    hDstAddr.u8EndPoint = WSN_DATA_SINK_ENDPOINT;

    asAfdeDataReq[0].u8SequenceNum = ais.u8AfTransactionSequence++;
    //asAfdeDataReq[0].u8DividedAfduLen = 6;
    asAfdeDataReq[0].u8DividedAfduLen = 1;

    au8Afdu[0] = cCharIn;
    /*au8Afdu[1] = 0;

    au8Afdu[2] = sTempHumiditySensor.u16TempReading;
	au8Afdu[3] = sTempHumiditySensor.u16TempReading >> 8;
    au8Afdu[4] = sTempHumiditySensor.u16HumidReading;
	au8Afdu[5] = sTempHumiditySensor.u16HumidReading >> 8;*/

    afdeDataRequest(hDstAddr,                   /* Destination address */
                    WSN_DATA_SOURCE_ENDPOINT,   /* Source endpoint */
                    WSN_PROFILE_ID,             /* Profile ID */
                    WSN_CID_SENSOR_READINGS,    /* Cluster ID */
                    MSG,                        /* Frame type */
                    1,                          /* Transactions */
                    asAfdeDataReq,              /* Transaction info */
                    au8Afdu,                    /* Transaction data */
                    APS_TXOPTION_NONE,          /* Transmit options */
                    SUPPRESS_ROUTE_DISCOVERY,   /* Route discovery mode */
                    0);                         /* Radius count */
}

/****************************************************************************
 *
 * NAME: vTimer0Config
 *
 * DESCRIPTION:
 * PWM output from timer0 in Delta Sigma mode
 * with a low period of 0x8000 and a high of (0xffff - 0x8000)
 *
 * PWM output from timer1 in standard PWM mode
 * with a low period of 0x8000 and a high of (0xffff - 0x8000)
 *
 * PARAMETERS:      Name            RW  Usage
 * None.
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * In this mode the period of timer0 is fixed
 * at 2^16 or 2^17 and the u16Lo the SPACE period
 ****************************************************************************/
PRIVATE void vTimer0Config(void)
{

/* set up timer 0 for PWM */
   vAHI_TimerEnable(E_AHI_TIMER_0,
                     0x00,    // bu fen pin
                     FALSE,   // dusable interrupts when output is high
                     FALSE,   // disable interrupts when period completed and output goes low
                     TRUE);   // PWM enable
    vAHI_TimerClockSelect(E_AHI_TIMER_0,
                          FALSE,   // use internal 16MHZ clock
                          TRUE);   // gate the output pin when the gate input is high?
    vAHI_TimerStartRepeat(E_AHI_TIMER_0,
                         // 0x8000,       // low period (space)
                          //0xffff
                             0x00d2,     //Low 210
                             0x01a4      //period 240
                          );      // period  38kHz
/* set up timer 1 for interrupt */
  /* vAHI_TimerEnable(E_AHI_TIMER_1,
                    0x00,
                    FALSE,
                    TRUE, // enable interrupts when period completed and ?
                    FALSE);  //PWM enable
   vAHI_TimerClockSelect(E_AHI_TIMRE_1,
                         FALSE, //use internal 16MHZ clock
                         TRUE); //?
*/
}
/***************************************************************

***************************************************************/
PRIVATE void vTimer1Config(void)
    {
/* set up timer 0 for capture */
        vAHI_TimerEnable(E_AHI_TIMER_1,
                         0x00,
                         FALSE,
                         FALSE,
                         FALSE);
        vAHI_TimerClockSelect(E_AHI_TIMER_1,
                              FALSE,
                              FALSE);
    }
PRIVATE void vTimer1ReadCapture()
{

      vAHI_TimerStartCapture(E_AHI_TIMER_1);
      /* wait for capture complete */
      while((u8AHI_TimerFired(E_AHI_TIMER_1) & E_AHI_TIMER_INT_PERIOD) == FALSE)
      {
      }
      vAHI_TimerReadCapture(E_AHI_TIMER_1,
                              &u16Hi,
                              &u16Lo);
      if(u16Lo > u16Hi)   /* have we rolled over */
        {
            i32Data = u16Lo - u16Hi ;
        }
        else
        {
            i32Data = 0x10000 + u16Lo - u16Hi ;
        }
}
/*void vDio2_Output(uint8 Rec_Char)
{
   vAHI_DioSetDirection(0x00,0x04);  //set DIO2 as output
//   output=0;   // initial output : low

}

// TIMER 1 interrupt to toggle DIO2 output

PUBLIC void vAHI_Timer1RegisterCallback (PR_HWINT_APPCALLBACK PrTimer1Callback)
{

if(output==1)
   vAHI_DioSetOutput(0x04,0x00); // set DIO2 output on
   else vAHI_DioSetOutput(0x00,0x04); // set DIO2 output off

   output=~output;
}*/

/*PRIVATE void vTimer1Config(void)
{

/* set up timer 0 for PWM */
   /*vAHI_TimerEnable(E_AHI_TIMER_1,
                     0x00,    // bu fen pin
                     FALSE,   // dusable interrupts when output is high
                     FALSE,   // disable interrupts when period completed and output goes low
                     TRUE);   // PWM enable
    vAHI_TimerClockSelect(E_AHI_TIMER_1,
                          FALSE,   // use internal 16MHZ clock
                          TRUE);   // gate the output pin when the gate input is high?
    vAHI_TimerStartRepeat(E_AHI_TIMER_1,
                         // 0x8000,       // low period (space)
                          //0xffff
                             //0x00d2,     //Low 210
                             //0x01a4      //period 240
                             time_rise,
                             time_down
                          );      // period  38kHz
/* set up timer 1 for interrupt */
  /* vAHI_TimerEnable(E_AHI_TIMER_1,
                    0x00,
                    FALSE,
                    TRUE, // enable interrupts when period completed and ?
                    FALSE);  //PWM enable
   vAHI_TimerClockSelect(E_AHI_TIMRE_1,
                         FALSE, //use internal 16MHZ clock
                         TRUE); //?
*/
//}
/*
PRIVATE void TIMER1_INT_SET(void)
{
/* set up timer 1 for interrupt */
  /*vAHI_TimerEnable(E_AHI_TIMER_1,
                    0x00, //yu fen pin
                    FALSE, //bIntRiseEnable
                    TRUE, // enable interrupts when period completed and ?
                    FALSE);  //PWM enable
   vAHI_TimerClockSelect(E_AHI_TIMER_1,
                         FALSE, //use internal 16MHZ clock
                         TRUE); //?
   repeat_count=128;
   time_half=time_last/2;
   vAHI_TimerStartRepeat(E_AHI_TIMER_1,time_half,time_last);
   vAHI_Timer1RegisterCallback (vTimer1ISR);
}*/

/***************************************************************

***************************************************************/
void Init_DIO2 (void)
{
   vAHI_DioSetDirection(0x00,0x04);  //set DIO2 as output
   vAHI_DioSetOutput(0x00,0x04);     // initial output : low
}

PRIVATE void vTickTimerConfig(void)
{
/* set up Tick Timer to interrupt */
    vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_DISABLE);
    /* vTickTimerISR is the function that is called when we get a TickTimer Interrupt */
    vAHI_TickTimerInit(vTickTimerISR);
    vAHI_TickTimerWrite(0);
    vAHI_TickTimerInterval(time_rise); // for 10s, modified!
    vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_RESTART);
    vAHI_TickTimerIntEnable(TRUE);
}

/***************************************************************

***************************************************************/

// TIMER 1 interrupt to toggle DIO2 output

PRIVATE void vTickTimerISR(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
if(count<512)
{
 if(output==0)
     vAHI_DioSetOutput(0x04,0x00); // set DIO2 output on
  else
     vAHI_DioSetOutput(0x00,0x04); // set DIO2 output off
  output=~output;
  //time_last_sign=0;
  count=count+4;
  time_rise=data[count]+data[count+1]*256+data[count+2]*256*256+data[count+3]*256*256*256;
  //printf("\n\rtime_rise: %x", time_rise);
  time_rise=2*time_rise;//8M==>16M
  vTickTimerConfig();
}
/*else
{
if(time_last_sign==0)
 {
 time_last=data[count]+data[count+1]*256;
 time_last_sign=1;
 time_count=data[count+2]+data[count+3]*256;
 time_half=time_last/2;
 vAHI_TimerStartRepeat(E_AHI_TIMER_1,time_half,time_last);
 }
 else if(time_count>0)
 {
 time_last=65536;
 time_count=repeat_count-1;
 time_half=time_last/2;
 vAHI_TimerStartRepeat(E_AHI_TIMER_1,time_half,time_last);
 }
 else if(time_count=0)
 {
 if(output==1)
     vAHI_DioSetOutput(0x04,0x00); // set DIO2 output on
  else
     vAHI_DioSetOutput(0x00,0x04); // set DIO2 output off
  output=~output;
  time_last_sign=0;
  count=count+4;
  time_last=data[count]+data[count+1]*256;
  time_half=time_last/2;
  vAHI_TimerStartRepeat(E_AHI_TIMER_1,time_half,time_last);
 }
}
}*/
else
{
      //vAHI_TickTimerConfigure(E_AHI_TICK_TIMER_DISABLE);
          vAHI_TickTimerIntEnable(FALSE);
}
}

/****************************************************************************/
/***               Functions called by the stack                          ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: JZA_vZdpResponse
 *
 * DESCRIPTION:
 * Called when a ZDP response frame has been received. In this application no
 * action is taken as no ZDP responses are anticipated.
 *
 * PARAMETERS:      Name           RW  Usage
 *                  u8Type         R   ZDP response type
 *                  pu8Payload     R   Payload buffer
 *                  u8PayloadLen   R   Length of payload
 *
 ****************************************************************************/
PUBLIC void JZA_vZdpResponse(uint8  u8Type,
                             uint8  u8LQI,
                             uint8 *pu8Payload,
                             uint8  u8PayloadLen)

{
}

PRIVATE void data_copy(uint8 *DstData,uint8 *SrcData)
{
    static uint16 num_count=0;
    for(num_count=0;num_count<512; num_count++)
    {
        *(DstData+num_count)=*(SrcData+num_count);
    }
}

/****************************************************************************
 *
 * NAME: JZA_pu8AfMsgObject
 *
 * DESCRIPTION:
 * Called when a MSG transaction has been received with a matching endpoint.
 * In this application no action is taken as no MSG transactions are expected.
 *
 * PARAMETERS:      Name           RW  Usage
 *                  afSrcAddr      R   Address of sender device
 *                  dstEndPoint    R   Endpoint at receiver
 *                  clusterID      R   Pointer to cluster ID
 *                  afduLength     R   Pointer to length of data
 *                  pAfdu          R   Data array
 *
 * RETURNS:
 * NULL
 *
 ****************************************************************************/
PUBLIC uint8 JZA_u8AfMsgObject(AF_ADDRTYPE sAfSrcAddr,
                               uint8       u8ClusterID,
                               uint8       u8DstEndPoint,
                               uint8       u8LQI,
                               uint8      *pau8AfduInd,
                               uint8      *pu8ClusterIDRsp,
                               uint8      *pau8AfduRsp)
{
      /*printf("\n\r\n\renter rec function!");
      printf("\n\rsAfSrcAddr.hAddrMode: %x",sAfSrcAddr.hAddrMode);
      printf("\n\rDEV_16BIT_ADDR: %x",DEV_16BIT_ADDR);
      printf("\n\ru8DstEndPoint: %x",u8DstEndPoint);
      printf("\n\rWSN_DATA_SOURCE_ENDPOINT: %x",WSN_DATA_SOURCE_ENDPOINT);
      printf("\n\ru8ClusterID: %x",u8ClusterID);
      printf("\n\rWSN_CID_SENSOR_READINGS: %x",WSN_CID_SENSOR_READINGS);*/

     if ((sAfSrcAddr.hAddrMode == DEV_16BIT_ADDR) &&
        (u8DstEndPoint == WSN_DATA_SOURCE_ENDPOINT))
    {
        if(u8ClusterID == WSN_CID_SENSOR_READINGS)
        {
           // printf("Data Received!");
            Rec_Char=*pau8AfduInd;
            /*************************** for debugging ********************/
            if(sAfSrcAddr.u16Address==0x0000)
            {
                printf("\n\r\n\rCommand Received from Coordinator!");
                //printf("\n\rThe Command is %x\n", Rec_Char);
                //vSendData();
            }
            //printf("\n\r\n\rThe address is %x", sAfSrcAddr.u16Address);

            /***************************end for debugging ********************/

            switch(Rec_Char)
            {
             case 1:
             case 3: vLedControl(0,1); printf("\n\r\n\rTurn on LED1!");
              //data_copy(data,data1);break;
                    for(num_count=0;num_count<512; num_count++)
                    {
                     *(data+num_count)=*(data1+num_count);
                    }
                    break;
             case 5:
             case 7: vLedControl(0,0); printf("\n\r\n\rTurn off LED1!");
             //data_copy(data,data2);break;
                    for(num_count=0;num_count<512; num_count++)
                    {
                     *(data+num_count)=*(data2+num_count);
                    }
                    break;
             case 2:
             case 4: vLedControl(1,1); printf("\n\r\n\rTurn on LED2!");
             //data_copy(data,data3);break;
                     for(num_count=0;num_count<512; num_count++)
                    {
                     *(data+num_count)=*(data3+num_count);
                    }
                    break;
             case 6:
             case 8: vLedControl(1,0); printf("\n\r\n\rUTurn off LED2!");
             //data_copy(data,data4); break;
                     for(num_count=0;num_count<512; num_count++)
                    {
                     *(data+num_count)=*(data4+num_count);
                    }
                    break;

             default: printf("\n\r\n\rUndefined command!"); break;
            }

               //************************************
            if(Rec_Char<(MAX_COMMAND+1) && (Rec_Char>0)) // if receive a command from coordinator
              {
               vTimer0Config();  //TIMER0 PWM output: 38kHz*/ ok
               Init_DIO2();
               //vTimer1Config();  //TIMER1 ok
               //for(count=4;count<512;count=count+8)
               count=4;
               time_rise=data[count]+data[count+1]*256+data[count+2]*256*256+data[count+3]*256*256*256;
               //printf("\n\rtime_rise: %x", time_rise);
               time_rise=2*time_rise;//8M==>16M
              // printf("\n\rtime_rise_2: %x", time_rise);
               vTickTimerConfig();
               /*   for(count=4;count<512;count=count+4)
               {    // num count in data
               time_rise=data[count]+data[count+1]*256;
              //time_half=time_last/2;
               time_down=data[count+4]+data[count+5]*256;
               printf("\n\rtime_rise: %x", time_rise);
               printf("\n\rtime_down: %x", time_down);
               // above OK//
               vAHI_TimerStartSingleShot(E_AHI_TIMER_1,
                                         time_rise,
                                         0x0000
                                        );
               vAHI_TimerStartSingleShot(E_AHI_TIMER_1,
                                         0x0000,
                                         time_down
                                        );
               /*if(time_down_temp>65535)
               {
                   time_down=0xffff;
                   vAHI_TimerStartRepeat(E_AHI_TIMER_1,
                                         time_rise;
                                         time_down;
                                        );
               }
               else
               {
               vAHI_TimerStartRepeat(E_AHI_TIMER_1,
                                     time_rise;
                                     time_down_temp;
                                    );
               }
               }

              // TIMER1_INT_SET();
              }*/
                //************************************

        }
    }
}

    return 0;
}

/****************************************************************************
 *
 * NAME: JZA_vAfKvpResponse
 *
 * DESCRIPTION:
 * Called after a KVP transaction with acknowledgement request, when the
 * acknowledgement arrives. In this application no action is taken as no
 * KVP transaction acknowledgements are expected.
 *
 * PARAMETERS:      Name                   RW  Usage
 *                  srcAddressMod          R   Address of sender device
 *                  transactionSequenceNum R   KVP transaction number
 *                  commandTypeIdentifier  R   KVP command type
 *                  dstEndPoint            R   Endpoint at receiver
 *                  clusterID              R   Cluster ID
 *                  attributeIdentifier    R   KVP attribute ID
 *                  errorCode              R   Result code
 *                  afduLength             R   Length of payload data
 *                  pAfdu                  R   Payload data array
 *
 ****************************************************************************/
PUBLIC void JZA_vAfKvpResponse(AF_ADDRTYPE         srcAddressMod,
                               uint8               u8LQI,
                               uint8               transactionSequenceNum,
                               AF_COMMAND_TYPE_ID  commandTypeIdentifier,
                               uint8               dstEndPoint,
                               uint8               clusterID,
                               uint16              attributeIdentifier,
                               uint8               errorCode,
                               uint8               afduLength,
                               uint8              *pAfdu )
{
}

/****************************************************************************
 *
 * NAME: JZA_eAfKvpObject
 *
 * DESCRIPTION:
 * Called when a KVP transaction has been received with a matching endpoint.
 *
 * PARAMETERS:      Name           RW  Usage
 *                  afSrcAddr      R   Address of sender device
 *                  u8DstEndpoint  R   Endpoint at receiver
 *                  pu8ClusterId   R   Pointer to cluster ID
 *                  eCommandTypeId R   KVP command type
 *                  u16AttributeId R   KVP attribute ID
 *                  pu8AfduLength  R   Pointer to length of data
 *                  pu8Afdu        R   Data array
 *
 * RETURNS:
 * AF_ERROR_CODE
 *
 ****************************************************************************/
PUBLIC AF_ERROR_CODE JZA_eAfKvpObject(AF_ADDRTYPE         afSrcAddr,
                                      uint8               u8LQI,
                                      uint8               u8DstEndpoint,
                                      uint8               u8SequenceNum,
                                      uint8              *pu8ClusterId,
                                      AF_COMMAND_TYPE_ID  eCommandTypeId,
                                      uint16              u16AttributeId,
                                      uint8              *pu8AfduLength,
                                      uint8              *pu8Afdu)
{
    return KVP_SUCCESS;
}

/****************************************************************************
 *
 * NAME: JZA_vAppDefineTasks
 *
 * DESCRIPTION:
 * Called by Zigbee stack during initialisation to allow the application to
 * initialise any tasks that it requires. This application requires none.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void JZA_vAppDefineTasks(void)
{
}

/****************************************************************************
 *
 * NAME: JZA_vPeripheralEvent
 *
 * DESCRIPTION:
 * Called when a hardware event causes an interrupt. This function is called
 * from within the interrupt context so should be brief. In this case, the
 * information is placed on a simple FIFO queue to be processed later.
 *
 * PARAMETERS: Name          RW  Usage
 *             u32Device     R   Peripheral generating interrupt
 *             u32ItemBitmap R   Bitmap of interrupt sources within peripheral
 *
 ****************************************************************************/
PUBLIC void JZA_vPeripheralEvent(uint32 u32Device, uint32 u32ItemBitmap)
{


    if (u32Device == E_AHI_DEVICE_UART0)
        {
            /* If data has been received */
    if ((u32ItemBitmap & 0x000000FF) == E_AHI_UART_INT_RXDATA)
            {
                cCharIn = ((u32ItemBitmap & 0x0000FF00) >> 8);
                vSendData();
            }
            else if (u32ItemBitmap== E_AHI_UART_INT_TX)
            {
                vUART_TxCharISR();
            }
        }
}

/****************************************************************************
 *
 * NAME: JZA_vAppEventHandler
 *
 * DESCRIPTION:
 * Called regularly by the task scheduler. This function reads the hardware
 * event queue and processes the events therein. It is important that this
 * function exits after a relatively short time so that the other tasks are
 * not adversely affected.
 *
 ****************************************************************************/
PUBLIC void JZA_vAppEventHandler(void)
{
    uint8 u8Msg;
    uint8 u8TimerId;

    if (!bAppTimerStarted)
    {
        if (bNwkJoined)
        {
            bAppTimerStarted = TRUE;
            (void)bBosCreateTimer(vAppTick, &u8Msg, 0, (APP_TICK_PERIOD_ms / 10), &u8TimerId);
        }
    }
}

/****************************************************************************
 *
 * NAME: JZA_boAppStart
 *
 * DESCRIPTION:
 * Called by Zigbee stack during initialisation.
 *
 * RETURNS:
 * TRUE
 *
 ****************************************************************************/
PUBLIC bool_t JZA_boAppStart(void)
{
    JZS_vStartStack();
    return TRUE;
}

/****************************************************************************
 *
 * NAME: JZA_vStackEvent
 *
 * DESCRIPTION:
 * Called by Zigbee stack to pass an event up to the application.
 *
 * RETURNS:
 * TRUE
 *
 ****************************************************************************/
PUBLIC void JZA_vStackEvent(teJZS_EventIdentifier eEventId,
                            tuJZS_StackEvent *puStackEvent)
{
    if (eEventId == JZS_EVENT_NWK_JOINED_AS_ROUTER)
    {

    uint8 u8InputClusterCnt      = 1;
    uint8 au8InputClusterList[]  = {WSN_CID_SENSOR_READINGS};
    uint8 u8OutputClusterCnt     = 1;
    uint8 au8OutputClusterList[] = {WSN_CID_SENSOR_READINGS};

    (void)afmeAddSimpleDesc(WSN_DATA_SOURCE_ENDPOINT,
                            WSN_PROFILE_ID,
                            0x0000,
                            0x00,
                            0x00,
                            u8InputClusterCnt,
                            au8InputClusterList,
                            u8OutputClusterCnt,
                            au8OutputClusterList);

        bNwkJoined = TRUE;
        vSendData();
    }
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
