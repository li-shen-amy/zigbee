/****************************************************************************
 *
 * MODULE:			  Coordinator
 *
 * COMPONENT:          $RCSfile: Coordinator.c,v $
 *
 * VERSION:            $Name:  $
 *
 * REVISION:           $Revision:  $
 *
 * DATED:              $Date: 2009/9/25  $
 *
 * STATUS:             $State: $
 *
 * AUTHOR:             Li
 *
 * DESCRIPTION:
 *
 * Implements a Network Coordinator Node using Jennic Zigbee
 * stack. Receives data from host using UART, and retransmits to compatible nodes via the radio.
 *
 * LAST MODIFIED BY:   $Author: Li $
 *                     $Modtime: $
 *
 ****************************************************************************


/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <LedControl.h>
#include <AppHardwareApi.h>
#include <Utilities.h>
#include <JZ_Api.h>

#include "printf.h"
#include "uart.h"
#include "WSN_Profile.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
/* Timing values */
#define APP_TICK_PERIOD_ms			 500

#define UART                    E_AHI_UART_0
#define UART_BAUD_RATE          115200

#if UART == E_AHI_UART_0
    #define UART_START_ADR   0x30000000UL
#else
    #define UART_START_ADR   0x40000000UL
#endif

#define UART_LCR_OFFSET  0x0C
#define UART_DLM_OFFSET  0x04

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vInit(void);
PRIVATE void vToggleLed(void *pvMsg, uint8 u8Dummy);
PRIVATE void vUART_Init(void);
PRIVATE void vUART_SetBuadRate(uint32 u32BaudRate);
//PRIVATE void vSendData(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE bool_t bNwkStarted = FALSE;
PRIVATE bool_t bAppTimerStarted = FALSE;
PRIVATE uint8 cCharIn;
PRIVATE uint8 site;
PRIVATE uint8 command;
PRIVATE uint16 addr_site2=0x143e;

/****************************************************************************
 *
 * NAME: AppColdStart
 *
 * DESCRIPTION:
 * Entry point for application from boot loader. Initialises system and runs
 * main loop.
 *
 * RETURNS:
 * Never returns.
 *
 ****************************************************************************/
PUBLIC void AppColdStart(void)
{
	/* Set network information */
	JZS_sConfig.u32Channel = WSN_CHANNEL;
	JZS_sConfig.u16PanId   = WSN_PAN_ID;

    /* General initialisation */
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
    vLedInitFfd();
    vLedControl(0,0);
    vLedControl(1,0);


    /* set uart */
    vUART_Init();

    /* Start BOS */
    (void)bBosRun(TRUE);

    /* No return from the above function call */
}

/****************************************************************************
 *
 * NAME: vToggleLed
 *
 * DESCRIPTION:
 * Gets called by a BOS timer. Toggles LED1 to indicate we are alive.
 *
 ****************************************************************************/
PRIVATE void vToggleLed(void *pvMsg, uint8 u8Dummy)
{
    uint8 u8Msg;
    uint8 u8TimerId;
    static bool_t bToggle;

	if (bToggle)
	{
        vLedControl(0,0);
	}
    else
	{
        vLedControl(0,1);
	}
    bToggle = !bToggle;

    (void)bBosCreateTimer(vToggleLed, &u8Msg, 0, (APP_TICK_PERIOD_ms / 10), &u8TimerId);
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
 * Transmit sensor data to router.
 *
 ****************************************************************************/
PRIVATE void vSendData(void)
{
    AFDE_DATA_REQ_INFO  asAfdeDataReq[1];
    AF_ADDRTYPE         hDstAddr;
    uint8               au8Afdu[1];

    hDstAddr.hAddrMode  = DEV_16BIT_ADDR;
   // printf("\n\r\n\rcommand is %d",cCharIn);
    switch(cCharIn)
    {
      case 1:
      case 5:
      case 2:
      case 6: hDstAddr.u16Address=0x0001; /*printf("\n\r\n\rto 0x0001"); */ break;
      case 3:
      case 7:
      case 4:
      case 8: hDstAddr.u16Address=addr_site2; /*printf("\n\r\n\rto %x",addr_site2);*/ break;
      default: break;
    }
    hDstAddr.u8EndPoint = WSN_DATA_SOURCE_ENDPOINT;

    asAfdeDataReq[0].u8SequenceNum = ais.u8AfTransactionSequence++;
    asAfdeDataReq[0].u8DividedAfduLen = 1;

    au8Afdu[0] = cCharIn;

    afdeDataRequest(hDstAddr,                   /* Destination address */
                    WSN_DATA_SINK_ENDPOINT,   /* Source endpoint */
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

/****************************************************************************/
/***               Functions called by the stack                          ***/
/****************************************************************************/

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
void JZA_vAppEventHandler(void)
{
    uint8 u8Msg;
    uint8 u8TimerId;

    if (!bAppTimerStarted)
    {
        if (bNwkStarted)
        {
            bAppTimerStarted = TRUE;
            (void)bBosCreateTimer(vToggleLed, &u8Msg, 0, (APP_TICK_PERIOD_ms / 10), &u8TimerId);
        }
    }
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
                switch(cCharIn)
                {
                    case 1:
                    case 5: site=0x01; command=0x01; break;
                    case 2:
                    case 6: site=0x01; command=0x02; break;
                    case 3:
                    case 7: site=0x02; command=0x01; break;
                    case 4:
                    case 8: site=0x02; command=0x02; break;
                    default: break;
                }
                if(cCharIn<0x05)
               {
                   printf("\n\r\n\rTo Site %x, Turn on LED %x ",site,command);
                }
                else if(cCharIn<0x09)
                {
                    printf("\n\r\n\rTo Site %x, Turn off LED %x ",site,command);
                }
                else
                {
                    printf("\n\r\n\rUndefined command!");
                }
                vSendData();
            }
            else if(u32ItemBitmap==E_AHI_UART_INT_TX)
            {
                vUART_TxCharISR();
            }
        }

   static bool_t bToggle_1;

	if (bToggle_1)
	{
        vLedControl(1,0);
	}
    else
	{
        vLedControl(1,1);
	}
    bToggle_1 = !bToggle_1;


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
 * NAME: JZA_boAppStart
 *
 * DESCRIPTION:
 * Called by Zigbee stack during initialisation. Sets up the profile
 * information and starts the networking activity
 *
 * RETURNS:
 * TRUE
 *
 ****************************************************************************/
PUBLIC bool_t JZA_boAppStart(void)
{
    uint8 u8InputClusterCnt      = 1;
    uint8 au8InputClusterList[]  = {WSN_CID_SENSOR_READINGS};
    uint8 u8OutputClusterCnt     = 1;
    uint8 au8OutputClusterList[] = {WSN_CID_SENSOR_READINGS};

    (void)afmeAddSimpleDesc(WSN_DATA_SINK_ENDPOINT,
                            WSN_PROFILE_ID,
                            0x0000,
                            0x00,
                            0x00,
                            u8InputClusterCnt,
                            au8InputClusterList,
                            u8OutputClusterCnt,
                            au8OutputClusterList);

    JZS_vStartStack();

    return TRUE;
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
 * NAME: JZA_pu8AfMsgObject
 *
 * DESCRIPTION:
 * Called when a MSG transaction has been received with a matching endpoint.
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
	/*uint16 u16BattVoltage;
	uint16 u16Temperature;
	uint16 u16Humidity;*/
        uint8 Rec_Char;

    if ((sAfSrcAddr.hAddrMode == DEV_16BIT_ADDR) &&
        (u8DstEndPoint == WSN_DATA_SINK_ENDPOINT))
    {
        if(u8ClusterID == WSN_CID_SENSOR_READINGS)
        {
           /* u16BattVoltage  = *(pau8AfduInd + 1);
            u16BattVoltage  = u16BattVoltage << 8;
            u16BattVoltage |= *pau8AfduInd;

            u16Temperature  = *(pau8AfduInd + 3);
            u16Temperature  = u16Temperature << 8;
            u16Temperature |= *(pau8AfduInd + 2);

            u16Humidity  = *(pau8AfduInd + 5);
            u16Humidity  = u16Humidity << 8;
            u16Humidity |= *(pau8AfduInd + 4);*/

            Rec_Char=*pau8AfduInd;
            if(sAfSrcAddr.u16Address!=0x0001)
            {
                addr_site2=sAfSrcAddr.u16Address;
            }

            printf("\n\r\n\rDevice Address %x Jointed!", sAfSrcAddr.u16Address);
            //printf(" JoinedThe Command is %d", Rec_Char);


           /* vTxSerialDataFrame(sAfSrcAddr.u16Address,
                               u16Humidity,
                               u16Temperature,
                               u16BattVoltage);*/
        }
    }
    return 0;
}

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
    if (eEventId == JZS_EVENT_NWK_STARTED)
    {
        bNwkStarted = TRUE;
    }
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
