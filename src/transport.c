/**
 *   @file  transport.c
 *
 *   @brief
 *      This file contain the UART, XMODEM transport specific funtions.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <stdio.h>
#include <string.h>

/* MMWSDK include file. */
#include <ti/drivers/qspiflash/qspiflash.h>
#include <ti/common/sys_common.h>

/* SBL internal include file. */
#include <ti/utils/sbl/include/sbl_internal.h>

/* CRC16 include file. */
#include <ti/utils/sbl/include/crc16.h>

#include <ti/drivers/canfd/canfd.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/drivers/soc/include/reg_toprcm.h>
#include <ti/drivers/soc/soc.h>
#include <include/transport.h>

/**************************************************************************
 ************************** Local Definitions *****************************
 **************************************************************************/
#define SBL_UART_LOG_BUFF_SIZE 128U
#define SBL_XMODEM_HEADER_SIZE 3U
#define SBL_XMODEM_BUFFER_SIZE 1024U
#define SBL_XMODEM_CRC_SIZE 2U
#define SBL_XMODEM_DATABUFFER_SIZE (SBL_XMODEM_BUFFER_SIZE + SBL_XMODEM_HEADER_SIZE + SBL_XMODEM_CRC_SIZE + 1U)
#define SBL_SOH 0x01U
#define SBL_STX 0x02U
#define SBL_EOT 0x04U
#define SBL_ACK 0x06U
#define SBL_NAK 0x15U
#define SBL_CAN 0x18U
#define SBL_CAN_RECV 0x20U

#define MMWDEMO_HEADER 0x7U
#define MMWDEMO_PADDING 0x8U
#define CAN_MESSAGE_MMWDEMO_MAX 0x8U /*(messge type (6) + header (1) + padding(1))*/
#define CAN_MESSAGE_MMWDEMO_HEADER 0xC1
#define CAN_MESSAGE_MMWDEMO_PADDING 0xB1
#define CAN_MSGOBJ_HEADER 0x7U
#define CAN_MSGOBJ_PADDING 0x8U

#define SBL_CANFD_DATA_MSG_ID 0x29E
#define SBL_CANFD_TERMINATE_MSG_ID 0x19E
#define BUFFER_LENGTH 4096U
#define NUM_PKT_IN_BUFF (BUFFER_LENGTH / 64)

/**************************************************************************
 *************************** Function Definitions *************************
 **************************************************************************/
static int32_t SBL_verifyCRC(uint8_t *dataBuffer, uint32_t dataLength, uint32_t verifyCRC);
static void SBL_discardInput(void);

uint8_t select_mode = 0;
// int32_t SBL_transportWrite_CAN_FD(void);

typedef enum MmwDemo_output_message_type_e
{
    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS = 1,

    /*! @brief   Range profile */
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE,

    /*! @brief   Noise floor profile */
    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Stats information */
    MMWDEMO_OUTPUT_MSG_STATS,

    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO,

    /*! @brief   Samples to calculate static azimuth/elevation heatmap, (all virtual antennas exported) */
    MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP,

    /*! @brief   temperature stats from Radar front end */
    MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS,

    MMWDEMO_OUTPUT_MSG_MAX
} MmwDemo_output_message_type;

typedef enum mmwDemo_can_message_type_e
{
    /*! @brief   List of detected points */
    CAN_MESSAGE_MMWDEMO_DETECTED_POINTS = 0xD1,
    /*! @brief   Range profile */
    CAN_MESSAGE_MMWDEMO_RANGE_PROFILE,
    /*! @brief   Noise floor profile */
    CAN_MESSAGE_MMWDEMO_NOISE_PROFILE,
    /*! @brief   Samples to calculate static azimuth  heatmap */
    CAN_MESSAGE_MMWDEMO_AZIMUT_STATIC_HEAT_MAP,
    /*! @brief   Range/Doppler detection matrix */
    CAN_MESSAGE_MMWDEMO_RANGE_DOPPLER_HEAT_MAP,
    /*! @brief   Stats information */
    CAN_MESSAGE_MMWDEMO_STATS
} mmwDemo_can_message_type;

CANFD_Handle canHandle;

CANFD_MCANInitParams mcanCfgParams[1] = {0};
CANFD_MCANBitTimingParams mcanBitTimingParams;
CANFD_MsgObjHandle txMsgObjHandle;
CANFD_MsgObjHandle rxMsgObj2Handle;
CANFD_MsgObjHandle rxMsgObj3Handle;
CANFD_MCANMsgObjCfgParams txMsgObjectParams;
CANFD_MCANMsgObjCfgParams rxMsgObject2Params;
CANFD_MCANMsgObjCfgParams rxMsgObject3Params;
CANFD_MCANFrameType frameType = CANFD_MCANFrameType_FD;

uint8_t can_txData[64U];
uint8_t rxData[64U];

#pragma DATA_ALIGN(candataBuff, 64);
uint8_t candataBuff[BUFFER_LENGTH];

volatile uint32_t gLastMsgFlag = 0;
volatile uint32_t gRxPkts = 0, gErrStatusInt = 0, gPktsWrt = 0;
uint32_t rxDataLength;

static void sblErrStatusCallback(CANFD_Handle handle, CANFD_Reason reason,
                                 CANFD_ErrStatusResp *errStatusResp)
{
    gErrStatusInt++;

    return;
}
/* Registered callback function to receive data. */
static void sblDataCallback(CANFD_MsgObjHandle handle, CANFD_Reason reason)
{
    int32_t errCode, retVal;
    uint32_t id;
    CANFD_MCANFrameType rxFrameType;
    CANFD_MCANXidType rxIdType;

    uint32_t buffOffset = (gRxPkts % NUM_PKT_IN_BUFF);
    if (reason == CANFD_Reason_RX)
    {
        retVal = CANFD_getData(handle, &id, &rxFrameType, &rxIdType, &rxDataLength, &rxData[0], &errCode);

        if (retVal < 0)
        {
            SBL_printf("Error: CAN receive data failed [Error code %d]\n", errCode);
            return;
        }
        if (rxData[0] == 0x01 && rxData[1] == 0x01 && rxData[2] == 0x01) // sensor init ( reset )
        {
            select_mode = rxData[3];
            SBL_printf(" Mode : %d \n", select_mode);
        }

        /* Message ID for terminate message? */
        if (id == SBL_CANFD_TERMINATE_MSG_ID)
        {
            gLastMsgFlag = 1;
        }
        gRxPkts++;
    }
    return;
}

static void MCANParamInit(void)
{
    int32_t errCode, retVal;
    /*Intialize CANFD configuration parameters. */
    memset(mcanCfgParams, sizeof(CANFD_MCANInitParams), 0);
    mcanCfgParams->fdMode = 0x1U;
    mcanCfgParams->brsEnable = 0x1U;
    mcanCfgParams->txpEnable = 0x0U;
    mcanCfgParams->efbi = 0x0U;
    mcanCfgParams->pxhddisable = 0x0U;
    mcanCfgParams->darEnable = 0x1U;
    mcanCfgParams->wkupReqEnable = 0x1U;
    mcanCfgParams->autoWkupEnable = 0x1U;
    mcanCfgParams->emulationEnable = 0x0U;
    mcanCfgParams->emulationFAck = 0x0U;
    mcanCfgParams->clkStopFAck = 0x0U;
    mcanCfgParams->wdcPreload = 0x0U;
    mcanCfgParams->tdcEnable = 0x1U;
    mcanCfgParams->tdcConfig.tdcf = 0U;
    mcanCfgParams->tdcConfig.tdco = 8U;
    mcanCfgParams->monEnable = 0x0U;
    mcanCfgParams->asmEnable = 0x0U;
    mcanCfgParams->tsPrescalar = 0x0U;
    mcanCfgParams->tsSelect = 0x0U;
    mcanCfgParams->timeoutSelect = CANFD_MCANTimeOutSelect_CONT;
    mcanCfgParams->timeoutPreload = 0x0U;
    mcanCfgParams->timeoutCntEnable = 0x0U;
    mcanCfgParams->filterConfig.rrfe = 0x1U;
    mcanCfgParams->filterConfig.rrfs = 0x1U;
    mcanCfgParams->filterConfig.anfe = 0x1U;
    mcanCfgParams->filterConfig.anfs = 0x1U;
    mcanCfgParams->msgRAMConfig.lss = 127U;
    mcanCfgParams->msgRAMConfig.lse = 64U;
    mcanCfgParams->msgRAMConfig.txBufNum = 32U;
    mcanCfgParams->msgRAMConfig.txFIFOSize = 0U;
    mcanCfgParams->msgRAMConfig.txBufMode = 0U;
    mcanCfgParams->msgRAMConfig.txEventFIFOSize = 0U;
    mcanCfgParams->msgRAMConfig.txEventFIFOWaterMark = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO0size = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO0OpMode = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO0waterMark = 0U;
    mcanCfgParams->msgRAMConfig.rxFIFO1size = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO1waterMark = 64U;
    mcanCfgParams->msgRAMConfig.rxFIFO1OpMode = 64U;
    mcanCfgParams->eccConfig.enable = 1;
    mcanCfgParams->eccConfig.enableChk = 1;
    mcanCfgParams->eccConfig.enableRdModWr = 1;
    mcanCfgParams->errInterruptEnable = 1U;
    mcanCfgParams->dataInterruptEnable = 1U;
    mcanCfgParams->appErrCallBack = sblErrStatusCallback;
    mcanCfgParams->appDataCallBack = sblDataCallback;

    /* Setup the PINMUX to bring out the XWR18XX CANFD pins. */
    // Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINE14_PADAE,PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    // Pinmux_Set_FuncSel(SOC_XWR68XX_PINE14_PADAE,SOC_XWR68XX_PINE14_PADAE_CANFD_TX);
    // Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PIND13_PADAD,PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    // Pinmux_Set_FuncSel(SOC_XWR68XX_PIND13_PADAD,SOC_XWR68XX_PIND13_PADAD_CANFD_RX);

    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINE14_PADAE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINE14_PADAE, SOC_XWR68XX_PINE14_PADAE_CANFD_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PIND13_PADAD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PIND13_PADAD, SOC_XWR68XX_PIND13_PADAD_CANFD_RX);
    /* Initialize the CANFD driver. */
    canHandle = CANFD_init(0, mcanCfgParams, &errCode);
    if (canHandle == NULL)
    {
        SBL_printf("Error: CANFD Module Initialization failed [Error code %d]\n", errCode);
        return;
    }
    /*500Kbps NomBitRate: (40)/(((6+5+4)+1)*5)*/
    mcanBitTimingParams.nomBrp = 0x5U;
    mcanBitTimingParams.nomPropSeg = 0x6U;
    mcanBitTimingParams.nomPseg1 = 0x5U;
    mcanBitTimingParams.nomPseg2 = 0x4U;
    mcanBitTimingParams.nomSjw = 0x1U;
    /* Configure the bit timing parameters. */
    // mcanBitTimingParams.nomBrp = 0x2U;
    // mcanBitTimingParams.nomPropSeg = 0x8U;
    // mcanBitTimingParams.nomPseg1 = 0x6U;
    // mcanBitTimingParams.nomPseg2 = 0x5U;
    // mcanBitTimingParams.nomSjw = 0x1U;

    //// Nominal Bit rate = (40)/(((3+4+2)+1)*1) = 4Mhz
    mcanBitTimingParams.dataBrp = 0x1U;
    mcanBitTimingParams.dataPropSeg = 0x3U;
    mcanBitTimingParams.dataPseg1 = 0x4U;
    mcanBitTimingParams.dataPseg2 = 0x2U;
    mcanBitTimingParams.dataSjw = 0x1U;

    // mcanBitTimingParams.dataBrp = 0x1U;
    // mcanBitTimingParams.dataPropSeg = 0x2U;
    // mcanBitTimingParams.dataPseg1 = 0x2U;
    // mcanBitTimingParams.dataPseg2 = 0x3U;
    // mcanBitTimingParams.dataSjw = 0x1U;

    retVal = CANFD_configBitTime(canHandle, &mcanBitTimingParams, &errCode);
    if (retVal < 0)
    {
        SBL_printf("Error: CANFD Module configure bit time failed [Error code %d]\n", errCode);
        return;
    }

    /* Setup the transmit message object */
    txMsgObjectParams.direction = CANFD_Direction_TX;
    txMsgObjectParams.msgIdType = CANFD_MCANXidType_29_BIT;
    txMsgObjectParams.msgIdentifier = 0xD1;

    txMsgObjHandle = CANFD_createMsgObject(canHandle, &txMsgObjectParams, &errCode);
    if (txMsgObjHandle == NULL)
    {
        SBL_printf("Error: CANFD create Tx message object failed [Error code %d]\n", errCode);
        return;
    }

    /*********Update meta image message object has been received. Proceed with downloading
    the image and writing to flash. *******/

    /* Setup the receive message object for receiving data packets. */
    rxMsgObject2Params.direction = CANFD_Direction_RX;
    rxMsgObject2Params.msgIdType = CANFD_MCANXidType_29_BIT;
    rxMsgObject2Params.msgIdentifier = SBL_CANFD_DATA_MSG_ID;
    rxMsgObj2Handle = CANFD_createMsgObject(canHandle, &rxMsgObject2Params, &errCode);
    if (rxMsgObj2Handle == NULL)
    {
        SBL_printf("Error: CANFD create Rx message object failed [Error code %d]\n", errCode);
        return;
    }
    /* Setup the receive message object for receiving terminate packets. */
    rxMsgObject3Params.direction = CANFD_Direction_RX;
    rxMsgObject3Params.msgIdType = CANFD_MCANXidType_29_BIT;
    rxMsgObject3Params.msgIdentifier = 0xA1; // SBL_CANFD_TERMINATE_MSG_ID;
    rxMsgObj3Handle = CANFD_createMsgObject(canHandle, &rxMsgObject3Params, &errCode);
    if (rxMsgObj3Handle == NULL)
    {
        SBL_printf("Error: CANFD create Rx message object failed [Error code %d]\n", errCode);
        return;
    }
}

int32_t Can_Transmit_Schedule(uint32_t msg_id, uint8_t *txmsg, uint32_t len)
{
    volatile uint32_t index = 0;
    int32_t retVal = 0;
    int32_t errCode = 0;
    // System_printf ("MEssage %x  len %d\n", msg_id, len);
    if (frameType == CANFD_MCANFrameType_FD)
    {
        // Task_sleep(1);
        SOC_microDelay(600);
        while (len > 64U)
        {
            retVal = CANFD_transmitData(txMsgObjHandle, msg_id, CANFD_MCANFrameType_FD, 64U, &txmsg[index], &errCode);
            index = index + 64U;
            len = len - 64U;
            SOC_microDelay(600);
            // Task_sleep(1);
        }
        retVal = CANFD_transmitData(txMsgObjHandle, msg_id, CANFD_MCANFrameType_FD, len, &txmsg[index], &errCode);
    }
    else
    {
        while (len > 8U)
        {
            retVal = CANFD_transmitData(txMsgObjHandle, msg_id, CANFD_MCANFrameType_CLASSIC, 8U, &txmsg[index], &errCode);
            if (retVal < 0)
            {
                continue;
            }
            index = index + 8U;
            len = len - 8U;
        }

        retVal = CANFD_transmitData(txMsgObjHandle, msg_id, CANFD_MCANFrameType_CLASSIC, len, &txmsg[index], &errCode);

        while (retVal < 0)
        {
            // System_printf("Debug: Error transmitting CAN data %x , Errcode %x\n", retVal, errCode);
            retVal = CANFD_transmitData(txMsgObjHandle, msg_id, CANFD_MCANFrameType_CLASSIC, len, &txmsg[index], &errCode);
        }
    }
    return retVal;
}

mmwDemo_can_message_type Get_CanMessageIdentifier(MmwDemo_output_message_type type)
{
    mmwDemo_can_message_type can_msgId;

    if (type == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_DETECTED_POINTS;
    }
    else if (type == MMWDEMO_OUTPUT_MSG_RANGE_PROFILE)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_RANGE_PROFILE;
    }
    else if (type == MMWDEMO_OUTPUT_MSG_NOISE_PROFILE)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_NOISE_PROFILE;
    }
    else if (type == MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_AZIMUT_STATIC_HEAT_MAP;
    }
    else if (type == MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_RANGE_DOPPLER_HEAT_MAP;
    }
    else if (type == MMWDEMO_OUTPUT_MSG_STATS)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_STATS;
    }
    else if (type == MMWDEMO_HEADER)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_HEADER;
    }
    else if (type == MMWDEMO_PADDING)
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_PADDING;
    }
    else
    {
        can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_MAX;
    }

    if (txMsgObjectParams.msgIdType == CANFD_MCANXidType_29_BIT)
    {
        can_msgId |= ~(0xFFFFFFFF);
    }
    return can_msgId;
}

/*!
 *  @b Description
 *  @n
 *      This function is used to verify the CRC or checksum of a block of data.
 *
 *  @param[in]  dataBuffer
 *      Pointer to the data buffer.
 *  @param[in]  dataLength
 *      Size of the data buffer.
 *  @param[in]  verifyCRC
 *      Flag indicating whether CRC or checksum has to be verified.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   SBL Error code
 */
static int32_t SBL_verifyCRC(uint8_t *dataBuffer, uint32_t dataLength, uint32_t verifyCRC)
{
    int32_t retVal = MINUS_ONE;

    if (dataLength != 0)
    {
        if (verifyCRC)
        {
            uint16_t CRC16Bit;
            uint16_t rxCRC;

            /* Compute the 16 bit CRC. */
            CRC16Bit = crc16_ccitt(dataBuffer, dataLength);

            /* Read the CRC from the packet. */
            rxCRC = (dataBuffer[dataLength] << 8U) + dataBuffer[dataLength + 1U];

            if (CRC16Bit == rxCRC)
            {
                retVal = 0;
            }
        }
        else
        {
            uint8_t checksum = 0;
            uint32_t index;

            /* Add all the bytes while dropping any carry overs to compute checksum. */
            for (index = 0; index < dataLength; index++)
            {
                checksum += dataBuffer[index];
            }

            /* Compare to the 8 bit checksum received in the packet */
            if (checksum == dataBuffer[dataLength])
            {
                retVal = 0;
            }
        }
    }
    return retVal;
}

/*!
 *  @b Description
 *  @n
 *      This function is used to discard the incoming UART stream incase an error is detected.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
static void SBL_discardInput(void)
{
    uint8_t rxData;

    UART_read(gSblMCB.uartHandle, (uint8_t *)&rxData, 1U);
    return;
}

/*!
 *  @b Description
 *  @n
 *      This function initializes the UART interface that is
 *      used as transport to download the file.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void SBL_transportInit(void)
{

    MCANParamInit();
    /* Initialize the UART */
    UART_init();
}

/*!
 *  @b Description
 *  @n
 *      This function de-initializes the UART interface that is
 *      used as transport to download the file.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
void SBL_transportDeinit(void)
{
    /* Close the UART peripheral */
    UART_close(gSblMCB.uartHandle);
}

/*!
 *  @b Description
 *  @n
 *      Opens the UART instance in the specified mode.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   SBL Error code
 */
int32_t SBL_transportConfig(void)
{
    UART_Params params;
    int32_t retVal;

    UART_Params_init(&params);

    params.clockFrequency = SOC_getMSSVCLKFrequency(gSblMCB.socHandle, &retVal);
    params.baudRate = SBL_UART_BAUDRATE;
    params.isPinMuxDone = 1U;
    params.readDataMode = UART_DATA_BINARY;
    params.writeDataMode = UART_DATA_BINARY;
    params.readTimeout = 1000U;
    params.readEcho = UART_ECHO_OFF;
    params.readReturnMode = UART_RETURN_FULL;

    /* Open the UART Instance */
    gSblMCB.uartHandle = UART_open(0, &params);
    if (gSblMCB.uartHandle == NULL)
    {
        retVal = SBL_EINVAL;
    }
    else
    {
        retVal = SBL_EOK;
    }
    return retVal;
}

/*!
 *  @b Description
 *  @n
 *      This function is used to read a input character from the UART peripheral.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
int32_t SBL_transportRead(uint8_t *buffer, uint32_t size)
{
    int32_t retVal;

    /* Read using the UART peripheral */
    retVal = UART_read(gSblMCB.uartHandle, (uint8_t *)buffer, size);

    return retVal;
}
/*!
 *  @b Description
 *  @n
 *      This function is used to read a input character from the UART peripheral.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
int32_t SBL_transportWrite_CAN_FD(void)
{
    int32_t retVal = 0;
    int32_t i = 0;
    txMsgObjectParams.msgIdentifier = Get_CanMessageIdentifier((MmwDemo_output_message_type)CAN_MESSAGE_MMWDEMO_MAX);
    for (i = 0; i < 10; i++)
    {
        can_txData[0] = 0xAA;
        can_txData[1] = 0x00; // ROA_Sensor_Status
        can_txData[2] = 0x00;
        can_txData[3] = 0x00;
        Can_Transmit_Schedule(txMsgObjectParams.msgIdentifier, (uint8_t *)&can_txData, 5);
    }

    return retVal;
}

/*!
 *  @b Description
 *  @n
 *      Takes the Formatted input string and dumps it on the UART console
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval Not Applicable
 */
void SBL_printf(const uint8_t *pcFormat, ...)
{
    uint8_t *pcTemp = NULL;
    uint8_t logUartBuff[SBL_UART_LOG_BUFF_SIZE + 1U];
    va_list list;
    int32_t iRet = 0;

    pcTemp = &logUartBuff[0U];

    va_start(list, pcFormat);
    iRet = vsnprintf((char *)pcTemp, SBL_UART_LOG_BUFF_SIZE, (const char *)pcFormat, list);
    va_end(list);
    if ((iRet > MINUS_ONE) && (iRet < SBL_UART_LOG_BUFF_SIZE))
    {
        UART_writePolling(gSblMCB.uartHandle, (uint8_t *)pcTemp, iRet);
    }
    return;
}

/*!
 *  @b Description
 *  @n
 *      This function downloads the application meta image file over UART using XMODEM.
 *      The downloaded file is written to SFLASH pointed to by the flash address.
 *      XMODEM receive routine is implemented based on the following sender<->receiver handshake.
 *      NAK and CAN handshake is not shown below.
 *
 *      RECEIVER ("s -k foo.bar")       SENDER ("foo.bar open x.x minutes")
 *
 *          C
 *                                          STX 01 FE Data[1024] CRC CRC
 *          ACK
 *                                          STX 02 FD Data[1024] CRC CRC
 *          ACK
 *                                          SOH 03 FC Data[128] CRC CRC
 *          ACK
 *                                          SOH 04 FB Data[100] CPMEOF[28] CRC CRC
 *          ACK
 *                                          EOT
 *          ACK
 *
 *  @param[in]  qspiFlashHandle
 *      Handle of QSPI Flash module.
 *  @param[in]  flashAddr
 *      Address of SFLASH location where the application meta image is written to.
 *  @param[in]  maxSize
 *      Maximum size of meta image.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Number of bytes read.
 *  @retval
 *      Error   -   SBL Error code.
 */
int32_t SBL_transportDownloadFile(QSPIFlash_Handle qspiFlashHandle, uint32_t flashAddr, uint32_t maxSize)
{
    uint8_t dataBuffer[SBL_XMODEM_DATABUFFER_SIZE];
    uint8_t txData, rxData;
    uint32_t dataBufferSize = 0, computeCRC;
    uint32_t index, extraDataLen;
    uint8_t optionCRC;
    uint32_t discardPacket = 0;
    int32_t dataLength = 0;
    int32_t retrans = 0;
    int32_t retVal = 0;
    uint8_t packetId = 1;
    uint8_t *ptrData;

    SBL_printf("Debug: Start the image download using XMODEM over UART\r\n");

    /* Initiate the transfer by requesting the transmission with CRC. */
    optionCRC = 'C';
    computeCRC = 1;

    while (1)
    {
        for (index = 0; index < SBL_XMODEM_MAX_WAIT; index++)
        {
            /* Start the transfer with the 16 byte CRC */
            if (optionCRC)
            {
                UART_writePolling(gSblMCB.uartHandle, &optionCRC, 1U);
            }
            retVal = UART_read(gSblMCB.uartHandle, (uint8_t *)&rxData, 1U);

            /* Check if 1 byte was read? */
            if (retVal != 1U)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        if (retVal == 1)
        {
            switch (rxData)
            {
            case SBL_SOH:
            {
                /* Data will be sent in 128 byte chunks. */
                dataBufferSize = 128U;
                goto receive;
            }
            case SBL_STX:
            {
                /* Data will be sent in 1024 byte chunks. */
                dataBufferSize = 1024U;
                goto receive;
            }
            case SBL_EOT:
            {
                /* Transfer is done. Return the number of bytes read. */
                gSblMCB.trans.numACKS++;
                txData = SBL_ACK;
                UART_writePolling(gSblMCB.uartHandle, &txData, 1U);
                goto exitGetFile;
            }
            case SBL_CAN:
            {
                /* Check if Cancel wasn't sent by mistake. Check for a 2nd CAN message */
                retVal = UART_read(gSblMCB.uartHandle, (uint8_t *)&rxData, 1U);

                /* Check if 1 byte was read? */
                if (retVal != 1U)
                {
                    continue;
                }
                else
                {
                    if (rxData == SBL_CAN)
                    {
                        SBL_discardInput();
                        gSblMCB.trans.numACKS++;
                        txData = SBL_ACK;
                        UART_writePolling(gSblMCB.uartHandle, &txData, 1U);
                        dataLength = SBL_ECANCEL;
                        goto exitGetFile;
                    }
                    else
                    {
                        gSblMCB.trans.numNAKS++;
                        txData = SBL_NAK;
                        UART_writePolling(gSblMCB.uartHandle, &txData, 1U);
                    }
                }
                break;
            }
            default:
            {
                printf("Debug: Unsupported character received\n");
                SBL_discardInput();
                gSblMCB.trans.numNAKS++;
                txData = SBL_NAK;
                UART_writePolling(gSblMCB.uartHandle, &txData, 1U);
                continue;
            }
            }
        }
        else
        {
            /* Request for transmission with CRC failed. Try checksum mode */
            if (optionCRC == 'C')
            {
                /* Proceed with no CRC */
                optionCRC = SBL_NAK;
                computeCRC = 0;
                continue;
            }

            /* Both CRC and non-CRC tries were unsuccessful.
             * Cancel the download. */
            SBL_discardInput();
            txData = SBL_CAN;
            UART_writePolling(gSblMCB.uartHandle, &txData, 1U);
            UART_writePolling(gSblMCB.uartHandle, &txData, 1U);
            UART_writePolling(gSblMCB.uartHandle, &txData, 1U);
            dataLength = SBL_ESYNC;
            goto exitGetFile;
        }
    receive:
        /* We have succesfully started the transfer. We don't need to send 'C' anymore */
        optionCRC = 0;

        discardPacket = 0;

        /* Total datalength to read = 128 or 1024 bytes based on 0 byte of header + (16 bit CRC or 8 bit checksum) + 3 bytes of header */
        ptrData = dataBuffer;

        /* Start by storing the first byte of header */
        *ptrData++ = rxData;

        /* We have already read 1st byte. */
        extraDataLen = SBL_XMODEM_HEADER_SIZE - 1U;

        if (computeCRC)
        {
            /* Read 16 bit CRC. */
            extraDataLen += 2U;
        }
        else
        {
            /* Read 8 bit checksum. */
            extraDataLen += 1U;
        }

        /* Check if we should start a valid packet receive? */
        if (dataBufferSize == 0)
        {
            discardPacket = 1U;
        }
        else
        {
            /* Read the remaining bytes */
            for (index = 0; index < (dataBufferSize + extraDataLen); index++)
            {
                retVal = UART_read(gSblMCB.uartHandle, (uint8_t *)&rxData, 1U);

                if (retVal != 1U)
                {
                    discardPacket = 1U;
                    break;
                }
                else
                {
                    *ptrData++ = rxData;
                }
            }
        }

        /* Check if the entire packet was received.
         * The sum header of the 2 bytes is 0XFF.
         * The packet Id should either be current packet or
         * previous packet(In this case don't send a NACK, it will cause a retransmission. ACK the packet but don't store it).
         * The CRC or checksum of the data is valid.
         */
        if (discardPacket != 1U &&
                (dataBuffer[1U] + dataBuffer[2U] == 0xFFU) &&
                (dataBuffer[1U] == (uint8_t)packetId - 1U) ||
            dataBuffer[1U] == packetId &&
                (SBL_verifyCRC(&dataBuffer[3U], dataBufferSize, computeCRC) == 0))
        {
            if (dataBuffer[1] == packetId)
            {
                dataLength += dataBufferSize;

                if (dataLength > maxSize)
                {
                    /* Cancel the download and return error. */
                    dataLength = SBL_EMAXMEM;
                    goto exitGetFile;
                }
                else
                {
                    QSPIFlash_singleWrite(qspiFlashHandle, flashAddr, dataBufferSize, (uint8_t *)&dataBuffer[3]);

                    flashAddr += dataBufferSize;
                    packetId++;
                    gSblMCB.trans.numRxPackets++;
                }
            }

            /* Packet was successfully received and stored. ACK the packet */
            gSblMCB.trans.numACKS++;
            txData = SBL_ACK;
            UART_writePolling(gSblMCB.uartHandle, &txData, 1U);
            continue;
        }
        else
        {
            /* Failure: Check if we exceeded the limit of unsuccessful retransmissions */
            if (retrans++ > SBL_XMODEM_MAX_RETRANSMISSIONS)
            {
                gSblMCB.trans.retransErrors++;
                SBL_discardInput();
                txData = SBL_CAN;
                UART_writePolling(gSblMCB.uartHandle, &txData, 1U);
                UART_writePolling(gSblMCB.uartHandle, &txData, 1U);
                UART_writePolling(gSblMCB.uartHandle, &txData, 1U);
                dataLength = SBL_ERETRANS;
                goto exitGetFile;
            }

            /* One of the 4 conditions above was not met. Send a NACK requesting retransmission */
            gSblMCB.trans.numNAKS++;
            SBL_discardInput();
            txData = SBL_NAK;
            UART_writePolling(gSblMCB.uartHandle, &txData, 1U);
            continue;
        }
    }
exitGetFile:
    return dataLength;
}
