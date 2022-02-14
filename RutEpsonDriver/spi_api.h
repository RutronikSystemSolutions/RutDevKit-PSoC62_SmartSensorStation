/////////////////////////////////////////////////////////////////////////////////
// File Name: spi_api.h
//
// Description: Header file for API specification
//
// Author: SEIKO EPSON
//
// History: 2008/04/18 1st. design
//
// Copyright(c) SEIKO EPSON CORPORATION 2008, All rights reserved.
//
// $Id: spi_api.h,v 1.1.1.1 2008/08/28 07:12:47 bish2310 Exp $
/////////////////////////////////////////////////////////////////////////////////

#ifndef	_SPI_API_H_
#define	_SPI_API_H_/*PSoC62 Header Files*/#include "cycfg_pins.h"#include "cyhal.h"
#ifdef __cplusplus
extern "C" {
#endif/*PSoC6 Adaptation*/#define EPSON_MSGRDY_PIN	ARDU_IO7#define EPSON_STBY_PIN		ARDU_IO6#define EPSON_RESET_PIN		ARDU_IO4#define EPSON_MUTE_PIN		ARDU_IO1#define EPSON_CLKI_PIN		ARDU_CS#define EPSON_SPI_CS_PIN	ARDU_IO3#define EPSON_SPI_MOSI		ARDU_MOSI#define EPSON_SPI_MISO		ARDU_MISO#define EPSON_SPI_CLK		ARDU_CLK#define EPSON_SPI_HANDLE	epson_spi_obj#define EPSON_SPI_FREQ		2000000UL
// SPI transfer
#define	SPI_MSGRDY_TIMEOUT	1
// Error definition for API function
#define SPIERR_TIMEOUT				1
#define SPIERR_SUCCESS				0
#define SPIERR_NULL_PTR				-1
#define SPIERR_GET_ERROR_CODE		-2
#define SPIERR_RESERVED_MESSAGE_ID	-3
#define SPIERR_ISC_VERSION_RESP		-4
// Definition of checksum function
//#define CHECKSUM 0 No need in simple application
cy_rslt_t EPSON_Initialize(void);// RUTRINIK MODIFICATION STARTint S1V30340_Initialize_Audio_Config(void);
int S1V30340_Play_Specific_Audio(unsigned char aucIscSequencer_element);int S1V30340_Wait_For_Termination(void);int S1V30340_Stop_Specific_Audio(void);// RUTRINIK MODIFICATION END
unsigned char SPI_SendReceiveByte(unsigned char	ucSendData);
int SPI_SendMessageSPI_SendMessage(unsigned char *pucSendMessage, unsigned short *pusReceivedMessageID);int SPI_ReceiveMessage(unsigned short	*pusReceivedMessageID);int SPI_SendMessage_simple(unsigned char *pucSendMessage, int iSendMessageLength);int SPI_ReceiveMessage_simple(unsigned short *pusReceivedMessageID);int SPI_SendMessage(unsigned char *pucSendMessage,unsigned short *pusReceivedMessageID);void GPIO_S1V30340_Reset(int iValue);void GPIO_ControlStandby(int iValue);void GPIO_ControlMute(int iValue);
// function for test
unsigned short GetMessageErrorCode(void);
unsigned short GetBlockedMessageID(void);
unsigned short GetSequenceStatus(void);
#ifdef __cplusplus
}
#endif
#endif //!_SPI_API_H_
