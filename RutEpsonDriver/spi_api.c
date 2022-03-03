/////////////////////////////////////////////////////////////////////////////////
// File Name: spi_api.c
//
// Description: Sample for API specification
//
// Author: SEIKO EPSON
//
// History: 2008/04/18 1st. design
//
// Copyright(c) SEIKO EPSON CORPORATION 2008, All rights reserved.
//
// $Id: spi_api.c,v 1.1.1.1 2008/08/28 07:12:47 bish2310 Exp $
/////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>#include <stdbool.h>
#include "spi_api.h"
#include "isc_msgs.h"#include "FreeRTOS.h"#include "task.h"
#define MAX_RECEIVED_DATA_LEN	20
static unsigned char	aucReceivedData[MAX_RECEIVED_DATA_LEN];
static unsigned short	usMessageErrorCode;
static unsigned short	usBlockedMessageID;
static unsigned short	usSequenceStatus;// sequencer message tableunsigned char aucIscSequencerConfigReq_S_0001[] = {	0x00, 0xAA, 0x10, 0x00, 0xC4, 0x00, 0x01, 0x00, 0x01, 0x00,	// file event PS_0001	0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00,	// padding data to skip ISC_SEQUENCER_CONFIG_RESP.	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,};// length of above request messageint iIscSequencerConfigReqLen_S_0001 = sizeof(aucIscSequencerConfigReq_S_0001);////////////////////////////////////////////////////////////////////// ISC_SEQUENCER_CONFIG_REQ for S_0002////////////////////////////////////////////////////////////////////// 0    PS_0002.EOV 1    Der Slave ist Epson////////////////////////////////////////////////////////////////////// sequencer message tableunsigned char aucIscSequencerConfigReq_S_0002[] ={	0x00, 0xAA, 0x10, 0x00, 0xC4, 0x00, 0x01, 0x00, 0x01, 0x00,	// file event PS_0002	0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x01, 0x00,	// padding data to skip ISC_SEQUENCER_CONFIG_RESP.	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,};// length of above request messageint iIscSequencerConfigReqLen_S_0002 = sizeof(aucIscSequencerConfigReq_S_0002);////////////////////////////////////////////////////////////////////// ISC_SEQUENCER_CONFIG_REQ for S_0003////////////////////////////////////////////////////////////////////// 0    PS_0003.EOV 2    Dies ist der Test////////////////////////////////////////////////////////////////////// sequencer message tableunsigned char aucIscSequencerConfigReq_S_0003[] ={	0x00, 0xAA, 0x10, 0x00, 0xC4, 0x00, 0x01, 0x00, 0x01, 0x00,	// file event PS_0003	0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x02, 0x00,	// padding data to skip ISC_SEQUENCER_CONFIG_RESP.	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,};// length of above request messageint iIscSequencerConfigReqLen_S_0003 = sizeof(aucIscSequencerConfigReq_S_0003);unsigned char aucIscSequencerConfigReq_S_0004[] ={	0x00, 0xAA, 0x10, 0x00, 0xC4, 0x00, 0x01, 0x00, 0x01, 0x00,	// file event PS_0003	0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x03, 0x00,	// padding data to skip ISC_SEQUENCER_CONFIG_RESP.	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,};// length of above request messageint iIscSequencerConfigReqLen_S_0004 = sizeof(aucIscSequencerConfigReq_S_0004);unsigned char aucIscSequencerConfigReq_S_0005[] ={	0x00, 0xAA, 0x10, 0x00, 0xC4, 0x00, 0x01, 0x00, 0x01, 0x00,	// file event PS_0003	0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x04, 0x00,	// padding data to skip ISC_SEQUENCER_CONFIG_RESP.	//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,};// length of above request messageint iIscSequencerConfigReqLen_S_0005 = sizeof(aucIscSequencerConfigReq_S_0005);uint8_t check_spi_rx[30];uint8_t check_spi_tx[30];int i;int try ;/*PSoC62 Global Variables*/cyhal_spi_t epson_spi_obj;
/////////////////////////////////////////////////////////////////////////////////
//	SPI_Initialize
//
//	description:
//		Initialize SPI I/F
//
//	argument:
//		None
//
//	return:
//		None
/////////////////////////////////////////////////////////////////////////////////
cy_rslt_t EPSON_Initialize(void)
{
	cy_rslt_t result = CY_RSLT_SUCCESS;	/*SPI Initialization*/	result = cyhal_spi_init(			&EPSON_SPI_HANDLE,			EPSON_SPI_MOSI,			EPSON_SPI_MISO,			EPSON_SPI_CLK,			NC,			NULL,			8,			CYHAL_SPI_MODE_00_MSB,			false);    if(result != CY_RSLT_SUCCESS)    {  	  return result;    }    result = cyhal_spi_set_frequency(&EPSON_SPI_HANDLE, EPSON_SPI_FREQ);    if(result != CY_RSLT_SUCCESS)    {    	return result;    }    /*GPIO Initialization*/    result = cyhal_gpio_init( EPSON_MSGRDY_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);    if (result != CY_RSLT_SUCCESS)    {return result;}    result = cyhal_gpio_init( EPSON_STBY_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);    if (result != CY_RSLT_SUCCESS)    {return result;}    result = cyhal_gpio_init( EPSON_RESET_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);    if (result != CY_RSLT_SUCCESS)    {return result;}    result = cyhal_gpio_init( EPSON_MUTE_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);    if (result != CY_RSLT_SUCCESS)    {return result;}    result = cyhal_gpio_init( EPSON_CLKI_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_NONE, false);    if (result != CY_RSLT_SUCCESS)    {return result;}    result = cyhal_gpio_init( EPSON_SPI_CS_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);    if (result != CY_RSLT_SUCCESS)    {return result;}    return result;
}///////////////////////////////////////////////////////////////////////////////////	S1V30340_Initialize_Audio_Config////	description://		1)Software Reset , 2)Registry key-code, 3)Get version info., 4)Set volume & sampling freq.////	argument://		None////	return://		iError->S1V30340 inner error value/////////////////////////////////////////////////////////////////////////////////
int S1V30340_Initialize_Audio_Config(void){	unsigned short	usReceivedMessageID;				// S1V3034x usReceivedMessageID value holder	int				iError=0;							// S1V3034x inner error value	int message_ready=0 ;								// Control value which shows if data from slave is available	bool Request_Response_Success;  					// Control value for master Request and Slave response handling	Request_Response_Success=0; 						// default value	i=0;	/*Send ISC_RESET_REQ*/    while(Request_Response_Success==false)    {    	SPI_SendMessage(aucIscResetReq, &usReceivedMessageID);    	i=0;    	message_ready = cyhal_gpio_read(EPSON_MSGRDY_PIN);    	if ( message_ready == 1 )    	{    		iError = SPI_ReceiveMessage(&usReceivedMessageID);    		i=0;    		if (iError < SPIERR_SUCCESS || usReceivedMessageID != ID_ISC_RESET_RESP)    		{    			return iError;    		}    		else    		{    			Request_Response_Success=1;    		}    	}    	CyDelay(1);    }	/////////////////////////////////	// Registry key-code	/////////////////////////////////    Request_Response_Success=0;    /*Send ISC_TEST_REQ*/ 	while(Request_Response_Success==0) 	{ 		SPI_SendMessage(aucIscTestReq, &usReceivedMessageID); 		i=0; 		/* Check if the device accept the response from host */ 		message_ready = cyhal_gpio_read(EPSON_MSGRDY_PIN); 		if ( message_ready == 1 ) 		{ 			iError = SPI_ReceiveMessage(&usReceivedMessageID); 			i=0; 			if (iError < SPIERR_SUCCESS || usReceivedMessageID != ID_ISC_TEST_RESP) 			{ 				return iError; 			} 			else 			{ 				Request_Response_Success=1; 			} 		} 		CyDelay(1); 	}	/////////////////////////////////	// Get version info.	///////////////////////////////// 	Request_Response_Success=0; 	/*Send ISC_VERSION_REQ*/	while(Request_Response_Success==0)	{		SPI_SendMessage(aucIscVersionReq, &usReceivedMessageID);		i=0;		message_ready = cyhal_gpio_read(EPSON_MSGRDY_PIN);		if ( message_ready == 1 )		{			iError = SPI_ReceiveMessage(&usReceivedMessageID);			i=0;			Request_Response_Success=1;			if (iError < SPIERR_SUCCESS || usReceivedMessageID != ID_ISC_VERSION_RESP)			{				return iError;			}			else			{				Request_Response_Success=1;			}		}		CyDelay(1);	}	/////////////////////////////////	// Set volume & sampling freq.	///////////////////////////////// 	Request_Response_Success=0; 	/*Send ISC_AUDIO_CONFIG_REQ*/	while(Request_Response_Success==0)	{		SPI_SendMessage(aucIscAudioConfigReq, &usReceivedMessageID);		i=0;		message_ready = cyhal_gpio_read(EPSON_MSGRDY_PIN);		if ( message_ready == 1 )		{			iError = SPI_ReceiveMessage(&usReceivedMessageID);			i=0;			Request_Response_Success=1;			if (iError < SPIERR_SUCCESS || usReceivedMessageID != ID_ISC_AUDIO_CONFIG_RESP)			{				return iError;			}			else			{				Request_Response_Success=true;			}		}		CyDelay(1);	}	return iError;}///////////////////////////////////////////////////////////////////////////////////	S1V30340_Play_Specific_Audio////	description://		Play Audio from ROM////	argument://		None////	return://		iError->S1V30340 inner error value/////////////////////////////////////////////////////////////////////////////////int S1V30340_Play_Specific_Audio(unsigned char aucIscSequencer_element){	unsigned short	usReceivedMessageID;// S1V3034x usReceivedMessageID value holder	int				iError=0;			// S1V3034x inner error value	int message_ready ;					// Control value which shows if data from slave is available	bool Request_Response_Success;  	// Control value for master Request and Slave response handling	Request_Response_Success=false; 	// default value	// manipulate	unsigned char aucIscSequencerConfig[] =	{		0x00, 0xAA, 0x10, 0x00, 0xC4, 0x00, 0x01, 0x00, 0x01, 0x00,		// file event PS_0002		0x00, 0x00, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00,		// padding data to skip ISC_SEQUENCER_CONFIG_RESP.		//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,		// Check		/*		 * Array length consist of 18 elements		 *		 * element of interest is 17 so ->  aucIscSequencerConfigReq_S[16];		 *		 */	};	aucIscSequencerConfig[16]=aucIscSequencer_element;	/*Send ISC_SEQUENCER_CONFIG_REQ*/	while(Request_Response_Success==false)	{		SPI_SendMessage(aucIscSequencerConfig, &usReceivedMessageID);		i=0;		message_ready = cyhal_gpio_read(EPSON_MSGRDY_PIN);		if ( message_ready == 1 )		{			iError = SPI_ReceiveMessage(&usReceivedMessageID);			i=0;			Request_Response_Success=1;			if (iError < SPIERR_SUCCESS || usReceivedMessageID != ID_ISC_SEQUENCER_CONFIG_RESP)			{				goto exit_playback;			}			else			{				Request_Response_Success=true;			}		}		CyDelay(1);	}	aucIscSequencerStartReq[6] = 0;	Request_Response_Success=false;	/*Send ISC_SEQUENCER_START_REQ*/	while(Request_Response_Success==false)	{		SPI_SendMessage(aucIscSequencerStartReq, &usReceivedMessageID);		i=0;		message_ready = cyhal_gpio_read(EPSON_MSGRDY_PIN);		if ( message_ready == 1 )		{			iError = SPI_ReceiveMessage(&usReceivedMessageID);			i=0;			if (iError < SPIERR_SUCCESS || usReceivedMessageID != ID_ISC_SEQUENCER_START_RESP)			{				goto exit_playback;			}			else			{				Request_Response_Success=true;			}		}		CyDelay(1);	}	exit_playback:	return iError;}/*Wait for playback termination*/int S1V30340_Wait_For_Termination(void){	unsigned short	usReceivedMessageID;// S1V3034x usReceivedMessageID value holder	int	iError=0;			// S1V3034x inner error value	int message_ready ;					// Control value which shows if data from slave is available	do	{		message_ready = cyhal_gpio_read(EPSON_MSGRDY_PIN);		if(message_ready == 1)		{			/* Receive ISC_SEQUENCER_STATUS_IND */			iError = SPI_ReceiveMessage(&usReceivedMessageID);		}		if (iError < SPIERR_SUCCESS || usReceivedMessageID == ID_ISC_MSG_BLOCKED_RESP)		{			return iError;		}		//CyDelay(1);		vTaskDelay(pdMS_TO_TICKS(1));	} while (usReceivedMessageID != ID_ISC_SEQUENCER_STATUS_IND);	return iError;}///////////////////////////////////////////////////////////////////////////////////	S1V30340_Stop_Specific_Audio////	description://		Stop Audio from ROM////	argument://		None////	return://		iError->S1V30340 inner error value/////////////////////////////////////////////////////////////////////////////////int S1V30340_Stop_Specific_Audio(void){	unsigned short	usReceivedMessageID;// S1V3034x usReceivedMessageID value holder	int				iError=0;			// S1V3034x inner error value	int message_ready ;					// Control value which shows if data from slave is available	bool Request_Response_Success;  	// Control value for master Request and Slave response handling	Request_Response_Success=false; 	// default value	while(Request_Response_Success==false)	{	SPI_SendMessage(aucIscSequencerStopReq, &usReceivedMessageID);	i=0;	message_ready = cyhal_gpio_read(EPSON_MSGRDY_PIN);		if ( message_ready == 1 )		{			iError = SPI_ReceiveMessage(&usReceivedMessageID);			i=0;			Request_Response_Success=1;			if (iError < SPIERR_SUCCESS || usReceivedMessageID != ID_ISC_SEQUENCER_STOP_RESP)			{				return iError;			}			else			{				Request_Response_Success=true;			}		}	}	return iError;}
/////////////////////////////////////////////////////////////////////////////////
//	SPI_SendReceiveByte
//
//	description:
//		Send and receive 1 byte data by SPI
//
//	argument:
//		ucSendData - send 1 byte data
//
//	return:
//		received 1 byte data
/////////////////////////////////////////////////////////////////////////////////
unsigned char SPI_SendReceiveByte(unsigned char	ucSendData)
{
	unsigned char	ucReceivedData = 0x0;	uint8_t spi_rx=0;	uint8_t spi_tx=0x00;	/* RUTRONIK Manipulation of Driver */	spi_tx=ucSendData;	check_spi_tx[i]=spi_tx;	cyhal_gpio_write(EPSON_SPI_CS_PIN, false);	CyDelay(1);	cyhal_spi_transfer(&EPSON_SPI_HANDLE, &spi_tx, 1, &spi_rx, 1, 0xFF);	cyhal_gpio_write(EPSON_SPI_CS_PIN, true);	ucReceivedData=spi_rx;	check_spi_rx[i]=spi_rx;	i++;	//printf("Received data :0x%0X \n\r",spi_rx);	ucReceivedData=spi_rx;
	return ucReceivedData;
}
/////////////////////////////////////////////////////////////////////////////////
//	SPI_SendMessage_simple
//
//	description:
//		Send message
//
//	argument:
//		pucSendMessage 		- pointer to send message data
//		iSendMessageLength	- length of send message data
//
//	return:
//		error code
/////////////////////////////////////////////////////////////////////////////////
int SPI_SendMessage_simple(unsigned char *pucSendMessage, int iSendMessageLength)
{
	int	i;
	if (pucSendMessage == NULL)	{
		return SPIERR_NULL_PTR;
	}

	for (i = 0; i < iSendMessageLength; i++)	{
		SPI_SendReceiveByte(*pucSendMessage++);
	}
	return SPIERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////
//	SPI_SendMessage
//
//	description:
//		Send message
//
//	argument:
//		pucSendMessage			- pointer to send message data
//		pusReceivedMessageID	- Received message ID
//
//	return:
//		error code
/////////////////////////////////////////////////////////////////////////////////
int SPI_SendMessage(unsigned char *pucSendMessage, unsigned short *pusReceivedMessageID)
{
	unsigned char	ucReceivedData;				// temporary received data
	unsigned short	usSendLength;
	int				iReceivedCounts = 0;		// byte counts of received message (except prefix "0x00 0xAA")
	unsigned short	usReceiveLength = 0;		// length of received message
#ifdef CHECKSUM
	unsigned char	ucCheckSum = 0;				// volue of check sum
	int				iSentCounts = 0;			// byte counts of sent message (including prefix "0x00 0xAA")
#endif
	if (pucSendMessage == NULL || pusReceivedMessageID == NULL)	{
		return SPIERR_NULL_PTR;
	}
	*pusReceivedMessageID = 0xFFFF;
	usSendLength = pucSendMessage[3];
	usSendLength = (usSendLength << 8) | pucSendMessage[2];
#ifdef CHECKSUM
	usSendLength += HEADER_LEN + 1;
#else
	usSendLength += HEADER_LEN;
#endif
	while (usSendLength > 0 || usReceiveLength > 0)	{
		if (usSendLength == 0)		{
			ucReceivedData = SPI_SendReceiveByte(0);
#ifdef CHECKSUM
		}		else if (usSendLength == 1)		{
			ucReceivedData = SPI_SendReceiveByte(ucCheckSum);
			usSendLength--;
#endif
		}		else		{
#ifdef CHECKSUM
			if (iSentCounts >= HEADER_LEN)			{
				ucCheckSum += *pucSendMessage;
			}
			iSentCounts++;
#endif
			ucReceivedData = SPI_SendReceiveByte(*pucSendMessage++);
			usSendLength--;
		}
		// check message prefix(0xAA)
		if (usReceiveLength == 0 && ucReceivedData == 0xAA)		{
			usReceiveLength = 2; // set the length of message length field
		}
		else if (usReceiveLength > 0)		{
			if (iReceivedCounts < MAX_RECEIVED_DATA_LEN)			{
				aucReceivedData[iReceivedCounts] = ucReceivedData;
			}
			if (iReceivedCounts == 1)			{
				usReceiveLength = ucReceivedData;
				usReceiveLength = (usReceiveLength << 8) | aucReceivedData[iReceivedCounts-1];
				usReceiveLength -= 2; // set the length except message length field
			}
			iReceivedCounts++;
			usReceiveLength--;
		}
	}
	if (iReceivedCounts > 0)	{
		unsigned short	usId;
		usId  = aucReceivedData[3];
		usId  = (usId << 8) | aucReceivedData[2];
		*pusReceivedMessageID = usId;
	}
	return SPIERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////
//	SPI_ReceiveMessage_simple
//
//	description:
//		receive message
//
//	argument:
//		pusReceivedMessageID - Received message ID
//
//	return:
//		error code
/////////////////////////////////////////////////////////////////////////////////
int SPI_ReceiveMessage_simple(unsigned short	*pusReceivedMessageID)
{
	int				i;
	unsigned char	aucHeader[2];
	unsigned char	usTmp;
	unsigned short	usLen = 0;
	unsigned short	usId = 0x0;
	if (pusReceivedMessageID == NULL)	{
		return SPIERR_NULL_PTR;
	}
	*pusReceivedMessageID = 0xFFFF;
	aucHeader[0] = SPI_SendReceiveByte(0);
	aucHeader[1] = SPI_SendReceiveByte(0);
	while (1)	{
		if (aucHeader[0] == 0x00 && aucHeader[1] == 0xaa)		{
			usTmp = SPI_SendReceiveByte(0);
			usLen = SPI_SendReceiveByte(0);
			usLen = (usLen << 8) + usTmp;
			usTmp = SPI_SendReceiveByte(0);
			usId  = SPI_SendReceiveByte(0);
			usId  = (usId << 8) + usTmp;
			for (i = 4; i < usLen; i++)			{
				SPI_SendReceiveByte(0);
			}
			break;
		}
		aucHeader[0] = aucHeader[1];
		aucHeader[1] = SPI_SendReceiveByte(0);
	}
	*pusReceivedMessageID = usId;
	return SPIERR_SUCCESS;
}
/////////////////////////////////////////////////////////////////////////////////
//	SPI_ReceiveMessage
//
//	description:
//		Receive message
//
//	argument:
//		pusReceivedMessageID - Received message ID
//		iReceivedMessageLength - Length of receive message
//								  0 : Use length in message
//	return:
//		Error code
/////////////////////////////////////////////////////////////////////////////////
// table used to establish transmit
const unsigned char aucIscVersionResp[LEN_ISC_VERSION_RESP] ={
	0x14, 0x00, 0x06, 0x00, 0x01, 0x00, 0x01, 0x00, 
	0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 
};
int SPI_ReceiveMessage(unsigned short	*pusReceivedMessageID)
{
	unsigned short	i;
	unsigned char	aucHeader[2];
	unsigned char	usTmp;
	unsigned short	usLen = 0;
	unsigned short	usId = 0x0;
	int				iReceivedCounts = 0;
	int				iTimeOut = 0;
	if (pusReceivedMessageID == NULL)	{
		return SPIERR_NULL_PTR;
	}
	*pusReceivedMessageID = 0xFFFF;
	usMessageErrorCode = 0;
	usBlockedMessageID = 0;
	aucHeader[0] = SPI_SendReceiveByte(0);
	aucHeader[1] = SPI_SendReceiveByte(0);
	while (1)	{
		if (aucHeader[0] == 0x00 && aucHeader[1] == 0xaa)		{
			usTmp = aucReceivedData[iReceivedCounts++] = SPI_SendReceiveByte(0);
			usLen = aucReceivedData[iReceivedCounts++] = SPI_SendReceiveByte(0);
			usLen = (usLen << 8) + usTmp;
			usTmp = aucReceivedData[iReceivedCounts++] = SPI_SendReceiveByte(0);
			usId  = aucReceivedData[iReceivedCounts++] = SPI_SendReceiveByte(0);
			usId  = (usId << 8) + usTmp;
			for (i = 4; i < usLen; i++)			{
				if (iReceivedCounts < MAX_RECEIVED_DATA_LEN)				{
					aucReceivedData[iReceivedCounts++] = SPI_SendReceiveByte(0);
				}				else				{
					SPI_SendReceiveByte(0);
				}
			}
			break;
		}
		aucHeader[0] = aucHeader[1];
		aucHeader[1] = SPI_SendReceiveByte(0);
		if (iTimeOut >=  SPI_MSGRDY_TIMEOUT)		{
			return SPIERR_TIMEOUT;
		}
		iTimeOut++;
	}
	*pusReceivedMessageID = usId;

	// check RESP or IND message.
	switch (usId)	{
	case ID_ISC_RESET_RESP://4
	case ID_ISC_AUDIO_PAUSE_IND://4
	case ID_ISC_PMAN_STANDBY_EXIT_IND://4
	case ID_ISC_UART_CONFIG_RESP://4
	case ID_ISC_UART_RCVRDY_IND://4
	case ID_ISC_AUDIODEC_READY_IND://0x11->17
		break;
	case ID_ISC_TEST_RESP://6
	case ID_ISC_ERROR_IND://6
	case ID_ISC_AUDIO_CONFIG_RESP://6
	case ID_ISC_AUDIO_VOLUME_RESP://6
	case ID_ISC_AUDIO_MUTE_RESP://6
	case ID_ISC_PMAN_STANDBY_ENTRY_RESP://6
	case ID_ISC_AUDIODEC_CONFIG_RESP://6
	case ID_ISC_AUDIODEC_DECODE_RESP://6
	case ID_ISC_AUDIODEC_PAUSE_RESP://6
	case ID_ISC_AUDIODEC_ERROR_IND://6
	case ID_ISC_SEQUENCER_CONFIG_RESP://6
	case ID_ISC_SEQUENCER_START_RESP://6
	case ID_ISC_SEQUENCER_STOP_RESP://6
	case ID_ISC_SEQUENCER_PAUSE_RESP://6
	case ID_ISC_SEQUENCER_ERROR_IND://6
	case ID_ISC_AUDIODEC_STOP_RESP://20
		usMessageErrorCode = aucReceivedData[5];
		usMessageErrorCode = (usMessageErrorCode << 8) + aucReceivedData[4];
		break;
	case ID_ISC_SEQUENCER_STATUS_IND://6
		usSequenceStatus = aucReceivedData[5];
		usSequenceStatus = (usSequenceStatus << 8) + aucReceivedData[4];
		break;
	case ID_ISC_VERSION_RESP://20
		for (i = 0; i < LEN_ISC_VERSION_RESP; i++)		{
			if (aucReceivedData [i] != aucIscVersionResp[i])			{
				return SPIERR_ISC_VERSION_RESP;
				break;
			}
		}
		break;
	case ID_ISC_MSG_BLOCKED_RESP://8
		usBlockedMessageID = aucReceivedData[5];
		usBlockedMessageID = (usBlockedMessageID << 8) + aucReceivedData[4];
		usMessageErrorCode = aucReceivedData[7];
		usMessageErrorCode = (usMessageErrorCode << 8) + aucReceivedData[6];
		break;
	default:
		return SPIERR_RESERVED_MESSAGE_ID;
		break;
	}
	if (usMessageErrorCode != 0)	{
		return SPIERR_GET_ERROR_CODE;
	}
	return SPIERR_SUCCESS;
}
///////////////////////////////////////////////////////////////////////
//	GPIO_ControlChipSelect
//
//	description:
//		NSCSS control for Device CS(Chip Select) High/Low control
//
//	argument:
//		iValue		Signal value High:1  Low:0
///////////////////////////////////////////////////////////////////////
void GPIO_S1V30340_Reset(int iValue)
{
	if (iValue)	{		cyhal_gpio_write(EPSON_RESET_PIN, true);
	}	else	{		cyhal_gpio_write(EPSON_RESET_PIN, false);
	}
}
///////////////////////////////////////////////////////////////////////
//	GPIO_ControlStandby
//
//	description:
//		STAND-BY control for Device STBY(Stand-by) High/Low control
//
//	argument:
//		iValue		Signal value High:1  Low:0
///////////////////////////////////////////////////////////////////////
void GPIO_ControlStandby(int iValue)
{
	if (iValue)	{		cyhal_gpio_write(EPSON_STBY_PIN, true);
	}	else	{		cyhal_gpio_write(EPSON_STBY_PIN, false);
	}
}
///////////////////////////////////////////////////////////////////////
//	GPIO_ControlMute
//
//	description:
//		MUTE control for Device MUTE control
//
//	argument:
//		iValue		Signal value Mute  enable:1  disable:0
///////////////////////////////////////////////////////////////////////
void GPIO_ControlMute(int iValue)
{
	if (iValue)	{		cyhal_gpio_write(EPSON_MUTE_PIN, true);
	}	else	{		cyhal_gpio_write(EPSON_MUTE_PIN, false);
	}
}
///////////////////////////////////////////////////////////////////////
// get blocked errorcode field in ISC_*_RESP or ISC_*_IND
///////////////////////////////////////////////////////////////////////
unsigned short GetMessageErrorCode(void)
{
	return usMessageErrorCode;
}
///////////////////////////////////////////////////////////////////////
// get blocked message id field in ISC_MSG_BLOCKED_RESP
///////////////////////////////////////////////////////////////////////
unsigned short GetBlockedMessageID(void)
{
	return usBlockedMessageID;
}
///////////////////////////////////////////////////////////////////////
// get sequnece status field in ISC_SEQUENCER_STATUS_IND
///////////////////////////////////////////////////////////////////////
unsigned short GetSequenceStatus(void)
{
	return usSequenceStatus;
}
