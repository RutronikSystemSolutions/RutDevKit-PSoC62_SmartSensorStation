/////////////////////////////////////////////////////////////////////////////////
// File Name: isc_msgs.h
//
// Description: Speech message definition
//
// Author: SEIKO EPSON
//
// History: 2008/04/17 1st. design
//
// Copyright(c) SEIKO EPSON CORPORATION 2008, All rights reserved.
//
// $Id: isc_msgs.h,v 1.1.1.1 2008/08/28 07:12:47 bish2310 Exp $
/////////////////////////////////////////////////////////////////////////////////
#ifndef _ISC_MSGS_H_
#define	_ISC_MSGS_H_

#include <stdio.h>
#include <stdlib.h>

//////////////////////////////////////////////////
// MACRO DEFINITION
//////////////////////////////////////////////////
#define _GET_HIGH_BYTE(usVal)		(unsigned char)((usVal) >> 8)
#define _GET_LOW_BYTE(usVal)		(unsigned char)((usVal) & 0xFF)

//////////////////////////////////////////////////
// MESSAGE ID DEFINITION
//////////////////////////////////////////////////
#define	ID_START							0xAA

// System message
#define ID_ISC_RESET_REQ					0x0001
#define ID_ISC_RESET_RESP					0x0002
#define ID_ISC_TEST_REQ						0x0003
#define ID_ISC_TEST_RESP					0x0004
#define ID_ISC_VERSION_REQ					0x0005
#define ID_ISC_VERSION_RESP					0x0006
#define ID_ISC_ERROR_IND					0x0000
#define ID_ISC_MSG_BLOCKED_RESP				0x0007
// Audio message
#define ID_ISC_AUDIO_CONFIG_REQ				0x0008
#define ID_ISC_AUDIO_CONFIG_RESP			0x0009
#define ID_ISC_AUDIO_VOLUME_REQ				0x0010
#define ID_ISC_AUDIO_VOLUME_RESP			0x0011
#define ID_ISC_AUDIO_MUTE_REQ				0x000C
#define ID_ISC_AUDIO_MUTE_RESP				0x000D
#define ID_ISC_AUDIO_PAUSE_IND				0x007C
// UART transfer message
#define ID_ISC_UART_CONFIG_REQ				0xF1D2
#define ID_ISC_UART_CONFIG_RESP				0xFFFE
#define ID_ISC_UART_RCVRDY_IND				0xFFFC
// Power management message
#define ID_ISC_PMAN_STANDBY_ENTRY_REQ		0x0064
#define ID_ISC_PMAN_STANDBY_ENTRY_RESP		0x0065
#define ID_ISC_PMAN_STANDBY_EXIT_IND		0x0066
// Streaming playing message
#define ID_ISC_AUDIODEC_CONFIG_REQ			0x006B
#define ID_ISC_AUDIODEC_CONFIG_RESP			0x006C
#define ID_ISC_AUDIODEC_DECODE_REQ			0x006D
#define ID_ISC_AUDIODEC_DECODE_RESP			0x006E
#define ID_ISC_AUDIODEC_READY_IND			0x006F
#define ID_ISC_AUDIODEC_PAUSE_REQ			0x0070
#define ID_ISC_AUDIODEC_PAUSE_RESP			0x0071
#define ID_ISC_AUDIODEC_STOP_REQ			0x0072
#define ID_ISC_AUDIODEC_STOP_RESP			0x0073
#define ID_ISC_AUDIODEC_ERROR_IND			0x007B
// Sequence playing message
#define ID_ISC_SEQUENCER_CONFIG_REQ	 		0x00C4
#define ID_ISC_SEQUENCER_CONFIG_RESP		0x00C5
#define ID_ISC_SEQUENCER_START_REQ			0x00C6
#define ID_ISC_SEQUENCER_START_RESP			0x00C7
#define ID_ISC_SEQUENCER_STOP_REQ			0x00C8
#define ID_ISC_SEQUENCER_STOP_RESP			0x00C9
#define ID_ISC_SEQUENCER_PAUSE_REQ			0x00CA
#define ID_ISC_SEQUENCER_PAUSE_RESP			0x00CB
#define ID_ISC_SEQUENCER_STATUS_IND			0x00CC
#define ID_ISC_SEQUENCER_ERROR_IND			0x00CD

//////////////////////////////////////////////////
// MESSAGE LENGTH DEFINITION
//////////////////////////////////////////////////
#define HEADER_LEN		2
#define MARGIN_LEN		1
// System message
#define LEN_ISC_RESET_REQ					0x0006
#define LEN_ISC_RESET_RESP					0x0004
#define LEN_ISC_TEST_REQ					0x000C
#define LEN_ISC_TEST_RESP					0x0006
#define LEN_ISC_VERSION_REQ					0x0004
#define LEN_ISC_VERSION_RESP				0x0014
#define LEN_ISC_ERROR_IND					0x0006
#define LEN_ISC_MSG_BLOCKED_RESP			0x0008
// Audio message
#define LEN_ISC_AUDIO_CONFIG_REQ			0x000C
#define LEN_ISC_AUDIO_CONFIG_RESP			0x0006
#define LEN_ISC_AUDIO_VOLUME_REQ			0x0006
#define LEN_ISC_AUDIO_VOLUME_RESP			0x0006
#define LEN_ISC_AUDIO_MUTE_REQ				0x0006
#define LEN_ISC_AUDIO_MUTE_RESP				0x0006
#define LEN_ISC_AUDIO_PAUSE_IND				0x0004
// UART transfer message
#define LEN_ISC_UART_CONFIG_REQ				0x0008
#define LEN_ISC_UART_CONFIG_RESP			0x0004
#define LEN_ISC_UART_RCVRDY_IND				0x0004
// Power management message						  
#define LEN_ISC_PMAN_STANDBY_ENTRY_REQ		0x0004
#define LEN_ISC_PMAN_STANDBY_ENTRY_RESP		0x0006
#define LEN_ISC_PMAN_STANDBY_EXIT_IND		0x0004
// Streaming playing message					  
#define LEN_ISC_AUDIODEC_CONFIG_REQ			0x0010
#define LEN_ISC_AUDIODEC_CONFIG_RESP		0x0006
#define LEN_HEAD_ISC_AUDIODEC_DECODE_REQ	0x0008
#define LEN_ISC_AUDIODEC_DECODE_RESP		0x0006
#define LEN_ISC_AUDIODEC_READY_IND			0x0011
#define LEN_ISC_AUDIODEC_PAUSE_REQ			0x0008
#define LEN_ISC_AUDIODEC_PAUSE_RESP			0x0006
#define LEN_ISC_AUDIODEC_STOP_REQ			0x0006
#define LEN_ISC_AUDIODEC_STOP_RESP			0x0014
#define LEN_ISC_AUDIODEC_ERROR_IND			0x0006
// Sequence playing message						  
#define LEN_HEAD_ISC_SEQUENCER_CONFIG_REQ	0x0008
#define LEN_EVENT_ISC_SEQUENCER_CONFIG_REQ	0x0008
#define LEN_ISC_SEQUENCER_CONFIG_RESP		0x0006
#define LEN_ISC_SEQUENCER_START_REQ			0x0006
#define LEN_ISC_SEQUENCER_START_RESP		0x0006
#define LEN_ISC_SEQUENCER_STOP_REQ			0x0004
#define LEN_ISC_SEQUENCER_STOP_RESP			0x0006
#define LEN_ISC_SEQUENCER_PAUSE_REQ			0x0006
#define LEN_ISC_SEQUENCER_PAUSE_RESP		0x0006
#define LEN_ISC_SEQUENCER_STATUS_IND		0x0006
#define LEN_ISC_SEQUENCER_ERROR_IND			0x0006
#define	INIT_AUDIO_VOLUME					0x38	// Audio volume (0x00 - 0x43)

/////////////////////////////////////
// data array of request message
/////////////////////////////////////
extern unsigned char aucIscResetReq[];
extern unsigned char aucIscTestReq[];
extern unsigned char aucIscVersionReq[];
extern unsigned char aucIscPmanStandbyEntryReq[];
extern unsigned char aucIscAudioConfigReq[];
extern unsigned char aucIscAudioVolumeReq[];
extern unsigned char aucIscAudioMuteReq[];
extern unsigned char aucIscAudiodecConfigReq[];
extern unsigned char aucIscAudiodecPauseReq[];
extern unsigned char aucIscAudiodecStopReq[];
extern unsigned char aucIscSequencerConfigReq[];
extern unsigned char aucIscSequencerStartReq[];
extern unsigned char aucIscSequencerStopReq[];
extern unsigned char aucIscSequencerPauseReq[];

extern int iIscResetReqLen;
extern int iIscTestReqLen;
extern int iIscVersionReqLen;
extern int iIscPmanStandbyEntryReqLen;
extern int iIscAudioConfigReqLen;
extern int iIscAudioVolumeReqLen;
extern int iIscAudioMuteReqLen;
extern int iIscAudiodecConfigReqLen;
extern int iIscAudiodecPauseReqLen;
extern int iIscAudiodecStopReqLen;
extern int iIscSequencerConfigReqLen;
extern int iIscSequencerStartReqLen;
extern int iIscSequencerStopReqLen;
extern int iIscSequencerPauseReqLen;

extern int hello_len;
extern int hello_offset;
extern unsigned char hello_table[];

//////////////////////////////////////////////////
// TYPE DEFINITION
//////////////////////////////////////////////////
typedef struct tagAUDIODEC_DECODE_REQ {
	unsigned char	*pucData;			// pointer to request messages table
	int				iLen;				// length of table
	int				iBlockLen;			// offset value for accesssing to the request message in table.
} AUDIODEC_DECODE_REQ;

#endif //!_ISC_MSGS_H_
