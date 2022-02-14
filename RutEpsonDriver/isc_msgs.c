/////////////////////////////////////////////////////////////////////////////////
// File Name: isc_msgs.c
//
// Description: Speech message definition
//
// Author: SEIKO EPSON
//
// History: 2008/04/25 1st. design
//
// Copyright(c) SEIKO EPSON CORPORATION 2008, All rights reserved.
//
// $Id: isc_msgs.c,v 1.1.1.1 2008/08/28 07:12:47 bish2310 Exp $
/////////////////////////////////////////////////////////////////////////////////
#include "isc_msgs.h"
unsigned char aucIscResetReq[HEADER_LEN + LEN_ISC_RESET_REQ] = {
	0x00, ID_START,
	LEN_ISC_RESET_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_RESET_REQ), _GET_HIGH_BYTE(ID_ISC_RESET_REQ),
	0x00, 0x00,
};
int iIscResetReqLen = sizeof(aucIscResetReq);

#ifdef CHECKSUM
unsigned char aucIscTestReq[HEADER_LEN + LEN_ISC_TEST_REQ] = {
	0x00, ID_START,
	LEN_ISC_TEST_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_TEST_REQ), _GET_HIGH_BYTE(ID_ISC_TEST_REQ),
	0x01, 0x00,				// Check-Sum 1:Enable 0:Disable
	0x01, 0x00,				// 1:Full duplex, 0:Half duplex
	0xe1, 0x48, 0xe6, 0x55, // key-code
};
#else
unsigned char aucIscTestReq[HEADER_LEN + LEN_ISC_TEST_REQ] = {
	0x00, ID_START,
	LEN_ISC_TEST_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_TEST_REQ), _GET_HIGH_BYTE(ID_ISC_TEST_REQ),
	0x00, 0x00,				// Check-Sum 1:Enable 0:Disable
	0x01, 0x00,				// 1:Full duplex, 0:Half duplex
	//0xe1, 0x48, 0xe6, 0x55, // key-code	0x85, 0x1F, 0xAA, 0x1A, // key-code
};
#endif

int iIscTestReqLen = sizeof(aucIscTestReq);

unsigned char aucIscVersionReq[HEADER_LEN + LEN_ISC_VERSION_REQ] = {
	0x00, ID_START,
	LEN_ISC_VERSION_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_VERSION_REQ), _GET_HIGH_BYTE(ID_ISC_VERSION_REQ),
};

int iIscVersionReqLen = sizeof(aucIscVersionReq);

unsigned char aucIscPmanStandbyEntryReq[HEADER_LEN + LEN_ISC_PMAN_STANDBY_ENTRY_REQ] = {
	0x00, ID_START,
	LEN_ISC_PMAN_STANDBY_ENTRY_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_PMAN_STANDBY_ENTRY_REQ), _GET_HIGH_BYTE(ID_ISC_PMAN_STANDBY_ENTRY_REQ),
};

int iIscPmanStandbyEntryReqLen = sizeof(aucIscPmanStandbyEntryReq);
  
unsigned char aucIscAudioConfigReq[HEADER_LEN + LEN_ISC_AUDIO_CONFIG_REQ] = {
	0x00, ID_START,
	LEN_ISC_AUDIO_CONFIG_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_AUDIO_CONFIG_REQ), _GET_HIGH_BYTE(ID_ISC_AUDIO_CONFIG_REQ),
	0x00,							// reserved
	INIT_AUDIO_VOLUME,				// audio gain
	0x00, 							// reserved
	0x09,							// audio sample rate
	0x00,							// reserved
	0x00,							// reserved
	0x00,							// reserved
	0x00,							// reserved
};

int iIscAudioConfigReqLen = sizeof(aucIscAudioConfigReq);

unsigned char aucIscAudioVolumeReq[HEADER_LEN + LEN_ISC_AUDIO_VOLUME_REQ] = {
	0x00, ID_START,
	LEN_ISC_AUDIO_VOLUME_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_AUDIO_VOLUME_REQ), _GET_HIGH_BYTE(ID_ISC_AUDIO_VOLUME_REQ),
	0x00, 0x00, 					// audio gain inc
};

int iIscAudioVolumeReqLen = sizeof(aucIscAudioVolumeReq);

unsigned char aucIscAudioMuteReq[HEADER_LEN + LEN_ISC_AUDIO_MUTE_REQ] = {
	0x00, ID_START,
	LEN_ISC_AUDIO_MUTE_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_AUDIO_MUTE_REQ), _GET_HIGH_BYTE(ID_ISC_AUDIO_MUTE_REQ),
	0x00,							// Mute 0x01:enable, 0x00:disable
	0x00,
};

int iIscAudioMuteReqLen = sizeof(aucIscAudioMuteReq);

unsigned char aucIscAudiodecConfigReq[HEADER_LEN + LEN_ISC_AUDIODEC_CONFIG_REQ] = {
	0x00, ID_START,
	LEN_ISC_AUDIODEC_CONFIG_REQ,	0x00,
	_GET_LOW_BYTE(ID_ISC_AUDIODEC_CONFIG_REQ), _GET_HIGH_BYTE(ID_ISC_AUDIODEC_CONFIG_REQ),
	0x00,							// input source
	0x09,							// file type EOV
	0x00,							// flags (reserved)
	0x00,							// channels (reserved)
	0x80, 0x3e, 0x00, 0x00,			// sampling rate
	0x00, 0x00,	0x00, 0x00, 		// reserved
};

int iIscAudiodecConfigReqLen = sizeof(aucIscAudiodecConfigReq);

unsigned char aucIscAudiodecPauseReq[HEADER_LEN + LEN_ISC_AUDIODEC_PAUSE_REQ] = {
	0x00, ID_START,
	LEN_ISC_AUDIODEC_PAUSE_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_AUDIODEC_PAUSE_REQ), _GET_HIGH_BYTE(ID_ISC_AUDIODEC_PAUSE_REQ),
	0x00,							// pause 0x01:enable, 0x00:disable
	0x00,
	0x00, 0x00,
};

int iIscAudiodecPauseReqLen = sizeof(aucIscAudiodecPauseReq);

unsigned char aucIscAudiodecStopReq[HEADER_LEN + LEN_ISC_AUDIODEC_STOP_REQ] = {
	0x00, ID_START,
	LEN_ISC_AUDIODEC_STOP_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_AUDIODEC_STOP_REQ), _GET_HIGH_BYTE(ID_ISC_AUDIODEC_STOP_REQ),
	0x00, 0x00,
};

int iIscAudiodecStopReqLen = sizeof(aucIscAudiodecStopReq);

unsigned char aucIscSequencerStartReq[HEADER_LEN + LEN_ISC_SEQUENCER_START_REQ] = {
	0x00, ID_START,
	LEN_ISC_SEQUENCER_START_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_SEQUENCER_START_REQ), _GET_HIGH_BYTE(ID_ISC_SEQUENCER_START_REQ), 
	0x00,							// notification 0:disable, 1:enable
	0x00,
};

int iIscSequencerStartReqLen = sizeof(aucIscSequencerStartReq);

unsigned char aucIscSequencerStopReq[HEADER_LEN + LEN_ISC_SEQUENCER_STOP_REQ] = {
	0x00, ID_START,
	LEN_ISC_SEQUENCER_STOP_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_SEQUENCER_STOP_REQ), _GET_HIGH_BYTE(ID_ISC_SEQUENCER_STOP_REQ), 
};

int iIscSequencerStopReqLen = sizeof(aucIscSequencerStopReq);

unsigned char aucIscSequencerPauseReq[HEADER_LEN + LEN_ISC_SEQUENCER_PAUSE_REQ] = {
	0x00, ID_START,
	LEN_ISC_SEQUENCER_PAUSE_REQ, 0x00,
	_GET_LOW_BYTE(ID_ISC_SEQUENCER_PAUSE_REQ), _GET_HIGH_BYTE(ID_ISC_SEQUENCER_PAUSE_REQ),
	0x00,							// pause 0x01:enable, 0x00:disable
	0x00,
};

int iIscSequencerPauseReqLen = sizeof(aucIscSequencerPauseReq);
