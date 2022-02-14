/////////////////////// visiGenieSerial 06/10/2015 ///////////////////////
//
//      Library to utilize the 4D Systems Genie interface to displays
//      that have been created using the Visi-Genie creator platform.
//      This is intended to be used with the Arduino platform.
//
//      Improvements/Updates by
//        4D Systems Engineering, October 2015, www.4dsystems.com.au
//        4D Systems Engineering, September 2015, www.4dsystems.com.au
//        4D Systems Engineering, August 2015, www.4dsystems.com.au
//        4D Systems Engineering, May 2015, www.4dsystems.com.au
//        Matt Jenkins, March 2015, www.majenko.com
//        Clinton Keith, January 2015, www.clintonkeith.com
//        4D Systems Engineering, July 2014, www.4dsystems.com.au
//        Clinton Keith, March 2014, www.clintonkeith.com
//        Clinton Keith, January 2014, www.clintonkeith.com
//        4D Systems Engineering, January 2014, www.4dsystems.com.au
//        4D Systems Engineering, September 2013, www.4dsystems.com.au
//      Written by
//        Rob Gray (GRAYnomad), June 2013, www.robgray.com
//      Based on code by
//        Gordon Henderson, February 2013, <projects@drogon.net>
//
//      Copyright (c) 2012-2013 4D Systems Pty Ltd, Sydney, Australia
/*********************************************************************
 * This file is part of visiGenieSerial:
 *    visiGenieSerial is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    visiGenieSerial is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with visiGenieSerial.
 *    If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************/

/* Is this an Arduino board? if so, set to 1. For other boards like nrf52, Tiva launchpad, MSP, set to 0 */
#define ARDUINO_BASED 0
/* TODO::I need to research this. I only see 4D test this condition in GetcharSerial, however I do not see any #define. 
  I looked in the Arduino headers too with no luck. For now I will define it here. It may act some sort of 
  differentiation between hardware serial and software serial within Arduino? */
#define SERIAL
/* If you are using an Arduino based board, or old school Wiring, include those headers, else skip */
#if (ARDUINO_BASED == 1)
	#if defined(ARDUINO) && ARDUINO >= 100
		#include "Arduino.h"
	#else
		#include "WProgram.h"
    #endif
#else
    /* Grabbed from Arduino headers for port */
    #define lowByte(w) ((uint8_t) ((w) & 0xff))
    #define highByte(w) ((uint8_t) ((w) >> 8))
#endif

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef visiGenieSerial_h
#define visiGenieSerial_h

#undef GENIE_DEBUG

#define GENIE_VERSION    "VisiGenieSerial 05-01-2017"

// Genie commands & replies:

#define GENIE_ACK               0x06
#define GENIE_NAK               0x15

#define TIMEOUT_PERIOD          1000
#define RESYNC_PERIOD           100

#define GENIE_READ_OBJ          0
#define GENIE_WRITE_OBJ         1
#define GENIE_WRITE_STR         2
#define GENIE_WRITE_STRU        3
#define GENIE_WRITE_CONTRAST    4
#define GENIE_REPORT_OBJ        5
#define GENIE_REPORT_EVENT      7
#define GENIEM_WRITE_BYTES      8
#define GENIEM_WRITE_DBYTES     9
#define GENIEM_REPORT_BYTES     10
#define GENIEM_REPORT_DBYTES    11

// Objects
//    the manual says:
//        Note: Object IDs may change with future releases; it is not
//        advisable to code their values as constants.

#define GENIE_OBJ_DIPSW         0
#define GENIE_OBJ_KNOB          1
#define GENIE_OBJ_ROCKERSW      2
#define GENIE_OBJ_ROTARYSW      3
#define GENIE_OBJ_SLIDER        4
#define GENIE_OBJ_TRACKBAR      5
#define GENIE_OBJ_WINBUTTON     6
#define GENIE_OBJ_ANGULAR_METER 7
#define GENIE_OBJ_COOL_GAUGE    8
#define GENIE_OBJ_CUSTOM_DIGITS 9
#define GENIE_OBJ_FORM          10
#define GENIE_OBJ_GAUGE         11
#define GENIE_OBJ_IMAGE         12
#define GENIE_OBJ_KEYBOARD      13
#define GENIE_OBJ_LED           14
#define GENIE_OBJ_LED_DIGITS    15
#define GENIE_OBJ_METER         16
#define GENIE_OBJ_STRINGS       17
#define GENIE_OBJ_THERMOMETER   18
#define GENIE_OBJ_USER_LED      19
#define GENIE_OBJ_VIDEO         20
#define GENIE_OBJ_STATIC_TEXT   21
#define GENIE_OBJ_SOUND         22
#define GENIE_OBJ_TIMER         23
#define GENIE_OBJ_SPECTRUM      24
#define GENIE_OBJ_SCOPE         25
#define GENIE_OBJ_TANK          26
#define GENIE_OBJ_USERIMAGES    27
#define GENIE_OBJ_PINOUTPUT     28
#define GENIE_OBJ_PININPUT      29
#define GENIE_OBJ_4DBUTTON      30
#define GENIE_OBJ_ANIBUTTON     31
#define GENIE_OBJ_COLORPICKER   32
#define GENIE_OBJ_USERBUTTON    33

// Structure to store replies returned from a display

#define GENIE_FRAME_SIZE        6

/* Following 4D's User*.* convention. A users config implements the hardware/software configurations, in this case UART
   and RTC based functions. I opted to conform to Arduino's Serial interface && millis function to make the port easier.
   The idea is that the consumer will implement their own read/write/available functions for UART, and use whatever
   mechanism available for uptime. */
typedef bool     (*UserUartAvailFn)(void);
typedef uint8_t  (*UserUartReadFn)(void);
typedef void     (*UserUartWriteFn)(uint32_t val);
typedef uint32_t (*UserRtcMillisFn)(void);

typedef struct UserApiConfig {
	UserUartAvailFn  available;
	UserUartReadFn   read;
	UserUartWriteFn  write;
	UserRtcMillisFn  millis;
} UserApiConfig;

typedef struct FrameReportObj {
    uint8_t        cmd;
    uint8_t        object;
    uint8_t        index;
    uint8_t        data_msb;
    uint8_t        data_lsb;
} FrameReportObj;

typedef struct MagicReportHeader {
    uint8_t         cmd;
    uint8_t         index;
    uint8_t         length;
} MagicReportHeader;

/////////////////////////////////////////////////////////////////////
// The Genie frame definition
//
// The union allows the data to be referenced as an array of uint8_t
// or a structure of type FrameReportObj, eg
//
//    genieFrame f;
//    f.bytes[4];
//    f.reportObject.data_lsb
//
//    both methods get the same byte
//
typedef union GenieFrame {
    uint8_t             bytes[GENIE_FRAME_SIZE];
    FrameReportObj      reportObject;
} GenieFrame;

#define MAX_GENIE_EVENTS    16    // MUST be a power of 2
#define MAX_GENIE_FATALS    10
#define MAX_LINK_STATES     20

typedef struct EventQueueStruct {
    GenieFrame    frames[MAX_GENIE_EVENTS];
    uint8_t        rd_index;
    uint8_t        wr_index;
    uint8_t        n_events;
} EventQueueStruct;

typedef void        (*UserEventHandlerPtr) (void);
typedef void        (*UserBytePtr)(uint8_t, uint8_t);
typedef void        (*UserDoubleBytePtr)(uint8_t, uint8_t);

/////////////////////////////////////////////////////////////////////
// User API functions
// These function prototypes are the user API to the library
//
    void        genieInitWithConfig (UserApiConfig *config);
    bool        genieReadObject          (uint16_t object, uint16_t index);
    void    	genieWriteObject         (uint16_t object, uint16_t index, uint16_t data);
    void        genieWriteContrast       (uint16_t value);
    uint16_t    genieWriteStr            (uint16_t index, char *string);
    /* These need to be ported. I'll get to them later
	uint16_t	WriteStr			(uint16_t index, long n) ;
	uint16_t	WriteStr			(uint16_t index, long n, int base) ;
	uint16_t	WriteStr			(uint16_t index, unsigned long n) ;
	uint16_t	WriteStr			(uint16_t index, unsigned long n, int base) ;
	uint16_t	WriteStr			(uint16_t index, int n) ;
	uint16_t	WriteStr			(uint16_t index, int n, int base) ;
	uint16_t	WriteStr			(uint16_t index, unsigned int n) ;
	uint16_t	WriteStr			(uint16_t index, unsigned int n, int base) ;
	uint16_t	WriteStr			(uint16_t index, const String &s);
#ifdef AVR
	uint16_t	WriteStr			(uint16_t index, const __FlashStringHelper *ifsh);
#endif
	uint16_t	WriteStr			(uint16_t index, double n, int digits);
	uint16_t	WriteStr			(uint16_t index, double n);
     */
    uint16_t    genieWriteStrU           (uint16_t index, uint16_t *string);
    bool        genieEventIs             (GenieFrame * e, uint8_t cmd, uint8_t object, uint8_t index);
    uint16_t    genieGetEventData        (GenieFrame * e);
    bool        genieDequeueEvent        (GenieFrame * buff);
    uint16_t    genieDoEvents            (bool DoHandler);
    void        genieAttachEventHandler  (UserEventHandlerPtr userHandler);
    void        genieAttachMagicByteReader (UserBytePtr userHandler);
    void        genieAttachMagicDoubleByteReader (UserDoubleBytePtr userHandler);
    void        geniePulse               (int32_t pin);
    void        genieAssignDebugPort     (UserApiConfig *config);

    // Genie Magic functions (ViSi-Genie Pro Only)

    uint16_t    genieWriteMagicBytes     (uint16_t index, uint8_t *bytes, uint16_t len);
    uint16_t    genieWriteMagicDBytes    (uint16_t index, uint16_t *bytes, uint16_t len);

    uint8_t     genieGetNextByte         (void);
    uint16_t    genieGetNextDoubleByte   (void);

#ifndef TRUE
#define TRUE    (1==1)
#define FALSE    (!TRUE)
#endif

#define ERROR_NONE           0
#define ERROR_TIMEOUT       -1    // 255  0xFF
#define ERROR_NOHANDLER     -2    // 254  0xFE
#define ERROR_NOCHAR        -3    // 253  0xFD
#define ERROR_NAK           -4    // 252  0xFC
#define ERROR_REPLY_OVR     -5    // 251  0xFB
#define ERROR_RESYNC        -6    // 250  0xFA
#define ERROR_NODISPLAY     -7    // 249  0xF9
#define ERROR_BAD_CS        -8    // 248  0xF8

#define GENIE_LINK_IDLE           0
#define GENIE_LINK_WFAN           1 // waiting for Ack or Nak
#define GENIE_LINK_WF_RXREPORT    2 // waiting for a report frame
#define GENIE_LINK_RXREPORT       3 // receiving a report frame
#define GENIE_LINK_RXEVENT        4 // receiving an event frame
#define GENIE_LINK_SHDN           5
#define GENIE_LINK_RXMBYTES       6 // receiving magic bytes
#define GENIE_LINK_RXMDBYTES      7 // receiving magic dbytes

#define GENIE_EVENT_NONE    0
#define GENIE_EVENT_RXCHAR  1

#endif
