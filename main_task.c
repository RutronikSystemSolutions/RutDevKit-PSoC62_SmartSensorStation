
/*
 * main_task.c
 *
 *  Created on: 2021-12-06
 *      Author: GDR
 */

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main_task.h"
#include "spi_api.h"
#include "bmi270_app.h"
#include "visiGenieSerial.h"
#include "sensors_task.h"
#include "speech_task.h"

#define ARDU_BAUD_RATE       		600000
#define SCOPE_SCALE					0.0042735042735043

static cy_rslt_t ardu_uart_init();

/* UserApiConfig */
static bool uartAvailHandler(void);
static uint8_t uartReadHandler(void);
static void uartWriteHandler(uint32_t val);
static uint32_t uartGetMillis(void);
static void resetDisplay(void);

/* Event handlers */
static void myGenieEventHandler(void);

TaskHandle_t main_task_handle = NULL;

/*Arduino UART object*/
cyhal_uart_t ardu_uart;

static UserApiConfig userConfig =
{
	.available = uartAvailHandler,
	.read =  uartReadHandler,
	.write = uartWriteHandler,
	.millis = uartGetMillis
};

_Bool alarm_enabled = false;

void main_task(void *param)
{
	(void) param;
	cy_rslt_t result;
	int ecfg_result = -1;
	uint8_t motion_cnt = 0;
	speech_t speech_msg;

	printf("main task has started.\r\n");

    /*Initialize The Arduino UART*/
    result = ardu_uart_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize ViSi-Genie*/
    genieInitWithConfig(&userConfig);
    genieAttachEventHandler(myGenieEventHandler);
    resetDisplay();
    genieWriteContrast(15);
    genieWriteStr(0, GENIE_VERSION);
    vTaskDelay(pdMS_TO_TICKS(2000));

	/*Initialize Epson ASIC*/
	result = EPSON_Initialize();
	if(result != CY_RSLT_SUCCESS)
	{
		printf("Text-to-Speech Engine failed to initialize.\r\n");
		 CY_ASSERT(0);
	}

	/*Epson ASIC Hard-Reset Procedure*/
	GPIO_S1V30340_Reset(1);
	vTaskDelay(pdMS_TO_TICKS(100));
	GPIO_S1V30340_Reset(0);
	vTaskDelay(pdMS_TO_TICKS(100));
	GPIO_S1V30340_Reset(1);
	vTaskDelay(pdMS_TO_TICKS(500));

	/*Set Epson ASIC mute signal(MUTE) to High(disable)*/
	GPIO_ControlMute(0);

	/*Set Epson ASIC standby signal(STBYEXIT) to Low(deassert)*/
	GPIO_ControlStandby(0);
	vTaskDelay(pdMS_TO_TICKS(100));

	/*Configure Epson ASIC*/
	ecfg_result = S1V30340_Initialize_Audio_Config();
	while(ecfg_result != 0)
	{
		printf("S1V30340 SPI Initialization failed. \n\r");
		vTaskDelay(pdMS_TO_TICKS(1000));
		ecfg_result = S1V30340_Initialize_Audio_Config();
	}
	printf("S1V30340 SPI Initialization succeeded. \n\r");

	/*Play the greeting*/
	GPIO_ControlMute(1); /*Mute - OFF*/
	S1V30340_Play_Specific_Audio(GREETING_PHRASE);
	S1V30340_Wait_For_Termination();
	GPIO_ControlMute(0); /*Mute - ON*/

	for(;;)
	{
        /*Visi Genie Events*/
        genieDoEvents(true);

        /*Form0*/
        genieWriteObject(GENIE_OBJ_ANGULAR_METER, 0, (int16_t)sensor_data_storage.sgp_voc_index);
        genieWriteObject(44, 0, (int16_t)((sensor_data_storage.sht_temperature)/1000));
        genieWriteObject(GENIE_OBJ_GAUGE, 0, (int16_t)(sensor_data_storage.sht_humidity/1000));
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 6, (int16_t)((sensor_data_storage.sht_temperature/10)));
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 7, (int16_t)(sensor_data_storage.sht_humidity/10));

        /*Form1*/
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 0, sensor_data_storage.bmi_acc_x);
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 1, sensor_data_storage.bmi_acc_y);
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 2, sensor_data_storage.bmi_acc_z);
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 3, sensor_data_storage.bmi_gyr_x);
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 4, sensor_data_storage.bmi_gyr_y);
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 5, sensor_data_storage.bmi_gyr_z);
        genieWriteObject(GENIE_OBJ_SCOPE, 0, (int16_t)(sensor_data_storage.bmi_acc_x * SCOPE_SCALE));
        genieWriteObject(GENIE_OBJ_SCOPE, 0, (int16_t)(sensor_data_storage.bmi_acc_y * SCOPE_SCALE));
        genieWriteObject(GENIE_OBJ_SCOPE, 0, (int16_t)(sensor_data_storage.bmi_acc_z * SCOPE_SCALE));
        genieWriteObject(GENIE_OBJ_SCOPE, 1, (int16_t)(sensor_data_storage.bmi_gyr_x * SCOPE_SCALE));
        genieWriteObject(GENIE_OBJ_SCOPE, 1, (int16_t)(sensor_data_storage.bmi_gyr_y * SCOPE_SCALE));
        genieWriteObject(GENIE_OBJ_SCOPE, 1, (int16_t)(sensor_data_storage.bmi_gyr_z * SCOPE_SCALE));
        if(motion)
        {
        	if(alarm_enabled)
        	{
        		speech_msg.phrase_number = 1;
        		(void)xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 0 );
        	}
        	motion_cnt = 8;
        	motion = false;
        }
        if(motion_cnt)
        {
        	motion_cnt--;
        	genieWriteObject(GENIE_OBJ_USER_LED, 0, 1);
        	if(!motion_cnt)
        	{
        		genieWriteObject(GENIE_OBJ_USER_LED, 0, 0);
        	}
        }

        /*Form2*/
        genieWriteObject(GENIE_OBJ_COOL_GAUGE, 0, (uint16_t)((sensor_data_storage.bmp_pressure/1000)));
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 14, (uint16_t)(sensor_data_storage.bmp_pressure/10));
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 8, (uint16_t)(sensor_data_storage.bmp_temperature*100));
        genieWriteObject(GENIE_OBJ_METER, 0, (uint16_t)((sensor_data_storage.bme_pressure/1000)));
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 9, (uint16_t)(sensor_data_storage.bme_pressure/10));
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 10, (uint16_t)(sensor_data_storage.bme_temperature*100));
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 11, (uint16_t)(sensor_data_storage.bme_gas_resistance/100000));
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 12, (uint16_t)(sensor_data_storage.bme_humidity*100));
        genieWriteObject(GENIE_OBJ_LED_DIGITS, 13, (uint16_t)(sensor_data_storage.bme_gas_index));
	}
}

static cy_rslt_t ardu_uart_init(void)
{
	cy_rslt_t result;
	uint32_t actualbaud;

    /* Initialize the UART configuration structure */
    const cyhal_uart_cfg_t uart_config =
    {
        .data_bits = 8,
        .stop_bits = 1,
        .parity = CYHAL_UART_PARITY_NONE,
        .rx_buffer = NULL,
        .rx_buffer_size = 0
    };

    /* Initialize the UART Block */
    result = cyhal_uart_init(&ardu_uart, ARDU_TX, ARDU_RX, NC, NC, NULL, &uart_config);
	if (result != CY_RSLT_SUCCESS)
	{return result;}

	result = cyhal_uart_set_baud(&ardu_uart, ARDU_BAUD_RATE, &actualbaud);
	if (result != CY_RSLT_SUCCESS)
	{return result;}

	/*Connect internal pull-up resistor*/
	cyhal_gpio_configure(ARDU_RX, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP);

	return result;
}

/* UserApiConfig Handlers */
static bool uartAvailHandler(void)
{
  return (_Bool)cyhal_uart_readable(&ardu_uart);
}

static uint8_t uartReadHandler(void)
{
	uint8_t byte;

	(void)cyhal_uart_getc(&ardu_uart, &byte,0xFFFFFFFF);

	return byte;
}

static void uartWriteHandler(uint32_t val)
{
	(void)cyhal_uart_putc(&ardu_uart,val);
}

static uint32_t uartGetMillis(void)
{
  return (uint32_t)xTaskGetTickCount();
}

static void resetDisplay(void)
{
	cyhal_gpio_write(ARDU_IO8, false);
	vTaskDelay(pdMS_TO_TICKS(500));
	cyhal_gpio_write(ARDU_IO8, true);
	vTaskDelay(pdMS_TO_TICKS(2000));
}

/*ViSi Genie Event Handler*/
static void myGenieEventHandler(void)
{
  GenieFrame Event;
  genieDequeueEvent(&Event);
  int32_t button_val = 0;

  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON)
    {
      if (Event.reportObject.index == 0)
      {
    	  button_val = genieGetEventData(&Event);
    	  if(button_val)
    	  {
    		  alarm_enabled = true;
    	  }
    	  else
		  {
    		  alarm_enabled = false;
		  }
      }
    }
  }
}
