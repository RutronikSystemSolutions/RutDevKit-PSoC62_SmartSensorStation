
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
#include "timers.h"

#define ARDU_BAUD_RATE       		115200
#define SCOPE_SCALE					0.0042735042735043

static cy_rslt_t ardu_uart_init();

/* UserApiConfig */
static bool uartAvailHandler(void);
static uint8_t uartReadHandler(void);
static void uartWriteHandler(uint32_t val);
static uint32_t uartGetMillis(void);
static void resetDisplay(void);

/* Display Event Handler */
static void myGenieEventHandler(void);

/*Speech engine functions*/
static void Speech_Check_Callback(TimerHandle_t xTimer);
static void Phrase_Repeat_Callback(TimerHandle_t xTimer);
static co2status_t DecodeCO2Level(int co2eq_ppm);

TaskHandle_t main_task_handle = NULL;
TimerHandle_t phrase_repeat = NULL;
TimerHandle_t speech_check = NULL;

/*Arduino UART object and configuration*/
cyhal_uart_t ardu_uart;
static UserApiConfig userConfig =
{
	.available = uartAvailHandler,
	.read =  uartReadHandler,
	.write = uartWriteHandler,
	.millis = uartGetMillis
};

/*Global variable for motion detection*/
_Bool alarm_enabled = true;

/*Global variables for CO2 Speech Engine*/
long ph_rep_id = 1;
long sp_ch_id = 2;
co2status_t co2_state_prev = CO2_UNDEFINED;
_Bool check = false;

void main_task(void *param)
{
	(void) param;
	cy_rslt_t result;
	int ecfg_result = -1;
	uint8_t motion_cnt = 0;
	BaseType_t msg_status;
	speech_t speech_msg;
	char level_str[16] = {0};
	uint16_t old_co2_pas = 0;
	uint16_t old_co2_scd = 0;
	int32 old_sht_temp = 0;
	int32 old_sht_hum = 0;
	int16_t old_acc_x = 0;
	int16_t old_acc_y = 0;
	int16_t old_acc_z = 0;
	int16_t old_gyr_x = 0;
	int16_t old_gyr_y = 0;
	int16_t old_gyr_z = 0;
	float old_bme_temp = 0;
	float old_bme_hum = 0;
	float old_bme_pres = 0;
	float old_bme_gre = 0;
	uint8_t old_bme_ind = 0;
	double old_bmp_temp = 0;
	double old_bmp_pres = 0;
	double old_bmp_alt = 0;
	co2status_t co2_state = CO2_UNDEFINED;

	printf("main task has started.\r\n");

    /*Initialize The Arduino UART*/
    result = ardu_uart_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize ViSi-Genie*/
    genieInitWithConfig(&userConfig);
    genieAttachEventHandler(myGenieEventHandler);
    resetDisplay();
    genieWriteContrast(8);
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
	speech_msg.phrase_number = GREETING_PHRASE;
	(void)xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 0 );

	/*Initial CO2 speech engine check timer startup.*/
	speech_check = xTimerCreate("Check", SPEECH_ENGINE_INIT, pdFALSE, &sp_ch_id, Speech_Check_Callback);
	xTimerStart(speech_check, 100);

	for(;;)
	{
        /*Visi Genie Events*/
        genieDoEvents(true);

        /*Form0*/
        genieWriteObject(GENIE_OBJ_ANGULAR_METER, 0, (int16_t)sensor_data_storage.sgp_voc_index);
        genieWriteObject(44, 0, (int16_t)((sensor_data_storage.sht_temperature)/1000));
        genieWriteObject(GENIE_OBJ_GAUGE, 0, (int16_t)(sensor_data_storage.sht_humidity/1000));
        if(old_sht_temp != sensor_data_storage.sht_temperature)
        {
        	old_sht_temp = sensor_data_storage.sht_temperature;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%.2f", (float)(sensor_data_storage.sht_temperature/1000.0));
            genieWriteStr (2, level_str);
        }

        if(old_sht_hum != sensor_data_storage.sht_humidity)
        {
        	old_sht_hum = sensor_data_storage.sht_humidity;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%.2f", (float)(sensor_data_storage.sht_humidity/1000.0));
            genieWriteStr (3, level_str);
        }

        /*Form1*/
        if(old_acc_x != sensor_data_storage.bmi_acc_x)
        {
        	old_acc_x = sensor_data_storage.bmi_acc_x;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%d", old_acc_x);
            genieWriteStr (4, level_str);
        }
        if(old_acc_y != sensor_data_storage.bmi_acc_y)
        {
        	old_acc_y = sensor_data_storage.bmi_acc_y;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%d", old_acc_y);
            genieWriteStr (5, level_str);
        }
        if(old_acc_z != sensor_data_storage.bmi_acc_z)
        {
        	old_acc_z = sensor_data_storage.bmi_acc_z;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%d", old_acc_z);
            genieWriteStr (6, level_str);
        }
        if(old_gyr_x != sensor_data_storage.bmi_gyr_x)
        {
        	old_gyr_x = sensor_data_storage.bmi_gyr_x;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%d", old_gyr_x);
            genieWriteStr (7, level_str);
        }
        if(old_gyr_y != sensor_data_storage.bmi_gyr_y)
        {
        	old_gyr_y = sensor_data_storage.bmi_gyr_y;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%d", old_gyr_y);
            genieWriteStr (8, level_str);
        }
        if(old_gyr_z != sensor_data_storage.bmi_gyr_z)
        {
        	old_gyr_z = sensor_data_storage.bmi_gyr_z;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%d", old_gyr_z);
            genieWriteStr (9, level_str);
        }

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
        		speech_msg.phrase_number = MOTION_DETECTED;
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
        genieWriteObject(GENIE_OBJ_COOL_GAUGE, 1, (uint16_t)((sensor_data_storage.bme_pressure/1000)));

        if(old_bme_temp != sensor_data_storage.bme_temperature)
        {
        	old_bme_temp = sensor_data_storage.bme_temperature;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%.2f", old_bme_temp);
            genieWriteStr (13, level_str);
        }

        if(old_bme_hum != sensor_data_storage.bme_humidity)
        {
        	old_bme_hum = sensor_data_storage.bme_humidity;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%.2f", old_bme_hum);
            genieWriteStr (15, level_str);
        }

        if(old_bme_pres != sensor_data_storage.bme_pressure)
        {
        	old_bme_pres = sensor_data_storage.bme_pressure;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%.2f", old_bme_pres/1000);
            genieWriteStr (12, level_str);
        }

        if(old_bme_gre != sensor_data_storage.bme_gas_resistance)
        {
        	old_bme_gre = sensor_data_storage.bme_gas_resistance;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%.0f", old_bme_gre/100000);
            genieWriteStr (14, level_str);
        }

        if(old_bme_ind != sensor_data_storage.bme_gas_index)
        {
        	old_bme_ind = sensor_data_storage.bme_gas_index;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%d", old_bme_ind);
            genieWriteStr (16, level_str);
        }

        if(old_bmp_temp != sensor_data_storage.bmp_temperature)
        {
        	old_bmp_temp = sensor_data_storage.bmp_temperature;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%.2f", old_bmp_temp);
            genieWriteStr (10, level_str);
        }

        if(old_bmp_pres != sensor_data_storage.bmp_pressure)
        {
        	old_bmp_pres = sensor_data_storage.bmp_pressure;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%.2f", old_bmp_pres/1000);
            genieWriteStr (11, level_str);
        }

        if(old_bmp_alt != sensor_data_storage.bmp_altitude)
        {
        	old_bmp_alt = sensor_data_storage.bmp_altitude;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%.2f", old_bmp_alt);
            genieWriteStr (17, level_str);
        }

        /*Form3*/
        memset(level_str, 0x00, sizeof(level_str));
        sprintf(level_str, "%d ppm", sensor_data_storage.pas_co2);
        if(old_co2_pas != sensor_data_storage.pas_co2)
        {
        	genieWriteStr (0, level_str);
        	old_co2_pas = sensor_data_storage.pas_co2;
        }

        uint16_t gauge_level = (uint16_t)(sensor_data_storage.pas_co2/30);
        if(gauge_level > 100)
        {gauge_level = 100;}
        genieWriteObject(GENIE_OBJ_GAUGE, 1, gauge_level);

        memset(level_str, 0x00, sizeof(level_str));
        sprintf(level_str, "%d ppm", sensor_data_storage.scd_co2);
        if(old_co2_scd != sensor_data_storage.scd_co2)
        {
        	genieWriteStr (1, level_str);
        	old_co2_scd = sensor_data_storage.scd_co2;
        }

        gauge_level = (uint16_t)(sensor_data_storage.scd_co2/30);
        if(gauge_level > 100)
        {gauge_level = 100;}
        genieWriteObject(GENIE_OBJ_GAUGE, 2, gauge_level);


		/*Speech engine periodic check*/
		if(check)
		{
			/*Self lock-out*/
			check = false;

			/*Check the current status of the CO2 concentration*/
			co2_state = DecodeCO2Level(sensor_data_storage.pas_co2);

			/*Play warning messages according to the CO2 concentration*/
			msg_status = pdFAIL;
			switch (co2_state)
			{
				case CO2_NORMAL:
				{
					/*Play the message only once on state change*/
					if(co2_state_prev != CO2_NORMAL)
					{
						speech_msg.phrase_number = CO2_NORMAL_PHRASE;
						msg_status = xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
						if(msg_status == pdPASS)
						{
							if(phrase_repeat != NULL)
							{
								xTimerDelete(phrase_repeat, 100);
								phrase_repeat = NULL;
							}

							co2_state_prev = CO2_NORMAL;
						}
					}
					break;
				}
				case CO2_MEDIUM:
				{
					/*Play the message every xx seconds*/
					if(co2_state_prev != CO2_MEDIUM)
					{
						speech_msg.phrase_number = CO2_MEDIUM_PHRASE;
						msg_status = xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
						if(msg_status == pdPASS)
						{
							if(phrase_repeat != NULL)
							{
								xTimerDelete(phrase_repeat, 100);
								phrase_repeat = NULL;
							}

							co2_state_prev = CO2_MEDIUM;

							phrase_repeat = xTimerCreate("Repeat", CO2_MEDIUM_REPEAT, pdFALSE, &ph_rep_id, Phrase_Repeat_Callback);
							xTimerStart(phrase_repeat, 100);
						}
					}
					break;
				}
				case CO2_MAXIMUM:
				{
					/*Play the message every xx seconds*/
					if(co2_state_prev != CO2_MAXIMUM)
					{
						speech_msg.phrase_number = CO2_MAXIMUM_PHRASE;
						msg_status = xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
						if(msg_status == pdPASS)
						{
							if(phrase_repeat != NULL)
							{
								xTimerDelete(phrase_repeat, 100);
								phrase_repeat = NULL;
							}

							co2_state_prev = CO2_MAXIMUM;

							phrase_repeat = xTimerCreate("Repeat", CO2_MAXIMUM_REPEAT, pdFALSE, &ph_rep_id, Phrase_Repeat_Callback);
							xTimerStart(phrase_repeat, 100);
						}
					}
					break;
				}
				case CO2_UNDEFINED:
				{
					if(phrase_repeat != NULL)
					{
						xTimerDelete(phrase_repeat, 100);
						phrase_repeat = NULL;
					}
					break;
				}
				default:
				{
					break;
				}
			}

			/*Start the timer which will activate next speech engine check*/
			if(speech_check != NULL)
			{
				xTimerDelete(speech_check, 100);
				speech_check = NULL;
			}
			speech_check = xTimerCreate("Check", SPEECH_ENGINE_CHECK, pdFALSE, &sp_ch_id, Speech_Check_Callback);
			xTimerStart(speech_check, 100);
		}
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
	vTaskDelay(pdMS_TO_TICKS(3000));
}

/*ViSi Genie Event Handler*/
static void myGenieEventHandler(void)
{
  GenieFrame Event;
  genieDequeueEvent(&Event);
  int32_t button_val = 0;
  char msg_str[16] = {0};

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

    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)
    {
  	  if (Event.reportObject.index == 5)
  	  {
  		  genieWriteObject(GENIE_OBJ_FORM, 3, 1);
          memset(msg_str, 0x00, sizeof(msg_str));
          sprintf(msg_str, "%d ppm", sensor_data_storage.pas_co2);
          genieWriteStr (0, msg_str);

          memset(msg_str, 0x00, sizeof(msg_str));
          sprintf(msg_str, "%d ppm", sensor_data_storage.scd_co2);
          genieWriteStr (1, msg_str);
  	  }
  	  if (Event.reportObject.index == 6)
  	  {
  		genieWriteObject(GENIE_OBJ_FORM, 2, 1);
  	  }
  	  if (Event.reportObject.index == 8)
  	  {
  		  genieWriteObject(GENIE_OBJ_FORM, 3, 1);
          memset(msg_str, 0x00, sizeof(msg_str));
          sprintf(msg_str, "%d ppm", sensor_data_storage.pas_co2);
          genieWriteStr (0, msg_str);

          memset(msg_str, 0x00, sizeof(msg_str));
          sprintf(msg_str, "%d ppm", sensor_data_storage.scd_co2);
          genieWriteStr (1, msg_str);
  	  }
    }
  }
}

static co2status_t DecodeCO2Level(int co2eq_ppm)
{
	if(co2eq_ppm >= LVL_NORMAL && co2eq_ppm < LVL_MEDIUM)
	{
		return CO2_NORMAL;
	}
	else if (co2eq_ppm >= LVL_MEDIUM && co2eq_ppm < LVL_MAXIMUM)
	{
		return CO2_MEDIUM;
	}
	else if (co2eq_ppm >=  LVL_MAXIMUM)
	{
		return CO2_MAXIMUM;
	}
	else

	return CO2_UNDEFINED;
}

void Phrase_Repeat_Callback(TimerHandle_t xTimer)
{
	xTimerDelete(phrase_repeat, 100);
	phrase_repeat = NULL;
	co2_state_prev = CO2_UNDEFINED;
}

void Speech_Check_Callback(TimerHandle_t xTimer)
{
	xTimerDelete(speech_check, 100);
	speech_check = NULL;
	check = true;
}
