
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
#define WDT_USED
#define WDT_TIME_OUT_MS             4000

/* WDT object */
cyhal_wdt_t wdt_obj;

/*4D Systems display uart initialization*/
static cy_rslt_t ardu_uart_init();

void initialize_wdt(void);

/* UserApiConfig */
static bool uartAvailHandler(void);
static uint8_t uartReadHandler(void);
static void uartWriteHandler(uint32_t val);
static uint32_t uartGetMillis(void);
static void resetDisplay(void);

/* Display Event Handler */
static void myGenieEventHandler(void);

/*Speech engine functions*/
void Speech_Check_Callback(TimerHandle_t xTimer);
void Phrase_Repeat_Callback(TimerHandle_t xTimer);
void Form0_Activate_Callback(TimerHandle_t xTimer);
void Temp_Reset_Callback(TimerHandle_t xTimer);
void VOC_Reset_Callback(TimerHandle_t xTimer);
void Humid_Reset_Callback(TimerHandle_t xTimer);
static co2status_t DecodeCO2Level(int co2eq_ppm);
static voc_status_t DecodeVOCLevel(int voc);
static temp_status_t DecodeTempLevel(int temperature);
static humid_status_t DecodeHumidLevel(int humidity);

TaskHandle_t main_task_handle = NULL;
TimerHandle_t phrase_repeat = NULL;
TimerHandle_t speech_check = NULL;
TimerHandle_t form0_activate = NULL;
TimerHandle_t voc_flag_reset = NULL;
TimerHandle_t temp_flag_reset = NULL;
TimerHandle_t humid_flag_reset = NULL;

/*Arduino UART object and configuration*/
cyhal_uart_t ardu_uart;
static UserApiConfig userConfig =
{
	.available = uartAvailHandler,
	.read =  uartReadHandler,
	.write = uartWriteHandler,
	.millis = uartGetMillis
};

long ph_rep_id = 1;
long sp_ch_id = 2;
long form_act_id = 3;
long voc_tim_id = 4;
long temp_tim_id = 5;
long hum_tim_id = 6;

co2status_t co2_state_prev = CO2_UNDEFINED;
_Bool check_co2 = false;
_Bool form0_return = false;
_Bool voc_ok = true;
_Bool temp_ok = true;
_Bool humid_ok = true;

void main_task(void *param)
{
	(void) param;
	cy_rslt_t result;
	int ecfg_result = -1;
	uint8_t motion_led_cnt = 0;
	uint8_t motion_alm_cnt = 0;
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
	voc_status_t voc_state = VOC_UNDEFINED;
	temp_status_t temp_state = TEMP_UNDEFINED;
	humid_status_t humid_state = HUMID_UNDEFINED;

	printf("main task has started.\r\n");

    /*Initialize The Arduino UART*/
    result = ardu_uart_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize ViSi-Genie*/
    genieInitWithConfig(&userConfig);
    genieAttachEventHandler(myGenieEventHandler);
    resetDisplay();
    vTaskDelay(pdMS_TO_TICKS(1000));
    genieWriteContrast(12);
    vTaskDelay(pdMS_TO_TICKS(1000));
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

	/*CO2 speech engine check timer*/
	speech_check = xTimerCreate("Check", SPEECH_ENGINE_INIT, pdFALSE, &sp_ch_id, Speech_Check_Callback);
	xTimerStart(speech_check, 100);

	/*Home page return timer*/
	form0_activate = xTimerCreate("Form0Act", ACTIVATE_FORM0_MSEC, pdTRUE, &form_act_id, Form0_Activate_Callback);
	xTimerStart(form0_activate, 100);

	/*VOC alarm reset timer*/
	voc_flag_reset = xTimerCreate("VOCRst", VOC_ALARM_REPEAT, pdTRUE, &voc_tim_id, VOC_Reset_Callback);
	xTimerStart(voc_flag_reset, 100);

	/*Temperature alarm reset timer*/
	temp_flag_reset = xTimerCreate("TempRst", TEMP_ALARM_REPEAT, pdTRUE, &temp_tim_id, Temp_Reset_Callback);
	xTimerStart(temp_flag_reset, 100);

	/*Humidity alarm reset timer*/
	humid_flag_reset = xTimerCreate("HumidRst", HUMID_ALARM_REPEAT, pdTRUE, &hum_tim_id, Humid_Reset_Callback);
	xTimerStart(humid_flag_reset, 100);

	/*Initialize the WDT*/
#ifdef WDT_USED
	initialize_wdt();
#endif

	for(;;)
	{
#ifdef WDT_USED
        /* Reset WDT */
        cyhal_wdt_kick(&wdt_obj);
#endif

        /*Visi Genie Events*/
        genieDoEvents(true);

        /*Return to Form0 on request*/
        if(form0_return)
        {
        	form0_return = false;
        	genieWriteObject(GENIE_OBJ_FORM, 0, 1);
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
        genieWriteObject(GENIE_OBJ_SCOPE, 4, (int16_t)(sensor_data_storage.bmi_acc_y * SCOPE_SCALE));
        genieWriteObject(GENIE_OBJ_SCOPE, 5, (int16_t)(sensor_data_storage.bmi_acc_z * SCOPE_SCALE));
        genieWriteObject(GENIE_OBJ_SCOPE, 1, (int16_t)(sensor_data_storage.bmi_gyr_x * SCOPE_SCALE));
        genieWriteObject(GENIE_OBJ_SCOPE, 2, (int16_t)(sensor_data_storage.bmi_gyr_y * SCOPE_SCALE));
        genieWriteObject(GENIE_OBJ_SCOPE, 3, (int16_t)(sensor_data_storage.bmi_gyr_z * SCOPE_SCALE));
        if(motion)
        {
        	if(!motion_alm_cnt)
        	{
        		motion_alm_cnt = 154;
        		speech_msg.phrase_number = MOTION_DETECTED;
        		(void)xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 0 );
        	}
        	motion_led_cnt = 8;
        	motion = false;
        }
        if(motion_led_cnt)
        {
        	motion_led_cnt--;
        	genieWriteObject(GENIE_OBJ_USER_LED, 0, 1);
        	if(!motion_led_cnt)
        	{
        		genieWriteObject(GENIE_OBJ_USER_LED, 0, 0);
        	}
        }
        if(motion_alm_cnt)
        {
        	motion_alm_cnt--;
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
            sprintf(level_str, "%.2f", old_bmp_alt - sensor_data_storage.bmp_altitude_offset);
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

        /*Form5*/
        gauge_level = sensor_data_storage.slider_pos;
        if(gauge_level > 99)
        {gauge_level = 99;}
        genieWriteObject(35, 0, gauge_level);

		/*CO2 speech engine periodic check*/
		if(check_co2)
		{
			/*Self lock-out*/
			check_co2 = false;

			/*Check the current status of the CO2 concentration*/
			co2_state = DecodeCO2Level(sensor_data_storage.scd_co2);

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

        /*Form6*/
        genieWriteObject(GENIE_OBJ_ANGULAR_METER, 1, (int16_t)sensor_data_storage.sgp_voc_index);
        genieWriteObject(44, 1, (int16_t)((sensor_data_storage.sht_temperature)/1000));
        genieWriteObject(GENIE_OBJ_GAUGE, 3, (int16_t)(sensor_data_storage.sht_humidity/1000));
        if(old_sht_temp != sensor_data_storage.sht_temperature)
        {
        	old_sht_temp = sensor_data_storage.sht_temperature;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%.2f", (float)(sensor_data_storage.sht_temperature/1000.0));
            genieWriteStr (18, level_str);
        }

        if(old_sht_hum != sensor_data_storage.sht_humidity)
        {
        	old_sht_hum = sensor_data_storage.sht_humidity;
            memset(level_str, 0x00, sizeof(level_str));
            sprintf(level_str, "%.2f", (float)(sensor_data_storage.sht_humidity/1000.0));
            genieWriteStr (19, level_str);
        }

        voc_state = DecodeVOCLevel(sensor_data_storage.sgp_voc_index);
		if(voc_state != VOC_NORMAL)
		{
        	if(voc_ok)
        	{
        		voc_ok = false;

        		if(voc_state == VOC_MEDIUM)
        		{speech_msg.phrase_number = VOC_MEDIUM_PHRASE;}
        		else
        		{speech_msg.phrase_number = VOC_HIGH_PHRASE;}

        		(void)xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 0 );
        	}
		}

        temp_state = DecodeTempLevel(sensor_data_storage.sht_temperature/1000);
        if(temp_state != TEMP_NORMAL)
        {
        	if(temp_ok)
        	{
        		temp_ok = false;

        		if(temp_state == TEMP_LOW)
        		{speech_msg.phrase_number = TEMP_LOW_PHRASE;}
        		else
        		{speech_msg.phrase_number = TEMP_HIGH_PHRASE;}

        		(void)xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 0 );
        	}
        }

        humid_state = DecodeHumidLevel(sensor_data_storage.sht_humidity/1000);
        if(humid_state != HUMID_NORMAL)
        {
        	if(humid_ok)
        	{
        		humid_ok = false;

        		if(humid_state == HUMID_LOW)
        		{speech_msg.phrase_number = HUMID_LOW_PHRASE;}
        		else
        		{speech_msg.phrase_number = HUMID_HIGH_PHRASE;}

        		(void)xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 0 );
        	}
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
  char msg_str[16] = {0};
  co2status_t co2_state = CO2_UNDEFINED;
  voc_status_t voc_state = VOC_UNDEFINED;
  temp_status_t temp_state = TEMP_UNDEFINED;
  humid_status_t humid_state = HUMID_UNDEFINED;
  speech_t speech_msg;

  if (Event.reportObject.cmd == GENIE_REPORT_EVENT)
  {
    if (Event.reportObject.object == GENIE_OBJ_WINBUTTON)
    {
    	xTimerReset(form0_activate, 100);

    	if (Event.reportObject.index == 20)
    	{genieWriteObject(GENIE_OBJ_FORM, 6, 1);}

    	if (Event.reportObject.index == 2)
    	{genieWriteObject(GENIE_OBJ_FORM, 6, 1);}

    	if (Event.reportObject.index == 4)
    	{genieWriteObject(GENIE_OBJ_FORM, 1, 1);}

    	if (Event.reportObject.index == 15)
    	{genieWriteObject(GENIE_OBJ_FORM, 13, 1);}

    	if (Event.reportObject.index == 8)
    	{genieWriteObject(GENIE_OBJ_FORM, 11, 1);}

    	if (Event.reportObject.index == 17)
    	{genieWriteObject(GENIE_OBJ_FORM, 0, 1);}

    	if (Event.reportObject.index == 9)
    	{genieWriteObject(GENIE_OBJ_FORM, 4, 1);}

    	if (Event.reportObject.index == 3)
    	{
    		genieWriteObject(GENIE_OBJ_FORM, 2, 1);

            memset(msg_str, 0x00, sizeof(msg_str));
            sprintf(msg_str, "%.2f", sensor_data_storage.bme_temperature);
            genieWriteStr (13, msg_str);

            memset(msg_str, 0x00, sizeof(msg_str));
            sprintf(msg_str, "%.2f", sensor_data_storage.bme_humidity);
            genieWriteStr (15, msg_str);

            memset(msg_str, 0x00, sizeof(msg_str));
            sprintf(msg_str, "%.2f", sensor_data_storage.bme_pressure/1000);
            genieWriteStr (12, msg_str);

            memset(msg_str, 0x00, sizeof(msg_str));
            sprintf(msg_str, "%.0f", sensor_data_storage.bme_gas_resistance/100000);
            genieWriteStr (14, msg_str);

            memset(msg_str, 0x00, sizeof(msg_str));
            sprintf(msg_str, "%d", sensor_data_storage.bme_gas_index);
            genieWriteStr (16, msg_str);
    	}

    	if (Event.reportObject.index == 7)
    	{genieWriteObject(GENIE_OBJ_FORM, 5, 1);}

    	if (Event.reportObject.index == 13)
    	{genieWriteObject(GENIE_OBJ_FORM, 9, 1);}

    	if (Event.reportObject.index == 14)
    	{genieWriteObject(GENIE_OBJ_FORM, 7, 1);}

    	if (Event.reportObject.index == 16)
    	{genieWriteObject(GENIE_OBJ_FORM, 15, 1);}

    	if (Event.reportObject.index == 11)
    	{
    		genieWriteObject(GENIE_OBJ_FORM, 3, 1);

    		memset(msg_str, 0x00, sizeof(msg_str));
    		sprintf(msg_str, "%d ppm", sensor_data_storage.pas_co2);
    		genieWriteStr (0, msg_str);

    		memset(msg_str, 0x00, sizeof(msg_str));
    		sprintf(msg_str, "%d ppm", sensor_data_storage.scd_co2);
    		genieWriteStr (1, msg_str);
    	}

    	if (Event.reportObject.index == 12)
    	{genieWriteObject(GENIE_OBJ_FORM, 0, 1);}

    	if (Event.reportObject.index == 0)
    	{genieWriteObject(GENIE_OBJ_FORM, 0, 1);}

    	if (Event.reportObject.index == 18)
    	{genieWriteObject(GENIE_OBJ_FORM, 1, 1);}

    	if (Event.reportObject.index == 22)
    	{genieWriteObject(GENIE_OBJ_FORM, 8, 1);}

    	if (Event.reportObject.index == 21)
    	{genieWriteObject(GENIE_OBJ_FORM, 4, 1);}

    	if (Event.reportObject.index == 23)
    	{genieWriteObject(GENIE_OBJ_FORM, 7, 1);}

    	if (Event.reportObject.index == 24)
    	{genieWriteObject(GENIE_OBJ_FORM, 10, 1);}

    	if (Event.reportObject.index == 25)
    	{genieWriteObject(GENIE_OBJ_FORM, 4, 1);}

    	if (Event.reportObject.index == 26)
    	{genieWriteObject(GENIE_OBJ_FORM, 9, 1);}

    	if (Event.reportObject.index == 28)
    	{genieWriteObject(GENIE_OBJ_FORM, 12, 1);}

    	if (Event.reportObject.index == 27)
    	{genieWriteObject(GENIE_OBJ_FORM, 4, 1);}

    	if (Event.reportObject.index == 29)
    	{genieWriteObject(GENIE_OBJ_FORM, 11, 1);}

    	if (Event.reportObject.index == 31)
    	{genieWriteObject(GENIE_OBJ_FORM, 14, 1);}

    	if (Event.reportObject.index == 30)
    	{genieWriteObject(GENIE_OBJ_FORM, 4, 1);}

    	if (Event.reportObject.index == 32)
    	{genieWriteObject(GENIE_OBJ_FORM, 13, 1);}

    	if (Event.reportObject.index == 33)
    	{genieWriteObject(GENIE_OBJ_FORM, 4, 1);}

    	if (Event.reportObject.index == 6)
    	{
    		genieWriteObject(GENIE_OBJ_FORM, 2, 1);

            memset(msg_str, 0x00, sizeof(msg_str));
            sprintf(msg_str, "%.2f", sensor_data_storage.bme_temperature);
            genieWriteStr (13, msg_str);

            memset(msg_str, 0x00, sizeof(msg_str));
            sprintf(msg_str, "%.2f", sensor_data_storage.bme_humidity);
            genieWriteStr (15, msg_str);

            memset(msg_str, 0x00, sizeof(msg_str));
            sprintf(msg_str, "%.2f", sensor_data_storage.bme_pressure/1000);
            genieWriteStr (12, msg_str);

            memset(msg_str, 0x00, sizeof(msg_str));
            sprintf(msg_str, "%.0f", sensor_data_storage.bme_gas_resistance/100000);
            genieWriteStr (14, msg_str);

            memset(msg_str, 0x00, sizeof(msg_str));
            sprintf(msg_str, "%d", sensor_data_storage.bme_gas_index);
            genieWriteStr (16, msg_str);
    	}

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

    	if (Event.reportObject.index == 10)
    	{
			co2_state = DecodeCO2Level(sensor_data_storage.scd_co2);
			switch (co2_state)
			{
				case CO2_NORMAL:
				{
					speech_msg.phrase_number = CO2_NORMAL_PHRASE;
					xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
					break;
				}
				case CO2_MEDIUM:
				{
					speech_msg.phrase_number = CO2_MEDIUM_PHRASE;
					xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
					break;
				}
				case CO2_MAXIMUM:
				{
					speech_msg.phrase_number = CO2_MAXIMUM_PHRASE;
					xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
					break;
				}
				case CO2_UNDEFINED:
				{
					break;
				}
				default:
				{
					break;
				}
			}
    	}

    	if (Event.reportObject.index == 19)
    	{
			voc_state = DecodeVOCLevel(sensor_data_storage.sgp_voc_index);
			switch (voc_state)
			{
				case VOC_NORMAL:
				{
					speech_msg.phrase_number = VOC_NORMAL_PHRASE;
					xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
					break;
				}
				case VOC_MEDIUM:
				{
					speech_msg.phrase_number = VOC_MEDIUM_PHRASE;
					xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
					break;
				}
				case VOC_HIGH:
				{
					speech_msg.phrase_number = VOC_HIGH_PHRASE;
					xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
					break;
				}
				case VOC_UNDEFINED:
				{
					break;
				}
				default:
				{
					break;
				}
			}

			temp_state = DecodeTempLevel(sensor_data_storage.sht_temperature/1000);
			switch (temp_state)
			{
				case TEMP_NORMAL:
				{
					speech_msg.phrase_number = TEMP_NORMAL_PHRASE;
					xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
					break;
				}
				case TEMP_LOW:
				{
					speech_msg.phrase_number = TEMP_LOW_PHRASE;
					xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
					break;
				}
				case TEMP_HIGH:
				{
					speech_msg.phrase_number = TEMP_HIGH_PHRASE;
					xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
					break;
				}
				case TEMP_UNDEFINED:
				{
					break;
				}
				default:
				{
					break;
				}
			}

			humid_state = DecodeHumidLevel(sensor_data_storage.sht_humidity/1000);
			switch (humid_state)
			{
				case HUMID_NORMAL:
				{
					speech_msg.phrase_number = HUMID_NORMAL_PHRASE;
					xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
					break;
				}
				case HUMID_LOW:
				{
					speech_msg.phrase_number = HUMID_LOW_PHRASE;
					xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
					break;
				}
				case HUMID_HIGH:
				{
					speech_msg.phrase_number = HUMID_HIGH_PHRASE;
					xQueueSend(speech_MsgQueue, (void *)&(speech_msg), 100 );
					break;
				}
				case HUMID_UNDEFINED:
				{
					break;
				}
				default:
				{
					break;
				}
			}
    	}
    }

    if (Event.reportObject.object == GENIE_OBJ_4DBUTTON)
    {
      if (Event.reportObject.index == 1)
      {
    	  sensor_data_storage.bmp_altitude_offset = sensor_data_storage.bmp_altitude;
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

static voc_status_t DecodeVOCLevel(int voc)
{
	if(voc >= VOC_LVL_NORMAL && voc < VOC_LVL_MEDIUM)
	{
		return VOC_NORMAL;
	}
	else if(voc >= VOC_LVL_MEDIUM && voc < VOC_LVL_HIGH)
	{
		return VOC_MEDIUM;
	}
	else if(voc >= VOC_LVL_HIGH)
	{
		return VOC_HIGH;
	}
	else

	return VOC_UNDEFINED;
}

static temp_status_t DecodeTempLevel(int temperature)
{
	if(temperature >= TEMP_LVL_NORMAL && temperature < TEMP_LVL_HIGH)
	{
		return TEMP_NORMAL;
	}
	else if(temperature < TEMP_LVL_NORMAL)
	{
		return TEMP_LOW;
	}
	else if(temperature >= TEMP_LVL_HIGH)
	{
		return TEMP_HIGH;
	}
	else

	return TEMP_UNDEFINED;
}

static humid_status_t DecodeHumidLevel(int humidity)
{
	if(humidity >= HUM_LVL_NORMAL && humidity < HUM_LVL_HIGH)
	{
		return HUMID_NORMAL;
	}
	else if(humidity < HUM_LVL_NORMAL)
	{
		return HUMID_LOW;
	}
	else if(humidity >= HUM_LVL_HIGH)
	{
		return HUMID_HIGH;
	}
	else

	return HUMID_UNDEFINED;
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
	check_co2 = true;
}

void Form0_Activate_Callback(TimerHandle_t xTimer)
{
	form0_return = true;
}

void Temp_Reset_Callback(TimerHandle_t xTimer)
{
	temp_ok = true;
}

void VOC_Reset_Callback(TimerHandle_t xTimer)
{
	voc_ok = true;
}

void Humid_Reset_Callback(TimerHandle_t xTimer)
{
	humid_ok = true;
}

void initialize_wdt()
{
    cy_rslt_t result;

    /* Initialize the WDT */
    result = cyhal_wdt_init(&wdt_obj, WDT_TIME_OUT_MS);

    /* WDT initialization failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}
