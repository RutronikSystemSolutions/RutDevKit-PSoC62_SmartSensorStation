/*
 * sensors_task.c
 *
 *  Created on: 2021-12-06
 *      Author: GDR
 */

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "sensors_task.h"
#include "bme688_app.h"
#include "bmp390_app.h"
#include "bmi270_app.h"
#include "sht4x.h"
#include "sgp40.h"
#include "sensirion_voc_algorithm.h"
#include "scd4x.h"
#include "xensiv_pasco2_mtb.h"
#include "math.h"

/*Priority for sensor interrupts*/
#define MOSE_IRQ_PRIORITY		7

#define DEFAULT_PRESSURE_REF_HPA    (0x3F7)     /* Default atmospheric pressure to compensate for (hPa) */

TaskHandle_t env_sensors_task_handle = NULL;
TaskHandle_t env_note_handle = NULL;
SemaphoreHandle_t i2c_mutex = NULL;
TaskHandle_t mot_sensors_task_handle = NULL;
TaskHandle_t mot_note_handle = NULL;

/*SCP Timer Object */
cyhal_timer_t env_sensors_timer;

sensor_data_t sensor_data_storage = {0};

xensiv_pasco2_t xensiv_pasco2;

extern cyhal_i2c_t I2C_scb3;

_Bool motion = false;

/*VOC Index Algorithm Parameters*/
VocAlgorithmParams voc_algorithm_params;

void imu_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);
static void env_sens_timer_isr(void *callback_arg, cyhal_timer_event_t event);
static cy_rslt_t env_sens_timer_init(void);

/*Sensor Fusion Interrupt Data*/
cyhal_gpio_callback_data_t imu_int_data =
{
		.callback = imu_interrupt_handler,
		.callback_arg = NULL,

};

void env_sensors_task(void *param)
{
	(void) param;
	cy_rslt_t result;
	uint32_t signal;
    int8_t rslt = 0;
    uint8_t n_fields;
    uint16_t serial_number[3];
    int16_t error = 0;
    static _Bool sensor_read = false;
    uint16_t co2_ppm = 0;

    /*Initialize Sensor Fusion Board Interrupt*/
    result = cyhal_gpio_init(ARDU_IO2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    /*Register callback function */
    cyhal_gpio_register_callback(ARDU_IO2, &imu_int_data);
    /* Enable rising edge interrupt events */
    cyhal_gpio_enable_event(ARDU_IO2, CYHAL_GPIO_IRQ_RISE, MOSE_IRQ_PRIORITY, true);

    /*Initialize the BMP390 Sensor*/
    sensor_data_storage.bmp_altitude_offset = 0.0;
    result = bmp390_app_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Initialize the BME688 Sensor*/
    result = bme688_app_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Check the SHT4x sensor*/
    if(sht4x_probe() != SHT4X_STATUS_OK)
    {
    	printf("SHT4x Sensor Failure\n\r");
    	CY_ASSERT(0);
    }

    /*Check the SGP40 sensor*/
    if(sgp40_probe()  != SGP40_STATUS_OK)
    {
    	printf("SGP40 Sensor Failure\n\r");
    	CY_ASSERT(0);
    }

    /*VOC Index Algorithm Stack Initialization*/
    VocAlgorithm_init(&voc_algorithm_params);

    /*Initiate the SCD4x sensor*/
    scd4x_wake_up();
    scd4x_stop_periodic_measurement();
    scd4x_reinit();
    error = scd4x_get_serial_number(&serial_number[0], &serial_number[1], &serial_number[2]);
    if (error)
    {
    	printf("SCD4x Sensor Failure\n\r");
    	CY_ASSERT(0);
    }
    else
    {
        printf("SCD4x serial: 0x%04x%04x%04x\n\r", serial_number[0], serial_number[1], serial_number[2]);
        error = scd4x_start_periodic_measurement();
        if(error)
        {
        	printf("Could not start SCD4x periodic measurements\n\r");
        	CY_ASSERT(0);
        }
    }

    /* Initialize PAS CO2 sensor with default parameter values */
    result = xensiv_pasco2_mtb_init_i2c(&xensiv_pasco2, &I2C_scb3);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("PAS CO2 device initialization error.\n\r");
        CY_ASSERT(0);
    }

    /*Initialize SensorsTimer*/
    result = env_sens_timer_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Create a task for motion sensors data collection*/
    xTaskCreate(motion_sensors_task, "mot_sensors task", configMINIMAL_STACK_SIZE*4, NULL, configMAX_PRIORITIES - 0, &mot_sensors_task_handle);
    if(mot_sensors_task_handle == NULL)
    {
    	printf("Error: could not create mot_sensors task.\r\n");
    	CY_ASSERT(0);
    }

    /*Initialize the BMI270 Sensor*/
    result = bmi270_app_init();
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    printf("sensors task has started.\r\n");
    for(;;)
    {
    	env_note_handle = xTaskGetCurrentTaskHandle();
    	signal = ulTaskNotifyTakeIndexed( 0, pdTRUE, portMAX_DELAY );

    	/*Collect the data from the sensors every 100ms*/
    	if(signal)
    	{
    		cyhal_gpio_toggle(LED1);

        	/*** Read the BMP390 data ***/
            rslt = bmp3_get_status(&bmp_status, &dev);
            /* Read temperature, pressure and pressure data */
            if (rslt == BMP3_OK)
            {
                rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &bmp_data, &dev);
                /* NOTE : Read status register again to clear data ready interrupt status */
                (void) bmp3_get_status(&bmp_status, &dev);
                sensor_data_storage.bmp_temperature = bmp_data.temperature;
                sensor_data_storage.bmp_pressure = bmp_data.pressure;
                sensor_data_storage.bmp_altitude = 44330.0 * (1.0 - pow(sensor_data_storage.bmp_pressure / 101325, 1/5.255));
            }

            /*** Read the BME688 data ***/
            rslt = bme68x_get_data(BME68X_PARALLEL_MODE, bme_data, &n_fields, &bme);
            for (uint8_t i = 0; i < n_fields; i++)
            {
                if (bme_data[i].status == BME68X_VALID_DATA)
                {
                	sensor_data_storage.bme_temperature = bme_data[i].temperature;
                	sensor_data_storage.bme_humidity = bme_data[i].humidity;
                	sensor_data_storage.bme_pressure = bme_data[i].pressure;
                	sensor_data_storage.bme_gas_resistance = bme_data[i].gas_resistance;
                	sensor_data_storage.bme_gas_index = bme_data[i].gas_index;
                }
            }

            /*** Measure / Read the SGP40, SHT41, SCD41, PAS_CO2 data  ***/
            if(!sensor_read)
            {
            	sht4x_measure();
            	sgp40_measure_raw_with_rht(sensor_data_storage.sht_humidity, sensor_data_storage.sht_temperature);
            	scd4x_set_ambient_pressure((uint16_t)(sensor_data_storage.bmp_pressure/100));
            }
            else
            {
            	sht4x_read(&sensor_data_storage.sht_temperature, &sensor_data_storage.sht_humidity);
            	sgp40_read_raw(&sensor_data_storage.sgp_sraw_voc);
            	VocAlgorithm_process(&voc_algorithm_params, sensor_data_storage.sgp_sraw_voc, &sensor_data_storage.sgp_voc_index);
            	scd4x_read_measurement(&sensor_data_storage.scd_co2, &sensor_data_storage.scd_temperature, &sensor_data_storage.scd_humidity);
            	result = xensiv_pasco2_mtb_read(&xensiv_pasco2, (uint16_t)(sensor_data_storage.bmp_pressure/100), &co2_ppm);
                if (result == CY_RSLT_SUCCESS)
                {
                    sensor_data_storage.pas_co2 = co2_ppm;
                    printf("CO2 PAS %d ppm CO2 SCD %d ppm\r\n", sensor_data_storage.pas_co2, sensor_data_storage.scd_co2);
                }
            }

            /*Toggle measure/read cycles*/
            sensor_read = !sensor_read;
    	}
    }
}

static void env_sens_timer_isr(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(env_note_handle != NULL)
    {
        vTaskNotifyGiveIndexedFromISR( env_note_handle, 0, &xHigherPriorityTaskWoken );
        env_note_handle = NULL;
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

static cy_rslt_t env_sens_timer_init(void)
{
	 cy_rslt_t result;
	 const cyhal_timer_cfg_t scp_cfg =
	 {
	     .compare_value = 0,                 /* Timer compare value, not used */
	     .period = 999,                       /* Defines the timer period - 10 Hz */
	     .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
	     .is_compare = false,                /* Don't use compare mode */
	     .is_continuous = true,              /* Run the timer indefinitely */
	     .value = 0                          /* Initial value of counter */
	 };

	 result = cyhal_timer_init(&env_sensors_timer, NC, NULL);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 result = cyhal_timer_configure(&env_sensors_timer, &scp_cfg);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 result = cyhal_timer_set_frequency(&env_sensors_timer, 10000);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 cyhal_timer_register_callback(&env_sensors_timer, env_sens_timer_isr, NULL);

	 cyhal_timer_enable_event(&env_sensors_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3, true);

	 result =  cyhal_timer_start(&env_sensors_timer);
	 if (result != CY_RSLT_SUCCESS)
	 {return result;}

	 return result;
}

void motion_sensors_task(void *param)
{
	(void) param;
	uint32_t signal;
	int8_t rslt;
	_Bool skip_first = true;

    for(;;)
    {
    	begin:
    	mot_note_handle = xTaskGetCurrentTaskHandle();
    	signal = ulTaskNotifyTakeIndexed( 0, pdTRUE, portMAX_DELAY );
    	if(signal)
    	{
    		cyhal_gpio_toggle(LED2);

            /* Get accel and gyro data for x, y and z axis. */
            rslt = bmi270_get_sensor_data(sensor_data, 2, &bmi2_dev);
            if(rslt == BMI2_OK)
            {
            	sensor_data_storage.bmi_acc_x = sensor_data[ACCEL].sens_data.acc.x;
            	sensor_data_storage.bmi_acc_y = sensor_data[ACCEL].sens_data.acc.y;
            	sensor_data_storage.bmi_acc_z = sensor_data[ACCEL].sens_data.acc.z;
            	sensor_data_storage.bmi_gyr_x = sensor_data[GYRO].sens_data.gyr.x;
            	sensor_data_storage.bmi_gyr_y = sensor_data[GYRO].sens_data.gyr.y;
            	sensor_data_storage.bmi_gyr_z = sensor_data[GYRO].sens_data.gyr.z;
            }
            else
            {
                bmi2_error_codes_print_result(rslt);
                if(rslt != BMI2_OK)
                {
                	CY_ASSERT(0);
                }
            }

            /*Clear the interrupt status, detect motion*/
            rslt = bmi2_get_int_status(&int_status, &bmi2_dev);
            if (int_status & BMI270_ANY_MOT_STATUS_MASK)
            {
            	if(skip_first)
            	{
            		skip_first = false;
            		goto begin;
            	}

            	/*Motion detected*/
            	motion = true;
            }
            bmi2_error_codes_print_result(rslt);
            if(rslt != BMI2_OK)
            {
            	CY_ASSERT(0);
            }
    	}
    }
}

/* Interrupt handler callback function */
void imu_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
	CY_UNUSED_PARAMETER(handler_arg);
    CY_UNUSED_PARAMETER(event);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(mot_note_handle != NULL)
    {
        vTaskNotifyGiveIndexedFromISR( mot_note_handle, 0, &xHigherPriorityTaskWoken );
        mot_note_handle = NULL;
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}
