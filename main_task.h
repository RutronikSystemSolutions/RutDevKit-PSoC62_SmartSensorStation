/*
 * main_task.h
 *
 *  Created on: 2021-12-06
 *      Author: GDR
 */

#ifndef MAIN_TASK_H_
#define MAIN_TASK_H_

#define GREETING_PHRASE			0
#define CO2_NORMAL_PHRASE		1
#define CO2_MEDIUM_PHRASE		2
#define CO2_MAXIMUM_PHRASE		3
#define MOTION_DETECTED			6

#define LVL_NORMAL		0
#define LVL_MEDIUM		1000
#define LVL_MAXIMUM		2000

#define CO2_MEDIUM_REPEAT	60000U
#define CO2_MAXIMUM_REPEAT	30000U
#define SPEECH_ENGINE_CHECK	5000U
#define SPEECH_ENGINE_INIT	10000U

typedef enum co2status
{
	CO2_NORMAL,
	CO2_MEDIUM,
	CO2_MAXIMUM,
	CO2_UNDEFINED
}co2status_t;

extern TaskHandle_t main_task_handle;
void main_task(void *param);

#endif /* MAIN_TASK_H_ */
