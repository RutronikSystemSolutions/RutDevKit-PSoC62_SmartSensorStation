/*
 * speech_task.c
 *
 *  Created on: 2021-12-06
 *      Author: GDR
 */

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "spi_api.h"
#include "speech_task.h"

TaskHandle_t speech_task_handle = NULL;
QueueHandle_t speech_MsgQueue;

void speech_task(void *param)
{
	(void) param;
	speech_t msg = { .phrase_number = 0 };
	BaseType_t msg_status;

	printf("speech task has started.\r\n");

	for(;;)
	{
		msg_status = xQueueReceive( speech_MsgQueue, &(msg), portMAX_DELAY);
		if (msg_status == pdPASS)
		{
			/*Play the track...*/
			GPIO_ControlMute(1); /*Mute - OFF*/
			S1V30340_Play_Specific_Audio(msg.phrase_number);
			S1V30340_Wait_For_Termination();
			GPIO_ControlMute(0); /*Mute - ON*/
		}
	}
}
