/*
 * speech_task.h
 *
 *  Created on: 2021-12-06
 *      Author: GDR
 */

#ifndef SPEECH_TASK_H_
#define SPEECH_TASK_H_

typedef struct speech_engine
{
	 uint8_t phrase_number;
}speech_t;

extern TaskHandle_t speech_task_handle;
extern QueueHandle_t speech_MsgQueue;

void speech_task(void *param);

#endif /* SPEECH_TASK_H_ */
