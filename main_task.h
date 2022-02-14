/*
 * main_task.h
 *
 *  Created on: 2021-12-06
 *      Author: GDR
 */

#ifndef MAIN_TASK_H_
#define MAIN_TASK_H_

#define GREETING_PHRASE		0

extern TaskHandle_t main_task_handle;
void main_task(void *param);

#endif /* MAIN_TASK_H_ */
