/*
 * Blink_LED.c
 *
 *  Created on: 2018年10月11日
 *      Author: YJP-E1
 */


#include "main.h"
#include "stm32f0xx_hal.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "Blink_LED.h"

static void BlinkLED_Task(void *pvParameters)
{
	TickType_t XlastFlashTime;
	XlastFlashTime = xTaskGetTickCount();

	while(1)
	{

		vTaskDelayUntil(&XlastFlashTime,500);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	}
}

void BlinkLED_Init()
{
	xTaskCreate(BlinkLED_Task,"BlinkLED",configMINIMAL_STACK_SIZE,NULL,1,NULL);
}
