#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

xQueueHandle touchpadQueue;

#define touchpadSCL GPIO_NUM_22
#define touchpadSDO GPIO_NUM_21

static void IRAM_ATTR touchpad_gpio_isr_handler(void *arg)
{
	uint16_t bitmap = 0;
	gpio_isr_handler_remove(touchpadSDO);
	
	ets_delay_us(100);
	for (int i = 0; i < 16; i++)
	{
		gpio_set_level(touchpadSCL, 0);
		ets_delay_us(50);
		bitmap |= !gpio_get_level(touchpadSDO) << i;
		gpio_set_level(touchpadSCL, 1);
		ets_delay_us(50);
	}
	xQueueSendFromISR(touchpadQueue, &bitmap, NULL);
	vTaskDelay(2 / portTICK_RATE_MS);
	gpio_isr_handler_add(touchpadSDO, touchpad_gpio_isr_handler, (void *)touchpadSDO);
}

void touch_task(void *pvParameters)
{
	touchpadQueue = xQueueCreate(10, sizeof(uint16_t));

	gpio_config_t conf;

	conf.intr_type = GPIO_INTR_DISABLE;
	conf.mode = GPIO_MODE_OUTPUT;
	conf.pin_bit_mask = 1 << touchpadSCL;
	conf.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&conf);

	conf.intr_type = GPIO_INTR_NEGEDGE;
	conf.mode = GPIO_MODE_INPUT;
	conf.pin_bit_mask = 1 << touchpadSDO;
	conf.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&conf);

	gpio_set_level(touchpadSCL, 1);
	gpio_isr_handler_add(touchpadSDO, touchpad_gpio_isr_handler, (void *)touchpadSDO);

	uint16_t bitmap;
	while (1)
	{
		if (xQueueReceive(touchpadQueue, &bitmap, portMAX_DELAY))
			for (int i = 0; i < 16; i++)
				if (bitmap & (1 << i))
					printf("Touch at button %d\n", i + 1);
	}
}

void app_main()
{
	gpio_install_isr_service(0);
	xTaskCreatePinnedToCore(&touch_task, "touchtask", 2048, NULL, 5, NULL, 1);
}
