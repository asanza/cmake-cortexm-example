/*
 * Copyright (c) 2023 Diego Asanza <f.asanza@gmail.com>
 * Created on Mon May 29 2023
 */

/* SPDX-License-Identifier: BSD 3-Clause  */

#include <FreeRTOS.h>
#include <task.h>
#include <hal_gpio.h>

#define LED1_PIN GPIO_PORTA(5)

static void blinky( void *args )
{
    hal_gpio_init_out(LED1_PIN, 1, false);
    while(1)
    {
        vTaskDelay(100);
        hal_gpio_toggle(LED1_PIN);
    }
}

int main( void )
{
    xTaskCreate(blinky, "blinky", 128, NULL, 1, NULL);
    vTaskStartScheduler();
    while(1);
}