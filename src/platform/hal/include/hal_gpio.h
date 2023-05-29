/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */


#ifndef HAL_GPIO_H
#define HAL_GPIO_H
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define GPIO_PORTA(x) (16 * 0 + x)
#define GPIO_PORTB(x) (16 * 1 + x)
#define GPIO_PORTC(x) (16 * 2 + x)
#define GPIO_PORTD(x) (16 * 3 + x)

enum hal_gpio_pull {
  HAL_GPIO_PULL_NONE,
  HAL_GPIO_PULL_UP,
  HAL_GPIO_PULL_DOWN,
};

enum hal_gpio_irq_trigger {
  HAL_GPIO_TRIG_NONE,
  HAL_GPIO_TRIG_RISING,
  HAL_GPIO_TRIG_FALLING,
  HAL_GPIO_TRIG_BOTH,
  HAL_GPIO_TRIG_LOW,
  HAL_GPIO_TRIG_HIGH
};

typedef void (*hal_gpio_irq_handler_t)(void);

int hal_gpio_init_in(int pin, enum hal_gpio_pull pull);
int hal_gpio_init_out(int pin, int val, bool od);
int hal_gpio_init_af(int pin, enum hal_gpio_pull pull, uint8_t od);
void hal_gpio_write(int pin, int val);
int hal_gpio_read(int pin);
int hal_gpio_toggle(int pin);
int hal_gpio_irq_init(int pin, hal_gpio_irq_handler_t handler,
                      enum hal_gpio_irq_trigger trigger,
                      enum hal_gpio_pull pull);
void hal_gpio_irq_enable(int pin);
void hal_gpio_irq_disable(int pin);

#ifdef __cplusplus
}
#endif

#endif /* HAL_GPIO_H */
