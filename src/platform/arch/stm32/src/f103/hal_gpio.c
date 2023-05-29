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


#include <hal_gpio.h>
#include <stm32f1xx_hal_gpio.h>
#include <stm32f1xx_hal_rcc.h>
#include "hal_assert.h"

/* Helper functions to enable/disable interrupts. */
#define __HAL_DISABLE_INTERRUPTS(x)                                            \
  do {                                                                         \
    x = __get_PRIMASK();                                                       \
    __disable_irq();                                                           \
  } while (0);

#define __HAL_ENABLE_INTERRUPTS(x)                                             \
  do {                                                                         \
    if (!x) {                                                                  \
      __enable_irq();                                                          \
    }                                                                          \
  } while (0);

#define GPIO_MASK(pin) (1 << MCU_GPIO_PIN_NUM(pin))

/*
 * Helper macros to extract components from a pin descriptor.
 */
#define MCU_GPIO_PIN_NUM(pin) (0x000F & pin)
#define MCU_GPIO_PIN_PORT(pin) (0x000F & (pin >> 4))
#define MCU_GPIO_PIN_PAD(pin) (0x00FF & pin)

#if defined GPIOE_BASE
#define HAL_GPIO_PORT_COUNT (5)
#define HAL_GPIO_PORT_LIST GPIOA, GPIOB, GPIOC, GPIOD, GPIOE
#else
#error "No GPIO ports found - MCU not supported!"
#endif


/* Port index to port address map */
static GPIO_TypeDef *const portmap[HAL_GPIO_PORT_COUNT] = {HAL_GPIO_PORT_LIST};

/* Storage for GPIO callbacks. */
struct gpio_irq_obj {
  hal_gpio_irq_handler_t isr;
};

static struct gpio_irq_obj gpio_irq_handlers[16];

/**
 * ext irq handler
 *
 * Handles the gpio interrupt attached to a gpio pin.
 *
 * @param index
 */
static void ext_irq_handler(int index)
{
  uint32_t mask;

  mask = 1 << index;
  if (__HAL_GPIO_EXTI_GET_IT(mask) != RESET) {
    __HAL_GPIO_EXTI_CLEAR_IT(mask);
    gpio_irq_handlers[index].isr();
  }
}

/**
 * hal gpio clk enable
 *
 * Enable the port peripheral clock
 *
 * @param port_idx
 */
static void hal_gpio_clk_enable(uint32_t port_idx)
{
  switch (port_idx) {
  case 0:
    __HAL_RCC_GPIOA_CLK_ENABLE();
    break;
  case 1:
    __HAL_RCC_GPIOB_CLK_ENABLE();
    break;
  case 2:
    __HAL_RCC_GPIOC_CLK_ENABLE();
    break;
  case 3:
    __HAL_RCC_GPIOD_CLK_ENABLE();
    break;
  case 4:
    __HAL_RCC_GPIOE_CLK_ENABLE();
    break;
  default:
    HAL_ASSERT(0);
    break;
  }
}

/**
 * hal gpio pin to irq
 *
 * Converts the logical pin number to the IRQ number associated with the
 * external interrupt for that particular GPIO.
 *
 * @param pin
 *
 * @return IRQn_Type
 */
static IRQn_Type hal_gpio_pin_to_irq(int pin)
{
  int index;
  IRQn_Type irqn;

  index = MCU_GPIO_PIN_NUM(pin);
  if (index <= 4) {
    irqn = (IRQn_Type)(EXTI0_IRQn + index);
  } else if (index <= 9) {
    irqn = EXTI9_5_IRQn;
  } else {
    irqn = EXTI15_10_IRQn;
  }

  return irqn;
}


static void hal_gpio_set_nvic(IRQn_Type irqn)
{
  NVIC_EnableIRQ(irqn);
}

/**
 * hal gpio init
 *
 * Called to initialize a gpio.
 *
 * @param pin
 * @param cfg
 *
 * @return int
 */
static int hal_gpio_init_stm(int pin, GPIO_InitTypeDef *cfg)
{
  int port;
  uint32_t mcu_pin_mask;

  /* Is this a valid pin? */
  port = MCU_GPIO_PIN_PORT(pin);
  if (port >= HAL_GPIO_PORT_COUNT) {
    return -1;
  }

  mcu_pin_mask = GPIO_MASK(pin);
  cfg->Pin = mcu_pin_mask;

  /* Enable the GPIO clock */
  hal_gpio_clk_enable(port);

  /* Initialize pin as an input, setting proper mode */
  HAL_GPIO_Init(portmap[port], cfg);

  return 0;
}

/**
 * gpio init in
 *
 * Initializes the specified pin as an input
 *
 * @param pin   Pin number to set as input
 * @param pull  pull type
 *
 * @return int  0: no error; -1 otherwise.
 */
int hal_gpio_init_in(int pin, enum hal_gpio_pull pull)
{
  int rc;
  GPIO_InitTypeDef init_cfg;

  init_cfg.Mode = GPIO_MODE_INPUT;
  init_cfg.Pull = pull;

  rc = hal_gpio_init_stm(pin, &init_cfg);
  return rc;
}

/**
 * gpio init out
 *
 * Initialize the specified pin as an output, setting the pin to the specified
 * value.
 *
 * @param pin Pin number to set as output
 * @param val Value to set pin
 * @param od: if true output is open-drain, push-pull otherwise.
 *
 * @return int  0: no error; -1 otherwise.
 */
int hal_gpio_init_out(int pin, int val, bool od)
{
  GPIO_InitTypeDef cfg;
  int port;

  /* Is this a valid pin? */
  port = MCU_GPIO_PIN_PORT(pin);
  if (port >= HAL_GPIO_PORT_COUNT) {
    return -1;
  }

  /* Enable the GPIO clock */
  hal_gpio_clk_enable(port);

  /* Write initial output value */
  hal_gpio_write(pin, val);

  cfg.Pin = GPIO_MASK(pin);
  if(od == true){
    cfg.Mode = GPIO_MODE_OUTPUT_OD;
  } else {
    cfg.Mode = GPIO_MODE_OUTPUT_PP;
  }
  cfg.Pull = GPIO_NOPULL;
#if defined(GPIO_SPEED_FREQ_VERY_HIGH)
  cfg.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
#elif defined(GPIO_SPEED_HIGH)
  cfg.Speed = GPIO_SPEED_HIGH;
#else
  cfg.Speed = GPIO_SPEED_FREQ_HIGH;
#endif

  /* Initialize pin as an output, setting proper mode */
  HAL_GPIO_Init(portmap[port], &cfg);

  return 0;
}

/**
 * gpio init af
 *
 * Configure the specified pin for AF.
 */
int hal_gpio_init_af(int pin, enum hal_gpio_pull pull,
                     uint8_t od)
{
  GPIO_InitTypeDef gpio;

  if (!od) {
    gpio.Mode = GPIO_MODE_AF_PP;
  } else {
    gpio.Mode = GPIO_MODE_AF_OD;
  }
  gpio.Pull = pull;
#if defined(GPIO_SPEED_FREQ_VERY_HIGH)
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
#elif defined(GPIO_SPEED_HIGH)
  gpio.Speed = GPIO_SPEED_HIGH;
#else
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
#endif

  return hal_gpio_init_stm(pin, &gpio);
}

/**
 * gpio write
 *
 * Write a value (either high or low) to the specified pin.
 *
 * @param pin Pin to set
 * @param val Value to set pin (0:low 1:high)
 */
void hal_gpio_write(int pin, int val)
{
  int port;
  uint32_t mcu_pin_mask;
  GPIO_PinState state;

  port = MCU_GPIO_PIN_PORT(pin);
  mcu_pin_mask = GPIO_MASK(pin);

  if (val) {
    state = GPIO_PIN_SET;
  } else {
    state = GPIO_PIN_RESET;
  }

  HAL_GPIO_WritePin(portmap[port], mcu_pin_mask, state);
}

/**
 * gpio read
 *
 * Reads the specified pin.
 *
 * @param pin Pin number to read
 *
 * @return int 0: low, 1: high
 */
int hal_gpio_read(int pin)
{
  int port;
  uint32_t mcu_pin_mask;

  port = MCU_GPIO_PIN_PORT(pin);
  mcu_pin_mask = GPIO_MASK(pin);
  return HAL_GPIO_ReadPin(portmap[port], mcu_pin_mask);
}

/**
 * gpio toggle
 *
 * Toggles the specified pin
 *
 * @param pin Pin number to toggle
 *
 * @return current pin state int 0: low 1 : high
 */
int hal_gpio_toggle(int pin)
{
  int pin_state = (hal_gpio_read(pin) != 1);
  hal_gpio_write(pin, pin_state);
  return pin_state;
}

/**
 * gpio irq init
 *
 * Initialize an external interrupt on a gpio pin
 *
 * @param pin       Pin number to enable gpio.
 * @param handler   Interrupt handler
 * @param arg       Argument to pass to interrupt handler
 * @param trig      Trigger mode of interrupt
 * @param pull      Push/pull mode of input.
 *
 * @return int
 */
int hal_gpio_irq_init(int pin, hal_gpio_irq_handler_t handler,
                      enum hal_gpio_irq_trigger trig, enum hal_gpio_pull pull)
{
  int rc;
  int irqn;
  int index;
  uint32_t pin_mask;
  uint32_t mode;
  GPIO_InitTypeDef init_cfg;

  /* Configure the gpio for an external interrupt */
  rc = 0;
  switch (trig) {
  case HAL_GPIO_TRIG_NONE:
    rc = -1;
    break;
  case HAL_GPIO_TRIG_RISING:
    mode = GPIO_MODE_IT_RISING;
    break;
  case HAL_GPIO_TRIG_FALLING:
    mode = GPIO_MODE_IT_FALLING;
    break;
  case HAL_GPIO_TRIG_BOTH:
    mode = GPIO_MODE_IT_RISING_FALLING;
    break;
  case HAL_GPIO_TRIG_LOW:
    rc = -1;
    break;
  case HAL_GPIO_TRIG_HIGH:
    rc = -1;
    break;
  default:
    rc = -1;
    break;
  }

  /* Check to make sure no error has occurred */
  if (!rc) {
    /* Disable interrupt and clear any pending */
    hal_gpio_irq_disable(pin);
    pin_mask = GPIO_MASK(pin);
    __HAL_GPIO_EXTI_CLEAR_FLAG(pin_mask);

    /* Set the gpio irq handler */
    index = MCU_GPIO_PIN_NUM(pin);
    gpio_irq_handlers[index].isr = handler;

    /* Configure the GPIO */
    init_cfg.Mode = mode;
    init_cfg.Pull = pull;
    rc = hal_gpio_init_stm(pin, &init_cfg);
    if (!rc) {
      /* Enable interrupt vector in NVIC */
      irqn = hal_gpio_pin_to_irq(pin);
      hal_gpio_set_nvic((IRQn_Type)irqn);
    }
  }

  return rc;
}

/**
 * gpio irq release
 *
 * No longer interrupt when something occurs on the pin. NOTE: this function
 * does not change the GPIO push/pull setting nor does it change the
 * SYSCFG EXTICR registers. It also does not disable the NVIC interrupt enable
 * setting for the irq.
 *
 * @param pin
 */
void hal_gpio_irq_release(int pin)
{
  int index;
  uint32_t pin_mask;

  /* Disable the interrupt */
  hal_gpio_irq_disable(pin);

  /* Clear any pending interrupts */
  pin_mask = GPIO_MASK(pin);
  __HAL_GPIO_EXTI_CLEAR_FLAG(pin_mask);

  /* Clear out the irq handler */
  index = MCU_GPIO_PIN_NUM(pin);
  gpio_irq_handlers[index].isr = NULL;
}

/**
 * gpio irq enable
 *
 * Enable the irq on the specified pin
 *
 * @param pin
 */
void hal_gpio_irq_enable(int pin)
{
  uint32_t ctx;
  uint32_t mask;

  mask = GPIO_MASK(pin);

  __HAL_DISABLE_INTERRUPTS(ctx);
  EXTI->IMR |= mask;
  __HAL_ENABLE_INTERRUPTS(ctx);
}

/**
 * gpio irq disable
 *
 *
 * @param pin
 */
void hal_gpio_irq_disable(int pin)
{
  uint32_t ctx;
  uint32_t mask;

  mask = GPIO_MASK(pin);
  __HAL_DISABLE_INTERRUPTS(ctx);
  EXTI->IMR &= ~mask;
  __HAL_ENABLE_INTERRUPTS(ctx);
}

void EXTI0_IRQHandler(void)
{
  ext_irq_handler(0);
}

void EXTI1_IRQHandler(void)
{
  ext_irq_handler(1);
}

void EXTI2_IRQHandler(void)
{
  ext_irq_handler(2);
}

void EXTI3_IRQHandler(void)
{
  ext_irq_handler(3);
}

void EXTI4_IRQHandler(void)
{
  ext_irq_handler(4);
}

void EXTI9_5_IRQHandler(void)
{
  int index;

  for (index = 5; index <= 9; ++index) {
    ext_irq_handler(index);
  }
}

void EXTI15_10_IRQHandler(void)
{
  int index;

  for (index = 10; index <= 15; ++index) {
    ext_irq_handler(index);
  }
}
