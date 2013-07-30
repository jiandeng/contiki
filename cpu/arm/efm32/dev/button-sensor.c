/*
 * Copyright (c) 2013, CITTI - http://www.citti.com.cn/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup stk3300-platform
 * @{
 *
 * \file
 *		Button driver for EFM32
 * \author
 *		Jiandeng <jiandeng.develop@gmail.com>
 */

#include "dev/button-sensor.h"
#include "lib/sensors.h"
#include "sys/energest.h"
#include "sys/timer.h"
#include "contiki.h"
#include "efm32.h"

static struct timer debouncetimer;
/*---------------------------------------------------------------------------*/
/**
 * \brief Common initialiser for all buttons
 * \param port GPIO port number corresponding to the button's pin
 * \param pin GPIO Pin number corresponding to the button's pin
 */
static void
init_pin(uint32_t port, uint32_t pin)
{
    GPIO->P[port].DOUTSET = (1UL << pin); // Pull up resister
    GPIO->EXTIRISE &= (1UL << pin); // Disable rising edge
    GPIO->EXTIFALL |= (1UL << pin); // Enable fall edge
    if(pin >= 8) {
        GPIO->P[port].MODEH &= ~(_GPIO_P_MODEH_MODE8_MASK << ((pin- 8)  << 2));
        GPIO->P[port].MODEH |= _GPIO_P_MODEH_MODE8_INPUTPULL << ((pin - 8) << 2); // Input with pull
        GPIO->EXTIPSELH &= ~(_GPIO_EXTIPSELH_EXTIPSEL8_MASK << ((pin- 8)  << 2));
        GPIO->EXTIPSELH |= port << ((pin - 8) << 2); // Selected as interrupt port
    }
    else {
        GPIO->P[port].MODEL &= ~(_GPIO_P_MODEL_MODE0_MASK << (pin << 2));
        GPIO->P[port].MODEL |= _GPIO_P_MODEL_MODE0_INPUTPULL << (pin << 2);
        GPIO->EXTIPSELL &= ~(_GPIO_EXTIPSELL_EXTIPSEL0_MASK << (pin << 2));
        GPIO->EXTIPSELL |= port << (pin << 2);
    }

    GPIO->IFC |= (1UL << pin); // Clear interrupt flag
    GPIO->IEN |= (1UL << pin); // Enable interrupt
}
/*---------------------------------------------------------------------------*/
/**
 * \brief GPIO interrupt hanlder
 */
void
GPIO_ODD_IRQHandler(void)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  if(!timer_expired(&debouncetimer)) {
    return;
  }
  timer_set(&debouncetimer, CLOCK_SECOND / 8);

  if(GPIO->IF & (1UL << BUTTON_CONF_UP_PIN)) {
    GPIO->IFC = (1UL << BUTTON_CONF_UP_PIN);
    sensors_changed(&button_up_sensor);
  } 
  if(GPIO->IF & (1UL << BUTTON_CONF_DOWN_PIN)) {
    GPIO->IFC = (1UL << BUTTON_CONF_DOWN_PIN);
    sensors_changed(&button_down_sensor);
  } 
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief GPIO interrupt hanlder
 */
void
GPIO_EVEN_IRQHandler(void)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  if(!timer_expired(&debouncetimer)) {
    return;
  }
  timer_set(&debouncetimer, CLOCK_SECOND / 8);

  if(GPIO->IF & (1UL << BUTTON_CONF_UP_PIN)) 
  {
    GPIO->IFC = (1UL << BUTTON_CONF_UP_PIN);
    sensors_changed(&button_up_sensor);
  } 
  if(GPIO->IF & (2UL << BUTTON_CONF_DOWN_PIN)) {
    GPIO->IFC = (1UL << BUTTON_CONF_DOWN_PIN);
    sensors_changed(&button_down_sensor);
  } 
  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Init function for the select button.
 *
 * Parameters are ignored. They have been included because the prototype is
 * dictated by the core sensor api. The return value is also not required by
 * the API but otherwise ignored.
 *
 * \param type ignored
 * \param value ignored
 * \return ignored
 */
static int
config_up(int type, int value)
{
  init_pin(BUTTON_CONF_UP_PORT, BUTTON_CONF_UP_PIN);

  return 1;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Init function for the change button.
 *
 * Parameters are ignored. They have been included because the prototype is
 * dictated by the core sensor api. The return value is also not required by
 * the API but otherwise ignored.
 *
 * \param type ignored
 * \param value ignored
 * \return ignored
 */
static int
config_down(int type, int value)
{
  init_pin(BUTTON_CONF_DOWN_PORT, BUTTON_CONF_DOWN_PIN);

  return 1;
}
/*---------------------------------------------------------------------------*/
void
button_sensor_init()
{
  /* Enable gpio interrupt */
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);

  /* Init debounce timer */
  timer_set(&debouncetimer, 0);
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(button_up_sensor, BUTTON_SENSOR, NULL, config_up, NULL);
SENSORS_SENSOR(button_down_sensor, BUTTON_SENSOR, NULL, config_down, NULL);

/** @} */
