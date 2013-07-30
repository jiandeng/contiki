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
 * 		LED driver implementation for EFM32 STK3300
 * \author
 *		Jiandeng <jiandeng.develop@gmail.com>
 */

#include "dev/leds.h"
#include "contiki.h"
#include "efm32.h"

/*---------------------------------------------------------------------------*/
static void
init_pin(uint32_t port, uint32_t pin)
{
    GPIO->P[port].DOUTCLR = (1UL << pin);
    if(pin >= 8)
    {
        GPIO->P[port].MODEH &= ~(_GPIO_P_MODEH_MODE8_MASK << ((pin- 8)  << 2));
        GPIO->P[port].MODEH |= _GPIO_P_MODEH_MODE8_PUSHPULL << ((pin - 8) << 2);
    }
    else
    {
        GPIO->P[port].MODEL &= ~(_GPIO_P_MODEL_MODE0_MASK << (pin << 2));
        GPIO->P[port].MODEL |= _GPIO_P_MODEL_MODE0_PUSHPULL << (pin << 2);
    }
}

void
leds_arch_init(void)
{
    init_pin(LEDS_CONF_YELLOW_PORT, LEDS_CONF_YELLOW_PIN);
}
/*---------------------------------------------------------------------------*/
static uint32_t
get_pin(uint32_t port, uint32_t pin)
{
    return (GPIO->P[port].DOUT & (1UL << pin)); 
}

unsigned char
leds_arch_get(void)
{
    return get_pin(LEDS_CONF_YELLOW_PORT, LEDS_CONF_YELLOW_PIN) ? LEDS_YELLOW : 0; 
}
/*---------------------------------------------------------------------------*/
static void
set_pin(uint32_t port, uint32_t pin, uint32_t value)
{
    if(value)
    {
        GPIO->P[port].DOUTSET = (1UL << pin);
    }
    else
    {
        GPIO->P[port].DOUTCLR = (1UL << pin);
    }
}
void
leds_arch_set(unsigned char leds)
{
    set_pin(LEDS_CONF_YELLOW_PORT, LEDS_CONF_YELLOW_PIN, leds & LEDS_YELLOW);
}
/*---------------------------------------------------------------------------*/
/** @} */
