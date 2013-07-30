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
 *		Clock driver implementation for EFM32
 * \author
 *		Jiandeng <jiandeng.develop@gmail.com>
 */

#include "contiki.h"
#include "sys/clock.h"
#include "efm32.h"

/*--------------------------------------------------------------------------*/
static volatile clock_time_t count;
static volatile unsigned long current_seconds = 0;
static unsigned int second_countdown = CLOCK_SECOND;
/*---------------------------------------------------------------------------*/
void
LETIMER0_IRQHandler(void)
{
    LETIMER0->IFC = LETIMER_IEN_UF;

    count++;
    if(etimer_pending()) {
        etimer_request_poll();
    }

    if(--second_countdown == 0) {
        current_seconds++;
        second_countdown = CLOCK_SECOND;
    }
}
/*---------------------------------------------------------------------------*/
void
clock_init(void)
{
    /* Counts the number of ticks. */
    count = 0;
    /* Enable and select the LFXO */
    CMU->OSCENCMD = CMU_OSCENCMD_LFXOEN;
    while(!(CMU->STATUS & CMU_STATUS_LFXORDY));
    /* Enable clocks */
    CMU->HFPERCLKEN0 = CMU_HFPERCLKEN0_GPIO;
    CMU->HFCORECLKEN0 = CMU_HFCORECLKEN0_LE;
    CMU->LFCLKSEL = CMU_LFCLKSEL_LFA_LFXO;
    CMU->LFACLKEN0 = CMU_LFACLKEN0_LETIMER0;

    /* Config and start the LETIMER */
    LETIMER0->CMD = LETIMER_CMD_STOP | LETIMER_CMD_CLEAR;
    LETIMER0->CTRL = LETIMER_CTRL_REPMODE_FREE | LETIMER_CTRL_COMP0TOP;
    LETIMER0->COMP0 = 32768 / CLOCK_SECOND;
    LETIMER0->IFC = LETIMER_IEN_UF;
    LETIMER0->IEN = LETIMER_IEN_UF;
    LETIMER0->CMD = LETIMER_CMD_START;

    /* Enable LETIMER interrupt */
    NVIC_ClearPendingIRQ(LETIMER0_IRQn);
    NVIC_EnableIRQ(LETIMER0_IRQn);
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  return count;
}
/*---------------------------------------------------------------------------*/
/**
 * Delay the CPU for a multiple of TODO
 */
void
clock_delay(unsigned int i)
{
  for(; i > 0; i--) {           /* Needs fixing XXX */
    unsigned j;
    for(j = 50; j > 0; j--) {
      asm("nop");
    }
  }
}
/*---------------------------------------------------------------------------*/
/**
 * Wait for a multiple of 1 ms.
 */
void
clock_wait(clock_time_t i)
{
  clock_time_t start;

  start = clock_time();
  while(clock_time() - start < (clock_time_t) i);
}
/*---------------------------------------------------------------------------*/
unsigned long
clock_seconds(void)
{
  return current_seconds;
}
/*--------------------------------------------------------------------------*/
/** @} */
