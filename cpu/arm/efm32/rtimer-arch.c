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
 *		Real-timer specific implementation for EFM32
 * \author
 *		Jiandeng <jiandeng.develop@gmail.com>
 */

#include "sys/energest.h"
#include "sys/rtimer.h"
#include "contiki.h"
#include "efm32.h"
/*---------------------------------------------------------------------------*/
void
RTC_IRQHandler(void)
{
    RTC->IFC = RTC_IFC_COMP0;
    RTC->IEN = 0x0000;
    ENERGEST_ON(ENERGEST_TYPE_IRQ);
    rtimer_run_next();
    ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
    /* Enable clocks */
    CMU->LFAPRESC0 &= ~_CMU_LFAPRESC0_RTC_MASK;
    CMU->LFAPRESC0 |= CMU_LFAPRESC0_RTC_DIV4;
    CMU->LFACLKEN0 |= CMU_LFACLKEN0_RTC;

    /* Config and start the RTC */
    RTC->CTRL = 0x0000;
    RTC->IEN = 0x0000;
    RTC->CTRL = RTC_CTRL_EN;

    /* Enable LETIMER interrupt */
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_EnableIRQ(RTC_IRQn);
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_disable_irq(void)
{
    RTC->IEN = 0x0000;
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_enable_irq(void)
{
    RTC->IEN = RTC_IEN_COMP0;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
rtimer_arch_now(void)
{
  return RTC->CNT;
}

/*---------------------------------------------------------------------------*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
    RTC->COMP0 = t;
    RTC->IFC = RTC_IEN_COMP0;
    RTC->IEN = RTC_IEN_COMP0;
}
/*---------------------------------------------------------------------------*/
/** @} */
