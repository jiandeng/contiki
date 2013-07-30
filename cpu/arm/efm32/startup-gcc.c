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
 *		Startup file for EFM32
 * \author
 *		Jiandeng <jiandeng.develop@gmail.com>
 */

#include "inttypes.h"

extern int main(void);
typedef void (*ISR_func)(void);
  
#define SECTION(x) __attribute__ ((section(#x)))
#define ISR_VECTOR_SECTION SECTION(.isr_vector)

static void sys_reset(void) __attribute__((naked));
void NMI_handler(void) __attribute__((interrupt));
void HardFault_handler(void) __attribute__((interrupt));
void MemManage_handler(void) __attribute__((interrupt));
void BusFault_handler(void) __attribute__((interrupt));
void UsageFault_handler(void) __attribute__((interrupt));
static void unhandled_int(void) __attribute__((interrupt));

#define UNHANDLED_ALIAS __attribute__((weak, alias("unhandled_int")));
void Main_Stack_End(void);
void HardFault_handler(void)__attribute__((weak, alias("dHardFault_handler")));
void MemManage_handler(void)__attribute__((weak, alias("dMemManage_handler")));
void BusFault_handler(void) __attribute__((weak, alias("dBusFault_handler")));
void UsageFault_handler(void)__attribute__((weak, alias("dUsageFault_handler")));
 void Reserved_handler(void) UNHANDLED_ALIAS;
 void SVCall_handler(void) UNHANDLED_ALIAS;
 void DebugMonitor_handler(void) UNHANDLED_ALIAS;
 void PendSV_handler(void) UNHANDLED_ALIAS;
 void SysTick_handler(void) UNHANDLED_ALIAS;
 void DMA_IRQHandler(void) UNHANDLED_ALIAS;
 void GPIO_EVEN_IRQHandler(void) UNHANDLED_ALIAS;
 void TIMER0_IRQHandler(void) UNHANDLED_ALIAS;
 void USART0_RX_IRQHandler(void) UNHANDLED_ALIAS;
 void USART0_TX_IRQHandler(void) UNHANDLED_ALIAS;
 void ACMP0_IRQHandler(void) UNHANDLED_ALIAS;
 void ADC0_IRQHandler(void) UNHANDLED_ALIAS;
 void DAC0_IRQHandler(void) UNHANDLED_ALIAS;
 void I2C0_IRQHandler(void) UNHANDLED_ALIAS;
 void GPIO_ODD_IRQHandler(void) UNHANDLED_ALIAS;
 void TIMER1_IRQHandler(void) UNHANDLED_ALIAS;
 void USART1_RX_IRQHandler(void) UNHANDLED_ALIAS;
 void USART1_TX_IRQHandler(void) UNHANDLED_ALIAS;
 void LESENSE_IRQHandler(void) UNHANDLED_ALIAS;
 void LEUART0_IRQHandler(void) UNHANDLED_ALIAS;
 void LETIMER0_IRQHandler(void) UNHANDLED_ALIAS;
 void PCNT0_IRQHandler(void) UNHANDLED_ALIAS;
 void RTC_IRQHandler(void) UNHANDLED_ALIAS;
 void CMU_IRQHandler(void) UNHANDLED_ALIAS;
 void VCMP_IRQHandler(void) UNHANDLED_ALIAS;
 void LCD_IRQHandler(void) UNHANDLED_ALIAS;
 void MSC_IRQHandler(void) UNHANDLED_ALIAS;
 void AES_IRQHandler(void) UNHANDLED_ALIAS;

const ISR_func isr_vector[76] ISR_VECTOR_SECTION =
  {
    Main_Stack_End,
    sys_reset,
    NMI_handler,
    HardFault_handler,
    MemManage_handler,
    BusFault_handler,
    UsageFault_handler,
    Reserved_handler,
    Reserved_handler,
    Reserved_handler,
    Reserved_handler,
    SVCall_handler,
    DebugMonitor_handler,
    Reserved_handler,
    PendSV_handler,
    SysTick_handler,

    /* External Interrupts */
    DMA_IRQHandler,
    GPIO_EVEN_IRQHandler,
    TIMER0_IRQHandler,
    USART0_RX_IRQHandler,
    USART0_TX_IRQHandler,
    ACMP0_IRQHandler,
    ADC0_IRQHandler,
    DAC0_IRQHandler,
    I2C0_IRQHandler,
    GPIO_ODD_IRQHandler,
    TIMER1_IRQHandler,
    USART1_RX_IRQHandler,
    USART1_TX_IRQHandler,
    LESENSE_IRQHandler,
    LEUART0_IRQHandler,
    LETIMER0_IRQHandler,
    PCNT0_IRQHandler,
    RTC_IRQHandler,
    CMU_IRQHandler,
    VCMP_IRQHandler,
    LCD_IRQHandler,
    MSC_IRQHandler,
    AES_IRQHandler
  };

extern uint8_t _data[];
extern uint8_t _etext[];
extern uint8_t _edata[];
static void copy_initialized(void)
{
  uint8_t *ram = _data;
  uint8_t *rom = _etext;
  while(ram < _edata) {
    *ram++ = *rom++;
  }
}

extern uint8_t __bss_start[];
extern uint8_t __bss_end[];
static void clear_bss(void)
{
  uint8_t *m = __bss_start;
  while(m < __bss_end) {
    *m++ = 0;
  }
}

static void sys_reset(void)
{
  copy_initialized();
  clear_bss();
  main();
  while(1);
}

void NMI_handler(void)
{
  while(1);
}

static void dHardFault_handler(void)
{
  while(1);
}

static void dUsageFault_handler(void)
{
  while(1);
}

static void dMemManage_handler(void)
{
  while(1);
}

static void dBusFault_handler(void)
{
  while(1);
}

static void unhandled_int(void)
{
  while(1);
}
