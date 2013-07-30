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
 * \defgroup stk3300-platform
 *
 * The EFM32TG STK3300 platform.
 *
 * @{
 *
 * \file
 *		platform-conf.h for STK3300.
 * \author
 *		Jiandeng <jiandeng.develop@gmail.com>
 */

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__
  
#include <inttypes.h>
#include <string.h>   /* For memcpm() */

/* Platform-dependent definitions */ 
#define CC_CONF_REGISTER_ARGS           0
#define CC_CONF_FUNCTION_POINTER_ARGS   1
#define CC_CONF_FASTCALL
#define CC_CONF_VA_ARGS                 1
#define CC_CONF_INLINE                  inline
#define EFM32TG840F32
  
#define CCIF
#define CLIF

typedef unsigned short uip_stats_t;
typedef unsigned long clock_time_t;
typedef unsigned long rtimer_clock_t;

#define CLOCK_CONF_SECOND               1024
#define RTIMER_CLOCK_LT(a,b)            ((signed short)((a)-(b)) < 0)
  
/* LEDs ports STK3300 */ 
#define LEDS_CONF_RED_PORT              0
#define LEDS_CONF_RED_PIN               0
#define LEDS_CONF_GREEN_PORT            0
#define LEDS_CONF_GREEN_PIN             0
#define LEDS_CONF_YELLOW_PORT           3
#define LEDS_CONF_YELLOW_PIN            7 

/* Buttons ports STK3300 */
#define BUTTON_CONF_UP_PORT             3
#define BUTTON_CONF_UP_PIN              8
#define BUTTON_CONF_DOWN_PORT           1
#define BUTTON_CONF_DOWN_PIN            11
 
#define UIP_ARCH_ADD32                  1
#define UIP_ARCH_CHKSUM                 0
#define UIP_CONF_BYTE_ORDER             UIP_LITTLE_ENDIAN
  
#endif  /* __PLATFORM_CONF_H__ */
/** @} */
