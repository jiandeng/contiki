/* This is a very simple led-test program.
 * It aims to test the yellow led driver
 *
 * The led toggles all the time
 *
 * Author: Jiandeng <jiandeng.develop@gmail.com>
 */

#include "contiki.h"
#include "dev/leds.h"

/*---------------------------------------------------------------------------*/
static struct etimer et_led;
/*---------------------------------------------------------------------------*/
PROCESS(led_test_process, "LED test process");
AUTOSTART_PROCESSES(&led_test_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(led_test_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {
    etimer_set(&et_led, CLOCK_SECOND / 2);

    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);

    leds_toggle(LEDS_YELLOW);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
