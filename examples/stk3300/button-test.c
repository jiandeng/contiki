/* This is a very simple button_test program.
 * It aims to test the button-sensor
 *
 * Push PB0 to turn off the yellow led, push PB1 to turn on the yellow led
 *
 * Author: jiandeng <jiandeng.develop@gmail.com>
 */

#include "contiki.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"

/*---------------------------------------------------------------------------*/
PROCESS(button_test_process, "Button test process");
AUTOSTART_PROCESSES(&button_test_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(button_test_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {
    PROCESS_YIELD();

    if(ev == sensors_event) {
      if(data == &button_down_sensor) {
        leds_on(LEDS_YELLOW);
      } else if(data == &button_up_sensor) {
        leds_off(LEDS_YELLOW);
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
