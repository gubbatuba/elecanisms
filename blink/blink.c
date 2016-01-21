#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "ui.h"
#include "timer.h"

int a[] = {10,100,1000,10000, 1000, 100};
// int i,*p;

int16_t main(void) {
    init_clock();
    init_ui();
    init_timer();

    led_on(&led1);
    //led_on(&led3);
    timer_setPeriod(&timer2, 0.05);
    timer_start(&timer2);
    int counter = 0;
    while (1) {
        if (timer_flag(&timer2)) {
            timer_lower(&timer2);
            led_toggle(&led1);
            led_toggle(&led3);
            // timer_setPeriod(&timer2, 0.5);
        }
        if (timer_flag(&timer2) && ) {
        	timer_lower(&timer2);

        }
        //led_write(&led2, !sw_read(&sw2));
        //led_write(&led3, !sw_read(&sw3));
    }
}