#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "ui.h"
#include "timer.h"

// FOR MINIPROJECT 1, PLEASE SEE CONTROL.C.
// THIS IS UNUSED IN MINIPROJECT 1.

int16_t main(void) {
    init_clock();
    init_ui();
    init_timer();

    led_on(&led1); // Lights will blink police-style
                   // (alternating red and blue)
    timer_setPeriod(&timer2, 0.02);
    timer_start(&timer2);

    int counter = 0; // Counter for timing use
    int current_period_index = 0; // Selects lights' current period
    int periods[] = {1, 3, 5, 7, 10, 12, 15, 12, 10, 7, 5, 3}; // Sequence of periods
    int num_periods = 12; // The number of periods to loop through

    while (1) {
        if (timer_flag(&timer2)) {
            timer_lower(&timer2);
            counter++;
            if (counter == periods[current_period_index]) {
                led_toggle(&led1);
                led_toggle(&led3);
                counter = 0; // Reset counter
                current_period_index = ((current_period_index + 1) % num_periods); 
                // Select the next period. Iterate through the array 'ring-style'.
            }
        }
    }
}
