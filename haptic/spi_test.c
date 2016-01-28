#include <p24FJ128GB206.h>
#include "config.h"
#include "common.h"
#include "ui.h"
#include "pin.h"
#include "spi.h"
#include "oc.h"

int main(void) {
    int test = 0;
    test = 2 + 4; // Tests to see whether scons builds were working.
    float spi_freq = 24000;
    uint8_t spi_trans = 255;
    // Pins
    _PIN* SPI_SCK = &D[2];
    _PIN* SPI_MOSI = &D[0];
    _PIN* SPI_MISO = &D[1];
    _PIN* SPI_CS = &D[3];    

    _PIN* PWM_I1 = &D[8];
    _PIN* PWM_I2 = &D[7];

    float pwm_freq = 500; // ~245Hz is minimum
    uint16_t pwm_duty = 32768;
    init_clock();
    init_ui();
    init_timer();
    init_pin();
    init_oc();
    init_spi();
    uint8_t spi_res = 0;

    timer_setPeriod(&timer1, 1);
    timer_setPeriod(&timer2, 0.30);
    timer_start(&timer1);
    timer_start(&timer2);
    // _PIN doug = D[4];
    // spi1;
    // spi_open(&spi1, &D[1], &D[0], &D[2], spi_freq); //24000 is chosen fairly arbitrarily. Revise later
    // spi_transfer(&spi1, spi_trans); // Does angular encoder datasheet suggest 16-bit data returns? p.15
   
    oc_pwm(&oc1, PWM_I1, NULL, pwm_freq, pwm_duty);
    while (1){
        if (timer_flag(&timer2)) {
            timer_lower(&timer2);
            led_toggle(&led2);
        }

        if (timer_flag(&timer1)) {
            timer_lower(&timer1);
            led_toggle(&led1);
            pwm_duty = pwm_duty/2;
            pin_write(PWM_I1, pwm_duty);
        }

        // Main event loop
    }
}