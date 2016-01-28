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
    _PIN* PWM_I1 = &D[8];
    _PIN* PWM_I2 = &D[7];
    float pwm_freq = 500; // ~245Hz is minimum
    uint16_t pwm_duty = 32768 + 16384;
    init_clock();
    init_ui();
    init_pin();
    init_oc();
    init_spi();
    uint8_t spi_res = 0;
    // _PIN doug = D[4];
    // spi1;
    // spi_open(&spi1, &D[1], &D[0], &D[2], spi_freq); //24000 is chosen fairly arbitrarily. Revise later
    // spi_transfer(&spi1, spi_trans); // Does angular encoder datasheet suggest 16-bit data returns? p.15

    
    oc_pwm(&oc1, PWM_I1, NULL, pwm_freq, pwm_duty);
    int doug = 0;
    // while (1){
    //     doug = 2+2;
    // }

}