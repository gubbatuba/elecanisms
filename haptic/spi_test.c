#include <p24FJ128GB206.h>
#include <stdio.h>
#include "config.h"
#include "common.h"
#include "ui.h"
#include "pin.h"
#include "spi.h"
#include "oc.h"
#include "spi_test.h"

void spi_parse_data(uint16_t data, unsigned char *angle_array) {
    unsigned char data_size = 16;  // Size of input data (in bits)
    unsigned char this_bit;
    unsigned char par;
    unsigned char ef;
    unsigned char par_count = 0;
    int i = 0;
    for (i; i < data_size; ++i) {
        this_bit = data & (1 << i) ? 1:0;
        if (i == 0) {
            // Even parity bit
            par = this_bit;
        } else if (i == 1) {
            // Error flag
            ef = this_bit;
        } else if (i > 1 && i > data_size) {
            // Transmitted data [14 bits]
            angle_array[i-2] = this_bit;
            if (this_bit == 1)
                par_count++;
        }
    }
    // Evaluate transmission
    if (par != par_count) {
        printf("%s\n", "ERR: Angle measurement parity error.");
    }
    if (ef == 1) {
        printf("%s\n", "ERR: Error flag thrown by AS5048 angle sensor.");
    }
}

void spi_send_READ(_SPI *self) {
    spi_transfer(self, spi_READ1);
    spi_transfer(self, spi_READ2);
}

uint16_t spi_send_NOP_read_data(_SPI *self) {
    uint8_t msb = spi_transfer(self, spi_NOP12);
    uint8_t lsb = spi_transfer(self, spi_NOP12);
    return (uint16_t)((msb << 8) | lsb);
}

void read_angle_sensor(unsigned char *angle_array) {
    pin_clear(SPI_CS);  // Assert CS LOW
    spi_send_READ(spi_inst);
    pin_set(SPI_CS);  // Assert CS HIGH
    pin_clear(SPI_CS);  // Reassert CS LOW
    // Retrieve raw angle data
    uint16_t raw_angle = spi_send_NOP_read_data(spi_inst);
    spi_parse_data(raw_angle, angle_array);
}

uint16_t pwm_duty_pct_to_int(float *percent) {
    return (uint16_t)(*percent * DUTY_MAX);
}

float pwm_duty_int_to_pct(uint16_t *frac) {
    return ((float)(*frac)/DUTY_MAX);
}

void pwm_set_duty(_PIN *pwm_pin, float percent) {
    uint8_t duty_frac = pwm_duty_pct_to_int(&percent);
    pin_write(pwm_pin, duty_frac);
}

void setup(void) {
    // Initialize PIC24 modules.
    init_clock();
    init_ui();
    init_timer();
    init_pin();
    init_oc();
    init_spi();

    // Configure single SPI comms. system
    pin_digitalOut(SPI_CS);
    spi_open(spi_inst, SPI_MISO, SPI_MOSI, SPI_SCK, spi_freq);

    // Configure & start timers used.
    timer_setPeriod(&timer1, 1);
    timer_setPeriod(&timer2, 0.30);  // Timer for LED operation/status blink
    timer_start(&timer1);
    timer_start(&timer2);

    // Configure single PWM signal
    oc_pwm(&oc1, PWM_I1, NULL, pwm_freq, pwm_duty);
}

int main(void) {
    setup();
    unsigned char angle_array[14];  // LSB will be angle_array[0]

    float pwm_duty_array[4] = {0.90, 0.40, 0, 0.50};
    uint8_t pwm_duty_index = 0;
    while (1) {
        if (timer_flag(&timer2)) {
            // Blink green light to show normal operation.
            timer_lower(&timer2);
            led_toggle(&led2);
        }
        if (timer_flag(&timer1)) {
            // PWM Test area. Change motor speed every second, looping
            // through an array of possible speeds (array length = 4)
            timer_lower(&timer1);
            pwm_set_duty(PWM_I1, pwm_duty_array[pwm_duty_index]);
            pwm_duty_index = (pwm_duty_index + 1) % 4;
        }
        read_angle_sensor(angle_array);  // Updates angle_array
    }
}
