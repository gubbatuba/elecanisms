#include <p24FJ128GB206.h>
#include <stdio.h>
#include <stdint.h>
#include "config.h"
#include "common.h"
#include "ui.h"
#include "usb.h"
#include "pin.h"
#include "spi.h"
#include "oc.h"
#include "uart.h"
#include "control.h"


WORD enc_readReg(WORD address) {
    WORD cmd, result;
    cmd.w = 0x4000|address.w;      // set 2nd MSB to 1 for a read
    cmd.w |= parity(cmd.w) << 15;  // calculate even parity for

    pin_clear(SPI_CS);
    spi_transfer(&spi1, cmd.b[1]);
    spi_transfer(&spi1, cmd.b[0]);
    pin_set(SPI_CS);

    pin_clear(SPI_CS);
    result.b[1] = spi_transfer(&spi1, 0);
    result.b[0] = spi_transfer(&spi1, 0);
    pin_set(SPI_CS);
    return result;
}

void VendorRequests(void) {
    WORD32 address;
    WORD result;

    switch (USB_setup.bRequest) {
        case TOGGLE_LED1:
            led_toggle(&led1);
            BD[EP0IN].bytecount = 0;  // set EP0 IN byte count to 0
            BD[EP0IN].status = 0xC8;  // send packet as DATA1, set UOWN bit
            break;
        case TOGGLE_LED2:
            led_toggle(&led2);
            BD[EP0IN].bytecount = 0;  // set EP0 IN byte count to 0
            BD[EP0IN].status = 0xC8;  // send packet as DATA1, set UOWN bit
            break;
        case READ_SW1:
            BD[EP0IN].address[0] = (uint8_t)sw_read(&sw1);
            BD[EP0IN].bytecount = 1;  // set EP0 IN byte count to 1
            BD[EP0IN].status = 0xC8;  // send packet as DATA1, set UOWN bit
            break;
        case ENC_READ_REG:
            result = enc_readReg(USB_setup.wValue);
            BD[EP0IN].address[0] = result.b[0];
            BD[EP0IN].address[1] = result.b[1];
            BD[EP0IN].bytecount = 2;  // set EP0 IN byte count to 1
            BD[EP0IN].status = 0xC8;  // send packet as DATA1, set UOWN bit
            break;
        case TOGGLE_LED3:
            led_toggle(&led3);
            BD[EP0IN].bytecount = 0;  // set EP0 IN byte count to 0
            BD[EP0IN].status = 0xC8;  // send packet as DATA1, set UOWN bit
            break;
        case READ_SW2:
            BD[EP0IN].address[0] = (uint8_t)sw_read(&sw2);
            BD[EP0IN].bytecount = 1;  // set EP0 IN byte count to 1
            BD[EP0IN].status = 0xC8;  // send packet as DATA1, set UOWN bit
            break;
        case READ_SW3:
            BD[EP0IN].address[0] = (uint8_t)sw_read(&sw3);
            BD[EP0IN].bytecount = 1;  // set EP0 IN byte count to 1
            BD[EP0IN].status = 0xC8;  // send packet as DATA1, set UOWN bit
            break;
        default:
            USB_error_flags |= 0x01;  // set Request Error Flag
    }
}

void VendorRequestsIn(void) {
    switch (USB_request.setup.bRequest) {
        default:
            USB_error_flags |= 0x01;  // set Request Error Flag
    }
}

void VendorRequestsOut(void) {
//    WORD32 address;
//
//    switch (USB_request.setup.bRequest) {
//        case ENC_WRITE_REGS:
//            enc_writeRegs(USB_request.setup.wValue.b[0], BD[EP0OUT].address, USB_request.setup.wLength.b[0]);
//            break;
//        default:
//            USB_error_flags |= 0x01;                    // set Request Error Flag
//    }
}

uint16_t pwm_duty_pct_to_int(float *percent) {
    return (uint16_t)(*percent * DUTY_MAX);
}

float pwm_duty_int_to_pct(uint16_t *frac) {
    return ((float)(*frac)/DUTY_MAX);
}

void pwm_set_duty(float percent) {
    uint16_t duty_frac = pwm_duty_pct_to_int(&percent);
    printf("Computed duty frac %d from pct %f.\r\n", duty_frac, percent);
    if (pwm_direction == 1) {
        pin_write(PWM_I1, duty_frac);
    } else {
        pin_write(PWM_I2, duty_frac);
    }
}

void pwm_set_direction(unsigned char direction) {
    // Direction is a bit [0 or 1] specifying the direction
    // the motor should be commanded to turn. 1 is "forwards",
    // and 0 is "reverse". Assumes fast decay mode operation.
    if (pwm_direction != direction) {
        // The direction to be set is different than the motor's current
        // direction. A change should be made.
        uint16_t prev_duty;
        pwm_direction = direction;  // Update pwm_direction
        if (direction == 1) {
            // If 1, PWM_I1 should PWM, PWM_I2 should be 0.
            printf("Setting motor direction FORWARD...\r\n");
            prev_duty = pin_read(PWM_I2);
            pin_write(PWM_I2, (uint16_t)(0));
            pin_write(PWM_I1, prev_duty);
        } else if (direction == 0) {
            // If 0, PWM_I1 should 0, PWM_I2 should be 1.
            printf("Setting motor direction REVERSE...\r\n");
            prev_duty = pin_read(PWM_I1);
            pin_write(PWM_I1, (uint16_t)(0));
            pin_write(PWM_I2, prev_duty);
        } else {
            printf("ERR: Invalid PWM direction %d received.\r\n", direction);
        }
    }
}

void setup(void) {
    // Initialize PIC24 modules.
    init_clock();
    init_ui();
    init_timer();
    init_pin();
    init_oc();
    init_spi();
    init_uart();

    // Configure single SPI comms. system
    pin_digitalOut(SPI_CS);
    pin_set(SPI_CS);
    spi_open(spi_inst, SPI_MISO, SPI_MOSI, SPI_SCK, spi_freq, spi_mode);

    // Configure & start timers used.
    timer_setPeriod(&timer1, 1);
    timer_setPeriod(&timer2, 0.75);  // Timer for LED operation/status blink
    timer_start(&timer1);
    timer_start(&timer2);


    // Configure dual PWM signals for bidirectional motor control
    oc_pwm(&oc1, PWM_I1, NULL, pwm_freq, pwm_duty);
    oc_pwm(&oc2, PWM_I2, NULL, pwm_freq, pwm_duty);

    InitUSB();                              // initialize the USB registers and
                                            // serial interface engine
    while (USB_USWSTAT != CONFIG_STATE) {   // while periph. is not configured,
        ServiceUSB();                       // service USB requests
    }
}

int main(void) {
    setup();
    unsigned char angle_array[14];  // LSB will be angle_array[0]

    // float pwm_duty_array[4] = {0.40, 0.41, 0.42, 0.43};
    // float pwm_duty_array[4] = {0.12, 0.12, 0, 0};

    uint8_t pwm_duty_index = 0;
    pwm_set_direction(!pwm_direction);
    pwm_set_duty(.95);
    while (1) {
        if (timer_flag(&timer2)) {
            // Blink green light to show normal operation.
            timer_lower(&timer2);
            led_toggle(&led2);
        }
        // if (timer_flag(&timer1)) {
        //     // PWM Test area. Change motor speed every second, looping
        //     // through an array of possible speeds (array length = 4)
        //     timer_lower(&timer1);
        //     led_toggle(&led3);
        //     printf("Set PWM duty to %f.\r\n", pwm_duty_array[pwm_duty_index]);
        //     pwm_set_duty(pwm_duty_array[pwm_duty_index]);
        //     // if (pwm_duty_index == 2) {
        //     //     pwm_set_direction(!pwm_direction);
        //     // }
        //     pwm_duty_index = (pwm_duty_index + 1) % 4;
        // }
        if (!sw_read(&sw2)) {
            // If switch 2 is pressed, the UART output terminal is cleared.
            printf("%s", clear);
        }
        ServiceUSB();
    }
}
