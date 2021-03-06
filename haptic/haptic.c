#include <p24FJ128GB206.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "common.h"
#include "ui.h"
#include "usb.h"
#include "pin.h"
#include "spi.h"
#include "oc.h"
#include "uart.h"
#include "haptic.h"

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

uint16_t pwm_duty_pct_to_int(float *percent) {
    return (uint16_t)(*percent * DUTY_MAX);
}

float pwm_duty_int_to_pct(uint16_t *frac) {
    return ((float)(*frac)/DUTY_MAX);
}

void pwm_set_duty(float percent) {
    uint16_t duty_frac = pwm_duty_pct_to_int(&percent);
    // printf("Computed duty frac %d from pct %f.\r\n", duty_frac, percent);
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
            // printf("Setting motor direction FORWARD...\r\n");
            prev_duty = pin_read(PWM_I2);
            pin_write(PWM_I2, (uint16_t)(0));
            pin_write(PWM_I1, prev_duty);
        } else if (direction == 0) {
            // If 0, PWM_I1 should 0, PWM_I2 should be 1.
            // printf("Setting motor direction REVERSE...\r\n");
            prev_duty = pin_read(PWM_I1);
            pin_write(PWM_I1, (uint16_t)(0));
            pin_write(PWM_I2, prev_duty);
        } else {
            printf("ERR: Invalid PWM direction %d received.\r\n", direction);
        }
    }
}

uint16_t spi_read_ticks() {
    uint8_t data[2];
    WORD raw_data = enc_readReg((WORD)(0x3FFF));
    // printf("%d\r\n", raw_data);
    // printf("UNMASKED DATA: %X.%X \r\n", raw_data.b[1], raw_data.b[0]);
    data[1] = raw_data.b[1] & (0x3F);
    data[0] = raw_data.b[0];
    // printf("MASKED DATA: %X.%X \r\n", data[1], data[0]);
    uint16_t full_data = ((uint16_t)data[1] << 8) | data[0];
    // printf("ENCODER TICKS: %d\r\n", full_data);
    return full_data;
}

//Find the number of ticks moved
double encoder_counter(uint16_t current_ticks, uint16_t previous_ticks, double previous_count) {
    // pwm_direction = 1, we should see increase in ticks. Current - Previous should be 
    int difference = (int)(current_ticks) - (int)(previous_ticks);
    // printf("diff: %d\r\n", difference);
    // printf("DIFF: %d\n", difference);
    // if (difference >= 10) {
    //     if (pwm_direction == 0) {
    //         difference = -16384 + difference;
    //     }
    // } else if (difference <= -10) {
    //     if (pwm_direction == 1) {
    //         difference = 16384 + difference;
    //     }
    // }
    //Find the number of ticks moved
    if (difference > 8192) {
        difference = 16384 - difference;
    }
    if (difference < -8192) {
        difference = -16384 - difference;
    }
    // printf("diff; %d\r\n", difference);
    double new_count = previous_count + (double)(difference);
    // printf("diff; %f\r\n", (double)(difference));
    return new_count;
}

//Change master count to degs
double count_to_deg(double new_count) {
    double degs = new_count/714.15;
    return degs; 
}

void motor_control(double degs, double target_degs) {
    double diff = degs - target_degs;
    float new_duty;
    double threshold = 1;
    unsigned char direction;

    if (diff > threshold) {
        direction = 1;
        new_duty = 0.85;
    } else if (diff < -threshold) {
        direction = 0;
        new_duty = 0.85;
    } else {
        direction = pwm_direction;
        new_duty = 0.0;
    }
    pwm_set_direction(direction);
    pwm_set_duty(new_duty);
}

float PID_control(PID *self) {
    float error = self->set_point - self->position;
    float deriv = (self->position - self->prev_position);
    self->integ_state += error;
    if (self->integ_state > self->integ_max) {
        self->integ_state = self->integ_max;
    } else if (self->integ_state < self->integ_min) {
        self->integ_state = self->integ_min;
    };
    float pterm = self->Kp * error;
    float iterm = self->Ki * self->integ_state;
    float dterm = self->Kd * deriv;
    self->prev_position = self->position;

    return pterm + iterm + dterm;
}

float pid_to_pwm(float pid_command, float pct_duty) {
    float new_duty = pct_duty + pid_command;
    bool in_boundary = (new_duty > -PWM_MIN) && (new_duty < PWM_MIN);
    if ((new_duty < pwm_duty) && in_boundary) {
        // Coerce to -PWM_MIN
        new_duty = -PWM_MIN;
    } else if ((new_duty > pwm_duty) && in_boundary) {
        // Coerce to PWM_MIN
        new_duty = PWM_MIN;
    } else if (new_duty > PWM_MAX) {
        new_duty = PWM_MAX;
    } else if (new_duty < -PWM_MAX) {
        new_duty = -PWM_MAX;
    }
    // printf("NEWDUTY: %f\r\n", new_duty);
    if (new_duty > 0) {
        pwm_set_direction(0);
    } else {
        pwm_set_direction(1);
    }
    // printf("%f\r\n",new_duty );
    float set_duty = fabsf(new_duty);
    // printf("%f\r\n", set_duty );
    pwm_set_duty(set_duty);
    return set_duty;
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
    timer_setPeriod(&timer2, .4);  // Timer for LED operation/status blink
    timer_setPeriod(&timer3, LOOP_TIME); //super fast timer!
    timer_start(&timer1);
    timer_start(&timer2);
    timer_start(&timer3);

    pos_control.Kp = KP;
    pos_control.Kd = KD;
    pos_control.Ki = KI;
    pos_control.dt = LOOP_TIME;
    pos_control.integ_min = -100;
    pos_control.integ_max = 100;
    pos_control.integ_state = 0;
    pos_control.prev_position = 0;

    // Configure dual PWM signals for bidirectional motor control
    oc_pwm(&oc1, PWM_I1, NULL, pwm_freq, pwm_duty);
    oc_pwm(&oc2, PWM_I2, NULL, pwm_freq, pwm_duty);
    // pin_analogIn(MOTOR_VOLTAGE);

    InitUSB();                              // initialize the USB registers and
                                            // serial interface engine
    while (USB_USWSTAT != CONFIG_STATE) {   // while periph. is not configured,
        ServiceUSB();                       // service USB requests
    }
}

int main(void) {
    setup();
    // unsigned char angle_array[14];  // LSB will be angle_array[0]

    // float pwm_duty_array[4] = {0.40, 0.41, 0.42, 0.43};
    // float pwm_duty_array[4] = {0.12, 0.12, 0, 0};

    // pwm_set_direction(!pwm_direction);
    // pwm_set_duty(0);
    float pos_array[5] = {-20, -10, 0, 10, 20};
    uint8_t pos_i = 0;
    printf("%s\n", "STARTING LOOP");
    double encoder_master_count = 0;
    double degs = 0;
    uint16_t current_ticks = 0;
    uint16_t previous_ticks = spi_read_ticks();
    // double target_degs = 10;
    pos_control.set_point = 20;
    float pid_command = 0;
    while (1) {
        if (timer_flag(&timer2)) {
            // Blink green light to show normal operation.
            timer_lower(&timer2);
            led_toggle(&led2);
            printf("%s\r\n", "BLINK LIGHT");
            printf("MASTER COUNT: %f\r\n", encoder_master_count);
            printf("MASTER DEGS: %f\r\n", degs);
            // printf("MOTOR VOLTS: %d\r\n", pin_read(MOTOR_VOLTAGE));
            printf("PID INFO: SP: %f, POS: %f, CMD: %f, duty: %f, dir %f.\r\n", pos_control.set_point, pos_control.position, pid_command, pwm_pct_duty, pwm_direction);
        }

        if (timer_flag(&timer1)) {
            timer_lower(&timer1);
            pos_control.set_point = pos_array[pos_i];
            pos_i = (pos_i + 1) % 5;
        }
        if (!sw_read(&sw2)) {
            // If switch 2 is pressed, the UART output terminal is cleared.
            printf("%s", clear);
        }
        current_ticks = spi_read_ticks();
        encoder_master_count = encoder_counter(current_ticks, previous_ticks, encoder_master_count);
        degs = count_to_deg(encoder_master_count);
        pos_control.position = degs;
        pid_command = PID_control(&pos_control);
        pwm_pct_duty = pid_to_pwm(pid_command, pwm_pct_duty);
        previous_ticks = current_ticks;
        ServiceUSB();
    }
}