#include <p24FJ128GB206.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "config.h"
#include "common.h"
#include "ui.h"
#include "usb.h"
#include "pin.h"
#include "spi.h"
#include "oc.h"
#include "uart.h"
#include "haptic_spring.h"

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
float encoder_counter(uint16_t current_ticks, uint16_t previous_ticks, float previous_count) {
    // pwm_direction = 1, we should see increase in ticks. Current - Previous should be 
    int difference = (int)(current_ticks) - (int)(previous_ticks);
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
    float new_count = previous_count + (float)(difference);
    return new_count;
}

uint16_t pwm_duty_pct_to_int(float *percent) {
    return (uint16_t)(*percent * DUTY_MAX);
}

float pwm_duty_int_to_pct(uint16_t *frac) {
    return ((float)(*frac)/DUTY_MAX);
}

void pwm_set_raw_duty(uint16_t raw_duty) {
    if (pwm_direction == 1) {
        pin_write(PWM_I1, raw_duty);
    } else {
        pin_write(PWM_I2, raw_duty);
    }
}
void pwm_set_duty(float percent) {
    uint16_t duty_frac = pwm_duty_pct_to_int(&percent);
    // printf("Computed duty frac %d from pct %f.\r\n", duty_frac, percent);
    pwm_set_raw_duty(duty_frac);
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

void read_motor_current(MOTOR *mot) {
    // uint16_t raw_volts = pin_read(MOTOR_VOLTAGE) >> 6; // Shift to get only the 10 bits from the ADC
    // printf("%d\r\n", raw_volts);
    mot->volts = ((float)(mot->raw_volts)/MAX_ADC_OUTPUT * MAX_ANALOG_VOLTAGE) - 1.5; // Should match scope
    // printf("Raw: %d, Comp: %f\r\n", raw_volts, motor_volts);
    mot->current = (mot->volts * 0.1)/MOTOR_VOLTAGE_RESISTOR;
}

float PID_control(PID *self) {
    pin_set(DEBUGD0);
    float error = self->set_point - self->position;
    float deriv = (self->position - self->prev_position)/self->dt;
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
    // printf("PID. SP: %2f, POS: %2f, CMD: %2f\r\n", self->set_point, self->position, pterm + iterm + dterm);
    pin_clear(DEBUGD0);
    return pterm + iterm + dterm;
}

float pid_to_pwm(float pid_command, bool set_point_sign) {
    // if (set_point > 0) {        // If we want "positive" torque
    //     pwm_set_direction(1);
    // } else {
    //     pwm_set_direction(0);
    // }
    float new_pwm = pwm_duty + pid_command;
    if (new_pwm > MAX_DUTY) {
        new_pwm = MAX_DUTY;
    } else if (new_pwm < 0) {
        new_pwm = 0;
    }
    if (set_point_sign) {        // If we want "positive" torque
        pwm_set_direction(0);
    } else {
        pwm_set_direction(1);
    }
    pwm_set_duty(new_pwm);
    
    return new_pwm;
}

float spring_model(float theta) {
    // Uses Hooke's law for torsional spring to model expected torque for 
    // our virtual spring.
    return -SPRING_CONSTANT * theta;
}

bool read_sign(float theor_torque) {
    if (theor_torque > 0) return 1;
    else return 0;
}
float convert_motor_torque(float current) {
    // Converts motor current to effective torque
    return fabsf(current * MOTOR_TORQUE_COEF);
}

//Change master count to degs
float count_to_deg(float new_count) {
    float degs = new_count/714.15;
    // printf("%f\r\n", degs);
    return degs; 
}

void VendorRequests(void) {
    WORD32 address;
    WORD result;

    switch (USB_setup.bRequest) {
        WORD temp;
        case TOGGLE_LED1:
            led_toggle(&led1);
            BD[EP0IN].bytecount = 0;  // set EP0 IN byte count to 0
            BD[EP0IN].status = 0xC8;  // send packet as DATA1, set UOWN bit
            break;
        case ENC_READ_REG:
            result = enc_readReg(USB_setup.wValue);
            BD[EP0IN].address[0] = result.b[0];
            BD[EP0IN].address[1] = result.b[1];
            BD[EP0IN].bytecount = 2;  // set EP0 IN byte count to 1
            BD[EP0IN].status = 0xC8;  // send packet as DATA1, set UOWN bit
            break;
        case SET_KP:
            // val1 = USB_setup.wValue.w;
            // val2 = USB_setup.wIndex.w;
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case SET_KI:
            // val1 = USB_setup.wValue.w;
            // val2 = USB_setup.wIndex.w;
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case SET_KD:
            // val1 = USB_setup.wValue.w;
            // val2 = USB_setup.wIndex.w;
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case SET_DUTY:
            pwm_set_raw_duty(USB_setup.wValue.w);
            // val2 = USB_setup.wIndex.w;
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case GET_MOTOR_CURRENT:
            temp.w = motor.current;
            BD[EP0IN].address[0] = temp.b[0];
            BD[EP0IN].address[1] = temp.b[1];
            BD[EP0IN].bytecount = 2;    // set EP0 IN byte count to 4
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case GET_SETPOINT:
            temp.w = cur_control.set_point;
            BD[EP0IN].address[0] = temp.b[0];
            BD[EP0IN].address[1] = temp.b[1];
            BD[EP0IN].bytecount = 2;    // set EP0 IN byte count to 4
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;         
        default:
            USB_error_flags |= 0x01;  // set Request Error Flag
    }
}

void VendorRequestsIn(void) {
    switch (USB_request.setup.bRequest) {
        default:
            USB_error_flags |= 0x01;                    // set Request Error Flag
    }
}

void VendorRequestsOut(void) {
    switch (USB_request.setup.bRequest) {
        default:
            USB_error_flags |= 0x01;                    // set Request Error Flag
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
    timer_setPeriod(&timer2, 1);  // Timer for LED operation/status blink
    timer_setPeriod(&timer3, LOOP_TIME); // Timer for main control loop
    timer_start(&timer1);
    timer_start(&timer2);
    timer_start(&timer3);

    // Configure motor current conversion coefficient
    CURRENT_CONV_COEF = MAX_ADC_OUTPUT * MOTOR_VOLTAGE_RESISTOR;
    cur_control.Kp = KP;
    cur_control.Kd = KD;
    cur_control.Ki = KI;
    cur_control.dt = LOOP_TIME;
    cur_control.integ_min = -100;
    cur_control.integ_max = 100;
    cur_control.integ_state = 0;
    read_motor_current(&motor);
    cur_control.prev_position = convert_motor_torque(motor.current);
    // Configure dual PWM signals for bidirectional motor control
    oc_pwm(&oc1, PWM_I1, NULL, pwm_freq, pwm_duty);
    oc_pwm(&oc2, PWM_I2, NULL, pwm_freq, pwm_duty);
    pin_analogIn(MOTOR_VOLTAGE);
    pin_digitalOut(DEBUGD0);
    pin_digitalOut(DEBUGD1);

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
    pwm_set_duty(0);
    pwm_set_direction(0);
    
    printf("%s\r\n", "STARTING LOOP");
    float encoder_master_count = 0;
    float degs = 0;
    uint16_t current_ticks = 0;
    uint16_t previous_ticks = spi_read_ticks();
    float target_degs = 10;
    float motor_current = 0;  // current through motor in amperes
    float pid_command = 0;
    float theor_torque;
    bitset(&IEC0, 2);
    
    while (1) {
        if (timer_flag(&timer2)) {
            // Blink green light to show normal operation.
            timer_lower(&timer2);
            led_toggle(&led2);
            printf("DUTY: %3f, dir: %d\r\n", pwm_duty, pwm_direction);

            // printf("%s\r\n", "BLINK LIGHT");
            // printf("MASTER COUNT: %f\r\n", encoder_master_count);
            // printf("MASTER DEGS: %f\r\n", degs);
            // printf("MOTOR VOLTS: %d\r\n", pin_read(MOTOR_VOLTAGE) >> 6);
            // printf("PID INFO: SP: %2f, POS: %2f, CMD: %2f", cur_control.set_point, cur_control.position, pid_command);
            printf("I: %2f, T: %2f\r\n", motor.current, cur_control.position);
            // printf("Current: %f \r\n", motor.current);
        }

        if (timer_flag(&timer3)) {
            timer_lower(&timer3);
            led_toggle(&led3);
            // __builtin__disi(0x3FFF);
            theor_torque = spring_model(degs);  // Outputs theoretical torque predicted by spring model
            cur_control.set_point = fabsf(theor_torque);
            cur_control.neg_set_point = read_sign(theor_torque);
            read_motor_current(&motor);
            cur_control.position = convert_motor_torque(motor.current);
            // printf("CURTICKS: %d\r\n", current_ticks);
            // printf("MASTCOUNT: %d\r\n", encoder_master_count);
            // printf("DEGS: %f\r\n", degs);
            // printf("Th: %f, Ac: %f, I: %f\r\n", cur_control.set_point, cur_control.position, motor_current);

            pid_command = PID_control(&cur_control);
            
            pwm_duty = pid_to_pwm(pid_command, cur_control.neg_set_point);
            // printf("degs: %3f, theo_torque: %3f, read_torque: %3f, pid_cmd: %3f.\r\n", degs, cur_control.set_point, cur_control.position, pid_command);
            // __builtin__disi(0x0000);
        }

        if (!sw_read(&sw2)) {
            // If switch 2 is pressed, the UART output terminal is cleared.
            printf("%s", clear);
        }
        
        ServiceUSB();
        current_ticks = spi_read_ticks();
        encoder_master_count = encoder_counter(current_ticks, previous_ticks, encoder_master_count);
        degs = count_to_deg(encoder_master_count);
        previous_ticks = current_ticks;
        pin_toggle(DEBUGD1);  // Heartbeat signal
        
    }
}

void __attribute__((interrupt, auto_psv)) _OC1Interrupt(void) {
    bitclear(&IFS0, 2);
    motor.raw_volts = pin_read(MOTOR_VOLTAGE) >> 6; // Shift to get only the 10 bits from the ADC
}
