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
#include "haptic_texture.h"

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
double encoder_counter(uint16_t current_ticks, uint16_t previous_ticks, double previous_count) {
    int difference = (int)(current_ticks) - (int)(previous_ticks);
    if (difference > 8192) {
        difference = 16384 - difference;
    }
    if (difference < -8192) {
        difference = -16384 - difference;
    }
    if (current_ticks > previous_ticks){
        current_direction = 0;
    }
    if (current_ticks < previous_ticks){
        current_direction = 1;
    }
    double new_count = previous_count + (double)(difference);
    return new_count;

}

float texture(double degs){
    double dist = 4;
    double bump_dist = 2;
    float new_duty;
    unsigned char direction = pwm_direction;
    if ((degs > (light_stick_deg-dist)) && (degs < (light_stick_deg+dist))){
        direction = (direction+1)%2;
        new_duty = 0.65;
        // printf("light stick\r\n");  
    } else if  ((degs > (heavy_stick_deg-dist)) && (degs < (heavy_stick_deg+dist))){
        direction = (direction+1)%2;
        new_duty = 0.95;
        // printf("heavy stick\r\n");  
    } else if  ((degs > (light_slip_deg-dist)) && (degs < (light_slip_deg+dist))){
        direction = current_direction;
        new_duty = 0.65;
        // printf("light slip\r\n");  
    } else if  ((degs > (heavy_slip_deg-dist)) && (degs < (heavy_slip_deg+dist))){
        direction = current_direction;
        new_duty = 0.85;
        // printf("heavy slip\r\n degs %f\r\n",degs );  
    } else if  ((degs > (speed_bump-bump_dist)) && (degs < (speed_bump+bump_dist))){
        direction = (direction+1)%2;
        new_duty = 0.55;
        // printf("speed_bump\r\n degs %f\r\n",degs );  
    } 
    else {
        new_duty = 0.0;
    }
    pwm_set_direction(direction);
    pwm_set_duty(new_duty);
    return new_duty;
}

//Change master count to degs
double count_to_deg(double new_count) {
	double degs = new_count/714.15;
	return degs; 
}

void motor_control(double degs, double target_degs){
	double diff = degs - target_degs;
	float new_duty;
	double threshold = 1;
	unsigned char direction;
	if (diff > threshold){
		direction = 1;
		new_duty = 0.85;
	}
	else if (diff < -threshold){
		direction = 0;
		new_duty = 0.85;
	}
	else {
		direction = pwm_direction;
		new_duty = 0.0;
	}
	pwm_set_direction(direction);
	pwm_set_duty(new_duty);
}

void VendorRequests(void) {
    WORD32 address;
    WORD result;
    WORD temp;
    WORD temp0, temp1;
    float move_degs;

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
        case SET_TEXTURE_LT_ST:
            light_stick_deg = ((double)(USB_setup.wValue.w)/(double)(USB_setup.wIndex.w)) - 50;
            printf("Set wall angle to %4f\r\n", light_stick_deg);
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case SET_TEXTURE_HV_ST:
            heavy_stick_deg= ((double)(USB_setup.wValue.w)/(double)(USB_setup.wIndex.w)) - 50;
            printf("Set wall angle to %4f\r\n", heavy_stick_deg);
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case SET_TEXTURE_LT_SL:
            light_slip_deg = ((double)(USB_setup.wValue.w)/(double)(USB_setup.wIndex.w)) - 50;
            printf("Set wall angle to %4f\r\n", light_slip_deg);
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case SET_TEXTURE_HV_SL:
            heavy_slip_deg = ((double)(USB_setup.wValue.w)/(double)(USB_setup.wIndex.w)) - 50;
            printf("Set wall angle to %4f\r\n", heavy_slip_deg);
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        case SET_TEXTURE_SB:
            speed_bump = ((double)(USB_setup.wValue.w)/(double)(USB_setup.wIndex.w)) - 50;
            printf("Set wall angle to %4f\r\n", speed_bump);
            BD[EP0IN].bytecount = 0;    // set EP0 IN byte count to 0 
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
            break;
        // case READ_TEXTURE_PWM:
        //     READ_TEXTURE_PWM=(degs + 50) * 100;
        //     temp.w = round(move_degs);
        //     BD[EP0IN].address[0] = temp.b[0];
        //     BD[EP0IN].address[1] = temp.b[1];
        //     BD[EP0IN].bytecount = 2;    // set EP0 IN byte count to 2
        //     BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        //     break;
        // case READ_TEXTURE_DIRECTION:
        //     move_degs=(degs + 50) * 100;
        //     temp.w = round(move_degs);
        //     BD[EP0IN].address[0] = temp.b[0];
        //     BD[EP0IN].address[1] = temp.b[1];
        //     BD[EP0IN].bytecount = 2;    // set EP0 IN byte count to 2
        //     BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        //     break;
        // case READ_TEXTURE_ANGLE:
        //     move_degs=(degs + 50) * 100;
        //     temp.w = round(move_degs);
        //     BD[EP0IN].address[0] = temp.b[0];
        //     BD[EP0IN].address[1] = temp.b[1];
        //     BD[EP0IN].bytecount = 2;    // set EP0 IN byte count to 2
        //     BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
        //     break;
        case READ_HAPTIC_POS:
            temp0.w = round((degs + 50) * 100);
            BD[EP0IN].address[0] = temp0.b[0];
            BD[EP0IN].address[1] = temp0.b[1];
            float step1 = (current_pwm * (float)((pwm_direction == 1) ? 1:-1));
            temp1.w = round( (step1 + 5) * 1000);
            BD[EP0IN].address[2] = temp1.b[0];
            BD[EP0IN].address[3] = temp1.b[1];
            BD[EP0IN].bytecount = 4;    // set EP0 IN byte count to 2
            BD[EP0IN].status = 0xC8;    // send packet as DATA1, set UOWN bit
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
    switch (USB_request.setup.bRequest) {
        default:
            USB_error_flags |= 0x01;                    // set Request Error Flag
    }
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
    // unsigned char angle_array[14];  // LSB will be angle_array[0]

    // float pwm_duty_array[4] = {0.40, 0.41, 0.42, 0.43};
    // float pwm_duty_array[4] = {0.12, 0.12, 0, 0};

    // pwm_set_direction(!pwm_direction);
    // pwm_set_duty(0);
    printf("%s\n", "STARTING LOOP");
    double encoder_master_count = 0;
    uint16_t current_ticks = 0;
    uint16_t previous_ticks = spi_read_ticks();
    double target_degs = 35;
    while (1) {
        if (timer_flag(&timer2)) {
            // Blink green light to show normal operation.
            timer_lower(&timer2);
            led_toggle(&led2);
            // printf("%s\r\n", "BLINK LIGHT");
            // printf("MASTER COUNT: %f\r\n", encoder_master_count);
            // printf("MASTER DEGS: %f\r\n", degs);
        }
        if (!sw_read(&sw2)) {
            // If switch 2 is pressed, the UART output terminal is cleared.
            printf("%s", clear);
        }
        current_ticks = spi_read_ticks();
        encoder_master_count = encoder_counter(current_ticks, previous_ticks, encoder_master_count);
        degs = count_to_deg(encoder_master_count);
        current_pwm = texture(degs);
        previous_ticks = current_ticks;
        ServiceUSB();
    }
}

