/*
 Haptic Controller Configuration
*/

// SPI Configuration
uint8_t spi_NOP12 = 0x00;
/* SPI READ command:
    PAR = 1;
    RWn = 1;
    Address = 0x3FFF = b1111111111111;
    Read command = 0xFFFF = b111111111111111;
*/
uint8_t spi_READ1 = 0xFF;   // First half of READ transmission [MSB]
uint8_t spi_READ2 = 0xFF;   // Second half of READ transmission [LSB]
float spi_freq = 9500000;     // 24000 is chosen fairly arbitrarily. Revise later.
_SPI* spi_inst = &spi1;     // SPI instance (address of)
uint8_t spi_mode = 1;

_PIN* SPI_SCK = &D[2];      // Serial Clock pin
_PIN* SPI_MOSI = &D[0];     // MOSI pin
_PIN* SPI_MISO = &D[1];     // MISO pin
_PIN* SPI_CS = &D[3];       // Chip select pin

#define TOGGLE_LED1         1
#define TOGGLE_LED2         2
#define READ_SW1            3
#define ENC_WRITE_REG       4
#define ENC_READ_REG        5
#define TOGGLE_LED3         8
#define READ_SW2            9
#define READ_SW3            10

#define REG_MAG_ADDR        0x3FFE

// PWM Configuration
float pwm_freq = 8000;       // ~245Hz is minimum, 35000 was original
uint16_t pwm_duty = 0;      // Safely start with no commanded motor motion
float pwm_pct_duty = 0;
unsigned char pwm_direction = 1;  // Initialize motor for forward motion
_PIN* PWM_I1 = &D[8];       // Input 1 to motor driver chip
_PIN* PWM_I2 = &D[7];       // Input 2 to motor driver chip

_PIN* MOTOR_VOLTAGE = &A[0];       // Motor control voltage measurement
long DUTY_MAX = 65536;      // Value used for converting between uint16_t
                            // fractional representations and float percentages.

char clear[5]  = {27, '[', '2', 'J', 0};  // Terminal CLEAR sequence

// Angle Conversion
// convrate = 715.15
uint16_t head_center = 0;

// PID Control
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float set_point;
    float dt;
    float position;
    float prev_position;
    float integ_state;
    float integ_max, integ_min;
} PID;
PID pos_control;
#define LOOP_TIME 0.0005
float PWM_MIN = 0;
float PWM_MAX = 0.70;
#define KP .1
#define KI 0
#define KD 0
