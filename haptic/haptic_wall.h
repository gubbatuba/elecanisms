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

#define TOGGLE_LED1     1
#define TOGGLE_LED2     2
#define TOGGLE_LED3     3
#define READ_SW1        4
#define READ_SW2        5
#define READ_SW3        6
#define ENC_READ_REG    7
#define SET_PID_P       8
#define SET_PID_I       9
#define SET_PID_D       10
#define SET_SPRING_CONSTANT     11
#define READ_POSITION           12
#define READ_CURRENT            13
#define READ_VELOCITY           14
#define SET_DAMPER_COEF         15
#define READ_WALL_ANGLE			16

#define REG_MAG_ADDR        0x3FFE

// PWM Configuration
float pwm_freq = 3000;       // ~245Hz is minimum, 35000 was original
uint16_t pwm_duty = 0;      // Safely start with no commanded motor motion
unsigned char pwm_direction = 1;  // Initialize motor for forward motion
unsigned char current_direction; // the direction the motor is moving
_PIN* PWM_I1 = &D[8];       // Input 1 to motor driver chip
_PIN* PWM_I2 = &D[7];       // Input 2 to motor driver chip
long DUTY_MAX = 65536;      // Value used for converting between uint16_t
                            // fractional representations and float percentages.

char clear[5]  = {27, '[', '2', 'J', 0};  // Terminal CLEAR sequence

// Angle Conversion
// convrate = 715.15
uint16_t head_center = 0;
double WALL_ANGLE = 20;

