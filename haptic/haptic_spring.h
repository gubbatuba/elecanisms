/*
 Haptic Controller Configuration
*/

// SPI Configuration
static const float spi_freq = 9500000;
_SPI* spi_inst = &spi1;     // SPI instance (address of)
static const uint8_t spi_mode = 1;

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

#define REG_MAG_ADDR        0x3FFE

// PWM Configuration
float pwm_freq = 300;       // ~245Hz is minimum, 35000 was original
float pwm_duty = 0;      // Safely start with no commanded motor motion
unsigned char pwm_direction = 1;  // Initialize motor for forward motion
_PIN* PWM_I1 = &D[8];       // Input 1 to motor driver chip
_PIN* PWM_I2 = &D[7];       // Input 2 to motor driver chip

_PIN* MOTOR_VOLTAGE = &A[0];       // Motor control voltage measurement
static const long DUTY_MAX = 65536;  // Value used for converting uint16_t
                            // fractional representations and float percentages.

static const char clear[5]  = {27, '[', '2', 'J', 0};  // Term. CLEAR sequence

// Position data
float degs = 0;

// Motor Current Conversion
static const float MAX_ANALOG_VOLTAGE = 3;
static const float MAX_ADC_OUTPUT = 1023;
static const float MOTOR_VOLTAGE_RESISTOR = 0.075;  // Ohms
float CURRENT_CONV_COEF;

// PID Control
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float set_point;
    bool neg_set_point;
    float dt;
    float position;
    float prev_position;
    float integ_state;
    float integ_max, integ_min;
} PID;
PID cur_control;

typedef struct {
    uint16_t raw_volts;
    float volts;
    float current;
} MOTOR;
MOTOR motor;


uint8_t interrupted = 0;
// Spring model
float SPRING_CONSTANT = .08;

// Motor Configuration
// http://www.jameco.com/Jameco/Products/ProdDS/238473.PDF
float MOTOR_TORQUE_COEF = 1.06;
// float PWM_MIN = 0.25;
float MAX_DUTY = 0.60;

_PIN* DEBUGD0 = &D[12]; 
_PIN* DEBUGD1 = &D[13]; 
#define LOOP_TIME 0.001

#define KP .3
#define KI 0
#define KD .01

#define round(x) ((x)>=0?(uint16_t)((x)+0.5):(uint16_t)((x)-0.5))
