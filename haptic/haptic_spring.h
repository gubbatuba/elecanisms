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

#define TOGGLE_LED1         1
#define TOGGLE_LED2         2
#define READ_SW1            3
#define ENC_WRITE_REG       4
#define ENC_READ_REG        5
#define TOGGLE_LED3         8
#define READ_SW2            9
#define READ_SW3            10
#define SET_PID             11

#define REG_MAG_ADDR        0x3FFE

// PWM Configuration
float pwm_freq = 3000;       // ~245Hz is minimum, 35000 was original
uint16_t pwm_duty = 0;      // Safely start with no commanded motor motion
unsigned char pwm_direction = 1;  // Initialize motor for forward motion
_PIN* PWM_I1 = &D[8];       // Input 1 to motor driver chip
_PIN* PWM_I2 = &D[7];       // Input 2 to motor driver chip

_PIN* MOTOR_VOLTAGE = &A[0];       // Motor control voltage measurement
static const long DUTY_MAX = 65536;  // Value used for converting uint16_t
                            // fractional representations and float percentages.

static const char clear[5]  = {27, '[', '2', 'J', 0};  // Term. CLEAR sequence

// Angle Conversion
// convrate = 715.15

// Motor Current Conversion
static const double MAX_ANALOG_VOLTAGE = 3;
static const double MAX_ADC_OUTPUT = 1023;
static const double MOTOR_VOLTAGE_RESISTOR = 0.075;  // Ohms
double CURRENT_CONV_COEF;

// PID Control
typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double set_point;
    double dt;
    double position;
    double prev_position;
    double integ_state;
    double integ_max, integ_min;
} PID;

// Spring model
double SPRING_CONSTANT = 1;

// Motor Configuration
// http://www.jameco.com/Jameco/Products/ProdDS/238473.PDF
double MOTOR_TORQUE_COEF = 1.06;
double PWM_MIN = 0.25;
double PWM_MAX = 0.99;

#define LOOP_TIME 0.005

PID cur_control;

