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
float spi_freq = 24000;  // 24000 is chosen fairly arbitrarily. Revise later
_SPI* spi_inst = &spi1;  // SPI instance (address of)

_PIN* SPI_SCK = &D[2];
_PIN* SPI_MOSI = &D[0];
_PIN* SPI_MISO = &D[1];
_PIN* SPI_CS = &D[3];

// PWM Configuration
float pwm_freq = 500;       // ~245Hz is minimum
uint16_t pwm_duty = 0;      // Start with no commanded motor motion
_PIN* PWM_I1 = &D[8];       // Input 1 to motor driver chip
_PIN* PWM_I2 = &D[7];       // Input 2 to motor driver chip
uint32_t DUTY_MAX = 65536;
