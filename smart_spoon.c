#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#define MPU_ADDR 0x68  // I2C address of the MPU6050
#define F_CPU 16000000UL // Define CPU frequency for delay functions

// I2C Functions for AVR
void I2C_init(void);
void I2C_start(void);
void I2C_stop(void);
void I2C_write(uint8_t data);
uint8_t I2C_read_ack(void);
uint8_t I2C_read_nack(void);

// MPU6050 Functions
void MPU6050_init(void);
void MPU6050_read(int16_t* AcX, int16_t* AcY, int16_t* AcZ, int16_t* GyX, int16_t* GyY);

void servo_init();
void servo_write(uint8_t angle);
uint16_t map(int value, int fromLow, int fromHigh, int toLow, int toHigh);

// Define I2C functions (simplified for ATmega328P)
void I2C_init() {
    TWSR = 0; // Set prescaler to 1
    TWBR = 72; // Set bit rate register (for 100kHz clock)
    TWCR = (1 << TWEN); // Enable I2C
}

void I2C_start() {
    TWCR = (1<<TWSTA) | (1<<TWEN) | (1<<TWINT);  // Send START condition
    while (!(TWCR & (1<<TWINT)));  // Wait for transmission to complete
}

void I2C_stop() {  
    TWCR = (1<<TWSTO) | (1<<TWINT) | (1<<TWEN);  // Send STOP condition
    while (TWCR & (1<<TWSTO));
}

void I2C_write(uint8_t data) {
    TWDR = data; // Load data into TWDR register
    TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
    while (!(TWCR & (1 << TWINT))); // Wait for transmission to complete
}

uint8_t I2C_read_ack() {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // Start receiving with ACK
    while (!(TWCR & (1 << TWINT))); // Wait for reception to complete
    return TWDR; // Return received data
}

uint8_t I2C_read_nack() {
    TWCR = (1 << TWINT) | (1 << TWEN); // Start receiving without ACK
    while (!(TWCR & (1 << TWINT))); // Wait for reception to complete
    return TWDR; // Return received data
}

void MPU6050_init() {
    I2C_start(); // Start I2C communication
    I2C_write(MPU_ADDR << 1); // Write to the MPU6050 (0x68)
    I2C_write(0x6B); // Power management register
    I2C_write(0x00); // Wake up the MPU6050 (clear sleep bit)
    I2C_stop(); // Stop I2C communication
}

void MPU6050_read(int16_t* AcX, int16_t* AcY, int16_t* AcZ, int16_t* GyX, int16_t* GyY) {
    I2C_start();
    I2C_write(MPU_ADDR << 1); // Write to MPU6050
    I2C_write(0x3B); // Start with ACCEL_XOUT_H
    I2C_start();  // Restart and switch to reading
    I2C_write((MPU_ADDR << 1) | 1); // Read from MPU6050

    // Read accelerometer data
    *AcX = (I2C_read_ack() << 8) | I2C_read_ack();
    *AcY = (I2C_read_ack() << 8) | I2C_read_ack();
    *AcZ = (I2C_read_ack() << 8) | I2C_read_ack();

    // Read gyroscope data
    *GyX = (I2C_read_ack() << 8) | I2C_read_ack();
    *GyY = (I2C_read_nack() << 8) | I2C_read_nack();
    I2C_stop();
}

// Servo control functions (basic PWM implementation)
void servo_init() {
    DDRB |= (1<<DDB1); // Set PWM pin as output (OC1A on pin 15 / PB1)
    TCCR1A = (1<<COM1A1) | (1<<WGM11); // Fast PWM mode, non-inverted
    TCCR1B = (1<<WGM12) |(1<<WGM13) | (1<<CS11);   // Fast PWM mode, prescaler 8
    ICR1 = 19999; // Set the top value for 50Hz frequency
}

void servo_write(uint8_t angle) {
    uint16_t pulse = map(angle, 0, 180, 1000, 2000); // Map angle to pulse width
    OCR1A = pulse; // Set the pulse width
}

uint16_t map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

int main() {
    int16_t AcX, AcY, AcZ, GyX, GyY;
    float angleEstimateX = 0, angleEstimateY = 0;
    float alpha = 0.98, dt = 0.1;
    int gyroXOffset = -239, gyroYOffset = 230;

    I2C_init();
    MPU6050_init();
    servo_init();

    while (1) {
        MPU6050_read(&AcX, &AcY, &AcZ, &GyX, &GyY);

        GyX -= gyroXOffset;
        GyY -= gyroYOffset;

        float gyroRateX = GyX / 131.0;
        float gyroRateY = GyY / 131.0;

        float accelAngleX = atan2(AcY, AcZ) * 180 / M_PI;
        float accelAngleY = atan2(AcX, AcZ) * 180 / M_PI;

        angleEstimateX = alpha * (angleEstimateX + gyroRateX * dt) + (1 - alpha) * accelAngleX;
        angleEstimateY = alpha * (angleEstimateY + gyroRateY * dt) + (1 - alpha) * accelAngleY;

        int mappedAngleX = (int)fmin(fmax(angleEstimateX + 90, 0), 180);
        int mappedAngleY = (int)fmin(fmax(angleEstimateY + 90, 0), 180);

        servo_write(mappedAngleX);
        servo_write(mappedAngleY);

        _delay_ms(100);
    }
    return 0;
}
