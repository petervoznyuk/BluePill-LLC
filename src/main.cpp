#include <Arduino.h>
#include <SPI.h>
#define ENC_CHIP_SELECT_LEFT A4
#define ENC_CHIP_SELECT_RIGHT PB5
#define DUTY_CYCLE_CONVERSION 1024 // Accepted duty cycle values are 0-1024
#define PWM_FREQ_HZ 10000
#define LEFT_MOTOR_PWM_PIN PB_7
#define RIGHT_MOTOR_PWM_PIN PB_8

using namespace std;


void setup()
{
    Serial.begin(460800);
    SPI.beginTransaction(SPISettings(460800, MSBFIRST, SPI_MODE1));
    
    pinMode(ENC_CHIP_SELECT_LEFT, OUTPUT);
    pinMode(ENC_CHIP_SELECT_RIGHT, OUTPUT);
    
    // Turn green LED on
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, LOW);

    pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
    
    pwm_start(LEFT_MOTOR_PWM_PIN, PWM_FREQ_HZ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(RIGHT_MOTOR_PWM_PIN, PWM_FREQ_HZ, 0, RESOLUTION_10B_COMPARE_FORMAT);

    float start_time = millis();
}

void loop()
{
    float loop_start_time = millis();

    array<float,2> angles = read_motor_angles();
    Serial.print("Left motor Angle: ");
    Serial.print(angles[0]);
    Serial.print("\tRight motor Angle: ");
    Serial.println(angles[1]);

    for (int i = 0; i < 100; i++) {
        set_motor_speeds(i, i);
        delay(1);
    }
}

/*
  Read left and right motor angles from the encoders.
  Angles are returned in degrees.
*/
array<float, 2> read_motor_angles()
{
    array<float, 2> angles;
    u_int16_t serial_response; // incoming byte from the SPI
    int chips[2] = {ENC_CHIP_SELECT_LEFT, ENC_CHIP_SELECT_RIGHT};

    for (int i = 0; i < 2; i++) {
        digitalWrite(chips[i], LOW);
        serial_response = SPI.transfer16(0x3FFF);
        digitalWrite(chips[i], HIGH);
        angles[i] = (serial_response & 0b0011111111111111) * 360.0 / 16384; // TODO: Subtract starting angles or use the encoder's zeroing capabilities
    }

    return angles;
}

/*
  Command a motor velocity to the specified motor.
  input values should be in the range [-100,100].
*/
void set_motor_speeds(float left, float right)
{
    if (left >= 0) {
        // Set left motor direction pin to "forward"
    } else {
        // Set left motor direction pin to "backward"
    }
    pwm_start(LEFT_MOTOR_PWM_PIN, PWM_FREQ_HZ, floor(abs(left) / 100.0 * DUTY_CYCLE_CONVERSION), RESOLUTION_10B_COMPARE_FORMAT);

    if (right >= 0) {
        // Set right motor direction pin to "forward"
    } else {
        // Set right motor direction pin to "backward"
    }
    pwm_start(RIGHT_MOTOR_PWM_PIN, PWM_FREQ_HZ, floor(abs(right) / 100.0 * DUTY_CYCLE_CONVERSION), RESOLUTION_10B_COMPARE_FORMAT);
}
