#include <Arduino.h>
#include <SPI.h>
#define ENC_CHIP_SELECT_LEFT A4
#define ENC_CHIP_SELECT_RIGHT PB5
#define DUTY_CYCLE_CONVERSION 1024 // Accepted duty cycle values are 0-1024
#define PWM_FREQ_HZ 10000
#define LEFT_MOTOR_PWM_PIN PB_7
#define RIGHT_MOTOR_PWM_PIN PB_8
#define ROLLOVER_ANGLE_DEGS 180

using namespace std;

// Define Global Variables
int left_revolutions = 0;
int right_revolutions = 0;
float current_left_angle = 0;
float current_right_angle = 0;


/*
  Read left and right motor angles from the encoders.
  Angles are returned in degrees.
*/
array<float, 2> read_motor_angles()
{
    array<float, 2> angles;
    u_int16_t serial_response; // incoming byte from the SPI
    int chips[2] = {ENC_CHIP_SELECT_LEFT, ENC_CHIP_SELECT_RIGHT};

    // TODO: Subtract starting angles or use the encoder's zeroing capabilities

    digitalWrite(chips[0], LOW);
    serial_response = SPI.transfer16(0x3FFF);
    digitalWrite(chips[0], HIGH);
    angles[0] = (serial_response & 0b0011111111111111) * 360.0 / 16384;

    if(angles[0] - current_left_angle > ROLLOVER_ANGLE_DEGS) {
        left_revolutions -= 1;
    } else if(current_left_angle - angles[0] > ROLLOVER_ANGLE_DEGS) {
        left_revolutions += 1;
    }

    current_left_angle = angles[0];

    digitalWrite(chips[1], LOW);
    serial_response = SPI.transfer16(0x3FFF);
    digitalWrite(chips[1], HIGH);
    angles[1] = (serial_response & 0b0011111111111111) * 360.0 / 16384;

    if(angles[1] - current_right_angle > ROLLOVER_ANGLE_DEGS) {
        right_revolutions -= 1;
    } else if (current_right_angle - angles[1] > ROLLOVER_ANGLE_DEGS) {
        right_revolutions += 1;
    }

    current_right_angle = angles[1];
    
    return angles;
}

/*
  Command a motor velocity to the specified motor.
  input values should be in the range [-100,100].
*/
void set_motor_pwms(float left, float right)
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
    Serial.print("Right motor Angle: ");
    Serial.println(angles[1] + right_revolutions * 360);
    
    // Serial.print("\tRight motor Angle: ");
    // Serial.println(angles[1]);

    // for (int i = 0; i < 100; i++) {
    //     set_motor_pwms(i, i);
    //     delay(1);
    // }
}