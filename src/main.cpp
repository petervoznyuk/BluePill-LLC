#include <Arduino.h>
#include <SPI.h>
#define ENC_CHIP_SELECT_LEFT PA4
#define ENC_CHIP_SELECT_RIGHT PB5
#define LEFT_MOTOR_PWM_PIN PB_7
#define RIGHT_MOTOR_PWM_PIN PB_8
#define LEFT_MOTOR_DIR_PIN PB3
#define RIGHT_MOTOR_DIR_PIN PB4
#define DUTY_CYCLE_CONVERSION 1024 // Accepted duty cycle values are 0-1024
#define PWM_FREQ_HZ 10000
#define ROLLOVER_ANGLE_DEGS 180
#define PULLEY_RADIUS 0.035 // In meters


using namespace std;

// Define Global Variables
int left_revolutions = 0;
int right_revolutions = 0;
float current_left_angle = 0;
float current_right_angle = 0;
float left_offset = 0;
float right_offset = 0;
float start_time;


/*
Read left and right motor angles from the encoders.
Angles are returned in degrees.
*/
array<float, 2> read_motor_angles() {
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

    angles[0] = angles[0] + 360.0 * left_revolutions - left_offset;
    angles[1] = angles[1] + 360.0 * right_revolutions - right_offset;
    
    return angles;
}


/*
Command a motor velocity to the specified motor.
input values should be in the range [-100,100].
*/
void set_motor_pwms(float left, float right) {
    if (left >= 0) {
        digitalWrite(LEFT_MOTOR_DIR_PIN, LOW);
    } else {
        digitalWrite(LEFT_MOTOR_DIR_PIN, HIGH);
    }
    pwm_start(LEFT_MOTOR_PWM_PIN, PWM_FREQ_HZ, floor(abs(left) / 100.0 * DUTY_CYCLE_CONVERSION), RESOLUTION_10B_COMPARE_FORMAT);

    if (right >= 0) {
        digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);
    } else {
        digitalWrite(RIGHT_MOTOR_DIR_PIN, HIGH);
    }
    pwm_start(RIGHT_MOTOR_PWM_PIN, PWM_FREQ_HZ, floor(abs(right) / 100.0 * DUTY_CYCLE_CONVERSION), RESOLUTION_10B_COMPARE_FORMAT);
}

/*
Return the sum of the PID terms
*/
float pid(float kp, float ki, float kd, float error, float accumulated_error, float previous_error, float dt) {
    float error_derivative = (error - previous_error) / dt;

    return error * kp + accumulated_error * ki + error_derivative * kd;
}

/*
Takes motor angles in degrees
and converts them into cartesian position of the mallet in meters
*/
array<float,2> theta_to_xy(float theta_l, float theta_r) {
    float x = (theta_l + theta_r) * PULLEY_RADIUS * PI / 360;
    float y = (theta_l - theta_r) * PULLEY_RADIUS * PI / 360;

    return {{-x, -y}};
}

/*
Takes cartesian position of the mallet (x, y) in meters and converts
it into motor angles in encoder ticks (2048 ticks per revolution)
*/
array<float,2> xy_to_theta(float x, float y) {
    float theta_l = (x + y) / PULLEY_RADIUS * 360 / (2*PI);
    float theta_r = (x - y) / PULLEY_RADIUS * 360 / (2*PI);

    return {{-theta_l, -theta_r}};
}

/*
Home the table in the bottom left corner 
and zero the encoders.
*/
void home_table(float x_speed, float y_speed, float position_threshold) {
    //Home X
    Serial.println("Homing in X...");
    set_motor_pwms(-x_speed, -x_speed);
    delay(200);

    float previous_left_encoder = read_motor_angles()[0];

    while(true) {
        delay(100);
        if(abs(previous_left_encoder - read_motor_angles()[0]) < position_threshold) {
            set_motor_pwms(0, 0);
            Serial.println("Homed in X");
            break;
        }
        previous_left_encoder = read_motor_angles()[0];
    }

    //Home Y
    Serial.println("Homing in Y...");
    set_motor_pwms(-y_speed, y_speed);
    delay(200);

    previous_left_encoder = read_motor_angles()[0];

    while(true) {
        delay(100);
        if(abs(previous_left_encoder - read_motor_angles()[0]) < position_threshold) {
            set_motor_pwms(0, 0);
            Serial.println("Homed in Y");
            break;
        }
        previous_left_encoder = read_motor_angles()[0];
    }

    //Nudge into the corner
    set_motor_pwms(-x_speed, 0);
    delay(500);
    set_motor_pwms(0, 0);
    left_revolutions = 0;
    right_revolutions = 0;
    left_offset = read_motor_angles()[0];
    right_offset = read_motor_angles()[1];
    Serial.println("Fully Homed");
}


void setup() {
    Serial.begin(460800);
    SPI.beginTransaction(SPISettings(460800, MSBFIRST, SPI_MODE1));
    
    pinMode(ENC_CHIP_SELECT_LEFT, OUTPUT);
    pinMode(ENC_CHIP_SELECT_RIGHT, OUTPUT);
    
    // Turn green LED on
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, LOW);

    pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
    
    pwm_start(LEFT_MOTOR_PWM_PIN, PWM_FREQ_HZ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    pwm_start(RIGHT_MOTOR_PWM_PIN, PWM_FREQ_HZ, 0, RESOLUTION_10B_COMPARE_FORMAT);
    digitalWrite(LEFT_MOTOR_DIR_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);

    start_time = millis();

    home_table(7, 6, 10);
}

void loop() {
    // float loop_start_time = millis();

    // Serial.println(millis() - start_time);
    
    int speed = 5;

    set_motor_pwms(speed, -speed);
    delay(2000);
    set_motor_pwms(-speed, speed);
    delay(2000);

    array<float,2> angles = read_motor_angles();
    array<float,2> pos = theta_to_xy(angles[0], angles[1]);
    Serial.print("x (cm): ");
    Serial.print(pos[0] * 100);
    Serial.print("\ty (cm): ");
    Serial.println(pos[1] * 100);
    delay(5);

    // for (int i = 0; i < 100; i++) {
    //     set_motor_pwms(i, i);
    //     delay(1);
    // }
}