#include <Arduino.h>
#include <SPI.h>
#include <cmath>
#include <ArduinoEigen.h>
// #include "ArduinoEigen/ArduinoEigenCommon.h"


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
float previous_left_angle = 0;
float previous_right_angle = 0;
float left_offset = 0;
float right_offset = 0;
float start_time;
float circle_radius = 0.15; //Meters
float circle_period = 1; //Seconds
float angular_frequency = 2 * PI / circle_period;
float left_accumulated_error = 0;
float right_accumulated_error = 0;
float left_previous_error = 0;
float right_previous_error = 0;
double previous_time;
float kp = 1;
float ki = 0;
float kd = 0.0006;
float max_pwm = 20;

float tf = 1;

array<float,6> cx;
array<float,6> cy;

/*
Read left and right motor angles from the encoders.
Angles are returned in degrees.
*/
array<float, 2> read_motor_angles() {
    array<float, 2> angles;
    u_int16_t serial_response; // incoming byte from the SPI
    int chips[2] = {ENC_CHIP_SELECT_LEFT, ENC_CHIP_SELECT_RIGHT};

    digitalWrite(chips[0], LOW);
    serial_response = SPI.transfer16(0x3FFF);
    digitalWrite(chips[0], HIGH);
    angles[0] = (serial_response & 0b0011111111111111) * 360.0 / 16384;

    if(angles[0] - previous_left_angle > ROLLOVER_ANGLE_DEGS) {
        left_revolutions -= 1;
    } else if(previous_left_angle - angles[0] > ROLLOVER_ANGLE_DEGS) {
        left_revolutions += 1;
    }

    previous_left_angle = angles[0];

    digitalWrite(chips[1], LOW);
    serial_response = SPI.transfer16(0x3FFF);
    digitalWrite(chips[1], HIGH);
    angles[1] = (serial_response & 0b0011111111111111) * 360.0 / 16384;

    if(angles[1] - previous_right_angle > ROLLOVER_ANGLE_DEGS) {
        right_revolutions -= 1;
    } else if (previous_right_angle - angles[1] > ROLLOVER_ANGLE_DEGS) {
        right_revolutions += 1;
    }

    previous_right_angle = angles[1];

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
float pid(float kp, float ki, float kd, float error, float accumulated_error, float previous_error, double dt) {
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
it into motor angles in degrees
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
    set_motor_pwms(-x_speed, -x_speed);
    delay(200);

    float previous_left_encoder = read_motor_angles()[0];

    while(true) {
        delay(100);
        if(abs(previous_left_encoder - read_motor_angles()[0]) < position_threshold) {
            set_motor_pwms(0, 0);
            break;
        }
        previous_left_encoder = read_motor_angles()[0];
    }

    //Home Y
    set_motor_pwms(-y_speed, y_speed);
    delay(200);

    previous_left_encoder = read_motor_angles()[0];

    while(true) {
        delay(100);
        if(abs(previous_left_encoder - read_motor_angles()[0]) < position_threshold) {
            set_motor_pwms(0, 0);
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

void command_motors(float x_pos, float y_pos, double current_time, double previous_time) {
    array<float, 2> target_angles = xy_to_theta(x_pos, y_pos);

    array<float,2> actual_angles = read_motor_angles();

    float left_error = target_angles[0] - actual_angles[0];
    float right_error = target_angles[1] - actual_angles[1];

    float left_pid = pid(kp, ki, kd, -left_error, left_accumulated_error, left_previous_error, current_time - previous_time);
    float right_pid = pid(kp, ki, kd, -right_error, right_accumulated_error, right_previous_error, current_time - previous_time);

    float left_pwm = fmin(fmax(-max_pwm, left_pid), max_pwm);
    float right_pwm = fmin(fmax(-max_pwm, right_pid), max_pwm);

    Serial.print(current_time*1000);
    Serial.print(",");
    Serial.print(x_pos*100);
    Serial.print(",");
    Serial.print(y_pos*100);
    Serial.print(",");
    Serial.print(left_error);
    Serial.print(",");
    Serial.print(right_error);
    Serial.print(",");
    Serial.print(left_pwm);
    Serial.print(",");
    Serial.println(right_pwm);

    set_motor_pwms(left_pwm, right_pwm);
}


array<float,3> get_intermediate_point(float x, float y, float vx, float vy, float final_time, float straight_length) {
    float final_velocity_magnitude = sqrt(pow(vx,2) + pow(vy,2));
    float reverse_x_component = -vx / final_velocity_magnitude * straight_length;
    float reverse_y_component = -vy / final_velocity_magnitude * straight_length;
    
    float intermediate_x = x + reverse_x_component;
    float intermediate_y = y + reverse_y_component;

    float intermediate_t = final_time - straight_length / final_velocity_magnitude;

    return {{intermediate_x, intermediate_y, intermediate_t}};
}

array<float,12> get_trajectory_coeffs(float t0, float x0, float y0, float vx0, float vy0, float tf, float xf, float yf, float vxf, float vyf, float d) {
    array<float,3> intermediate_params = get_intermediate_point(xf, yf, vxf, vyf, tf, 0.1);

    float x1 = intermediate_params[0];
    float y1 = intermediate_params[1];
    float t1 = intermediate_params[2];
    
    Eigen::Matrix<float,6,6> mat;

    mat << 1, t0, pow(t0,2), pow(t0,3), pow(t0,4), pow(t0,5),
        0, 1, 2*t0, 3*pow(t0,2), 4*pow(t0,3), 5*pow(t0,4),
        1, t1, pow(t1,2), pow(t1,3), pow(t1,4), pow(t1,5),
        0, 1, 2*t1, 3*pow(t1,2), 4*pow(t1,3), 5*pow(t1,4),
        1, tf, pow(tf,2), pow(tf,3), pow(tf,4), pow(tf,5),
        0, 1, 2*tf, 3*pow(tf,2), 4*pow(tf,3), 5*pow(tf,4);

    Eigen::Vector<float,6> bx(x0, vx0, x1, vxf, xf, vxf);

    Eigen::Vector<float,6> ax = mat.colPivHouseholderQr().solve(bx);

    Eigen::Vector<float,6> by(y0, vy0, y1, vyf, yf, vyf);

    Eigen::Vector<float,6> ay = mat.colPivHouseholderQr().solve(by);

    return {{ax[0], ax[1], ax[2], ax[3], ax[4], ax[5], ay[0], ay[1], ay[2], ay[3], ay[4], ay[5]}};
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

    home_table(9, 5, 10);

    // Trajectory parameters
    float t0 = 0;
    array<float,2> start_angles = read_motor_angles();
    array<float,2> start_position = theta_to_xy(start_angles[0], start_angles[1]);
    float x0 = start_position[0];
    float y0 = start_position[1];
    float vx0 = 0;
    float vy0 = 0;

    tf = 1;
    float xf = 0.5;
    float yf = 0.5;
    float vxf = 0.1;
    float vyf = 0;

    array<float,12> coeffs = get_trajectory_coeffs(t0, x0, y0, vx0, vy0, tf, xf, yf, vxf, vyf, 0.05);

    cx = {{coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5]}};
    cy = {{coeffs[6], coeffs[7], coeffs[8], coeffs[9], coeffs[10], coeffs[11]}};

    delay(500);

    read_motor_angles(); //Need a dummy call to get the previous angle variable set properly
    // left_offset = read_motor_angles()[0];
    // right_offset = read_motor_angles()[1];

    Serial.println("BEGIN CSV");
    Serial.println("Time(ms),X_Target(cm),Y_Target(cm),Left_Error(deg),Right_Error(deg),Left_PWM,Right_PWM");

    previous_time = 0;
    start_time = micros();
}

void loop() {
    double t = (micros() - start_time) / 1000000.0;

    // float x_pos = circle_radius * (cos(angular_frequency * current_time)-1);
    // float y_pos = circle_radius * sin(angular_frequency * current_time);

    float x_pos = cx[0] + cx[1]*t + cx[2]*pow(t,2) + cx[3]*pow(t,3) + cx[4]*pow(t,4) + cx[5]*pow(t,5);
    float y_pos = cy[0] + cy[1]*t + cy[2]*pow(t,2) + cy[3]*pow(t,3) + cy[4]*pow(t,4) + cy[5]*pow(t,5);

    // Serial.println(cx[0]);
    // Serial.println(cx[1]);
    // Serial.println(cx[2]);
    // Serial.println(cx[3]);
    // Serial.println(cx[4]);
    // Serial.println(cx[5]);
    // Serial.println(cy[0]);
    // Serial.println(cy[1]);
    // Serial.println(cy[2]);
    // Serial.println(cy[3]);
    // Serial.println(cy[4]);
    // Serial.println(cy[5]);
    // Serial.print("Time (s): ");
    // Serial.print(t);
    // Serial.print("\tX (cm): ");
    // Serial.print(x_pos*100);
    // Serial.print("\tY (cm): ");
    // Serial.println(y_pos*100);
    command_motors(x_pos, y_pos, t, previous_time);

    previous_time = t;

    // delay(100);

    if (t > tf) {
        set_motor_pwms(0.0,0.0);
        exit(0);
    }
}