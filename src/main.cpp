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
#define PULLEY_RADIUS 0.035 //meters
#define X_OFFSET 0.0699 //meters
#define Y_OFFSET 0.0508 //meters
#define X_MIN X_OFFSET //meters
#define X_MAX 1-X_OFFSET //meter
#define Y_MIN Y_OFFSET //meters
#define Y_MAX 1-Y_OFFSET //meters

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
float kp = 1.0;
float ki = 0;
// float kd = 0.0002;
float kd = 0.0;
float max_pwm = 100;

// ==================================
// Figure-eight trajectory
// float traj_durations[] = {0.2,0.2,0.2,0.2,0.2};
// array<array<float, 4>, 5> x_traj_coeffs = {{{0.0699,0,1.0903,-0.6602},{0.5,0.2,0.68,-0.52},{0.86,0,-0.88,0.52},{0.5,-0.2,-0.71,0.54},{0.13,0,0.91,-0.54}}};
// array<array<float, 4>, 5> y_traj_coeffs = {{{0.0508,0,0.9476,-0.4984},{0.5,0.4,-0.2,-0.2},{0.5,-0.6,0.8,-0.2},{0.5,0.4,-0.2,-0.2},{0.5,-0.6,0.8,-0.2}}};
// ==================================
// // 3 shots v6 (best) (1,2,3)
// float traj_durations[] = {0.2,0.3,0.3,0.2,0.3,0.3,0.3};
// array<array<float, 4>, 7> x_traj_coeffs = {{{0.0699,0,1.2717,-0.8478},{0.4938,0,-0.9939,0.7201},{0.22,0.1725,0.495,-0.3875},{0.5,0,0.015,-0.01},{0.505,0,-0.015,0.01},{0.5,0,0.975,-0.72},{0.755,-0.21,-0.345,0.3}}};
// array<array<float, 4>, 7> y_traj_coeffs = {{{0.0508,0,0.1476,-0.0984},{0.1,0,0.12,0.27},{0.49,1.05,-3.27,1.83},{0.1,0,0.37,0.02},{0.49,1.2,-3.57,1.98},{0.1,0,-0.18,0.57},{0.49,1.35,-3.87,2.13}}};
// bool do_loop = false;
// 3 shots (2,3,1)
// float traj_durations[] = {0.2,0.3,0.3,0.2,0.3,0.3,0.3};
// array<array<float, 4>, 7> x_traj_coeffs = {{{0.0699,0,1.2717,-0.8478},{0.5,0,0.015,-0.01},{0.505,0,-0.015,0.01},{0.5,0,0.975,-0.72},{0.755,-0.21,-0.345,0.3},{0.4938,0,-0.9939,0.7201},{0.22,0.1725,0.495,-0.3875}}};
// array<array<float, 4>, 7> y_traj_coeffs = {{{0.0508,0,0.1476,-0.0984},{0.1,0,0.37,0.02},{0.49,1.2,-3.57,1.98},{0.1,0,-0.18,0.57},{0.49,1.35,-3.87,2.13},{0.1,0,0.12,0.27},{0.49,1.05,-3.27,1.83},}};
// bool do_loop = false;
// ==================================
// Four-leaf clover
float traj_durations[] = {0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2};
array<array<float, 4>, 9> x_traj_coeffs = {{{0.0699,0,0.8903,-0.4602},{0.5,0.4,-0.2,-0.2},{0.5,-0.6,0.8,-0.2},{0.5,0.4,-0.2,-0.2},{0.5,-0.6,0.8,-0.2},{0.5,0.4,0.31,-0.34},{0.87,0,-0.71,0.34},{0.5,-0.4,-0.34,0.36},{0.12,0,0.74,-0.36}}};
array<array<float, 4>, 9> y_traj_coeffs = {{{0.0508,0,0.9476,-0.4984},{0.5,0.4,0.31,-0.34},{0.87,0,-0.71,0.34},{0.5,-0.4,-0.31,0.34},{0.13,0,0.71,-0.34},{0.5,0.4,-0.2,-0.2},{0.5,-0.6,0.8,-0.2},{0.5,0.4,-0.2,-0.2},{0.5,-0.6,0.8,-0.2}}};
bool do_loop = true;
// ==================================
// Four-leaf Slow-ver
// float traj_durations[] = {0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6};
// array<array<float, 4>, 9> x_traj_coeffs = {{{0.0699,0,0.9903,-0.5602},{0.5,0.3,0,-0.3},{0.5,-0.6,0.78,-0.18},{0.5,0.42,-0.24,-0.18},{0.5,-0.6,0.78,-0.18},{0.5,0.42,0.27,-0.32},{0.87,0,-0.69,0.32},{0.5,-0.42,-0.3,0.34},{0.12,0,0.72,-0.34}}};
// array<array<float, 4>, 9> y_traj_coeffs = {{{0.0508,0,1.0476,-0.5984},{0.5,0.3,0.51,-0.44},{0.87,0,-0.69,0.32},{0.5,-0.42,-0.27,0.32},{0.13,0,0.69,-0.32},{0.5,0.42,-0.24,-0.18},{0.5,-0.6,0.78,-0.18},{0.5,0.42,-0.24,-0.18},{0.5,-0.6,0.78,-0.18}}};
// bool do_loop = true;

int path_section_num = -1;
// ==================================

array<float,2> prev_pos = {{0,0}};
float t;
float tf;
float path_start_time;

float traj_duration = -1; //Initialize to non-zero because it gets used to find the initial velocity in generate_path()
float xf;
float yf;
float vxf;
float vyf;
float xf_prev, yf_prev;

float ff[2][2][4] ={{{3.790419e-06,6.066717e-03,6.925599e-02,2.323943e-18},
{-1.501957e-06,-2.396739e-03, 6.811798e-17,-6.262513e-18}}, 
{ {-1.501957e-06,-2.396739e-03, 8.406599e-17, 2.020639e-18},
{3.790419e-06, 6.052580e-03, 4.669578e-02,-4.398398e-17}}};


array<float,4> cx = {{0,0,0,0}};
array<float,4> cy = {{0,0,0,0}};

bool DISABLE_MOTORS = true;

/*
Read left and right motor angles from the encoders.
Angles are returned in degrees.
*/
array<float, 2> read_motor_angles() {
    array<float, 2> angles;
    uint16_t serial_response; // incoming byte from the SPI
    int chips[2] = {ENC_CHIP_SELECT_LEFT, ENC_CHIP_SELECT_RIGHT};

    digitalWrite(chips[0], LOW);
    serial_response = SPI.transfer16(0x3FFF);
    digitalWrite(chips[0], HIGH);
    angles[0] = (serial_response & 0b0011111111111111) * 360.0 / 16384;

    if(angles[0] - previous_left_angle > ROLLOVER_ANGLE_DEGS) {
        left_revolutions += 1;
    } else if(previous_left_angle - angles[0] > ROLLOVER_ANGLE_DEGS) {
        left_revolutions -= 1;
    }

    previous_left_angle = angles[0];

    digitalWrite(chips[1], LOW);
    serial_response = SPI.transfer16(0x3FFF);
    digitalWrite(chips[1], HIGH);
    angles[1] = (serial_response & 0b0011111111111111) * 360.0 / 16384;

    if(angles[1] - previous_right_angle > ROLLOVER_ANGLE_DEGS) {
        right_revolutions += 1;
    } else if (previous_right_angle - angles[1] > ROLLOVER_ANGLE_DEGS) {
        right_revolutions -= 1;
    }

    previous_right_angle = angles[1];

    angles[0] = -angles[0] + 360.0 * left_revolutions - left_offset;
    angles[1] = -angles[1] + 360.0 * right_revolutions - right_offset;
    
    return angles;
}

/*
Takes motor angles in degrees
and converts them into cartesian position of the mallet in meters
*/
array<float,2> theta_to_xy(float theta_l, float theta_r) {
    float x = (theta_l + theta_r) * PULLEY_RADIUS * PI / 360;
    float y = (theta_l - theta_r) * PULLEY_RADIUS * PI / 360;

    return {{x, y}};
}

/*
Takes cartesian position of the mallet (x, y) in meters and converts
it into motor angles in degrees
*/
array<float,2> xy_to_theta(float x, float y) {
    float theta_l = (x + y) / PULLEY_RADIUS * 360 / (2*PI);
    float theta_r = (x - y) / PULLEY_RADIUS * 360 / (2*PI);

    return {{theta_l, theta_r}};
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
float pid(float error, float accumulated_error, float previous_error, double dt) {
    float error_derivative = (error - previous_error) / dt;

    return error * kp + accumulated_error * ki + error_derivative * kd;
}

/*
Return the sum of the feed forward terms. This function determines what the feed forward controller
contributes to the PWM at a given time. The ff matrix transforms the [theta_l,theta_r] vector, with 
its derivatives, into [voltage_l, voltage_r].
*/
array<float,2> feed_forward(float u) {
    float power_2 = u*u;
    float power_3 = power_2*u;

    float traj_2 = traj_duration * traj_duration;
    float traj_3 = traj_2 * traj_duration;

    float x = cx[0] + cx[1]*u + cx[2]*power_2 + cx[3]*power_3;
    float y = cy[0] + cy[1]*u + cy[2]*power_2 + cy[3]*power_3;
    array<float, 2> thetas = xy_to_theta(x*PI/180, y*PI/180);

    float x_vel = cx[1] + 2*cx[2]*u + 3*cx[3]*power_2;
    float y_vel = cy[1] + 2*cy[2]*u + 3*cy[3]*power_2;
    array<float, 2> theta_vel = xy_to_theta(x_vel*PI/180/traj_duration, y_vel*PI/180/traj_duration);

    float x_accel = 2*cx[2] + 6*cx[3]*u;
    float y_accel = 2*cy[2] + 6*cy[3]*u;
    array<float, 2> theta_accel = xy_to_theta(x_accel*PI/180/traj_2, y_accel*PI/180/traj_2);

    float x_jerk = 6*cx[3];
    float y_jerk = 6*cy[3];
    array<float, 2> theta_jerk = xy_to_theta(x_jerk*PI/180/traj_3, y_jerk*PI/180/traj_3);


    //The next 4 lines take the feed forward gains from the simulink model, which are stored in the ff variable as a 2x2x4 matrix, and multiply
    //by the corresponding x position, velocity, accel, and jerk which are calculated above.
    float left_feed_forward = 100/24*(ff[0][0][3]*thetas[0] + ff[0][0][2]*theta_vel[0] + ff[0][0][1]*theta_accel[0]+ff[0][0][0]*theta_jerk[0]);
    left_feed_forward += 100/24*(ff[0][1][3]*thetas[1] + ff[0][1][2]*theta_vel[1] + ff[0][1][1]*theta_accel[1]+ff[0][1][0]*theta_jerk[1]);
    float right_feed_forward = 100/24*(ff[1][0][3]*thetas[0] + ff[1][0][2]*theta_vel[0] + ff[1][0][1]*theta_accel[0]+ff[1][0][0]*theta_jerk[0]);
    right_feed_forward += 100/24*(ff[1][1][3]*thetas[1] + ff[1][1][2]*theta_vel[1] + ff[1][1][1]*theta_accel[1]+ff[1][1][0]*theta_jerk[1]);
    return {{left_feed_forward, right_feed_forward}};
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
    // set_motor_pwms(-x_speed, 0);
    // delay(500);
    // set_motor_pwms(0, 0);
    delay(500);

    //Offsets to set the (0,0) in (x,y) coordinates to the bottom left corner of the table
    array<float,2> global_theta_offsets = xy_to_theta(X_OFFSET, Y_OFFSET);

    left_revolutions = 0;
    right_revolutions = 0;
    left_offset = read_motor_angles()[0] - global_theta_offsets[0];
    right_offset = read_motor_angles()[1] - global_theta_offsets[1];
    Serial.println("Fully Homed");
}

void command_motors(float x_pos, float y_pos, double current_time, double previous_time) {
    array<float, 2> target_angles = xy_to_theta(x_pos, y_pos);

    array<float,2> actual_angles = read_motor_angles();

    //Add accumulated error update

    float left_error = target_angles[0] - actual_angles[0];
    float right_error = target_angles[1] - actual_angles[1];

    float left_pid = pid(left_error, left_accumulated_error, left_previous_error, current_time - previous_time);
    float right_pid = pid(right_error, right_accumulated_error, right_previous_error, current_time - previous_time);

    left_previous_error = left_error;
    right_previous_error = right_error;

    left_accumulated_error += left_error;
    right_accumulated_error += right_error;

    array<float, 2> feed_forward_values = feed_forward((current_time-path_start_time)/traj_duration);
    
    float left_feed_forward = fmin(fmax(-max_pwm, feed_forward_values[0]), max_pwm);
    float right_feed_forward = fmin(fmax(-max_pwm, feed_forward_values[1]), max_pwm);

    float left_pwm = fmin(fmax(-max_pwm, left_pid + left_feed_forward), max_pwm);
    float right_pwm = fmin(fmax(-max_pwm, right_pid + right_feed_forward), max_pwm);

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
    Serial.print(left_pid);
    Serial.print(",");
    Serial.print(right_pid);
    Serial.print(",");
    Serial.print(feed_forward_values[0]);
    Serial.print(",");
    Serial.println(feed_forward_values[1]);

    set_motor_pwms(left_pwm, right_pwm);
}

/*
Get the target position, velocity, and arrival time from the high-level controller
*/
void get_target_from_hlc() {
    path_section_num++;
    
    if (path_section_num >= sizeof(traj_durations) / sizeof(int)) {
        path_section_num = 1;
        if (!do_loop) {
            set_motor_pwms(0,0);
            exit(0);
        }
    }

    cx = x_traj_coeffs[path_section_num];
    cy = y_traj_coeffs[path_section_num];
    traj_duration = traj_durations[path_section_num];
    tf = t + traj_durations[path_section_num];
}

void setup() {
    Serial.begin(460800);
    SPI.beginTransaction(SPISettings(460800, MSBFIRST, SPI_MODE1));
    pinMode(LED_BUILTIN, OUTPUT); // set this pin as output
    
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

    read_motor_angles(); //Need a dummy call to get the previous angle variable set properly

    home_table(15, 6, 10);

    Serial.println("BEGIN CSV");
    Serial.println("Time(ms),X_Target(cm),Y_Target(cm),Left_Error(deg),Right_Error(deg),Left_PID,Right_PID,Left_Feed_Forward,Right_Feed_Forward,X_Puck(cm),Y_Puck(cm)");

    previous_time = 0;
    start_time = micros();
    tf = 0;
}

void loop() {
    t = (micros() - start_time) / 1000000.0;

    if (t > tf) {
        path_start_time = t;
        get_target_from_hlc();
    }
    
    float u = (t-path_start_time) / traj_duration;
    float power_2 = u*u;
    float power_3 = power_2*u;

    float x_pos = cx[0] + cx[1]*u + cx[2]*power_2 + cx[3]*power_3;
    float y_pos = cy[0] + cy[1]*u + cy[2]*power_2 + cy[3]*power_3;

    if(x_pos < X_MIN) {
        x_pos = X_MIN;
    } else if(x_pos > X_MAX) {
        x_pos = X_MAX;
    }

    if(y_pos < Y_MIN) {
        y_pos = Y_MIN;
    } else if(y_pos > Y_MAX) {
        y_pos = Y_MAX;
    }

    command_motors(x_pos, y_pos, t, previous_time);

    previous_time = t;
    xf_prev = xf;
    yf_prev = yf;
}

