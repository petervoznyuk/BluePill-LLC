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
#define X_MIN 0.0 //meters
#define X_MAX 0.80 //meters
#define Y_MIN 0.0 //meters
#define Y_MAX 0.85 //meters

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

array<float,2> prev_pos = {{0,0}};
float t;
float tf;
float path_start_time;

float traj_duration;
float xf;
float yf;
float vxf;
float vyf;
float xf_prev, yf_prev;

float ff[2][2][4] = {{{3.575853e-06, 5.718267e-03, 5.958733e-02, -6.040430e-17},
{-1.716522e-06, -2.739130e-03, -5.986824e-17, 0}}, 
{{-1.716522e-06, -2.739130e-03, -4.501154e-17, -1.848923e-31},
{3.575853e-06, 5.710188e-03, 4.669578e-02, 5.416168e-18}}};


array<float,4> cx;
array<float,4> cy;


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
    set_motor_pwms(-x_speed, 0);
    delay(500);
    set_motor_pwms(0, 0);
    delay(500);

    left_revolutions = 0;
    right_revolutions = 0;
    left_offset = read_motor_angles()[0];
    right_offset = read_motor_angles()[1];
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

    float left_pwm = fmin(fmax(-max_pwm, left_pid + feed_forward_values[0]), max_pwm);
    float right_pwm = fmin(fmax(-max_pwm, right_pid + feed_forward_values[1]), max_pwm);

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

    // set_motor_pwms(left_pwm, right_pwm);
}

/*
Get the target position, velocity, and arrival time from the high-level controller
*/
void get_target_from_hlc() {
    // TODO: In future, read the serial interface and update target position and velocity if information is available.
    // For now, use fixed time intervals instead. Add more "else if" conditions to add path segments.

    traj_duration = 1;

    if (t < traj_duration) {
        cx = {0, 0, 1.29000000000000, -0.860000000000000};
        cy = {0, 0, 0.300000000000000, -0.200000000000000};
    } else if (t < 2*traj_duration) {
        cx = {0.430000000000000, 0, -0.738000000000000, 0.548000000000000};
        cy = {0.100000000000000, 0, -0.210000000000000, 0.560000000000000};
    } 
    else if (t < 3*traj_duration) {
        cx = {0.240000000000000, -0.132000000000000, 0.834000000000000, -0.512000000000000};
        cy = {0.450000000000000, -0.990000000000000, 0.930000000000000, -0.290000000000000};
    }
    else if (t < 4*traj_duration) {
        cx = {0.430000000000000, 0, 0, 5.55111512312578e-17};
        cy = {0.100000000000000, 0, 0.0900000000000000, 0.260000000000000};
    }
        else if (t < 5*traj_duration) {
        cx = {0.430000000000000, 0, 0, 5.55111512312578e-17};
        cy = {0.450000000000000, 2.40000000000000, -5.85000000000000, 3.10000000000000};
    }
        else if (t < 6*traj_duration) {
        cx = {0.430000000000000, 0, 0.753000000000000, -0.558000000000000};
        cy = {0.100000000000000, 0, -0.210000000000000, 0.560000000000000};
    }
        else if (t < 7*traj_duration) {
        cx = {0.625000000000000, 0.132000000000000, -0.849000000000000, 0.522000000000000};
        cy = {0.450000000000000, -0.990000000000000, 0.930000000000000, -0.290000000000000};
    } else {
        set_motor_pwms(0, 0);
        exit(0);
    }
    tf = t + traj_duration;
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

    read_motor_angles(); //Need a dummy call to get the previous angle variable set properly

    home_table(9, 6, 10);

    Serial.println("BEGIN CSV");
    Serial.println("Time(ms),X_Target(cm),Y_Target(cm),Left_Error(deg),Right_Error(deg),Left_PID,Right_PID,Left_Feed_Forward,Right_Feed_Forward");

    previous_time = 0;
    start_time = micros();
    tf = 0;
}

void loop() {

    t = (micros() - start_time) / 1000000.0;

    // Update xf, yf, vxf, vyf, and traj_duration

    if (t > tf) {
        get_target_from_hlc();
        path_start_time = t;
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


