#include <Arduino.h>
#include <SPI.h>
#include <cmath>
#include <ArduinoEigen.h>
// #include "ArduinoEigen/ArduinoEigenCommon.h"

// SWITCHED LEFT AND RIGHT
#define ENC_CHIP_SELECT_LEFT PB7
#define ENC_CHIP_SELECT_RIGHT PB6
#define LEFT_MOTOR_PWM_PIN PA_8
#define RIGHT_MOTOR_PWM_PIN PA_10
#define LEFT_MOTOR_DIR_PIN PB15
#define RIGHT_MOTOR_DIR_PIN PA9

// // PREVIOUS DIRECTIONS
// #define ENC_CHIP_SELECT_LEFT PB6 // prev PA4
// #define ENC_CHIP_SELECT_RIGHT PB7 // prev PB5
// #define LEFT_MOTOR_PWM_PIN PA_8 // PA_10 prev PB_7
// #define RIGHT_MOTOR_PWM_PIN PA_10 // PA_8 prev PB_8
// #define LEFT_MOTOR_DIR_PIN PB15 // PA9 prev PB3
// #define RIGHT_MOTOR_DIR_PIN PA9 // PB15 prev PB4

#define SPI_MISO_PIN PB4
#define SPI_SCLK_PIN PB3
#define ENABLE_MOTOR_PIN PB11
#define DUTY_CYCLE_CONVERSION 1024 // Accepted duty cycle values are 0-1024
#define PWM_FREQ_HZ 10000
#define ROLLOVER_ANGLE_DEGS 180
#define PULLEY_RADIUS 0.035 //meters
#define X_OFFSET 0.0699 //meters
#define Y_OFFSET 0.0508 //meters
#define X_MIN X_OFFSET //meters
#define X_MAX 1-X_OFFSET //meters
#define Y_MIN Y_OFFSET //meters
#define Y_MAX 1-Y_OFFSET //meters

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
float max_pwm = 20;

std::array<float,2> prev_pos = {{0,0}};
float t;
float tf;
float path_start_time;

float traj_duration = -1; //Initialize to non-zero because it gets used to find the initial velocity in generate_path()
float xf;
float yf;
float vxf;
float vyf;
float xf_prev, yf_prev;
float x_puck, y_puck;

// OLD FEEDFORWARD PARAMETERS
// float ff[2][2][4] ={{{3.790419e-06,6.066717e-03,6.925599e-02,2.323943e-18},
// {-1.501957e-06,-2.396739e-03, 6.811798e-17,-6.262513e-18}}, 
// { {-1.501957e-06,-2.396739e-03, 8.406599e-17, 2.020639e-18},
// {3.790419e-06, 6.052580e-03, 4.669578e-02,-4.398398e-17}}};

float ff[2][2][4] = {{{4.474910e-06, 7.149068e-03, 5.342087e-02,-9.214972e-17,},
{-1.964001e-06,-3.134045e-03, 7.759782e-17, 9.697753e-17,} }, 
{ {-1.964001e-06,-3.134045e-03, 4.969833e-17, 2.967669e-18,},
{4.474910e-06,7.149377e-03,5.391504e-02,6.167752e-17,} } };


std::array<float,4> cx = {{0,0,0,0}};
std::array<float,4> cy = {{0,0,0,0}};

bool DISABLE_MOTORS = true;

/*
Read left and right motor angles from the encoders.
Angles are returned in degrees.
*/
std::array<float, 2> read_motor_angles() {
    std::array<float, 2> angles;
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
std::array<float,2> theta_to_xy(float theta_l, float theta_r) {
    float x = (theta_l + theta_r) * PULLEY_RADIUS * PI / 360;
    float y = (theta_l - theta_r) * PULLEY_RADIUS * PI / 360;

    return {{x, y}};
}

/*
Takes cartesian position of the mallet (x, y) in meters and converts
it into motor angles in degrees
*/
std::array<float,2> xy_to_theta(float x, float y) {
    float theta_l = (x + y) / PULLEY_RADIUS * 360 / (2*PI);
    float theta_r = (x - y) / PULLEY_RADIUS * 360 / (2*PI);
    
    return {{theta_l, theta_r}};
}


/*
Command a motor velocity to the specified motor.
input values should be in the range [-100,100].
*/
void set_motor_pwms(float left, float right) {
    float enable_motors;
    enable_motors = digitalRead(ENABLE_MOTOR_PIN);
    if (enable_motors == LOW) {
        left = 0;
        right = 0;
    }
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
std::array<float,2> feed_forward(float u) {
    float power_2 = u*u;
    float power_3 = power_2*u;

    float traj_2 = traj_duration * traj_duration;
    float traj_3 = traj_2 * traj_duration;

    float x = cx[0] + cx[1]*u + cx[2]*power_2 + cx[3]*power_3;
    float y = cy[0] + cy[1]*u + cy[2]*power_2 + cy[3]*power_3;
    std::array<float, 2> thetas = xy_to_theta(x*PI/180, y*PI/180);

    float x_vel = cx[1] + 2*cx[2]*u + 3*cx[3]*power_2;
    float y_vel = cy[1] + 2*cy[2]*u + 3*cy[3]*power_2;
    std::array<float, 2> theta_vel = xy_to_theta(x_vel*PI/180/traj_duration, y_vel*PI/180/traj_duration);

    float x_accel = 2*cx[2] + 6*cx[3]*u;
    float y_accel = 2*cy[2] + 6*cy[3]*u;
    std::array<float, 2> theta_accel = xy_to_theta(x_accel*PI/180/traj_2, y_accel*PI/180/traj_2);

    float x_jerk = 6*cx[3];
    float y_jerk = 6*cy[3];
    std::array<float, 2> theta_jerk = xy_to_theta(x_jerk*PI/180/traj_3, y_jerk*PI/180/traj_3);


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

    //Offsets to set the (0,0) in (x,y) coordinates to the bottom left corner of the table
    std::array<float,2> global_theta_offsets = xy_to_theta(X_OFFSET, Y_OFFSET);

    left_revolutions = 0;
    right_revolutions = 0;
    left_offset = read_motor_angles()[0] - global_theta_offsets[0];
    right_offset = read_motor_angles()[1] - global_theta_offsets[1];
    Serial2.println("Fully Homed");
}

void command_motors(float x_pos, float y_pos, double current_time, double previous_time) {
    std::array<float, 2> target_angles = xy_to_theta(x_pos, y_pos);

    std::array<float,2> actual_angles = read_motor_angles();
    std::array<float, 2> actual_pos = theta_to_xy(actual_angles[0], actual_angles[1]);
    //Add accumulated error update

    float left_error = target_angles[0] - actual_angles[0];
    float right_error = target_angles[1] - actual_angles[1];

    float left_pid = pid(left_error, left_accumulated_error, left_previous_error, current_time - previous_time);
    float right_pid = pid(right_error, right_accumulated_error, right_previous_error, current_time - previous_time);

    left_previous_error = left_error;
    right_previous_error = right_error;

    left_accumulated_error += left_error;
    right_accumulated_error += right_error;

    std::array<float, 2> feed_forward_values = feed_forward((current_time-path_start_time)/traj_duration);
    
    float left_feed_forward = fmin(fmax(-max_pwm, feed_forward_values[0]), max_pwm);
    float right_feed_forward = fmin(fmax(-max_pwm, feed_forward_values[1]), max_pwm);

    float left_pwm = fmin(fmax(-max_pwm, left_pid + left_feed_forward), max_pwm);
    float right_pwm = fmin(fmax(-max_pwm, right_pid + right_feed_forward), max_pwm);

    Serial2.print(current_time*1000);
    Serial2.print(",");
    Serial2.print(x_pos*100);
    Serial2.print(",");
    Serial2.print(y_pos*100);
    Serial2.print(",");
    Serial2.print(actual_pos[0]*100);
    Serial2.print(",");
    Serial2.print(actual_pos[1]*100);
    Serial2.print(",");
    Serial2.print(actual_angles[0]);
    Serial2.print(",");
    Serial2.print(actual_angles[1]);
    Serial2.print(",");
    Serial2.print(left_error);
    Serial2.print(",");
    Serial2.print(right_error);
    Serial2.print(",");
    Serial2.print(left_pid);
    Serial2.print(",");
    Serial2.print(right_pid);
    Serial2.print(",");
    Serial2.print(feed_forward_values[0]);
    Serial2.print(",");
    Serial2.print(feed_forward_values[1]);
    Serial2.print(",");
    Serial2.print(left_pwm);
    Serial2.print(",");
    Serial2.println(right_pwm);

    set_motor_pwms(left_pwm, right_pwm);
}

/*
Generate the path and its coefficients for the given HLC target. Calculations from here:
https://en.wikipedia.org/wiki/B%C3%A9zier_curve
*/

void generate_path(float x_puck, float y_puck, float vf_magnitude, float path_time) {
    float table_length = 2;
    float x_goal = 0.52;
    float mallet_plus_puck_radius = 0.05 + 0.03175; 

    std::array<float,2> current_angles = read_motor_angles();
    float x_initial = theta_to_xy(current_angles[0], current_angles[1])[0];
    float y_initial = theta_to_xy(current_angles[0], current_angles[1])[1];

    float u = (t-path_start_time) / traj_duration;

    float vx_initial;
    float vy_initial;

    if ((t + 0.01 - path_start_time) / traj_duration > 1.0) {
        float power_2 = u*u;

        vx_initial = (cx[1] + 2*cx[2]*u + 3*cx[3]*power_2)/traj_duration;
        vy_initial = (cy[1] + 2*cy[2]*u + 3*cy[3]*power_2)/traj_duration;
    } else {
        vx_initial = 0;
        vy_initial = 0;
    }
    float vf_x = x_goal - x_puck;
    float vf_y = table_length - y_puck;

    float original_vf_norm = sqrt(vf_x*vf_x + vf_y*vf_y);

    //Back off from the "intecept point" because the collision occurs when the mallet and puck
    //are radius_puck + radius_mallet apart from each other.
    float intercept_point_x = x_puck - vf_x/original_vf_norm*mallet_plus_puck_radius;
    float intercept_point_y = y_puck - vf_y/original_vf_norm*mallet_plus_puck_radius;

    //Scale the final velocity at the intercept
    float v_3_x = vf_x / original_vf_norm * vf_magnitude;
    float v_3_y = vf_y / original_vf_norm * vf_magnitude;

    //Find control point locations
    float q1_x = x_initial + vx_initial * path_time/3;
    float q1_y = y_initial + vy_initial * path_time/3;
    
    float q2_x = intercept_point_x - v_3_x * path_time / 3;
    float q2_y = intercept_point_y - v_3_y * path_time / 3;

    cx = {{x_initial, 3*q1_x-3*x_initial, 3*x_initial-6*q1_x+3*q2_x, 3*q1_x-x_initial-3*q2_x+intercept_point_x}};
    cy = {{y_initial, 3*q1_y-3*y_initial, 3*y_initial-6*q1_y+3*q2_y, 3*q1_y-y_initial-3*q2_y+intercept_point_y}};

    traj_duration = path_time;
    path_start_time = t;
    tf = path_start_time + path_time;
}

/*
Get the target position, velocity, and arrival time from the high-level controller
*/
void get_target_from_hlc() {

    // Generate coefficients for the current path
    // call generate_path() which sets global cx and cy arrays for the path
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

HardwareSerial Serial2(PA3, PA2);
void setup() {
    Serial2.begin(460800);
    SPI.setMISO(SPI_MISO_PIN);
    SPI.setSCLK(SPI_SCLK_PIN);
    SPI.beginTransaction(SPISettings(460800, MSBFIRST, SPI_MODE1));
    pinMode(LED_BUILTIN, OUTPUT); // set this pin as output
    
    pinMode(ENC_CHIP_SELECT_LEFT, OUTPUT);
    pinMode(ENC_CHIP_SELECT_RIGHT, OUTPUT);
    pinMode(ENABLE_MOTOR_PIN, INPUT);
    
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

    home_table(9, 5, 10);

    while (!Serial.available()) {
        // wait for serial to become available
    }
    delay(500);

    Serial2.println("BEGIN CSV");
    Serial2.println("Time(ms),X_Target(cm),Y_Target(cm),X_Puck(cm),Y_Puck(cm),Left_Angle(deg),Right_Angle(deg),Left_Error(deg),Right_Error(deg),Left_PID,Right_PID,Left_Feed_Forward,Right_Feed_Forward,Left_PWM, Right_PWM");

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