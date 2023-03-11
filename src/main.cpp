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
float kd = 0.0002;
// float kd = 0.0;
float max_pwm = 95;

array<float,2> prev_pos = {{0,0}};
float t;
float tf;

float traj_duration;
float xf;
float yf;
float vxf;
float vyf;
float xf_prev, yf_prev;

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

    accumulated_error += error;

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
    float final_velocity_magnitude = sqrt(vx*vx + vy*vy);
    float reverse_x_component = -vx / final_velocity_magnitude * straight_length;
    float reverse_y_component = -vy / final_velocity_magnitude * straight_length;
    
    float intermediate_x = x + reverse_x_component;
    float intermediate_y = y + reverse_y_component;

    float intermediate_t = final_time - straight_length / final_velocity_magnitude;

    return {{intermediate_x, intermediate_y, intermediate_t}};
}

array<float,12> get_trajectory_coeffs(float t0, float x0, float y0, float vx0, float vy0, float tf, float xf, float yf, float vxf, float vyf, float d) {
    array<float,3> intermediate_params = get_intermediate_point(xf, yf, vxf, vyf, tf, d);

    float x1 = intermediate_params[0];
    float y1 = intermediate_params[1];
    float t1 = intermediate_params[2];
    
    Eigen::Matrix<float,6,6> mat;

    float t0_3 = t0*t0*t0;
    float t0_4 = t0_3*t0;
    float t0_5 = t0_4*t0;

    float t1_3 = t1*t1*t1;
    float t1_4 = t1_3*t1;
    float t1_5 = t1_4*t1;
    
    float tf_3 = tf*tf*tf;
    float tf_4 = tf_3*tf;
    float tf_5 = tf_4*tf;

    mat << 1, t0, t0*t0, t0_3, t0_4, t0_5,
        0, 1, 2*t0, 3*t0*t0, 4*t0_3, 5*t0_4,
        1, t1, t1*t1, t1_3, t1_4, t1_5,
        0, 1, 2*t1, 3*t1*t1, 4*t1_3, 5*t1_4,
        1, tf, tf*tf, tf_3, tf_4, tf_5,
        0, 1, 2*tf, 3*tf*tf, 4*tf_3, 5*tf_4;

    Eigen::ColPivHouseholderQR<Eigen::Matrix<float, 6, 6, 0, 6, 6>> qr = mat.colPivHouseholderQr();

    Eigen::Vector<float,6> bx(x0, vx0, x1, vxf, xf, vxf);
    Eigen::Vector<float,6> ax = qr.solve(bx);

    Eigen::Vector<float,6> by(y0, vy0, y1, vyf, yf, vyf);
    Eigen::Vector<float,6> ay = qr.solve(by);

    return {{ax[0], ax[1], ax[2], ax[3], ax[4], ax[5], ay[0], ay[1], ay[2], ay[3], ay[4], ay[5]}};
}

/*
Get the target position, velocity, and arrival time from the high-level controller
*/
void get_target_from_hlc() {
    // TODO: In future, read the serial interface and update target position and velocity if information is available.
    // For now, use fixed time intervals instead. Add more "else if" conditions to add path segments.

    float first_traj_duration = 0.3;
    float second_traj_duration = 0.35;
    float third_traj_duration = 0.5;

    if (t < first_traj_duration) {
        traj_duration = first_traj_duration;
        xf = 0.412;
        yf = 0.3;
        vxf = 0.0;
        vyf = 3.0;
    } else if (t < first_traj_duration + second_traj_duration) {
        traj_duration = second_traj_duration;
        xf = 0.412;
        yf = 0.1;
        vxf = 0.0;
        vyf = -0.01;
    } 
    // else if (t < first_traj_duration + second_traj_duration + third_traj_duration) {
    //     traj_duration = third_traj_duration;
    //     xf = 0.412;
    //     yf = 0.1;
    //     vxf = 0.0;
    //     vyf = -0.01;
    // }
}

/*
Modifies the variables cx and cy to contain the polynomial coefficients
for the new path the mallet should follow.
*/
void update_trajectory_coeffs(float t) {
    float power_2 = t*t;
    float power_3 = power_2*t;
    float power_4 = power_3*t;
    float power_5 = power_4*t;

    // Alt 1: Estimate mallet velocity based on current/previous positions and time difference
    // array<float,2> start_angles = read_motor_angles();
    // array<float,2> current_pos = theta_to_xy(start_angles[0], start_angles[1]);
    // float x0 = current_pos[0];
    // float y0 = current_pos[1];
    // float vx0 = (current_pos[0] - prev_pos[0]) / (current_time - previous_time);
    // float vy0 = (current_pos[1] - prev_pos[1]) / (current_time - previous_time);
    
    // Alt 2: Use desired mallet velocity from the derivative of the target polynomial
    float x0 = cx[0] + cx[1]*t + cx[2]*power_2 + cx[3]*power_3 + cx[4]*power_4 + cx[5]*power_5;
    float y0 = cy[0] + cy[1]*t + cy[2]*power_2 + cy[3]*power_3 + cy[4]*power_4 + cy[5]*power_5;

    float vx0 = cx[1] + cx[2]*2*t + cx[3]*3*power_2 + cx[4]*4*power_3 + cx[5]*5*power_4;
    float vy0 = cy[1] + cy[2]*2*t + cy[3]*3*power_2 + cy[4]*4*power_3 + cy[5]*5*power_4;

    tf = t + traj_duration;

    array<float,12> coeffs = get_trajectory_coeffs(t, x0, y0, vx0, vy0, tf, xf, yf, vxf, vyf, 0.05);

    cx = {{coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5]}};
    cy = {{coeffs[6], coeffs[7], coeffs[8], coeffs[9], coeffs[10], coeffs[11]}};
}

bool check_path_bounds(array<float,6> x_coeffs, array<float,6> y_coeffs, float t0, float tf) {
    int num_points = 30;
    float x_pos;
    float y_pos;

    float delta_t = (tf - t0) / num_points;

    for(int i=0;i<num_points;i++) {
        float t = delta_t*i + t0;

        float power_2 = t*t;
        float power_3 = power_2*t;
        float power_4 = power_3*t;
        float power_5 = power_4*t;

        x_pos = ceil((x_coeffs[0] + x_coeffs[1]*t + x_coeffs[2]*power_2 + x_coeffs[3]*power_3 + x_coeffs[4]*power_4 + x_coeffs[5]*power_5)*1000.0)/1000.0;
        y_pos = ceil((y_coeffs[0] + y_coeffs[1]*t + y_coeffs[2]*power_2 + y_coeffs[3]*power_3 + y_coeffs[4]*power_4 + y_coeffs[5]*power_5)*1000.0)/1000.0;

        if(x_pos < X_MIN || x_pos > X_MAX) {
            Serial.println("X Bound Violated");

            for(int i=0;i<num_points;i++) {
                float t = delta_t*i + t0;
                float power_2 = t*t;
                float power_3 = power_2*t;
                float power_4 = power_3*t;
                float power_5 = power_4*t;
                Serial.println(ceil((x_coeffs[0] + x_coeffs[1]*t + x_coeffs[2]*power_2 + x_coeffs[3]*power_3 + x_coeffs[4]*power_4 + x_coeffs[5]*power_5)*1000.0)/1000.0);
            }
            return false;
        }
        if(y_pos < Y_MIN || y_pos > Y_MAX) {
            Serial.println("Y Bound Violated");

            for(int i=0;i<num_points;i++) {
                float t = delta_t*i + t0;
                float power_2 = t*t;
                float power_3 = power_2*t;
                float power_4 = power_3*t;
                float power_5 = power_4*t;
                Serial.println(ceil((y_coeffs[0] + y_coeffs[1]*t + y_coeffs[2]*power_2 + y_coeffs[3]*power_3 + y_coeffs[4]*power_4 + y_coeffs[5]*power_5)*1000.0)/1000.0);
            }
            return false;
        }
    }
    return true;
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

    home_table(12, 12, 10);

    Serial.println("BEGIN CSV");
    Serial.println("Time(ms),X_Target(cm),Y_Target(cm),Left_Error(deg),Right_Error(deg),Left_PWM,Right_PWM");

    previous_time = 0;
    start_time = micros();
}

void loop() {
    float x_pos;
    float y_pos;

    t = (micros() - start_time) / 1000000.0;

    // Update xf, yf, vxf, vyf, and traj_duration
    get_target_from_hlc();

    // If HLC gave us a new target, update the path
    if (xf != xf_prev || yf != yf_prev) {
        update_trajectory_coeffs(t);
    }

    if (t < (tf + 0.01)) {
        float power_2 = t*t;
        float power_3 = power_2*t;
        float power_4 = power_3*t;
        float power_5 = power_4*t;

        float x_pos = cx[0] + cx[1]*t + cx[2]*power_2 + cx[3]*power_3 + cx[4]*power_4 + cx[5]*power_5;
        float y_pos = cy[0] + cy[1]*t + cy[2]*power_2 + cy[3]*power_3 + cy[4]*power_4 + cy[5]*power_5;

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
    } else {
        set_motor_pwms(0, 0);
        exit(0);
    }

    previous_time = t;
    xf_prev = xf;
    yf_prev = yf;

    // Needed if we use the actual initial velocity instead of the ideal velocity
    // array<float,2> angles = read_motor_angles();
    // array<float,2> prev_pos = theta_to_xy(angles[0], angles[1]);
}