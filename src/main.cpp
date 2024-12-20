#include <Arduino.h>
#include <ArduinoEigen.h>
#include <SPI.h>
#include <math.h>

#include <cmath>

#define ENC_CHIP_SELECT_LEFT PB7
#define ENC_CHIP_SELECT_RIGHT PB6
#define LEFT_MOTOR_PWM_PIN PA_8
#define RIGHT_MOTOR_PWM_PIN PA_10
#define LEFT_MOTOR_DIR_PIN PB15
#define RIGHT_MOTOR_DIR_PIN PA9

#define SPI_MISO_PIN PB4
#define SPI_SCLK_PIN PB3
#define ENABLE_MOTOR_PIN PB11

#define DUTY_CYCLE_CONVERSION 1024  // Accepted duty cycle values are 0-1024
#define PWM_FREQ_HZ 10000           //
#define ROLLOVER_ANGLE_DEGS 180     //
#define PULLEY_RADIUS 0.035         // meters
#define MALLET_RADIUS 0.05
#define PUCK_RADIUS 0.03175
#define MAX_PWM 70

// Coordinate Definitions
#define ICE_WIDTH 0.99                     // meters - total width of the playing surface
#define ICE_HEIGHT 1.99                    // meters - total height of the playing surface
#define X_OFFSET 0.0699                    // meters -
#define Y_OFFSET 0.0508                    // meters -
#define X_MIN X_OFFSET                     // meters - minimum x coordinate which mallet can go to
#define X_MAX ICE_WIDTH - X_OFFSET         // meters - maximum x coordinate which mallet can go to
#define Y_MIN Y_OFFSET                     // meters - minimum y coordinate which mallet can go to
#define Y_MAX (ICE_HEIGHT / 2) - (2*Y_OFFSET)  // meters - maximum y coordinate which mallet can go to

// Agent Constants
#define INTERSECTION_Y 0.2          // meters - y coordinate to intersect an approaching puck at
#define HOME_Y 0.1                  // meters - y coordinate to return to and wait for next approaching shot
#define MID_HOME_Y 0.3              // meters - y coordinate to return to and wait for next approaching shot if mid
#define FAR_HOME_Y 0.5              // meters - y coordinate to return to and wait for next approaching shot if far out
#define STATIONARY_THRESHOLD 0.02   // meters - if puck has travelled less than this distance between frames it is stationary
#define MALLET_STATIONARY_THRESHOLD 0.007 // meters - if mallet within threshold do not make new path
#define FRAMERATE 60                // FPS - Of the camera
#define DEFAULT_INTERCEPT_TIME 0.3  // s - Time to destination if destination is stationary
#define DEFAULT_INTERCEPT_SPEED 4   // m/s - Speed to hit puck at
#define LATENCY_OFFSET 0.04         // s - loop latency to offset path time
#define MIN_DEFENSE_TIME 0.08       // s - minimum path time to defend
#define MAX_DEFENSE_TIME 0.2        // s - maximum path time to defend
#define MIN_ATTACK_TIME 0.1        // s - minimum path time to attack
#define MAX_ATTACK_TIME 0.6         // s - maximum path time to attack
#define MAX_TRAVEL_DISTANCE 1.41    // m - maximum travel distance ~= sqrt(2)
#define MIN_MALLET_INIT_VELO 0       // m/s - minimum mallet velocity
#define MAX_MALLET_INIT_VELO 5      // m/s - max mallet initial velocity

// PID Controller Constants
#define KP 0.8
#define KI 0.0
#define KD 0.0175

HardwareSerial Serial2(PA3, PA2);

// Define Global Variables
int left_revolutions = 0;
int right_revolutions = 0;
float previous_left_angle = 0;
float previous_right_angle = 0;
float left_offset = 0;
float right_offset = 0;
float start_time;
float left_accumulated_error = 0;
float right_accumulated_error = 0;
float left_previous_error = 0;
float right_previous_error = 0;
double previous_time;

float t;
float tf;
float path_start_time;

float traj_duration = -1;  // Initialize to non-zero because it gets used to find the initial velocity in generate_path()
float x_puck, y_puck;

// Define variables to home when puck is missing
float puck_missing_frames = 0.0;
float PUCK_MISSING_HOME_THRESHOLD = 5; // Number of frames the puck is missing to home

float xp_prev = -1;
float yp_prev = -1;

// prev path final velocities
float v_3_xp = 0;
float v_3_yp = 0;

// // Apr 5
// float ff[2][2][4] = { { {4.531477e-06, 7.240509e-03, 5.529498e-02,-2.047470e-17,},
// {-1.889263e-06,-3.014782e-03, 9.035429e-17,-1.536679e-17,} }, 
// { {-1.889263e-06,-3.014782e-03, 7.149001e-17, 8.619992e-31,},
// {4.531477e-06,7.241515e-03,5.690094e-02,8.419528e-17,} } };

// Apr 8
// float ff[2][2][4] = { { {4.440386e-06,7.095690e-03,5.615559e-02,1.336924e-16,},
// {-1.796661e-06,-2.867012e-03, 2.259524e-16,-4.480237e-17,} }, 
// { {-1.796661e-06,-2.867012e-03, 9.129494e-18, 2.517678e-17,},
// {4.440386e-06,7.097541e-03,5.910830e-02,7.178437e-17,} } };

// Apr 9
float ff[2][2][4] = { { {4.460893e-06,7.129924e-03,5.856578e-02,1.511237e-16,},
{-1.828284e-06,-2.917475e-03, 5.766519e-17,-7.761393e-17,} }, 
{ {-1.828284e-06,-2.917475e-03,-8.880149e-17, 1.024920e-16,},
{4.460893e-06, 7.131214e-03, 6.062391e-02,-1.027947e-16,} } };

std::array<float, 4> cx = {{0, 0, 0, 0}};
std::array<float, 4> cy = {{0, 0, 0, 0}};

bool STATE_LOGGING = false;
float frame_time;
float frame_time_prev;

// Start Agent Code
enum State {
    DEFEND,
    ATTACK,
    HOME
};

enum Direction {
    APPROACHING,
    STATIONARY,
    LEAVING
};

// Function prototypes
float location_of_intersection(float x1, float y1, float x2, float y2);
float time_to_intersection(float x1, float y1, float x2, float y2);
State agent_state_selector(float xm, float ym, float x1, float y1, float x2, float y2);
float bounce_coordinate_calculator(bool left_bounce, float xi, float yi);
float get_attack_angle(float x1, float y1, float x2, float y2);
void execute_shot(float t, float x, float y, float theta, float v, State game_state);
void classical_agent(float xm, float ym, float x1, float y1, float x2, float y2);
std::array<float, 2> read_motor_angles();
std::array<float, 2> theta_to_xy(float theta_l, float theta_r);
std::array<float, 2> xy_to_theta(float x, float y);
void set_motor_pwms(float left, float right);
float pid(float error, float accumulated_error, float previous_error, double dt);
void home_table(float x_speed, float y_speed, float position_threshold);
void command_motors(float x_pos, float y_pos, double current_time, double previous_time);
void generate_path(float x_puck, float y_puck, float vf_theta, float vf_magnitude, float path_time, State game_state);
bool read_camera();
float read_float(char end);

float location_of_intersection(float x1, float y1, float x2, float y2) {
    float h = y1 - INTERSECTION_Y;
    float w = h * (x1 - x2) / (y1 - y2) - x1;
    if (int(floor(w / ICE_WIDTH)) % 2 == 0) {
        return fmod(w, ICE_WIDTH);
    } else {
        return ICE_WIDTH - fmod(w, ICE_WIDTH);
    }
}

float time_to_intersection(float x1, float y1, float x2, float y2) {
    float h = y1 - INTERSECTION_Y;
    float w = h * (x1 - x2) / (y1 - y2) - x1;
    float position_delta = sqrtf((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
    float speed = position_delta * fmin(60, 1000/(frame_time-frame_time_prev));
    return sqrt(h * h + w * w) / speed;
}

State agent_state_selector(float xm, float ym, float x1, float y1, float x2, float y2) {
    bool puck_in_our_half = y2 < (0.7 * ICE_HEIGHT) / 2;
    Direction puck_direction = LEAVING;

    // IF PUCK MISSING GO HOME
    if (puck_missing_frames > PUCK_MISSING_HOME_THRESHOLD) {
        return HOME;
    }

    // IF PUCK PAST MALLET GO HOME
    if (y2 < ym + 0.02) {
        return HOME;
    }

    if (abs(y2 - y1) < STATIONARY_THRESHOLD && abs(x2 - x1) < STATIONARY_THRESHOLD) {
        puck_direction = STATIONARY;
    } else if (y1 - y2 > 0.0) {
        puck_direction = APPROACHING;
    }

    // if (puck_direction == APPROACHING) {
    //     if (abs(y2 - y1) < 2 * STATIONARY_THRESHOLD && puck_in_our_half) {
    //         return ATTACK;
    //     }
    //     else {
    //         return DEFEND;
    //     }
    // } 
    // else if (puck_direction == STATIONARY && puck_in_our_half) {
    //     return ATTACK;
    // }

    if (puck_direction == STATIONARY && puck_in_our_half) {
        return ATTACK;
    }
    else if (puck_direction == APPROACHING) {
        return DEFEND;
    }
    else {
        return HOME;
    }
}

float bounce_coordinate_calculator(bool left_bounce, float xi, float yi) {
    if (left_bounce) {
        return (2 * xi * ICE_HEIGHT + ICE_WIDTH * yi) / (ICE_WIDTH + 2 * xi);
    } else {
        return (2 * (ICE_WIDTH - xi) * ICE_HEIGHT + ICE_WIDTH * yi) / (ICE_WIDTH + 2 * (ICE_WIDTH - xi));
    }
}

float get_attack_angle(float x1, float y1, float x2, float y2) {
    float angle_rads = atan((y1 - y2) / (x1 - x2));
    if (angle_rads >= 0) {
        return angle_rads;
    }
    else {
        return angle_rads + PI;
    }
}

void execute_shot(float t, float x, float y, float theta, float v, State game_state) {
    if (STATE_LOGGING) {
        Serial2.println("t, x_cmd, y_cmd, theta, v");
        Serial2.print(t);
        Serial2.print(",");
        Serial2.print(x);
        Serial2.print(",");
        Serial2.print(y);
        Serial2.print(",");
        Serial2.print(theta);
        Serial2.print(",");
        Serial2.print(v);
    }

    generate_path(x, y, theta, v, t, game_state);
}

void classical_agent(float xm, float ym, float x1, float y1, float x2, float y2) {
    State new_state = agent_state_selector(xm, ym, x1, y1, x2, y2);

    float time_to_destination = DEFAULT_INTERCEPT_TIME;
    float destination_x = 0;
    float destination_y = 0;
    float approach_angle = 0;
    float approach_speed = DEFAULT_INTERCEPT_SPEED;

    int shot_type = 0;
    bool left_bounce = false;
    bool straight_shot = false;
    float x_bounce = 0;
    float y_bounce = 0;
    float intersection_x = 0;

    float y_speed = (y2-y1)/ ((frame_time - frame_time_prev)/1000);
    float x_speed = (x2-x1)/ ((frame_time - frame_time_prev)/1000);
    float puck_table_width = (ICE_WIDTH - 2*PUCK_RADIUS);
    float x_final = x2;
    float total_dist = 0;
    float y_intersect_dist = 0;
    float intersect_time = 0;

    switch (new_state) {
        case HOME:
            if (STATE_LOGGING) {
                Serial2.println("HOME STATE");
            }
            // if (ym > FAR_HOME_Y + 0.05) {
            //     destination_y = FAR_HOME_Y;
            // }
            // else 
            if (ym > MID_HOME_Y + 0.05) {
                destination_y = MID_HOME_Y;
            }
            else {
                destination_y = HOME_Y;
            }

            destination_x = ICE_WIDTH / 2;

            approach_angle = atan((ym - HOME_Y) / (xm - ICE_WIDTH / 2));
            approach_speed = 0;
            break;
        case ATTACK:
            if (STATE_LOGGING) {
                Serial2.println("ATTACK STATE");
            }
            shot_type = 2;  // 0 is left, 1 is right, 2 is straight
            left_bounce = shot_type == 0;
            straight_shot = shot_type == 2;

            // time_to_destination = fmin(fmax(time_to_destination - LATENCY_OFFSET, MIN_ATTACK_TIME), MAX_ATTACK_TIME);
            total_dist = sqrt(pow(x2-xm, 2) + pow(x2-ym, 2));
            time_to_destination = fmin(fmax(total_dist/MAX_TRAVEL_DISTANCE, MIN_ATTACK_TIME), MAX_ATTACK_TIME);

            x_final = abs(x2 - PUCK_RADIUS + x_speed*(time_to_destination + LATENCY_OFFSET));

            if (int(x_final/puck_table_width) % 2 == 0) {
                x2 = fmod(x_final, puck_table_width) + PUCK_RADIUS;
            }
            else {
                x2 = puck_table_width - fmod(x_final, puck_table_width) + PUCK_RADIUS;
            }
            y2 = y2 + y_speed*(time_to_destination + LATENCY_OFFSET);

            if (straight_shot) {
                approach_angle = get_attack_angle(x2, y2, ICE_WIDTH / 2, ICE_HEIGHT);
                destination_x = x2;
                destination_y = y2;
            } else {
                y_bounce = bounce_coordinate_calculator(left_bounce, x2, y2);
                x_bounce = 0;
                if (!left_bounce) {
                    x_bounce = ICE_WIDTH;
                }
                approach_angle = get_attack_angle(x2, y2, x_bounce, y_bounce);
                destination_x = x2;
                destination_y = y2;
            }

            approach_speed = DEFAULT_INTERCEPT_SPEED * ( (1-abs((destination_x-ICE_WIDTH/2))/(ICE_WIDTH/2)) * 0.5 + 0.5 );
            approach_speed = fmin(fmax(approach_speed, 0), DEFAULT_INTERCEPT_SPEED);
            break;
        case DEFEND:
            if (STATE_LOGGING) {
                Serial2.println("DEFEND STATE");
            }
            // shot_type = 2;  // 0 is left, 1 is right, 2 is straight
            // left_bounce = shot_type == 0;
            // straight_shot = shot_type == 2;
            // intersection_x = location_of_intersection(x1, y1, x2, y2);
            // if (straight_shot) {
            //     approach_angle = get_attack_angle(intersection_x, INTERSECTION_Y, ICE_WIDTH / 2, ICE_HEIGHT);
            //     destination_x = intersection_x;
            //     destination_y = INTERSECTION_Y;
            //     time_to_destination = time_to_intersection(x1, y1, x2, y2);
            // } else {
            //     y_bounce = bounce_coordinate_calculator(left_bounce, x2, y2);
            //     x_bounce = 0;
            //     if (!left_bounce) {
            //         x_bounce = ICE_WIDTH;
            //     }
            //     approach_angle = get_attack_angle(intersection_x, INTERSECTION_Y, x_bounce, y_bounce);
            //     destination_x = intersection_x;
            //     destination_y = INTERSECTION_Y;
            //     time_to_destination = time_to_intersection(x1, y1, x2, y2);
            // }

            y_intersect_dist = abs(y2-INTERSECTION_Y);
            intersect_time = abs(y_intersect_dist/y_speed);

            time_to_destination = fmin(fmax(intersect_time, MIN_DEFENSE_TIME), MAX_DEFENSE_TIME);

            x_final = abs(x2 - PUCK_RADIUS + x_speed*(intersect_time+LATENCY_OFFSET));

            if (int(x_final/puck_table_width) % 2 == 0) {
                destination_x = fmod(x_final, puck_table_width) + PUCK_RADIUS;
            }
            else {
                destination_x = puck_table_width - fmod(x_final, puck_table_width) + PUCK_RADIUS;
            }
            destination_x = (destination_x-ICE_WIDTH/2) * fmin(time_to_destination/intersect_time, 1) + ICE_WIDTH/2;
            
            // Serial2.print(y2*100);
            // Serial2.print(",");
            // Serial2.print(y1*100);
            // Serial2.print(",");
            // Serial2.print(frame_time);
            // Serial2.print(",");
            // Serial2.print(frame_time_prev);
            // Serial2.print(",");
            // Serial2.print(y_dist);
            // Serial2.print(",");
            // Serial2.print(travel_time);
            // Serial2.print(",");
            // Serial2.println(time_to_destination);

            destination_x = fmin(fmax(destination_x, X_MIN), X_MAX);
            destination_y = INTERSECTION_Y;
            if (xm < destination_x){
                approach_angle = PI/8;
            }
            else {
                approach_angle = 7*PI/8;
            }
            approach_speed = 0;
            break;
    }
    if (abs(destination_x-xm) < MALLET_STATIONARY_THRESHOLD && abs(destination_y-ym) < MALLET_STATIONARY_THRESHOLD) {
        v_3_xp = 0;
        v_3_yp = 0;
        return;
    }
    else {
        execute_shot(time_to_destination, destination_x, destination_y, approach_angle, approach_speed, new_state);
    }
}
// End Agent Code

/*
Read left and right motor angles from the encoders.
Angles are returned in degrees.
*/
std::array<float, 2> read_motor_angles() {
    std::array<float, 2> angles;
    uint16_t serial_response;  // incoming byte from the SPI
    int chips[2] = {ENC_CHIP_SELECT_LEFT, ENC_CHIP_SELECT_RIGHT};

    digitalWrite(chips[0], LOW);
    serial_response = SPI.transfer16(0x3FFF);
    digitalWrite(chips[0], HIGH);
    angles[0] = (serial_response & 0b0011111111111111) * 360.0 / 16384;

    if (angles[0] - previous_left_angle > ROLLOVER_ANGLE_DEGS) {
        left_revolutions += 1;
    } else if (previous_left_angle - angles[0] > ROLLOVER_ANGLE_DEGS) {
        left_revolutions -= 1;
    }

    previous_left_angle = angles[0];

    digitalWrite(chips[1], LOW);
    serial_response = SPI.transfer16(0x3FFF);
    digitalWrite(chips[1], HIGH);
    angles[1] = (serial_response & 0b0011111111111111) * 360.0 / 16384;

    if (angles[1] - previous_right_angle > ROLLOVER_ANGLE_DEGS) {
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
std::array<float, 2> theta_to_xy(float theta_l, float theta_r) {
    float x = (theta_l + theta_r) * PULLEY_RADIUS * PI / 360;
    float y = (theta_l - theta_r) * PULLEY_RADIUS * PI / 360;

    return {{x, y}};
}

/*
Takes cartesian position of the mallet (x, y) in meters and converts
it into motor angles in degrees
*/
std::array<float, 2> xy_to_theta(float x, float y) {
    float theta_l = (x + y) / PULLEY_RADIUS * 360 / (2 * PI);
    float theta_r = (x - y) / PULLEY_RADIUS * 360 / (2 * PI);

    return {{theta_l, theta_r}};
}

/*
Command a motor velocity to the specified motor.
input values should be in the range [-100,100].
*/
void set_motor_pwms(float left, float right) {
    // Serial2.println("COMMANDING MOTORS");
    bool enable_motors = digitalRead(ENABLE_MOTOR_PIN);
    if (!enable_motors) {
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

    return error * KP + accumulated_error * KI + error_derivative * KD;
}

/*
Return the sum of the feed forward terms. This function determines what the feed forward controller
contributes to the PWM at a given time. The ff matrix transforms the [theta_l,theta_r] vector, with
its derivatives, into [voltage_l, voltage_r].
*/
std::array<float, 2> feed_forward(float u) {
    float power_2 = u * u;
    float power_3 = power_2 * u;

    float traj_2 = traj_duration * traj_duration;
    float traj_3 = traj_2 * traj_duration;

    float x = cx[0] + cx[1] * u + cx[2] * power_2 + cx[3] * power_3;
    float y = cy[0] + cy[1] * u + cy[2] * power_2 + cy[3] * power_3;
    std::array<float, 2> thetas = xy_to_theta(x * PI / 180, y * PI / 180);

    float x_vel = cx[1] + 2 * cx[2] * u + 3 * cx[3] * power_2;
    float y_vel = cy[1] + 2 * cy[2] * u + 3 * cy[3] * power_2;
    std::array<float, 2> theta_vel = xy_to_theta(x_vel * PI / 180 / traj_duration, y_vel * PI / 180 / traj_duration);

    float x_accel = 2 * cx[2] + 6 * cx[3] * u;
    float y_accel = 2 * cy[2] + 6 * cy[3] * u;
    std::array<float, 2> theta_accel = xy_to_theta(x_accel * PI / 180 / traj_2, y_accel * PI / 180 / traj_2);

    float x_jerk = 6 * cx[3];
    float y_jerk = 6 * cy[3];
    std::array<float, 2> theta_jerk = xy_to_theta(x_jerk * PI / 180 / traj_3, y_jerk * PI / 180 / traj_3);

    // The next 4 lines take the feed forward gains from the simulink model, which are stored in the ff variable as a 2x2x4 matrix, and multiply
    // by the corresponding x position, velocity, accel, and jerk which are calculated above.
    float left_feed_forward = 100 / 24 * (ff[0][0][3] * thetas[0] + ff[0][0][2] * theta_vel[0] + ff[0][0][1] * theta_accel[0] + ff[0][0][0] * theta_jerk[0]);
    left_feed_forward += 100 / 24 * (ff[0][1][3] * thetas[1] + ff[0][1][2] * theta_vel[1] + ff[0][1][1] * theta_accel[1] + ff[0][1][0] * theta_jerk[1]);
    float right_feed_forward = 100 / 24 * (ff[1][0][3] * thetas[0] + ff[1][0][2] * theta_vel[0] + ff[1][0][1] * theta_accel[0] + ff[1][0][0] * theta_jerk[0]);
    right_feed_forward += 100 / 24 * (ff[1][1][3] * thetas[1] + ff[1][1][2] * theta_vel[1] + ff[1][1][1] * theta_accel[1] + ff[1][1][0] * theta_jerk[1]);
    return {{left_feed_forward, right_feed_forward}};
}

/*
Home the table in the bottom left corner
and zero the encoders.
*/
void home_table(float x_speed, float y_speed, float position_threshold) {
    // Home X
    set_motor_pwms(-x_speed, -x_speed);
    delay(200);

    float previous_left_encoder = read_motor_angles()[0];

    while (true) {
        delay(100);
        if (abs(previous_left_encoder - read_motor_angles()[0]) < position_threshold) {
            set_motor_pwms(0, 0);
            break;
        }
        previous_left_encoder = read_motor_angles()[0];
    }

    // Home Y
    set_motor_pwms(-y_speed, y_speed);
    delay(200);

    previous_left_encoder = read_motor_angles()[0];

    while (true) {
        delay(100);
        if (abs(previous_left_encoder - read_motor_angles()[0]) < position_threshold) {
            set_motor_pwms(0, 0);
            break;
        }
        previous_left_encoder = read_motor_angles()[0];
    }

    // Nudge into the corner
    set_motor_pwms(-x_speed, 0);
    delay(500);
    set_motor_pwms(0, 0);
    delay(500);

    // Offsets to set the (0,0) in (x,y) coordinates to the bottom left corner of the table
    std::array<float, 2> global_theta_offsets = xy_to_theta(X_OFFSET, Y_OFFSET);

    left_revolutions = 0;
    right_revolutions = 0;
    left_offset = read_motor_angles()[0] - global_theta_offsets[0];
    right_offset = read_motor_angles()[1] - global_theta_offsets[1];
    Serial2.println("Fully Homed");
}

void command_motors(float x_pos, float y_pos, double current_time, double previous_time) {
    std::array<float, 2> target_angles = xy_to_theta(x_pos, y_pos);

    std::array<float, 2> actual_angles = read_motor_angles();
    std::array<float, 2> actual_pos = theta_to_xy(actual_angles[0], actual_angles[1]);
    // Add accumulated error update

    float left_error = target_angles[0] - actual_angles[0];
    float right_error = target_angles[1] - actual_angles[1];

    float left_pid = pid(left_error, left_accumulated_error, left_previous_error, current_time - previous_time);
    float right_pid = pid(right_error, right_accumulated_error, right_previous_error, current_time - previous_time);

    left_previous_error = left_error;
    right_previous_error = right_error;

    left_accumulated_error += left_error;
    right_accumulated_error += right_error;

    std::array<float, 2> feed_forward_values = feed_forward((current_time - path_start_time) / traj_duration);

    float left_feed_forward = fmin(fmax(-MAX_PWM, feed_forward_values[0]), MAX_PWM);
    float right_feed_forward = fmin(fmax(-MAX_PWM, feed_forward_values[1]), MAX_PWM);

    float left_pwm = fmin(fmax(-MAX_PWM, left_pid + left_feed_forward), MAX_PWM);
    float right_pwm = fmin(fmax(-MAX_PWM, right_pid + right_feed_forward), MAX_PWM);

    if (!STATE_LOGGING) {
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
        Serial2.print(right_pwm);
        Serial2.print(",");
        Serial2.print(x_puck*100);
        Serial2.print(",");
        Serial2.println(y_puck*100);
    }


    set_motor_pwms(left_pwm, right_pwm);
}

// Helper function to clip the value to its min and max
float clip_value(float inVal, float minVal, float maxVal) {
    return fmin(fmax(inVal, minVal), maxVal);
}

/*
Generate the path and its coefficients for the given HLC target. Calculations from here:
https://en.wikipedia.org/wiki/B%C3%A9zier_curve
*/

void generate_path(float x_puck, float y_puck, float vf_theta, float vf_magnitude, float path_time, State game_state) {
    std::array<float, 2> current_angles = read_motor_angles();
    float x_initial = theta_to_xy(current_angles[0], current_angles[1])[0];
    float y_initial = theta_to_xy(current_angles[0], current_angles[1])[1];

    bool do_offset;

    if (game_state == ATTACK) {
        do_offset = true;
    }
    else {
        do_offset = false;
    }

    // float u = (t - path_start_time) / traj_duration;

    float vx_initial;
    float vy_initial;

    // if ((t + 0.01 - path_start_time) / traj_duration > 1.0) {
    //     float power_2 = u * u;
    //     vx_initial = 0;
    //     vy_initial = 0;

    //     // vx_initial = (cx[1] + 2 * cx[2] * u + 3 * cx[3] * power_2) / traj_duration;
    //     // vy_initial = (cy[1] + 2 * cy[2] * u + 3 * cy[3] * power_2) / traj_duration;
    // } else {
    //     vx_initial = 0;
    //     vy_initial = 0;
    // }
    // if (game_state == ATTACK) {
    //     vx_initial = v_3_xp;
    //     vy_initial = v_3_yp;
    // }
    // else {
    //     vx_initial = 0;
    //     vy_initial = 0;
    // }

    vx_initial = v_3_xp;
    vy_initial = v_3_yp;

    vx_initial = clip_value(vx_initial, MIN_MALLET_INIT_VELO, MAX_MALLET_INIT_VELO);
    vy_initial = clip_value(vy_initial, MIN_MALLET_INIT_VELO, MAX_MALLET_INIT_VELO);

    float vf_x = cos(vf_theta);
    float vf_y = sin(vf_theta);

    // Back off from the "intecept point" because the collision occurs when the mallet and puck
    // are radius_puck + radius_mallet apart from each other.
    float intercept_point_x;
    float intercept_point_y;
    if (do_offset) {
        intercept_point_x = x_puck - vf_x * (MALLET_RADIUS + PUCK_RADIUS)/2;
        intercept_point_y = y_puck - vf_y * (MALLET_RADIUS + PUCK_RADIUS)/2;
    }
    else {
        intercept_point_x = x_puck;
        intercept_point_y = y_puck;
    }

    // Clip intercept point to within table
    intercept_point_x = clip_value(intercept_point_x, X_MIN, X_MAX);
    intercept_point_y = clip_value(intercept_point_y, Y_MIN, Y_MAX);

    // Scale the final velocity at the intercept
    float v_3_x = vf_x * vf_magnitude;
    float v_3_y = vf_y * vf_magnitude;

    // Set previous final velocity
    v_3_xp = v_3_x;
    v_3_yp = v_3_y;

    // Find control point locations
    float q1_x = x_initial + vx_initial * path_time / 3;
    float q1_y = y_initial + vy_initial * path_time / 3;

    float q2_x = intercept_point_x - v_3_x * path_time / 3;
    float q2_y = intercept_point_y - v_3_y * path_time / 3;

    // Clip control points to within table
    q1_x = clip_value(q1_x, X_MIN, X_MAX);
    q1_y = clip_value(q1_y, Y_MIN, Y_MAX);

    q2_x = clip_value(q2_x, X_MIN, X_MAX);
    q2_y = clip_value(q2_y, Y_MIN, Y_MAX);

    cx = {{x_initial, 3 * q1_x - 3 * x_initial, 3 * x_initial - 6 * q1_x + 3 * q2_x, 3 * q1_x - x_initial - 3 * q2_x + intercept_point_x}};
    cy = {{y_initial, 3 * q1_y - 3 * y_initial, 3 * y_initial - 6 * q1_y + 3 * q2_y, 3 * q1_y - y_initial - 3 * q2_y + intercept_point_y}};

    traj_duration = path_time;
    path_start_time = t;
    tf = path_start_time + path_time;
}

float read_float(char end) {
    char buffer[10];
    byte index = 0;
    bool isNegative = false;
    while (true) {
        while (Serial2.available() == 0) {
        }  // wait for a byte to be available
        char incoming = Serial2.read();
        if (incoming == '-') {
            isNegative = true;
        } else if (incoming == '.') {
            buffer[index++] = incoming;
        } else if (incoming >= '0' && incoming <= '9') {
            buffer[index++] = incoming;
        } else if (incoming == end) {
            break;
        }
    }
    buffer[index] = '\0';
    float number = atof(buffer);
    if (isNegative) {
        number *= -1;
    }
    return number;
}

bool read_camera() {
    float temp;
    if (Serial2.available()) {
        frame_time = read_float(',');
        x_puck = read_float(',');
        y_puck = read_float(',');
        puck_missing_frames = read_float(',');
        temp = read_float('\n');
        return true;
    } else {
        return false;
    }
}

void setup() {
    Serial2.begin(460800);
    SPI.setMISO(SPI_MISO_PIN);
    SPI.setSCLK(SPI_SCLK_PIN);
    SPI.beginTransaction(SPISettings(460800, MSBFIRST, SPI_MODE1));
    pinMode(LED_BUILTIN, OUTPUT);  // set this pin as output

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

    read_motor_angles();  // Need a dummy call to get the previous angle variable set properly

    home_table(13, 8, 10);

    while (!Serial2.available()) {
        // wait for serial to become available
    }
    delay(500);

    Serial2.println("BEGIN CSV");
    Serial2.println("Time(ms),X_Target(cm),Y_Target(cm),X_Mallet(cm),Y_Mallet(cm),Left_Angle(deg),Right_Angle(deg),Left_Error(deg),Right_Error(deg),Left_PID,Right_PID,Left_Feed_Forward,Right_Feed_Forward,Left_PWM,Right_PWM,X_Puck,Y_Puck");

    // Serial2.println("Time(ms),X_Target(cm),Y_Target(cm),X_Puck(cm),Y_Puck(cm),Left_Angle(deg),Right_Angle(deg),Left_Error(deg),Right_Error(deg),Left_PID,Right_PID,Left_Feed_Forward,Right_Feed_Forward,Left_PWM,Right_PWM,x_puck,y_puck");

    while (!read_camera()) {
        // wait for first frame to be saved
        Serial2.println("WAITING FOR CAMERA");
    }
    frame_time_prev = frame_time;
    xp_prev = x_puck;
    yp_prev = y_puck;

    previous_time = 0;
    start_time = micros();
    tf = 0;
}

void loop() {
    t = (micros() - start_time) / 1000000.0;

    std::array<float, 2> current_angles = read_motor_angles();
    std::array<float, 2> current_pos = theta_to_xy(current_angles[0], current_angles[1]);

    float prev_missing_frames = puck_missing_frames;

    if (read_camera() && t > (path_start_time + traj_duration)) {
        // Serial2.println("CAMERA READ");
        // Only play if two frames read in a row from camera
        if (puck_missing_frames == prev_missing_frames) {
            classical_agent(current_pos[0], current_pos[1], xp_prev, yp_prev, x_puck, y_puck);
        }
        frame_time_prev = frame_time;
        xp_prev = x_puck;
        yp_prev = y_puck;
    }
    else {
        v_3_xp = 0;
        v_3_yp = 0;
    }

    if (t > tf) {
        set_motor_pwms(0,0);
        return;
    }

    float u = (t - path_start_time) / traj_duration;

    if (abs(u) > 1){
        set_motor_pwms(0,0);
        return;
    }

    float power_2 = u * u;
    float power_3 = power_2 * u;

    float x_pos = cx[0] + cx[1] * u + cx[2] * power_2 + cx[3] * power_3;
    float y_pos = cy[0] + cy[1] * u + cy[2] * power_2 + cy[3] * power_3;

    if (x_pos < X_MIN) {
        x_pos = X_MIN;
    } else if (x_pos > X_MAX) {
        x_pos = X_MAX;
    }

    if (y_pos < Y_MIN) {
        y_pos = Y_MIN;
    } else if (y_pos > Y_MAX) {
        y_pos = Y_MAX;
    }

    command_motors(x_pos, y_pos, t, previous_time);
    previous_time = t;
}