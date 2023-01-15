#include <Arduino.h>
#include <RoboClaw.h>
#include <SPI.h>
#define CHIP_SELECT_LEFT A4
#define CHIP_SELECT_RIGHT PB5
#define MOTOR_LEFT 0x81
#define MOTOR_RIGHT 0x80
#define two_to_the_14 16384
#define PULLEY_RADIUS 3.5306 // 2.78 inches in cm
#define ANGLE_THRESH 300 //this was 350
#define NUM_READS 200
#define OMEGA 360.0
#define window 40

const float ticks_to_deg = 360.0/two_to_the_14;



#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

class MalletController {
    public:
        float px = 45*.2; 
        float ix = 0;
        float dx = 0.066*45*.277;
        float py = 72*.2;
        float iy = 0;
        float dy = 0.066*72*0.5;

        // make sure this value is smaller than window!!
        // Used for integrating
        static const int position_window = 500;
        
        //Coefficients for savgol filter

        float savgol[window] = {10000000000};

        //Array for storing computing integral error
        float pos_error[2][position_window] = {0};

        float angle_reading[2] = {0,0};
        float prev_angle[2] = {0,0};
        float current_total_angle[2] = {0.0,0.0};
        int num_zerocrosses[2] = {0,0};

        float current_velocity[2] = {0,0};
        float velocity_hist[2][window] = {0,0};
        float xy_hist[2][window] = {0,0};
        float time_hist[window] = {0,0};
        float window_step_size;

        float start_angles[2]= {0,0};
        float xy[2] = {0,0};
        float desired_xy[2];
        float desired_velocity[2];
        float desired_acc[2];
        float integral_error[2] = {0,0};

        float effort_x = 0;
        float effort_y = 0;
        float err_x_pos = 0;
        float err_y_pos = 0;
        float err_x_vel = 0;
        float err_y_vel = 0;

        float time_step = 0;

        int effort_m1 = 0;
        int effort_m2 = 0;
        int loop_counter = 0;

        RoboClaw* roboclaw_p;
        HardwareSerial* serial_p;



    private:

        void update_savgol_coeff(float coeffs[]){
            float start = 6.0/(window*1.0*(window+1));
            float del = -12.0/(1.0*(window-1)*(window)*(window+1));

            for (int i = 0; i<window;i++){
                coeffs[i] = start;
                start = start+del;
            }
        }
        
        void update_desired_path();
        void update_desired_path_position(float time, float x_coeffs[], float y_coeffs[], float ret_val[]);
        void write_to_motor_simple(uint8_t val);
        void update_xy();
        void make_total_angle(float total_angle[], float angle[], int crosses[]);
        void update_desired_path_velocity(float time, float x_coeffs[], float y_coeffs[], float ret_vel[]);
        void update_desired_path_acc(float time, float x_coeffs[], float y_coeffs[], float ret_acc[]);
        void update_velocity(float xy[], float vel[], float xy_hist[2][window]);
        void compute_int_error();
        void zeroCrossing(int crosses[], float velocity[], float  angle[]);
        void update_coeffs(float curr_xy[], float curr_vel[], float curr_acc[], 
            float final_xy[], float final_vel[], float final_acc[], 
            float T, float x_coeffs[], float y_coeffs[]);
        float start_time=0;


    public:

        MalletController() {
            // SPI.beginTransaction(SPISettings(115200, MSBFIRST, SPI_MODE1));
            //Coefficients for savgol filter
            update_savgol_coeff(savgol);
            pinMode(CHIP_SELECT_LEFT, OUTPUT);
            pinMode(ENC_CHIP_SELECT_RIGHT, OUTPUT);
            digitalWrite(CHIP_SELECT_LEFT, HIGH);
            digitalWrite(ENC_CHIP_SELECT_RIGHT, HIGH);
            readAngle(start_angles);
            // zeroCrossing(num_zerocrosses,current_velocity, angle_reading);
            make_total_angle(current_total_angle,angle_reading,num_zerocrosses);
            update_xy();
            
        }

        bool update();
        void setPID();
        void setPath(float final_xy[], float final_vel[], float final_acc[], float time_step, float current_time);
        float x_coeffs[6] = {0,0,0,0,0,0};
        float y_coeffs[6] = {0,0,0,0,0,0};
        void write_to_motor(uint8_t address, int val);
        void readAngle(float result[]);  
        void clear_history();       
};