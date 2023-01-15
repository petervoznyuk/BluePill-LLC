#include <Arduino.h>
#include <RoboClaw.h>
#include "controller.h"
#include <SPI.h>


void MalletController::update_desired_path_position(float time, float x_coeffs[], float y_coeffs[], float ret_val[]){
  ret_val[0] = 0;
  ret_val[1] = 0;
  float tpow = 1;
  for (int i = 5; i>=0; i--){
    ret_val[0] += x_coeffs[i]*tpow;
    ret_val[1] += y_coeffs[i]*tpow;
    tpow *= time;
  }
}

void MalletController::update_desired_path_velocity(float time, float x_coeffs[], float y_coeffs[], float ret_vel[]){
  ret_vel[0] = 0;
  ret_vel[1] = 0;
  float tpow = 1;
  for (int i = 4; i>=0; i--){
    ret_vel[0] += x_coeffs[i]*tpow*(4-i+1);
    ret_vel[1] += y_coeffs[i]*tpow*(4-i+1);
    tpow *= time;
  }
}

void MalletController::update_desired_path_acc(float time, float x_coeffs[], float y_coeffs[], float ret_acc[]){
  ret_acc[0] = 0;
  ret_acc[1] = 0;
  float tpow = 1;
  for (int i = 3; i>=0; i--){
    ret_acc[0] += x_coeffs[i]*tpow*(4-i+1)*(3-i+1);
    ret_acc[1] += y_coeffs[i]*tpow*(4-i+1)*(3-i+1);
    tpow *= time;
  }
}

void MalletController::update_xy(){
  xy[0] = (current_total_angle[0]+current_total_angle[1])/2*PULLEY_RADIUS*PI/180;
  xy[1] = (current_total_angle[0]-current_total_angle[1])/2*PULLEY_RADIUS*PI/180;

  for(int i=window-1;i>0;i--){
    xy_hist[0][i] = xy_hist[0][i-1];
    xy_hist[1][i] = xy_hist[1][i-1];
  }
  xy_hist[0][0] = xy[0];
  xy_hist[1][0] = xy[1];
}

void MalletController::write_to_motor(uint8_t address, int val){
  if (val<0){
    val = MAX(val,-127);
    (*roboclaw_p).BackwardM1(address, (uint8_t) abs(val));
  }
  else {
    val = MIN(val,127);
    (*roboclaw_p).ForwardM1(address ,(uint8_t) val);
  }
}

//Read from or write to register from the SCP1000:
void MalletController::readAngle(float result[]) {
  int chips[2] = {CHIP_SELECT_LEFT,ENC_CHIP_SELECT_RIGHT};
  
  for (int i = 0; i<2; i++) {
    u_int16_t reads; // incoming byte from the SPI
    digitalWrite(chips[i], LOW);
    reads = SPI.transfer16(0xFF);
    digitalWrite(chips[i],HIGH);
    result[i] = 360.0-(reads & 0b0011111111111111)*ticks_to_deg-start_angles[i];
    }
}

void MalletController::zeroCrossing(int crosses[], float velocity[], float  angle[]){
    for (int i=0; i<2; i++) {
      if ((angle[i] - prev_angle[i]) > ANGLE_THRESH) {
          crosses[i] -= 1;
        }
        else if ((-angle_reading[i] + prev_angle[i]) > ANGLE_THRESH){
          crosses[i] += 1;
        }
        prev_angle[i] = angle_reading[i];
      }
  }

void MalletController::make_total_angle(float total_angle[], float angle[], int crosses[]){
  for (int i=0;i<2;i++){
    total_angle[i] = angle[i]+crosses[i]*360.0;
  }
}

void MalletController::compute_int_error(){
    for (int i = 0; i<2; i++) {
      // subtract off the last value from mean
      integral_error[i] = integral_error[i] - pos_error[i][position_window -1]/position_window;
    }
    
    for(int i=position_window-1;i>0;i--){
    pos_error[0][i] = pos_error[0][i-1];
    pos_error[1][i] = pos_error[1][i-1];
  }

    // calculate the new error
    for(int i = 0; i<2; i++){
       pos_error[i][0] = desired_xy[i]-xy[i];
    }

    for(int i = 0; i<2; i++){
       integral_error[i] = integral_error[i]+pos_error[i][0]/position_window;
    }
}

void MalletController::update_velocity(float xy[2], float vel[2], float xy_hist[2][window]){


  float sum_vel[2] = {0,0};

  for (int i = 0;i<2;i++){
    for (int j = 0; j < window;j++){
      sum_vel[i] += xy_hist[i][j]*savgol[j];
    }
  }

  vel[0] = sum_vel[0]/(window_step_size);
  vel[1] = sum_vel[1]/(window_step_size);


  for(int i=window-1;i>0;i--){
    velocity_hist[0][i] = velocity_hist[0][i-1];
    velocity_hist[1][i] = velocity_hist[1][i-1];
  }


  velocity_hist[0][0] = vel[0];
  velocity_hist[1][0] = vel[1];

}

bool MalletController::update(){

  float time_on_path = 1.0*micros()*1e-6-start_time;
  for(int i=window-1;i>0;i--){
    time_hist[i] = time_hist[i-1];
  }

  time_hist[0] = time_on_path;
  window_step_size = (time_hist[0] - time_hist[window-1])/(window-1);

  readAngle(angle_reading);
  zeroCrossing(num_zerocrosses,current_velocity, angle_reading);
  make_total_angle(current_total_angle,angle_reading,num_zerocrosses);
  update_xy();


  if (loop_counter > window){
    update_velocity(xy, current_velocity,xy_hist);
    loop_counter = 0;
  }

  loop_counter++;

  if (time_on_path>=this->time_step){
    return true;
  }

  update_desired_path_position(time_on_path,x_coeffs, y_coeffs, desired_xy);
  update_desired_path_velocity(time_on_path, x_coeffs, y_coeffs, desired_velocity);


  err_x_pos = (desired_xy[0]-xy[0]);
  err_y_pos = (desired_xy[1]-xy[1]);
  err_x_vel = (desired_velocity[0]-current_velocity[0]);
  err_y_vel = (desired_velocity[1]-current_velocity[1]);

  effort_x = px*err_x_pos+dx*err_x_vel;
  effort_y = py*err_y_pos+dy*err_y_vel;
 
  effort_m1 = (effort_x+effort_y)/PULLEY_RADIUS;
  effort_m2 = (effort_x-effort_y)/PULLEY_RADIUS;

  return false;
}

void MalletController::update_coeffs(float curr_xy[2], float curr_vel[2], float curr_acc[2], float final_xy[2], float final_vel[2], float final_acc[2], float T, float x_coeffs[6], float y_coeffs[6]){
  float Vx[] = {curr_xy[0],final_xy[0],curr_vel[0],final_vel[0],curr_acc[0],final_acc[0]};
  

  x_coeffs[0] = (-6.0/(pow (T, 5.0)))*Vx[0] + (6.0/(pow (T, 5.0)))*Vx[1] + (-3.0/(pow (T, 4.0)))*Vx[2] + (-3.0/(pow (T, 4.0)))*Vx[3] + (-0.5/(pow (T, 3.0)))*Vx[4] + (0.5/(pow (T, 3.0)))*Vx[5];
  x_coeffs[1] = (15.0/(pow (T, 4.0)))*Vx[0] + (-15.0/(pow (T, 4.0)))*Vx[1] + (8.0/(pow (T, 3.0)))*Vx[2] + (7.0/(pow (T, 3.0)))*Vx[3] + (1.5/(pow (T, 2.0)))*Vx[4] + (-1.0/(pow (T, 2.0)))*Vx[5];
  x_coeffs[2] = (-10.0/(pow (T, 3.0)))*Vx[0] + (10.0/(pow (T, 3.0)))*Vx[1] + (-6.0/(pow (T, 2.0)))*Vx[2] + (-4.0/(pow (T, 2.0)))*Vx[3] + (-3.0/(2.0*T))*Vx[4] + (1.0/(2.0*T))*Vx[5]; 
  x_coeffs[3] = 0.5*Vx[4];
  x_coeffs[4] = Vx[2];
  x_coeffs[5] = Vx[0];

  float Vy[] = {curr_xy[1],final_xy[1],curr_vel[1],final_vel[1],curr_acc[1],final_acc[1]};

  y_coeffs[0] = (-6.0/(pow (T, 5.0)))*Vy[0] + (6.0/(pow (T, 5.0)))*Vy[1] + (-3.0/(pow (T, 4.0)))*Vy[2] + (-3.0/(pow (T, 4.0)))*Vy[3] + (-0.5/(pow (T, 3.0)))*Vy[4] + (0.5/(pow (T, 3.0)))*Vy[5];
  y_coeffs[1] = (15.0/(pow (T, 4.0)))*Vy[0] + (-15.0/(pow (T, 4.0)))*Vy[1] + (8.0/(pow (T, 3.0)))*Vy[2] + (7.0/(pow (T, 3.0)))*Vy[3] + (1.5/(pow (T, 2.0)))*Vy[4] + (-1.0/(pow (T, 2.0)))*Vy[5];
  y_coeffs[2] = (-10.0/(pow (T, 3.0)))*Vy[0] + (10.0/(pow (T, 3.0)))*Vy[1] + (-6.0/(pow (T, 2.0)))*Vy[2] + (-4.0/(pow (T, 2.0)))*Vy[3] + (-3.0/(2.0*T))*Vy[4] + (1.0/(2.0*T))*Vy[5]; 
  y_coeffs[3] = 0.5*Vy[4];
  y_coeffs[4] = Vy[2];
  y_coeffs[5] = Vy[0];
}

void MalletController::setPath(float final_xy[], float final_vel[], float final_acc[], float deltaT, float current_time){
  if (current_time > time_step) {
    desired_acc[0] = 0;
    desired_acc[1] = 0;
  } else {
    update_desired_path_acc(current_time, x_coeffs, y_coeffs , desired_acc);
  }

  this->start_time = current_time;

  update_coeffs(xy, current_velocity, desired_acc, final_xy, final_vel, final_acc, deltaT, x_coeffs, y_coeffs);
  
  this->time_step = deltaT;
} 


void MalletController::clear_history() {
  for (int i=0; i<2; i++) {
    xy[i] = 0.0;
    current_velocity[i] = 0.0;
    current_total_angle[i] = 0.0;
    num_zerocrosses[i] = 0;
    for (int j=0; j<window; j++) {
      velocity_hist[i][j] = 0.0;
      xy_hist[i][j] = 0.0;
    }
  }
  for (int i=0; i<window; i++) {
    time_hist[window] = 0.0;
  }
}

void MalletController::setPID() {
  int flag = 1;
  String var1 = "";
  String var2 = "";
  String var3 = "";
  String var4 = "";
  String var5 = "";
  String var6 = "";
  Serial.println("Please enter effort values px ix dx py iy dy");


  // check for incoming serial data:
  while(flag == 1){

  while(Serial.available() > 0) {
    Serial.println("blah");

    var1 = Serial.readStringUntil(' '); // writes in the string all the inputs till a comma
    Serial.read(); 
    var2 = Serial.readStringUntil(' ');
    Serial.read(); 
    var3 = Serial.readStringUntil(' ');
    Serial.read(); 
    var4 = Serial.readStringUntil(' ');
    Serial.read(); 
    var5 = Serial.readStringUntil(' ');
    Serial.read(); 
    var6 = Serial.readStringUntil('\n');
    delay(10);
  }


  if (var1 != ""){
    px = var1.toFloat();
    ix = var2.toFloat();
    dx = var3.toFloat();
    py = var4.toFloat();
    iy = var5.toFloat();
    dy = var6.toFloat();

    Serial.print("Px: ");
    Serial.println(px);
    Serial.print("Ix: ");
    Serial.println(ix);
    Serial.print("Dx: ");
    Serial.println(dx);
    Serial.print("Py: ");
    Serial.println(py);
    Serial.print("Iy: ");
    Serial.println(iy);
    Serial.print("Dy: ");
    Serial.println(dy);

    delay(2000);

    Serial.println("zoomin");
    flag = 0;
    }
  }
}