#include "model.h"

void GantryModel::set_coeffs(float x_coeffs[6], float y_coeffs[6]){
    this->x_coeffs = x_coeffs;
    this->y_coeffs = y_coeffs;
}

void GantryModel::get_xy_der(float t){
    float a = t*t*t*t;
    
    this->xy1[0] = 0;
    this->xy1[1] = 0;
    this->xy2[0] = 0;
    this->xy2[1] = 0;
    this->xy3[0] = 0;
    this->xy3[1] = 0;

    for(int i = 0; i < 6; i++){

        float to_add_x = (5-i)*this->x_coeffs[i]*(a);
        float to_add_y = (5-i)*this->y_coeffs[i]*(a);

        this->xy1[0] = this->xy1[0] + to_add_x;
        this->xy1[1] = this->xy1[1] + to_add_y;
        
        if (i < 4){

            this->xy2[0] = this->xy2[0] + (5-i-1)*to_add_x/(t);
            this->xy2[1] = this->xy2[1] + (5-i-1)*to_add_y/(t);

            if (i<3){
                this->xy3[0] = this->xy3[0] + (5-i-1)*(5-i-2)*to_add_x/(t*t);
                this->xy3[1] = this->xy3[1] + (5-i-1)*(5-i-2)*to_add_y/(t*t);
                }
            }

        a = a/t;
    }

}

float* GantryModel::get_effort(float time){
   
    static float V[2] = {0};

    get_xy_der(time);
    get_first_dir();
    get_second_dir();
    get_third_dir();
    
    V[0] = this->A[0]*this->theta3[0]+this->A[1]*this->theta2[0]+this->A[2]*this->theta1[0]+this->B[0]*this->theta3[1]+this->B[1]*this->theta2[1]+this->B[2]*this->theta1[1];
    V[1] = this->B[0]*this->theta3[0]+this->B[1]*this->theta2[0]+this->B[2]*this->theta1[0]+this->A[0]*this->theta3[1]+this->A[1]*this->theta2[1]+this->A[2]*this->theta1[1];

    return V;
}

void GantryModel::get_theta(float theta_matrix[], float xy_matrix[]){
    for (int i = 0; i < 6; i++){
    theta_matrix[0] = xy_matrix[0]+xy_matrix[1];
    theta_matrix[1] = xy_matrix[0]-xy_matrix[1];

    }
    theta_matrix[0] = theta_matrix[0]/this->R;
    theta_matrix[1] = theta_matrix[1]/this->R;
}

void GantryModel::get_first_dir(){
   get_theta(this->theta1 ,this->xy1);
}

void GantryModel::get_second_dir(){
   get_theta(this->theta2,this->xy2);
}

void GantryModel::get_third_dir(){
   get_theta(this->theta3,this->xy3);
}