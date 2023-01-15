

class GantryModel
{
private:

    float A[3] = { 0.900555094152792,	4.89815533615442,	0.0564943472288371};
    float B[3] = {0.899994333925378,	4.89120127441819,	-0.0463090759726549};

    // new ramp ceoffs
    // float A[3] = {0.00338637791472368,	0.00583826471299068, 0.114049522708959};
    // float B[3] = {-0.00232015939854998,-0.00705025835843183,	-0.0212928418857712};

    // new step coeffs
    // float A[3] = {0.00213052462653085,	0.0307510306542813,	0.132961429507223};
    // float B[3] = {-0.00141458223317764,	-0.00983851249298195, 0.0141591992993925};

    float* x_coeffs = {0};
    float* y_coeffs = {0};


    void get_theta(float theta_matrix[], float coeff_matrix[]);
    void get_xy_der(float t);
    
    void get_first_dir();
    void get_second_dir();
    void get_third_dir();

 

    float R = 3.5306;

public:

    float theta[2] = {0};
    float theta1[2] = {0};
    float theta2[2] = {0};
    float theta3[2] = {0};

    float xy[6][2] = {0};
    float xy1[2] = {0};
    float xy2[2] = {0};
    float xy3[2] = {0};


    void set_coeffs(float* x_coeffs, float* y_coeffs);
    float* get_effort(float time);
};

