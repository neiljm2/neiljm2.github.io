#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = (-25.19*PI)/180;//-0.37;
float offset_Enc3_rad = (14.56*PI)/180;//0.27;

// Your global variables.
long mycount = 0;
float thetaD = 0;
float dthetaD = 0;
float ddthetaD = 0;
int tmod = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

long arrayindex = 0;
int UARTprint = 0;

float calc_x = 0;
float calc_y = 0;
float calc_z = 0;

float print_x = 0;
float print_y = 0;
float print_z = 0;

float DHth1 = 0;
float DHth2 = 0;
float DHth3 = 0;
float Mth1 = 0;
float Mth2 = 0;
float Mth3 = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float L = 0.254;
float d; //NM intermediate variables for IK calculations
float r;
float beta;
float gamma;
float alpha;

//velocity variables
float Theta1_old = 0;
float W1_raw = 0;
float W1_old1 = 0;
float W1_old2 = 0;
float W1 = 0;

float Theta2_old = 0;
float W2_raw = 0;
float W2_old1 = 0;
float W2_old2 = 0;
float W2 = 0;

float Theta3_old = 0;
float W3_raw = 0;
float W3_old1 = 0;
float W3_old2 = 0;
float W3 = 0;

float u_fric1 = 0;
float Vpos1 = 0.2;
float Cpos1 = 0.3637;
float Vneg1 = 0.2377;
float Cneg1 = -0.2948;
float Vhigh1 = 3.6;

float u_fric2 = 0;
float Vpos2 = 0.24;
float Cpos2 = 0.4;
float Vneg2 = 0.27;
float Cneg2 = -0.453;
float Vhigh2 = 1;

float u_fric3 = 0;
float Vpos3 = 0.18;
float Cpos3 = 0.48;
float Vneg3 = 0.2132;
float Cneg3 = -0.45;
float Vhigh3 = 1;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

float err1 = 0;
float err2 = 0;
float err3 = 0;

float err1prev = 0;
float err2prev = 0;
float err3prev = 0;

float kp1 = 100;
float kp2 = 100;
float kp3 = 100;
float kd1 = 4;
float kd2 = 5;
float kd3 = 3;
float ki1 = 1200;
float ki2 = 1200;
float ki3 = 1200;

float I1k = 0;
float I2k = 0;
float I3k = 0;
float tau1test = 0;
float I1kprev = 0;
float I2kprev = 0;
float I3kprev = 0;
float errband = 0.05;
float a1 = 0;
float b1 = 0;
float c1 = PI/2.0;
float d1 = -PI/3.0;
float a2 = -40.0*PI/3.0;
float b2 = 12.0*PI;
float c2 = -7.0*PI/2.0;
float d2 = PI/3.0;

float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;
float t = 0;

float funx = 0;
float funy = 0;
float funz = 0;
float thetaD1 = 0.0;
float thetaD2 = 0.0;
float thetaD3 = 0.0;

float p1 = 0.03;
float p2 = 0.0128;
float p3 = 0.0076;
float p4 = 0.0753;
float p5 = 0.0298;

float D11 = 0.03;
float D12 = 0.0;
float D21 = 0.0;
float D22 = 0.0128;

float C11 = 0.0;
float C12 = 0.0;
float C21 = 0.0;
float C22 = 0.0;

float G1 = 0.0;
float G2 = 0.0;

float derr1 = 0.0;
float derr2 = 0.0;
float derr3 = 0.0;
float ath2 = 0.0;
float ath3 = 0.0;
float sin3_2 = 0.0;
float cos3_2 = 0.0;

float Kpinv2 = 1500;
float Kpinv3 = 2000;
float Kdinv2 = 85;
float Kdinv3 = 125;

float dthetaD1 = 0.0;
float ddthetaD1 = 0.0;
float dthetaD2 = 0.0;
float ddthetaD2 = 0.0;
int whattoplot = 1;
float step1 = 0.25;
float step2 = 0.3;

int controller = 0;

float step = 0.0;

float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0;
float thetax = 0;
float thetay = 0;
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;
//// Rotation zxy and its Transpose
//cosz = cos(thetaz);
//sinz = sin(thetaz);
//cosx = cos(thetax);
//sinx = sin(thetax);
//cosy = cos(thetay);
//siny = sin(thetay);
//RT11 = R11 = cosz*cosy-sinz*sinx*siny;
//RT21 = R12 = -sinz*cosx;
//RT31 = R13 = cosz*siny+sinz*sinx*cosy;
//RT12 = R21 = sinz*cosy+cosz*sinx*siny;
//RT22 = R22 = cosz*cosx;
//RT32 = R23 = sinz*siny-cosz*sinx*cosy;
//RT13 = R31 = -cosx*siny;
//RT23 = R32 = sinx;
//RT33 = R33 = cosx*cosy;

float JvT[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float tauvec[3][1] = {{0}, {0}, {0}};
float forcevec[3][1] = {{0}, {0}, {0}};
float rot_WN[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float rot_NW[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float rotx[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float roty[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float rotxy[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float rotz[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float KPx = 1000.0;
float KPy = 1000.0;
float KPz = 1000.0;
float KDx = 50.0;
float KDy = 50.0;
float KDz = 50.0;
float KPxN = 1500.0;
float KPyN = 1500.0;
float KPzN = 1500.0;
float KDxN = 50.0;
float KDyN = 50.0;
float KDzN = 50.0;
float xD = 0.25;
float yD = 0.25;
float zD = 0.25;
float dxD = 0;
float dyD = 0;
float dzD = 0;
float x = 0;
float y = 0;
float z = 0;
float dx = 0;
float dy = 0;
float dz = 0;

float dx_raw = 0;
float x_old = 0;
float dx_old1 = 0;
float dx_old2 = 0;
float dy_raw = 0;
float y_old = 0;
float dy_old1 = 0;
float dy_old2 = 0;
float dz_raw = 0;
float z_old = 0;
float dz_old1 = 0;
float dz_old2 = 0;

float ff = 0;
float FZcmd = 0.0;
float Kt = 6.0;
float FZoffset = 12;

float KP_N[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float KD_N[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float error_W[3][1] = {{0}, {0}, {0}};
float errordot_W[3][1] = {{0}, {0}, {0}};
float error_N[3][1] = {{0}, {0}, {0}};
float errordot_N[3][1] = {{0}, {0}, {0}};
float term1[3][1] = {{0}, {0}, {0}};
float term2[3][1] = {{0}, {0}, {0}};
float bigterm_N[3][1] = {{0}, {0}, {0}};
float bigterm_W[3][1] = {{0}, {0}, {0}};

float xa = 0;
float ya = 0;
float za = 0;
float xb = 0;
float yb = 0;
float zb = 0;
float x1 = 0.25;
float y1 = 0.25;
float z1 = 0.25;
float x2 = 0.4;
float y2 = 0.0;
float z2 = 0.25;
float delx = 0;
float dely = 0;
float delz = 0;
float distAB = 0;
float t_total = 0;
float t_start = 0;
int state = 0;
float ramp = 0.0;
void matrixTranspose(float A[][3], float A_trans[][3]);
void matrixVectorMultiplication(float A[][3], float B[][1], float C[][1]);
void matrixMultiplication(float A[][3], float B[][3], float C[][3]);
void rotationMatrix(float theta, float R[3][3], char axis);

typedef struct point_tag {
    float x;
    float y;
    float z;
    float thz;
    float V;
    int mode;
} point;

#define XYZSTIFF 1
#define ZSTIFF 2
#define XZSTIFF 3
#define XYSTIFF 4
#define LOWSTIFF 5

#define NUM_POINTS 22
// find all your own x,y,z points and angles for weaking.
point points[NUM_POINTS] = {{0.138, 0.000, 0.420, 0, 0.2, LOWSTIFF},
                            {0.138, 0.000, 0.420, 0, 0.2, LOWSTIFF},
                            {0.25, 0.0, 0.5, 0, 0.2, XYZSTIFF}, //out away from home
                            {0.24, 0.25, 0.41, 0, 0.2, XYZSTIFF}, //home->hole
                            {0.03, 0.335, 0.20, 0, 0.2, XYZSTIFF}, //above hole
                            {0.03, 0.335, 0.117, 0, 0.05, ZSTIFF}, //in hole
                            {0.03, 0.335, 0.117, 0, 0.05, ZSTIFF}, //in hole wait
                            {0.03, 0.34, 0.20, 0, 0.2, ZSTIFF}, //above hole
                            {0.209, 0.102, 0.249, 0, 0.2, XYZSTIFF}, //avoid obs
                            {0.395, 0.102, 0.198, 0, 0.2, XYZSTIFF}, //maze start
                            {0.420, 0.061, 0.198, -0.927, 0.05, XZSTIFF}, //maze corner 1 point 1
                            {0.418, 0.047, 0.198, 0.0, 0.05, XYZSTIFF}, //maze 1.2
                            {0.404, 0.040, 0.198, 0.0, 0.05, XYZSTIFF}, //maze 1.3
                            {0.336, 0.052, 0.198, -0.262, 0.05, XZSTIFF}, //maze 2.1
                            {0.328, 0.043, 0.198, 0.0, 0.05, XYZSTIFF}, //maze 2.2
                            {0.327, 0.032, 0.198, 0.0, 0.05, XYZSTIFF}, //maze 2.3
                            {0.396, -0.049, 0.198, -0.927, 0.05, XZSTIFF}, //maze end
                            {0.396, -0.049, 0.225, 0.0, 0.05, XYZSTIFF}, //maze out
                            {0.209, 0.102, 0.249, 0, 0.2, XYZSTIFF}, //avoid obs
                            {0.28, 0.225, 0.32, 0, 0.2, XYZSTIFF}, //above egg
                            {0.28, 0.225, 0.28, 0, 0.05, XYSTIFF}, //push egg
                            {0.28, 0.225, 0.28, 0, 0.05, XYSTIFF}, //push egg wait
};

void mains_code(void);
void main(void)
{
    mains_code();
}
// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {
    //FK------------------------------------------------------------------------------------------------
    x = L*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    y = L*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor));
    z = L*cos(theta2motor)-(L*sin(theta3motor))+L;
    dx_raw = (x - x_old)/0.001;
    dx = (dx_raw + dx_old1 + dx_old2)/3.0;
    dy_raw = (y - y_old)/0.001;
    dy = (dy_raw + dy_old1 + dy_old2)/3.0;
    dz_raw = (z - z_old)/0.001;
    dz = (dz_raw + dz_old1 + dz_old2)/3.0;

    //Trajectory------------------------------------------------------------------------------------------------

    //CE: Load next point

    t = mycount*0.001;
    if (state == NUM_POINTS){state = 2;}
    if ((t - t_start) >= t_total){
        xa = x;
        ya = y;
        za = z;
        xb = points[state].x;
        yb = points[state].y;
        zb = points[state].z;
        delx = xb - xa;
        dely = yb - ya;
        delz = zb - za;
        distAB = sqrt((xa-xb)*(xa-xb) + (ya-yb)*(ya-yb) + (za-zb)*(za-zb));
        t_total = distAB/points[state].V;
        t_start = t;
        state++;
    }
    xD = delx*(t-t_start)/t_total + xa;
    yD = dely*(t-t_start)/t_total + ya;
    zD = delz*(t-t_start)/t_total + za;
    if (state-1 == 1){
        xD = xa;
        yD = ya;
        zD = za;
        t_total = 3.0;
    }
    if (state-1 == 6){
        xD = xa;
        yD = ya;
        zD = za;
        t_total = 1.0;
    }


    //Velocity
    W1_raw = (theta1motor - Theta1_old)/0.001;
    W1 = (W1_raw + W1_old1 + W1_old2)/3.0;

    W2_raw = (theta2motor - Theta2_old)/0.001;
    W2 = (W2_raw + W2_old1 + W2_old2)/3.0;

    W3_raw = (theta3motor - Theta3_old)/0.001;
    W3 = (W3_raw + W3_old1 + W3_old2)/3.0;

    // Jacobian Transpose
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    JT_11 = -0.254*sinq1*(cosq3 + sinq2);
    JT_12 = 0.254*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 0.254*cosq1*(cosq2 - sinq3);
    JT_22 = 0.254*sinq1*(cosq2 - sinq3);
    JT_23 = -0.254*(cosq3 + sinq2);
    JT_31 = -0.254*cosq1*sinq3;
    JT_32 = -0.254*sinq1*sinq3;
    JT_33 = -0.254*cosq3;

    JvT[0][0] = JT_11;
    JvT[0][1] = JT_12;
    JvT[0][2] = JT_13;

    JvT[1][0] = JT_21;
    JvT[1][1] = JT_22;
    JvT[1][2] = JT_23;

    JvT[2][0] = JT_31;
    JvT[2][1] = JT_32;
    JvT[2][2] = JT_33;

    //Fric comp------------------------------------------------------------------------------------------------
    if (W1 > 0.1){
        u_fric1 = Vpos1*W1 + Cpos1;
    } else if (W1 < -0.1) {
        u_fric1 = Vneg1*W1 + Cneg1;
    } else {
        u_fric1 = Vhigh1*W1;
    }

    if (W2 > 0.05){
        u_fric2 = Vpos2*W2 + Cpos2;
    } else if (W2 < -0.05) {
        u_fric2 = Vneg2*W2 + Cneg2;
    } else {
        u_fric2 = Vhigh2*W2;
    }

    if (W3 > 0.05){
        u_fric3 = Vpos3*W3 + Cpos3;
    } else if (W3 < -0.05) {
        u_fric3 = Vneg3*W3 + Cneg3;
    } else {
        u_fric3 = Vhigh3*W3;
    }

    //Control Law------------------------------------------------------------------------------------------------
    //Task Space Feedforward

    //rotation line

    rotationMatrix(points[state-1].thz, rotz, 'z');

    matrixTranspose(rotz, rot_NW);

    if (points[state-1].mode == 1){//xyzstiff
        KP_N[0][0] = KPxN;
        KP_N[1][1] = KPyN;
        KP_N[2][2] = KPzN;

        KD_N[0][0] = KDxN;
        KD_N[1][1] = KDyN;
        KD_N[2][2] = KDzN;
    }
    if (points[state-1].mode == 2){//zstiff
        KP_N[0][0] = 400.0;
        KP_N[1][1] = 400.0;
        KP_N[2][2] = KPzN;

        KD_N[0][0] = 20.0;
        KD_N[1][1] = 20.0;
        KD_N[2][2] = KDzN;
    }
    if (points[state-1].mode == 3){//xzstiff
        KP_N[0][0] = KPxN;
        KP_N[1][1] = 0.0;
        KP_N[2][2] = KPzN;

        KD_N[0][0] = KDxN;
        KD_N[1][1] = 0.0;
        KD_N[2][2] = KDzN;
    }
    if (points[state-1].mode == 4){//xystiff
        KP_N[0][0] = KPxN;
        KP_N[1][1] = KPyN;
        KP_N[2][2] = 375.0;

        KD_N[0][0] = KDxN;
        KD_N[1][1] = KDyN;
        KD_N[2][2] = 10.0;
    }
    if (points[state-1].mode == 5){//lowstiff
        KP_N[0][0] = 0.1*KPxN;
        KP_N[1][1] = 0.1*KPyN;
        KP_N[2][2] = 0.1*KPzN;

        KD_N[0][0] = 0.1*KDxN;
        KD_N[1][1] = 0.1*KDyN;
        KD_N[2][2] = 0.1*KDzN;

    }

//    KP_N[0][0] = KPxN;
//    KP_N[1][1] = KPyN;
//    KP_N[2][2] = KPzN;
//
//    KD_N[0][0] = KDxN;
//    KD_N[1][1] = KDyN;
//    KD_N[2][2] = KDzN;
//    x = 0.25;
//    y = 0.15;
//    z = 0.55;
//    xD = 0.45;
//    yD = 0.28;
//    zD = 0.1;

    error_W[0][0] = (xD - x);
    error_W[1][0] = (yD - y);
    error_W[2][0] = (zD - z);

    errordot_W[0][0] = (dxD - dx);
    errordot_W[1][0] = (dyD - dy);
    errordot_W[2][0] = (dzD - dz);

    matrixVectorMultiplication(rot_NW,error_W,error_N); //error in N frame
    matrixVectorMultiplication(KP_N,error_N,term1); // error in N frame with gains

    matrixVectorMultiplication(rot_NW,errordot_W,errordot_N); //dot error in N frame
    matrixVectorMultiplication(KD_N,errordot_N,term2); // dot in N frame with gains

    bigterm_N[0][0] = term1[0][0] + term2[0][0]; //all errors in N frame
    bigterm_N[1][0] = term1[1][0] + term2[1][0]; //all errors in N frame
    bigterm_N[2][0] = term1[2][0] + term2[2][0]; //all errors in N frame

    matrixVectorMultiplication(rotz,bigterm_N,bigterm_W); // all errors in W frame
    matrixVectorMultiplication(JvT,bigterm_W,tauvec); //taus

    ramp = mycount/1000.0;
    if (ramp >= 1.0){ramp = 1.0;}
    *tau1 = ramp*(tauvec[0][0] + ff*u_fric1);
    *tau2 = ramp*(tauvec[1][0] + ff*u_fric2);
    *tau3 = ramp*(tauvec[2][0] + ff*u_fric3);

    //Motor torque limitation(Max: 5 Min: -5)

    if (*tau1 > 5.0){*tau1 = 5.0;}
    if (*tau1 < -5.0){*tau1 = -5.0;}
    if (*tau2 > 5.0){*tau2 = 5.0;}
    if (*tau2 < -5.0){*tau2 = -5.0;}
    if (*tau3 > 5.0){*tau3 = 5.0;}
    if (*tau3 < -5.0){*tau3 = -5.0;}

    // save past states-----------------------------------------

    Theta1_old = theta1motor;
    W1_old2 = W1_old1;
    W1_old1 = W1;

    Theta2_old = theta2motor;
    W2_old2 = W2_old1;
    W2_old1 = W2;

    Theta3_old = theta3motor;
    W3_old2 = W3_old1;
    W3_old1 = W3;

    x_old = x;
    dx_old2 = dx_old1;
    dx_old1 = dx;

    y_old = y;
    dy_old2 = dy_old1;
    dy_old1 = dy;

    z_old = z;
    dz_old2 = dz_old1;
    dz_old1 = dz;

    if ((mycount%50)==0) {
        theta1array[arrayindex] = theta1motor;
        if (arrayindex >= 99) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }
    }

    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }

    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    if (whattoplot == 1){
        Simulink_PlotVar1 = theta1motor;
        Simulink_PlotVar2 = theta2motor;
        Simulink_PlotVar3 = 0.0;
        Simulink_PlotVar4 = thetaD1;
    }
    if (whattoplot == 2){
        Simulink_PlotVar1 = 0.0;
        Simulink_PlotVar2 = 0.0;
        Simulink_PlotVar3 = theta3motor;
        Simulink_PlotVar4 = thetaD2;
    }
    if (whattoplot == 3){
        Simulink_PlotVar1 = err1;
        Simulink_PlotVar2 = err2;
        Simulink_PlotVar3 = err3;
        Simulink_PlotVar4 = 0.0;
    }
    if (whattoplot == 4){
        Simulink_PlotVar1 = thetaD1;
        Simulink_PlotVar2 = thetaD2;
        Simulink_PlotVar3 = thetaD3;
        Simulink_PlotVar4 = 0.0;
    }

    mycount++;
}

void printing(void){
    if (whattoprint == 0) {
        serial_printf(&SerialA, "XYZ %.3f, %.3f, %.3f\n\r\n\n\n", x, y, z);
    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }
}


// For matrix-vector multiplication (3x3 matrix multiplied by 3x1 vector)
// This function gets 3x3 matrix (A) and 3x1 vector (B) as arguments then returns their multiplication C = AB
void matrixVectorMultiplication(float A[][3], float B[][1], float C[][1]) {
    // Initialize result to zero
    for(int i=0; i<3; i++) {
        C[i][0] = 0;
    }

    // Perform multiplication
    for(int i=0; i<3; i++) {
        for(int k=0; k<3; k++) {
            C[i][0] += A[i][k]*B[k][0];
        }
    }
}

// For matrix multiplication (3x3 matrix multiplied by 3x3 matrix)
// This function gets 3x3 matrix (A) and 3x3 matrix (B) as arguments then returns their multiplication C = AB
void matrixMultiplication(float A[][3], float B[][3], float C[][3]) {
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            C[i][j] = 0;
            for(int k=0; k<3; k++) {
                C[i][j] += A[i][k]*B[k][j];
            }
        }
    }
}

// For matrix transpose (3x3 matrix)
// This function gets 3x3 matrix (A) as arguments then returns its transpose A_trans
void matrixTranspose(float A[][3], float A_trans[][3]) {
    // Size of A should be 3x3
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            A_trans[i][j] = A[j][i];
        }
    }
}

// For rotation matrix (3x3 matrix)
// This function gets a scalar theta and a rotation axis type (x, y, z) then returns a rotational matrix
// that rotates around the rotation axis by theta
void rotationMatrix(float theta, float R[3][3], char axis) {
    if (axis == 'x') {
        R[0][0] = 1;
        R[0][1] = 0;
        R[0][2] = 0;
        R[1][0] = 0;
        R[1][1] = cos(theta);
        R[1][2] = -sin(theta);
        R[2][0] = 0;
        R[2][1] = sin(theta);
        R[2][2] = cos(theta);
    }
    else if (axis == 'y') {
        R[0][0] = cos(theta);
        R[0][1] = 0;
        R[0][2] = sin(theta);
        R[1][0] = 0;
        R[1][1] = 1;
        R[1][2] = 0;
        R[2][0] = -sin(theta);
        R[2][1] = 0;
        R[2][2] = cos(theta);
    }
    else if (axis == 'z') {
        R[0][0] = cos(theta);
        R[0][1] = -sin(theta);
        R[0][2] = 0;
        R[1][0] = sin(theta);
        R[1][1] = cos(theta);
        R[1][2] = 0;
        R[2][0] = 0;
        R[2][1] = 0;
        R[2][2] = 1;
    }
}

//for fun traj
//    err1 = thetaD1 - theta1motor;
//    err2 = thetaD2 - theta2motor;
//    err3 = thetaD3 - theta3motor;

//    err1 = thetaD - theta1motor;
//    err2 = thetaD - theta2motor;
//    err3 = thetaD - theta3motor;

//IK
//    d = sqrtf(calc_x*calc_x + calc_y*calc_y + (calc_z - L)*(calc_z - L));
//    r = sqrtf(calc_x*calc_x + calc_y*calc_y);
//    beta = acos((-d*d + 2*L*L)/(2*L*L));
//    alpha = (PI - beta)/2;
//    gamma = atan2(L-calc_z,r);
//
//    DHth1 = atan2(calc_y,calc_x);
//    DHth2 = gamma - alpha;
//    DHth3 = PI - beta;
//
//    Mth1 = DHth1;
//    Mth2 = HALFPI + DHth2;
//    Mth3 = DHth3 + Mth2 - HALFPI;

//    pulse trajectory
    //check every 4 seconds
//    if ((mycount % 4000 == 0) && (thetaD > -0.1) && (thetaD < 0.1)){ //
//        thetaD = PI/6.0; //if thetaD is near 0, set to pi/6
//    } else if ((mycount % 4000 == 0) && (thetaD > PI/6.0 - 0.1) && (thetaD < PI/6.0 + 0.1)){
//        thetaD = 0; //if at pi/6, set to 0
//    }

//fun trajectory xyz
//    tmod = mycount % 6000;
//    t = tmod*0.001;
//    funx = 0.1*cos(t*TWOPI/6.0) + .3;
//    funy = 0.1*sin(2*t*TWOPI/6.0);
//    funz = 0.1*sin(2*t*TWOPI/6.0) + 0.4;

//IK for fun trajectory
//    d = sqrtf(funx*funx + funy*funy + (funz - L)*(funz - L));
//    r = sqrtf(funx*funx + funy*funy);
//    beta = acos((-d*d + 2*L*L)/(2*L*L));
//    alpha = (PI - beta)/2;
//    gamma = atan2(L-funz,r);
//    DHth1 = atan2(funy,funx);
//    DHth2 = gamma - alpha;
//    DHth3 = PI - beta;
//    thetaD1 = DHth1;
//    thetaD2 = HALFPI + DHth2;
//    thetaD3 = DHth3 + Mth2 - HALFPI;




// integral calc
//    if (err1 < errband && err1 > -errband){
//        I1k = I1kprev + (err1 + err1prev)/2*0.001;
//    } else {I1k = 0.0;}
//    if (err2 < errband && err2 > -errband){
//        I2k = I2kprev + (err2 + err2prev)/2*0.001;
//    } else {I2k = 0.0;}
//    if (err3 < errband && err3 > -errband){
//        I3k = I3kprev + (err3 + err3prev)/2*0.001;
//    } else {I3k = 0.0;}

//typedef struct steptraj_s {
//    long double b[5];
//    long double a[5];
//    long double xk[5];
//    long double yk[5];
//    float qd_old;
//    float qddot_old;
//    int size;
//} steptraj_t;
//
//steptraj_t traj1 = {8.7501303587660538e-08L,3.5000521435064215e-07L,5.2500782152596325e-07L,3.5000521435064215e-07L,8.7501303587660538e-08L,
//                        1.0000000000000000e+00L,-3.8624078624078622e+00L,5.5943229358462769e+00L,-3.6012594820435488e+00L,8.6934580862599187e-01L,
//                        0,0,0,0,0,
//                        0,0,0,0,0,
//                        0,
//                        0,
//                        5};
//steptraj_t traj2 = {8.7501303587660538e-08L,3.5000521435064215e-07L,5.2500782152596325e-07L,3.5000521435064215e-07L,8.7501303587660538e-08L,
//                        1.0000000000000000e+00L,-3.8624078624078622e+00L,5.5943229358462769e+00L,-3.6012594820435488e+00L,8.6934580862599187e-01L,
//                        0,0,0,0,0,
//                        0,0,0,0,0,
//                        0,
//                        0,
//                        5};
//
//
//// this function must be called every 1ms.
//void implement_discrete_tf(steptraj_t *traj, float step, float *qd, float *qd_dot, float *qd_ddot) {
//    int i = 0;
//
//    traj->xk[0] = step;
//    traj->yk[0] = traj->b[0]*traj->xk[0];
//    for (i = 1;i<traj->size;i++) {
//        traj->yk[0] = traj->yk[0] + traj->b[i]*traj->xk[i] - traj->a[i]*traj->yk[i];
//    }
//
//    for (i = (traj->size-1);i>0;i--) {
//        traj->xk[i] = traj->xk[i-1];
//        traj->yk[i] = traj->yk[i-1];
//    }
//
//    *qd = traj->yk[0];
//    *qd_dot = (*qd - traj->qd_old)*1000;  //0.001 sample period
//    *qd_ddot = (*qd_dot - traj->qddot_old)*1000;
//
//    traj->qd_old = *qd;
//    traj->qddot_old = *qd_dot;
//}
