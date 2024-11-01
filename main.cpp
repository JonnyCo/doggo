#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "BezierCurve.h"
#include "MotorShield.h" 
#include "HardwareSetup.h"
#include "Matrix.h"
#include "MatrixMath.h"

#define BEZIER_ORDER_FOOT    7
#define NUM_INPUTS (21 + 2*(BEZIER_ORDER_FOOT+1)) //Maybe 
#define NUM_OUTPUTS 37

#define PULSE_TO_RAD (2.0f*3.14159f / 1200.0f)

// Initializations
Serial pc(USBTX, USBRX);    // USB Serial Terminal
ExperimentServer server;    // Object that lets us communicate with MATLAB
Timer t;                    // Timer to measure elapsed time of experiment

QEI encoderA(PE_9,PE_11, NC, 1200, QEI::X4_ENCODING);  // MOTOR A ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING);  // MOTOR B ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderC(PC_6, PC_7, NC, 1200, QEI::X4_ENCODING);  // MOTOR C ENCODER (no index, 1200 counts/rev, Quadrature encoding)
QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING);// MOTOR D ENCODER (no index, 1200 counts/rev, Quadrature encoding)

MotorShield motorShield(24000); //initialize the motor shield with a period of 12000 ticks or ~20kHZ
Ticker currentLoop;

//Matrices
Matrix MassMatrix(2,2);
Matrix Jacobian_Front(2,2);
Matrix JacobianT_Front(2,2);
Matrix InverseMassMatrix(2,2);
Matrix temp_product_Front(2,2);
Matrix Lambda_Front(2,2);

//Matrix MassMatrix_Back(2,2);
Matrix Jacobian_Back(2,2);
Matrix JacobianT_Back(2,2);
//Matrix InverseMassMatrix_Back(2,2);
Matrix temp_product_Back(2,2);
Matrix Lambda_Back(2,2);

// Variables for q1
float current1;
float current_des1 = 0;
float prev_current_des1 = 0;
float current_int1 = 0;
float angle1;
float velocity1;
float duty_cycle1;
float angle1_init;

// Variables for q2
float current2;
float current_des2 = 0;
float prev_current_des2 = 0;
float current_int2 = 0;
float angle2;
float velocity2;
float duty_cycle2;
float angle2_init;

// Variables for q3
float current3;
float current_des3 = 0;
float prev_current_des3 = 0;
float current_int3 = 0;
float angle3;
float velocity3;
float duty_cycle3;
float angle3_init;

// Variables for q4
float current4;
float current_des4 = 0;
float prev_current_des4 = 0;
float current_int4 = 0;
float angle4;
float velocity4;
float duty_cycle4;
float angle4_init;

// Fixed kinematic parameters
const float l_OA=.011; 
const float l_OB=.042; 
const float l_AC=.096; 
const float l_DE=.091;
const float m1 =.0393 + .2;
const float m2 =.0368; 
const float m3 = .00783;
const float m4 = .0155;
const float I1 = 0.0000251;  //25.1 * 10^-6;
const float I2 = 0.0000535;  //53.5 * 10^-6;
const float I3 = 0.00000925; //9.25 * 10^-6;
const float I4 = 0.0000222;  //22.176 * 10^-6;
const float l_O_m1=0.032;
const float l_B_m2=0.0344; 
const float l_A_m3=0.0622;
const float l_C_m4=0.0610;
const float N = 18.75;
const float Ir = 0.0035/pow(N,2);

// Timing parameters
float current_control_period_us = 200.0f;     // 5kHz current control loop
float impedance_control_period_us = 1000.0f;  // 1kHz impedance control loop
float start_period, traj_period, end_period;

// Control parameters
float current_Kp_Front = 4.0f;         
float current_Ki_Front = 0.4f;           
float current_int_max_Front = 3.0f;       
float duty_max_Front;      
float K_xx_Front;
float K_yy_Front;
float K_xy_Front;
float D_xx_Front;
float D_xy_Front;
float D_yy_Front;

float current_Kp_Back = 4.0f;         
float current_Ki_Back = 0.4f;           
float current_int_max_Back = 3.0f;       
float duty_max_Back;      
float K_xx_Back;
float K_yy_Back;
float K_xy_Back;
float D_xx_Back;
float D_xy_Back;
float D_yy_Back;

// Objects for 7th order Cartesian foot trajectory
BezierCurve rDesFoot_bez_Front(2,BEZIER_ORDER_FOOT);
BezierCurve rDesFoot_bez_Back(2,BEZIER_ORDER_FOOT);

// Model parameters
float supply_voltage = 12;     // motor supply voltage
float R = 2.0f;                // motor resistance
float k_t = 0.18f;             // motor torque constant
float nu = 0.0005;             // motor viscous friction

// Current control interrupt function
void CurrentLoop()
{
    // This loop sets the motor voltage commands using PI current controllers with feedforward terms.
    //use the motor shield as follows:
    //motorShield.motorAWrite(DUTY CYCLE, DIRECTION), DIRECTION = 0 is forward, DIRECTION =1 is backwards.
        
    //Motor A    
    current1 = -(((float(motorShield.readCurrentA())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;                                  // measure velocity        
    float err_c1 = current_des1 - current1;                                             // current errror
    current_int1 += err_c1;                                                             // integrate error
    current_int1 = fmaxf( fminf(current_int1, current_int_max_Front), -current_int_max_Front);      // anti-windup
    float ff1 = R*current_des1 + k_t*velocity1;                                         // feedforward terms
    duty_cycle1 = (ff1 + current_Kp_Front*err_c1 + current_Ki_Front*current_int1)/supply_voltage;   // PI current controller
    
    float absDuty1 = abs(duty_cycle1);
    if (absDuty1 > duty_max_Front) {
        duty_cycle1 *= duty_max_Front / absDuty1;
        absDuty1 = duty_max_Front;
    }    
    if (duty_cycle1 < 0) { // backwards
        motorShield.motorAWrite(absDuty1, 1);
    } else { // forwards
        motorShield.motorAWrite(absDuty1, 0);
    }             
    prev_current_des1 = current_des1; 
    
    //Motor B
    current2  = -(((float(motorShield.readCurrentB())/65536.0f)*30.0f)-15.0f);       // measure current
    velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
    float err_c2 = current_des2 - current2;                                             // current error
    current_int2 += err_c2;                                                             // integrate error
    current_int2 = fmaxf( fminf(current_int2, current_int_max_Front), -current_int_max_Front);      // anti-windup   
    float ff2 = R*current_des2 + k_t*velocity2;                                         // feedforward terms
    duty_cycle2 = (ff2 + current_Kp_Front*err_c2 + current_Ki_Front*current_int2)/supply_voltage;   // PI current controller
    
    float absDuty2 = abs(duty_cycle2);
    if (absDuty2 > duty_max_Front) {
        duty_cycle2 *= duty_max_Front / absDuty2;
        absDuty2 = duty_max_Front;
    }    
    if (duty_cycle2 < 0) { // backwards
        motorShield.motorBWrite(absDuty2, 1);
    } else { // forwards
        motorShield.motorBWrite(absDuty2, 0);
    }             
    prev_current_des2 = current_des2; 

    //Motor C   
    current3 = -(((float(motorShield.readCurrentC())/65536.0f)*30.0f)-15.0f);           // measure current
    velocity3 = encoderC.getVelocity() * PULSE_TO_RAD;                                  // measure velocity        
    float err_c3 = current_des3 - current3;                                             // current errror
    current_int3 += err_c3;                                                             // integrate error
    current_int3 = fmaxf( fminf(current_int3, current_int_max_Back), -current_int_max_Back);      // anti-windup
    float ff3 = R*current_des3 + k_t*velocity3;                                         // feedforward terms
    duty_cycle3 = (ff3 + current_Kp_Back*err_c3 + current_Ki_Back*current_int3)/supply_voltage;   // PI current controller
    
    float absDuty3 = abs(duty_cycle3);
    if (absDuty3 > duty_max_Back) {
        duty_cycle3 *= duty_max_Back / absDuty3;
        absDuty3 = duty_max_Back;
    }    
    if (duty_cycle3 < 0) { // backwards
        motorShield.motorCWrite(absDuty3, 1);
    } else { // forwards
        motorShield.motorCWrite(absDuty3, 0);
    }             
    prev_current_des3 = current_des3; 
    
    //Motor D
    current4  = -(((float(motorShield.readCurrentD())/65536.0f)*30.0f)-15.0f);       // measure current
    velocity4 = encoderD.getVelocity() * PULSE_TO_RAD;                                  // measure velocity  
    float err_c4 = current_des4 - current4;                                             // current error
    current_int4 += err_c4;                                                             // integrate error
    current_int4 = fmaxf( fminf(current_int4, current_int_max_Back), -current_int_max_Back);      // anti-windup   
    float ff4 = R*current_des4 + k_t*velocity4;                                         // feedforward terms
    duty_cycle4 = (ff4 + current_Kp_Back*err_c4 + current_Ki_Back*current_int4)/supply_voltage;   // PI current controller
    
    float absDuty4 = abs(duty_cycle4);
    if (absDuty4 > duty_max_Back) {
        duty_cycle4 *= duty_max_Back / absDuty4;
        absDuty4 = duty_max_Back;
    }    
    if (duty_cycle4 < 0) { // backwards
        motorShield.motorDWrite(absDuty4, 1);
    } else { // forwards
        motorShield.motorDWrite(absDuty4, 0);
    }             
    prev_current_des4 = current_des4; 
}

void setInputParams(const float* input_params) {
    start_period    = input_params[0];  // First buffer time, before trajectory
    traj_period     = input_params[1];  // Trajectory time/length
    end_period      = input_params[2];  // Second buffer time, after trajectory

    angle1_init     = input_params[3];  // Initial angle for q1 (rad)
    angle2_init     = input_params[4];  // Initial angle for q2 (rad)
    angle3_init     = input_params[5];
    angle4_init     = input_params[6];

    K_xx_Front      = input_params[7];  // Foot stiffness N/m
    K_yy_Front      = input_params[8];
    K_xy_Front      = input_params[9];
    D_xx_Front      = input_params[10]; // Foot damping N/(m/s)
    D_yy_Front      = input_params[11];
    D_xy_Front      = input_params[12];

    K_xx_Back       = input_params[13]; // Foot stiffness N/m
    K_yy_Back       = input_params[14];
    K_xy_Back       = input_params[15];
    D_xx_Back       = input_params[16]; // Foot damping N/(m/s)
    D_yy_Back       = input_params[17];
    D_xy_Back       = input_params[18];

    duty_max_Front  = input_params[19]; // Maximum duty factor
    duty_max_Back   = input_params[20];
}

void setGainsFromInput(const float* input_params) {
    // Front gains
    K_xx_Front = input_params[7];    // Foot stiffness N/m
    K_yy_Front = input_params[8];    
    K_xy_Front = input_params[9];   
    D_xx_Front = input_params[10];   
    D_yy_Front = input_params[11];    
    D_xy_Front = input_params[12];   

    // Back gains
    K_xx_Back = input_params[13];    // Foot stiffness N/m
    K_yy_Back = input_params[14];    
    K_xy_Back = input_params[15];   
    D_xx_Back = input_params[16];   
    D_yy_Back = input_params[17];   
    D_xy_Back = input_params[18];   
}


void setFootPoints(float* foot_pts, const float* input_params, int startIndex, int numPoints) {
    for (int i = 0; i < numPoints; i++) {
        foot_pts[i] = input_params[startIndex + i];
    }
    rDesFoot_bez_Front.setPoints(foot_pts);
    rDesFoot_bez_Back.setPoints(foot_pts);
}

void setupExperiment() {
    t.reset();
    t.start();
    encoderA.reset();
    encoderB.reset();
    encoderC.reset();
    encoderD.reset();

    motorShield.motorAWrite(0, 0); // Turn motor A off
    motorShield.motorBWrite(0, 0); // Turn motor B off
    motorShield.motorCWrite(0, 0); // Turn motor C off
    motorShield.motorDWrite(0, 0); // Turn motor D off
}

void setGainsFront(float K_xx, float K_yy, float D_xx, float D_yy, float K_xy, float D_xy) {
    K_xx_Front = K_xx;
    K_yy_Front = K_yy;
    D_xx_Front = D_xx;
    D_yy_Front = D_yy;
    K_xy_Front = K_xy;
    D_xy_Front = D_xy;
}

void setGainsBack(float K_xx, float K_yy, float D_xx, float D_yy, float K_xy, float D_xy) {
    K_xx_Back = K_xx;
    K_yy_Back = K_yy;
    D_xx_Back = D_xx;
    D_yy_Back = D_yy;
    K_xy_Back = K_xy;
    D_xy_Back = D_xy;
}

void calculateDynamics(float M11, float M12, float M22,
                       float Jx_th1, float Jx_th2, 
                       float Jy_th1, float Jy_th2, 
                       float Jx_th3, float Jx_th4, 
                       float Jy_th3, float Jy_th4) {
    // Clear and populate Mass Matrix
    MassMatrix.Clear();
    MassMatrix << M11 << M12
               << M12 << M22;

    // Populate Jacobian matrix for Front
    Jacobian_Front.Clear();
    Jacobian_Front << Jx_th1 << Jx_th2
                   << Jy_th1 << Jy_th2;

    // Populate Jacobian matrix for Back
    Jacobian_Back.Clear();
    Jacobian_Back << Jx_th3 << Jx_th4
                  << Jy_th3 << Jy_th4;

    // Calculate Lambda matrix for Front
    JacobianT_Front = MatrixMath::Transpose(Jacobian_Front);
    InverseMassMatrix = MatrixMath::Inv(MassMatrix);
    temp_product_Front = Jacobian_Front * InverseMassMatrix * JacobianT_Front;
    Lambda_Front = MatrixMath::Inv(temp_product_Front); 

    // Calculate Lambda matrix for Back
    JacobianT_Back = MatrixMath::Transpose(Jacobian_Back);
    temp_product_Back = Jacobian_Back * InverseMassMatrix * JacobianT_Back;
    Lambda_Back = MatrixMath::Inv(temp_product_Back); 
}

void cleanupAfterExperiment() {
    // Indicate that the experiment is complete
    server.setExperimentComplete();
    
    // Detach the current loop
    currentLoop.detach();
    
    // Turn off all motors
    motorShield.motorAWrite(0, 0); // Turn motor A off
    motorShield.motorBWrite(0, 0); // Turn motor B off
    motorShield.motorCWrite(0, 0); // Turn motor C off
    motorShield.motorDWrite(0, 0); // Turn motor D off
}

int main (void)
{
    // Link the terminal with our server and start it up
    server.attachTerminal(pc);
    server.init();
    
    // Continually get input from MATLAB and run experiments
    float input_params[NUM_INPUTS];
    
    
    while(1) {
        
        // If there are new inputs, this code will run
        
        if (server.getParams(input_params,NUM_INPUTS)) {

            //Set input parameters
            setInputParams(input_params);

            // Get foot trajectory points
            float foot_pts[2 * (BEZIER_ORDER_FOOT + 1)];
            setFootPoints(foot_pts, input_params, 21, 2 * (BEZIER_ORDER_FOOT + 1));

            // Attach current loop interrupt
            currentLoop.attach_us(CurrentLoop,current_control_period_us);
                        
            // Setup experiment
            setupExperiment();

                         
            // Run experiment
            while( t.read() < start_period + traj_period + end_period) { 
                 
                // Read encoders to get motor states
                angle1 = encoderA.getPulses() * PULSE_TO_RAD + angle1_init;       
                velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;
                 
                angle2 = encoderB.getPulses() * PULSE_TO_RAD + angle2_init;       
                velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;    

                angle3 = encoderC.getPulses() * PULSE_TO_RAD + angle3_init;       
                velocity3 = encoderC.getVelocity() * PULSE_TO_RAD;
                 
                angle4 = encoderD.getPulses() * PULSE_TO_RAD + angle4_init;       
                velocity4 = encoderD.getVelocity() * PULSE_TO_RAD;        
                
                const float th1 = angle1;
                const float th2 = angle2;
                const float th3 = angle3;
                const float th4 = angle4;

                const float dth1= velocity1;
                const float dth2= velocity2;
                const float dth3= velocity3;
                const float dth4= velocity4;
 
                // Calculate the Jacobian
                float Jx_th1 = l_AC*cos(th1+th2) + l_DE*cos(th1) + l_OB*cos(th1);
                float Jx_th2 = l_AC*cos(th1+th2);
                float Jy_th1 = l_AC*sin(th1+th2) + l_DE*sin(th1) + l_OB*sin(th1);//Maybe????
                float Jy_th2 = l_AC*sin(th1+th2);

                float Jx_th3 = l_AC*cos(th3+th4) + l_DE*cos(th3) + l_OB*cos(th3);
                float Jx_th4 = l_AC*cos(th3+th4);
                float Jy_th3 = l_AC*sin(th3+th4) + l_DE*sin(th3) + l_OB*sin(th3);
                float Jy_th4 = l_AC*sin(th3+th4);
                                
                // Calculate the forward kinematics (position and velocity)
                float xFoot_Front = l_AC*sin(th1+th2) + l_DE*sin(th1) + l_OB*sin(th1);
                float yFoot_Front = -l_AC*cos(th1+th2) - l_DE*cos(th1) - l_OB*cos(th1);
                float dxFoot_Front = dth1*(l_AC*cos(th1+th2) + l_DE*cos(th1) + l_OB*cos(th1)) + dth2*(l_AC*cos(th1+th2));
                float dyFoot_Front = dth1*(l_AC*sin(th1+th2) + l_DE*sin(th1) + l_OB*sin(th1)) + dth2*(l_AC*sin(th1+th2));       

                float xFoot_Back = l_AC*sin(th3+th4) + l_DE*sin(th3) + l_OB*sin(th3);
                float yFoot_Back = -l_AC*cos(th3+th4) - l_DE*cos(th3) - l_OB*cos(th3);
                float dxFoot_Back = dth3*(l_AC*cos(th3+th4) + l_DE*cos(th3) + l_OB*cos(th3)) + dth4*(l_AC*cos(th3+th4));
                float dyFoot_Back = dth3*(l_AC*sin(th3+th4) + l_DE*sin(th3) + l_OB*sin(th3)) + dth4*(l_AC*sin(th3+th4));    

                // Set gains based on buffer and traj times, then calculate desired x,y from Bezier trajectory at current time if necessary
                float teff  = 0;
                float vMult = 0;
                if( t < start_period) {
                    if (K_xx_Front > 0 || K_yy_Front > 0) {
                        setGainsFront(100, 100, 5, 5, 0, 0);
                    }

                    if (K_xx_Back > 0 || K_yy_Back > 0) {
                        setGainsBack(100, 100, 5, 5, 0, 0);
                    }
                    teff = 0;

                }
                else if (t < start_period + traj_period)
                {
                    setGainsFromInput(input_params);

                    teff = (t-start_period);
                    vMult = 1;
                }
                else
                {
                    teff = traj_period;
                    vMult = 0;
                }
                
                // Get desired foot positions and velocities
                float rDesFoot_Front[2] , vDesFoot_Front[2];
                float rDesFoot_Back[2] , vDesFoot_Back[2];
                rDesFoot_bez_Front.evaluate(teff/traj_period,rDesFoot_Front);
                rDesFoot_bez_Front.evaluateDerivative(teff/traj_period,vDesFoot_Front);
                rDesFoot_bez_Back.evaluate(teff/traj_period,rDesFoot_Back);
                rDesFoot_bez_Back.evaluateDerivative(teff/traj_period,vDesFoot_Back);

                vDesFoot_Front[0]/=traj_period;
                vDesFoot_Front[1]/=traj_period;
                vDesFoot_Front[0]*=vMult;
                vDesFoot_Front[1]*=vMult;

                vDesFoot_Back[0]/=traj_period;
                vDesFoot_Back[1]/=traj_period;
                vDesFoot_Back[0]*=vMult;
                vDesFoot_Back[1]*=vMult;
                
                // Calculate the inverse kinematics (joint positions and velocities) for desired joint angles           
                float xFoot_inv_Front = -rDesFoot_Front[0];
                float yFoot_inv_Front = rDesFoot_Front[1];                
                float l_OE_Front = sqrt( (pow(xFoot_inv_Front,2) + pow(yFoot_inv_Front,2)) );
                float alpha_Front = abs(acos( (pow(l_OE_Front,2) - pow(l_AC,2) - pow((l_OB+l_DE),2))/(-2.0f*l_AC*(l_OB+l_DE)) ));
                float th2_des = -(3.14159f - alpha_Front); 
                float th1_des = -((3.14159f/2.0f) + atan2(yFoot_inv_Front,xFoot_inv_Front) - abs(asin( (l_AC/l_OE_Front)*sin(alpha_Front) )));

                float xFoot_inv_Back = rDesFoot_Back[0];
                float yFoot_inv_Back = rDesFoot_Back[1];                
                float l_OE_Back = sqrt( (pow(xFoot_inv_Back,2) + pow(yFoot_inv_Back,2)) );
                float alpha_Back = abs(acos( (pow(l_OE_Back,2) - pow(l_AC,2) - pow((l_OB+l_DE),2))/(-2.0f*l_AC*(l_OB+l_DE)) ));
                float th4_des = -(3.14159f - alpha_Back); 
                float th3_des = -((3.14159f/2.0f) + atan2(yFoot_inv_Back,xFoot_inv_Back) - abs(asin( (l_AC/l_OE_Back)*sin(alpha_Back) )));
                
                float dd_Front = (Jx_th1*Jy_th2 - Jx_th2*Jy_th1);
                float dth1_des = (1.0f/dd_Front) * (  Jy_th2*vDesFoot_Front[0] - Jx_th2*vDesFoot_Front[1] );
                float dth2_des = (1.0f/dd_Front) * ( -Jy_th1*vDesFoot_Front[0] + Jx_th1*vDesFoot_Front[1] );

                float dd_Back = (Jx_th3*Jy_th4 - Jx_th4*Jy_th3);
                float dth3_des = (1.0f/dd_Back) * (  Jy_th4*vDesFoot_Back[0] - Jx_th4*vDesFoot_Back[1] );
                float dth4_des = (1.0f/dd_Back) * ( -Jy_th3*vDesFoot_Back[0] + Jx_th3*vDesFoot_Back[1] );
        
                // Calculate error variables
                float e_x_Front = rDesFoot_Front[0] - xFoot_Front;
                float e_y_Front = rDesFoot_Front[1] - yFoot_Front;
                float de_x_Front = vDesFoot_Front[0] - dxFoot_Front;
                float de_y_Front = vDesFoot_Front[1] - dyFoot_Front;

                float e_x_Back = rDesFoot_Back[0] - xFoot_Back;
                float e_y_Back = rDesFoot_Back[1] - yFoot_Back;
                float de_x_Back = vDesFoot_Back[0] - dxFoot_Back;
                float de_y_Back = vDesFoot_Back[1] - dyFoot_Back;

                // Calculate virtual force on foot
                float fx_Front = K_xx_Front*e_x_Front + K_xy_Front*e_y_Front + D_xx_Front*de_x_Front + D_xy_Front*de_y_Front;
                float fy_Front = K_xy_Front*e_x_Front + K_yy_Front*e_y_Front + D_xy_Front*de_x_Front + D_yy_Front*de_y_Front;

                float fx_Back = K_xx_Back*e_x_Back + K_xy_Back*e_y_Back + D_xx_Back*de_x_Back + D_xy_Back*de_y_Back;
                float fy_Back = K_xy_Back*e_x_Back + K_yy_Back*e_y_Back + D_xy_Back*de_x_Back + D_yy_Back*de_y_Back;

                // Calculate mass matrix elements
                float M11 = I1 + I2 + I3 + I4 + Ir + Ir*pow(N,2) + pow(l_AC,2)*m4 + pow(l_A_m3,2)*m3 + pow(l_B_m2,2)*m2 + pow(l_C_m4,2)*m4 + pow(l_OA,2)*m3 + pow(l_OB,2)*m2 + pow(l_OA,2)*m4 + pow(l_O_m1,2)*m1 + 2*l_C_m4*l_OA*m4 + 2*l_AC*l_C_m4*m4*cos(th2) + 2*l_AC*l_OA*m4*cos(th2) + 2*l_A_m3*l_OA*m3*cos(th2) + 2*l_B_m2*l_OB*m2*cos(th2); 
                float M12 = I2 + I3 + pow(l_AC,2)*m4 + pow(l_A_m3,2)*m3 + pow(l_B_m2,2)*m2 + Ir*N + l_AC*l_C_m4*m4*cos(th2) + l_AC*l_OA*m4*cos(th2) + l_A_m3*l_OA*m3*cos(th2) + l_B_m2*l_OB*m2*cos(th2); 
                float M22 = Ir*pow(N,2) + m4*pow(l_AC,2) + m3*pow(l_A_m3,2) + m2*pow(l_B_m2,2) + I2 + I3;
                
                //Populate Matrices
                calculateDynamics(M11, M12, M22,Jx_th1, Jx_th2, Jy_th1, Jy_th2, Jx_th3, Jx_th4, Jy_th3, Jy_th4);
                    
                // Pull elements of Lambda matrix
                float L11_Front = Lambda_Front.getNumber(1,1);
                float L12_Front = Lambda_Front.getNumber(1,2);
                float L21_Front = Lambda_Front.getNumber(2,1);
                float L22_Front = Lambda_Front.getNumber(2,2);    

                float L11_Back = Lambda_Back.getNumber(1,1);
                float L12_Back = Lambda_Back.getNumber(1,2);
                float L21_Back = Lambda_Back.getNumber(2,1);
                float L22_Back = Lambda_Back.getNumber(2,2);            
                                
                // Set desired currents
                float tau1 = fx_Front*L11_Front*Jx_th1+fy_Front*L12_Front*Jy_th1;
                float tau2 = fx_Front*L21_Front*Jx_th2+fy_Front*L22_Front*Jy_th2;
                float tau3 = fx_Back*L11_Back*Jx_th3+fy_Back*L12_Back*Jy_th3;
                float tau4 = fx_Back*L21_Back*Jx_th4+fy_Back*L22_Back*Jy_th4;

                current_des1 = tau1/k_t;      
                current_des2 = tau2/k_t;  
                current_des3 = tau3/k_t;      
                current_des4 = tau4/k_t;  

                // Form output to send to MATLAB     
                float output_data[NUM_OUTPUTS];
                // current time
                output_data[0] = t.read();
                // motor 1 state
                output_data[1] = angle1;
                output_data[2] = velocity1;  
                output_data[3] = current1;
                output_data[4] = current_des1;
                output_data[5] = duty_cycle1;
                // motor 2 state
                output_data[6] = angle2;
                output_data[7] = velocity2;
                output_data[8] = current2;
                output_data[9] = current_des2;
                output_data[10]= duty_cycle2;
                // motor 3 state
                output_data[11] = angle3;
                output_data[12] = velocity3;  
                output_data[13] = current3;
                output_data[14] = current_des3;
                output_data[15] = duty_cycle3;
                // motor 4 state
                output_data[16] = angle4;
                output_data[17] = velocity4;
                output_data[18] = current4;
                output_data[19] = current_des4;
                output_data[20]= duty_cycle4;
                // foot state
                //front foot
                output_data[21] = xFoot_Front;
                output_data[22] = yFoot_Front;
                output_data[23] = dxFoot_Front;
                output_data[24] = dyFoot_Front;
                output_data[25] = rDesFoot_Front[0];
                output_data[26] = rDesFoot_Front[1];
                output_data[27] = vDesFoot_Front[0];
                output_data[28] = vDesFoot_Front[1];
                //back foot
                output_data[29] = xFoot_Back;
                output_data[30] = yFoot_Back;
                output_data[31] = dxFoot_Back;
                output_data[32] = dyFoot_Back;
                output_data[33] = rDesFoot_Back[0];
                output_data[34] = rDesFoot_Back[1];
                output_data[35] = vDesFoot_Back[0];
                output_data[36] = vDesFoot_Back[1];
                
                // Send data to MATLAB
                server.sendData(output_data,NUM_OUTPUTS);

                wait_us(impedance_control_period_us);   
            }
            
            // Cleanup after experiment
            cleanupAfterExperiment();

        
        } // end if
        
    } // end while
    
} // end main