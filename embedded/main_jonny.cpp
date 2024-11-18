// Problem still exist in the interpolation I believe
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

#define NUM_INPUTS 34  
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

Matrix Jacobian_Back(2,2);
Matrix JacobianT_Back(2,2);
Matrix temp_product_Back(2,2);
Matrix Lambda_Back(2,2);

// Variables for qs
float current1, current2, current3, current4;
float current_des1 = 0, current_des2 = 0, current_des3 = 0, current_des4 = 0;
float prev_current_des1 = 0, prev_current_des2 = 0, prev_current_des3 = 0, prev_current_des4 = 0;
float current_int1 = 0, current_int2 = 0, current_int3 = 0, current_int4 = 0;
float angle1, angle2, angle3, angle4;
float velocity1, velocity2, velocity3, velocity4;
float duty_cycle1, duty_cycle2, duty_cycle3, duty_cycle4;
float angle1_init = 0.0;
float angle2_init = -3.14/2;
float angle3_init = 0.0;
float angle4_init = -3.14/2;
float curr_time = 0;


//Desired variables
float th1_des, th2_des, th3_des, th4_des;
float dth1_des, dth2_des, dth3_des, dth4_des;

float q1_des[5];
float q2_des[5];
float q3_des[5];
float q4_des[5];


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
float start_period, traj_period, end_period, total_experiment_time; 
Timer experiment_timer; 

// Control parameters
float current_Kp_Front = 4.0f;         
float current_Ki_Front = 0.4f;           
float current_int_max_Front = 3.0f;       
float duty_max_Front = 0.4;      

float current_Kp_Back = 4.0f;         
float current_Ki_Back = 0.4f;           
float current_int_max_Back = 3.0f;       
float duty_max_Back = 0.4;     // look into depricating 

float K_h, K_k, D_h, D_k;


int cycles;
float cycle_period;

// Model parameters
float supply_voltage = 12;     // motor supply voltage
float R = 2.0f;                // motor resistance
float k_t = 0.18f;             // motor torque constant
float nu = 0.0005;             // motor viscous friction


// Current control interrupt function
void CurrentLoop(){
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

void setInputParams(const float* input_params, float* q1_des, float* q2_des, float* q3_des, float* q4_des) {
    // Extract parameters in the order given in input_params array
    cycles          = input_params[0];
    start_period    = input_params[1];  // First buffer time, before trajectory
    traj_period     = input_params[2];
    cycle_period    = input_params[2]; // Trajectory time/length
    end_period      = input_params[3];  // Second buffer time, after trajectory

    angle1_init     = input_params[4];  // Initial angle for q1 (rad)
    angle2_init     = input_params[5];  // Initial angle for q2 (rad)
    angle3_init     = input_params[6];  // Initial angle for q3 (rad)
    angle4_init     = input_params[7];  // Initial angle for q4 (rad)

    K_h             = input_params[8];  // Hip stiffness
    K_k             = input_params[9];  // Knee stiffness
    D_h             = input_params[10]; // Hip damping
    D_k             = input_params[11]; // Knee damping
    
    duty_max_Front  = input_params[12]; // Maximum duty cycle for front
    duty_max_Back   = input_params[13]; // Maximum duty cycle for back

    // Extract desired angles
    for (int i = 0; i < 5; i++) { // 5 trajectory points for each joint
        q1_des[i] = input_params[14 + i * 4];     // q1_des_1, q1_des_2, ..., q1_des_5
        q2_des[i] = input_params[15 + i * 4];     // q2_des_1, q2_des_2, ..., q2_des_5
        q3_des[i] = input_params[16 + i * 4];     // q3_des_1, q3_des_2, ..., q3_des_5
        q4_des[i] = input_params[17 + i * 4];     // q4_des_1, q4_des_2, ..., q4_des_5
    }
}


// Removed lots of code from here

void setInterpPos(float* q1_des, float* q2_des, float* q3_des, float* q4_des, float cycle_period, float curr_time) {

    // Calculate the time fraction into the current cycle
    float cycle_fraction = fmod(curr_time, cycle_period) / cycle_period;

    // Determine the segment based on the cycle_fraction (5 segments total)
    int segment = static_cast<int>(cycle_fraction * 5);

    // Calculate the local fraction within the current segment
    float local_fraction = (cycle_fraction * 5) - segment;

    // Interpolate within each segment
    switch (segment) {
        case 0:
            th1_des = q1_des[0] + local_fraction * (q1_des[1] - q1_des[0]);
            th2_des = q2_des[0] + local_fraction * (q2_des[1] - q2_des[0]);
            th3_des = q3_des[0] + local_fraction * (q3_des[1] - q3_des[0]);
            th4_des = q4_des[0] + local_fraction * (q4_des[1] - q4_des[0]);
            break;
        case 1:
            th1_des = q1_des[1] + local_fraction * (q1_des[2] - q1_des[1]);
            th2_des = q2_des[1] + local_fraction * (q2_des[2] - q2_des[1]);
            th3_des = q3_des[1] + local_fraction * (q3_des[2] - q3_des[1]);
            th4_des = q4_des[1] + local_fraction * (q4_des[2] - q4_des[1]);
            break;
        case 2:
            th1_des = q1_des[2] + local_fraction * (q1_des[3] - q1_des[2]);
            th2_des = q2_des[2] + local_fraction * (q2_des[3] - q2_des[2]);
            th3_des = q3_des[2] + local_fraction * (q3_des[3] - q3_des[2]);
            th4_des = q4_des[2] + local_fraction * (q4_des[3] - q4_des[2]);
            break;
        case 3:
            th1_des = q1_des[3] + local_fraction * (q1_des[4] - q1_des[3]);
            th2_des = q2_des[3] + local_fraction * (q2_des[4] - q2_des[3]);
            th3_des = q3_des[3] + local_fraction * (q3_des[4] - q3_des[3]);
            th4_des = q4_des[3] + local_fraction * (q4_des[4] - q4_des[3]);
            break;
        case 4:
            th1_des = q1_des[4];
            th2_des = q2_des[4];
            th3_des = q3_des[4];
            th4_des = q4_des[4];
            break;
    }
}

void setInterpVel(float* q1_des, float* q2_des, float* q3_des, float* q4_des, float cycle_period, float curr_time) {

    // Calculate the time fraction into the current cycle
    float cycle_fraction = fmod(curr_time, cycle_period) / cycle_period;

    // Determine the segment based on the cycle_fraction (5 segments total)
    int segment = static_cast<int>(cycle_fraction * 5);

    // Calculate the local fraction within the current segment
    float local_fraction = (cycle_fraction * 5) - segment;

    // Segment time interval (1/5 of the total cycle period)
    float segment_time = cycle_period / 5.0;

    // Interpolate velocity within each segment
    switch (segment) {
        case 0:
            dth1_des = (q1_des[1] - q1_des[0]) / segment_time;
            dth2_des = (q2_des[1] - q2_des[0]) / segment_time;
            dth3_des = (q3_des[1] - q3_des[0]) / segment_time;
            dth4_des = (q4_des[1] - q4_des[0]) / segment_time;
            break;
        case 1:
            dth1_des = (q1_des[2] - q1_des[1]) / segment_time;
            dth2_des = (q2_des[2] - q2_des[1]) / segment_time;
            dth3_des = (q3_des[2] - q3_des[1]) / segment_time;
            dth4_des = (q4_des[2] - q4_des[1]) / segment_time;
            break;
        case 2:
            dth1_des = (q1_des[3] - q1_des[2]) / segment_time;
            dth2_des = (q2_des[3] - q2_des[2]) / segment_time;
            dth3_des = (q3_des[3] - q3_des[2]) / segment_time;
            dth4_des = (q4_des[3] - q4_des[2]) / segment_time;
            break;
        case 3:
            dth1_des = (q1_des[4] - q1_des[3]) / segment_time;
            dth2_des = (q2_des[4] - q2_des[3]) / segment_time;
            dth3_des = (q3_des[4] - q3_des[3]) / segment_time;
            dth4_des = (q4_des[4] - q4_des[3]) / segment_time;
            break;
        case 4:
            dth1_des = 0.0f;
            dth2_des = 0.0f;
            dth3_des = 0.0f;
            dth4_des = 0.0f;
            break;
    }
}

void setupExperiment() {
    t.reset();
    t.start();
    experiment_timer.reset();
    experiment_timer.start();
    total_experiment_time = cycle_period*cycles;
    encoderA.reset();
    encoderB.reset();
    encoderC.reset();
    encoderD.reset();

    motorShield.motorAWrite(0, 0); // Turn motor A off
    motorShield.motorBWrite(0, 0); // Turn motor B off
    motorShield.motorCWrite(0, 0); // Turn motor C off
    motorShield.motorDWrite(0, 0); // Turn motor D off
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
    //float dummy_params[37];
    
    while(1) {
        
        // If there are new inputs, this code will run
        if (server.getParams(input_params,NUM_INPUTS)){
        //if (server.getParams(input_params,NUM_INPUTS)) { //Get input

            //Manually set input
            //float input_params[25] = {1.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 15.000000, 15.000000, 0.500000, 0.500000};           float q1[5], q2[5], q3[5], q4[5];
            setInputParams(input_params,q1_des,q2_des,q3_des,q4_des);
            //pc.printf("%f \n\r",cycle_period);

            // Attach current loop interrupt
            currentLoop.attach_us(CurrentLoop,current_control_period_us);
                        
            // Setup experiment
            setupExperiment();

            cycles = 5; //Make this an input      
            // Run experiment
                while( t.read() < cycle_period*cycles) { 
                    //pc.printf("---------------\n\r");
                    //pc.printf("t.read()=%f \n\r",t.read());
                    // Read encoders to get motor states
                    angle1 = encoderA.getPulses() * PULSE_TO_RAD + angle1_init;       
                    velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;
                    //pc.printf("%f \n\r", angle1);
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
                    //pc.printf("Jxq1%f \n\r",Jx_th1);       
                    // Calculate the forward kinematics (position and velocity)
                    float xFoot_Front = l_AC*sin(th1+th2) + l_DE*sin(th1) + l_OB*sin(th1);
                    float yFoot_Front = -l_AC*cos(th1+th2) - l_DE*cos(th1) - l_OB*cos(th1);
                    float dxFoot_Front = dth1*(l_AC*cos(th1+th2) + l_DE*cos(th1) + l_OB*cos(th1)) + dth2*(l_AC*cos(th1+th2));
                    float dyFoot_Front = dth1*(l_AC*sin(th1+th2) + l_DE*sin(th1) + l_OB*sin(th1)) + dth2*(l_AC*sin(th1+th2));       

                    float xFoot_Back = l_AC*sin(th3+th4) + l_DE*sin(th3) + l_OB*sin(th3);
                    float yFoot_Back = -l_AC*cos(th3+th4) - l_DE*cos(th3) - l_OB*cos(th3);
                    float dxFoot_Back = dth3*(l_AC*cos(th3+th4) + l_DE*cos(th3) + l_OB*cos(th3)) + dth4*(l_AC*cos(th3+th4));
                    float dyFoot_Back = dth3*(l_AC*sin(th3+th4) + l_DE*sin(th3) + l_OB*sin(th3)) + dth4*(l_AC*sin(th3+th4));    
                    
                    
                    // Get desired foot positions and velocities
                    float rDesFoot_Front[2] , vDesFoot_Front[2];
                    float rDesFoot_Back[2] , vDesFoot_Back[2];

                    setInterpPos(q1_des, q2_des, q3_des, q4_des, cycle_period, curr_time);
                    setInterpVel(q1_des, q2_des, q3_des, q4_des, cycle_period, curr_time);

                    //pc.printf("q11%f \n\r",q1[1]);
                    //pc.printf("q1des%f \n\r",th1_des);
                   

                    //Joint Space torque
                    float tauq1 = K_h*(th1_des-th1)+D_h*(dth1_des-dth1);
                    float tauq2 = K_k*(th2_des-th2)+D_k*(dth2_des-dth2);
                    float tauq3 = K_h*(th3_des-th3)+D_h*(dth3_des-dth3);
                    float tauq4 = K_k*(th4_des-th4)+D_k*(dth4_des-dth4);
                    //pc.printf("%f \n\r",tauq1);
                    current_des1 = tauq1/k_t;      
                    current_des2 = tauq2/k_t;  
                    current_des3 = tauq3/k_t;      
                    current_des4 = tauq4/k_t; 
                    //pc.printf("%f \n\r",current_des1);
                    // pc.printf("Angle 1:%f \n\r",th1);
                    // pc.printf("Angle 2:%f \n\r",th2);
                    // pc.printf("Angle 3:%f \n\r",th3);
                    // pc.printf("Angle 4:%f \n\r",th4);
                    // pc.printf("---------\n\r");

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
                    // //back foot
                    output_data[29] = xFoot_Back;
                    output_data[30] = yFoot_Back;
                    output_data[31] = dxFoot_Back;
                    output_data[32] = dyFoot_Back;
                    output_data[33] = rDesFoot_Back[0];
                    output_data[34] = rDesFoot_Back[1];
                    output_data[35] = vDesFoot_Back[0];
                    output_data[36] = vDesFoot_Back[1];
                    pc.printf("th1_des: %f, th2_des: %f, th3_des: %f, th4_des: %f\n", th1_des, th2_des, th3_des, th4_des);

                    // Send data to MATLAB
                    server.sendData(output_data,NUM_OUTPUTS);

                    wait_us(impedance_control_period_us);  
                }
                t.reset();
            // Cleanup after experiment
                cleanupAfterExperiment();
        } // end if
        
    } // end while
    
} // end main
