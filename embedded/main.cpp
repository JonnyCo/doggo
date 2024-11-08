#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "ExperimentServer.h"
#include "QEI.h"
#include "MotorShield.h" 

#define NUM_INPUTS 11  // Adjusted for joint inputs (initial angles, desired torques)
#define NUM_OUTPUTS 17  // Adjusted for comprehensive output data
#define PULSE_TO_RAD (2.0f * 3.14159f / 1200.0f)

// Initializations
Serial pc(USBTX, USBRX); // USB Serial Terminal
ExperimentServer server; // Object that lets us communicate with MATLAB
Timer t;                 // Timer to measure elapsed time of experiment

QEI encoderA(PE_9, PE_11, NC, 1200, QEI::X4_ENCODING); // Motor A encoder
QEI encoderB(PA_5, PB_3, NC, 1200, QEI::X4_ENCODING); // Motor B encoder
QEI encoderC(PC_6, PC_7, NC, 1200, QEI::X4_ENCODING); // Motor C encoder
QEI encoderD(PD_12, PD_13, NC, 1200, QEI::X4_ENCODING); // Motor D encoder

MotorShield motorShield(24000); // Motor shield initialization
Ticker currentLoop;

// Variables for motor control
float current1, current2, current3, current4;
float current_des1 = 0, current_des2 = 0, current_des3 = 0, current_des4 = 0;
float angle1, angle2, angle3, angle4;
float velocity1, velocity2, velocity3, velocity4;
float duty_cycle1, duty_cycle2, duty_cycle3, duty_cycle4;
float angle1_init, angle2_init, angle3_init, angle4_init;

// Control parameters
float current_Kp = 4.0f;
float current_Ki = 0.4f;
float current_int1 = 0, current_int2 = 0, current_int3 = 0, current_int4 = 0;
float current_int_max = 3.0f;
float duty_max = 0.4;
float supply_voltage = 12;
float R = 2.0f;
float k_t = 0.18f;

// Current control interrupt function
void CurrentLoop() {
    // Read motor positions and velocities
    angle1 = encoderA.getPulses() * PULSE_TO_RAD + angle1_init;
    velocity1 = encoderA.getVelocity() * PULSE_TO_RAD;

    angle2 = encoderB.getPulses() * PULSE_TO_RAD + angle2_init;
    velocity2 = encoderB.getVelocity() * PULSE_TO_RAD;

    angle3 = encoderC.getPulses() * PULSE_TO_RAD + angle3_init;
    velocity3 = encoderC.getVelocity() * PULSE_TO_RAD;

    angle4 = encoderD.getPulses() * PULSE_TO_RAD + angle4_init;
    velocity4 = encoderD.getVelocity() * PULSE_TO_RAD;

    // Calculate errors and desired currents for joint control
    float err1 = current_des1 - current1;
    float err2 = current_des2 - current2;
    float err3 = current_des3 - current3;
    float err4 = current_des4 - current4;

    current_int1 = fmaxf(fminf(current_int1 + err1, current_int_max), -current_int_max);
    current_int2 = fmaxf(fminf(current_int2 + err2, current_int_max), -current_int_max);
    current_int3 = fmaxf(fminf(current_int3 + err3, current_int_max), -current_int_max);
    current_int4 = fmaxf(fminf(current_int4 + err4, current_int_max), -current_int_max);

    // PI controllers
    duty_cycle1 = (R * current_des1 + k_t * velocity1 + current_Kp * err1 + current_Ki * current_int1) / supply_voltage;
    duty_cycle2 = (R * current_des2 + k_t * velocity2 + current_Kp * err2 + current_Ki * current_int2) / supply_voltage;
    duty_cycle3 = (R * current_des3 + k_t * velocity3 + current_Kp * err3 + current_Ki * current_int3) / supply_voltage;
    duty_cycle4 = (R * current_des4 + k_t * velocity4 + current_Kp * err4 + current_Ki * current_int4) / supply_voltage;

    // Clamp and set motor commands
    duty_cycle1 = fminf(fmaxf(duty_cycle1, -duty_max), duty_max);
    duty_cycle2 = fminf(fmaxf(duty_cycle2, -duty_max), duty_max);
    duty_cycle3 = fminf(fmaxf(duty_cycle3, -duty_max), duty_max);
    duty_cycle4 = fminf(fmaxf(duty_cycle4, -duty_max), duty_max);

    motorShield.motorAWrite(abs(duty_cycle1), duty_cycle1 < 0);
    motorShield.motorBWrite(abs(duty_cycle2), duty_cycle2 < 0);
    motorShield.motorCWrite(abs(duty_cycle3), duty_cycle3 < 0);
    motorShield.motorDWrite(abs(duty_cycle4), duty_cycle4 < 0);

    // Prepare data to send back to MATLAB
    float output_data[NUM_OUTPUTS];
    output_data[0] = t.read(); // Time
    output_data[1] = angle1;   // Joint angle 1
    output_data[2] = velocity1; // Joint velocity 1
    output_data[3] = current1;  // Measured current 1
    output_data[4] = current_des1; // Desired current 1
    output_data[5] = duty_cycle1;  // Motor duty cycle 1
    output_data[6] = angle2;
    output_data[7] = velocity2;
    output_data[8] = current2;
    output_data[9] = current_des2;
    output_data[10] = duty_cycle2;
    output_data[11] = angle3;
    output_data[12] = velocity3;
    output_data[13] = current3;
    output_data[14] = current_des3;
    output_data[15] = duty_cycle3;
    output_data[16] = angle4;
    output_data[17] = velocity4;
    output_data[18] = current4;
    output_data[19] = current_des4;
    output_data[20] = duty_cycle4;

    // Send data to MATLAB
    server.sendData(output_data, NUM_OUTPUTS);
}

void setInputParams(const float* input_params) {
    // Initialize joint angles
    angle1_init = input_params[0];
    angle2_init = input_params[1];
    angle3_init = input_params[2];
    angle4_init = input_params[3];

    // Set desired currents based on desired torques (k_t conversion)
    current_des1 = input_params[4] / k_t; // Desired torque converted to current
    current_des2 = input_params[5] / k_t;
    current_des3 = input_params[6] / k_t;
    current_des4 = input_params[7] / k_t;
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

void cleanupAfterExperiment() {
    server.setExperimentComplete();
    currentLoop.detach();
    motorShield.motorAWrite(0, 0);
    motorShield.motorBWrite(0, 0);
    motorShield.motorCWrite(0, 0);
    motorShield.motorDWrite(0, 0);
}

int main() {
    server.attachTerminal(pc);
    server.init();
    float input_params[NUM_INPUTS];

    while (1) {
        if (server.getParams(input_params, NUM_INPUTS)) {
            setInputParams(input_params);
            setupExperiment();
            currentLoop.attach_us(CurrentLoop, 200); // 5kHz control loop

            while (t.read() < 10) { // Run for a specified duration
                wait_us(1000); // 1kHz update
            }

            cleanupAfterExperiment();
        }
    }
}
