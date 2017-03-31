/*
 * This code will supprt the testing rig we built for the control 
 * theory event helod on 32 Mar 2017 with Kenneth Sebesta.
 * 
 * The purpose of this demo is to introduce UAV members to the use of PID
 * control loops to stabalize a system. In this case, we have a DC motor 
 * at the end of a vehicel arm attached to a pivot point which is clamped 
 * to the table. 
 * 
 * Each team should be able to use the data from the onboard IMU to set the angle 
 * of the arm to be +- 30 deg from level using an arduino and the required sensors
 * 
 * Equipment: 
 *  - Test Rig
 *  - DC motor
 *  - ESC
 *  - IMU
 *  - Arduino Uno 
 *  
 */
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>

#define PID_INPUT   0   // read the value from the IMU 
#define PID_OUTPUT  3   // set the value form the ESC
#define ESC_PIN     10  // Where the ESC is connected


Servo ESC;  // Create the ESC Servo
 
double Setpoint;
double Input, Output; // input is form IMU & output is going to the ESC

double Kp=1, Ki=0, Kd=0;
PID motor_PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); // set the characteristics of the controller

void setup() {

  // Setting up the ESC
  ESC.attach(ESC_PIN);
  
  // Setting up the PID Controller
  Setpoint;   // set this for the angle you want

  //turn the PID on
  motor_PID.SetMode(AUTOMATIC);
 
}

void loop() {

  // Execute PID Controller
  motor_PID.Compute();
  ESC.write(Output);
  delay(15);

}

