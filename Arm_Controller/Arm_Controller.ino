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


#define PID_INPUT 0
#define PID_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=1, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {

  // Setting up the PID Controller
  Input; //get this from 
  Setpoint = 100;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
 
}

void loop() {

  // PID Controller
  myPID.Compute();

}

