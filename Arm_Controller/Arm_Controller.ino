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
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define PID_INPUT   0   // read the value from the IMU 
#define PID_OUTPUT  3   // set the value form the ESC
#define ESC_PIN     10  // Where the ESC is connected (PWM)


Servo ESC;  // Create the ESC Servo
 
double setpoint;
double Input;   // input is form IMU
double Output;  // output is going to the ESC

double Kp=1, Ki=0 , Kd=0;
PID motor_PID(&Input, &Output, &setpoint, Kp, Ki, Kd, DIRECT); // set the characteristics of the controller

void setup() {

  // Serial command line interface
  Serial.begin(9600);
  
  // Setting up the ESC
  ESC.attach(ESC_PIN);
  
  // Setting up the PID Controller
  setpoint;   // set this for the angle you want

  //turn the PID on
  motor_PID.SetMode(AUTOMATIC);
 
}

void loop() {

  // Get the setpoint value from user input
  if (Serial.available() > 0) {
    // read the incoming byte:
    setpoint = Serial.read();
  
    // say what you got:
    Serial.print("I received: ");
    Serial.println(setpoint, DEC);
  }
  else {
    setpoint = 0; // default value
  }
  
  // Execute PID Controller
  motor_PID.Compute();
  ESC.write(Output);
  delay(15);

}

