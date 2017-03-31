/*
 * Organization   : Boston University UAV Team
 * Authors        : Nick Brisco, Connor McCann
 * Event          : Control Theory Event
 * Date           : 31 Mar 2017
 * 
 * This code will supprt the testing rig we built for the control 
 * theory event helod on 32 Mar 2017 with Kenneth Sebesta.
 * 
 * The purpose of this demo is to introduce UAV members to the use of PID
 * control loops to stabalize a system. In this case, we have a DC motor 
 * at the end of a vehicel arm attached to a pivot point which is clamped 
 * to the table. 
 * 
 * Each team should be able to use the data from the onboard IMU to set the angle 
 * of the arm to be +- ~30 deg from level using an arduino and the required sensors
 * 
 * Equipment: 
 *  - Test Rig (Designed by Andrew Lee)
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

#define IMU_INPUT   0   // read the value from the IMU 
#define ESC_PIN     10  // Where the ESC is connected (PWM)
#define TESTING     1

Servo ESC; // Create the ESC Servo
 
double setpoint;
double Input; // input is form IMU
double Output; // output is going to the ESC

double Kp=1, Ki=0 , Kd=0;
PID motor_PID(&Input, &Output, &setpoint, Kp, Ki, Kd, DIRECT); // set the characteristics of the controller

char command = '\0'; // null value for default command

//==========================================================================
//                            SETUP LOOP
//==========================================================================
void setup() {
  Serial.begin(9600); // Serial command line interface
  ESC.attach(ESC_PIN); // Setting up the ESC
  //ESC.writeMicroseconds(2000);
  //delay(3000);
  ESC.writeMicroseconds(1000);
  delay(3000);
  //ESC.writeMicroseconds(2000);
  //delay(3000);
  setpoint; // set this for the angle you want
  motor_PID.SetMode(AUTOMATIC); //turn the PID on

  Serial.println("Please enter CAPITAL R to run");
}

//==========================================================================
//                             MAIN LOOP
//==========================================================================
void loop() {

  // Command line safety
  while (command != 'R') {
    if (Serial.available()) {
      command = Serial.read();
      if (command != 'R') {
        Serial.println("Wrong Command...Try Again");
      }
      else {
        Serial.println("Will Start in 3 Seconds");
        delay(3000);
        Serial.println("Lift Off!");
      }
      }
    }

  if (TESTING) {
    ESC.writeMicroseconds(1500);
    Serial.println("Testing ESC...");
    delay(100);
  }
  else {
    // handle the user command line interface
    setpoint = 0; // we received a run command so set the setpoint to level
    motor_PID.Compute(); // Execute PID Controller
    ESC.write(Output);
    delay(15);
  }
}

