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


void setup() {
 
}

void loop() {

}
