#include <PID_v1.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <stdint.h>

#define ESC_PIN     10  // Where the ESC is connected (PWM)
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

Servo ESC; // Create the ESC Servo
MPU6050 mpu;
unsigned long oldTime;
unsigned long dt;
int16_t inAccelRawY;
int16_t inAccelRawZ;
int16_t inGyroRaw;
int16_t gyrooffset;
double zapprox;
double yapprox;
double accelapprox;
double gyroapprox=0;
double gyrorate;
double alpha = .98;
double angleapprox;
double output;
double setpoint = 0;
//double Kp=6;
//double Ki=.003;
//double Kd=1000;
double Kp=1;
double Ki=.001;
double Kd=500*.5;
double Kff = 1300;
double pterm;
double dterm;
double iterm = 0;
char command = '\0'; // null value for default command
bool program_run = false;


void get_PIDs(void) {
  /* Command Line Format
   *  r - to run the program
   *  k - to terminate the program
   *  p value\n
   *  i value\n
  */

  String input_string = "\0";
  char input_char = '\0';
  String coef = "\0";
  int space = 0;
  int newLine = 0;
  float value = 0;

  // Get the command line date from the user input
  while (Serial.available()) {
    delay(3); // allow the input buffer to fill

    if (Serial.available() > 0) {
      input_char = (char)Serial.read(); // gets one byte from serial buffer
      input_string += input_char; // add it to our running string
    }
  }

  // Parse the command line input string
  if (input_string.length() > 0) {
    
    space = input_string.indexOf(' ');
    newLine = input_string.indexOf('\n');
    coef = input_string.substring(0,space);
    
    if (coef.equals("q")) {
      Serial.println("turning off the motor");
      program_run = false;
    }
    else if (coef.equals("r")) {
      Serial.println("Starting Motor");
      Serial.print("p: "); Serial.println(Kp);
      Serial.print("i: "); Serial.println(Ki);
      Serial.print("d: "); Serial.println(Kd);
      Serial.print("ff: "); Serial.println(Kff);
      Serial.print("setpoint: "); Serial.println(setpoint);
      Serial.println("--------------------------------------");
      program_run = true;
    }

    // otherwise determine what commands we received
    value = input_string.substring(space,newLine).toFloat(); // 0 if nothing is passed to it

    // Set the correct coefficient
    if (coef.equals("set")) {
      setpoint = (double)value;
      Serial.print("Setpoint: "); Serial.println(value);
    }
    else if (coef.equals("p")) {
      Kp = value;
      Serial.print("p: "); Serial.println(value);
    }
    else if (coef.equals("i")) {
      Ki = value;
      Serial.print("i: "); Serial.println(value);
    } 
    else if (coef.equals("d")) {
      Kd = value;
      Serial.print("d: "); Serial.println(value);
    }
    else if (coef.equals("ff")) {
      Kff = value;
      Serial.print("ff: "); Serial.println(Kff);
    }
    else if (coef.equals("reset")) {
      Kp=1;
      Ki=.001;
      Kd=500*.5;
      Kff = 1300;
      setpoint = 0;
      Serial.println("Resetting our control coefficients");
    }
  }
}


void calibrateGyro(int samples){
  long long sum = 0;
  for(int i = 0; i < samples; i++){
    sum += mpu.getRotationX();
    delay(1); //allow time for new values
  }
  gyrooffset = (double)sum/(double)samples;
  Serial.print("Gyro calibration value is: ");
  Serial.println(gyrooffset);
}


void setup() {
  Serial.begin(115200); // Serial command line interface
  Wire.begin();
  Wire.setClock(1000000L);
  while (!Serial);
  Serial.println(F("Initializing MPU..."));
  mpu.setSleepEnabled(false);
  delay(100);     //wait for powerup
  calibrateGyro(100);

  ESC.attach(ESC_PIN); // Setting up the ESC
  ESC.writeMicroseconds(1000);
  delay(3000);

  ESC.writeMicroseconds(1150);
  delay(1000);

  oldTime = micros();

  Serial.println("======================================");
  Serial.println("           Your Commands:");
  Serial.println("======================================");
  Serial.println("r: to run");
  Serial.println("q: to stop");
  Serial.println("set <value> : set the setpoint");
  Serial.println();
  Serial.println("p <value>   : set the P-term");
  Serial.println("i <value>   : set the I-term");
  Serial.println("d <value>   : set the D-term");
  Serial.println("ff <value>  : set the feed forward term");
  Serial.println();
  Serial.println("reset: returns system parameters to default");
  Serial.println("======================================");
  Serial.println("                End");
  Serial.println("======================================");
}

static bool isFirstLoop = true;

void loop() {

  get_PIDs(); // Get the user input for adjusting values

  //Serial.print("loop time is: "); 
  do {
    dt = micros()-oldTime;
  } while (dt <2000);
  if (dt == 0) {
    return;
  }
  
  oldTime += 2000;

  //Serial.print(dt); Serial.print("   ");
  inAccelRawZ = mpu.getAccelerationZ();
  inAccelRawY = mpu.getAccelerationY();
  inGyroRaw = mpu.getRotationX();

  accelapprox = atan(inAccelRawY/sqrt(pow(inAccelRawZ,2) + pow(inAccelRawZ,2)))*57.2958;
  gyrorate = ((inGyroRaw - gyrooffset)/131.0)*(dt/1000000.0);
  gyroapprox = gyrorate + angleapprox;
  angleapprox = alpha*gyroapprox + (1.0 - alpha)*accelapprox;

  pterm = -Kp*(angleapprox-setpoint);
  //Serial.print("pterm: ");Serial.println(pterm);
  dterm = -Kd*gyrorate;
  //Serial.print("dterm: ");Serial.println(dterm);
  if (program_run) {
    iterm += -Ki*(angleapprox-setpoint);
  } else {
    iterm = 0;
  }

  if (iterm > 200) {
    iterm = 200;
  } else if (iterm < -200) {
    iterm = -200;
  }

  output = pterm + dterm + iterm + Kff;
 
  if(output>1500){
    output=1500;
  } else if (output<1100) {
    output=1100;
  }

  if (program_run) {
    ESC.write(output);
  } else {
    ESC.write(1000); // off
  }
  
  //  Serial.println(output);

}
