
int ki,kp,kd;

int program_run = 0;

void get_PIDs(void) {
  
  /* Command Line Format
   *  r - to run the program
   *  k - to terminate the program
   *  p value\n
   *  i value\n
   *  d value\n
  */

  String input_string = "\0";
  char input_char = '\0';
  String coef = "\0";
  int space = 0;
  int newLine = 0;
  int value = 0;

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
    
    coef = input_string.substring(0,1);
    
    if (coef.equals("k")) {
      Serial.println("turning off the motor");
      // turn off esc
      program_run = 0;
    }
    else if (coef.equals("r")) {
      program_run = 1;
      Serial.println("Starting Motor");
    }

    // otherwise determine what commands we received
    space = input_string.indexOf(' ');
    newLine = input_string.indexOf('\n');
    value = input_string.substring(space,newLine).toFloat(); // 0 if nothing is passed to it

    // Set the correct coefficient
    if (coef.equals("p")) {
      kp = value;
      Serial.print("kp: "); Serial.println(value);
    }
    else if (coef.equals("i")) {
      ki = value;
      Serial.print("ki: "); Serial.println(value);
    } 
    else if (coef.equals("d")) {
      kd = value;
      Serial.print("kd: "); Serial.println(value);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  get_PIDs();

}
