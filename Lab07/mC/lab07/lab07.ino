/* 
IRM Lab07 : Closed-loop control I

*/

// Include the required libraries here (already done for you)
#include <string.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>


// global variables to control each motor
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *myMotor1 = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *myMotor2 = AFMS.getStepper(200, 2);

// pinouts for limit switches
int switch1 = 6; // motor 1
int switch2 = 7; // motor 1
int switch3 = 4; // motor 2
int switch4 = 5; // motor 2

// step size (Number of steps in each iteration)
int stepSize = 20;
// maximum speed in rpm of each motor
int maxSpeed = 500;

// Functions

int move_steps (int steps, int dir, int motor) {
   int switch_check;
   int step_limit, step_stop, step_count, out = 0;
   int switch_press, ref;
  
  // Check the motor and direction of movement, and set the limiting switch accordingly
   if (motor == 1) {
      if (dir == 1) {
          ls = 7; // limiting switch = switch 2
      }
      else if (dir == 2) {
          ls = 6; // limiting switch = switch 1
      }
      else {
          out = 53;   // if a problem occour in the function, 5 is returned
      }
   }
   else if (motor == 2) {
      if (dir == 1) {
          ls = 5; // limiting switch = switch 4
      }
      else if (dir == 2) {
          ls = 4; // limiting switch = switch 3
      }
      else {
          out = 53;
      }
   }
   else {
      out = 53;
   }

  // Limit the total number of steps to 999
  step_limit = 999;  
 
  // Create a loop, which is executed if steps > 0 and the limit switch has not been reached, 
  // in the loop, move the desired motor in small steps (stepsize). Execute the loop until you moved
  // the whole distance
  step_count = 0;
  switch_check = -1;
  ref = digitalRead(ls);
    
  if(step_limit <= steps) {
    step_stop = step_limit;
  }
  else {
    step_stop = steps;
  }
  
  if (steps != 0 & out == 0) {
    while (step_count <= step_stop) {
      if (dir == 1) {
        step(1, FORWARD, MICROSTEP);
      }
      else if (dir == 2) {
        step(1, BACKWARD,MICROSTEP);
      }
          
      switch_press = digitalRead(ls);
      if (switch_press != ref) {
        switch_check = 0;
        break;
      }
      
      step_count++
    }
  }

  // After running the loop, return ASCII for the switches
    if (switch_check == 0)  {
      switch (ls) {
        // If switch 2 is pressed, return '2'
        case 7 :
          out = 50;
          break;
        // If switch 1 is pressed, return '1'
        case 6 :
          out = 49;
          break;
        // If switch 4 is pressed, return '4'
        case 5 :
          out = 52;
          break;
        case 4 :
        // If switch 3 is pressed, return '3'
          out = 51;
          break;
        default :
          out = 53;
          break;
      }
    }
   
    // Else, return '0'
    else {
      out = 48;
    }

    return out;   // return the correct message
   
}


void setup() {
  
  // Start the serial communication at 9600 baud rate
  Serial.begin(BAUDRATE);
  // Set serial communication timeout to 10
  Serial.setTimeout(10);
  // Start the Adafruit Motor Shield and set the maximum speed of the stepper to 500 rpm
  AFMS.begin();
  myMotor1->setSpeed(maxSpeed);
  myMotor2->setSpeed(maxSpeed);

  // Set the input pins
  pinMode(switch1, INPUT);
  pinMode(switch2, INPUT);
  pinMode(switch3, INPUT);
  pinMode(switch4, INPUT);

}



void loop() {
  // Initialize parameters
  char command[50];
  char check;
  byte read_check;

  int int_check;
  int steps, dir, motor;
  int c4, c3, c2;

  // Check if there is a command on the serial port
  if (Serial.available() > 0)
  {
    read_check = Serial.readBytes(command,20);
    // If the command is not 5 bytes long, discard it
    if ((int)read_check == 5)
    {
      command[5] = '\0';
      check = '0';
    }
    else
    {      
      command[0] = '\0';
      check = '5';
      Serial.print(check);
    }
  }
  
  // Proper command contains 5 bytes:
  //  First byte is the stage number: 1 or 2
  //  Second byte is the direction: 1 or 2
  //  Third - Fifth bytes are number of steps: 000 - 999
  if (command[0]!=0)
  {
    // Check the first byte and if it is not '1' or '2' discard it
    // First byte determines the stage (stepper motor) that needs to be moved
    motor = (command[0]-'0');
    if (motor != 1 | motor != 2) {
      check = '5';
    }
  
    // Check the second byte and if it is not '1' or '2' discard it
    // Second byte determines the direction
    dir = (command[1]-'0');
    if (dir != 1 | dir != 2) {
      check = '5';
    }
      
    // Check that third to fifth bytes are between '0' and '9'
    // make sure to convert from chars to integers (subtract 48, the ASCII constant) and multiply accordingly
    c4 = (command[4]-'0');
    c3 = (command[3]-'0');
    c2 = (command[2]-'0');
    if (c4 >= 0 & c4 <= 9 & c3 >= 0 & c3 <= 9 & c2 >= 0 & c2 <= 9) {
      steps = c4+10*c3+100*c2;
    }
    //steps = 1*(command[4]-'0')+10*(command[3]-'0')+100*(command[2]-'0');
    
    // If everything is fine, move the motors
    if (check == '5') {
      // if check == '5' something went wrong
      // do nothing, it will print out '5'
    }
    else {
      int_check = move_steps (steps, dir, motor);
      check = int_check;    // divided into two steps to make sure the conversion is made correctly   
    }
    
    // Check is sent over the serial back to microprocessor:
    //     '0' if motor moved 
    //     '1' if switch 1 is pressed
    //     '2' if switch 2 is pressed
    //     '3' if switch 3 is pressed
    //     '4' if switch 4 is pressed
    //     '5' if command is bad
    Serial.print(check);
  
    // Reset the command
    command[0] = '\0';
    Serial.flush();
  }
  
  // Delay in ms
  delay(5);
}
