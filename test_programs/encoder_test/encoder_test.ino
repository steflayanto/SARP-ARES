/* 
  TEST PROGRAM TO RUN ONE MOTOR AND ENCODER
  
  Prints encoder steps (literally how many bars rotated through on the 
  optical encoder) to serial monitor every time encoder reads a new value.
  
  Reads in input in the form of an INTEGER and NEWLINE (newline is important).
  
  Input is read in 100s of steps, and set to be target position. For example,
  if an input of '50' is given, target position will be 5000 steps.
  
  Simple controller implemented to move motor to within 50 steps of target
  value, then a brake is applied.
  
  ENCODER LIBRARY: https://www.pjrc.com/teensy/td_libs_Encoder.html

  For PINOUTS, check the #define below

  Stefan Layanto
  
*/

#include <Servo.h>
#include <Encoder.h>

#define PWM_PIN 9
#define DIR_PIN 12
#define BRAKE_PIN 10
#define ENC_PIN_1 2
#define ENC_PIN_2 3

Servo pwm;  // create servo object to control a one motor driver
Encoder enc(ENC_PIN_1, ENC_PIN_2);
// twelve servo objects can be created on most boards

long oldPosition  = -999;
int input = -1;
long target = 0;
int turnSpeed = 1;

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Encoder Test:");
  pwm.attach(PWM_PIN);          // attaches the PWM pin on pin 9 to the servo object
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);
}

void loop() {
  
  if (Serial.available() >= 2) { //Handle input. IMPORTANT: Requires a newline
    input = Serial.parseInt(); //read integer
    Serial.read(); //read new line
    Serial.print("Recieved input: ");
    Serial.print(input);
    target = 100 * (long) input; // scale to 100s of steps and set as target
    Serial.println();
  }
  
  long newPosition = enc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.println(newPosition);
  }

  // VERY DUMB, SIMPLE CONTROLLER. I have ideas for improvements.
  if (input == -1) { //if input is -1, don't do anything. this is the initial value.
    pwm.detach(); //detaching servo is best way to make it stop completely and eliminate jittering.
    turnSpeed = 0; //turnspeed stop
  } else {
    pwm.attach(PWM_PIN); //attach servo in case detached
    if (abs(newPosition - target) > 500) {
      turnSpeed = 36; //turnspeed to 20% (turn speed 0-180 maps to 0-100%, so 36 -> 20%)
    } else {
      turnSpeed = 1;
    }
    if (newPosition > target + 50) { //go one direction
      digitalWrite(BRAKE_PIN, LOW);
      digitalWrite(DIR_PIN, LOW);
    } else if (newPosition < target - 50) { //go the other. these two can be swapped by swapping wires or swapping high/low of dir pin
      digitalWrite(BRAKE_PIN, LOW);
      digitalWrite(DIR_PIN, HIGH);
    } else {
      turnSpeed = 0; //stop rotation
      input = -1; //reset input to default
      digitalWrite(BRAKE_PIN, HIGH); //hit the braaake
    }
    pwm.write(turnSpeed);
  }
}
