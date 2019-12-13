Servo motor1, motor2;  // servo objects to control a motor drivers
Encoder enc1(M1_ENC_PIN_1, M1_ENC_PIN_2);
Encoder enc2(M2_ENC_PIN_1, M2_ENC_PIN_2);

long m1_oldPosition  = -999;
long m2_oldPosition  = -999;
//int input = -1;
//long target = 0;
//int turnSpeed = 1;

void encoderSetup() {
  motor1.attach(M1_PWM_PIN);
  motor1.attach(M1_PWM_PIN);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_BRAKE_PIN, OUTPUT);

  /*
    motor2.attach(M2_PWM_PIN);
    motor2.attach(M2_PWM_PIN);
    pinMode(M2_DIR_PIN, OUTPUT);
    pinMode(M2_BRAKE_PIN, OUTPUT);
  */
}

void printOnMovement() {
  long m1_newPosition = enc1.read();
  if (m1_newPosition != m1_oldPosition) {
    m1_oldPosition = m1_newPosition;
    Serial.println(m1_newPosition);
  }
  long m2_newPosition = enc2.read();
  if (m2_newPosition != m2_oldPosition) {
    m2_oldPosition = m2_newPosition;
    Serial.println(m2_newPosition);
  }
}

// accepts int for the motor, and a percentage value for PWM output (-100% to 100%)
// if motor was braked earlier, it cancels the brake
void writeToMotor(int motor, int percentOutput) {
  boolean reverse = percentOutput < 0;
  int value = constrain(abs(percentOutput), 0, 100);
  switch (motor) {
    case 1:
      digitalWrite(M1_BRAKE_PIN, LOW);
      digitalWrite(M1_DIR_PIN, reverse); // booleans are interchangeable with high/low
      motor1.write(map(value, 0, 100, 0, 180));
      break;
    case 2:
      digitalWrite(M2_BRAKE_PIN, LOW);
      digitalWrite(M2_DIR_PIN, reverse); // booleans are interchangeable with high/low
      motor2.write(map(value, 0, 100, 0, 180));
      break;
  }
}

void brakeMotor(int motor) {
  switch (motor) {
    case 1:
      writeToMotor(1, 0);
      digitalWrite(M1_BRAKE_PIN, HIGH); //hit the braaake
      break;
    case 2:
      writeToMotor(2, 0);
      digitalWrite(M2_BRAKE_PIN, HIGH); //hit the braaake
      break;
  }

}
