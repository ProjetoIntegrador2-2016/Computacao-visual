// Clockwise and counter-clockwise definitions.
#define CW  1
#define CCW 0

// Motor definitions to make life easier:
#define MOTOR_A 0
#define MOTOR_B 1

// Pin definitions for Motors
const byte PWMA = 12;  // PWM control (speed) for motor A
const byte PWMB = 13; // PWM control (speed) for motor B

// driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed
void driveArdumoto(byte motor, byte spd) {
  if (motor == MOTOR_A) {
    //digitalWrite(DIRA, dir);
    analogWrite(PWMA, spd);
    
  } else if (motor == MOTOR_B) {
    //digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd);
  }  
}

void stopRobot(){
  driveArdumoto(MOTOR_A, 0);
  driveArdumoto(MOTOR_B, 0);
}

void stopArdumoto(byte motor) {
  driveArdumoto(motor, 0);
}

// setupArdumoto initialize all pins
void setupArdumoto() {
  // All pins should be setup as outputs:
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
}

void softStartMotors(int pwmA, int pwmB){

  for (int i = 0; i < max(pwmA, pwmB); i++){
    if(pwmA > 0){
      driveArdumoto(MOTOR_A, i);
    }

    if(pwmB > 0){
      driveArdumoto(MOTOR_B, i);  
    }
    
  }
  
}
