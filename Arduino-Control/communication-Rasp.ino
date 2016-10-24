// Clockwise and counter-clockwise definitions.
// Depending on how you wired your motors, you may need to swap.
#define CW  0
#define CCW 1

// Motor definitions to make life easier:
#define MOTOR_A 0
#define MOTOR_B 1

#define STOP 's'
#define FOWARD 'f'
#define BACKWARD 'b'
#define LEFT 'l'
#define RIGHT 'r'

const byte PWMA = 3;  // PWM control (speed) for motor A
const byte PWMB = 11; // PWM control (speed) for motor B
const byte DIRA = 12; // Direction control for motor A
const byte DIRB = 13; // Direction control for motor B

char incoming = 'q';

void setup()
{
  Serial.begin(9600);
  setupArdumoto();
}

void loop()
{
  
  if (Serial.available()){
    
    incoming = Serial.read();
    switch(incoming){
      case 'f':
        driveArdumoto(MOTOR_A, CCW, 100);
        driveArdumoto(MOTOR_B, CCW, 100);
      break;
      case 'b':
        driveArdumoto(MOTOR_A, CW, 100);
        driveArdumoto(MOTOR_B, CW, 100);
      break;
      case 'l':
        analogWrite(PWMA, 50);
        analogWrite(PWMB, 255);
      break;
      case 'r':
        analogWrite(PWMA, 255);
        analogWrite(PWMB, 50);
      break;
      default:
        stopArdumoto(MOTOR_A);
        stopArdumoto(MOTOR_B);
    }
    delay(1000);
    if(incoming != 'q'){
      Serial.write(incoming);
    }
    delay(1000);
    
  }
}

// driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed
void driveArdumoto(byte motor, byte dir, byte spd)
{
  if (motor == MOTOR_A)
  {
    digitalWrite(DIRA, dir);
    analogWrite(PWMA, spd);
  }
  else if (motor == MOTOR_B)
  {
    digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd);
  }  
}

// stopArdumoto makes a motor stop
void stopArdumoto(byte motor)
{
  driveArdumoto(motor, 0, 0);
}

// setupArdumoto initialize all pins
void setupArdumoto()
{
  // All pins should be setup as outputs:
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  // Initialize all pins as low:
  digitalWrite(PWMA, LOW);
  digitalWrite(PWMB, LOW);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
}
