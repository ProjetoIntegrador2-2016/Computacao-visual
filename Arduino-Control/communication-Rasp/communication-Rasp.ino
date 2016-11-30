#include <NewPing.h>
#include "pid.hpp"
#include "vector.hpp"
#include "motors.hpp"

// ############################### CONSTANTS ########################################

// Define constraints of the robot
#define DISTANCE_MOTOR_CENTER  7
#define WHEEL_RADIUS 3

// Utrasonic sensor definitions
#define SONAR_NUM     3
#define MAX_DISTANCE  50 // Distancia de deteccao
#define MIN_DISTANCE  20
#define PING_INTERVAL 33 //Intervalo entre as medicoes - valor minimo 29ms

// Pin definitions for Ultrasonic sensors
const byte TRIGGER_F = 7;
const byte ECHO_F = 6;
const byte TRIGGER_R = 10;
const byte ECHO_R = 9;
const byte TRIGGER_L = 5;
const byte ECHO_L = 4;

// Debug Constant
const byte DEBUG = 1;

// ############################### GLOBALS #######################################

// PID Globals
PID<float> distancePID(0.5, 0.025, 0, 20); // SetPoint 20 (distance)
PID<vector> angularPID(0.5, 0.008, 0, vector(0.0, 20.0)); // SetPoint 90 degrees

vector oldSample(0.0, 0.0);
long lastTime = 0;

// Ultrasonic sensors globals
unsigned long pingTimer[SONAR_NUM]; // Vezes que a medicao deve ocorrer
unsigned int cm[SONAR_NUM]; //Armazena o numero de medicoes
uint8_t currentSensor = 0; // Armazena o numero do sensor ativo

// Assign Ultrassonic sensors to pins
NewPing sonar[SONAR_NUM] = { 
  NewPing(TRIGGER_F, ECHO_F, MAX_DISTANCE),
  NewPing(TRIGGER_R, ECHO_R, MAX_DISTANCE),
  NewPing(TRIGGER_L, ECHO_L, MAX_DISTANCE),
};

// ################################# SETUP ########################################

void setup(){
  Serial.begin(9600);
  Serial1.begin(9600);
  setupArdumoto();
  
 //Inicia a primeira medicao com 75ms
  pingTimer[0] = millis() + 75;
  //Define o tempo de inicializacao de cada sensor
  for (uint8_t i = 1; i < SONAR_NUM; i++){
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }
    
  lastTime = millis();

  vector v(-1,-1);
  vector s(0.0, 20.0);
  double w = s - v;
  Serial.print(v.getAngle());
  Serial.print(" ");
  Serial.print(s.getAngle()); 
  Serial.print(" ");
  Serial.print(w);
  
}

// ################################## LOOP ########################################

void loop(){
  
  //loopSensors();
  
  float sampleXY[2];
  if (Serial1.available() > 0){
    
    // Read from serial
    sampleXY[0] = Serial1.parseFloat();
    sampleXY[1] = Serial1.parseFloat();
    Serial1.flush();
    
    vector sample = checkValues(sampleXY[0], sampleXY[1]);
    
    // Calculate PID gains
    angularPID.addNewSample(sample);
    //double distanceGain = distancePID.addNewSample(sample.calculateMagnitude());

    goToPosition(sample);
    
    if (oldSample.calculateMagnitude() > 0){
       
    }
    
    if (DEBUG){
      Serial.print("Vector: ");
      Serial.print(sample.xPos);Serial.print("--");Serial.print(sample.yPos);
      Serial.print("\nSamples: ");
      Serial.print(sampleXY[0]);Serial.print("--");Serial.print(sampleXY[1]);
      Serial.print("\n\n");
    }
    
    oldSample = sample;
  }
  
  Serial1.flush();
  Serial.flush();
}

// ############################ CONTROL METHODS ###################################

vector checkValues(float xPos, float yPos){
  vector result(xPos, yPos);

  if (yPos >= 0) {
    // Looks good, no invalid token
    result = filterDiscrepancies(xPos, yPos);
  
  } else { // Target not found, go to last known position then search it
    
    // If there is a last known position, go to it
    if (oldSample.calculateMagnitude() != 0){ 
      goToPosition(oldSample);
      delay(500); // Wait 0.5 s for target to appear
      
    } else { // Search target
      int dirA = CW;
      int dirB = CW;
      
      if (xPos < 0){ // Last position is to the left
        dirB = CCW;
      } else { // Last position to the right, spin right
        dirA = CCW;
      }
      
      driveArdumoto(MOTOR_A, dirA, 100);
      driveArdumoto(MOTOR_B, dirB, 100);
  
      // Spin for 0.05 s then look for target
      delay(50); 
      stopRobot();
      
      // Clear samples as it were in the begining
      result = vector();
      oldSample = result;
    }
  }
  
  return result;
}


// Seamless to a low pass filter
vector filterDiscrepancies(float xPos, float yPos){
  vector result(xPos, yPos);
  
  float oldX = oldSample.xPos;
  float oldY = oldSample.yPos;

  const int errorMargin = 8;
  
  // Check if new Pos is between the acceptable erro margin
  // only if the oldSample is bigger than (0, 0)
  if (oldSample.calculateMagnitude() != 0){
    
    if (xPos <= oldX + errorMargin && xPos >= oldX - errorMargin){
      // Looks good, do nothing
    } else {
      result.xPos = oldX;  
    }

    if (yPos <= oldY + errorMargin && yPos >= oldY - errorMargin){
      // Looks good, do nothing
    } else {
      result.yPos = oldY;  
    }
    
  } else {
    // Do nothing
  }

  return result;
}


void goToPosition(vector targetPosition){
  vector nullVector(0.0, 0.0);
  
  if (targetPosition != nullVector){
    int velocityLeftWheel = 0;
    int velocityRightWheel = 0;
    calculateWheelsVelocities(targetPosition, velocityLeftWheel, velocityRightWheel);
    
    int dirA = 0;
    int dirB = 0;
    computePwmAndDirections(dirA, dirB, velocityRightWheel, velocityLeftWheel);
  
    driveArdumoto(MOTOR_A, dirA, velocityRightWheel);
    driveArdumoto(MOTOR_B, dirB, velocityLeftWheel);
  
  } else {
    stopRobot();  
  }
}


void calculateWheelsVelocities(const vector targetPosition, int &leftWheel, int &rightWheel){
  float velocity = calculateVelocity(targetPosition);
  double omega = angularPID.process();
  
  leftWheel = (2 * velocity - omega * DISTANCE_MOTOR_CENTER) / (2 * WHEEL_RADIUS);
  rightWheel = (2 * velocity + omega * DISTANCE_MOTOR_CENTER) / (2 * WHEEL_RADIUS);

  if (DEBUG){
          Serial.print("Velocity\n");
          Serial.print("VL ");Serial.print(leftWheel);Serial.print(" = (2 * ");
          Serial.print(velocity);Serial.print(" - ");Serial.print(omega);
          Serial.print(" * 7) / ( 2 * 3)\n");
          Serial.print("VR ");Serial.print(rightWheel);Serial.print(" = (2 * ");
          Serial.print(velocity);Serial.print(" + ");Serial.print(omega);
          Serial.print(" * 7) / ( 2 * 3)\n");
  }
  
}


// Check direction for motors and assign correct speed
void computePwmAndDirections(int &dirA, int &dirB, int &pwmA, int &pwmB){
  const int pwmGainConstant = 128;

  if (pwmA > 0){ // Go foward
    pwmA += pwmGainConstant;
    dirA = CW;
  
  } else if (pwmA < 0){ // Go backward
    pwmA -= pwmGainConstant;
    pwmA = abs(pwmA);
    dirA = CCW;
  
  } else { // Stay put
    pwmA = 0;
  }
  
  if (pwmB > 0){
    pwmB += pwmGainConstant;
    dirB = CW;
  
  } else if (pwmB < 0){
    pwmB -= pwmGainConstant;
    pwmB = abs(pwmB);
    dirB = CCW;
  
  } else {
    pwmB = 0;
  }

  if (DEBUG){
    Serial.print("PWMs:");Serial.print(pwmB);Serial.print("~");Serial.print(pwmA);
    Serial.print("\n");
  }

}

// ############################ ULTRASONIC METHODS ################################

void loopSensors(){
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {         
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  
     
      if (i == 0 && currentSensor == SONAR_NUM - 1){
        oneSensorCycle();
      }
      
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
}


//Se receber um sinal (eco), calcula a distancia
void echoCheck() { 
  if (sonar[currentSensor].check_timer()) {
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
  }
}


void oneSensorCycle() { 
  // Ciclo de leitura do sensor
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    //Imprime os valores lidos pelos sensores, no serial monitor
    Serial.print("Sensor : ");
    Serial.print(i); 
    Serial.print(" = ");
    Serial.print(cm[i]);
    Serial.print(" cm - ");
  }
  Serial.println();
}


void differentialDrive() {
    //When the two wheels are spinning in the same direction 
    //and the same speed, the robot will move straight.
    
    //driveArdumoto(MOTOR_A, )
    //driveArdumoto(MOTOR_B, )
    
    //When the two wheels are spinning in the same direction, 
    //but with different speeds, the robot will turn away from 
    //the faster motor. For example, if the right wheel is spinning 
    //faster than the left, the motor will turn left.
    
    
    //If the two wheels are spinning with the same speed, 
    //but in opposite directions, the robot will rotate in place, 
    //spinning around the midpoint between the two wheels.
}



