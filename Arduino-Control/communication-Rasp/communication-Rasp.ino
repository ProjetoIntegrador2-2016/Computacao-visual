#include <NewPing.h>
#include "pid.hpp"
#include "vector.hpp"
#include "motors.hpp"

// ############################### CONSTANTS ########################################

// Define constraints of the robot
#define DISTANCE_MOTOR_CENTER  48
#define WHEEL_RADIUS 6

// Utrasonic sensor definitions
#define SONAR_NUM     3
#define MAX_DISTANCE  90 // Distancia de deteccao
#define MIN_DISTANCE  50
#define PING_INTERVAL 33 //Intervalo entre as medicoes - valor minimo 29ms

int state = 0;
vector old(0, 0);
long lastTime;
long waitTime = 1000;

const int GAIN = 60;

const byte DEBUG = 3;

// Pin definitions for Ultrasonic sensors
const byte TRIGGER_F = 25;
const byte ECHO_F = 24;
const byte TRIGGER_R = 29;
const byte ECHO_R = 28;
const byte TRIGGER_L = 23;
const byte ECHO_L = 22;

vector setpoint(0.0, 50.0);

// Ultrasonic sensors globals
unsigned long pingTimer[SONAR_NUM]; // Vezes que a medicao deve ocorrer
unsigned int cm[SONAR_NUM]; //Armazena o numero de medicoes
uint8_t currentSensor = 0; // Armazena o numero do sensor ativo

const byte FRONT = 0;
const byte RIGHT = 1;
const byte LEFT = 2;

// Assign Ultrassonic sensors to pins
NewPing sonar[SONAR_NUM] = { 
  NewPing(TRIGGER_F, ECHO_F, MAX_DISTANCE),
  NewPing(TRIGGER_R, ECHO_R, MAX_DISTANCE),
  NewPing(TRIGGER_L, ECHO_L, MAX_DISTANCE),
};


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
  
  Serial.print("Ready to roll!");
  
}

// ################################## LOOP ########################################

void loop(){
  
  
  float sampleXY[2];
  
  if (Serial1.available() > 0){
    // Read from serial
    sampleXY[0] = Serial1.parseFloat();
    sampleXY[1] = Serial1.parseFloat();
    
    //vector sample = checkValues(sampleXY[0], sampleXY[1]);
    vector sample(sampleXY[0], sampleXY[1]);
    Serial.print("X position: ");
    Serial.print(sampleXY[0]);
    Serial.print("----");
    Serial.print("Y position: ");
    Serial.print(sampleXY[1]);
    Serial.print("\n");

    if (lastTime + waitTime <= millis()){
      
      if (sampleXY[1] > 0 ) {
        
        loopSensors();


         if (sampleXY[0] < -4.0){

            if (cm[LEFT] < MIN_DISTANCE && cm[LEFT] > 0){
              goFoward(sample);
              
            } else if (!hasObstacle(FRONT)) {
              goLeft(sample);  
              
            } else {
              stopRobot();
            }
            
         } else if (sampleXY[0] > 4.0){
           
           if (hasObstacle(RIGHT)){
              goFoward(sample);
              
           } else if (!hasObstacle(FRONT)) {
              goRight(sample);
              
           }else {
              stopRobot();
           }
         
         } else {
            if (hasObstacle(FRONT)){
                stopRobot();
             } else {
                goFoward(sample);
             }
         
         }
        
        /* 
        if(sampleXY[0]>=-8.0 && sampleXY[0]<=8.0){
            goFoward(sample);
        }else if (sampleXY[0]<-8.0){
           goLeft(sample);
        }else{
            goRight(sample);
        }
        */
        
        old=sample;  
  
      } else { // Target not found, go to last known position then search it
        
        int dirA = 80;
        int dirB = 80;
        
        if (old.xPos < 0){ // Last position is to the left
          dirB = 0;
        } else { // Last position to the right, spin right
          dirA = 0;
        }
        
        for(int i = 0 ; i< 80; i++){
          
          if (dirA > 0){
            driveArdumoto(MOTOR_A, i);
            delay(5); 
          } else {
            driveArdumoto(MOTOR_B, i+20);
            delay(5); 
          } 
          
         }

        delay(50); 
        stopRobot();
        lastTime = millis();
        waitTime = 500;
   
      } 
       
    } else {
      Serial.print("Wait time...\n");
    }
    
    
  }

}

// ############################ CONTROL METHODS ###################################


void goFoward(vector targetPosition){
  vector nullVector(0.0, 0.0);
  
  int velocityLeftWheel = 20.0;
  int velocityRightWheel = velocityLeftWheel + GAIN;
  
  if (targetPosition != nullVector && targetPosition.calculateMagnitude() > MIN_DISTANCE){
    if (state ==1){
          
            driveArdumoto(MOTOR_A, velocityRightWheel);
            driveArdumoto(MOTOR_B, velocityLeftWheel);
    }else{
            int velocityLeftWheel = 0;
            int velocityRightWheel = 0;

            for(int i = 0 ; i< 255; i++){
          
            driveArdumoto(MOTOR_A, i);
            driveArdumoto(MOTOR_B, i);
            }

            driveArdumoto(MOTOR_A, velocityRightWheel);
            driveArdumoto(MOTOR_B, velocityLeftWheel);

            state =1; 
            
      }
  
  } else {
    stopRobot();
    state =0 ;  
  }
}

void goLeft(vector targetPosition){
  vector nullVector(0.0, 0.0);


  int velocityLeftWheel = 20.0;
  int velocityRightWheel = velocityLeftWheel + GAIN + 40;

  
  if (targetPosition != nullVector && targetPosition.calculateMagnitude() > MIN_DISTANCE){
    if (state ==1){
          
            driveArdumoto(MOTOR_A, velocityRightWheel);
            driveArdumoto(MOTOR_B, velocityLeftWheel);
    }else{
            int velocityLeftWheel = 0;
            int velocityRightWheel = 0;

            for(int i = 0 ; i< 255; i++){
          
            driveArdumoto(MOTOR_A, i);
            driveArdumoto(MOTOR_B, i);
            }

            driveArdumoto(MOTOR_A, velocityRightWheel);
            driveArdumoto(MOTOR_B, velocityLeftWheel);

            state =1; 
            
      }
  
  } else {
    stopRobot();
    state =0 ;  
  }
}

void goRight(vector targetPosition){
  vector nullVector(0.0, 0.0);


  int velocityLeftWheel = 20.0 + 40.0;
  int velocityRightWheel = velocityLeftWheel + GAIN;

  
  if (targetPosition != nullVector && targetPosition.calculateMagnitude() > MIN_DISTANCE){
    if (state ==1){
          
            driveArdumoto(MOTOR_A, velocityRightWheel);
            driveArdumoto(MOTOR_B, velocityLeftWheel);
    }else{
            int velocityLeftWheel = 0;
            int velocityRightWheel = 0;

            for(int i = 0 ; i< 255; i++){
          
            driveArdumoto(MOTOR_A, i);
            driveArdumoto(MOTOR_B, i);
            }

            driveArdumoto(MOTOR_A, velocityRightWheel);
            driveArdumoto(MOTOR_B, velocityLeftWheel);

            state =1; 
            
      }
  
  } else {
    stopRobot();
    state =0 ;  
  }
}

// ############################ ULTRASONIC METHODS ################################

bool hasObstacle(int sensor){
  bool has = false;
  
  if (cm[sensor] < MIN_DISTANCE && cm[sensor] > 0){
    has = true;
  }

  return has;
}

void loopSensors(){
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (millis() >= pingTimer[i]) {         
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  
     
      if (i == 0 && currentSensor == SONAR_NUM - 1){
        if(DEBUG == 3){
          oneSensorCycle();
        }
      }
      delay(10);
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

