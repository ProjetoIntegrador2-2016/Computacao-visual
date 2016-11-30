#include <NewPing.h>
#include <math.h> 

// Structure to represent vector and vector operations
// Operations based on angles
struct vector{
    float xPos;
    float yPos;
    
    vector(float x = 0.0, float y = 0.0)
    : xPos(x), yPos(y) {}
    
    float calculateMagnitude(){
        return sqrt(pow(xPos,2) + pow(yPos,2));
    }
    
    float getAngle(){
        return atan2(yPos, xPos) * 180/M_PI;
    }
    
    // Calculate the difference vector and return its magnitude
    // Used for target's distance walked 
    float getDifferenceVectorMag(const vector vec){
        vector aux = vector((xPos - vec.xPos), (yPos - vec.yPos));
        return aux.calculateMagnitude();
    }
    
    bool operator==(const vector &vec){
        return (vec.xPos == xPos) && (vec.yPos == yPos);
    }
    
    bool operator>(vector &vec){
        float magnitude = calculateMagnitude();
        float vecMagnitude = vec.calculateMagnitude();
        
        return (magnitude > vecMagnitude);
    }
    
    bool operator<(vector &vec){
        return !(*this > vec);
    }
    
    float operator+(vector &vec){
        return getAngle() - vec.getAngle();
    }
    
    // Angular error
    float operator-(vector &vec){
        return getAngle() - vec.getAngle();
    }
    
    operator double const(){
        return getAngle();
    }
};

// Generic PID class
template <class T>
class PID{
public:
    
    double error;
    T sample;
    T lastSample;
    double kP, kI, kD;
    double I;
    
    T setPoint;
    long lastProcess;
    
    PID(double _kP, double _kI, double _kD, T _set){
        kP = _kP;
        kI = _kI;
        kD = _kD;
        I = 0;
        setSetPoint(_set);
    }
    
    double addNewSample(T _sample){
        sample = _sample;
        
        return process();
    }
    
    void setSetPoint(T _setPoint){
        setPoint = _setPoint;
    }
    
    double process(){
        error = setPoint - sample;
        
        float deltaTime = (millis() - lastProcess) / 1000.0;
        lastProcess = millis();
        
        double P = error * kP;
        I = I + (error * kI) * deltaTime;
        double D = (lastSample - sample) * kD / deltaTime;
        lastSample = sample;
        
        double pid = P + I + D;
        
        return pid;
    }
    
};

// Clockwise and counter-clockwise definitions.
#define CW  1
#define CCW 0

// Motor definitions to make life easier:
#define MOTOR_A 0
#define MOTOR_B 1

// Define constraints of the robot
#define DISTANCE_MOTOR_CENTER  7
#define WHEEL_RADIUS 3

// Utrasonic sensor definitions
#define SONAR_NUM     3 //Numero de sensores
#define MAX_DISTANCE  50 //Distancia maxima de deteccao
#define MIN_DISTANCE  20
#define PING_INTERVAL 33 //Intervalo entre as medicoes - valor minimo 29ms

// Map pwm values to distance
#define PWM(x)  map(x, 20, 50, 0, 200)

// Pin definitions for Motors
const byte PWMA = 3;  // PWM control (speed) for motor A
const byte PWMB = 11; // PWM control (speed) for motor B
const byte DIRA = 12; // Direction control for motor A
const byte DIRB = 13; // Direction control for motor B

// Pin definitions for Ultrasonic sensors
const byte TRIGGER_F = 7;
const byte ECHO_F = 6;
const byte TRIGGER_R = 10;
const byte ECHO_R = 9;
const byte TRIGGER_L = 5;
const byte ECHO_L = 4;

// Debug Constant
const byte DEBUG = 1;

// PID Globals
PID<float> distancePID(0.5, 0.025, 0, 20); // SetPoint 20 (distance)
PID<vector> angularPID(0.5, 0.025, 0, vector(0.0, 20.0)); // SetPoint 90degree

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

void setup(){
  Serial.begin(19200);
  Serial1.begin(19200);
  setupArdumoto();
  
 //Inicia a primeira medicao com 75ms
  pingTimer[0] = millis() + 75;
  //Define o tempo de inicializacao de cada sensor
  for (uint8_t i = 1; i < SONAR_NUM; i++){
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
    }
    
    lastTime = millis();
}

void loop(){
  
  //loopSensors();
  
  float sampleXY[2];
  if (Serial1.available() > 0){
    
    sampleXY[0] = Serial1.parseFloat();
    sampleXY[1] = Serial1.parseFloat();
    Serial1.flush();
    
    vector sample(sampleXY[0], sampleXY[1]);
    
    double omega = angularPID.addNewSample(sample);
    //double distanceGain1 = distancePID.addSample(sample.calculateMagnitude());
    
    if (oldSample.calculateMagnitude() > 0){
        float velocity = calculateVelocity(sample, oldSample, lastTime);
        
        int vr = (2 * velocity + omega * DISTANCE_MOTOR_CENTER) / (2 * WHEEL_RADIUS);
        int vl = (2 * velocity - omega * DISTANCE_MOTOR_CENTER) / (2 * WHEEL_RADIUS);
        
        if (DEBUG){
          Serial.print("Velocity\n");
          Serial.print(vl);
          Serial.print("|");
          Serial.print(vr);
          Serial.print("\n");
        }
    }
    
   oldSample = sample;
    
    if (DEBUG){
        Serial.print(sampleXY[0]);
        Serial.print("--");
        Serial.print(sampleXY[1]);
        Serial.println();
    }
  }

}


float calculateVelocity(vector oldSample, vector newSample, long &lastTime){
    float distanceRun = newSample.getDifferenceVectorMag(oldSample);
    float deltaTime = (millis() - lastTime) / 1000.0; // Seconds
    int velocity =  distanceRun / deltaTime; // cm/s

    if (DEBUG){
      Serial.print("Delta Time = ");
      Serial.print(deltaTime);
      Serial.println();
    }
    
    lastTime = millis();
    return velocity;
}


// driveArdumoto drives 'motor' in 'dir' direction at 'spd' speed
void driveArdumoto(byte motor, byte dir, byte spd) {
  if (motor == MOTOR_A) {
    digitalWrite(DIRA, dir);
    analogWrite(PWMA, spd);
    
  } else if (motor == MOTOR_B) {
    digitalWrite(DIRB, dir);
    analogWrite(PWMB, spd);
  }  
}

void stopArdumoto(byte motor) {
  driveArdumoto(motor, 0, 0);
}

// setupArdumoto initialize all pins
void setupArdumoto() {
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

void echoCheck() { 
  //Se receber um sinal (eco), calcula a distancia
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



