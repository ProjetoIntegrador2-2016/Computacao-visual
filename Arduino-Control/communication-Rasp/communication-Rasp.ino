#include <NewPing.h>

// Clockwise and counter-clockwise definitions.
// Depending on how you wired your motors, you may need to swap.
#define CW  1
#define CCW 0

// Motor definitions to make life easier:
#define MOTOR_A 0
#define MOTOR_B 1

#define STOP 's'
#define FOWARD 'f'
#define BACKWARD 'b'
#define LEFT 'l'
#define RIGHT 'r'
#define TURN_LEFT 'e'
#define TURN_RIGHT 'd'

const byte PWMA = 3;  // PWM control (speed) for motor A
const byte PWMB = 11; // PWM control (speed) for motor B
const byte DIRA = 12; // Direction control for motor A
const byte DIRB = 13; // Direction control for motor B

char incoming = 'n';
int last_cw = CW;

//Determina o numero de sensores no circuito
#define SONAR_NUM     3
//Determina a distancia maxima e minima de deteccao
#define MAX_DISTANCE 50
#define MIN_DISTANCE 20
//Intervalo de tempo entre as medicoes - valor minimo 29ms
#define PING_INTERVAL 33

//Armazena a quantidade de vezes que a medicao deve ocorrer, para cada sensor
unsigned long pingTimer[SONAR_NUM];
//Armazena o numero de medicoes
unsigned int cm[SONAR_NUM];
// Armazena o numero do sensor ativo
uint8_t currentSensor = 0;    

NewPing sonar[SONAR_NUM] = 
{ 
  //Inicializa os sensores nos pinos especificados
  //(pino_trigger, pino_echo, distancia_maxima)
  //Front Sensor 0
  NewPing(7, 6, MAX_DISTANCE),
  //Right Sensor 1
  NewPing(10, 9, MAX_DISTANCE),
  //Left Sensor 2
  NewPing(5, 4, MAX_DISTANCE),
};


void setup()
{
  Serial.begin(9600);
  setupArdumoto();

 //Inicia a primeira medicao com 75ms
  pingTimer[0] = millis() + 75;
  //Define o tempo de inicializacao de cada sensor
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  
}

void loop()
{

   //Faz um loop entre todos os sensores
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop entre todos os sensores
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
  
  if (Serial.available()){
    
    incoming = Serial.read();
    switch(incoming){
      case FOWARD:
        if(cm[0] > MIN_DISTANCE){
          driveArdumoto(MOTOR_A, CW, 100);
          driveArdumoto(MOTOR_B, CW, 100);
          last_cw = CW;  
          
        } else {
          stopArdumoto(MOTOR_A);
          stopArdumoto(MOTOR_B);
        }
      break;
      case BACKWARD:
        driveArdumoto(MOTOR_A, CCW, 100);
        driveArdumoto(MOTOR_B, CCW, 100);
        last_cw = CCW;
      break;
      case RIGHT:
        if(cm[1] > MIN_DISTANCE) {
          if (last_cw == CW) {
            analogWrite(PWMA, 50);
            analogWrite(PWMB, 200);
          } else {
            analogWrite(PWMA, 200);
            analogWrite(PWMB, 50);
          }
        } else if(cm[2] > MIN_DISTANCE) { // There is no object in left side
          // Continue last direction
          driveArdumoto(MOTOR_A, last_cw, 100);
          driveArdumoto(MOTOR_B, last_cw, 100);
          delay(2);
          
          // Go Right
          if (last_cw == CW) {
            analogWrite(PWMA, 50);
            analogWrite(PWMB, 200);
          } else {
            analogWrite(PWMA, 200);
            analogWrite(PWMB, 50);
          }
        }
        
      break;
      case LEFT:
        if(cm[2] > MIN_DISTANCE) {
          if (last_cw == CW){ 
            analogWrite(PWMA, 200);
            analogWrite(PWMB, 50);
          } else{
            analogWrite(PWMA, 50);
            analogWrite(PWMB, 200);
          }
        } else if(cm[1] > MIN_DISTANCE) {
          // Continue last direction
          driveArdumoto(MOTOR_A, last_cw, 100);
          driveArdumoto(MOTOR_B, last_cw, 100);
          delay(2);
          
          if (last_cw == CW){ 
            analogWrite(PWMA, 200);
            analogWrite(PWMB, 50);
          } else{
            analogWrite(PWMA, 50);
            analogWrite(PWMB, 200);
          }
        }
      break;
      case TURN_RIGHT:
         if(cm[1] > MIN_DISTANCE) {
           stopArdumoto(MOTOR_A);
           stopArdumoto(MOTOR_B);
           driveArdumoto(MOTOR_A, CCW, 200);
         } else {
           driveArdumoto(MOTOR_A, CW, 100);
           driveArdumoto(MOTOR_B, CW, 100);
           delay(5);
         }
      break;
      case TURN_LEFT:
        if(cm[2] > MIN_DISTANCE) {
           stopArdumoto(MOTOR_A);
           stopArdumoto(MOTOR_B);
           driveArdumoto(MOTOR_B, CCW, 200);
        } else {
           driveArdumoto(MOTOR_A, CW, 100);
           driveArdumoto(MOTOR_B, CW, 100);
           delay(5);          
        }
      break;
      default:
        stopArdumoto(MOTOR_A);
        stopArdumoto(MOTOR_B);
    }
    delay(1);
    
  }
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

// stopArdumoto makes a motor stop
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
