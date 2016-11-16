#include <NewPing.h>
#include <math.h> 

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

#define DISTANCE_MOTOR_CENTER  7
#define WHEEL_RADIUS 3

const byte PWMA = 3;  // PWM control (speed) for motor A
const byte PWMB = 11; // PWM control (speed) for motor B
const byte DIRA = 12; // Direction control for motor A
const byte DIRB = 13; // Direction control for motor B

char incoming = 'i';
int last_cw = CW;
double error_sum = 0;
double old_error = 0;
double Kp = 0.5;
double Ki = 0.025;
double old_phi = PI/2; // Start looking foward

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

char c = 'n';
String buffer = "";

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

long readLongFromBytes() {
  union l_tag {
    byte b[4];
    long lval;
    } l;
    
  l.b[0] = Serial1.read();
  l.b[1] = Serial1.read();
  l.b[2] = Serial1.read();
  l.b[3] = Serial1.read();
  return l.lval;
}

void setup()
{
  Serial.begin(19200);
  //Serial1.begin(19200);
  setupArdumoto();
/*
 //Inicia a primeira medicao com 75ms
  pingTimer[0] = millis() + 75;
  //Define o tempo de inicializacao de cada sensor
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
*/
}

void loop()
{

   //Faz um loop entre todos os sensores
  /*
   for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop entre todos os sensores
    if (millis() >= pingTimer[i]) {         
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  
      
      /*
      if (i == 0 && currentSensor == SONAR_NUM - 1){
        oneSensorCycle();
      }
      
      
      sonar[currentSensor].timer_stop();
      currentSensor = i;
      cm[currentSensor] = 0;
      sonar[currentSensor].ping_timer(echoCheck);
    }
  }
 */
  
  byte val[2];
  if (Serial.available()){
    
    while(Serial.available() < 2){}
    for(int i=0; i<2; i++){
      val[i] = Serial.read();
    }
    Serial.flush();
    /*
    Serial.print(val[0]);
    Serial.print("--");
    Serial.print(val[1]);
    Serial.println();
    */
    driveArdumoto(MOTOR_A, CW, val[0]);
    driveArdumoto(MOTOR_B, CW, val[1]);
  }
  
  /*
   * double pos[2] = {-1, -1}; // x and y position of the target
  Serial.print(buffer[buffer.length()-1]);
  if(buffer[buffer.length()-1] == '>'){
    
    for(int i = 0; i < 2; i++){
      int index = buffer.indexOf(","); // Find the next comma
      pos[i] = atol(buffer.substring(0, index).c_str()); // Extract the number
      buffer = buffer.substring(index+1); // Exclude the number read
    }
 
    double phi_desired = 0;
    
    if (pos[1] != 0){
      phi_desired = atan2(pos[0],pos[1]);
      if (phi_desired < 0){
        phi_desired += PI;
      }
    
      // -------------- PID
      double error = phi_desired - old_phi;
      //double error_dot = old_error; Used for Kd
      error_sum = error_sum + error;
    
      double gain = Kp * error + Ki * error_sum;
      //old_error = error;
      // -------------- END PID
    
      double velocity = pos[0]/cos(phi_desired);
      Serial.print(pos[0]);
      Serial.print("-");
      Serial.print(pos[1]);
      Serial.print("--------");
      Serial.print(gain);
    }
  
    old_phi = phi_desired;
  }
 */
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
