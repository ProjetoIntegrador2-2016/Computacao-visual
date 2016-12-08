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

// Calculate velocity of the target to imitate it
float calculateVelocity(vector oldSample, vector newSample, long &lastTime, int DEBUG = 0){
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

float calculateVelocity(vector newSample){
  float distance = newSample.calculateMagnitude();
  int minDistance = 40;
  int maxDistance = 90;
  int minPWM = 0;
  int maxPWM = 100;
  
  float velocity = map(distance, minDistance, maxDistance, minPWM, maxPWM);

  if (velocity > maxPWM){
    velocity = maxPWM;
  }
  
  return velocity;
}

