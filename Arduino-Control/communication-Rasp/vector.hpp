#include <math.h> 

// Structure to represent vector and vector operations
// Operations based on angles
struct vector{
    int xPos;
    int yPos;
    
    vector(int x = 0, int y = 0)
    : xPos(x), yPos(y) {}
    
    int calculateMagnitude(){
        return sqrt(pow(xPos,2) + pow(yPos,2));
    }
    
    int getAngle(){
        return atan2(yPos, xPos) * 180/M_PI;
    }
    
    // Calculate the difference vector and return its magnitude
    // Used for target's distance walked 
    int getDifferenceVectorMag(const vector vec){
        vector aux = vector((xPos - vec.xPos), (yPos - vec.yPos));
        return aux.calculateMagnitude();
    }
    
    bool operator==(const vector &vec){
        return (vec.xPos == xPos) && (vec.yPos == yPos);
    }
    
    bool operator>(vector &vec){
        int magnitude = calculateMagnitude();
        int vecMagnitude = vec.calculateMagnitude();
        
        return (magnitude > vecMagnitude);
    }
    
    bool operator<(vector &vec){
        return !(*this > vec);
    }
    
    int operator+(vector &vec){
        return getAngle() - vec.getAngle();
    }
    
    // Angular error
    int operator-(vector &vec){
        return getAngle() - vec.getAngle();
    }
    
    operator int const(){
        return getAngle();
    }
};

// Calculate velocity of the target to imitate it
int calculateVelocity(vector oldSample, vector newSample, long &lastTime, int DEBUG = 0){
    int distanceRun = newSample.getDifferenceVectorMag(oldSample);
    int deltaTime = (millis() - lastTime) / 1000; // Seconds
    int velocity =  distanceRun / deltaTime; // cm/s

    if (DEBUG){
      Serial.print("Delta Time = ");
      Serial.print(deltaTime);
      Serial.println();
    }
    
    lastTime = millis();
    return velocity;
}

int calculateVelocity(vector newSample){
  int distance = newSample.calculateMagnitude();
  int minDistance = 20;
  int maxDistance = 50;
  int minPWM = 0;
  int maxPWM = 200;
  int velocity = map(distance, minDistance, maxDistance, minPWM, maxPWM);

  return velocity;
}

