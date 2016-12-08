#include <math.h>

// Generic PID class
template <class T>
class PID{
public:
    
    double error;
    T sample;
    T lastSample;
    double kP, kI, kD;
    double I;
    double lastPid;
    T setPoint;
    long lastProcess;
    
    PID(double _kP, double _kI, double _kD, T _set){
        kP = _kP;
        kI = _kI;
        kD = _kD;
        I = 0;
        lastPid = 0;
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
        double pid = 0;
      
        error = setPoint - sample;

        if (!isnan(error)){ 
          
          float deltaTime = (millis() - lastProcess + 100) / 1000.0;
          //Serial.print(deltaTime);
          lastProcess = millis();
          
          double P = error * kP;
          I = I + (error * kI) * deltaTime;
          double D = (lastSample - sample) * kD / deltaTime;
          lastSample = sample;
          
          pid = P + I + D;
          lastPid = pid;
          
        } else {
          pid = lastPid;  
        
        }
        
        return pid;
    }

    void reset(){
      I = 0;
      lastPid = 0;
    }
};

