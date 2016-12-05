// Generic PID class
template <class T>
class PID{
public:
    
    int error;
    T sample;
    T lastSample;
    int kP, kI, kD;
    int I;
    
    T setPoint;
    long lastProcess;
    
    PID(int _kP, int _kI, int _kD, T _set){
        kP = _kP;
        kI = _kI;
        kD = _kD;
        I = 0;
        setSetPoint(_set);
    }
    
    int addNewSample(T _sample){
        sample = _sample;
        
        return process();
    }
    
    void setSetPoint(T _setPoint){
        setPoint = _setPoint;
    }
    
    int process(){
        error = setPoint - sample;
        
        int deltaTime = (millis() - lastProcess) / 1000;
        lastProcess = millis();
        
        int P = error * kP;
        I = I + (error * kI) * deltaTime;
        int D = (lastSample - sample) * kD / deltaTime;
        lastSample = sample;
        
        int pid = P + I + D;
        
        return pid;
    }

    void reset(){
      I = 0;
    }
};

