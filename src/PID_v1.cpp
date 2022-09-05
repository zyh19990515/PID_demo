#include<Arduino.h>
#include "PID_v1.h"

PID::PID(double* Input, double* Output, double* Setpoint, 
        double Kp, double Ki, double Kd, int POn, int ControllerDirection, double Alpha = 0){
    myInput = Input;
    myOnput = Output;
    mySetpoint = Setpoint;
    inAuto = false;
    
    alpha = Alpha;
    currDv = 0;
    lastDv = 0;
    PID::SetOutputLimits(0, 255);
    SampleTime = 100;
    lastTime = millis()-SampleTime;

}

// PID::PID(double* Input, double* Output, double* Setpoint, 
//         double Kp, double Ki, double Kd, int POn, int ControllerDirection)
//         :PID::(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection){

//         }

bool PID::compute(){
    unsigned long now = millis();
    unsigned long timeChange = (now-lastTime);
    if(timeChange>=SampleTime){
        double input = *myInput;
        double error = *mySetpoint-input;
        double dInput = (input-lastInput);
        
        if(abs(error)<20){
            outputSum+=(ki*error);
        }
        if(!pOnE){
            outputSum-=kp*dInput;
        }
        if(outputSum>outMax){
            outputSum=outMax;
        }
        else if(outputSum<outMin){
            outputSum=outMin;
        }

        double output;
        if(pOnE){
            output = kp*error;
        }
        else{
            output = 0;
        }

        currDv = -kd*(dInput)*(1.0-alpha)+alpha*lastDv;
        originalDv = -kd*dInput;
        output += outputSum+currDv;
        if(output>outMax){
            output=outMax;
        }
        else if(output<outMin){
            output=outMin;
        }

        lastInput = input;
        lastTime = now;
        lastDv = currDv;
        lastError = error;
        return true;


    }
    return false;
}

void PID::Initialize(){
    outputSum = *myOnput;
    lastInput = *myInput;
    lastError = 0;
    if(outputSum>outMax){
        outputSum = outMax;
    }
    else if(outputSum<outMin){
        outputSum = outMin;
    }
}

void PID::SetTunings(double Kp, double Ki, double Kd, int POn){
    if(Kp<0 || Ki<0 || Kd<0){
        return;
    }
    pOn = POn;
    pOnE = POn == P_ON_E;

    disKp = Kp;
    disKi = Ki;
    disKd = Kd;
    double SampleTimeInSec = ((double)SampleTime)/1000;
    kp = Kp;
    ki = Ki*SampleTimeInSec;
    kp = Kp/SampleTimeInSec;
    if(controllerDirection == REVERSE){
        kp = (0-kp);
        ki = (0-ki);
        kd = (0-kd);
    }

}

void PID::SetSampleTime(int NewSampleTime){
    if(NewSampleTime>0){
        double ratio = (double)NewSampleTime/(double)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (unsigned long)NewSampleTime;
    }
}

void PID::SetOutputLimits(double Min, double Max){
    if(Min >= Max){
        return;
    }
    outMax = Max;
    outMin = Min;

    if(inAuto){
        if(*myOnput > outMax){
            *myOnput = outMax;
        }
        else if(*myOnput < outMin){
            *myOnput = outMin;
        }
        if(outputSum>outMax){
            outputSum = outMax;
        }
        else if(outputSum<outMin){
            outputSum = outMin;
        }
    }
}

void PID::SetMode(int Mode){
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto){
        PID::Initialize();
    }
    inAuto = newAuto;
}

void PID::SetControllerDirection(int Direction){
    if(inAuto && Direction != controllerDirection){
        kp = (0-kp);
        ki = (0-ki);
        kd = (0-kd);
    }
    controllerDirection = Direction;
}

double PID::GetKp(){ return  disKp; }
double PID::GetKi(){ return  disKi;}
double PID::GetKd(){ return  disKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}
double PID::GetCurrDv(){return currDv;}
double PID::GetOriginalDv(){return originalDv;}