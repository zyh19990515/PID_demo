/*
舵机控制线接口D9、D10
电阻屏接口：
3.3V - VCC
GND - GND
D8 - X+
D7 - X-
D6 - Y+
D5 - Y-
A2 - Xout
A3 - Yout
*/


#include <Arduino.h>
#include <Servo.h>
#include<board.h>
#include<PID_v1.h>
#include<KalmanFilter.h>
// #define X1 A0 
// #define X2 A2 
// #define Y1 A3 
// #define Y2 A1 
#define Xresolution 237
#define Yresolution 195

#define SvrXPin 9
#define SvrYPin 10 
int    Interval  = 25;
double  Setpoint, Input, Output;
double  Setpoint1, Input1, Output1;
Servo  servoX;
Servo servoY;
#define ANGLE_MIN 60
#define ANGLE_MAX 120
//
//float  Kp  = 0.3;
//float Ki  = 0.06;
//float Kd  = 0.1;
//float Kp1 = 0.25;
//float Ki1 = 0.06;
//float Kd1 = 0.08;

float  Kp  = 0.32;
float Ki  = 0.05;
float Kd  = 0.095;
float Kp1 = 0.27;
float Ki1 = 0.05;
float Kd1 = 0.085;
double  R = 0;

float my_A = 1;
float my_C = 1;
float my_P_ = 1;
float my_P = 1;
float my_Q = 0.9;
float my_R = 1;
float my_x_ = 0;

int NoDataFrame = 0;
int InitFrame = 0;
bool SysInit = false;

int ServoXBlanceOut = 90; // x轴舵机初始水平输出 控制y轴运动
int ServoYBlanceOut = 90; // y轴舵机初始水平输出 控制x轴运动

PID Pid( &Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT,0.3);
PID Pid1( &Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, P_ON_E, DIRECT,0.3);
KalmanFilter KF(my_A, my_C, my_P_, my_P, my_Q, my_R, my_x_);
KalmanFilter KF1(my_A, my_C, my_P_, my_P, my_Q, my_R, my_x_);
void printLog();
double ReserveOutput(double output, double center);
void setup() {
  // put your setup code here, to run once:
  bdInit();
  
  Serial.begin(115200);
  while (!Serial){
    Serial.print(".");
    delay(1000);
  }
  Setpoint = 119;
  Setpoint1 = 98;
  Input = 0;
  Input1 = 0;

  servoX.attach(SvrXPin);
  servoY.attach(SvrYPin);

  Output = ServoXBlanceOut;
  Output1 = ServoYBlanceOut;
  servoX.write(Output);
  servoY.write(Output1);

  Pid.SetMode(AUTOMATIC);
  Pid1.SetMode(AUTOMATIC);
  Pid.SetOutputLimits(ANGLE_MIN,ANGLE_MAX);
  Pid1.SetOutputLimits(ANGLE_MIN,ANGLE_MAX);
  Pid.SetSampleTime(Interval);
  Pid1.SetSampleTime(Interval);
  Serial.println("init finish");
  
  

}

void loop() {
  // put your main code here, to run repeatedly:
  if(!SysInit){
    InitFrame++;
    SysInit = InitFrame>5;
  }

  TCPoint point = posDetect();
  if(point.x>0 && point.y>0){
    //servoX.attach(SvrXPin);
    //servoY.attach(SvrYPin);
    //double oldout = Output;
    //double oldout1 = Output1;
    Input = point.x;
    Input1 = point.y;

    Input = KF.estimate(Input);
    Input1 = KF1.estimate(Input1);
    Pid.compute();
    Pid1.compute();
    //Output = KF.estimate(Output);
    //Output1 = KF.estimate(Output1);
    servoX.write(ReserveOutput(Output, 119));
    servoY.write(Output1);
    printLog();
    
  }
}

double ReserveOutput(double output, double center)
{
  return output + 2.0*(center-output);
}

void printLog(){
  Serial.print("Input:");
  Serial.print(Input);
  Serial.print("Input1:");
  Serial.print(Input1);
  Serial.print("  Output:");
  Serial.print(Output);
  Serial.print("  Output1:");
  Serial.print(Output1);
  Serial.print("  CurrDv:");
  Serial.print(Pid.GetCurrDv());
  Serial.print("  CurrDv1:");
  Serial.print(Pid1.GetCurrDv());

}