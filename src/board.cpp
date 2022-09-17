#include<Arduino.h>
#include<board.h>
int Position_X,Position_Y; //X Y方向的测量值
void bdInit(){
    pinMode(XM, OUTPUT);          //电机控制引脚
    pinMode(XP, OUTPUT);          //电机控制引脚，
    pinMode(YM, OUTPUT);          //电机速度控制引脚
    pinMode(YP, OUTPUT);          //电机速度控制引脚
}

TCPoint posDetect(){
    TCPoint point = {0,0};
    digitalWrite(YM, LOW);  //给X方向+3.3V电压
    digitalWrite(YP, HIGH);   
    digitalWrite(XM, HIGH);  
    digitalWrite(XP, LOW); 
    Position_Y=analogRead(3)*4/15; //测量Y方向的坐标          
    //Serial.println(analogRead(3));
    digitalWrite(YM, HIGH);  
    digitalWrite(YP, LOW); 
    digitalWrite(XM, LOW);  
    digitalWrite(XP, HIGH); 
    delay(100);
    digitalWrite(YM, HIGH);  //给Y方向+3.3V电压//D3
    digitalWrite(YP, LOW); //  
    digitalWrite(XM, LOW);  //  
    digitalWrite(XP, HIGH); //  
    Position_X= analogRead(2)/5; //测量X方向的           
    digitalWrite(YM, LOW);  //
    digitalWrite(YP, HIGH); //  
    digitalWrite(XM, HIGH);  //  
    digitalWrite(XP, LOW); //  
    // Serial.print("Position_X:"); 
    // Serial.print(Position_X,DEC); 
    // Serial.print("  Position_Y:"); 
    // Serial.println(Position_Y,DEC);
                //=====延时等待
    point.x=Position_X;
    point.y=Position_Y;
    return point;
}

// double speedDetect(){
    
// }