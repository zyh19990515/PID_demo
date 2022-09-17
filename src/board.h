#include<Arduino.h>


#define YM        5    
#define YP        6  
#define XM        7  
#define XP        8  

struct TCPoint{
  int x;
  int y;
};


void bdInit();
TCPoint posDetect();
double speedDetect();