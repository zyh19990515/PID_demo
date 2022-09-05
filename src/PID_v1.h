#ifndef PID_v1_h
#define PID_v1_h



class PID
{
private:
    /* data */
    void Initialize();

    double disKp;
    double disKi;
    double disKd;

    double kp;
    double ki;
    double kd;

    int controllerDirection;
    int pOn;

    double* myInput;
    double* myOnput;
    double* mySetpoint;

    double lastDv;
    double currDv;
    double originalDv;
    double alpha;
    unsigned long lastTime;
    double outputSum, lastInput;
    double lastError;

    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto, pOnE;

public:

    #define AUTOMATIC	1
    #define MANUAL	0
    #define DIRECT  0
    #define REVERSE  1
    #define P_ON_M 0
    #define P_ON_E 1

    PID(double*, double*, double*, double, double, double, int, int, double);//构造函数，参数为输入、输出、指定点、PID参数(指定比例模式的重载函数)
    //PID(double*, double*, double*, double, double, double, int);
    //void SetMode(int mode);//设定PID为人控模式或自动模式，v1只有自动模式，此函数暂时不用
    bool compute();//PID主程序
    void SetOutputLimits(double, double);//将输出限定在指定范围，通常为0-255
    //Display 函数
    double GetKp();
    double GetKi();
    double GetKd();
    int GetMode();
    int GetDirection();
    double GetCurrDv();
    double GetOriginalDv();

    void SetTunings(double, double, double, int);
    void SetControllerDirection(int);
    void SetSampleTime(int);
    void SetMode(int Mode);
    //~PID();
};

// PID::PID(/* args */)
// {
// }

// PID::~PID()
// {
// }





#endif