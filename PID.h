#ifndef __PID_H
#define __PID_H

typedef struct PID
{
    float Target;
    float iSampling;//系统关测值/采样值
    float P;
    float I;
    float D;

    float iError;
    float LastError;
    float PrevError;

    float Integral; //Just using in the position type PID.

    float F;
    float Fmax;
    float Fmin;

    float dt;
    float Step;

    float sysArg;//因为这个值会用于每次反馈中系统参数的更新，所以可能选择直接用得到的调制度去乘以一个整数。

} PID, *pPID;

typedef float (*PIDDecayFun)(float);

void IncPIDInit(pPID self);
void PosPIDCalc_NormalSimple(pPID self);
void IncPIDCalc_NormalSimple(pPID self);


void IncPIDCalcDelta_Normal_TwoStage(pPID self, float FirstContrlPoint, float LasttContrlPoint, float FirstContrlScale);
void IncPIDCalcDelta_Normal_Decay(pPID self,PIDDecayFun deacyfun);
void IncPIDCalcDelta_NormalSampleAndF_Decay(pPID self,PIDDecayFun deacyfun);
#define PIDUpdateValue_P(self) ((self)->sysArg * (self)->F)
#define PIDUpdateValue_N(self) ((self)->sysArg * (1.0f - (self)->F))

float dsigmoidn(float z,float a);
float sigmoidabsx(float z,float a,float b);
float tanhabsx(float z,float a);
float px1(float z,float a);
float obliquestepfun(float z,float x);

void UserPIDInit(void);
#endif
