#include "PID.h"
#include <math.h>
/*
Author: LiuYuanlin
Date:   2021年12月29日
Time:   10点49分
Project:PID library 2.0
*/

/**
 * Get x sign bit only for little-endian
 * if x >= 0 then  1
 * if x <  0 then -1
 */
#define Sign(x) ((x) >= 0 ? 1 : -1)

void IncPIDInit(pPID self)
{
    self->Target = 0; //设定值,The "black box"'s output
    self->iSampling = 0;
    self->Step = 1; //返回结果比例

    self->errK = 0;   //当前误差 e(k)
    self->errKm1 = 0; //前1拍误差 e(k-1)
    self->errKm2 = 0; //前2拍误差 e(k-2)

    self->P = 0.f; //比例
    self->I = 0.f; //积分
    self->D = 0.f; //微分

    self->Integral = 0; //积分累加器,Just using in the position type PID.

    self->F = 0; //输出返回值,The control point of output
    self->Fmax = 1.0;
    self->Fmin = 0;
    self->dt = 1;
    self->sysArg = 0;

    self->pidDecayByAbsErrorFunc = PID_DECAY_FUNC_NULL;
}


void PosPIDCalc_NormalizedF(pPID self)
{
    //当前误差 calculate current error
    self->errK = self->Target - self->iSampling;

    // Proportional term
    float Pout = self->P * self->errK;

    // Integral term
    self->Integral += self->errK;
    float Iout = self->I * self->Integral;

    // Derivative term
    float Dout = self->D * (self->errK - self->errKm2);

    // Calculate total output
    self->errKm2 = self->errK;

    float F = Pout + Iout + Dout;

    // Restrict to max/min
    if (F >= self->Fmax)
        F = self->Fmax;
    else if (F <= self->Fmin)
        F = self->Fmin;

    self->F = F;
}


void IncPIDCalc_NormalizedF(pPID self)
{
    //当前误差 calculate current error
    self->errK = self->Target - self->iSampling;
    float delta = 0;
    float F = 0;
    delta = self->P * (self->errK - self->errKm1) +
            self->I * self->errK +
            self->D * (self->errK - 2*self->errKm1 + self->errKm2);

    // Calculate total output
    self->errKm2 = self->errKm1; // 移位: e(k-1) -> e(k-2)
    self->errKm1 = self->errK;    // 移位: e(k) -> e(k-1)

    F = self->F + delta;

    // Restrict to max/min
    if (F >= self->Fmax)
        F = self->Fmax;
    else if (F <= self->Fmin)
        F = self->Fmin;

    self->F = F;
}

void IncPIDCalcDelta_NormalizedF_TwoStage(pPID self, float FirstContrlPoint, float LasttContrlPoint, float FirstContrlScale)
{

    float F = 0;
    float absError = 0;

    //当前误差 calculate current error
    self->errK = self->Target - self->iSampling;
    absError = fabsf(self->errK);

    /*
    Please don't use auto formate to format this file, 
    because the pid calculate code is too long and it will
    be inconvenient to view if you use auto formate.
    */

    //Here , Think of F as delta.
    F = self->P * (self->errK - self->errKm1) +
        self->I * self->errK +
        self->D * (self->errK - 2 * self->errKm1 + self->errKm2);

    if (absError >= LasttContrlPoint && absError <= FirstContrlPoint)
    {
        F *= FirstContrlScale;
    }
    else
    {
        F = 0;
        self->errK = 0;
    }

    // Calculate total output
    self->errKm2 = self->errKm1; // 移位: e(k-1) -> e(k-2)
    self->errKm1 = self->errK;    // 移位: e(k) -> e(k-1)

    F = self->F + F;
    // Restrict to max/min
    if (F >= self->Fmax)
        F = self->Fmax;
    else if (F <= self->Fmin)
        F = self->Fmin;

    //update self->F at the end in order to avoid the impact of intermediate results on system running.
    self->F = F;
}

void IncPIDCalcDelta_NormalizedF_Decay(pPID self,PIDDecayFun deacyfun)
{

    float F = 0;

    //当前误差 calculate current error
    self->errK = self->Target - self->iSampling;
    /*
    Please don't use auto formate to format this file, 
    because the pid calculate code is too long and it will
    be inconvenient to view if you use auto formate.
    */

    //Here , Think of F as delta.
    F = self->P * (self->errK - self->errKm1) +
        self->I * self->errK +
        self->D * (self->errK - 2 * self->errKm1 + self->errKm2);

    F *= deacyfun(self->errK);

    // Calculate total output
    self->errKm2 = self->errKm1; // 移位: e(k-1) -> e(k-2)
    self->errKm1 = self->errK;    // 移位: e(k) -> e(k-1)

    F = self->F + F;
    // Restrict to max/min
    if (F >= self->Fmax)
        F = self->Fmax;
    else if (F <= self->Fmin)
        F = self->Fmin;

    //update self->F at the end in order to avoid the impact of intermediate results on system running.
    self->F = F;
}


void IncPIDCalcDelta_NormalizedFAndDecayFunInput_Decay(pPID self,PIDDecayFun deacyfun)
{
    float F = 0;
    
    //当前误差 calculate current error
    self->errK = self->Target - self->iSampling;
    /*
    Please don't use auto formate to format this file, 
    because the pid calculate code is too long and it will
    be inconvenient to view if you use auto formate.
    */

    //Here , Think of F as delta.
    F = self->P * (self->errK - self->errKm1) +
        self->I *  self->errK +
        self->D * (self->errK - 2 * self->errKm1 + self->errKm2);

    F *= deacyfun(self->errK / self->sysArg);

    // Calculate total output
    self->errKm2 = self->errKm1; // 移位: e(k-1) -> e(k-2)
    self->errKm1 = self->errK;    // 移位: e(k) -> e(k-1)

    F = self->F + F;
    // Restrict to max/min
    if (F >= self->Fmax)
        F = self->Fmax;
    else if (F <= self->Fmin)
        F = self->Fmin;

    //update self->F at the end in order to avoid the impact of intermediate results on system running.
    self->F = F;
}

void IncPIDCalcDeltaAutoDecay(pPID self){
    float F = 0;
    
    //当前误差 calculate current error
    self->errK = self->Target - self->iSampling;
    /*
    Please don't use auto formate to format this file, 
    because the pid calculate code is too long and it will
    be inconvenient to view if you use auto formate.
    */

    //Here , Think of F as delta.
    F = self->P * (self->errK - self->errKm1) +
        self->I * self->errK +
        self->D * (self->errK - 2 * self->errKm1 + self->errKm2);
    if(self->pidDecayByAbsErrorFunc != PID_DECAY_FUNC_NULL)
        F *= self->pidDecayByAbsErrorFunc(self->errK);

    // Calculate total output
    self->errKm2 = self->errKm1; // 移位: e(k-1) -> e(k-2)
    self->errKm1 = self->errK;    // 移位: e(k) -> e(k-1)

    F = self->F + F;
    // Restrict to max/min
    if (F >= self->Fmax)
        F = self->Fmax;
    else if (F <= self->Fmin)
        F = self->Fmin;

    //update self->F at the end in order to avoid the impact of intermediate results on system running.
    self->F = F;
}
/*
a > 0
y = 1 - \frac{4e^{-ax}}{(1+e^{-ax})^2} = \tanh^2(ax/2)
*/
float dsigmoidn(float z, float a)
{
    float e_ = exp(-a * z);
    float q_ = 1.0 + e_;
    return 1 - 4 * e_ / (q_ * q_);
}

/*
a > 0 and b > 0
y = 1/(1+b*e^(-a|x|))
Be careful of spills and Normalizedize errors if you use them.
*/
float sigmoidabsx(float z, float a, float b)
{
    z = fabsf(z);
    return 1.0f / (1 + b * exp(-a * z));
}

/*
a > 0
y = tanh(a|z|)
Be careful of spills and Normalizedize errors if you use them.
*/
float tanhabsx(float z, float a)
{
    z = fabsf(z);
    return tanh(a * z);
}

/*
a > 0
y = 1 - 1/sqrt(a|x|+1)
*/
float px1(float z, float a)
{
    z = fabsf(z);
    return 1 - 1.0f / sqrt(a * z + 1);
}

/*
x > 0
y = 1/x*|X|, |X| < x 
y = 1, |X| > x 
*/
float obliquestepfun(float z, float x)
{
    z = fabsf(z);
    if (z > x)
    {
        return 1;
    }
    else
    {
        return 1 / x * z;
    }
}