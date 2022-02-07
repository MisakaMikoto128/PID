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

    self->LastError = 0; //前2次误差值
    self->PrevError = 0; //前1次误差值

    self->P = 0.f; //比例
    self->I = 0.f; //积分
    self->D = 0.f; //微分

    self->iError = 0; //当前误差

    self->F = 0; //输出返回值,The control point of output
    self->Fmax = 1.0;
    self->Fmin = 0;
    self->dt = 1;
    self->sysArg = 0;

    self->pidDecayByAbsErrorFunc = PID_DECAY_FUNC_NULL;
}

/**
*@brief Position type PID  
*@return: None 
*/
void PosPIDCalc_NormalSimple(pPID self)
{
    float F = self->F;
    //当前误差
    self->iError = self->Target - self->iSampling;

    // Proportional term
    float Pout = self->P * self->iError;

    // Integral term
    self->Integral += self->iError;
    float Iout = self->I * self->Integral;

    // Derivative term
    float Dout = self->D * (self->iError - self->PrevError);

    // Calculate total output
    self->F = Pout + Iout + Dout;
    self->PrevError = self->iError;

    // Restrict to max/min
    if (F >= self->Fmax)
        F = self->Fmax;
    else if (F <= self->Fmin)
        F = self->Fmin;

    self->F = F;
}

/*
Fun:        incremental PID
Description:when Target minus SamplingValue equals is negitive,the normalized 
            system parameter, F , will increase, and if Target minus SamplingValue 
            equals is positive, then F will decrease.
            --> Target > iSampling -> F ↑, Target < iSampling -> F ↓
Return:     None          
*/
void IncPIDCalc_NormalSimple(pPID self)
{
    //当前误差
    self->iError = self->Target - self->iSampling;
    float delta = 0;
    float F = 0;
    delta = self->P * (self->iError - self->LastError) +
            self->I * self->iError +
            self->D * (self->iError - self->PrevError);

    // Calculate total output
    self->PrevError = self->LastError; // 更新前次误差
    self->LastError = self->iError;    // 更新上次误差

    F = self->F + delta;

    // Restrict to max/min
    if (F >= self->Fmax)
        F = self->Fmax;
    else if (F <= self->Fmin)
        F = self->Fmin;

    self->F = F;
}
/*
Fun:        incremental PID
Description:1. When Target minus SamplingValue equals is negitive,the normalized 
            system parameter, F , will increase, and if Target minus SamplingValue 
            equals is positive, then F will decrease.
            2. And the P,I,D parameter will multiply by FirstContrlScale when  
            LasttContrlPoint < absError < FirstContrlPoint.
            3. --> Target > iSampling -> F ↑, Target < iSampling -> F ↓
Return:     None          
*/
void IncPIDCalcDelta_Normal_TwoStage(pPID self, float FirstContrlPoint, float LasttContrlPoint, float FirstContrlScale)
{

    float F = 0;
    float absError = 0;

    //当前误差
    self->iError = self->Target - self->iSampling;
    absError = fabsf(self->iError);

    /*
    Please don't use auto formate to format this file, 
    because the pid calculate code is too long and it will
    be inconvenient to view if you use auto formate.
    */

    //Here , Think of F as delta.
    F = self->P * (self->iError - self->LastError) +
        self->I * self->iError +
        self->D * (self->iError - 2 * self->LastError + self->PrevError);

    if (absError >= LasttContrlPoint && absError <= FirstContrlPoint)
    {
        F *= FirstContrlScale;
    }
    else
    {
        F = 0;
        self->iError = 0;
    }

    // Calculate total output
    self->PrevError = self->LastError; // 更新前次误差
    self->LastError = self->iError;    // 更新上次误差

    F = self->F + F;
    // Restrict to max/min
    if (F >= self->Fmax)
        F = self->Fmax;
    else if (F <= self->Fmin)
        F = self->Fmin;

    //update self->F at the end in order to avoid the impact of intermediate results on system running.
    self->F = F;
}

/**
* @brief :        incremental PID
* @description:1. When Target minus SamplingValue equals is negitive,the normalized 
            system parameter, F , will increase, and if Target minus SamplingValue 
            equals is positive, then F will decrease.
            2. And the P,I,D parameter will multiply by decayfun(iError).
            3. --> Target > iSampling -> F ↑, Target < iSampling -> F ↓
* @eturn:     None      
* @note please sure the deacyfun is not NULL.    
*/
void IncPIDCalcDelta_Normal_Decay(pPID self,PIDDecayFun deacyfun)
{

    float F = 0;

    //当前误差
    self->iError = self->Target - self->iSampling;
    /*
    Please don't use auto formate to format this file, 
    because the pid calculate code is too long and it will
    be inconvenient to view if you use auto formate.
    */

    //Here , Think of F as delta.
    F = self->P * (self->iError - self->LastError) +
        self->I * self->iError +
        self->D * (self->iError - 2 * self->LastError + self->PrevError);

    F *= deacyfun(self->iError);

    // Calculate total output
    self->PrevError = self->LastError; // 更新前次误差
    self->LastError = self->iError;    // 更新上次误差

    F = self->F + F;
    // Restrict to max/min
    if (F >= self->Fmax)
        F = self->Fmax;
    else if (F <= self->Fmin)
        F = self->Fmin;

    //update self->F at the end in order to avoid the impact of intermediate results on system running.
    self->F = F;
}

/**
* @brief :        incremental PID
* @description:1. When Target minus SamplingValue equals is negitive,the normalized 
            system parameter, F , will increase, and if Target minus SamplingValue 
            equals is positive, then F will decrease.
            2. And the P,I,D parameter will multiply by decayfun(iError).
            3. --> Target > iSampling -> F ↑, Target < iSampling -> F ↓
            4. self->iError normalized.
* @return:     None 
* @note please sure the deacyfun is not NULL.
*/
void IncPIDCalcDelta_NormalSampleAndF_Decay(pPID self,PIDDecayFun deacyfun)
{
    float F = 0;
    
    //当前误差
    self->iError = self->Target - self->iSampling;
    /*
    Please don't use auto formate to format this file, 
    because the pid calculate code is too long and it will
    be inconvenient to view if you use auto formate.
    */

    //Here , Think of F as delta.
    F = self->P * (self->iError - self->LastError) +
        self->I *  self->iError +
        self->D * (self->iError - 2 * self->LastError + self->PrevError);

    F *= deacyfun(self->iError / self->sysArg);

    // Calculate total output
    self->PrevError = self->LastError; // 更新前次误差
    self->LastError = self->iError;    // 更新上次误差

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
y = 1 - 4*e^(-ax)/(1+b*e^(-ax))^2
*/
float dsigmoidn(float z, float a)
{
    float e_ = exp(-a * z);
    float q_ = 1.0 + e_;
    return 1 - 4 * e_ / q_ * q_;
}

/*
a > 0 and b > 0
y = 1/(1+b*e^(-a|x|))
Be careful of spills and normalize errors if you use them.
*/
float sigmoidabsx(float z, float a, float b)
{
    z = fabsf(z);
    return 1.0f / (b * exp(-a * z));
}

/*
a > 0
y = tanh(a|z|)
Be careful of spills and normalize errors if you use them.
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