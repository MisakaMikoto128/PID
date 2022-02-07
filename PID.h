#ifndef __PID_H
#define __PID_H
typedef float (*PIDDecayFun)(float);
#define PID_DECAY_FUNC_NULL ((PIDDecayFun)0)
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

    PIDDecayFun pidDecayByAbsErrorFunc; //PID参数随误差减小而减小的衰减函数，为NULL时表明不使用衰减。

} PID, *pPID;

void IncPIDInit(pPID self);

/**
* @brief     Position type PID. Have no decay function.
* @return    None 
*/
void PosPIDCalc_NormalizedF(pPID self);

/**
* @brief    incremental PID. Have no decay function.
*           when Target minus SamplingValue equals is negitive,the Normalizedized 
            system parameter, F , will increase, and if Target minus SamplingValue 
            equals is positive, then F will decrease.
            --> Target > iSampling -> F ↑, Target < iSampling -> F ↓
* @return    None          
*/
void IncPIDCalc_NormalizedF(pPID self);

/**
* @brief    incremental PID. Have no decay function.
*           1. When Target minus SamplingValue equals is negitive,the Normalizedized 
*           system parameter, F , will increase, and if Target minus SamplingValue 
*           equals is positive, then F will decrease.
*           2. And the P,I,D parameter will multiply by FirstContrlScale when  
*           LasttContrlPoint < absError < FirstContrlPoint.
*           3. --> Target > iSampling -> F ↑, Target < iSampling -> F ↓
* @return   None          
*/
void IncPIDCalcDelta_NormalizedF_TwoStage(pPID self, float FirstContrlPoint, float LasttContrlPoint, float FirstContrlScale);

/**
* @brief    incremental PID. F is normalized modulation degree. PID parameter decay 
*           function input is error.
*           1. When Target minus SamplingValue equals is negitive,the Normalizedized 
*           system parameter, F , will increase, and if Target minus SamplingValue 
*           equals is positive, then F will decrease.
*           2. And the P,I,D parameter will multiply by decayfun(iError).
*           3. --> Target > iSampling -> F ↑, Target < iSampling -> F ↓
* @return   None      
* @note please sure the deacyfun is not NULL.    
*/
void IncPIDCalcDelta_NormalizedF_Decay(pPID self,PIDDecayFun deacyfun);

/**
* @brief    incremental PID. F is normalized modulation degree. PID parameter decay 
*           function input is normalized error.deacyfun(self->iError / self->sysArg).
*           Not recommend to use this function.Beacuse 
*           the self->sysArg is system's modulation degree max, not the system's output max.
*           1. When Target minus SamplingValue equals is negitive,the Normalizedized 
*           system parameter, F , will increase, and if Target minus SamplingValue 
*           equals is positive, then F will decrease.
*           2. And the P,I,D parameter will multiply by decayfun(iError).
*           3. --> Target > iSampling -> F ↑, Target < iSampling -> F ↓
*           4. self->iError Normalizedized.
* @return    None 
* @note please sure the deacyfun is not NULL.
*/
void IncPIDCalcDelta_NormalizedFAndDecayFunInput_Decay(pPID self,PIDDecayFun deacyfun);

/**
* @brief    incremental PID. F is normalized modulation degree. PID parameter decay 
*           function input is normalized error.The PID.pidDecayByAbsErrorFunc need to
*           be choose by yourself.If PID.pidDecayByAbsErrorFunc is NULL, then PID parameter
*           will not decay.Recommend to use this function.
*           1. When Target minus SamplingValue equals is negitive,the Normalizedized 
*           system parameter, F , will increase, and if Target minus SamplingValue 
*           equals is positive, then F will decrease.
*           2. And the P,I,D parameter will multiply by decayfun(iError).
*           3. --> Target > iSampling -> F ↑, Target < iSampling -> F ↓
*           4. self->iError Normalizedized.
* @return    None 
*/
void IncPIDCalcDeltaAutoDecay(pPID self);
#define PIDUpdateValue_P(self) ((self)->sysArg * (self)->F)
#define PIDUpdateValue_N(self) ((self)->sysArg * (1.0f - (self)->F))

#define PIDSetSampleValue(self,value) ((self)->iSampling = (value))

float dsigmoidn(float z,float a);
float sigmoidabsx(float z,float a,float b);
float tanhabsx(float z,float a);
float px1(float z,float a);
float obliquestepfun(float z,float x);

void UserPIDInit(void);
#endif
