#include <stdio.h>
#include "PID.h"
#include <windows.h>
#include <math.h>
PID APID;
int CCR = 200;
int Seq = 0;
float piddecayfun(float z){
    //return sigmoidabsx(z,11,5);
    //return obliquestepfun(z, 0.0005);
    return px1(z, 200);
}

void UserPIDInit(){
    IncPIDInit(&APID); //This code is not necessary.
    APID.Fmax = 0.9;
    APID.Fmin = 0.1;
    APID.F = 0.5;
    APID.sysArg = 200000; // eg. In DC/DC buck control system , this value is PWM output timer's period.
    APID.P = 0.001;
    APID.I = 0.02;
    APID.D = 0;
    APID.Target = 100;
}

void loop1(){
    
    IncPIDCalcDelta_NormalizedF_Decay(&APID,piddecayfun); //Target - Sample > 0 , F↑.
    //when you using this pid library "PIDUpdateValue_P", you need to sure System control parameters are positively 
    //correlated with system output, otherwish you should use "PIDUpdateValue_N"
    CCR = PIDUpdateValue_P(&APID); //↑
    APID.iSampling = log(CCR)*10; //↑
    printf("APID.F = %f\r\n",APID.F);
    printf("CCR = %d\r\n",CCR);
    printf("iSampling = %f\r\n",APID.iSampling);
    printf("Seq = %d\r\n",Seq++);
    Sleep(10);
}


float piddecayfun2(float z){
    return sigmoidabsx(z,11,5);
    //return obliquestepfun(z, 0.005);
    //return px1(z, 50);
}

void loop2(){
    
    IncPIDCalcDelta_NormalizedFAndDecayFunInput_Decay(&APID,piddecayfun2); //Target - Sample > 0 , F↑. IncPIDCalcDelta_NormalizedF_Decay收敛的时候有个微小的震荡难以收敛。
    //when you using this pid library "PIDUpdateValue_P", you need to sure System control parameters are positively 
    //correlated with system output, otherwish you should use "PIDUpdateValue_N"
    CCR = PIDUpdateValue_P(&APID); //↑
    APID.iSampling = log(CCR)*10; //↑
    printf("APID.F = %f\r\n",APID.F);
    printf("CCR = %d\r\n",CCR);
    printf("iSampling = %f\r\n",APID.iSampling);
    printf("Seq = %d\r\n",Seq++);
    Sleep(10);
}


void loop3(){
    
    IncPIDCalc_NormalizedF(&APID); 
    CCR = PIDUpdateValue_P(&APID); //↑,assume CCR is stm32 timer pwm duty control register. 
    APID.iSampling = log(CCR)*10; //↑
    printf("APID.F = %f\r\n",APID.F);
    printf("CCR = %d\r\n",CCR);
    printf("iSampling = %f\r\n",APID.iSampling);
    printf("Seq = %d\r\n",Seq++);
    Sleep(10);
}

int main(){

    UserPIDInit();
    while (1)
    {
        loop1();
    }
    

    return 0;
}
