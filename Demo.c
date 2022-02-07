#include <stdio.h>
#include "PID.h"
#include <windows.h>
#include <math.h>
/*recommend useing as fellow:*/
PID APID;
int CCR = 200;
int Seq = 0;
float piddecayfun(float z)
{
    // return sigmoidabsx(z,11,5);
    // return obliquestepfun(z, 0.0005);
    return px1(z, 200);
}

void UserPIDInit()
{
    IncPIDInit(&APID); // This code is not necessary.
    APID.Fmax = 0.9;
    APID.Fmin = 0.1;
    APID.F = 0.5;
    APID.sysArg = 200000; // eg. In DC/DC buck control system , this value is PWM output timer's period.
    APID.P = 0.001;
    APID.I = 0.02;
    APID.D = 0;
    APID.Target = 100;
    APID.pidDecayByAbsErrorFunc = piddecayfun;
}

void loop()
{
    /*There are three steps to using this PID*/

    /*1. 获取采样值 get system sampling value. */
    PIDSetSampleValue(&APID, log(CCR) * 10); //  ↑

    /*2. 计算PID值 calc pid value.*/
    IncPIDCalcDeltaAutoDecay(&APID); //

    /*3. 更新PID值 update pid value to system.*/
    // eg.assume CCR is stm32 timer pwm duty control register.
    CCR = PIDUpdateValue_P(&APID); //   ↑
    //CCR = PIDUpdateValue_P(&APID); // ↓

    // show PID adjust result
    printf("APID.F = %f\r\n", APID.F);
    printf("CCR = %d\r\n", CCR);
    printf("iSampling = %f\r\n", APID.iSampling);
    printf("Seq = %d\r\n", Seq++);
    Sleep(10);
}

int main()
{

    UserPIDInit();
    while (1)
    {
        loop();
    }

    return 0;
}
