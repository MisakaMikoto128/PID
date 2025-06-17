#include <stdio.h>
#include "pid_q32.h"
#include <windows.h>
#include <math.h>
Inc_PID_Q32_t pid;

void User_PID_Init()
{
    // Init all fields as zero.
    Inc_PID_Q32_Init(&pid);
    pid.iFmax = 2000 << 12; // 放大
    pid.iFmin = 0;
    pid.iF = 0;
    pid.P = 108*7;
    pid.I = 78*17;
    pid.D = 0;
    pid.maxDelta = 100 << 12; // 放大
}

struct Simulated_Device_t
{
    uint32_t adc_sample_value; // 模拟设备的采样值
    float output_value;        // 模拟设备的输出值
    uint16_t ctrl_reg;         // 模拟设备的控制寄存器
    void (*update)(struct Simulated_Device_t *self);
};

void simulated_device_func(struct Simulated_Device_t *self)
{
    // 模拟设备输出值为正弦波
    self->output_value = sinf(self->ctrl_reg / 2000.0f) * 100.0f;
    // 模拟设备的ADC采样值
    self->adc_sample_value = self->output_value / 40.0f * 4096.0f / 3.3f;
}

struct Simulated_Device_t device = {
    .adc_sample_value = 0,
    .output_value = 0.0f,
    .ctrl_reg = 0,
    .update = simulated_device_func};

void loop_simulated_device()
{
    device.update(&device); // 更新模拟设备状态
    Sleep(1);               // 模拟延时
}

void loop_pid()
{
    // 直接用ADC寄存器值作为采样值，对应目标也为ADC寄存器值的
    pid.iSampling = device.adc_sample_value;
    Inc_PID_Q32_Update(&pid);
    // 更新后的控制器值应用到控制器上
    device.ctrl_reg = (uint16_t)(pid.iF >> 12); // 将控制器值缩小
    // 检查第15位是否为1（因为右移16位后，原来的第15位决定小数是否>0.5）
    int is_greater_than_half = (pid.iF & 0x8000) != 0;
    static int count = 0;
    if(count++ & 0x01)
    {
        device.ctrl_reg += is_greater_than_half;
    }

    Sleep(10);
}

// 设置目标值
float target_volt = 62.1867f; // 目标电压

void show_pid_status()
{
    printf("T:%d,S:%6d,P:%6d,I:%4d,D:%4d,F:%6d,Error:%7d,Prev:%6d,Last:%6d",
           pid.iTarget, pid.iSampling, pid.P, pid.I, pid.D, pid.iF, pid.iError, pid.iPrevError, pid.iLastError);
    printf(",Out:%.5f,ADC:%6d,Ctrl:%6d,err:%.5f\n",
           device.output_value, device.adc_sample_value, device.ctrl_reg, device.output_value - target_volt);
    fflush(stdout); // 刷新输出缓冲区
}

int main()
{
    User_PID_Init();

    // 将目标电压转换为对应的ADC值
    // // 假设ADC的分辨率为12位，参考电压为3.3V
    // uint16_t adc_target_value = (uint16_t)(target_volt / 40.0f / 3.3f * 4096.0f);
    // // 计算得到的ADC值作为目标值，缺点是精度损失
    // pid.iTarget = (int32_t)adc_target_value;

    // 假设ADC的分辨率为12位，参考电压为3.3V
    // 计算得到的ADC值作为目标值，放大弥补计算时候的精度损失
    pid.iTarget = (int32_t)(target_volt / 40.0f / 3.3f * 4096.0f);
    while (1)
    {
        show_pid_status();
        loop_pid();
        loop_simulated_device();
    }

    return 0;
}
