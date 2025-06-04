/**
 * @file pid_q32.c
 * @author Liu Yuanlin (liuyuanlins@outlook.com)
 * @brief
 * @version 0.1
 * @date 2025-06-04
 * @last modified 2025-06-04
 *
 * @copyright Copyright (c) 2025 Liu Yuanlin Personal.
 *
 */
#include "pid_q32.h"

void Inc_PID_Q32_Init(pInc_PID_Q32_t self)
{
    self->iTarget = 0;   // 目标值
    self->iSampling = 0; // 采样值

    self->P = 0; // 比例
    self->I = 0; // 积分
    self->D = 0; // 微分

    self->iError = 0;     // 当前误差
    self->iPrevError = 0; // 前1次误差值
    self->iLastError = 0; // 前2次误差值

    self->iF = 0;    // 控制器输出值
    self->iFmax = 0; // 控制器输出最大值
    self->iFmin = 0; // 控制器输出最小值
}

void Inc_PID_Q32_Update(pInc_PID_Q32_t self)
{
    // Calculate current error
    self->iError = self->iTarget - self->iSampling;
    int32_t delta = 0;
    int32_t F = 0;
    delta = self->P * (self->iError - self->iLastError) +
            self->I * self->iError +
            self->D * (self->iError - self->iPrevError);

    // Calculate total output
    // Update the previous error
    self->iPrevError = self->iLastError;
    // Update last error
    self->iLastError = self->iError;

    F = self->iF + delta;

    // Restrict to max/min
    if (F >= self->iFmax)
    {
        self->iF = self->iFmax;
    }
    else if (F <= self->iFmin)
    {
        self->iF = self->iFmin;
    }
    else
    {
        self->iF = F;
    }
}