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

    self->iErrK = 0;   // 当前误差 e(k)
    self->iErrKm1 = 0; // 前1拍误差 e(k-1)
    self->iErrKm2 = 0; // 前2拍误差 e(k-2)

    self->iF = 0;    // 控制器输出值
    self->iFmax = 0; // 控制器输出最大值
    self->iFmin = 0; // 控制器输出最小值

    // delta 限制初始化为int32_t最大/最小值
    self->maxDelta = INT32_MAX; // PID增量限制
    self->minDelta = INT32_MIN; // PID减量限制
}

void Inc_PID_Q32_Update(pInc_PID_Q32_t self)
{
    // Calculate current error
    self->iErrK = self->iTarget - self->iSampling;
    // P/I/D 是 Q32 定点数，32 位乘法在大误差下会溢出，必须用 64 位中间量
    int64_t F = 0;
    int64_t delta = (int64_t)self->P * (self->iErrK - self->iErrKm1) +
                    (int64_t)self->I * self->iErrK +
                    (int64_t)self->D * ((int64_t)self->iErrK - 2 * (int64_t)self->iErrKm1 + self->iErrKm2);

    // Calculate total output
    // 移位: e(k-1) -> e(k-2)
    self->iErrKm2 = self->iErrKm1;
    // 移位: e(k) -> e(k-1)
    self->iErrKm1 = self->iErrK;

    if (delta > self->maxDelta)
    {
        delta = self->maxDelta;
    }
    else if (delta < self->minDelta)
    {
        delta = self->minDelta;
    }

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

/**
 * @brief Update the PID controller with the current target and sampling values,
 *        and subtract the delta from the output.
 *
 * @param self Pointer to the PID controller instance.
 * @return int Returns 1 if output was limited to max, -1 if limited to min, 0 otherwise.
 */
int Inc_PID_Q32_Update_SubDelta(pInc_PID_Q32_t self)
{

    int res = 0;
    int64_t F = 0;
    // Calculate current error
    self->iErrK = self->iTarget - self->iSampling;
    // P/I/D 是 Q32 定点数，32 位乘法在大误差下会溢出，必须用 64 位中间量
    int64_t delta = (int64_t)self->P * (self->iErrK - self->iErrKm1) +
                    (int64_t)self->I * self->iErrK +
                    (int64_t)self->D * ((int64_t)self->iErrK - 2 * (int64_t)self->iErrKm1 + self->iErrKm2);

    // Calculate total output
    // 移位: e(k-1) -> e(k-2)
    self->iErrKm2 = self->iErrKm1;
    // 移位: e(k) -> e(k-1)
    self->iErrKm1 = self->iErrK;

    if (delta > self->maxDelta)
    {
        delta = self->maxDelta;
    }
    else if (delta < self->minDelta)
    {
        delta = self->minDelta;
    }

    F = self->iF - delta;

    // Restrict to max/min
    if (F >= self->iFmax)
    {
        self->iF = self->iFmax;
        res = 1; // Indicate that the output was limited to max
    }
    else if (F <= self->iFmin)
    {
        self->iF = self->iFmin;
        res = -1; // Indicate that the output was limited to min
    }
    else
    {
        self->iF = F;
    }
    return res; // Return the result of the update
}

void Inc_PID_Q32_Set_DeltaLimit(pInc_PID_Q32_t self, int32_t maxDelta, int32_t minDelta)
{
    self->maxDelta = maxDelta;
    self->minDelta = minDelta;
}

/**
 * @brief Reset the incremental PID controller states to zero
 *
 * @param self Pointer to the incremental PID controller structure
 * @return int Reset status (implementation defined)
 *
 * @details
 * This function resets the error states of the PID controller:
 * - Current error (iErrK)
 * - Previous error (iErrKm2)
 * - Last error (iErrKm1)
 * All values are set to 0, effectively clearing the controller's memory.
 */
int Inc_PID_Q32_Reset(pInc_PID_Q32_t self)
{
    self->iErrK = 0;
    self->iErrKm2 = 0;
    self->iErrKm1 = 0;

    return 0;
}