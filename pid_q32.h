/**
 * @file pid_q32.h
 * @author Liu Yuanlin (liuyuanlins@outlook.com)
 * @brief
 * @version 0.1
 * @date 2025-06-04
 * @last modified 2025-06-04
 *
 * @copyright Copyright (c) 2025 Liu Yuanlin Personal.
 *
 */
#ifndef PID_Q32_H
#define PID_Q32_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

    typedef struct
    {
        int32_t iTarget;   // 目标值
        int32_t iSampling; // 测量值
        int32_t P;         // 比例
        int32_t I;         // 积分
        int32_t D;         // 微分

        int32_t iError;     // 当前误差
        int32_t iPrevError; // 前1次误差值
        int32_t iLastError; // 前2次误差值

        int32_t iF;    // 传输给控制器的新控制值
        int32_t iFmax; // 传输给控制器的最大控制值
        int32_t iFmin; // 传输给控制器的最小控制值

        int32_t maxDelta; // PID增量限制
        int32_t minDelta; // PID减量限制
    } Inc_PID_Q32_t, *pInc_PID_Q32_t;

    void Inc_PID_Q32_Init(pInc_PID_Q32_t self);
    void Inc_PID_Q32_Update(pInc_PID_Q32_t self);
    int Inc_PID_Q32_Update_SubDelta(pInc_PID_Q32_t self);
    void Inc_PID_Q32_Set_DeltaLimit(pInc_PID_Q32_t self, int32_t maxDelta, int32_t minDelta);
    int Inc_PID_Q32_Reset(pInc_PID_Q32_t self);
#ifdef __cplusplus
}
#endif
#endif //! PID_Q32_H
