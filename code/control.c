#include "zf_common_headfile.h"
#include"control.h"

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

//#define START_SPEED_ADD 1

void stop (void)
{
    pwm_set_duty(ATOM0_CH4_P02_4, 0);
    pwm_set_duty(ATOM0_CH1_P33_9, SERVO_CENTER);
    pid_reset();

}
volatile int16 err = 0;
int16 P, I, D, last_err, last_last_err;
void pid_reset (void)
{
    I = 0;
    last_err = 0;
    last_last_err = 0;
}
int16 pid (void)
{
    int16 output;
    float kp = PID_KP, ki = PID_KI, kd = PID_KD;
    if (abs(err) < PID_INTEGRAL_GATE)
    {          // 误差小时才积分
        I += err;
        I = min(I, PID_I_LIMIT);
        I = max(I, -PID_I_LIMIT);
    }
    P = err;
    //I += err;
    D = (err - last_err) * DI_TONG_ZHI_HOU + (last_err - last_last_err) * (1 - DI_TONG_ZHI_HOU);
    P = min(P, PID_P_LIMIT);
    P = max(P, -PID_P_LIMIT);
    I = min(I, PID_I_LIMIT);
    I = max(I, -PID_I_LIMIT);
    D = min(D, PID_D_LIMIT);
    D = max(D, -PID_D_LIMIT);
    output = kp * P + ki * I + kd * D;
    last_last_err = last_err;
    last_err = err;

    return output;
}
void control (uint32 tick, int16 delta, int speed)
{

    if (tick < START_TIME)
    {
        //占空比逐渐增大
        uint32 now_max_speed = (speed > 0)      // 负 speed 参与 uint64 运算会变成极大占空比，先挡掉
                ? (uint32) min(speed, (uint64)((uint64)tick * (uint64)speed / START_TIME))
                : 0;                            //速度递增
        pwm_set_duty(ATOM0_CH4_P02_4, now_max_speed);
    }
    else
    {
        pwm_set_duty(ATOM0_CH4_P02_4, speed);
    }
    pwm_set_duty(ATOM0_CH1_P33_9, min(max(SERVO_CENTER + delta, SERVO_MIN), SERVO_MAX));   // 一个方向
}   // PID参数设置，提高P值能够使反应更快，但可能会过冲，设置i值，能够让整体轨迹更接近目标值，d值能够减小过冲（振荡）。
//调节顺序 / I 调节 / D 只在突变时起作用，三段原文
