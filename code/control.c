#include "zf_common_headfile.h"
#include"control.h"

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define START_TIME 30000000//0.3s
#define MAX_DELTA 200

//#define START_SPEED_ADD 1

void stop (void)
{
    pwm_set_duty(ATOM0_CH4_P02_4, 0);
    pwm_set_duty(ATOM0_CH1_P33_9, 750);
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
    float kp = 1, ki = 0.2, kd = 0.6;
    if (abs(err) < 100)
    {          // 误差小时才积分
        I += err;
        I = min(I, 40);
        I = max(I, -40);
    }
    P = err;
    //I += err;
    D = (err - last_err) * DI_TONG_ZHI_HOU + (last_err - last_last_err) * (1 - DI_TONG_ZHI_HOU);
    P = min(P, 80);
    P = max(P, -80);
    I = min(I, 40);
    I = max(I, -40);
    D = min(D, 50);
    D = max(D, -50);
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
        uint32 now_max_speed = (uint32) min(speed, (uint64)(tick*(uint64_t)speed/START_TIME));          //速度递增
        pwm_set_duty(ATOM0_CH4_P02_4, now_max_speed);
    }
    else
    {
        pwm_set_duty(ATOM0_CH4_P02_4, speed);
    }
    pwm_set_duty(ATOM0_CH1_P33_9, min(max(750 + delta,400), 1100));   // 一个方向
}   // PID参数设置，提高P值能够使反应更快，但可能会过冲，设置i值，能够让整体轨迹更接近目标值，d值能够减小过冲（振荡）。
//调节顺序 / I 调节 / D 只在突变时起作用，三段原文
