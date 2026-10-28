#include "zf_common_headfile.h"
#include"control.h"

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define START_TIME 30000000//0.3s
#define MAX_DELTA 200

//#define START_SPEED_ADD 1
#define MAX_SPEED 1000//0-10000限幅
static int16 delta=0;

void control(uint32 tick,int16 delta){

    if(tick<START_TIME){
        //占空比逐渐增大
        uint32 now_max_speed=(uint32)min(MAX_SPEED,(uint64)(tick*(uint64_t)MAX_SPEED/START_TIME));//速度递增
        pwm_set_duty(ATOM0_CH4_P02_4, now_max_speed);
    }
    else{
        pwm_set_duty(ATOM0_CH4_P02_4, MAX_SPEED);
    }
    pwm_set_duty(ATOM0_CH1_P33_9, min(max(750 + delta,400),1100));   // 一个方向
}
// PID参数设置，提高P值能够使反应更快，但可能会过冲，设置i值，能够让整体轨迹更接近目标值，d值能够减小过冲（振荡）。
// 调节顺序先kp后ki再kd，这里自己试一下，这里我只使用纯P调节
// I调节是有作用的，但是不要太大
//(由于是误差是离散的，不连续的，这里的D，只在数值突变的一刻，D才起调节作用，具体有没有用，理解代码，自己试一下)

