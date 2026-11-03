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
volatile int16 err=0;
int16 P, I, D,  last_err, last_last_err;
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
    {          // �����ʱ������
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
        //ռ�ձ�������
        uint32 now_max_speed = (uint32) min(speed, (uint64)(tick*(uint64_t)speed/START_TIME));          //�ٶȵ���
        pwm_set_duty(ATOM0_CH4_P02_4, now_max_speed);
    }
    else
    {
        pwm_set_duty(ATOM0_CH4_P02_4, speed);
    }
    pwm_set_duty(ATOM0_CH1_P33_9, min(max(750 + delta,400), 1100));   // һ������
}
// PID�������ã����Pֵ�ܹ�ʹ��Ӧ���죬�����ܻ���壬����iֵ���ܹ�������켣���ӽ�Ŀ��ֵ��dֵ�ܹ���С���壨�񵴣���
// ����˳����kp��ki��kd�������Լ���һ�£�������ֻʹ�ô�P����
// I�����������õģ����ǲ�Ҫ̫��
//(�������������ɢ�ģ��������ģ������D��ֻ����ֵͻ���һ�̣�D����������ã�������û���ã�������룬�Լ���һ��)

