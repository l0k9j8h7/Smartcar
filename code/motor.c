/*********************************************************************************************************************
 * 文件名称          motor.c
 * 作用              双电机驱动层：左右轮统一出口，附带编码器测速与里程
 * 编码              GBK（与工程其余源码一致，勿另存为 UTF-8）
 * 说明              本文件是唯一允许直接写电机 PWM 的地方；
 *                    control() / stop() / turn() 一律经 drive() 输出，避免左右轮各写各的
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "motor.h"

static const pwm_channel_enum   motor_pwm[2]     = { TURN_PWM_ZUO, TURN_PWM_YOU };
static const encoder_index_enum motor_encoder[2] = { TURN_ENCODER_ZUO, TURN_ENCODER_YOU };

static int16  motor_shang_ci[2];        // 上次采样的编码器原始计数，用于求增量
static int32  motor_li_cheng_chong[2];  // 累计里程（脉冲），可正可负
static int16  motor_su_du_pulse[2];     // 脉冲/秒
static int16  motor_su_du_mm_s[2];      // mm/s
static uint32 motor_shang_ci_tick;      // 上次测速时刻，system_getval 单位 10 ns

static int32 motor_xian_fu (int32 v)
{
    if (v < 0) return 0;                        // 当前硬件只前进，负值按停车
    if (v > MOTOR_PWM_MAX) return MOTOR_PWM_MAX;
    return v;
}

void motor_init (void)
{
    /* 方向脚先定电平再开 PWM：pwm_init 之前驱动板 IN2 悬空，轮子可能抽一下 */
    gpio_init(TURN_DIR_ZUO, GPO, MOTOR_DIR_ZUO_LEVEL, GPO_PUSH_PULL);
    gpio_init(TURN_DIR_YOU, GPO, MOTOR_DIR_YOU_LEVEL, GPO_PUSH_PULL);

    /* 编码器：左 TIM4(P02_8/P00_9)、右 TIM2(P33_7/P33_6)，2026-11-08 改线
     * 原来左轮挂 TIM5 是错的：T5 属 GPT2，T5CON.T5M 只实现 0~3（定时/计数/门控），
     * 增量接口模式（6/7）是 GPT1 的 T2/T3/T4 专属。encoder_quad_init 第一行
     * zf_assert(encoder_n <= TIM4_ENCODER) 就会命中，而断言默认开启、命中后 while(TRUE) 死等，
     * 现象是上电卡在 motor_init() 里、轮子完全不动且无任何串口输出。 */
    encoder_quad_init(TURN_ENCODER_ZUO, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
    encoder_quad_init(TURN_ENCODER_YOU, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);

    pwm_init(TURN_PWM_ZUO, MOTOR_PWM_PINLV, 0);     // 左电机
    pwm_init(TURN_PWM_YOU, MOTOR_PWM_PINLV, 0);     // 右电机，必须与左同频
    pwm_set_duty(TURN_PWM_ZUO, 0);
    pwm_set_duty(TURN_PWM_YOU, 0);

    motor_shang_ci[0] = encoder_get_count(TURN_ENCODER_ZUO);
    motor_shang_ci[1] = encoder_get_count(TURN_ENCODER_YOU);
    motor_li_cheng_chong[0] = 0;
    motor_li_cheng_chong[1] = 0;
    motor_su_du_pulse[0] = 0;
    motor_su_du_pulse[1] = 0;
    motor_su_du_mm_s[0] = 0;
    motor_su_du_mm_s[1] = 0;
    motor_shang_ci_tick = system_getval();
}

void drive (int16 zuo, int16 you)
{
    pwm_set_duty(motor_pwm[MOTOR_ZUO], (uint32)motor_xian_fu(zuo));
    pwm_set_duty(motor_pwm[MOTOR_YOU], (uint32)motor_xian_fu(you));
}

void drive_stop (void)
{
    drive(0, 0);
}

void motor_speed_update (void)
{
    uint32 now = system_getval();
    uint32 dt  = now - motor_shang_ci_tick;
    uint8  i;

    if (dt < MOTOR_CE_SU_WIN) return;           // 未到测速窗口，保留上次结果
    if (dt == 0) return;                        // 防除零

    for (i = 0; i < 2; i++)
    {
        int16 xian_zai = encoder_get_count(motor_encoder[i]);
        int16 zeng     = (int16)(xian_zai - motor_shang_ci[i]);    // int16 差值天然处理计数器回绕
        int64 su_du;

        motor_shang_ci[i]  = xian_zai;
        motor_li_cheng_chong[i] += zeng;

        /* 脉冲/秒 = 增量 / (dt × 10 ns)；用 int64 防 1e8 × 增量溢出 int32 */
        su_du = (int64)zeng * 100000000LL / (int64)dt;
        if (su_du >  32767LL) su_du =  32767LL;
        if (su_du < -32767LL) su_du = -32767LL;
        motor_su_du_pulse[i] = (int16)su_du;

        if (TURN_MEI_LUN_QUAN_CHONG_MEI_DU > 0)
        {
            int32 mm = (int32)((float)motor_su_du_pulse[i]
                             * (float)TURN_LUN_ZHOU_MM
                             / (float)TURN_MEI_LUN_QUAN_CHONG_MEI_DU);
            if (mm >  32767) mm =  32767;
            if (mm < -32767) mm = -32767;
            motor_su_du_mm_s[i] = (int16)mm;
        }
        else
        {
            motor_su_du_mm_s[i] = 0;            // 每圈脉冲没标定，不给假数据
        }
    }
    motor_shang_ci_tick = now;
}

int16 motor_speed_pulse (uint8 ce)
{
    return motor_su_du_pulse[ce ? MOTOR_YOU : MOTOR_ZUO];
}

int16 motor_speed_mm_s (uint8 ce)
{
    return motor_su_du_mm_s[ce ? MOTOR_YOU : MOTOR_ZUO];
}

int32 motor_li_cheng (uint8 ce)
{
    return motor_li_cheng_chong[ce ? MOTOR_YOU : MOTOR_ZUO];
}

void motor_li_cheng_qing_ling (void)
{
    motor_li_cheng_chong[0] = 0;
    motor_li_cheng_chong[1] = 0;
}
