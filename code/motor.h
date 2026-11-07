/*********************************************************************************************************************
 * 文件名称          motor.h
 * 作用              双电机驱动层：左右轮统一出口，附带编码器测速与里程
 * 编码              GBK（与工程其余源码一致，勿另存为 UTF-8）
 ********************************************************************************************************************/
#ifndef MOTOR_H
#define MOTOR_H

#include "zf_common_headfile.h"
#include "tiaocan.h"

#define MOTOR_ZUO 0                     // 数组下标：左轮
#define MOTOR_YOU 1                     // 数组下标：右轮

/* 初始化：方向脚电平 -> 两路编码器 -> 两路 PWM -> 清零并记录初值
 * 只调用一次，替代原来散在 cpu0_main.c 里的 encoder_quad_init / gpio_init / pwm_init 四行 */
void  motor_init        (void);

/* 双轮占空比输出，唯一出口。0 ~ MOTOR_PWM_MAX，负值按 0 处理（当前硬件只前进） */
void  drive             (int16 zuo, int16 you);
void  drive_stop        (void);

/* 测速与里程：motor_speed_update() 建议每帧调用一次，内部按 MOTOR_CE_SU_WIN 限频（默认 10 ms） */
void  motor_speed_update(void);
int16 motor_speed_pulse (uint8 ce);     // 脉冲/秒，正 = 前进
int16 motor_speed_mm_s  (uint8 ce);     // mm/s，依赖 TURN_MEI_LUN_QUAN_CHONG_MEI_DU 与轮周长
int32 motor_li_cheng    (uint8 ce);     // 累计脉冲，正 = 前进
void  motor_li_cheng_qing_ling (void);

#endif /* MOTOR_H */
