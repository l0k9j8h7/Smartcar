#include "tiaocan.h"                // 可调参数集中到 tiaocan.h，调车只改那一个文件

void control(uint32 tick,int16 delta,int speed);

void pid_reset (void);
int16 pid(uint32 dt_zhen);      // dt_zhen = 本帧与上一帧的间隔，单位 10 ns（20 ms = 2,000,000）
                                // 丢帧时用它折算 D、跳过积分，见 control.c 注释
void stop(void);

/* ==================== 编码器差速转弯（2026-11-06） ==================== */
typedef enum
{
    TURN_WAN_CHENG = 0,             // 正常走完全程
    TURN_CAN_SHU_CUO,               // 参数非法：每圈脉冲没标定
    TURN_CHAO_SHI,                  // 总时长超限，已强制停车
    TURN_DU_ZHUAN,                  // 编码器窗口内几乎不动：堵转 / 电机不转 / 编码器线松
} turn_jie_guo_enum;

int16 turn (int16 degrees);         // degrees: ±TURN_ZUI_DA_JIAO_DU（默认 ±360），正 = 右转，负 = 左转；阻塞执行
                                    // 常用值：90 直角弯、120/135 大弯、180 掉头、300 绕行近一圈
                                    // 越界会压到边界值执行；< TURN_ZUI_XIAO_JIAO_DU 直接返回 0 不动车

extern volatile int32 turn_lei_ji_chong[2]; // [0]左轮 [1]右轮，本次累计脉冲
extern volatile int16 turn_shi_ji_jiao_du;  // 按两轮平均里程反算的实际转角
extern volatile int16 turn_shi_ji_ban_jing; // 由实测左右轮速比反推的半径（mm）
extern volatile uint8 turn_jie_guo;         // 结果码，见 turn_jie_guo_enum
