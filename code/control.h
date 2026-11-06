#include "tiaocan.h"                // 可调参数集中到 tiaocan.h，调车只改那一个文件

void control(uint32 tick,int16 delta,int speed);

void pid_reset (void);
int16 pid(void);
void stop(void);

/* ==================== 编码器差速转弯（2026-11-06） ==================== */
typedef enum
{
    TURN_WAN_CHENG = 0,             // 正常走完全程
    TURN_CAN_SHU_CUO,               // 参数非法：每圈脉冲没标定
    TURN_CHAO_SHI,                  // 总时长超限，已强制停车
    TURN_DU_ZHUAN,                  // 编码器窗口内几乎不动：堵转 / 电机不转 / 编码器线松
} turn_jie_guo_enum;

int16 turn (int16 degrees);         // degrees: -180 ~ 180，正 = 右转，负 = 左转；阻塞执行

extern volatile int32 turn_lei_ji_chong[2]; // [0]左轮 [1]右轮，本次累计脉冲
extern volatile int16 turn_shi_ji_jiao_du;  // 按两轮平均里程反算的实际转角
extern volatile int16 turn_shi_ji_ban_jing; // 由实测左右轮速比反推的半径（mm）
extern volatile uint8 turn_jie_guo;         // 结果码，见 turn_jie_guo_enum
