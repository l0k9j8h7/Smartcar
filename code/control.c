#include "zf_common_headfile.h"
#include"control.h"
#include"motor.h"

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

//#define START_SPEED_ADD 1

void stop (void)
{
    drive_stop();                                   // 左右轮一起断，别只关左轮
    pwm_set_duty(ATOM0_CH1_P33_9, SERVO_CENTER);
    pid_reset();
}
volatile int16 err = 0;
int16 P, I, D, last_err, last_last_err;
/* 转弯后的舵机冷却帧（251e810 报告 §4.2）。turn() 收尾会把舵机回中，
 * 但紧随其后的一帧 control() 会拿「转弯前留下的 err」立刻反打舵 -- 刚转过来的车头
 * 又被掰回去，表现为弯道转不干净、出弯画龙。冷却期内 control() 只回中直行。 */
volatile uint8 turn_leng_que_zhen = 0;
void pid_reset (void)
{
    I = 0;
    last_err = 0;
    last_last_err = 0;
}
int16 pid (uint32 dt_zhen)
{
    int16 output;
    float kp = PID_KP, ki = PID_KI, kd = PID_KD;
    float d_raw;
    float dt_bi = 1.0f;
    uint8 dan_zhen;

    /* 丢帧折算（复核报告 §3.4 / §4.2 / §8 第 5 条）
     * 摄像头 MT9V03X_FPS_DEF = 50 → 固定 20 ms 送一帧，主循环是「等帧 → 处理 → 再等帧」。
     * 处理一旦超过 20 ms 就丢帧，控制周期只会离散成 20 / 40 / 60 ms，不是 1/处理时间 那种连续帧率。
     * 原实现既没有 dt，又让 I 按帧累加、D 按帧差分：丢了 1 帧，D 拿到的是「跨两帧的误差差」，
     * 数值相当于翻倍 -- 这就是「丢一帧后 D 项出尖峰」的根，不是 PID 参数没整定好。
     * 现在：丢帧的这帧不积分，D 按「标称周期 / 实际周期」折算；正常 20 ms 帧的行为与改动前完全一致。
     * dt_zhen 单位 10 ns（20 ms = 2,000,000），超过 1.5 倍标称即判丢帧。 */
    dan_zhen = (dt_zhen > (uint32)(PID_DT_NOMINAL_10NS + PID_DT_NOMINAL_10NS / 2)) ? 1 : 0;
    if (dan_zhen && dt_zhen < PID_DT_MAX_10NS)          // 断流那种秒级间隔不折算，按 1 帧处理
        dt_bi = (float)PID_DT_NOMINAL_10NS / (float)dt_zhen;

    if (!dan_zhen && abs(err) < PID_INTEGRAL_GATE)
    {          // 误差小时才积分；丢帧时不积分，否则按帧累加等于把积分增益放大
        I += err;
        I = min(I, PID_I_LIMIT);
        I = max(I, -PID_I_LIMIT);
    }
    P = err;
    //I += err;
    d_raw = ((err - last_err) * DI_TONG_ZHI_HOU + (last_err - last_last_err) * (1 - DI_TONG_ZHI_HOU)) * dt_bi;
    D = (int16)d_raw;
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
/* ===================================================================================================================
 * 速度闭环（2026-11-10，按审查 §9 第 6 条新增）
 * 前馈：巡线/丢线记忆算出来的目标占空比
 * 反馈：motor_speed_pulse()，每 10 ms 由 motor_speed_update() 刷新一次
 * 输出：叠加 PI 修正后的占空比，仍经 drive() 唯一出口
 * 关掉的条件有两个：SPEED_LOOP_ENABLE=0，或 SPEED_LOOP_PULSE_PER_DUTY 还没标定（=0）。
 * 后者是有意的 -- 宏开着但没标定就退化成开环，不会因为忘记标定把车跑飞。
 * =================================================================================================================== */
#if SPEED_LOOP_ENABLE && (SPEED_LOOP_PULSE_PER_DUTY > 0)
#define SPEED_LOOP_OK 1
#else
#define SPEED_LOOP_OK 0
#endif

#if SPEED_LOOP_OK
static int32 sp_ji_fen[2] = {0, 0};     // [0] 左轮 [1] 右轮，单位：占空比

static int32 su_du_huan (uint8 ce, int32 mu_biao_duty)
{
    int32 mu_biao_pulse, wu_cha, xiu;

    if (mu_biao_duty < SPEED_LOOP_ZUI_DI_DUTY)      // 起步/停车段：轮速反馈不可信，直接开环
    {
        sp_ji_fen[ce] = 0;
        return mu_biao_duty;
    }

    mu_biao_pulse = mu_biao_duty * SPEED_LOOP_PULSE_PER_DUTY / 1000;   // 目标轮速（脉冲/秒）
    wu_cha = mu_biao_pulse - (int32)motor_speed_pulse(ce);             // 正 = 实际慢了，该加油

    sp_ji_fen[ce] += wu_cha * SPEED_LOOP_KI / 10000;
    if (sp_ji_fen[ce] >  SPEED_LOOP_I_LIMIT) sp_ji_fen[ce] =  SPEED_LOOP_I_LIMIT;
    if (sp_ji_fen[ce] < -SPEED_LOOP_I_LIMIT) sp_ji_fen[ce] = -SPEED_LOOP_I_LIMIT;

    xiu = wu_cha * SPEED_LOOP_KP / 100 + sp_ji_fen[ce];                // 修正量（占空比）
    if (xiu >  SPEED_LOOP_CHA_SHANG_XIAN) xiu =  SPEED_LOOP_CHA_SHANG_XIAN;
    if (xiu < -SPEED_LOOP_CHA_SHANG_XIAN) xiu = -SPEED_LOOP_CHA_SHANG_XIAN;

    xiu += mu_biao_duty;                                              // 前馈为主，PI 只做修正
    if (xiu < 0) xiu = 0;
    if (xiu > MOTOR_PWM_MAX) xiu = MOTOR_PWM_MAX;
    return xiu;
}
#endif

void control (uint32 tick, int16 delta, int speed)
{
    int32 ji_chu = (speed > 0) ? (int32)speed : 0;      // 负 speed 会让无符号占空比变成极大值，先挡掉
    int32 zuo, you;

    /* 冷却期：舵机强制回中，只保留速度与软启动，等新一帧图像重新建立误差后再接管 */
    if (turn_leng_que_zhen)
    {
        turn_leng_que_zhen--;
        delta = 0;
    }

    if (tick < START_TIME)
    {
        /* 软启动：0.3 s 内线性爬到目标占空比 */
        uint32 jia_su = (uint32)((uint64)tick * (uint64)ji_chu / START_TIME);
        if (jia_su < (uint32)ji_chu)
            ji_chu = (int32)jia_su;
    }

    zuo = ji_chu;
    you = ji_chu;

#if MOTOR_DIFF_ENABLE
    {
        int32 pian  = (delta > 0) ? (int32)delta : -(int32)delta;
        int32 jian  = pian * MOTOR_DIFF_NEI_XISHU / DELTA_LIMIT;    // 打舵越多，内轮减得越多
        int32 zhuan = (int32)delta * MOTOR_DIFF_FANG_XIANG;

        if (jian > MOTOR_DIFF_NEI_XISHU) jian = MOTOR_DIFF_NEI_XISHU;
        if (zhuan > 0)
            you = ji_chu * (100 - jian) / 100;                      // 右转，右轮是内轮
        else if (zhuan < 0)
            zuo = ji_chu * (100 - jian) / 100;                      // 左转，左轮是内轮
    }
#endif

#if SPEED_LOOP_OK
    /* 速度闭环：左右轮各自修正（审查 §9 第 6 条）。MOTOR_ZUO/MOTOR_YOU 见 motor.h */
    zuo = su_du_huan(MOTOR_ZUO, zuo);
    you = su_du_huan(MOTOR_YOU, you);
#endif

    drive((int16)zuo, (int16)you);                                  // 两轮统一出口，右轮从此有输出
    pwm_set_duty(ATOM0_CH1_P33_9, min(max(SERVO_CENTER + delta, SERVO_MIN), SERVO_MAX));   // 一个方向
}   // PID参数设置，提高P值能够使反应更快，但可能会过冲，设置i值，能够让整体轨迹更接近目标值，d值能够减小过冲（振荡）。
//调节顺序 / I 调节 / D 只在突变时起作用，三段原文

/* ===================================================================================================================
 * 编码器差速转弯  turn(degrees)      前轮舵机定圆心 + 后轮左右电机差速补半径
 * 2026-11-12 新增第 0 步「触发后直行」：路口识别后先直行 TURN_TI_QIAN_MM 再转弯，
 * 即转弯点在识别点之后这一段距离处（2026-11-13 订正，参数见 tiaocan.h 第 6.6 节）。
 *
 * 为什么要混合
 *   纯阿克曼：舵机打死半径也下不来，180 度要 1 m 以上空间；
 *   纯差速  ：轮距只有 100 多 mm，原地转太磨胎、也折腾减速箱；
 *   混合    ：舵机给一个中等转角定圆心，左右轮按半径差分配转速，半径压到 150~400 mm。
 *
 * 数学（标定被拆成两个能直接量的长度，不再是黑盒脉冲数）
 *   L 轴距、T 轮距、每轮圈脉冲 N、轮周长 C
 *   目标转角 th，前轮角 d = min(th, TURN_ZUI_DA_ZHUAN_JIAO_DU)，半径 R = L / tan(d)
 *   每度脉冲 P = R * (pi/180) / (C/N)，左右轮共用
 *   绕同一圆心转，左右轮弧长比 = 半径比：(R - T/2) : (R + T/2)
 *
 * 闭环
 *   每 TURN_CE_SU_WIN_MS 比较一次左右轮实测弧度增量和期望值，PI 修正占空比差；
 *   丢线只看两轮平均里程，与串扰无关。单窗速率低于 TURN_DU_ZHUAN_ZHI 判堵转。
 *
 * 量程
 *   |degrees| <= TURN_ZUI_DA_JIAO_DU（默认 360）。大角度就是沿同一半径多转一会儿：
 *   目标弧长 = R * th(rad)，闭环按累计角度收口，120 / 300 度和 90 度走的是同一套逻辑。
 *   唯一跟着变的是超时保护：大角度得给更多时间，否则会被 TURN_CHAO_SHI_MS 提前打断。
 *   注意 300 度要绕出直径 2R 的圆（R=409 时约 0.8 m），场地先留出来。
 * =================================================================================================================== */

volatile int32 turn_lei_ji_chong[2] = {0, 0};   // [0]左轮 [1]右轮，本次累计脉冲（绝对值累加）
volatile int16 turn_shi_ji_jiao_du   = 0;       // 按两轮平均里程反算的实际转角
volatile int16 turn_shi_ji_ban_jing  = 0;       // 由实测左右轮速比反推的半径 mm，标定/排障看它
volatile uint8 turn_jie_guo          = TURN_WAN_CHENG;

static int32 turn_qu_jue_dui (int32 v)
{
    return (v < 0) ? -v : v;
}

// 左右占空比钳位后输出（不能直接改调用方的变量，差速环的调节量得留在外面）
static void turn_tuo_zhan (int32 cc_l, int32 cc_r)
{
    if (cc_l < 0) cc_l = 0;
    if (cc_r < 0) cc_r = 0;
    if (cc_l > TURN_ZUI_DA_ZHAN_KONG) cc_l = TURN_ZUI_DA_ZHAN_KONG;   // 仅钳局部副本
    if (cc_r > TURN_ZUI_DA_ZHAN_KONG) cc_r = TURN_ZUI_DA_ZHAN_KONG;
    drive((int16)cc_l, (int16)cc_r);                                  // 与巡线共用同一个双轮出口
}

int16 turn (int16 degrees)
{
    int16  shang_ci[2];         // 上一轮采样，用于求增量
    int32  qi_shi[2];           // 本窗口起点累计（必须 int32，见下）
    int16  bu;
    int16  fang_xiang;
    int16  jiao_du;
    int16  jiao_du_shou;
    int32  ban_jing;            // 目标半径 mm
    int32  ban_jing_ce;         // 由实测速比反推的当前半径
    float  mei_mm_chong;        // 每毫米脉冲 = 每圈脉冲 / 轮周长
    float  mei_du_ping_jun;     // 每度「两轮平均」脉冲（R=409 时约 537；必须浮点，取整会吃精度）
    float  mu_biao_jiao;        // 目标转角（度）
    float  yi_zou_chong;        // 两轮平均里程（脉冲）
    float  ban_jing_bi;         // 目标半径 / 实测半径，限幅 0.7~1.3
    float  yi_zou_jiao;         // 已走角度（按实测半径折算，浮点比较，不做整数折算）
    float  xi_wang_cha;         // 期望：每度「外轮 - 内轮」脉冲差
    float  cha_pi;              // 本窗口差速 PI 修正量，单位：占空比
    float  cha_pi_jifen;        // 积分项
    float  shi_ji_cha;          // 实测：每度「外轮 - 内轮」脉冲差
    int32  shi_ji_ban_jing;
    int32  cc_wai;
    int32  cc_nei;
    int32  cc_l;
    int32  cc_r;
    int32  d_zeng;
    uint32 t0;
    uint32 t_xian;              // 线里程到达目标的时刻（10 ns），收尾滑行按它算
    uint8  dao_dian;            // 里程已到点标志（不要用 t_xian==0 当哨兵，时钟可能刚好为 0）
    uint32 t_ce;
    uint32 t_now;
    uint32 dt;
    uint32 chao_shi_ms;         // 本次超时门限（基准 + 每度余量 × 角度）
    uint8  i;

    turn_lei_ji_chong[0] = 0;
    turn_lei_ji_chong[1] = 0;
    turn_shi_ji_jiao_du  = 0;
    turn_shi_ji_ban_jing = 0;
    turn_jie_guo         = TURN_WAN_CHENG;
    cha_pi               = 0;
    cha_pi_jifen         = 0;

    /* 量程收口：默认 ±360（整圈），量程外的调用直接压到边界，不至于失控。
     * 要把范围改大，只改 tiaocan.h 的 TURN_ZUI_DA_JIAO_DU，别在这里写死数字 */
    if (degrees >  TURN_ZUI_DA_JIAO_DU) degrees =  (int16)TURN_ZUI_DA_JIAO_DU;
    if (degrees < -TURN_ZUI_DA_JIAO_DU) degrees = -(int16)TURN_ZUI_DA_JIAO_DU;
    if (degrees <  TURN_ZUI_XIAO_JIAO_DU &&
        degrees > -TURN_ZUI_XIAO_JIAO_DU) return 0;
    if (TURN_MEI_LUN_QUAN_CHONG_MEI_DU <= 0)             // 没标定就别动
    {
        turn_jie_guo = TURN_CAN_SHU_CUO;
        return 0;
    }

    jiao_du    = (degrees > 0) ?  degrees : (int16)(-degrees);
    fang_xiang = (degrees > 0) ?  TURN_FANG_XIANG : (int16)(-TURN_FANG_XIANG);

    /* 超时按角度加：300 度走 2.1 m 弧长，固定 8 s 在慢车上会误判超时。
     * 40×360 = 14400，uint32 乘不会溢出；但总时长要守在 42 s 内（时钟 32 位回绕） */
    chao_shi_ms = (uint32)TURN_CHAO_SHI_MS
                + (uint32)TURN_CHAO_SHI_MEI_DU_MS * (uint32)jiao_du;

    /* ---- 目标半径：受前轮可用转角限制，R = L / tan(d) ---- */
    jiao_du_shou = (jiao_du > TURN_ZUI_DA_ZHUAN_JIAO_DU) ? TURN_ZUI_DA_ZHUAN_JIAO_DU : jiao_du;
    ban_jing = (int32)((float)TURN_AXIS_MM * 57.29578f / (float)jiao_du_shou + 0.5f);   // 小角度近似 L/d
    // 这里不再钳位：半径由舵机角度决定，钳了会与实际弧长对不上，导致提早断电。

    /* ---- 每毫米脉冲：单位统一从这里出发，后面全部是「长度 × 脉冲/mm」 ---- */
    mei_mm_chong = (float)TURN_MEI_LUN_QUAN_CHONG_MEI_DU / (float)TURN_LUN_ZHOU_MM;

    /* ---- 左右轮每度脉冲差，用浮点算，避免整数截断把小量吃成 0 ---- */
    /* 期望「每度外轮 - 内轮脉冲差」= 轮距弧长 × 每毫米脉冲 = T × (π/180) × 每毫米脉冲。
     * 原来这一行少了 (π/180)：算出 8282（其实单位是「脉冲/mm × mm」，根本不是「每度」），
     * 比正确值大约 57 倍，误差项长期顶在饱和值上，PI 变成恒定偏置 -- 台架上方向对、看着也准，
     * 但闭环从来没闭合过（审查 P0-1，数值 8282 与源码一致）。
     * 注意：别按审查报告那一版改成 (int32)(×1000) 定点：那只是把增益砍到 1/8，不是修量纲。 */
    xi_wang_cha  = mei_mm_chong * (float)TURN_LUN_JU_MM * 3.14159265f / 180.0f;
    /* 两轮平均里程（每度）：车身沿半径 R 前进的弧长 -> 脉冲。
     * 平均弧长 = R × (pi/180)，不是内轮弧长；差速环自己管左右分配，这里只管总量。 */
    mei_du_ping_jun = mei_mm_chong * (float)ban_jing * 3.14159265f / 180.0f;
    /* 目标"已走角度"直接就是 jiao_du，不用再折成脉冲：
     * 里程折算出来的 yi_zou_jiao 也是度，两边同量纲才不会出「45 度走成 90 度」这种错 */
    mu_biao_jiao    = (float)jiao_du;

    /* ---- 初始占空比：外轮基准，内轮按半径比缩 ---- */
    cc_wai = TURN_ZHUAN_SU_DU;
    cc_nei = (int32)((float)TURN_ZHUAN_SU_DU * (float)(ban_jing - TURN_LUN_JU_MM / 2) / (float)ban_jing);
    if (cc_nei < TURN_NEI_LUN_ZUI_DI) cc_nei = TURN_NEI_LUN_ZUI_DI;                     // 内轮太小等于刹车，留底
    cc_l = (fang_xiang > 0) ? cc_wai : cc_nei;                                          // 右转圆心在右，左轮是外轮
    cc_r = (fang_xiang > 0) ? cc_nei : cc_wai;

    /* ---- 0. 触发转弯后的直行段（2026-11-12 新增；2026-11-13 注释方向订正）----
     * 摄像头识别到路口后，先以 TURN_TI_QIAN_SU_DU 直行 TURN_TI_QIAN_MM 毫米，
     * 才进入下面的「停车打舵 + 差速转弯」流程：转弯点在识别点【之后】TURN_TI_QIAN_MM 处，
     * 不是路口之前（原注释「前移到路口前」与实现方向相反，已订正）。
     * 里程用双轮编码器平均脉冲闭环；编码器失效或堵转由
     * TURN_TI_QIAN_CHAO_SHI_MS 兜底，超时后照常进入转弯，不会一路直冲。
     * TURN_TI_QIAN_MM = 0 时本段整体跳过，行为与改动前完全一致。 */
    if (TURN_TI_QIAN_MM > 0)
    {
        float  mei_mm_zhi    = (float)TURN_MEI_LUN_QUAN_CHONG_MEI_DU / (float)TURN_LUN_ZHOU_MM;
        int32  zhi_lei_ji[2] = {0, 0};      // [0]左轮 [1]右轮，直行段各自累计脉冲（绝对值）
        int16  zhi_shang_ci[2];
        uint32 zhi_t0        = system_getval();
        uint8  zhi_i;
        int16  zhi_bu;

        zhi_shang_ci[0] = encoder_get_count(TURN_ENCODER_ZUO);
        zhi_shang_ci[1] = encoder_get_count(TURN_ENCODER_YOU);
        turn_tuo_zhan(TURN_TI_QIAN_SU_DU, TURN_TI_QIAN_SU_DU);      // 两轮同占空比 = 直行

        while (TRUE)
        {
            int32 zhi_he = 0;
            for (zhi_i = 0; zhi_i < 2; zhi_i++)
            {
                zhi_bu = (int16)(encoder_get_count(zhi_i ? TURN_ENCODER_YOU : TURN_ENCODER_ZUO)
                                 - zhi_shang_ci[zhi_i]);
                zhi_shang_ci[zhi_i] = (int16)(zhi_shang_ci[zhi_i] + zhi_bu);
                if (zhi_bu > TURN_BU_CHONG_SHANG_XIAN || zhi_bu < -TURN_BU_CHONG_SHANG_XIAN)
                    zhi_bu = 0;                                     // 野值丢弃，判据与转弯段一致
                zhi_lei_ji[zhi_i] += turn_qu_jue_dui(zhi_bu);
                zhi_he += zhi_lei_ji[zhi_i];
            }
            if ((float)zhi_he / 2.0f / mei_mm_zhi >= (float)TURN_TI_QIAN_MM)
                break;                                              // 双轮平均里程到达提前量
            if ((uint64)(system_getval() - zhi_t0) > (uint64)TURN_TI_QIAN_CHAO_SHI_MS * 100000ULL)
                break;                                              // 超时兜底，照样进入转弯
        }
        turn_tuo_zhan(0, 0);                                        // 到点停车，交给下面的打舵流程
    }

    /* ---- 1. 先停车打舵，等前轮到位再起步 ---- */
    turn_tuo_zhan(0, 0);
    pwm_set_duty(ATOM0_CH1_P33_9,
                 (uint32)((int32)SERVO_CENTER + (int32)fang_xiang * (int32)TURN_STEER_DUTY));
    system_delay_ms(TURN_DA_DUO_MS);

    /* ---- 2. 从此刻开始数里程 ---- */
    shang_ci[0] = encoder_get_count(TURN_ENCODER_ZUO);
    shang_ci[1] = encoder_get_count(TURN_ENCODER_YOU);
    qi_shi[0]   = 0;
    qi_shi[1]   = 0;
    t0  = system_getval();
    t_ce = t0;
    t_xian  = 0;
    dao_dian = 0;
    turn_tuo_zhan(cc_l, cc_r);

    /* ---- 3. 边走边数 + 差速闭环 ---- */
    while (TRUE)
    {
        for (i = 0; i < 2; i++)
        {
            bu = (int16)(encoder_get_count(i ? TURN_ENCODER_YOU : TURN_ENCODER_ZUO) - shang_ci[i]);
            shang_ci[i] = (int16)(shang_ci[i] + bu);
            if (bu > TURN_BU_CHONG_SHANG_XIAN || bu < -TURN_BU_CHONG_SHANG_XIAN)
            {
                bu = 0;                                                  // 野值（干扰/异常跳变）丢弃
            }
            turn_lei_ji_chong[i] += turn_qu_jue_dui(bu);
        }

        /* 里程：先按实测左右轮速比反推「当前实际半径」，再把两轮里程折成角度
         * （内轮走 φ(R-T/2)，外轮走 φ(R+T/2)，平均正好 = φR） */
        {
            int32 cha_ce = turn_lei_ji_chong[0] - turn_lei_ji_chong[1];
            if (cha_ce < 0) cha_ce = -cha_ce;
            if (cha_ce > 0)
                ban_jing_ce = (int32)((float)TURN_LUN_JU_MM / 2.0f
                               * (float)(turn_lei_ji_chong[0] + turn_lei_ji_chong[1])
                               / (float)cha_ce + 0.5f);            // 左右对称：取差值绝对值，左转右转都一样
            else
                ban_jing_ce = 0;                                   // 两轮同步 = 走直线 = 半径无穷大
        }
        if (ban_jing_ce < 1) ban_jing_ce = ban_jing;               // 起步两轮还没拉开差距，先按目标半径算
        yi_zou_chong = (float)(turn_lei_ji_chong[0] + turn_lei_ji_chong[1]) / 2.0f;

        /* 两轮里程差按理说 = (T/R) × 平均里程。实测差超过期望的 2 倍说明在打滑，
         * 实测半径不可信，退回目标半径，别被假半径骗着少转（也不会因此多转） */
        if (yi_zou_chong > 300.0f &&
            (float)turn_qu_jue_dui(turn_lei_ji_chong[0] - turn_lei_ji_chong[1])
                > 2.0f * (yi_zou_chong / (float)ban_jing) * (float)TURN_LUN_JU_MM)
        {
            ban_jing_ce = ban_jing;
        }
        /* 比值夹在 0.7~1.3（原来 0.5~1.5）：打滑 33% 时实测半径偏小，比值顶到 1.5，角度直接算少 33%，
         * 车就转不到位（审查 P0-1 补充项）。夹窄到 ±30% 等于「最多容忍 30% 打滑」，
         * 超出部分交给上面那段 2× 判据退回目标半径，别靠放宽夹持去兜。 */
        ban_jing_bi = (float)ban_jing / (float)ban_jing_ce;
        if (ban_jing_bi < 0.7f) ban_jing_bi = 0.7f;
        if (ban_jing_bi > 1.3f) ban_jing_bi = 1.3f;

        /* 全程用浮点比较，不折算成整数角度：
         * 一旦把已走角度取整，低车速时每步的增量不到 1 度，截断后剩余量永远差一点，
         * 只能靠超时退出（这个坑实测过：turn(90) 一路转到 8 秒超时才停） */
        yi_zou_jiao = yi_zou_chong * ban_jing_bi / mei_du_ping_jun;

        if (!dao_dian &&
            (mu_biao_jiao - yi_zou_jiao) <= (float)TURN_TI_QIAN_JIAO_DU)
        {
            dao_dian = 1;
            if (TURN_HUA_XING_MS == 0)
            {
                break;                                             // 不滑行：到点就断电量
            }
            t_xian = system_getval();                              // 记录断点，余下按时间滑行
            t_ce   = t_xian;
        }
        /* 注意：TC264 上 unsigned long 就是 32 位，100000UL 不会自动升 64 位，
         * 8 位/16 位宏一乘就溢出，所以统一用 uint64 强转再比。
         * TURN_HUA_XING_MS = 0 时这一支永远成立 -- 与"不滑行"的语义一致，只是编译器会提示。 */
        if (dao_dian)
        {
            if (TURN_HUA_XING_MS == 0) break;                      // 不滑行：到点立刻收工
            if ((uint64)(system_getval() - t_xian) >= (uint64)TURN_HUA_XING_MS * 100000ULL)
            {
                break;                                             // 滑行补足时间到，收工
            }
        }

        t_now = system_getval();
        dt    = (uint32)(t_now - t_ce);

        /* 窗口到期：比一次左右轮弧度差，PI 修正占空比差 */
        if ((uint64)dt >= (uint64)TURN_CE_SU_WIN_MS * 100000ULL)
        {
            d_zeng = (int32)((turn_lei_ji_chong[0] - qi_shi[0]) + (turn_lei_ji_chong[1] - qi_shi[1])) / 2;
            if (d_zeng < 1) d_zeng = 1;                            // 防除零，堵转由下面的速率判据兜底

            /* 实测「每度外轮 - 内轮脉冲差」：先把窗口里程折成角度，再算差，与 xi_wang_cha 同量纲。
             * 符号统一成「外轮 - 内轮」：右转外轮是左轮、左转外轮是右轮，两边都取正值，
             * 「外轮走少了」永远是同一个方向。原写法固定用「右 - 左」，右转时误差自带
             * 约 2×145 的恒定正偏置，方向对了也永远调不平（审查 P0-1 的另一半）。 */
            {
                float ce_du  = (float)d_zeng / mei_du_ping_jun;                  // 本窗口走过的角度（度）
                float ce_zuo = (float)(turn_lei_ji_chong[0] - qi_shi[0]);        // 左轮增量
                float ce_you = (float)(turn_lei_ji_chong[1] - qi_shi[1]);        // 右轮增量
                float ce_cha = (fang_xiang > 0) ? (ce_zuo - ce_you) : (ce_you - ce_zuo);
                shi_ji_cha = (ce_du > 0.001f) ? (ce_cha / ce_du) : xi_wang_cha;  // 太小就按期望值处理，别除爆
            }

            /* 积分按窗口时间归一化（审查 N-6）：原来每窗固定 /8，等于把积分增益和窗口长度绑死，
             * 改 TURN_CE_SU_WIN_MS 就悄悄改增益、窗口一大就过冲失稳。这里按「标称窗口 / 实际窗口」
             * 折算，标称窗口下与原行为等价，改窗口只影响采样率、不影响等效积分时间。 */
            {
                float dt_ms  = (float)dt / 100000.0f;                            // dt 单位 10 ns
                float win_bi = (dt_ms > 0.001f) ? ((float)TURN_CE_SU_WIN_MS / dt_ms) : 1.0f;
                float wu_cha = xi_wang_cha - shi_ji_cha;                         // 外轮走少了 -> 正 -> 外轮加压
                cha_pi_jifen += wu_cha * win_bi / 8.0f;
            }
            if (cha_pi_jifen >  (float)TURN_CHA_PI_JI_FEN_SHANG_XIAN) cha_pi_jifen =  (float)TURN_CHA_PI_JI_FEN_SHANG_XIAN;
            if (cha_pi_jifen < -(float)TURN_CHA_PI_JI_FEN_SHANG_XIAN) cha_pi_jifen = -(float)TURN_CHA_PI_JI_FEN_SHANG_XIAN;
            cha_pi = (xi_wang_cha - shi_ji_cha) * (float)TURN_CHA_PI_KP
                   + cha_pi_jifen * (float)TURN_CHA_PI_KI;

            /* 实测速比反推半径，台架标定时看它有没有接近 R = L/tan(d) */
            {
                int32 cha_ce = turn_lei_ji_chong[0] - turn_lei_ji_chong[1];
                if (cha_ce < 0) cha_ce = -cha_ce;
                if (cha_ce > 0)
                    shi_ji_ban_jing = (int32)((float)TURN_LUN_JU_MM / 2.0f
                                      * (float)(turn_lei_ji_chong[0] + turn_lei_ji_chong[1])
                                      / (float)cha_ce + 0.5f);
                else
                    shi_ji_ban_jing = 0;
            }
            if (shi_ji_ban_jing > 9999) shi_ji_ban_jing = 9999;
            turn_shi_ji_ban_jing = (int16)shi_ji_ban_jing;

            /* 堵转：窗口内两轮平均几乎不动 */
            if (d_zeng < TURN_DU_ZHUAN_ZHI)
            {
                turn_jie_guo = TURN_DU_ZHUAN;
                break;
            }

            if (fang_xiang > 0)   // 右转：左轮外、右轮内
            {
                cc_l = cc_wai + (int32)cha_pi;
                cc_r = cc_nei - (int32)cha_pi;
            }
            else                  // 左转：右轮外
            {
                cc_r = cc_wai + (int32)cha_pi;
                cc_l = cc_nei - (int32)cha_pi;
            }

            /* 本窗口起点用 int32 存（原来是 (int16) 强制转换）：turn_lei_ji_chong 是 int32 累计值，
             * 一旦超过 32 767 脉冲（R≈409 时约 61°），(int16) 会把起点压回负数，差值瞬间多出 65 536，
             * 堵转判据 d_zeng < TURN_DU_ZHUAN_ZHI 恒不成立，转弯只能等超时退出（审查 P0-2，已核实）。 */
            qi_shi[0] = turn_lei_ji_chong[0];
            qi_shi[1] = turn_lei_ji_chong[1];
            t_ce      = t_now;
        }

        if ((uint64)(t_now - t0) > (uint64)chao_shi_ms * 100000ULL)
        {
            turn_jie_guo = TURN_CHAO_SHI;
            break;
        }

        turn_tuo_zhan(cc_l, cc_r);
    }

    /* ---- 4. 收尾：断电、舵机回中、等车身滑停、PID 归零交回巡线 ---- */
    turn_tuo_zhan(0, 0);
    pwm_set_duty(ATOM0_CH1_P33_9, SERVO_CENTER);
    system_delay_ms(TURN_TING_CHE_MS);
    pid_reset();
    turn_leng_que_zhen = TURN_LENG_QUE_ZHEN;    // 交回巡线后先直行若干帧，别让旧 err 把舵机拉回去

    /* 回填实测角度：用真实里程 / 每度平均里程，别用浮点 angle 的取整结果 */
    if (mei_du_ping_jun > 0.5f)
    {
        yi_zou_jiao = (float)(turn_lei_ji_chong[0] + turn_lei_ji_chong[1]) / 2.0f / mei_du_ping_jun;
        jiao_du = (int16)(yi_zou_jiao + 0.5f);
    }
    else
        jiao_du = 0;
    if (degrees < 0) jiao_du = (int16)(-jiao_du);
    turn_shi_ji_jiao_du = jiao_du;

    return jiao_du;
}
