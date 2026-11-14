/*********************************************************************************************************************
 * 文件名称          path_plan.c
 * 作用              赛道路径规划：按编码器里程执行「巡线段 → 路口动作」的 36 步路线（含末端越行段）
 * 编码              GBK（与工程其余源码一致，勿另存为 UTF-8）
 * 来源              项目地图《规划路线.jpg》（01_项目文档/赛道路径规划/，按里程分 7 段顺时针行驶）。
 *                    原《赛道路径规划报告_v4》缺失，本表按地图 + 实车标定制订：约 27.45 m / 36 步 /
 *                    24 个 90°弯 / 21 个 45°弯 / 全程不倒车 / 跑过 R8 终点线后再越行一段才停（地图要求末端多走一段）
 * 设计要点
 *   1. 触发靠里程，不靠图像。摄像头看路口受光照、宽赛道、三极管断线影响，重复性差；
 *      编码器只要「每圈脉冲」标定过就稳定。图像仍然负责段内巡线，两者分工不重叠。
 *   2. 每段独立计数，不在段间累加：到点执行动作后立刻把脉冲基准清零，
 *      这样第 k 段的误差不会带到第 k+1 段（35 段累加的话，1% 误差就是 27 cm）。
 *   3. 转弯仍复用 control.c 的 turn()：它的舵机定圆心 + 左右轮差速闭环已经做过标定流程，
 *      这里不再另写一套开环打舵。
 *   4. 三极管符号（Q1~Q7）处主赛道线是断开的，图像一定判丢线，所以直穿段进入「盲行」：
 *      主循环看 path_plan_mang_xing() 后强制 err=0 直行，完全不看图像。
 * 上车必读
 *   ★ 段长按地图《规划路线.jpg》的几何直线段填，实车转弯是圆弧，转弯期间车还会往前走一段，
 *     所以每段的实际触发里程会和表里的段长有偏差。第一次上车先让它空跑一遍，
 *     把 path_plan_shi_ji_mm[] 抄下来逐段修 duan_chang_mm —— 这张表就是一版一版标出来的。
 *   ★ 本模块未在 ADS 编译、未上板，第一次一定要架起来/低速试。
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "path_plan.h"
#include "motor.h"
#include "control.h"

/* 主循环（cpu0_main.c）里丢线记忆的两个变量。转弯前必须清掉：
 * 否则转弯后第一帧刚好丢线，会拿「转弯前」的舵机偏置接管，把刚转过来的车头又掰回去。 */
extern volatile uint8 diu_xian_zhen;
extern volatile int16 diu_xian_delta;

/* ===================================================================================================================
 * 路线表 —— 逐行对应地图《规划路线.jpg》的 36 步执行序列（含末端越行段）
 *   duan_chang_mm：报告「本段长度」，单位 mm
 *   dong_zuo     ：报告「到达时动作」
 *   注释里的累计里程 = 报告「累计里程」，只作对账，程序不用它
 * 方向约定：右转 = 正角度（与 TURN_FANG_XIANG = +1 一致）；实车转向反了先改 TURN_FANG_XIANG，
 *           不要逐行改这张表。
 *
 * 两点说明，别当成 bug：
 *   1. 段长逐行相加（第 1~35 步，不含越行段）= 26 970 mm；加上末尾越行段
 *      PP_FINISH_OVER_RUN_MM（默认 500 mm）总里程约 27 470 mm。逐段独立触发，误差不跨段累积，
 *      段长按实车 path_plan_shi_ji_mm[] 回填，别去「凑平」累计值。
 *   2. 全程 24 个 90° 弯，本表显式 turn() 的有 14 个：其余 10 个在**段内**由巡线自己拐过去。
 *      本表只管路口，前提是车能稳定巡过 90° 弯 —— 巡线本身吃力的弯，本表也救不了。
 *   3. 地图要求末端「多走一段路」：跑过 R8 终点线（第 35 步）不立即停车，再直行
 *      PP_FINISH_OVER_RUN_MM 毫米（第 36 步）后才停；越行距离按赛道停止区实地标定。
 * =================================================================================================================== */
const pp_lu_xian_t path_plan_lu_xian[] =
{
/*  段长   动作                     步号   累计里程 / 报告描述 */
    { 1100, PP_DONG_ZUO_YOU_90,     1 },   // 1.10 m  R1 R14             右转90°
    { 1280, PP_DONG_ZUO_ZHI_XING,   2 },   // 2.38 m  R16                直行
    {  900, PP_DONG_ZUO_YOU_90,     3 },   // 3.28 m  R17                右转90°
    { 2140, PP_DONG_ZUO_ZHI_XING,   4 },   // 5.42 m  R15 电源（绕左小环回到发车点旁）  直行
    { 1100, PP_DONG_ZUO_ZHI_XING,   5 },   // 6.51 m  R1 R14             直行（同一段走第二遍）
    {  400, PP_DONG_ZUO_ZHI_CHUAN,  6 },   // 6.92 m  Q1                 直穿 Q1
    { 1400, PP_DONG_ZUO_YOU_90,     7 },   // 8.31 m  R2 R3 Q1           右转90°
    {  430, PP_DONG_ZUO_YOU_90,     8 },   // 8.75 m  —                  右转90°
    {  740, PP_DONG_ZUO_ZHI_XING,   9 },   // 9.49 m  R6                 直行
    {  640, PP_DONG_ZUO_ZUO_90,    10 },   // 10.12 m R5                 左转90°
    {  780, PP_DONG_ZUO_ZHI_CHUAN, 11 },   // 10.90 m Q4                 直穿 Q4
    { 1330, PP_DONG_ZUO_ZHI_CHUAN, 12 },   // 12.23 m R9 Q4 Q5           直穿 Q5
    {  490, PP_DONG_ZUO_YOU_90,    13 },   // 12.72 m Q5                 右转90°
    {  900, PP_DONG_ZUO_ZHI_CHUAN, 14 },   // 13.63 m Q6                 直穿 Q6
    {  980, PP_DONG_ZUO_ZUO_90,    15 },   // 14.60 m Q6                 左转90°
    {  490, PP_DONG_ZUO_ZHI_XING,  16 },   // 15.10 m R11                直行
    {  880, PP_DONG_ZUO_ZHI_XING,  17 },   // 15.97 m R12                直行
    {  950, PP_DONG_ZUO_ZHI_CHUAN, 18 },   // 16.92 m R13 Q7             直穿 Q7
    {  390, PP_DONG_ZUO_ZHI_XING,  19 },   // 17.31 m Q7                 直行（0.39 m 重复段）
    {  190, PP_DONG_ZUO_XIE_45,    20 },   // 17.50 m —                  斜行45°
    { 1690, PP_DONG_ZUO_YOU_90,    21 },   // 19.19 m R10（六边形环岛，必须绕行不切弦）  右转90°
    {  390, PP_DONG_ZUO_ZHI_CHUAN, 22 },   // 19.58 m Q7                 直穿 Q7
    { 1410, PP_DONG_ZUO_ZHI_XING,  23 },   // 20.99 m R19 Q7             直行
    {  640, PP_DONG_ZUO_ZHI_XING,  24 },   // 21.63 m R18                直行
    {  670, PP_DONG_ZUO_ZHI_CHUAN, 25 },   // 22.30 m Q2                 直穿 Q2
    {  770, PP_DONG_ZUO_ZUO_90,    26 },   // 23.06 m R4 Q2              左转90°
    {  460, PP_DONG_ZUO_ZHI_CHUAN, 27 },   // 23.52 m Q3                 直穿 Q3
    {  670, PP_DONG_ZUO_ZHI_XING,  28 },   // 24.19 m R7 Q3              直行
    {  210, PP_DONG_ZUO_YOU_90,    29 },   // 24.40 m —                  右转90°
    {  640, PP_DONG_ZUO_YOU_90,    30 },   // 25.04 m R18                右转90°
    {  290, PP_DONG_ZUO_YOU_90,    31 },   // 25.33 m —                  右转90°
    {  740, PP_DONG_ZUO_ZHI_XING,  32 },   // 26.07 m —                  直行
    {  240, PP_DONG_ZUO_YOU_90,    33 },   // 26.31 m —                  右转90°
    {  280, PP_DONG_ZUO_ZUO_90,    34 },   // 26.59 m —                  左转90°
    {  360, PP_DONG_ZUO_ZHI_XING, 35 },   // 26.95 m R8 越过终点线（先不停车）
    {  PP_FINISH_OVER_RUN_MM, PP_DONG_ZUO_TING, 36 }, // 27.45 m 末端多走一段(越行)后停车（地图要求）
};

const uint8 path_plan_bu_shu = (uint8)(sizeof(path_plan_lu_xian) / sizeof(path_plan_lu_xian[0]));

/* ---------------- 状态 ---------------- */
volatile uint8  path_plan_bu_hao          = 0;      // 已完成的步数，0 表示还在跑第 1 段
volatile uint16 path_plan_ben_duan_mm     = 0;      // 本段已走里程
volatile uint16 path_plan_shang_duan_mm   = 0;      // 上一段实际触发里程（标定看这个）
volatile uint8  path_plan_shi_hua         = 1;      // 0 = 每圈脉冲没标定
volatile uint8  path_plan_biao_ding_chao_shi = 0;   // 1 = 某段超时没走到
uint16          path_plan_shi_ji_mm[PP_BU_SHU_MAX]; // 每段实际触发里程，标定用

static int32  pp_chong_qi[2]   = {0, 0};    // 本段起点的累计脉冲（左/右）
static uint8  pp_mang_xing     = 0;         // 1 = 盲行中
static uint8  pp_ting_che      = 0;         // 1 = 已到终点
static uint8  pp_yun_xing      = 0;         // 1 = 已发车，里程才开始算
static uint32 pp_duan_qi_tick  = 0;         // 本段起始时刻，用于超时兜底

/* 当前段从起点走过的脉冲（双轮平均，取绝对值：本车只前进，倒车没有意义） */
static int32 pp_duan_chong (void)
{
    int32 l = motor_li_cheng(MOTOR_ZUO) - pp_chong_qi[0];
    int32 r = motor_li_cheng(MOTOR_YOU) - pp_chong_qi[1];

    if (l < 0) l = -l;
    if (r < 0) r = -r;
    return (l + r) / 2;
}

/* 脉冲 -> 毫米。每圈脉冲没标定时返回 0 并置 path_plan_shi_hua = 0，
 * 主循环据此不再动作 —— 宁可不动，也不能按假里程乱转弯。 */
static uint16 pp_duan_mm (void)
{
    int32 chong = pp_duan_chong();
    int32 mm;

    if (TURN_MEI_LUN_QUAN_CHONG_MEI_DU <= 0 || TURN_LUN_ZHOU_MM <= 0)
    {
        path_plan_shi_hua = 0;
        return 0;
    }
    path_plan_shi_hua = 1;

    mm = (int32)((float)chong * (float)TURN_LUN_ZHOU_MM / (float)TURN_MEI_LUN_QUAN_CHONG_MEI_DU);
    if (mm < 0)     mm = 0;
    if (mm > 30000) mm = 30000;             // 单段最长 2.14 m，3 m 已是异常，钳住防 uint16 溢出
    return (uint16)mm;
}

/* 段起点对齐到当前脉冲，并把本段里程清零 */
static void pp_duan_chong_qi (void)
{
    pp_chong_qi[0]   = motor_li_cheng(MOTOR_ZUO);
    pp_chong_qi[1]   = motor_li_cheng(MOTOR_YOU);
    path_plan_ben_duan_mm = 0;
    pp_duan_qi_tick  = system_getval();
}

void path_plan_init (void)
{
    uint8 i;

    path_plan_bu_hao          = 0;
    path_plan_ben_duan_mm     = 0;
    path_plan_shang_duan_mm   = 0;
    path_plan_biao_ding_chao_shi = 0;
    pp_mang_xing = 0;
    pp_ting_che  = 0;
    pp_yun_xing  = 0;
    for (i = 0; i < PP_BU_SHU_MAX; i++)
        path_plan_shi_ji_mm[i] = 0;
    pp_duan_chong_qi();
}

void path_plan_qing_ling (void)
{
    path_plan_init();
    pp_yun_xing = 1;                        // 发车：从此刻开始算约 27.45 m（含终点越行段）
}

uint8 path_plan_mang_xing (void)
{
    return pp_mang_xing;
}

uint8 path_plan_ting_che (void)
{
    return pp_ting_che;
}

/* ===================================================================================================================
 * 到点：执行本段对应的动作
 * 返回 1 表示「本帧不要再往下走巡线流程」（转弯阻塞过 / 已停车），主循环据此 continue
 * =================================================================================================================== */
static uint8 pp_zhi_xing_dong_zuo (uint8 dong_zuo)
{
    switch (dong_zuo)
    {
        case PP_DONG_ZUO_YOU_90:
            diu_xian_zhen  = 0;             // 清丢线记忆，别让转弯前的偏置接管转弯后的第一帧
            diu_xian_delta = 0;
            turn(PP_90_JIAO_DU);            // 正 = 右转，符号由 TURN_FANG_XIANG 统一
            return 1;

        case PP_DONG_ZUO_ZUO_90:
            diu_xian_zhen  = 0;
            diu_xian_delta = 0;
            turn(-PP_90_JIAO_DU);
            return 1;

        case PP_DONG_ZUO_XIE_45:
            diu_xian_zhen  = 0;
            diu_xian_delta = 0;
            turn(PP_XIE_45_JIAO_DU);        // 正 = 右斜 45°，左斜就把它改成 -45
            return 1;

        case PP_DONG_ZUO_ZHI_CHUAN:
            /* 三极管符号处线断了：接下来 PP_MANG_XING_MM 毫米完全不看图像，err=0 直行。
             * 本路口就是盲行起点，而调用方紧接着会把段基准清零，所以盲行里程自然从 0 起算。 */
            pp_mang_xing = 1;
            return 0;                       // 直穿不阻塞，本帧继续正常巡线流程

        case PP_DONG_ZUO_TING:
            stop();                         // 双轮断电 + 舵机回中 + pid 归零
            pp_ting_che = 1;
            return 1;

        case PP_DONG_ZUO_ZHI_XING:
        default:
            return 0;                       // 直行通过，什么都不做
    }
}

uint8 path_plan_geng_xin (void)
{
    const pp_lu_xian_t *ben_duan;
    uint16 mu_biao_mm;
    uint16 zou_le;

    if (pp_ting_che || !pp_yun_xing || path_plan_bu_hao >= path_plan_bu_shu)
        return 0;

    ben_duan = &path_plan_lu_xian[path_plan_bu_hao];
    zou_le   = pp_duan_mm();
    path_plan_ben_duan_mm = zou_le;

    /* 盲行退出：走满 PP_MANG_XING_MM 就交回巡线。
     * 这里刻意不 return —— 盲行只决定主循环「怎么打舵」，不该挡住下面的到点判定：
     * 报告里直穿之后紧跟着 390 / 460 mm 的短段，一旦挡住就会整段开过头。 */
    if (pp_mang_xing && zou_le >= (uint16)PP_MANG_XING_MM)
        pp_mang_xing = 0;

    /* 到点判据：本段里程 >= 段长 - 转弯冲出补偿。
     * PP_TURN_CHONG_GUO_MM 是「上一次转弯结束时车头已经越过路口中心多远」，
     * 从当前段里扣掉它，避免每段都晚触发同样一段距离。标定前保持 0。 */
    mu_biao_mm = ben_duan->duan_chang_mm;
    if (mu_biao_mm > (uint16)PP_TURN_CHONG_GUO_MM)
        mu_biao_mm = (uint16)(mu_biao_mm - (uint16)PP_TURN_CHONG_GUO_MM);
    else
        mu_biao_mm = 50;                    // 极短段兜底，别减成 0 导致原地立刻触发

    if (zou_le >= mu_biao_mm)
    {
        path_plan_shi_ji_mm[path_plan_bu_hao] = zou_le;     // 标定用：这段实际走了多少
        path_plan_shang_duan_mm = zou_le;

        if (pp_zhi_xing_dong_zuo(ben_duan->dong_zuo))
        {
            /* 动过车（转弯阻塞 / 停车）：本帧到此为止，主循环会 pid_reset 后 continue */
            if (!pp_ting_che)
            {
                path_plan_bu_hao++;
                pp_duan_chong_qi();         // 下一段从当前位置重新计数，误差不跨段累积
            }
            return 1;
        }

        /* 直行通过 / 直穿起步：不阻塞，本帧继续正常巡线 */
        path_plan_bu_hao++;
        pp_duan_chong_qi();
        return 0;
    }

    /* 超时兜底：编码器线松、轮胎打滑到不动、表里的段长写得太大，都会卡在这一段。
     * 卡住不是「继续往前蹭」的理由 —— 直接停车并置标志，等人看，避免冲出场地撞墙。 */
    if (PP_DUAN_CHAO_SHI_MS > 0)
    {
        if ((uint64)(system_getval() - pp_duan_qi_tick) > (uint64)PP_DUAN_CHAO_SHI_MS * 100000ULL)
        {
            path_plan_biao_ding_chao_shi = 1;
            stop();
            pp_ting_che = 1;
            return 1;
        }
    }
    return 0;
}
