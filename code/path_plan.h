/*********************************************************************************************************************
 * 文件名称          path_plan.h
 * 作用              赛道路径规划（项目地图《规划路线.jpg》36 步路线，含末端越行段）：按编码器里程触发
 *                    右转/左转/斜行/直穿断线/终点停车
 * 编码              GBK（与工程其余源码一致，勿另存为 UTF-8）
 * 说明              本模块只做「段长表 + 动作表」，不碰图像、不碰 PWM：
 *                    转弯仍然走 control.c 的 turn()（舵机定圆心 + 差速闭环），
 *                    直行仍然走巡线 control()，所有输出还是 motor.c 的 drive() 唯一出口。
 *                  和原来的「路口检测触发 turn(90)」是两条互斥的路子：
 *                    PATH_PLAN_ENABLE = 1 → 用本模块（按里程走完 36 步，路口检测关闭）
 *                    PATH_PLAN_ENABLE = 0 → 回到 cpu0_main.c 里原来的路口检测逻辑
 ********************************************************************************************************************/
#ifndef PATH_PLAN_H
#define PATH_PLAN_H

#include "zf_common_headfile.h"
#include "tiaocan.h"

#define PP_BU_SHU_MAX 40                // 路线表最大步数（v4 是 35 步，留余量）

/* 动作码：表里每一行 = 「走完本段到达这个路口，然后做什么」 */
typedef enum
{
    PP_DONG_ZUO_ZHI_XING = 0,           // 直行通过（继续巡线，什么都不做）
    PP_DONG_ZUO_YOU_90,                 // 右转 90°
    PP_DONG_ZUO_ZUO_90,                 // 左转 90°
    PP_DONG_ZUO_ZHI_CHUAN,              // 直穿三极管符号（线是断的 → 进入盲行）
    PP_DONG_ZUO_XIE_45,                 // 斜行 45°（左斜还是右斜由 tiaocan.h 的 PP_XIE_45_JIAO_DU 定）
    PP_DONG_ZUO_TING,                   // 第 36 步：越过 R8 终点线、再走完越行段后停车
} pp_dong_zuo_enum;

typedef struct
{
    uint16 duan_chang_mm;               // 本段长度 mm：上一个路口中心 → 本路口中心
    uint8  dong_zuo;                    // 到达本路口后执行的动作，见 pp_dong_zuo_enum
    uint8  bu_hao;                      // 地图里的步号 1~36，只用来对账，不参与运算
} pp_lu_xian_t;

extern const pp_lu_xian_t path_plan_lu_xian[];   // 路线表，长度见 path_plan_bu_shu
extern const uint8        path_plan_bu_shu;

/* 上电调一次：状态清零（此时还没发车，不发车也不会有动作） */
void  path_plan_init      (void);

/* 发车那一刻调一次：把编码器里程基准归零，从第 1 步开始算。
 * 放在 cpu0_main.c 的 if_go 0→1 那个瞬间，不要放在上电时 ——
 * 上电到发车之间车被推、被挪，都会把这约 27.45 m 的基准吃掉。 */
void  path_plan_qing_ling (void);

/* 主循环每帧调：算本段里程 -> 到点执行动作（转弯/盲行/停车）。
 * 返回 1 = 本帧动过车（调过 turn() 或 stop()），主循环应当 pid_reset() 后 continue：
 *   转弯是阻塞的（1~2 s），返回后手里的图像是转弯期间拍的旧帧，拿它算 err 会再打一次舵。
 * 返回 0 = 只是普通推进，正常走巡线流程。 */
uint8 path_plan_geng_xin  (void);

/* 1 = 正在直穿断线区：主循环必须给 err=0、固定速度直行，不能看图像 */
uint8 path_plan_mang_xing (void);

/* 1 = 已到终点（或某段超时兜底停车）：主循环应调用 stop() 并不再驱动 */
uint8 path_plan_ting_che  (void);

/* ---------------- 调试 / 标定用的观测量（TC264 调试器里直接看） ---------------- */
extern volatile uint8  path_plan_bu_hao;          // 已完成到第几步（0 = 还没到第 1 个路口）
extern volatile uint16 path_plan_ben_duan_mm;     // 本段已走里程 mm
extern volatile uint16 path_plan_shang_duan_mm;   // 上一段实际触发里程 mm（对照表里的段长，判断要不要改表）
extern volatile uint8  path_plan_shi_hua;         // 0 = 每圈脉冲没标定，里程算不出来，模块不会动作
extern volatile uint8  path_plan_biao_ding_chao_shi;  // 1 = 某段跑了 PP_DUAN_CHAO_SHI_MS 还没到点（编码器/轮胎/表值有问题）
extern uint16          path_plan_shi_ji_mm[PP_BU_SHU_MAX];  // 每段实际触发里程 mm，上车后照着这张表逐段修 duan_chang_mm

#endif /* PATH_PLAN_H */
