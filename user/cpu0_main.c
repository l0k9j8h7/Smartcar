/*********************************************************************************************************************
 * TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC264 开源库的一部分
 *
 * TC264 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          cpu0_main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.10.2
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/
/*

 typedef struct
 {
 struct
 {
 uint8   head                        ;                                   // 帧头
 uint8   check_sum                   ;                                   // 和校验
 uint8   cmd                         ;                                   // 命令功能字
 uint8   camera_type                 ;                                   // 摄像头类型
 uint8   image_width_buffer[2]       ;                                   // 图像宽度 缓冲
 uint8   image_height_buffer[2]      ;                                   // 图像高度 缓冲
 uint8   data_type                   ;                                   // 数据类型 用于确认大小端
 uint8   reserve[3]                  ;                                   // 保留
 }config;
 uint32      buffer_byte_size            ;                                   // 数据缓冲区 实际字节大小
 void        *data_buffer                ;                                   // 数据缓冲区
 }seekfree_assistant_camera_struct;

 */

#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
#include"control.h"
#include"image.h"
#include"tiaocan.h"
#include"motor.h"

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

extern volatile uint16 copy_flag;   // 与 cpu1_main.c 保持一致：uint8 靠 250→255 溢出当哨兵，跨核语义太脆
extern uint8_t copy[MT9V03X_H][MT9V03X_W];
extern  volatile uint8 busy;
int64 last_frame=0;//记录上一帧的时间
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
seekfree_assistant_camera_struct camera_obj;   // 逐飞助手摄像头放 CPU0 RAM
//摄像头部分暂时放在这里
extern volatile int16 err;
uint8_t image_copy[MT9V03X_H][MT9V03X_W];              // 188×120 = 22560 字节备份缓冲区（二值化/去噪/找线用，会被就地改写）
#pragma section all restore
/* image_raw 必须放在 cpu0_dsram 之外（审查报告 P0-1 / §8 第 2 条，这是本轮最要紧的一条）
 * DSPR0 只有 72 KB，扣掉 CSA 8k + ustack 2k + istack 1k + heap 2k + 2×256 B 预留后可用 58,880 B；
 * 而 cpu0_dsram 里躺着三块整帧缓冲：image_copy 22,560 + last_image_copy 22,560 + image_raw 22,560 = 67,680 B，
 * 溢出 8,800 B（8.6 KiB）--链接阶段就该报段放不下，之前没发现是因为没人按 LSL 口径核过。
 * 本工程 LSL 的 LCF_DEFAULT_HOST = CPU1，去掉 pragma 后本变量自动落 DSPR1（120 KB），
 * CPU1 读它是本核本地访问（图传拷贝更快），CPU0 每帧写一次走 SRI，代价约 0.2~0.4 ms/帧，可接受。
 * 两条禁止：① 不要把 image_raw 改回 cpu0_dsram；② 不要为了省 RAM 删 last_image_copy，
 * 它是二值化失败时整帧回退旧图的唯一来源（image.c 的回退分支，CONSIST_FAIL_MAX=5）。 */
uint8_t image_raw[MT9V03X_H][MT9V03X_W];               // 原始灰度帧快照，仅供逐飞助手图传，全程不做二值化
#pragma section all "cpu0_dsram"

/* ---------------- 丢线记忆 + 缓降速度（251e810 报告 §5.1 / §7.4 第 6 条） ----------------
 * 旧写法丢线就回中：车顺着上一次的偏差直直冲出去，"回中"等于放弃追赶。
 * 只看一帧也不行：单帧抖动或二值化失败就满舵，救回来又被自己甩出赛道。
 * 所以记「丢线前的打舵状态」并缓降速度：
 *   - 丢线第 1 帧接管：记住丢线前的舵机偏置与车速（上一次有效帧的值）
 *   - 丢线持续：速度每帧按 DIU_XIAN_JIAN_SU 递减，舵机维持记忆偏置，不抖不甩
 *   - 超过 DIU_XIAN_ZUI_DA_ZHEN 帧仍无线：按 DIU_XIAN_MANG_QU_BI 加大打舵，死马当活马医
 *   - 重新找到线：一切交回巡线（err / pid 重新建立），速度由正常路径给回 MAX_SPEED
 * 断流保护（0.5 s 无新帧）走的是另一条路，不进这里的记忆。 */
volatile uint8 diu_xian_zhen    = 0;       // 已连续丢线帧数，0 = 在线
volatile int16 diu_xian_delta   = 0;       // 记忆的舵机偏置（丢线前最后一次有效输出）
volatile int32 diu_xian_su_du   = 0;       // 记忆并逐帧衰减的车速（占空比）

/* 正常巡线帧调用：记住这一帧的输出与速度，供丢线时接管 */
static void diu_xian_ji_yi (int16 delta, int speed)
{
    diu_xian_delta   = delta;
    diu_xian_su_du   = (speed > 0) ? (int32)speed : 0;
    diu_xian_zhen    = 0;
}

/* 丢线帧调用：返回本帧该给的车速，舵机偏置从 diu_xian_delta 取 */
static int diu_xian_jiang_su (void)
{
    if (diu_xian_zhen < 255)
        diu_xian_zhen++;

    /* 丢线超时停车（修复版 tiaocan.h 的 DIU_XIAN_TING_CHE_ZHEN，审查 §9 第 6 条）：
     * 缓降最低只到 DIU_XIAN_ZUI_DI_SU(150)，150 帧（约 3 s）还找不到线，说明已经冲出赛道、
     * 摄像头失效或者车被架空，继续给电只会把车怼到墙上。这里直接返回 0 停住，等人工复位。 */
    if (diu_xian_zhen > DIU_XIAN_TING_CHE_ZHEN)
        return 0;

    if (diu_xian_zhen == 1)                              // 第 1 帧：直接沿用丢线前的车速
        return (int)diu_xian_su_du;

    diu_xian_su_du = diu_xian_su_du * (100 - DIU_XIAN_JIAN_SU) / 100;   // 之后每帧缓降
    if (diu_xian_su_du < DIU_XIAN_ZUI_DI_SU)
        diu_xian_su_du = DIU_XIAN_ZUI_DI_SU;

    if (diu_xian_zhen > DIU_XIAN_ZUI_DA_ZHEN)            // 一直找不到线：加大打舵再搏一次
    {
        if (diu_xian_delta > 0)
            diu_xian_delta = (int16)min(diu_xian_delta + DIU_XIAN_MANG_QU_BI, DELTA_LIMIT);
        else if (diu_xian_delta < 0)
            diu_xian_delta = (int16)max(diu_xian_delta - DIU_XIAN_MANG_QU_BI, -DELTA_LIMIT);
    }
    return (int)diu_xian_su_du;
}
// **************************** 代码区域 ****************************
int core0_main (void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    system_start(); //计时启动
    // 此处编写用户代码 例如外设初始化代码等
    zf_log(mt9v03x_init(), "mt9v03x init error");
    /* ---------------- 电机 / 编码器 / 舵机（按 2026-11-09 实车接线图） ----------------
     *   左电机   P02.4 = PWM(ATOM0_CH4)   P02.6 = DIR(GPIO)
     *   右电机   P02.5 = PWM(ATOM0_CH5)   P02.7 = DIR(GPIO)
     *   左编码器 P02.8 = A  P00.9 = B     -> TIM4 正交
     *   右编码器 P33.7 = A  P33.6 = B     -> TIM2 正交
     *   舵机     P33.9 = ATOM0_CH1，50 Hz
     *   P33.1    = GPIO 推挽输出高（用途未在硬件上确认，见下面 gpio_init 处注释）
     * 注意：P02.0~P02.3 是摄像头 VSY/PCLK/SCCB，P02.4~P02.7 全给了电机，P02 口已排满。
     * 三个坑别再踩：
     *   ① 左编码器不要用 TIM5(P21_7/P21_6)：T5 属 GPT2，T5CON 没有增量接口模式，
     *      encoder_quad_init 开头的 zf_assert(encoder_n <= TIM4_ENCODER) 会直接命中并死等，
     *      现象是上电轮子不动、串口只刷 Assert error；
     *   ② P33_5 已腾空，左编码器 B 相改用 P00_9（T4EUDA），它和摄像头 P00_0~P00_7 不冲突；
     *   ③ 图上写的舵机 P03.9 是笔误，TC264 没有 P03 口，实际就是 P33_9。
     * 具体引脚全在 tiaocan.h 第六节，改接线只改那里，不要在这里硬写。 */
    motor_init();                                   // 双电机 PWM + 两路方向脚 + 两路编码器，见 code/motor.c
    pwm_init(ATOM0_CH1_P33_9, 50, SERVO_CENTER);    // 舵机 50 Hz，中值 750
    /* P33_1 置高：代码里没有交代用途，硬件上也没确认。
     * 如果它其实是 DRV8701 的 EN/nSLEEP（低电平休眠、高电平使能），那这里置高是对的；
     * 但那样就意味着「上电即使能驱动」，调试时轮子可能一动就窜，建议核实后改成
     * 「发车前再拉高」。如果它只是无关的指示脚，保留即可。改之前先量一下 P33.1 到驱动的走线。 */
    gpio_init(P33_1, GPO, 1, GPO_PUSH_PULL);
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_camera_config(&camera_obj, SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
    MT9V03X_W,              // 图像宽度，通常是 188
            MT9V03X_H,              // 图像高度，通常是 120
            copy              // 传入备份缓冲区地址
            );

    //引脚仅供参考
    // 此处编写用户代码 例如外设初始化代码等
    mt9v03x_set_exposure_time(100);             // 初始化时设一次就够了，不要放进主循环
    image_init();
    cpu_wait_event_ready();         // 等待所有核心初始化完毕

#if TURN_TAI_JIAN_CE_SHI
    /* 台架自检：只在 TURN_TAI_JIAN_CE_SHI = 1 时编译进来，跑一次 turn(90) 就死循环，
     * 方便把车架起来看左右轮转向和速比。正式跑车必须改回 0。 */
    turn(90);
    while (TRUE)
    {
        /* 停在这里，用调试器看 turn_lei_ji_chong / turn_shi_ji_ban_jing / turn_jie_guo */
    }
#endif

    uint32 start_ticks = system_getval();       // ← 发车那一刻打时间戳
    uint8 if_go = 0;       //是否发车
    uint8 lu_kou_lian_xu = 0;   // 连续识别到路口的帧数（触发判据见 tiaocan.h 的 LU_KOU_QUE_REN_ZHEN）
    last_frame = start_ticks;   // last_frame 为最近一帧时间戳，初始化成发车时刻供断流保护使用
    while (TRUE)       //需要添加帧同步信号防止撕裂
    {
        motor_speed_update();                   // 10 ms 限频的编码器测速，接口见 motor.h
        uint32 now_ticks = system_getval();     // 本轮时间戳：帧间隔与断流判断共用一个时刻，避免重复取时钟
        uint32 ticks = now_ticks - start_ticks; //计算t
        if (ticks > START_TIME)
            if_go = 1;   //已经出发
        if (if_go)
            ticks = START_TIME + 1;

        // 摄像头断流保护：放在帧分支外，0.5 s 无新帧时也能持续回中降速（原写法在 continue 之后，是死代码）
        // system_getval() 是 uint32/10ns，42.95 s 回绕；用 uint32 减法（模运算天然跨回绕），
        // 原 int64 写法在回绕点会得到负差值，导致断流保护最长失效 43 s
        if ((uint32)(now_ticks - (uint32)last_frame) > WU_ZHEN_CHAO_SHI)
        {
            control(ticks, 0, MAX_SPEED / SLOW_DOWN_DIV);
            pid_reset();
            /* 2026-11-13 审查订正（§8 漏报）：断流输出后跳过本轮帧分支。
             * 否则同轮 finish_flag=1（如 turn() 阻塞后的首帧）会再次 control() 覆盖回中降速输出。 */
            continue;
        }

        if (mt9v03x_finish_flag)                 // 只有 DMA 搬完了一整帧才进入
        {
            mt9v03x_finish_flag = 0;            // 先清标志，再拷数据
            /* 本帧与上一帧的间隔（10 ns 单位），供 pid() 做丢帧折算（审查报告 §4.2 / §8 第 5 条）。
             * 摄像头 50 fps 固定 20 ms 送帧，处理超过 20 ms 就丢帧，控制周期只会是 20/40/60 ms。 */
            uint32 dt_zhen = (uint32)(now_ticks - (uint32)last_frame);

            /* ---------------- 图传握手（一）：先等 CPU1 拷完，再覆盖 image_raw ----------------
             * 旧写法把忙等放在两次 memcpy 之后，真正的撕裂窗口（这里）根本没被盖住，
             * 而且等的是「CPU1 开始拷」而不是「拷完」。现在 CPU1 的 busy 只覆盖 memcpy
             * （约 0.2 ms，见 cpu1_main.c），所以这里正常几十 us 就过；
             * 超时只是放弃本帧保护、照样覆盖，绝不把主循环钉死。 */
            if (busy)
            {
                uint32 wait_start = system_getval();
                while (busy && (system_getval() - wait_start) < TU_CHUAN_CHAO_SHI)
                {
                    // 等 CPU1 把上一批 image_raw 拷进发送缓冲
                }
            }

            /* 一次取原始帧，分两路用：image_raw 给逐飞助手看原始灰度图（不做任何处理），
             * image_copy 给二值化/找线用。两份内容在拷贝当时完全相同，之后互不影响。 */
            memcpy(image_raw,  mt9v03x_image, MT9V03X_IMAGE_SIZE);  // 此刻 DMA 已 disable，到下一个 VSYNC 前 mt9v03x_image 不会被写
            memcpy(image_copy, image_raw,     MT9V03X_IMAGE_SIZE);
            last_frame = now_ticks;

            copy_flag++;                        // 攒帧计数：满 ZU_SE_ZHEN_SHU 后由 CPU1 取走并清零

            double_value(image_copy);           // 只处理副本，处理多久都不会撕裂

            /* ---------------- 图传握手（二）：批次交接 ----------------
             * 2026-11-10 改：逐飞助手改传「原始灰度图像」，不再传二值化后的图。
             * 2026-11-12 修（审查报告 §5 / §8 第 3 条，双端一起改）：
             *   旧写法 CPU0 在这里 busy 忙等，条件却是 !busy -- 等的是「CPU1 开始拷」，方向反了；
             *   而且 busy 覆盖「拷贝 + 1.958 s 发送」，与 50 ms 超时差 39 倍，每次必撞超时。
             *   现在握手只剩两件事：① 上面那段「拷完再覆盖」（在帧顶，超时 5 ms）；
             *   ② 计数满 ZU_SE_ZHEN_SHU 就交给 CPU1，由 CPU1 拷进本地缓冲后自己清零。
             * CPU0 不再忙等，发送的 1.96 s 期间照常跑控制，一帧控制都不会因此被拖慢。 */

            if (!image_valid)                   // 二值化已判不可信（连续失败/硬阈值回退），白线大概率已丢，不再拿它算 err
            {
                control(ticks, diu_xian_delta, diu_xian_jiang_su());   // 丢线记忆 + 缓降
                pid_reset();
                continue;
            }
            xiao_qu_yu_shan_chu(image_copy);

            /* ---------------- 路口检测 → 触发转弯（2026-11-12 新增） ----------------
             * find_boxes() 在底行出现宽度 >= ZUI_XIAO_LU_KOU_KUAN_DU 的白段时返回 255（路口）。
             * 这里直接调 find_boxes 而不是 image_read：后者会对每个框跑一遍 CNN，主循环吃不消，
             * 路口触发只需要 255 这个结果。连续 LU_KOU_QUE_REN_ZHEN 帧都识别到才确认（防单帧噪声）。
             * 确认后调用 turn(LU_KOU_TURN_JIAO_DU)：turn() 内部先直行 TURN_TI_QIAN_MM 再差速转弯，
             * 所以实际转弯点在识别点之后 TURN_TI_QIAN_MM 处、不是路口之前（2026-11-13 订正；阻塞执行，期间不巡线）。
             * 转弯收尾会置 turn_leng_que_zhen，冷却期内不检测，防止同一个路口二次触发。 */
            if (LU_KOU_TURN_JIAO_DU != 0 && turn_leng_que_zhen == 0)
            {
                uint8 lu_kou_box[MAX_BOX][4][2];
                if (find_boxes(image_copy, lu_kou_box) == 255)
                {
                    if (lu_kou_lian_xu < 250) lu_kou_lian_xu++;
                    if (lu_kou_lian_xu >= LU_KOU_QUE_REN_ZHEN)
                    {
                        lu_kou_lian_xu = 0;
                        diu_xian_zhen  = 0;                 // 清丢线状态，别让转弯后的旧记忆接管
                        diu_xian_delta = 0;
                        turn(LU_KOU_TURN_JIAO_DU);          // 阻塞：先提前量直行，再差速转弯
                        continue;                           // 本帧到此为止，下一帧从冷却直行重新开始
                    }
                }
                else
                    lu_kou_lian_xu = 0;                     // 中断连续计数，重新攒
            }
            int tmp = MT9V03X_H - 1;
            int32 x = find_line(image_copy, MT9V03X_H - 1);
            while (x == -1)
            {
                tmp--;
                if(tmp<=FIND_LINE_MIN_ROW){   // 下界由 100 改为 10：原来只回溯 19 行，弯道出口会误判丢线
                    //找线找不到
                    x=-1;
                    break;
                }
                x = find_line(image_copy, tmp);
            }
            float x1 = qing_xie_du(x, image_copy);
            /* 斜率三态（image.h / 修复版 qing_xie_du_zhuang_tai，复核报告 §4-A 确认的订正口径）
             *   0 = 斜率有效，x1 直接用；
             *   1 = 底行宽度 120~150，真实宽赛道：x1 强制归零（只关前馈，err 照旧巡线），不判丢线；
             *   2 = 爬线失败 / 底行宽于 150（白墙、低对比噪声假线 159）：必须判丢线。
             * 旧版把 1 和 2 都当 -65535 处理，宽赛道因此被整帧误杀；而审查报告建议的「删掉外层判断」
             * 又会把 159 px 假线放进来算斜率、经 ×55 前馈直接打舵。所以这里显式看状态码，
             * 不比 -65535 的哨兵值判断意图。
             * 另：x1 的量纲是「像素/行」的斜率（典型 0.1~2.0，极值约 3.7），
             * 所以 SLOPE_LIMIT 必须按斜率量纲取（3.0）-- 旧的 100 用在斜率上永远不触发，
             * 所谓「斜率越界保护」形同虚设（复核报告 P1-6）。 */
            if (qing_xie_du_zhuang_tai != 0)
                x1 = 0.0f;
            int lline = (x >> 8) & 0xFF; /* 左边界列 0~187 */
            int rline = x & 0xFF; /* 右边界列 0~187 */
            /* 反色帧兜底（审查 N-2 / P2-5）：白墙、白斑、大面积反光会把极性校正骗去整帧取反，
             * 取反后「线」其实是整幅背景，宽度顶到 188，err 反而变好看（≈0），车会顺着墙跑。
             * 所以反色帧的赛道宽度超过 FAN_SE_ZUI_DA_KUAN_DU 就按丢线处理，不去追这面墙。 */
            /* 丢线判据（修复版状态码口径）：找不到线 / 斜率状态=2（爬线失败或宽于 150）
             * / 斜率有效但越界 / 边线贴边 / 反色帧过宽。状态 1（宽赛道）不在其中 -- 它就该继续巡线。 */
            if (x < 0 || qing_xie_du_zhuang_tai == 2
                    || x1 < -SLOPE_LIMIT || x1 > SLOPE_LIMIT
                    || lline < LINE_LEFT_MIN || rline > LINE_RIGHT_MAX
                    || (image_fan_se && (rline - lline) > FAN_SE_ZUI_DA_KUAN_DU))
            {
                control(ticks, diu_xian_delta, diu_xian_jiang_su());   // 丢线记忆 + 缓降
                pid_reset();
                continue;
            }
            err = ((lline + rline) - IMAGE_CENTER);
            int delta = pid(dt_zhen) + x1 * SLOPE_WEIGHT;     // pid 带帧间隔：丢帧时 I 不累、D 按时间折算
            if (delta > DELTA_LIMIT)
                delta = DELTA_LIMIT;
            if (delta < -DELTA_LIMIT)
                delta = -DELTA_LIMIT;
            /* 落后帧降速（审查 N-2）：image_lag=1 表示这帧边线来自上一帧旧图（最多老 5 帧、约 100 ms），
             * 按原速冲这 100 ms 正好是出赛道的高发段，先按 LUO_HOU_JIANG_SU_PCT 降速换反应时间；
             * 同时不刷新丢线记忆 -- 记忆必须是「新数据」算出来的，否则丢线时接管的是过期偏置。 */
            int su_du = MAX_SPEED;
            if (image_lag)
                su_du = MAX_SPEED * (100 - LUO_HOU_JIANG_SU_PCT) / 100;
            else
                diu_xian_ji_yi((int16)delta, su_du);         // 记住这一帧，万一下一帧丢线就靠它接管
            control(ticks, (int16) (delta), su_du);
        }
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
