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
uint8_t image_raw[MT9V03X_H][MT9V03X_W];               // 原始灰度帧快照，仅供逐飞助手图传，全程不做二值化

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
        }

        if (mt9v03x_finish_flag)                 // 只有 DMA 搬完了一整帧才进入
        {
            mt9v03x_finish_flag = 0;            // 先清标志，再拷数据
            /* 一次取原始帧，分两路用：image_raw 给逐飞助手看原始灰度图（不做任何处理），
             * image_copy 给二值化/找线用。两份内容在拷贝当时完全相同，之后互不影响。 */
            memcpy(image_raw,  mt9v03x_image, MT9V03X_IMAGE_SIZE);  // 此刻 DMA 已 disable，到下一个 VSYNC 前 mt9v03x_image 不会被写
            memcpy(image_copy, image_raw,     MT9V03X_IMAGE_SIZE);
            last_frame = now_ticks;

            double_value(image_copy);           // 只处理副本，处理多久都不会撕裂

            /* ---------------- 图传握手（审查 §9.4 第 8 条 / N-8） ----------------
             * 2026-11-10 改：逐飞助手改传「原始灰度图像」，不再传二值化后的图。
             * CPU1 现在从 image_raw 拷贝（见 cpu1_main.c），而 image_raw 只在上方 DMA 完成窗口里写入、从不被
             * double_value() / 去噪 / 找线改动，因此图传与图像处理完全解耦，上位机看到的是摄像头真实画面。
             * busy 仍由 CPU1 在 memcpy 之前置位、发完之后清，下面这段忙等盖住拷贝窗口，防止上下两帧拼接。
             * 耗时口径不变：MT9V03X 灰度与二值图都是 1 B/像素，一帧 22 560 B / 115200 8N1 ≈ 2.0 s，
             * ZU_SE_ZHEN_SHU=250 帧约 5 s 攒一次，图传约 5 s 一帧；要更实时只能提高波特率或减小帧尺寸。 */
            copy_flag++;
            if (copy_flag >= ZU_SE_ZHEN_SHU && !busy)
            {
                uint32 wait_start = system_getval();    // 忙等计时起点，单位 10 ns
                while (copy_flag >= ZU_SE_ZHEN_SHU && !busy
                        && (system_getval() - wait_start) < TU_CHUAN_CHAO_SHI)
                {
                    //wait（最多 50 ms）
                }
                if (copy_flag >= ZU_SE_ZHEN_SHU && !busy)
                    copy_flag = 0;   // 超时兜底：放弃本帧图传，避免下一帧重复等待
            }

            if (!image_valid)                   // 二值化已判不可信（连续失败/硬阈值回退），白线大概率已丢，不再拿它算 err
            {
                control(ticks, diu_xian_delta, diu_xian_jiang_su());   // 丢线记忆 + 缓降
                pid_reset();
                continue;
            }
            xiao_qu_yu_shan_chu(image_copy);
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
            int lline = (x >> 8) & 0xFF; /* 左边界列 0~187 */
            int rline = x & 0xFF; /* 右边界列 0~187 */
            /* 反色帧兜底（审查 N-2 / P2-5）：白墙、白斑、大面积反光会把极性校正骗去整帧取反，
             * 取反后「线」其实是整幅背景，宽度顶到 188，err 反而变好看（≈0），车会顺着墙跑。
             * 所以反色帧的赛道宽度超过 FAN_SE_ZUI_DA_KUAN_DU 就按丢线处理，不去追这面墙。 */
            if (x < 0 || x1 < -SLOPE_LIMIT || x1 > SLOPE_LIMIT || lline < LINE_LEFT_MIN || rline > LINE_RIGHT_MAX
                    || (image_fan_se && (rline - lline) > FAN_SE_ZUI_DA_KUAN_DU))
            {
                control(ticks, diu_xian_delta, diu_xian_jiang_su());   // 丢线记忆 + 缓降
                pid_reset();
                continue;
            }
            err = ((lline + rline) - IMAGE_CENTER);
            int delta = pid() + x1 * SLOPE_WEIGHT;
            if (delta > DELTA_LIMIT)
                delta = DELTA_LIMIT;
            if (delta < -DELTA_LIMIT)
                delta = -DELTA_LIMIT;
            /* 落后帧降速（审查 N-2）：image_lag=1 表示这帧边线来自上一帧旧图（最多老 5 帧、约 100 ms），
             * 按原速冲这 100 ms 正好是出赛道的高发段，先按 LUO_HOU_JIANG_SU_PCT 降速换反应时间；
             * 同时不刷新丢线记忆 —— 记忆必须是「新数据」算出来的，否则丢线时接管的是过期偏置。 */
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
