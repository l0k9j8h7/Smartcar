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

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

extern volatile uint8 copy_flag;
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
uint8_t image_copy[MT9V03X_H][MT9V03X_W];              // 188×120 = 22560 字节备份缓冲区
// **************************** 代码区域 ****************************
int core0_main (void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    system_start(); //计时启动
    // 此处编写用户代码 例如外设初始化代码等
    zf_log(mt9v03x_init(), "mt9v03x init error");
    //encoder_quad_init(TIM4_ENCODER,TIM4_ENCODER_CH1_P33_9, TIM4_ENCODER_CH2_P33_8);
    encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);
    pwm_init(ATOM0_CH4_P02_4, 17000, 0);    // 引脚在 zf_device/pwm 里按通道映射，电机
    pwm_init(ATOM0_CH1_P33_9, 50, SERVO_CENTER);    //舵机
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
    uint32 start_ticks = system_getval();       // ← 发车那一刻打时间戳
    uint8 if_go = 0;       //是否发车
    last_frame = start_ticks;   // last_frame 为最近一帧时间戳，初始化成发车时刻供断流保护使用
    while (TRUE)       //需要添加帧同步信号防止撕裂
    {
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
            memcpy(image_copy, mt9v03x_image, MT9V03X_IMAGE_SIZE);  // 此刻 DMA 已 disable，到下一个 VSYNC 前 mt9v03xb_image 不会被写
            //seekfree_assistant_camera_send(&camera_obj);   //发送寻找图像帧（发的是刚拷好的副本）
            copy_flag++;
            last_frame = now_ticks;

            // 图传忙等加超时：CPU1 卡在串口不置 busy 时不再永久阻塞 CPU0
            uint32 wait_start = system_getval();    // 忙等计时起点，单位 10 ns
            while (copy_flag >= ZU_SE_ZHEN_SHU && !busy
                    && (system_getval() - wait_start) < TU_CHUAN_CHAO_SHI)
            {
                //wait（最多 50 ms）
            }
            if (copy_flag >= ZU_SE_ZHEN_SHU && !busy)
                copy_flag = 0;   // 超时兜底：放弃本帧图传，避免下一帧重复等待

            double_value(image_copy);           // 只处理副本，处理多久都不会撕裂
            if (!image_valid)                   // 二值化已判不可信（连续失败/硬阈值回退），白线大概率已丢，不再拿它算 err
            {
                control(ticks, 0, MAX_SPEED / SLOW_DOWN_DIV);
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
            if (x < 0 || x1 < -SLOPE_LIMIT || x1 > SLOPE_LIMIT || lline < LINE_LEFT_MIN || rline > LINE_RIGHT_MAX)
            {
                control(ticks, 0, MAX_SPEED / SLOW_DOWN_DIV);   // 丢线时舵机回中并降速
                pid_reset();
                continue;
            }
            err = ((lline + rline) - IMAGE_CENTER);
            int delta = pid() + x1 * SLOPE_WEIGHT;
            if (delta > DELTA_LIMIT)
                delta = 150;
            if (delta < -DELTA_LIMIT)
                delta = -150;
            control(ticks, (int16) (delta), MAX_SPEED);
        }
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
