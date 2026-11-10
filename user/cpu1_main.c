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
 * 文件名称          cpu1_main
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

#include "zf_common_headfile.h"
#include "control.h"
/* camera_obj 定义在 cpu0_main.c 里，这里只声明不定义，告诉编译器它存在于别的文件 */

extern seekfree_assistant_camera_struct camera_obj;
#pragma section all "cpu1_dsram"
volatile uint8 busy=0;
/* copy_flag 由 CPU0 每帧自增、CPU1 发送后清零。原来声成 uint8，靠 250→255 溢出这种副作用
 * 当作"别再加了"的兜底，跨核语义太脆（251e810 报告 §5.2）。改 uint16 后不用再靠溢出，
 * CPU0 侧的忙等超时（TU_CHUAN_CHAO_SHI）才是唯一兜底。 */
volatile uint16 copy_flag=0;

extern uint8_t image_raw[MT9V03X_H][MT9V03X_W];    // CPU0 侧原始灰度帧，图传专用（未二值化）
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// **************************** 代码区域 ****************************

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
uint8_t copy[MT9V03X_H][MT9V03X_W];        // 逐飞助手图传发送缓冲，内容 = 原始灰度图像
void core1_main (void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
        /* 原来这里是「外层 while + 内层 while」，内层没有 break，外层实际只跑一轮，
         * 属于无意义的多层结构（251e810 报告 P1-7）。合成单层，逻辑完全等价。 */
        if (copy_flag>=ZU_SE_ZHEN_SHU)
        {           // 由 CPU0 置位的"可发送"标志
            /* 2026-11-10 改：图传改传原始灰度图，数据源从 image_copy（二值化后）换成 image_raw（原始帧）。
             * busy 仍提到 memcpy 之前（审查 N-8 / §9.4 第 8 条）：覆盖「拷贝 + 发送」全程，
             * 防止 CPU0 在拷贝窗口里刷新 image_raw 而拼出两帧。copy_flag 在拷贝完成后清零，
             * 下一次要等新的 250 帧。发送缓冲 copy 在 CPU1 本地 RAM，发送期间不会被改写。 */
            busy=1;
            memcpy(copy, image_raw, MT9V03X_IMAGE_SIZE);     // 原始灰度帧，不是二值化图

            copy_flag=0;
            seekfree_assistant_camera_send(&camera_obj);
            busy=0;
        }
    }
}
#pragma section all restore
// **************************** 代码区域 ****************************
