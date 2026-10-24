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
#define START_TIME 300000000//0.3s
int16 P,I,D,err,last_err;
int16 pid();
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
seekfree_assistant_camera_struct camera_obj;   // 逐飞助手摄像头放 CPU0 RAM
//摄像头部分暂时放在这里
uint8_t image_copy[MT9V03X_H][MT9V03X_W];              // 188×120 = 22560 字节备份缓冲区
// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    system_start(); //计时启动
    // 此处编写用户代码 例如外设初始化代码等
    zf_log(mt9v03x_init(), "mt9v03x init error");
    encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P00_7, TIM2_ENCODER_CH2_P00_8);
    pwm_init(ATOM0_CH4_P02_4, 17000, 0);// 引脚在 zf_device/pwm 里按通道映射，电机
    pwm_init(ATOM0_CH1_P33_9, 50, 750);//舵机
    gpio_init(P33_1, GPO, 0, GPO_PUSH_PULL);
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    seekfree_assistant_camera_config(
        &camera_obj,
        SEEKFREE_ASSISTANT_CAMERA_TYPE_MT9V03X,
        MT9V03X_W,              // 图像宽度，通常是 188
        MT9V03X_H,              // 图像高度，通常是 120
        image_copy              // 传入备份缓冲区地址
    );

    //引脚仅供参考
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    uint32 start_ticks = system_getval();       // ← 发车那一刻打时间戳
    uint8 if_go=0;//是否发车
    while (TRUE)//需要添加帧同步信号防止撕裂
    {
        memcpy(image_copy, mt9v03x_image, MT9V03X_IMAGE_SIZE);
        seekfree_assistant_camera_send(&camera_obj);//电脑看会掉帧
        // 此处编写需要循环执行的代码
        mt9v03x_set_exposure_time(100);//摄像头曝光时间
		//初始化结束

        uint32 ticks=system_getval()-start_ticks;//总tick
        if(ticks>START_TIME)if_go=1;
        if(if_go)ticks=START_TIME+1;
        //err需要计算
        double_value(image_copy);
        control(ticks,pid());

        // 此处编写需要循环执行的代码
    }
}
int16 pid()
{
  int16 output;
  float kp = 45, ki = 0, kd = 0;
  P = err;
  I += err;
  D = err - last_err;
  output = kp * P + ki * I + kd * D;
  last_err = err;
  return output;
}
#pragma section all restore
// **************************** 代码区域 ****************************
