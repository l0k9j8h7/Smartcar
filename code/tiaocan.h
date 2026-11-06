/*********************************************************************************************************************
 * 文件名称          tiaocan.h
 * 作用              集中存放实车调车需要修改的可调参数，改本文件即可，不必改动各 .c
 * 编码              GBK（与工程其余源码一致，勿另存为 UTF-8）
 * 说明              原有宏名一律保持不变，只是从各 .c / .h 搬到此处；新增宏均附中文注释
 *                    本文件只定义数值常量，不含类型定义，可被任意源文件包含
 ********************************************************************************************************************/
#ifndef TIAOCAN_H
#define TIAOCAN_H

/* ========== 一、电机、发车与图传 ========== */
#define MAX_SPEED 1000              // 电机最大占空比，逐飞库限幅 0~10000
#define START_TIME 30000000         // 软启动时长 0.3 s，system_getval 单位为 10 ns
#define ZU_SE_ZHEN_SHU 250          // CPU1 每累计多少帧发送一次图传
#define SLOW_DOWN_DIV 10            // 丢线/断流保护降速除数，实际速度为 MAX_SPEED / SLOW_DOWN_DIV

/* ========== 二、舵机与方向输出 ========== */
#define SERVO_CENTER 750            // 舵机中值占空比，回中时使用
#define SERVO_MIN 400               // 舵机输出下限
#define SERVO_MAX 1100              // 舵机输出上限
#define DELTA_LIMIT 150             // 主循环 delta 钳位范围，正负 DELTA_LIMIT
#define SLOPE_WEIGHT 55             // 斜率前馈权重，delta = pid() + x1 * SLOPE_WEIGHT
#define SLOPE_LIMIT 100             // 斜率绝对值超过此值判为丢线
#define LINE_LEFT_MIN 10            // 有效左边线的最小列号
#define LINE_RIGHT_MAX 178          // 有效右边线的最大列号
#define IMAGE_CENTER 187            // 图像中心列，err = (左边界 + 右边界) - IMAGE_CENTER

/* ========== 三、PID ========== */
#define PID_KP 1.0f                 // 比例系数，调大响应变快但易过冲
#define PID_KI 0.2f                 // 积分系数，让轨迹更贴近目标
#define PID_KD 0.6f                 // 微分系数，抑制振荡，只在突变时起主要作用
#define PID_INTEGRAL_GATE 100       // 误差绝对值小于此值才允许积分
#define PID_P_LIMIT 80              // P 项限幅，正负 PID_P_LIMIT
#define PID_I_LIMIT 40              // I 项限幅，正负 PID_I_LIMIT
#define PID_D_LIMIT 50              // D 项限幅，正负 PID_D_LIMIT
#define DI_TONG_ZHI_HOU 0.55        // D 项低通系数 0~1，越大越信任本次微分
#define MAX_DELTA 200               // 保留参数，当前代码未使用，供后续扩展限幅用

/* ========== 四、安全保护（修复版新增，实车优先复调这几项） ========== */
#define TU_CHUAN_CHAO_SHI 5000000   // 图传忙等超时 50 ms，防 CPU1 卡住时 CPU0 永久死等
#define WU_ZHEN_CHAO_SHI 50000000   // 摄像头断流判定 0.5 s，超时无新帧即回中降速
#define FIND_LINE_MIN_ROW 10        // 找线上溯最低行，原下界 100 只回溯 19 行
#define MIN_TRACK_WIDTH 30          // 小于此像素宽度视为单边线，正常双边线间距约 60~110
#define FAIL_RECENT_LIMIT 25        // 最近失败窗口超过此值即判二值化不可信
#define FAIL_RECENT_STEP 2          // 每帧二值化失败的累计权重，为成功衰减的 2 倍
#define FAIL_RECENT_DECAY 1         // 每帧二值化成功的衰减量，只衰减不清零
#define FAIL_RECENT_MAX 1000        // 失败窗口累计上限，防 uint16 溢出
#define CONSIST_FAIL_MAX 5          // 允许回退上一帧旧图的最大连续失败帧数

/* ========== 五、图像处理（原有宏，值保持不变） ========== */
#define LINEWIDTH 7                 // 找线滑动窗口宽度
#define YOUXIAOXIANKUAN 5           // 判定为有效线所需的窗口内白点数和
#define SECTIONWIDTH 5              // 白段最小宽度
#define MAXSECTIONS 32              // 一行白段上限，188/(5+1) 约 31，取 32
#define ZUI_DA_GAO_DU 50            // 计算斜率时向上爬的最大行数
#define YUN_XV_SHI_BAI_CI_SHU 5     // 爬线允许的失败次数
#define MAX_LINE_WIDTH 120         // 斜率爬线赛道最大宽度；报告 P1-2 建议 160，构造图像实测低对比噪声假线宽 159，取 120 兼顾宽赛道与噪声兜底
#define MAX_BIAS 50                 // 框识别允许的最大偏移
#define MAX_BOX 5                   // 最多识别的框数量
#define MIN_BOX_HEIGHT 10           // 框的最小高度
#define ZUI_XIAO_LU_KOU_KUAN_DU 65  // 判定路口的最小宽度
#define MAX_HEIGHT 80               // 框可出现的最大像素高度
#define MIN_AREA_AVR_LEN 16         // 面积平均算法的最小面积
#define AREA_MIN_LEN 8              // 面积算法的最小处理长度

#endif /* TIAOCAN_H */
