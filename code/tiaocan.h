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
#define SLOW_DOWN_DIV 10            // 断流保护（摄像头 0.5 s 无新帧）降速除数，实际速度为 MAX_SPEED / SLOW_DOWN_DIV
                                    // 注意：单帧丢线已改为「记忆 + 缓降」，不再直接用这个除数，见第四节 DIU_XIAN_*

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

/* --- 丢线记忆 + 缓降（报告 §5.1） --- */
#define DIU_XIAN_JIAN_SU 4          // 丢线后每帧速度衰减百分比（%）。一帧约 20 ms，4% 约 0.5 s 掉到一半，
                                    // 丢线后还想多冲一点就调小（2~3），想快点贴地就调大（6~8）
#define DIU_XIAN_ZUI_DI_SU 150      // 缓降的占空比下限，别让车"停在线外"彻底不动
#define DIU_XIAN_ZUI_DA_ZHEN 15     // 连续丢线超过这么多帧（约 0.3 s）仍找不到线，就加大打舵再搏一次
#define DIU_XIAN_MANG_QU_BI 20      // 上面那种情况每帧额外加多少偏置，累加到 DELTA_LIMIT 为止，0 = 不加

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


/* ========== 六、编码器差速转弯（turn 函数，2026-11-06） ========== */
/* 思路：前轮舵机定圆心 + 后轮左右电机差速补半径。
 * 标定拆成四个能直接量的长度/计数，先量再跑，别用拍的数。 */

/* --- 6.1 硬件 --- */
#define TURN_PWM_ZUO            ATOM0_CH4_P02_4     // 左电机 PWM：P02.4（ATOM0_CH4）
#define TURN_PWM_YOU            ATOM0_CH6_P02_6     // 右电机 PWM：P02.6（ATOM0_CH6）
#define TURN_DIR_ZUO            P02_5               // 左电机方向：P02.5（DRV8701E 的 IN2，普通 GPIO）
#define TURN_DIR_YOU            P02_7               // 右电机方向：P02.7（DRV8701E 的 IN2，普通 GPIO）
#define TURN_ENCODER_ZUO        TIM5_ENCODER        // 左轮编码器 TIM5: P21_7(A)/P21_6(B)
#define TURN_ENCODER_YOU        TIM2_ENCODER        // 右轮编码器 TIM2: P33_7(A)/P33_6(B)
#define TURN_FANG_XIANG         1                   // 转向极性：+1 = 正角度右转；实车相反改 -1
#define TURN_STEER_DUTY         250                 // 舵机相对 SERVO_CENTER 的偏置，配合 TURN_ZUI_DA_ZHUAN_JIAO_DU 一起调
                                                    // 注意两套舵机坐标系：巡线 delta 限幅 ±DELTA_LIMIT(150)，转弯这里 ±250。
                                                    // 250 对应 TURN_ZUI_DA_ZHUAN_JIAO_DU(28°)，改其中任何一个都要同步另一个，
                                                    // 否则会出现「标称 28° 实际 34°」，半径算不对、转弯角度跟着偏。

/* --- 6.2 车体几何与编码器（★必须实测，尺子量、转一圈数） --- */
#define TURN_AXIS_MM            200                 // 轴距：前轴到后轴，mm
#define TURN_LUN_JU_MM          110                 // 轮距：左右后轮中心距，mm
#define TURN_ZUI_DA_ZHUAN_JIAO_DU 28                // 前轮最大可用转角（度），决定最小半径 R = L/tan(d)
#define TURN_LUN_ZHOU_MM        204                 // 轮周长 mm = π × 轮径；用卷尺绕轮胎一圈量最准
#define TURN_MEI_LUN_QUAN_CHONG_MEI_DU 15360        // 每轮圈脉冲 = 编码器单圈计数 × 减速比；拿车推 10 圈数脉冲除以 10 最准

/* --- 6.3 速度与限幅 --- */
#define TURN_ZHUAN_SU_DU        700     // 外轮基准占空比（0~10000）
#define TURN_NEI_LUN_ZUI_DI     120     // 内轮占空比下限，太小等于给内轮刹手刹
#define TURN_ZUI_DA_ZHAN_KONG   2000    // 差速调节后的占空比上限，防 PI 跑飞
/* 下面两个半径限幅【当前不生效】：半径由舵机角度决定（R = L/tan(d)），
 * 强行钳位会让目标弧长和实际弧长对不上，出现"提前断电、转不够"。保留仅为调试时手动参考。 */
#define TURN_BAN_JING_MIN       150     // 参考值：半径下限 mm，再小就打滑、磨胎（未参与运算）
#define TURN_BAN_JING_MAX       400     // 参考值：半径上限 mm（未参与运算）
#define TURN_TI_QIAN_JIAO_DU    3       // 提前断电量（度）。实测转不到位就减小，冲过头就加大
#define TURN_ZUI_XIAO_JIAO_DU   1       // 小于此角度直接不动作（度）
/* 单次转角量程（度，取绝对值）。360 = 支持整圈，120/300 这类大角度直接 turn(120) / turn(300)。
 * 想原地连转两圈就把这里改到 720；再大要注意 TURN_CHAO_SHI_MS 的总时长别超过 42 s
 * （system_getval 是 32 位 10 ns 计数，约 42.9 s 就回绕一次）。 */
#define TURN_ZUI_DA_JIAO_DU     360

/* --- 6.4 差速闭环 --- */
#define TURN_CE_SU_WIN_MS       30      // 差速检测窗口（ms），太小噪声大、太大响应慢
#define TURN_DU_ZHUAN_ZHI       3       // 窗口内两轮平均脉冲低于此值判堵转
#define TURN_CHA_PI_KP          3       // 弧度差环比例：每差 1 个「每度脉冲」给几档占空比
#define TURN_CHA_PI_KI          1       // 弧度差环积分
#define TURN_CHA_PI_JI_FEN_SHANG_XIAN 400   // 积分限幅（单位：每度脉冲）

/* --- 6.5 时序与保护 --- */
#define TURN_DA_DUO_MS          200     // 打舵后等前轮到位（ms）
#define TURN_TING_CHE_MS        250     // 断电后等车身滑停（ms）
#define TURN_HUA_XING_MS        0       // 到点后继续给电的时间（ms）。默认 0 = 立刻断电；
                                        // 车轻、冲过头再试 80~200，车重、转不到位就保持 0
#define TURN_CHAO_SHI_MS        8000    // 单次转弯基准超时 ms（打舵+起停+小角度足够）
#define TURN_CHAO_SHI_MEI_DU_MS 40      // 每度追加的超时余量 ms。大角度按比例加：
                                        // 实际超时 = TURN_CHAO_SHI_MS + 本宏 × 角度
                                        // 300 度 -> 8 + 40×300/1000 = 20 s；总时长务必 < 42 s
#define TURN_BU_CHONG_SHANG_XIAN 2000   // 单次采样脉冲上限，超过视为野值丢弃
#define TURN_TAI_JIAN_CE_SHI    0       // 1 = 上电自检跑一次 turn(90)，仅台架标定用
#define TURN_LENG_QUE_ZHEN      18      // 转弯收尾后的舵机冷却帧数，一帧约 20 ms，18 帧≈0.36 s。
                                        // 期间 control() 强制回中直行，防止旧 err 把刚转过来的车头掰回去。
                                        // 转完还需要往前抢一段就把值调大（25~30），转完立刻要进弯就调小（8~10）。

/* 台架验收三步
 * ① 两轮都悬空，turn(90)：左右轮都朝同一方向、外轮明显更快；看 turn_shi_ji_ban_jing 是否接近 L/tan(d)。
 * ② 推车走直线 10 圈：编码器脉冲 / 10 = 每轮圈脉冲，填回 TURN_MEI_LUN_QUAN_CHONG_MEI_DU（左右轮分别量，取平均）。
 * ③ 地面画圆跑 turn(90)，量实际角度 X：如果半径对但角度不对，改的是「每圈脉冲」；半径不对改 TURN_ZUI_DA_ZHUAN_JIAO_DU。 */


/* ========== 七、双电机驱动层（motor.c，2026-11-07） ========== */
/* 这一节只管"两轮各给多少占空比"。巡线在 control.c、转弯在 turn()，两边都经 drive() 输出，
 * 不要再直接写电机 PWM，否则左右轮又会各写各的。 */

#define MOTOR_PWM_MAX           10000   // 单轮占空比硬限幅，逐飞库量程 0~10000
#define MOTOR_PWM_PINLV         17000   // 两路电机 PWM 频率，必须同频，不同频低速段两轮响应差很多
#define MOTOR_DIR_ZUO_LEVEL     0       // 左电机方向脚（P02.5 / DRV8701E IN2）前进电平，实车反了改 1
#define MOTOR_DIR_YOU_LEVEL     0       // 右电机方向脚（P02.7 / IN2）前进电平，实车反了改 1
#define MOTOR_CE_SU_WIN         1000000 // 测速窗口 10 ms（1 ms = 100000，system_getval 单位 10 ns）

/* 巡线差速：默认关闭，两轮同占空比。
 * 打开后按舵机偏置给内轮减速，减少阿克曼转弯时内外轮的拖滑；系数别大，先从 15~25 试。
 * 方向必须实车确认：若打舵与减速轮对不上（车画龙），把 MOTOR_DIFF_FANG_XIANG 改成 -1。 */
#define MOTOR_DIFF_ENABLE       0       // 1 = 开启巡线差速
#define MOTOR_DIFF_NEI_XISHU    25      // 内轮最多减 25%（百分比）
#define MOTOR_DIFF_FANG_XIANG   1       // +1 = delta 为正时右轮是内轮；实车相反改 -1

#endif /* TIAOCAN_H */
