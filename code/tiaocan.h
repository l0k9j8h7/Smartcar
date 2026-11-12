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
                                    // 注意图传本身不实时：一帧 22560 B，115200 8N1 约 2.0 s 才发完，
                                    // 250 帧（约 5 s）攒一次再花 2 s 发，等于约 5 s 一帧画面。
                                    // 想边跑边看图就接 USB/WiFi 模块改更高波特率，别靠改这个宏。
#define SLOW_DOWN_DIV 10            // 断流保护（摄像头 0.5 s 无新帧）降速除数，实际速度为 MAX_SPEED / SLOW_DOWN_DIV
                                    // 注意：单帧丢线已改为「记忆 + 缓降」，不再直接用这个除数，见第四节 DIU_XIAN_*

/* ========== 二、舵机与方向输出 ========== */
#define SERVO_CENTER 750            // 舵机中值占空比，回中时使用
#define SERVO_MIN 400               // 舵机输出下限
#define SERVO_MAX 1100              // 舵机输出上限
#define DELTA_LIMIT 150             // 主循环 delta 钳位范围，正负 DELTA_LIMIT
#define SLOPE_WEIGHT 55             // 斜率前馈权重，delta = pid() + x1 * SLOPE_WEIGHT
/* 斜率越界保护（复核报告 P1-6）：x1 的量纲是「像素/行」，典型 0.1~2.0，极值约 3.7
 * （ZUI_DA_GAO_DU=50 行内横跨 187 像素）。原值 100 用在斜率上永远不会触发，
 * 所谓「斜率越界判丢线」是死保护 -- 这是修复版漏掉的一条。
 * 现取 3.0：正常弯道都在 ±2 内，超过就说明这帧的边线已经不是赛道边（噪声假线/反色残留）。
 * 【必须上车复核】宽弯/急弯常被误判丢线，就按 0.5 一档往上放到 4.0；别回到 100。 */
#define SLOPE_LIMIT 3.0f            // 斜率绝对值超过此值判为丢线（单位：像素/行）
#define LINE_LEFT_MIN 10            // 有效左边线的最小列号
#define LINE_RIGHT_MAX 178          // 有效右边线的最大列号
#define IMAGE_CENTER 187            // 图像中心列，err = (左边界 + 右边界) - IMAGE_CENTER

/* ========== 三、PID ========== */
#define PID_KP 1.0f                 // 比例系数，调大响应变快但易过冲
#define PID_KI 0.2f                 // 积分系数，让轨迹更贴近目标
#define PID_KD 0.6f                 // 微分系数，抑制振荡，只在突变时起主要作用
/* 积分分离门限（审查 §6.1 / 订正版 §3 第 9 条）：err 有效量程是 -166~168
 * （lline>=10、rline<=178 夹出来的），原来门限写 100，弯道常见帧 |err|>=100 时 I 项
 * **完全不累积** -- 不是「贡献小」，是「不存在」，§4.2 整节论证的 I 项失效根因就在这里。
 * 现在取 187（> 168），等于取消分离、全量程积分，I 项才真正起作用。
 * 如果实车发现直道被积分带偏，先降 PID_KI，再考虑把这个门限调回 120~150。 */
#define PID_INTEGRAL_GATE 187       // 误差绝对值小于此值才允许积分
/* P 限幅 80 -> 200（审查 §6.2）：有效 err 量程 335 个整数取值，±80 限幅下约 52% 的区间里
 * P 是常数，弯道等于开环。提到 200 后线性区覆盖整个量程，饱和交给主循环的 DELTA_LIMIT(150)。 */
#define PID_P_LIMIT 200             // P 项限幅，正负 PID_P_LIMIT
#define PID_I_LIMIT 40              // I 项限幅，正负 PID_I_LIMIT
#define PID_D_LIMIT 50              // D 项限幅，正负 PID_D_LIMIT
#define DI_TONG_ZHI_HOU 0.55        // D 项低通系数 0~1，越大越信任本次微分
#define MAX_DELTA 200               // 保留参数，当前代码未使用，供后续扩展限幅用
/* 丢帧折算用的标称控制周期（复核报告 §4.2）：摄像头 50 fps = 20 ms/帧，system_getval 单位 10 ns。
 * PID_DT_MAX_10NS = 100 ms：超过它（断流、首帧）不按比例折算，直接当 1 帧处理，避免 dt 极小/极大放大噪声。 */
#define PID_DT_NOMINAL_10NS 2000000
#define PID_DT_MAX_10NS     10000000

/* ========== 四、安全保护（修复版新增，实车优先复调这几项） ========== */
#define TU_CHUAN_CHAO_SHI 500000    // 图传握手的「拷贝窗口」超时 5 ms（复核报告 §5 / §8 第 3 条）
                                    // busy 只覆盖 CPU1 的 memcpy（22 560 B，DSPR1 内本核访问约 0.1~0.2 ms），
                                    // 2026-11-12 起 cpu1_main.c 拷完即清 busy，不再覆盖后面 1.958 s 的发送，
                                    // 所以 5 ms 已是 25 倍余量。原值 50 ms 是对着「拷贝+发送」那个
                                    // 39 倍失配的旧窗口定的，已作废。
                                    // 超时语义：放弃本帧对 image_raw 的保护（最多拼一帧图传画面），不影响控制。
#define WU_ZHEN_CHAO_SHI 50000000   // 摄像头断流判定 0.5 s，超时无新帧即回中降速
#define FIND_LINE_MIN_ROW 10        // 找线上溯最低行，原下界 100 只回溯 19 行
#define MIN_TRACK_WIDTH 30          // 小于此像素宽度视为单边线，正常双边线间距约 60~110
#define FAIL_RECENT_LIMIT 25        // 最近失败窗口超过此值即判二值化不可信
#define FAIL_RECENT_STEP 2          // 每帧二值化失败的累计权重，为成功衰减的 2 倍
#define FAIL_RECENT_DECAY 1         // 每帧二值化成功的衰减量，只衰减不清零
#define FAIL_RECENT_MAX 1000        // 失败窗口累计上限，防 uint16 溢出
#define CONSIST_FAIL_MAX 5          // 允许回退上一帧旧图的最大连续失败帧数

/* --- 二值化可信度与极性（2026-11-09 对抗审查 N-2 / N-3 / P2-5） --- */
#define ZHI_XIN_DU 0.45f            // 大津法置信度门限 = 类间方差 / 总方差。原 0.6 对正常赛道（白线占 10%）偏高，
                                    // 会把好帧判成失败、退化成回退图或硬阈值（审查 N-3）。误判丢线多就再降 0.05，
                                    // 噪声帧被当有效帧（err 乱跳）就往上加。
#define ZUI_XIAO_DUI_BI_DU 20       // 最小绝对对比度 maxn-minn。低于此值判本帧没有有效黑白分界，
                                    // 不管置信度多高都不采纳：全黑/全白/低对比帧的类间方差仍可能很高，光看置信度拦不住。
#define FAN_SE_BAI_BI 60            // 白点占比超过此百分比才认定极性反了并整帧取反。原 50% 门限在
                                    // 白墙、白斑、大面积反光下会误反色，反色后「背景」被当成线，打舵直接反（审查 P2-5）。
#define FAN_SE_ZUI_DA_KUAN_DU 150   // 反色帧允许的最大双边线宽度（像素）。白墙误反色时「线」= 整幅背景，
                                    // 宽度会顶到 187，主循环据此判丢线，不去追这面墙。
#define LUO_HOU_JIANG_SU_PCT 25     // 回退旧图（image_lag=1）时的速度下调百分比：图像最多老 5 帧（约 100 ms），
                                    // 按原速冲这一段最容易出赛道，先降速换时间。

/* --- 丢线记忆 + 缓降（报告 §5.1） --- */
#define DIU_XIAN_JIAN_SU 4          // 丢线后每帧速度衰减百分比（%）。一帧约 20 ms，4% 约 0.5 s 掉到一半，
                                    // 丢线后还想多冲一点就调小（2~3），想快点贴地就调大（6~8）
#define DIU_XIAN_ZUI_DI_SU 150      // 缓降的占空比下限，别让车"停在线外"彻底不动
#define DIU_XIAN_ZUI_DA_ZHEN 15     // 连续丢线超过这么多帧（约 0.3 s）仍找不到线，就加大打舵再搏一次
#define DIU_XIAN_MANG_QU_BI 20      // 上面那种情况每帧额外加多少偏置，累加到 DELTA_LIMIT 为止，0 = 不加
/* 丢线超时停车（审查 §9 第 6 条的安全项）：缓降最低只到 DIU_XIAN_ZUI_DI_SU(150)，
 * 一直找不到线就会以 15% 占空比永远往前蹭，撞墙/冲出场地都不停。这里补一道硬保护：
 * 连续丢线超过此帧数直接给 0 占空比（舵机仍按记忆偏置，不突然回中）。
 * 一帧约 20 ms：150 帧 ≈ 3 s。嫌太容易停就加大（200~250），要更快停下就减小（80~100）。
 * 设为 0 = 关闭本保护，回到"永远缓降"。 */
#define DIU_XIAN_TING_CHE_ZHEN 150

/* ========== 五、图像处理（原有宏，值保持不变） ========== */
#define LINEWIDTH 7                 // 找线滑动窗口宽度
#define YOUXIAOXIANKUAN 5           // 判定为有效线所需的窗口内白点数和
#define SECTIONWIDTH 5              // 白段最小宽度
#define MAXSECTIONS 32              // 一行白段上限，188/(5+1) 约 31，取 32
#define ZUI_DA_GAO_DU 50            // 计算斜率时向上爬的最大行数
#define YUN_XV_SHI_BAI_CI_SHU 5     // 爬线允许的失败次数
#define BFS_QUEUE_LEN 1024          // 连通域 BFS 队列长度（像素个数，uint16 索引 -> 2 KB）。
                                    // 实车裁决 2026-11-12：1024 已够用，按实车连通块大小取 1024，省 6 KB DSPR0（审查曾建议扩 4096）。
#define MAX_LINE_WIDTH 120          // 斜率爬线时允许的赛道宽度（像素）：爬线过程中不许越爬越宽。
                                    // 构造图像实测低对比噪声假线宽 159，这个值压小是有意的。
                                    // 它只管「爬线」，不再决定整帧是否丢线（见下面 XUN_XIAN_ZUI_DA_KUAN_DU）。
#define XUN_XIAN_ZUI_DA_KUAN_DU 150 // 巡线可接受的最大底行宽度（审查 P0-1 订正版 / 复核 §4-A）。
                                    // 底行宽于它 = 宽到不像赛道（白墙、噪声假线 159），按丢线处理；
                                    // 120~150 之间 = 真实宽赛道（底行常在 110~140）：斜率不算，
                                    // err 照常巡线，不再整帧误杀。
                                    // 上限必须 <= 158 -- 取 160 就把 159 的假线放行了。
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
/* 2026-11-09 按实车接线图重排 P02 口：P02.4/P02.5 两路 PWM，P02.6/P02.7 两路方向。
 * 与上一版（P02.4 PWM + P02.5 DIR + P02.6 PWM + P02.7 DIR）是 P02.5 与 P02.6 互换角色，
 * 物理上两种接法都成立（P02_4~P02_7 既能做 ATOM0_CH4~CH7 输出，也能当普通 GPIO），
 * 差别只在「哪根线是 PWM、哪根线是方向」。所以改这两行必须和实际接线一致：
 *   - 如果板子已经按上一版焊好（P02.5 接 IN2、P02.6 接 PWM），把下面两行的 P02_5/P02_6 换回去；
 *   - 如果按新图接（P02.5 接 PWM、P02.6 接 IN2），保持现在的写法。
 * 接错的典型现象：某个轮子完全不动（PWM 打在方向脚上，占空比只会在 0/满之间跳），
 * 或者轮子只会一个方向猛转（方向脚被当成 PWM 拉动）。 */
#define TURN_PWM_ZUO            ATOM0_CH4_P02_4     // 左电机 PWM：P02.4（ATOM0_CH4）
#define TURN_PWM_YOU            ATOM0_CH5_P02_5     // 右电机 PWM：P02.5（ATOM0_CH5）
                                                    // 两路同在 ATOM0，CH4/CH5 还同属 AGC1 一组，
                                                    // 必须同频（MOTOR_PWM_PINLV 两路共用一个宏）；不同频会互相改周期
#define TURN_DIR_ZUO            P02_6               // 左电机方向：P02.6（DRV8701E 的 IN2，普通 GPIO）
#define TURN_DIR_YOU            P02_7               // 右电机方向：P02.7（DRV8701E 的 IN2，普通 GPIO）
#define TURN_ENCODER_ZUO        TIM4_ENCODER        // 左轮编码器 TIM4: P02_8(A)/P00_9(B)
                                                    // B 相两个可选：P00_9 = T4EUDA，P33_5 = T4EUDB，
                                                    // 功能等价，当前用 P00_9（与摄像头 P00_0~P00_7 不冲突）。
                                                    // 改 B 相要同时改 motor.c 里 encoder_quad_init 那行，两处必须一致
                                                    // 原来是 TIM5(P21_7/P21_6)：T5 属 GPT2，没有增量接口模式，
                                                    // encoder_quad_init 的 zf_assert 会直接死等，见 motor.c 注释
#define TURN_ENCODER_YOU        TIM2_ENCODER        // 右轮编码器 TIM2: P33_7(A)/P33_6(B)
                                                    // ★左右轮编码器极性必须一致（审查 N-10）：TIM4 与 TIM2 的计数方向
                                                    // 取决于 A/B 相接线顺序，接反了一个轮「前进」是负脉冲。
                                                    // 标定：把车推着往前走，看 motor_speed_pulse(0)/motor_speed_pulse(1)
                                                    // 是否同号；一正一负就把那一轮的 A/B 两根线对调（软件改计数符号也行，
                                                    // 但要同时在 motor.c 和 turn() 里改，别只改一处）。
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
/* 注意：2026-11-09 修正了差速环的量纲（审查 P0-1）：原来 xi_wang_cha = 每毫米脉冲 × 轮距，
 * 少了「度→弧度」换算，期望值被放大 180/π ≈ 57 倍，PI 长期顶在饱和值上，看着能转其实是恒定偏置。
 * 修正后误差单位统一成「每度脉冲差」（典型量级 145），上面两个增益保持原值即可有相近权感，
 * 但首次上车必须重新整定：先 KP 单独加到「外轮明显比内轮快」不抖，再加 KI。 */
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
                                        // 注意：本参数目前【不生效】：turn() 只在 TURN_TAI_JIAN_CE_SHI 台架自检里被调用，
                                        // 生产构建没有调用点（审查 N-9）。等把 turn() 真正接进巡线流程，它才开始起作用。

/* --- 6.5b 路口触发转弯接入主循环（2026-11-12 新增） ---
 * cpu0_main.c 每帧用 find_boxes() 做路口检测，连续 LU_KOU_QUE_REN_ZHEN 帧确认后调用
 * turn(LU_KOU_TURN_JIAO_DU)，转弯点由 6.6 节的 TURN_TI_QIAN_MM 控制在路口之前。 */
#define LU_KOU_TURN_JIAO_DU     90      // 识别到路口后执行的转角（度）：正=右转，负=左转。
                                        // 0 = 关闭（主循环连路口检测都不跑，回到纯巡线）。
                                        // 常用：90 直角、120/135 大弯、180 掉头；符号按赛道实际走向改
#define LU_KOU_QUE_REN_ZHEN     2       // 连续多少帧识别到路口才算确认（防单帧噪声误触发）；1 = 单帧即触发


/* --- 6.6 提前路口转弯（2026-11-12 新增） ---
 * turn() 被触发（路口识别）后不再在识别点原地转弯，而是先直行 TURN_TI_QIAN_MM 毫米，
 * 把转弯起始点前移到路口前这一段距离处。三个量都要实测标定：
 * 先打印「识别到路口那一刻车头离路口的实际距离」，再据此定 TURN_TI_QIAN_MM。 */
#define TURN_TI_QIAN_MM           200   // 触发转弯后继续直行的距离（mm）= 转弯起始点距路口的提前量。
                                        // 0 = 关闭提前量，识别点即转弯点（与旧行为一致）。
                                        // 数值不能超过摄像头能识别到路口的最大距离，否则没意义
#define TURN_TI_QIAN_SU_DU        500   // 提前量直行段占空比（0~10000），低于巡线速度，直行求稳
#define TURN_TI_QIAN_CHAO_SHI_MS  3000  // 直行段超时（ms）：编码器失效/堵转兜底，超时后照常进入转弯


/* 台架验收三步
 * ① 两轮都悬空，turn(90)：左右轮都朝同一方向、外轮明显更快；看 turn_shi_ji_ban_jing 是否接近 L/tan(d)。
 * ② 推车走直线 10 圈：编码器脉冲 / 10 = 每轮圈脉冲，填回 TURN_MEI_LUN_QUAN_CHONG_MEI_DU（左右轮分别量，取平均）。
 * ③ 地面画圆跑 turn(90)，量实际角度 X：如果半径对但角度不对，改的是「每圈脉冲」；半径不对改 TURN_ZUI_DA_ZHUAN_JIAO_DU。 */


/* ========== 七、双电机驱动层（motor.c，2026-11-07） ========== */
/* 这一节只管"两轮各给多少占空比"。巡线在 control.c、转弯在 turn()，两边都经 drive() 输出，
 * 不要再直接写电机 PWM，否则左右轮又会各写各的。 */

#define MOTOR_PWM_MAX           10000   // 单轮占空比硬限幅，逐飞库量程 0~10000
#define MOTOR_PWM_PINLV         17000   // 两路电机 PWM 频率，必须同频，不同频低速段两轮响应差很多
#define MOTOR_DIR_ZUO_LEVEL     0       // 左电机方向脚（P02.6 / DRV8701E IN2）前进电平，实车反了改 1
#define MOTOR_DIR_YOU_LEVEL     0       // 右电机方向脚（P02.7 / IN2）前进电平，实车反了改 1
#define MOTOR_CE_SU_WIN         1000000 // 测速窗口 10 ms（1 ms = 100000，system_getval 单位 10 ns）

/* 巡线差速：默认关闭，两轮同占空比。
 * 打开后按舵机偏置给内轮减速，减少阿克曼转弯时内外轮的拖滑；系数别大，先从 15~25 试。
 * 方向必须实车确认：若打舵与减速轮对不上（车画龙），把 MOTOR_DIFF_FANG_XIANG 改成 -1。 */
#define MOTOR_DIFF_ENABLE       0       // 1 = 开启巡线差速
#define MOTOR_DIFF_NEI_XISHU    25      // 内轮最多减 25%（百分比）
#define MOTOR_DIFF_FANG_XIANG   1       // +1 = delta 为正时右轮是内轮；实车相反改 -1

/* ========== 八、速度闭环（2026-11-10，按审查 §9 第 6 条新增） ========== */
/* 现状：control() 给出的是**开环占空比**，电池掉压、左右电机个体差异都会让实际轮速跑偏，
 * 而 motor.c 里 motor_speed_pulse() 早就在算了（cpu0_main.c 每帧调 motor_speed_update），
 * 只是没人读 -- 这一节就是把这条反馈接上。
 * 结构：前馈仍是巡线算出来的占空比，PI 只修正「实际轮速 - 目标轮速」的偏差，输出叠加在前馈上。
 * ★ 默认关闭。SPEED_LOOP_PULSE_PER_DUTY 是必须实测的标定量，拍一个数比开环更危险。
 *   标定三步：
 *     ① 车架起来（两轮悬空），SPEED_LOOP_ENABLE 置 1、KP/KI 都先置 0；
 *     ② 给一个固定占空比（比如 MAX_SPEED）跑 2 s 以上，读 motor_speed_pulse(0)/(1)；
 *     ③ 每占空比脉冲 = 读到的脉冲/秒 × 1000 / 当时占空比，左右轮各填一次取较小值。
 *        左右差得远说明机械阻力不同，先查装配，别用闭环硬补。
 *   标定完再放开 KP（从小往大，到"跟得上且不抖"），最后加 KI 消稳态误差。 */
#define SPEED_LOOP_ENABLE 0          // 1 = 开启速度闭环；没有标定之前保持 0
#define SPEED_LOOP_PULSE_PER_DUTY 0  // 每 1000 占空比对应的轮速（脉冲/秒）。★必须实测；0 = 未标定，自动退化为开环
                                     // 典型量级：满占空比空载约 5 000~15 000 脉冲/秒，具体看电机和编码器线数
#define SPEED_LOOP_KP 10             // 比例：偏差 1000 脉冲/秒 -> 修正 100 占空比（KP/100）
#define SPEED_LOOP_KI 20             // 积分：每帧累加 偏差×KI/10000，饱和到 ±SPEED_LOOP_I_LIMIT
#define SPEED_LOOP_I_LIMIT 300       // 积分限幅（占空比），防长时间堵转把积分憋死
#define SPEED_LOOP_ZUI_DI_DUTY 250   // 目标占空比低于此值不闭环：起步/停车段轮速反馈不可信，直接开环
#define SPEED_LOOP_CHA_SHANG_XIAN 400 // 单帧修正量上限（占空比），防一次跳变把车甩出去

#endif /* TIAOCAN_H */
