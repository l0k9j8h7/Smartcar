#include<image.h>
#include "tiaocan.h"    // 可调参数集中管理，调车只改这个文件
#include<stdlib.h>
#include"net_data.h"
#include "zf_common_headfile.h"//头文件禁止引用头文件防止内存超限
#pragma optimize 3   // 启用最高级别优化（对应 TASKING 的 Level 3 "Optimize most"）
#pragma optimize +inline     // 自动函数内联：将短小且不常被多处调用的函数体直接嵌入调用处，减少调用开销（可能增大代码体积）
#pragma optimize +unroll     // 循环展开：自动展开小循环，减少循环控制开销（可能增大代码体积，谨慎使用）
#pragma optimize +cse        // 公共子表达式消除（CSE）：检测重复出现的相同子表达式，避免重复计算
#pragma optimize +expression // 表达式简化：移除乘以0/1、加减0等无用表达式
#pragma optimize +propagate  // 常数传播：将已知值的变量替换为该常量
#pragma optimize +flow       // 控制流简化：合并分支、删除冗余跳转、优化 switch 语句等
#pragma optimize +schedule   // 指令调度：重新排列指令顺序以更好地利用 CPU 流水线
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define da_yu(a,b) ((a) > (b) ? 1 : 0)
#define xiao_yu(a,b) ((a) < (b) ? 1 : 0)
//分成6*4个小方块
// 邻域表全文件只读（无写入、无外部引用），声明为 static const 让编译器把 9 次
// 循环里的 s1[k]/s2[k] 常量折叠成固定偏移，惠及 9 邻域去噪与两处 BFS 热循环
static const int8 s1[9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
static const int8 s2[9] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
uint8 last_image_copy[MT9V03X_H][MT9V03X_W];
static int rowmap[256], colmap[256]; //预计算映射表

uint8 last_image_exist = 0;
float now_zhi_xin_du = 0;
float zhi_xin_du = 0.6;
typedef struct
{
    int16 n;   // 订正报告 P2-2：image_init 实测值域 0~252，int8 会溢出，故收窄为 int16 而非 int8
} ding_bi_shu_256;
struct dui
{
    uint16 head;   // 队列索引只到 1023，原 uint64 让热路径做 64 位自增（报告 P2-3）
    uint16 tail;
    uint16 queue[1024]; //边界
    uint16 len;
} q = {0}; //定义1024长度队列

void push (uint16 num)
{
    if (q.len >= 1024)
        return;
    q.queue[q.tail] = num;
    q.tail++;
    q.len++;
    if (q.tail == 1024)
        q.tail = 0;

}
void pop ()
{
    if (q.len == 0)
        return;
    q.queue[q.head] = 0;
    q.head++;
    q.len--;
    if (q.head == 1024)
        q.head = 0;

}
void clear ()
{
    q.head = 0;
    q.tail = 0;
    q.len = 0;
}
int8 empty ()
{
    if (q.len == 0)
        return 1;
    else
        return 0;
}
uint16 top ()
{
    return q.queue[q.head];
}
ding_bi_shu_256 wx[188], wy[120];
uint8 kuai_kuan[188]; // 像素点左边第一个分块的列索引
uint8 kuai_gao[120];
void image_init (void)
{

    for (int i = 0; i < MT9V03X_H; i++)
    {
        int k = i;
        if (k <= 20)
            k = 20;
        else if (k >= 99)
            k = 99;
        kuai_gao[i] = ((k) * 2 - 39) / 80;
    }
    for (int i = 0; i < MT9V03X_W; i++)
    {
        int l = i;
        if (l <= 24)
            l = 24;
        else if (l >= 164)
            l = 164;
        kuai_kuan[i] = ((l) * 2 - 47) / 94;
    }
    for (int i = 0; i < MT9V03X_H; i++)
    {
        int k = i;
        if (k <= 20)
            k = 20;
        else if (k >= 99)
            k = 99;
        wy[i].n = (k * 2 - 39 - kuai_gao[k] * 80) * 128 / 40;
    }
    for (int i = 0; i < MT9V03X_W; i++)
    {

        int l = i;
        if (l <= 24)
            l = 24;
        else if (l >= 164)
            l = 164;
        wx[i].n = (l * 2 - 47 - kuai_kuan[l] * 94) * 128 / 47;
    }
}
//uint8 kuai_zuo_biao[MT9V03X_H][MT9V03X_W];
/*void cha_zhi_update (int i, int j) //0 ↖ 1 ↗ 2↙ 3 ↘
{
    if (i <= 20)
        i = 20;
    else if (i >= 99)
        i = 99;
    if (j <= 24)
        j = 24;
    else if (j >= 164)
        j = 164; //使用最外围的点求值

     for (int k = 0; k < 4; k++)
     cha_zhi[k].n = 0;
     cha_zhi[0].n = (256 - wy[i].n) * (256 - wx[j].n);            //放大65536倍
     cha_zhi[1].n = (256 - wy[i].n) * (wx[j].n);
     cha_zhi[2].n = wy[i].n * (256 - wx[j].n);
     cha_zhi[3].n = wx[j].n * wy[i].n;

}*/
static inline uint8 to_uint8 (int32 tmp)
{
    return (uint8) (max(min(tmp,255), 0));
}
int64 consist_fail = 0;
uint8 image_valid = 1;          // 本帧二值化结果是否可信；0 表示白线已丢失，主循环应走丢线保护
uint16 fail_recent = 0;         // 最近窗口内二值化失败累计：成功帧只衰减不清零，避免交替成败时永远进不了保护
int16 fen_kuai_bu_chang[3][4] = {{0}};    //65536 max
int16 da_jin_fa (int minn, int maxn, uint16 huidu[], float zong_fang_cha, float zhi_xin_du)
{   //大津法：原实现每个阈值都从头累加直方图，是 O(256^2)；改为滚动前缀和 O(256)（报告 P1-5）
    int16 best_value = -1;
    float fangcha = 0;
    uint32 total_w = 0;         // 新增：参与分类的总像素数，用于把类间方差归一化到 0~1 量级
    uint64 total_u = 0;         // 新增：总灰度和 Σi·huidu[i]，64 位防溢出
    for (int t = minn; t <= maxn; t++)
    {
        total_w += huidu[t];
        total_u += (uint64)t * huidu[t];
    }
    uint32 w0 = huidu[minn];    // 新增：滚动累加的左类像素数，避免 256 个前缀和数组占用栈（TC264 ustack0 仅 2 KB）
    uint64 u0 = (uint64)minn * huidu[minn]; // 新增：滚动累加的左类灰度和
    for (int t = minn + 1; t < maxn; t++)   // 用 int 而非 uint8，避免 maxn=255 时 t++ 溢出成 0 导致死循环
    { //防止除0
        w0 += huidu[t];
        u0 += (uint64)t * huidu[t];
        uint64_t w1 = total_w - w0;
        uint64_t u1 = total_u - u0;
        if (w0 == 0 || w1 == 0)
            continue;
        float m0 = (float)u0 / (float)w0;   // 新增：左类均值；原版 u0/u1 是均值，这里保持同一含义
        float m1 = (float)u1 / (float)w1;   // 新增：右类均值
        // 类间方差按 N^2 归一化：原版 w0*w1*(m0-m1)^2 会到 1e13，float32 只剩 5~6 位有效数字（报告 P2-7）
        float between = ((float)w0 / total_w) * ((float)w1 / total_w) * (m0 - m1) * (m0 - m1);
        if (between > fangcha)
        {
            fangcha = between;
            best_value = t;
        }
    }
    // zong_fang_cha 同步改为「总方差」（原版是 总方差×N^2），两者同除 N^2，置信度判据与 now_zhi_xin_du 保持不变
    now_zhi_xin_du = (zong_fang_cha > 0) ? (fangcha / zong_fang_cha) : 0;
    return fangcha > zong_fang_cha * zhi_xin_du ? best_value : -1; //<0.6不采纳，该方差不要除以scale
}

void double_value (uint8_t image[][MT9V03X_W])
{ //二值化

    uint16 huidu[259] = {0}; //宽/高/像素种类

    uint32 scale = MT9V03X_H * MT9V03X_W;
    uint8 minn = 255, maxn = 0;
    int32 fa_zhi = 0;
    for (int i = 0; i < MT9V03X_H; i++)
    {
        for (int j = 0; j < MT9V03X_W; j++)
        {
            huidu[image[i][j]]++;
        }

    }
    for (int i = 1; i < MT9V03X_H - 1; i++)
    {
        for (int j = 1; j < MT9V03X_W - 1; j++)
        {

            uint8 x = image[i][j];
            uint32 sum = 0;

            minn = min(minn, x);
            maxn = max(maxn, x);
            int isolated = 8;
            for (int k = 0; k < 9; k++)
            {
                if (k == 4)
                    continue;                          // 跳过中心
                uint8 v = image[i + s1[k]][j + s2[k]];
                int d = v - x;
                if (d < 0)
                    d = -d;
                if (d <= 32)
                {
                    isolated--;
                    break;
                }         // 有个邻居同色
                sum += v;

            }
            if (isolated > 7)
            {                                    // 7 邻居异 → 判孤立
                huidu[image[i][j]]--;
                image[i][j] = (uint8) (sum >> 3);                 // 填 8 邻居均值
                huidu[image[i][j]]++;
            }
        }
    }

    int32 quan_jv_an_bu = 0;
    {
        int16 tmp = 0;
        for (int t = minn; t < maxn; t++)
        {
            tmp += huidu[t];
            if (tmp > 1200 || t >= (maxn - minn) / 5 + minn)
            {
                quan_jv_an_bu = t;
                break;
            }

        }
    }
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            int16 kuai_nei_huidu[256] = {0};                 //块内灰度
            uint8 minn = 255, maxn = 0;
            for (int k1 = 0; k1 < 40; k1++)
            {
                for (int k2 = 0; k2 < 47; k2++)
                {
                    uint8 now_dot = image[i * 40 + k1][j * 47 + k2];
                    kuai_nei_huidu[now_dot]++;
                    minn = min(now_dot, minn);
                    maxn = max(now_dot, maxn);
                }
            }
            int16 tmp = 0;
            fen_kuai_bu_chang[i][j] = minn;                 //取最小值防止不进入循环（minn=maxn)
            for (int16 t = minn; t < maxn; t++)
            {
                tmp += kuai_nei_huidu[t];
                if (tmp > 100 || t >= (maxn - minn) / 5 + minn)
                {

                    fen_kuai_bu_chang[i][j] = t;
                    break;
                }

            }

        }
    }

    // 补偿前重置 min/max：把原「补偿后再扫一遍全图」合并进下面这次写回遍历，
    // 严格保持「只统计 1..H-2 / 1..W-2 内部区域」的原有口径（分段 profile 中原占 6~8%）
    minn = 255, maxn = 0;
    {
        for (int i = 0; i < MT9V03X_H; i++)
        {
            // 循环不变量外提：同一行内 kuai_gao[i]/wy[i].n 恒定，Debug -O0 下编译器不会自动外提（报告 P2-1 续）
            int16 row_up = kuai_gao[i];
            int16 row_down = row_up + 1;
            int32 wy_n = wy[i].n;                             // 新增：行插值权重，避免每像素重复查表
            const int16 *tab_up = fen_kuai_bu_chang[row_up];  // 新增：本行上侧补偿表行首指针
            const int16 *tab_down = fen_kuai_bu_chang[row_down]; // 新增：本行下侧补偿表行首指针
            uint8 *row = image[i];                            // 新增：当前行首指针，省掉每像素二维地址计算
            int inner_row = (i > 0 && i < MT9V03X_H - 1);    // 新增：本行是否属于原 min/max 统计的内部区域
            // 新增：竖线插值只依赖列块索引 c 与行权重 wy_n，与像素 j 无关；kuai_kuan 值域 0~2
            // （image_init 由 (l*2-47)/94 且 l<=164 得出），故 c+1<=3，整行只需预算 4 个值，
            // 替代每像素 4 次补偿表查表 + 2 次乘加（分段 profile 中最大热点「补偿写回全图」）
            int32_t row_rv[4];
            for (int c = 0; c < 4; c++)
                row_rv[c] = tab_up[c] + (((tab_down[c] - tab_up[c]) * wy_n) >> 8);
            for (int j = 0; j < MT9V03X_W; j++)
            {
                uint8 *now_dot = &row[j];
                uint8 old_dot = *now_dot;                     // 新增：缓存原灰度，减少一次解引用
                huidu[old_dot]--;
                int16 col_left = kuai_kuan[j];
                int32_t rvL = row_rv[col_left];
                int32_t rvR = row_rv[col_left + 1];
                int32_t val = rvL + (((rvR - rvL) * wx[j].n) >> 8);
                int32_t bu_chang = quan_jv_an_bu - val;
                bu_chang = max(-40, min(40,bu_chang));
                *now_dot = to_uint8(bu_chang + old_dot);
                huidu[*now_dot]++;
                if (inner_row && j > 0 && j < MT9V03X_W - 1)
                {
                    minn = min(minn, *now_dot);
                    maxn = max(maxn, *now_dot);
                }
            }
        }
    }
    //处理后：总方差改由直方图 O(256) 一遍算出，替代原来两遍全图扫描（每遍 22560 像素，报告 §6 #5）
    int64 sum = 0;
    int64 sum2 = 0;             // 新增：Σ(i^2·huidu[i])，与 sum 一起由直方图得出 E[x^2]
    for (int t = 0; t < 256; t++)
    {
        sum  += (int64)t * huidu[t];
        sum2 += (int64)t * t * huidu[t];
    }
    float mean = (float)sum / (float)scale;                // 新增：精确平均灰度，替代原整数 avr
    float fangcha = (float)sum2 / (float)scale - mean * mean; // 总方差 = E[x^2]-(E[x])^2；与 da_jin_fa 的类间方差同尺度
    fa_zhi = da_jin_fa(minn, maxn, huidu, fangcha, zhi_xin_du);
    if (fa_zhi >= 0)
    {
        for (int i = 0; i < MT9V03X_H; i++)
        {
            for (int j = 0; j < MT9V03X_W; j++)
            {

                image[i][j] = image[i][j] > fa_zhi ? 255 : 0;

            }
        }
        memcpy(last_image_copy, image, MT9V03X_IMAGE_SIZE); // 此刻 DMA 已 disable，到下一个 VSYNC 前 mt9v03x_image 不会被写
        last_image_exist = 1;
        consist_fail = 0;
        if (fail_recent > 0)
            fail_recent -= FAIL_RECENT_DECAY;                       // 成功帧只衰减不清零：这样「成功/失败交替」也能累计到保护阈值
        image_valid = (fail_recent < FAIL_RECENT_LIMIT) ? 1 : 0;

    }
    else
    {
        consist_fail++;
        if (fail_recent < FAIL_RECENT_MAX)
            fail_recent += FAIL_RECENT_STEP;                   // 失败权重为成功的 2 倍，让不稳定二值化尽快进保护
        if (last_image_exist == 1 && consist_fail <= CONSIST_FAIL_MAX)
        {
            memcpy(image, last_image_copy, MT9V03X_IMAGE_SIZE);
            image_valid = (fail_recent < FAIL_RECENT_LIMIT) ? 1 : 0;   // 短期回退旧图仍可用，但失败过密就不再信任
        }
        else
        {
            for (int i = 0; i < MT9V03X_H; i++)
            {
                for (int j = 0; j < MT9V03X_W; j++)
                {

                    image[i][j] = image[i][j] > (maxn + minn) / 2 ? 255 : 0;

                }
            }
            image_valid = 0;                    // 连续失败 ≥6 帧改用硬阈值，白线基本全丢，明确判不可信
        }
    }
}
void xiao_qu_yu_shan_chu (uint8_t image[][MT9V03X_W])
{
    clear();
    for (int i = 0; i < MT9V03X_H; i++)
    {
        for (int j = 0; j < MT9V03X_W; j++)
        {
            if (image[i][j] == 255) //只处理白线
            {
                int size = 0;
                push((i<<8) + j);//位压
                image[i][j] = 1;
                while (!empty())
                {
                    int32 tmp = top(); //应该是front，懒得改了
                    pop();

                    size++;
                    for (int k = 0; k < 9; k++)
                    {

                        int hang = (tmp >>8) + s1[k], lie = (tmp&0xFF)+ s2[k];
                        if (hang < 0 || lie < 0 || hang >= MT9V03X_H || lie >= MT9V03X_W)
                        {
                            continue;
                        }
                        if (image[hang][lie] == 255)
                        {
                            image[hang][lie] = 1;
                            push((hang<<8) + lie);
                        }
                    }
                }
                if (size < 60)
                {
                    //删除
                    push((i<<8) + j);
                    image[i][j] = 0;
                    while (!empty())
                    {
                        int32 tmp = top();
                        pop();

                        for (int k = 0; k < 9; k++)
                        {

                            int hang = (tmp >>8) + s1[k], lie =( tmp &0xff)+ s2[k];
                            if (hang < 0 || lie < 0 || hang >= MT9V03X_H || lie >= MT9V03X_W)
                            {
                                continue;
                            }
                            if (image[hang][lie] == 1)
                            {
                                push((hang<<8) + lie);
                                image[hang][lie] = 0; //清0
                            }

                        }
                    }

                }

            }
        }

    }

}
int16 last_l_line = -1;          // 上一帧有效左边线列，用于单边线时判断这条线属于哪一侧
int16 last_r_line = -1;          // 上一帧有效右边线列，用于单边线时判断这条线属于哪一侧
int16 last_lr_width = -1;        // 上一帧有效赛道宽度（右-左），-1 表示尚无历史，此时不猜边线
int32 find_line (uint8_t image[][MT9V03X_W], int height)
{ //返回方向值(delta1)
    int sum1 = 0, sum2 = 0, max1 = 0, max2 = 0;
    int lline = -1, rline = -1, lmax = -1, rmax = -1;
    for (int i = 0; i < LINEWIDTH; i++)
    { //滑动数组来求线,先求右边线
        int j = MT9V03X_W - i - 1;
        sum1 += image[height][i]; //最下端寻找
        sum2 += image[height][j];
    }
    for (int i = LINEWIDTH; i < MT9V03X_W; i++)
    {
        int j = MT9V03X_W - i - 1;
        sum1 -= image[height][i - LINEWIDTH];
        sum2 -= image[height][j + LINEWIDTH];
        sum1 += image[height][i];
        sum2 += image[height][j];
        if (sum1 >= YOUXIAOXIANKUAN && image[height][i] != 0)
        {
            rline = i;
        }
        if (sum1 > max1)
        {
            max1 = sum1;
            rmax = i;
        }
        if (sum2 >= YOUXIAOXIANKUAN && image[height][j] != 0)
        {
            lline = j;
        }
        if (sum2 > max2)
        {
            max2 = sum2;
            lmax = j;
        }
    }
    if (lline == -1 && rline == -1)
    {
        return (max2 != 0 && max1 != 0) ? (lmax << 8) + rmax : (-1); //<0
    }
    if (lline == -1)
        lline = max(rline-LINEWIDTH, 0);
    if (rline == -1)
        rline = min(lline+LINEWIDTH, 187);

    // 单边线时补出的「另一边」其实是同一条白线的两端，(l+r) 会被当成赛道中点两倍而放大成满舵（N-1）
    if (rline - lline < MIN_TRACK_WIDTH)
    {
        if (last_lr_width <= 0)
            return -1;                  // 没有历史赛道信息就不猜，交给主循环走丢线保护
        int center = (lline + rline) >> 1;
        if (abs(center - last_r_line) <= abs(center - last_l_line))
        {
            // 当前线更像上一帧的右边线：左边线在其左侧，宽度沿用上一帧
            rline = center;
            lline = center - last_lr_width;
        }
        else
        {
            // 当前线更像上一帧的左边线：右边线在其右侧
            lline = center;
            rline = center + last_lr_width;
        }
        lline = max(lline, 0);
        lline = min(lline, MT9V03X_W - 1);
        rline = max(rline, 0);
        rline = min(rline, MT9V03X_W - 1);
    }
    else if (height == MT9V03X_H - 1)
    {
        // 只在底行、双边线有效时刷新记忆，避免上溯行把历史宽度污染成窄线
        last_l_line = lline;
        last_r_line = rline;
        last_lr_width = rline - lline;
    }
    return (lline << 8) + rline; //返回左右的值

}
int8 find_section (uint8_t p[][MT9V03X_W], int h, uint8 s[][2])
{ //0->l 1->r,返回段的个数
    int8 sum = 0;
    uint8 flag = 0;
    for (int i = 0; i < MT9V03X_W; i++)
    {
        if(sum>=MAXSECTIONS)return MAXSECTIONS;   // 饱和时返回上限而非 0：find_boxes 不会再 continue 丢掉路口关键行
        if (p[h][i])
        {
            if (flag == 0)
            {
                flag = 1;
                s[sum][0] = i;
                //sum++;
            }
            if (i == MT9V03X_W - 1)
            {
                s[sum][1] = i;
                sum++;
            }
        }
        else
        {
            if (flag == 1)
            {
                flag = 0;
                if (i - s[sum][0] < SECTIONWIDTH)
                {
                    s[sum][0] = 0;
                }
                else
                {
                    s[sum][1] = i - 1;
                    sum++;
                }
            }
        }
    }
    return sum;
}

float qing_xie_du (int32 x, uint8_t image[][MT9V03X_W])
{
    //x为参数
    if (x < 0)
        return -65535; //失败回退
    const int32 yuanzuo = x >> 8, yuanyou = x - (yuanzuo << 8);
    int32 lline = x >> 8, rline = x - (lline << 8), lh = MT9V03X_H - 1, rh = MT9V03X_H - 1;
    int8 lfail = 0, rfail = 0;
    while (rh > MT9V03X_H - 1 - ZUI_DA_GAO_DU /*高度相同*/&& rline != MT9V03X_W - 1/*到达地图边界或向上n格*/&& rline > lline
            && lline != 0 && lfail <= YUN_XV_SHI_BAI_CI_SHU && rfail <= YUN_XV_SHI_BAI_CI_SHU
            && rline - lline <= MAX_LINE_WIDTH)
    {
        while ((rline + 1) != MT9V03X_W - 1 && image[rh][rline + 1] && rline - lline <= MAX_LINE_WIDTH)
        { //一直向右转
            rline = rline + 1;
        }
        if (image[rh - 1][rline + 1]) //右上
        {
            rh--; //从下往上面需要用减
            rline++;
        }
        else if (image[rh - 1][rline])
        {
            rh--;
        }
        else if (image[rh - 1][rline - 1])
        {
            rh--;
            rline--;
        }
        else
            rfail++, rh--;
        while ((lline - 1) != 0 && image[lh][lline - 1] && rline - lline <= MAX_LINE_WIDTH)
        {
            lline = lline - 1;
        }
        if (image[lh - 1][lline - 1]) //
        {
            lh--;
            lline--;
        }
        else if (image[lh - 1][lline])
        {
            lh--;
        }
        else if (image[lh - 1][lline + 1])
        {
            lh--;
            lline++;
        }
        else
            lfail++, lh--;
    }
    if (lline == -1 && rline != -1)
        lline = max(0, rline-LINEWIDTH);
    if (rline == -1 && lline != -1)
        rline = min(187, lline+LINEWIDTH);
    if (MT9V03X_H - 1 - lh == 0 || MT9V03X_H - 1 - rh == 0 || lline == -1 || rline == -1)
        return -65535;
    return (((float) (yuanzuo - lline) / (MT9V03X_H - 1 - lh)) + ((float) (yuanyou - rline) / (MT9V03X_H - 1 - rh))) / 2;
}
uint8 find_boxes (uint8_t image[][MT9V03X_W], uint8 box[MAX_BOX][4][2])
{ //返回框的数量，但是找到路口返回255
    uint8 s[MAXSECTIONS][2] = {{0}};
    uint8 tmp = 0;
    uint8 flag = 0;
    int prc = MT9V03X_W / 2;
    for (int i = MT9V03X_H - 1; i > MT9V03X_H - 1 - MAX_HEIGHT; i--)
    {
        if (tmp >= MAX_BOX)
            break;
        int sum = find_section(image, i, s);
        if (sum == -1 || sum == 0)
        {
            continue;
        }
        int best = -1, best_v = 0xFF, best_p = -1;
        for (int j = 0; j < sum; j++)
        {
            int16_t mid = (s[j][0] + s[j][1]) >> 1;
            if (abs(mid-prc) < best_v)
            {

                best_v = abs(mid - prc);
                best = mid;
                best_p = j;
            }
        }
        if (best_v <= MAX_BIAS && (i <= MT9V03X_H - 6 || (s[best_p][1] >= prc && s[best_p][0] <= prc)))
            prc = best;

        if (sum >= 2)
        {
            if (flag == 0)
            {
                flag = 1;
                box[tmp][0][0] = s[0][0]; //0->x,1->y
                box[tmp][0][1] = (uint8) i;
                box[tmp][1][0] = s[sum - 1][1];
                box[tmp][1][1] = (uint8) i;
                box[tmp][2][1] = (uint8) i;
                box[tmp][2][0] = s[0][0];
                box[tmp][3][1] = (uint8) i;
                box[tmp][3][0] = s[sum - 1][1];
            }
            else
            {
                box[tmp][2][0] = min(box[tmp][2][0], s[0][0]); //0->x,1->y
                box[tmp][2][1] = (uint8) i;
                box[tmp][3][0] = max(box[tmp][3][0], s[sum - 1][1]);
                box[tmp][3][1] = (uint8) i;
            }
            //image_read
        }
        else
        {
            //sum==1
            if (s[0][1] - s[0][0] >= ZUI_XIAO_LU_KOU_KUAN_DU)
                return 255; //255代表找到路口
            if (flag == 1)
            {
                flag = 0;
                if (box[tmp][0][1] - box[tmp][3][1] < MIN_BOX_HEIGHT)
                    continue;
                tmp++;
            }
            //sum==1
        }

    }
    if (flag == 1)
    {
        if (box[tmp][0][1] - box[tmp][3][1] >= MIN_BOX_HEIGHT && tmp < MAX_BOX)
            tmp++;
    }
    return tmp;
}
int16_t c1[NCH1][16][16]; //conv1结果
int16_t p1[NCH1][8][8]; //pool1结果
int16_t p2[NCH1][10][10];
int16_t c2[NCH2][8][8];

int32_t read_ele (uint8_t image[18][18])
{
    memset(c2, 0, sizeof(c2));
    memset(c1, 0, sizeof(c1));
    memset(p1, 0, sizeof(p1));
    memset(p2, 0, sizeof(p2));
    int32_t gapq[NCH2];
    conv1_int(image, c1);
    pool1_int(c1, p1);
    //只写中央8*8
    for (int i = 0; i < NCH1; i++)
    {
        for (int r = 0; r < 8; r++)
        {
            for (int c = 0; c < 8; c++)
            {
                p2[i][r + 1][c + 1] = p1[i][r][c];
//边缘全为0
            }
        }
    }
    conv2_int(p2, c2);
    gap_int(c2, gapq);
    return max_score_kind(gapq, 0);
}

uint16_t g[18][18];
uint8 box[MAX_BOX][4][2];
//#define AREA_PAD_X100     135          // 1.35
//#define AREA_VAL_SCALE  255

uint8 image_read (uint8_t image[][MT9V03X_W], uint8_t result[]) //返回box个数，result是类别
{
    memset(box, 0, sizeof(box));
    int k = (int) find_boxes(image, box);
    if (k == 255)
    {
        result[0] = 255;
        return 255;
    }
    for (int i = 0; i < MT9V03X_H; i++)
    {
        for (int j = 0; j < MT9V03X_W; j++)
        {
            if (image[i][j])
                image[i][j] = 255;
        }
    }
    uint16_t cnt[18][18]; //累加计数
    uint8_t tmp[18][18]; //临时存储
    uint32 kind = 0;
    for (int t = 0; t < k; t++)
    {
        memset(g, 0, sizeof(g));
        memset(cnt, 0, sizeof(cnt));
        memset(tmp, 0, sizeof(tmp));
        int box_left = box[t][2][0], box_right = box[t][3][0] + 1, box_top = box[t][2][1], box_bottom = box[t][0][1]
                + 1;
        int box_center_x = (box_left + box_right) >> 1, box_center_y = (box_top + box_bottom) >> 1;
        int sqr_len = max(box_right - box_left, box_bottom - box_top);
        int left = box_center_x - (sqr_len >> 1), right = box_center_x + (sqr_len >> 1), top = box_center_y
                - (sqr_len >> 1), bottom = box_center_y + (sqr_len >> 1); //窗口
        for (int i = 0; i < sqr_len; i++)
        {
            rowmap[i] = i * 16 / sqr_len;
            colmap[i] = i * 16 / sqr_len;
        }
        int win_left = box_center_x - (sqr_len >> 1), win_top = box_center_y - (sqr_len >> 1);
        int x_bias = (box_left - win_left) * 16 / sqr_len, y_bias = (box_top - win_top) * 16 / sqr_len;

        if (max(box_right-box_left,box_bottom-box_top) >= MIN_AREA_AVR_LEN)
        {

            //if (sqr_len < AREA_MIN_WIN_W) sqr_len = AREA_MIN_WIN_W;
            //int per_area = sqr_len * sqr_len;
            for (int l = 0; l < sqr_len; l++)
            {
                for (int j = 0; j < sqr_len; j++)
                {
                    int x = box_center_x - (sqr_len >> 1) + l, y = box_center_y - (sqr_len >> 1) + j;
                    if (x < box_left || x >= box_right || y < box_top || y >= box_bottom)
                        continue;
                    if (x < 0 || x >= MT9V03X_W || y < 0 || y >= MT9V03X_H)
                        continue;
                    int xs = (x - box_left) * 16 / sqr_len + x_bias, ys = (y - box_top) * 16 / sqr_len + y_bias;
                    //int xe = xs + 1, ye = ys + 1;

                    g[ys][xs] += image[y][x];
                    cnt[ys][xs]++;

                }
            }
            for (int m = 0; m < 16; m++)
            {
                for (int n = 0; n < 16; n++)
                {
                    if (cnt[m][n] != 0)
                        tmp[m + 1][n + 1] = (uint8) ((uint16) (g[m][n]) / cnt[m][n]);
                    //else tmp[m][n]=1;
                }
            }
            kind = (int) read_ele(tmp);
        }
        else if (max(box_right - box_left, box_bottom - box_top) > AREA_MIN_LEN)
        {
            int gw = (box_right - box_left - 1) * 16 / sqr_len + 1;            //缩放后的宽和高
            int gh = (box_bottom - box_top - 1) * 16 / sqr_len + 1;            //防止越界
            win_left = box_center_x - 8, win_top = box_center_y - 8;
            for (int m = 0; m < 16; m++)
            {
                for (int n = 0; n < 16; n++)
                {
                    int xs = box_left + (2 * n + 1) * (box_right - box_left) / (2 * gw);
                    int ys = box_top + (2 * m + 1) * (box_bottom - box_top) / (2 * gh);
                    tmp[m + 1][n + 1] = image[ys][xs];
                }
            }

            kind = (int) read_ele(tmp);

        }
        else
            kind = 0;
        result[t] = kind;
    }
    return k;
}

