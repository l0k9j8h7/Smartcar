#include<image.h>
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
int8 s1[9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
int8 s2[9] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
uint8 last_image_copy[MT9V03X_H][MT9V03X_W];
static int rowmap[256], colmap[256]; //预计算映射表

uint8 last_image_exist = 0;
float now_zhi_xin_du = 0;
float zhi_xin_du = 0.6;
int16 row_up, row_down, col_left, col_right;
typedef struct
{
    int32 n;
} ding_bi_shu_256;
struct dui
{
    uint64 head;
    uint64 tail;
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
void cha_zhi_update (int i, int j) //0 ↖ 1 ↗ 2↙ 3 ↘
{
    if (i <= 20)
        i = 20;
    else if (i >= 99)
        i = 99;
    if (j <= 24)
        j = 24;
    else if (j >= 164)
        j = 164; //使用最外围的点求值
    /*
     for (int k = 0; k < 4; k++)
     cha_zhi[k].n = 0;
     cha_zhi[0].n = (256 - wy[i].n) * (256 - wx[j].n);            //放大65536倍
     cha_zhi[1].n = (256 - wy[i].n) * (wx[j].n);
     cha_zhi[2].n = wy[i].n * (256 - wx[j].n);
     cha_zhi[3].n = wx[j].n * wy[i].n;
     */
}
static inline uint8 to_uint8 (int32 tmp)
{
    return (uint8) (max(min(tmp,255), 0));
}
int64 consist_fail = 0;
int16 fen_kuai_bu_chang[3][4] = {{0}};    //65536 max
int16 da_jin_fa (int minn, int maxn, uint16 huidu[], float zong_fang_cha, float zhi_xin_du)
{
    int16 best_value = -1;
    float fangcha = 0;
    //大津法
    for (uint8 t = minn + 1; t < maxn; t++)
    { //防止除0
        uint64_t w0 = 0, w1 = 0;
        float u0 = 0, u1 = 0;
        for (int i = minn; i <= t; i++)
        {
            w0 += huidu[i];
            u0 += i * huidu[i];
        }
        for (int i = t + 1; i <= maxn; i++)
        {
            w1 += huidu[i];
            u1 += i * huidu[i];

        }
        u0 /= w0; //除以类内元素个数
        u1 /= w1; //同理
        if (((uint64) (w0 * w1)) * (u0 - u1) * (u0 - u1) > fangcha)
        {
            fangcha = ((uint64) (w0 * w1)) * (u0 - u1) * (u0 - u1);
            best_value = t;
        }

    }
    now_zhi_xin_du = fangcha / zong_fang_cha;
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

    {

        for (int i = 0; i < MT9V03X_H; i++)
        {

            for (int j = 0; j < MT9V03X_W; j++)
            {
                int k = i, l = j;
                uint8 *now_dot = &image[i][j];
                huidu[*now_dot]--;
                if (k <= 20)
                    k = 20;
                else if (k >= 99)
                    k = 99;
                if (l <= 24)
                    l = 24;
                else if (l >= 164)
                    l = 164;
                row_up = kuai_gao[i];
                row_down = row_up + 1;
                col_left = kuai_kuan[j];
                col_right = col_left + 1;
                cha_zhi_update(k, l);
                const int32_t A = fen_kuai_bu_chang[row_up][col_left];
                const int32_t B = fen_kuai_bu_chang[row_down][col_left];
                const int32_t C = fen_kuai_bu_chang[row_up][col_right];
                const int32_t D = fen_kuai_bu_chang[row_down][col_right];
                int32_t rvL = A + (((B - A) * wy[i].n) >> 8);          // 左边竖线上的点
                int32_t rvR = C + (((D - C) * wy[i].n) >> 8);          // 右边竖线上的点
                int32_t val = rvL + (((rvR - rvL) * wx[j].n) >> 8);    // 最终值

                int32_t bu_chang = quan_jv_an_bu - val;
                /* ((cha_zhi[0].n * fen_kuai_bu_chang[row_up][col_left]
                 + cha_zhi[1].n * fen_kuai_bu_chang[row_up][col_right]
                 + cha_zhi[2].n * fen_kuai_bu_chang[row_down][col_left]
                 + cha_zhi[3].n * fen_kuai_bu_chang[row_down][col_right]) >> 16);//计算时用了wx,wy 256定比数，需要除以65536
                 */
                bu_chang = max(-40, min(40,bu_chang));
                *now_dot = to_uint8(bu_chang + *now_dot);                 //地址
                huidu[*now_dot]++;
            }
        }
    }
    minn = 255, maxn = 0;                 //重新统计min,max
    for (int i = 1; i < MT9V03X_H - 1; i++)
    {
        for (int j = 1; j < MT9V03X_W - 1; j++)
        {

            uint8 x = image[i][j];
            minn = min(minn, x);
            maxn = max(maxn, x);
        }
    }
    //处理后
    int64 sum = 0;
    for (int i = 0; i < MT9V03X_H; i++)
    {
        for (int j = 0; j < MT9V03X_W; j++)
        {
            sum += image[i][j];

        }
    }
    int64 avr = sum / scale;
    sum = 0;
    for (int i = 0; i < MT9V03X_H; i++)
    {
        for (int j = 0; j < MT9V03X_W; j++)
        {
            int64 tmp = image[i][j] - avr;
            sum += tmp * tmp;

        }
    }
    float fangcha = (float) (sum * scale);                  //计算总方差,注意不要除以scale，能加快运行速度
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

    }
    else
    {
        consist_fail++;
        if (last_image_exist == 1 && consist_fail <= 5)
        {
            memcpy(image, last_image_copy, MT9V03X_IMAGE_SIZE);
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
                push(i * MT9V03X_W + j);
                image[i][j] = 1;
                while (!empty())
                {
                    int32 tmp = top(); //应该是front，懒得改了
                    pop();

                    size++;
                    for (int k = 0; k < 9; k++)
                    {

                        int hang = tmp / MT9V03X_W + s1[k], lie = tmp - tmp / MT9V03X_W * MT9V03X_W + s2[k];
                        if (hang < 0 || lie < 0 || hang >= MT9V03X_H || lie >= MT9V03X_W)
                        {
                            continue;
                        }
                        if (image[hang][lie] == 255)
                        {
                            image[hang][lie] = 1;
                            push(hang * MT9V03X_W + lie);
                        }
                    }
                }
                if (size < 60)
                {
                    //删除
                    push(i * MT9V03X_W + j);
                    image[i][j] = 0;
                    while (!empty())
                    {
                        int32 tmp = top();
                        pop();

                        for (int k = 0; k < 9; k++)
                        {

                            int hang = tmp / MT9V03X_W + s1[k], lie = tmp - tmp / MT9V03X_W * MT9V03X_W + s2[k];

                            if (hang < 0 || lie < 0 || hang >= MT9V03X_H || lie >= MT9V03X_W)
                            {
                                continue;
                            }
                            if (image[hang][lie] == 1)
                            {
                                push(hang * MT9V03X_W + lie);
                                image[hang][lie] = 0; //清0
                            }

                        }
                    }

                }

            }
        }

    }

}
#define LINEWIDTH 7
#define YOUXIAOXIANKUAN 5
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
        rline = min(lline-LINEWIDTH, 187);
    return (lline << 8) + rline; //返回左右的值

}
#define SECTIONWIDTH 5
#define MAXSECTIONS 8
int8 find_section (uint8_t p[][MT9V03X_W], int h, uint8 s[][2])
{ //0->l 1->r,返回段的个数
    int sum = 0;
    uint8 flag = 0;
    for (int i = 0; i < MT9V03X_W; i++)
    {
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
    return sum > MAXSECTIONS ? 0 : sum;
}
#define ZUI_DA_GAO_DU 50//仅仅用于计算斜率
#define YUN_XV_SHI_BAI_CI_SHU 5
#define MAX_LINE_WIDTH 80

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
#define MAX_BIAS 50
#define MAX_BOX 5
#define MIN_BOX_HEIGHT 10
#define ZUI_XIAO_LU_KOU_KUAN_DU 65
#define MAX_HEIGHT 80//仅仅用于计算box的最大可出现像素
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
#define MIN_AREA_AVR_LEN 16//面积平均算法的最小面积
#define AREA_MIN_LEN    8//最小处理长度
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

