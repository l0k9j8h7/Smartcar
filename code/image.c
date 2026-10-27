#include<image.h>
#include<stdlib.h>
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

uint8 last_image_exist = 0;
float now_zhi_xin_du = 0;
float zhi_xin_du = 0.6;

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
ding_bi_shu_256 cha_zhi[4];

void cha_zhi_update (int i, int j) //0 ↖ 1 ↗ 2↙ 3 ↘
{
    if (i <= 20)
        i = 20;
    else if (i >= 99)
        i = 99;
    if (j <= 24)
        j = 24;
    else if (j >= 164)
        j = 164;//使用最外围的点求值

    for (int k = 0; k < 4; k++)
        cha_zhi[k].n = 0;
    int32 row_up = (i * 2 - 39) / 80;
    int32 col_left = (j * 2 - 47) / 94;
    ding_bi_shu_256 wx, wy;
    wy.n = (i * 2 - 39 - row_up * 80) * 128 / 40;//越大反而越小
    wx.n = (j * 2 - 47 - col_left * 94) * 128 / 47;    //超级复杂的数学公式
    cha_zhi[0].n = (256 - wy.n) * (256 - wx.n);            //放大65536倍
    cha_zhi[1].n = (256 - wy.n) * (wx.n);
    cha_zhi[2].n = wy.n * (256 - wx.n);
    cha_zhi[3].n = wx.n * wy.n;

}
static inline uint8 to_uint8 (int64 tmp)
{
    return (uint8)(max(min(tmp,255), 0));
}
int64 consist_fail = 0;
int16 fen_kuai_bu_chang[3][4] ={{0}};    //65536 max
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
            fen_kuai_bu_chang[i][j]=minn;//取最小值防止不进入循环（minn=maxn)
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

            int16 row_up = ((k) * 2 - 39) / 80;
            int16 row_down = row_up + 1;
            int16 col_left = ((l) * 2 - 47) / 94;
            int16 col_right = col_left + 1;
            cha_zhi_update(k, l);

            int64_t bu_chang =
                    quan_jv_an_bu
                            - ((cha_zhi[0].n * fen_kuai_bu_chang[row_up][col_left]
                                    + cha_zhi[1].n * fen_kuai_bu_chang[row_up][col_right]
                                    + cha_zhi[2].n * fen_kuai_bu_chang[row_down][col_left]
                                    + cha_zhi[3].n * fen_kuai_bu_chang[row_down][col_right]) >> 16)/*计算时用了wx,wy 256定比数，需要除以65536*/;
            bu_chang = max(-40, min(40,bu_chang));
            *now_dot = to_uint8(bu_chang + *now_dot);                 //地址
            huidu[*now_dot]++;
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
    float fangcha = (float)(sum * scale);                  //计算总方差,注意不要除以scale，能加快运行速度
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
void xiao_qu_yu_shan_chu (uint8_t image[][MT9V03X_W]){
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
                else
                {
                    image[i][j] = 255;
                    push(i * MT9V03X_W + j);
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
                                image[hang][lie] = 255; //清0
                            }

                        }
                    }
                }
            }
        }

    }

}
