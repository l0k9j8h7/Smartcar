#include<image.h>
#include<stdlib.h>
#include "zf_common_headfile.h"//头文件禁止引用头文件防止内存超限
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
//分成6*4个小方块
int8 s1[9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
int8 s2[9] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
uint8 last_image_copy[MT9V03X_H][MT9V03X_W];
uint8 last_image_exist = 0;
float now_zhi_xin_du = 0;
float zhi_xin_du = 0.6;
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
uint8 to_uint8 (int64 tmp)
{
    return max(min(tmp,255), 0);
}
int64 consist_fail = 0;

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
    uint8 minn = 255, maxn = 0/*, jv_bu_min[3][4] = {0}, jv_bu_max[3][4] = {0}*/;
    int32 fa_zhi = 0;
    /*
     for (int i = 0; i < 3; i++)
     {
     for (int j = 0; j < 4; j++)
     {
     jv_bu_min[i][j] = 255;
     }
     }*/
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
            int16 an_bu = 0;
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
            for (int16 t = minn; t < maxn; t++)
            {
                tmp += kuai_nei_huidu[t];
                if (tmp > 100 || t >= (maxn - minn) / 5 + minn)
                {

                    an_bu = t;
                    break;
                }

            }
            for (int k1 = 0; k1 < 40; k1++)
            {
                for (int k2 = 0; k2 < 47; k2++)
                {
                    uint8 *now_dot = &image[i * 40 + k1][j * 47 + k2];
                    huidu[*now_dot]--;
                    int32 tmp = quan_jv_an_bu - an_bu;
                    if (tmp >= 80)
                        tmp = 80;
                    if (tmp <= -80)
                        tmp = -80;
                    *now_dot = to_uint8(tmp + *now_dot);                 //地址
                    huidu[*now_dot]++;
                }
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
            int32 tmp = image[i][j] - avr;
            sum += tmp * tmp;

        }
    }
    float fangcha = sum * scale;                  //计算总方差,注意不要除以scale，能加快运行速度
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
    static uint8 color[MT9V03X_H][MT9V03X_W] = {0};
    memset(color, 0, sizeof(color));
    //uint32 sz[8000]={0};
    int colors = 0;
    clear();
    for (int i = 0; i < MT9V03X_H; i++)
    {
        for (int j = 0; j < MT9V03X_W; j++)
        {
            if (!color[i][j] && image[i][j]) //只处理白线
            {
                int size = 0;
                push(i * MT9V03X_W + j);
                color[i][j] = ++colors;

                      /* 起头立刻清 */
                if (colors >= 255)
                    colors = 1;
                //size++;
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
                        if (!color[hang][lie] && image[hang][lie])
                        {
                            color[hang][lie] = colors;
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
                            if (color[hang][lie]==colors)
                            {
                                push(hang * MT9V03X_W + lie);
                                color[hang][lie] = 0;
                                image[hang][lie] = 0; //清0
                            }
                        }
                    }

                }
            }
        }

    }
}
