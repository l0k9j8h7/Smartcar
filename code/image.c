#include<image.h>
#include<stdlib.h>
#include "zf_common_headfile.h"//头文件禁止引用头文件防止内存超限
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
uint8 cmp (const void *a, const void *b)
{
    return *(uint8*) a - *(uint8*) b;
}
//分成6*4个小方块
int8 s1[9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
int8 s2[9] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
int16 da_jin_fa (int minn, int maxn, uint16 huidu[])
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
    return best_value;
}
void double_value (uint8_t image[][MT9V03X_W])
{ //二值化

    uint16 huidu[259] = {0}; //宽/高/像素种类

    uint32 scale = MT9V03X_H * MT9V03X_W;
    uint8 minn = 255, maxn = 0, jv_bu_min[3][4] = {0}, jv_bu_max[3][4] = {0};
    uint32_t fa_zhi = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            jv_bu_min[i][j] = 255;
        }
    }
    for (int i = 1; i < MT9V03X_H - 1; i++)
    {
        for (int j = 1; j < MT9V03X_W - 1; j++)
        {
            uint8 x = image[i][j];
            uint32 sum = 0;

            huidu[x]++;

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
                image[i][j] = (uint8) (sum >> 3);                 // 填 8 邻居均值
            }
        }
    }

    int32 quan_jv_an_bu = 0;
    {
        int16 tmp = 0;
        for (int t = minn; t < maxn; t++)
        {
            tmp += huidu[t];
            if (tmp > 1200 || t > (maxn - minn) / 5 + minn)
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
            int16 huidu[256] = {0};
            uint8 minn = 255, maxn = 0;
            int16 an_bu = 0;
            for (int k1 = 0; k1 < 40; k1++)
            {
                for (int k2 = 0; k2 < 47; k2++)
                {
                    uint8 now_dot = image[i * 40 + k1][j * 47 + k2];
                    huidu[now_dot]++;
                    minn = min(now_dot, minn);
                    maxn = max(now_dot, maxn);
                }
            }
            int16 tmp = 0;
            for (int16 t = minn; t < maxn; t++)
            {
                tmp += huidu[t];
                if (tmp > 100 || t > (maxn - minn) / 5 + minn)
                {
                    an_bu = t;
                    break;
                }

            }
            for (int k1 = 0; k1 < 40; k1++)
            {
                for (int k2 = 0; k2 < 47; k2++)
                {
                    image[i * 40 + k1][j * 47 + k2] += quan_jv_an_bu - an_bu;

                }
            }

        }
    }
    fa_zhi = da_jin_fa(minn, maxn, huidu);
    for (int i = 0; i < MT9V03X_H ; i++)
        {
            for (int j = 0; j < MT9V03X_W ; j++)
            {
               image[i][j]=image[i][j]>fa_zhi?255:0;
            }
        }

}
