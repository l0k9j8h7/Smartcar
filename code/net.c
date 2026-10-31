#include "zf_common_headfile.h"
#include"net_data.h"
#define H 60
#define W 24
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
static void conv1_int (const uint8_t g[][18], int16 out[NCH1][16][16])
{
    int on, r, c, dr, dc, i; //r,c:核偏移,dr,dc:核内偏移,i:第多少个权重,on:卷积核种类
    for (on = 0; on < NCH1; on++)
    {
        for (r = 0; r < 16; r++)
        {
            for (c = 0; c < 16; c++)
            {
                int32_t acc = B1[on]; //偏置
                i = 0;
                for (dr = 0; dr < 3; dr++)
                {
                    for (dc = 0; dc < 3; dc++)
                    {
                        acc += (int32_t) W1[on * 9 + i] * g[r + dr][c + dc]; //卷积和
                        i++;

                    }
                }
                out[on][r][c] = (int16_t) ((acc <= 0) ? 0 : (acc >> SH1));
            }
        }
    }

} //池化 ->4x8x8
static void pool1_int (const int16_t in[NCH1][16][16], int16_t out[NCH1][8][8])
{
    for (int on = 0; on < NCH1; on++)
    {
        for (int r = 0; r < 8; r++)
        {
            for (int c = 0; c < 8; c++)
            {
                out[on][r][c] = max(max(max(in[on][r*2][c*2],in[on][r*2][c*2+1]),in[on][r*2+1][c*2]),
                        in[on][r * 2 + 1][c * 2 + 1]); //取最大
            }
        }
    }
}

static void conv2_int (const int16_t p[NCH1][10][10], int16_t out[NCH2][8][8])
{
    for (int r = 0; r < 8; r++)
    {
        for (int c = 0; c < 8; c++)
        {
            int16_t win[NCH1][9] = {{0}}; //二维滑动窗口
            for (int i = 0; i < NCH1; i++)
            {
                int k = 0;
                for (int dr = 0; dr < 3; dr++)
                {
                    for (int dc = 0; dc < 3; dc++)
                    {
                        win[i][k] = p[i][r + dr][c + dc];
                        k++;

                    }
                }
            }
            for (int on = 0; on < NCH2; on++)
            {
                int32_t acc = B2[on];
                for (int i = 0; i < NCH1; i++)
                {
                    for (int k = 0; k < 9; k++)
                    {
                        acc += (int32_t) W2[(on * NCH1 + i) * 9 + k] * win[i][k];
                    }
                    //第几个核的第多少个通道的第多少个参数

                }
                out[on][r][c] = (int16_t) (max(acc,0) >> SH2);

            }

        }
    }
}
static void gap_int (const int16_t in[NCH2][8][8], int32_t out[NCH2])
{ //全局平均池化
    for (int on = 0; on < NCH2; on++)
    {
        int32 sum = 0;
        for (int r = 0; r < 4; r++)
        {
            for (int c = 0; c < 4; c++)
            {
                sum += max(max(max(in[on][r*2][c*2],in[on][r*2][c*2+1]),in[on][r*2+1][c*2]),
                        in[on][r * 2 + 1][c * 2 + 1]);
            }
        }
        out[on] = sum;
    }
}
static int32_t max_score (const int32_t in[NCH2], int32_t *out)
{
    int32_t best = -1;
    int32_t best_v = INT32_MIN; //-1

    for (int i = 0; i < NCH2; i++)
    {
        int32_t acc = FB[i];
        for (int j = 0; j < NCH2; j++)
        {
            acc += FW[i * NCH2 + j] * in[j];

        }
        if (out)
            out[i] = acc;
        if (acc > best_v)
            best_v = acc, best = i;
    }
    return best;

}
