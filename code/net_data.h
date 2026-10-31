#define NCH1 4//1输出通道数
#define NCH2 16//2通道数
#define SH1 11//真实权重=权重/（2^11）
#define SH2 9
#define SHF 8//存储时放大了多少倍
#define GAPSH 8   //尺度补偿，先乘再除
#define GAPF 16     //最后等价放大倍数
#include<stdint.h>
static const int16_t W1[36];//conv1 卷积核
static const int32_t B1[4];//conv1 偏置
static const int16_t W2[576];//conv2 卷积核
static const int32_t B2[16];//conv2 偏置
static const int16_t FW[256];//全连接权重
static const int32_t FB[16];//全连接偏置
#define NETSUM_W1 (6214L)//校验权重数据有没有被改坏
#define NETSUM_B1 (-953557L)
#define NETSUM_W2 (17596L)
#define NETSUM_B2 (-1302460L)
#define NETSUM_FW (-12901L)
#define NETSUM_FB (2784L)
static void conv1_int (const uint8_t g[][18], int16 out[NCH1][16][16]);
static void conv2_int (const int16_t p[NCH1][10][10], int16_t out[NCH2][8][8]);
static void gap_int(const int16_t in[NCH2][8][8],int32_t out[NCH2]);
static int32_t max_score(const int32_t in[NCH2],int32_t *out);

