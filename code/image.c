#include<image.h>
#include "zf_common_headfile.h"//头文件禁止引用头文件防止内存超限
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
void double_value(uint8_t image[][MT9V03X_W]){//二值化
    //平均数备用
    uint64_t huidu[259]={0};//冗余设计
    uint32 scale=MT9V03X_H*MT9V03X_W;
    uint8 minn=255,maxn=0;
    uint32_t avr=0;//0-6517800
    for(int i=0;i<MT9V03X_H;i++){
        for(int j=0;j<MT9V03X_W;j++){
            avr+=image[i][j];
            uint8 now_dot=image[i][j];
            huidu[now_dot]++;
            minn=min(minn,now_dot);
            maxn=max(maxn,now_dot);
        }
    }
    avr/=scale;
    uint8 best_value=-1;
    long double fangcha=0;
    //大津法
    for(uint8 t=minn;t<maxn;t++){
        uint64_t w0=0,w1=0;
        long double u0=0,u1=0;
        for(int i=minn;i<=t;i++){
            w0+=huidu[i];
            u0+=i*huidu[i];
        }
        for(int i=t+1;i<=maxn;i++){
            w1+=huidu[i];
            u1+=i*huidu[i];

        }
        u0/=w0;//除以类内元素个数
        u1/=w1;//同理
        if(((uint64)(w0*w1))*(u0-u1)*(u0-u1)>fangcha){
            fangcha=((uint64)(w0*w1))*(u0-u1)*(u0-u1);
            best_value=t;
        }
    }
    //todo:
    //处理成0和1（或0和255的值）
    //进阶大津法（分块）
}
