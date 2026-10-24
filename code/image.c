#include<image.h>
#include<stdlib.h>
#include "zf_common_headfile.h"//头文件禁止引用头文件防止内存超限
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
int cmp(const void *a, const void *b){
return *(int *)a - *(int *)b;
}

uint8 s1[9]={-1,0,1,-1,0,1,-1,0,1};
uint8 s2[9]={-1,-1,-1,0,0,0,1,1,1};
void double_value(uint8_t image[][MT9V03X_W]){//二值化
    //平均数备用
    uint64_t huidu[259]={0};//冗余设计
    uint32 scale=MT9V03X_H*MT9V03X_W;
    uint8 minn=255,maxn=0;
    uint32_t avr=0;//0-6517800
    //uint8 tong[255]={0},zhi[9]={0};
    for(int i=1;i<MT9V03X_H-1;i++){
        for(int j=1;j<MT9V03X_W-1;j++){
            //uint8 s1[9]={0,1,2,3,4,5,6,7,8};
            uint8 tmp[9]={0};
            //uint8 max_time;//最大项
            for(int i=0;i<9;i++){
                tmp[i]=image[s1[i]][s2[i]];
            }
            qsort(tmp,9,sizeof(tmp[0]),cmp);
            image[i][j]=tmp[4];
        }//去除胡椒噪声

    }

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
    float fangcha=0;
    //大津法
    for(uint8 t=minn;t<maxn;t++){
        uint64_t w0=0,w1=0;
        float u0=0,u1=0;
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
