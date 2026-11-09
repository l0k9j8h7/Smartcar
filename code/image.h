#include "zf_common_headfile.h"

extern uint8 last_image_copy[MT9V03X_H][MT9V03X_W];
extern uint8 image_valid;   // 本帧二值化结果是否可信，由 double_value() 更新，主循环据此判丢线
extern uint8 image_fan_se;  // 本帧是否被极性校正整帧取反（1 = 反色帧），主循环用它兜白墙误反色
extern uint8 image_lag;     // 本帧是否回退用了旧图（1 = 落后帧），主循环据此降速

void double_value (uint8_t image[][MT9V03X_W]);
void xiao_qu_yu_shan_chu (uint8_t image[][MT9V03X_W]);
int32 find_line (uint8_t image[][MT9V03X_W], int height);
float qing_xie_du (int32 x, uint8_t image[][MT9V03X_W]);
uint8 find_boxes (uint8_t image[][MT9V03X_W], uint8 box[][4][2]);
void image_init (void);
int8 find_section (uint8_t p[][MT9V03X_W], int h, uint8 s[][2]);
uint8 image_read (uint8_t image[][MT9V03X_W],uint8_t result[]);//返回box个数，result是类别
