#include "zf_common_headfile.h"

extern uint8 last_image_copy[MT9V03X_H][MT9V03X_W];

void double_value(uint8_t image[][MT9V03X_W]);
void xiao_qu_yu_shan_chu(uint8_t image[][MT9V03X_W]);
int32 find_line (uint8_t image[][MT9V03X_W]);
float qing_xie_du (int32 x, uint8_t image[][MT9V03X_W]);
void image_init();
