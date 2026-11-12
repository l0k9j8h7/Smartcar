#include "zf_common_headfile.h"

extern uint8 last_image_copy[MT9V03X_H][MT9V03X_W];
extern uint8 image_valid;   // 本帧二值化结果是否可信，由 double_value() 更新，主循环据此判丢线
extern uint8 image_fan_se;  // 本帧是否被极性校正整帧取反（1 = 反色帧），主循环用它兜白墙误反色
extern uint8 image_lag;     // 本帧是否回退用了旧图（1 = 落后帧），主循环据此降速
/* qing_xie_du() 的返回状态（对抗审查 P0-1 订正版：把「宽度超限」和「爬线失败」分开）
 *   0 = 斜率有效，x1 可以直接用
 *   1 = 底行宽度在 MAX_LINE_WIDTH(120)~XUN_XIAN_ZUI_DA_KUAN_DU(150) 之间：真实宽赛道，
 *       斜率不可用，但边线本身还在
 *       -> 主循环仍用 lline/rline 算 err 巡线，只把 x1 前馈关掉，不再整帧丢线
 *   2 = 爬线失败 / 入参非法 / 底行宽于 150（噪声假线）：必须判丢线 */
extern uint8 qing_xie_du_zhuang_tai;

void double_value (uint8_t image[][MT9V03X_W]);
void xiao_qu_yu_shan_chu (uint8_t image[][MT9V03X_W]);
int32 find_line (uint8_t image[][MT9V03X_W], int height);
float qing_xie_du (int32 x, uint8_t image[][MT9V03X_W]);
uint8 find_boxes (uint8_t image[][MT9V03X_W], uint8 box[][4][2]);
void image_init (void);
int8 find_section (uint8_t p[][MT9V03X_W], int h, uint8 s[][2]);
uint8 image_read (uint8_t image[][MT9V03X_W],uint8_t result[]);//返回box个数，result是类别
