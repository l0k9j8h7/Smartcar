#include "tiaocan.h"                // 可调参数集中到 tiaocan.h，调车只改那一个文件

void control(uint32 tick,int16 delta,int speed);

void pid_reset (void);
int16 pid(void);
void stop(void);
