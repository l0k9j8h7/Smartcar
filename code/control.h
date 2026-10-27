#ifndef __CONTROL_H
#define __CONTROL_H

#define START_TIME          30000000        /* 0.3 s£¨system_getval µ¥Î» 10 ns£© */
#define CTRL_PERIOD_US      5000
#define CTRL_PERIOD_TICK    (CTRL_PERIOD_US * 100u)
#define MAX_SPEED           1000

extern int16 P, I, D, err, last_err;

void  control      (uint32 tick, uint16 delta);
int16 pid          (void);
void  ctrl_set_err (int16 e);

#endif
