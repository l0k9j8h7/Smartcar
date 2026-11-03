void control(uint32 tick,int16 delta,int speed);

#define  ZU_SE_ZHEN_SHU 250
void pid_reset (void);
int16 pid(void);
void stop(void);
#define START_TIME 30000000//0.3s
#define DI_TONG_ZHI_HOU 0.55//0-1
