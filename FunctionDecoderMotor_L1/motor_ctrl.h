

//#define MOTOR_PWM_A 3
//#define MOTOR_PWM_B 11

#define MOTOR_PWM_A 9
#define MOTOR_PWM_B 10

extern void MOTOR_Init();
extern void MOTOR_Ack(void);
extern void MOTOR_Main(int inSpeedCmd, int inDirection);
