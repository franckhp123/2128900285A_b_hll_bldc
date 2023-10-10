
#include "PID.h"
#include "MainTask.h" 

/*-------------------------------------------------------------------------------------------------------------
  @Function: IncrementPI_Ctrl
  @Brief: 增量式PI控制器
  @Description: 根据当前值与预设目标值,采用增量式PI算法计算后得到对应的输出值
                △Uk = Kp(e(k) - e(k-1)) + e(k)*Kp*T/Ti
                     = Kp(1+T/Ti)*e(k) - Kp*e(k-1)
                     = K1*e(k) + K2*e(k-1)
                K1 = Kp(1 + T/Ti) = Kp(1 + T/kT) = Kp(1 + 1/k) = Kp(1 + Ki)
                K2 = -Kp
                U(k) = U(k-1) + △U(k)
  @Calls:
  @Called By: while(1)中maintask子任务
  @Input:  当前值、预设目标值、PI参数结构体指针  
  @Output: null
  @Return: PI控制器输出值
  @Remark: 
  @History:
  <Date> <Author> <Version> <Description>
  2008.10.20,Lichar,V1,初版
  2008.01.10,Lichar,V2,修改PI输出会溢出的BUG(用于限幅输出的值和累加的输出不是同一个值)
-------------------------------------------------------------------------------------------------------------*/
s32 IncrementPI_Ctrl(s16 ref,s16 fdb,S_INC_PI_PARA_T *ps)
{
   s32 err = 0;     

   err = ref - fdb; //目标值-当前值:得出本次偏差
   ps->out += (ps->K1*err + ps->K2*ps->lastErr);//上一次输出值=k1*本次偏差+k2*上一次偏差
   ps->lastErr = err; //存贮本次偏差

   if(ps->out > ps->outMax)      { ps->out = ps->outMax; }
   else if(ps->out < ps->outMin) { ps->out = ps->outMin; }  

   return(ps->out>>16);
}
#if 0
//增量式PI参数初始化
void SpeedIncPI_Init(void)               
{
   g_sSpeedIncPI.K1 = SPEED_K1_Q16;          //K1 = Kp*(1+Ki)           
   g_sSpeedIncPI.K2 = SPEED_K2_Q16 ;         //K2 = -Kp
   g_sSpeedIncPI.lastErr = SPEED_LastErr;    //上一次偏差值
   g_sSpeedIncPI.out = SPEED_OUT;            //上一次输出值  
   g_sSpeedIncPI.outMin = SPEED_OutMin_Q16;  //最小限幅
   g_sSpeedIncPI.outMax = SPEED_OutMax_Q16;  //最大限幅
}
#endif 

//增量式PI参数初始化
void SpeedIncPI_Init(void)            
{
   //g_sMotor.u16TargetSpeed = (u16)(64)*(g_sMotor.u8MotorRecvBuf[2]&0x3f);  
   
   g_sMotor.u16TargetSpeed =3100 ;  
   if(g_sMotor.u16TargetSpeed >= 1856) //>=29RPM*64减速比 正常割草时KP=0.05    
   {
         g_sSpeedIncPI.K1 = (int32_t)(65536.0L*0.01*(1.0+0.2));  //K1 = Kp*(1+Ki)         
	  g_sSpeedIncPI.K2 = (int32_t)(-65536.0L*0.01);           //K2 = -Kp
   }

   
   
   g_sSpeedIncPI.lastErr = SPEED_LastErr;                        //上一次偏差值   
   g_sSpeedIncPI.out     = SPEED_OUT;                            //上一次输出值 
   g_sSpeedIncPI.outMin  = (int32_t)(65536.0L*DUTY_15);         //最小限幅
   g_sSpeedIncPI.outMax  = (int32_t)(65536.0L*DUTY_FULL);       //最大限幅
 
   g_sMotor.PIRefSpeedUpValue = 20;          //给定速度步进值3000*0.01   
   g_sSpeedIncPI.u8RefSpeedTimeValue = 1;    //PI环周期给定时间基准值(10ms)  
}






















