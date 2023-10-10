
#include "PID.h"
#include "MainTask.h" 

/*-------------------------------------------------------------------------------------------------------------
  @Function: IncrementPI_Ctrl
  @Brief: ����ʽPI������
  @Description: ���ݵ�ǰֵ��Ԥ��Ŀ��ֵ,��������ʽPI�㷨�����õ���Ӧ�����ֵ
                ��Uk = Kp(e(k) - e(k-1)) + e(k)*Kp*T/Ti
                     = Kp(1+T/Ti)*e(k) - Kp*e(k-1)
                     = K1*e(k) + K2*e(k-1)
                K1 = Kp(1 + T/Ti) = Kp(1 + T/kT) = Kp(1 + 1/k) = Kp(1 + Ki)
                K2 = -Kp
                U(k) = U(k-1) + ��U(k)
  @Calls:
  @Called By: while(1)��maintask������
  @Input:  ��ǰֵ��Ԥ��Ŀ��ֵ��PI�����ṹ��ָ��  
  @Output: null
  @Return: PI���������ֵ
  @Remark: 
  @History:
  <Date> <Author> <Version> <Description>
  2008.10.20,Lichar,V1,����
  2008.01.10,Lichar,V2,�޸�PI����������BUG(�����޷������ֵ���ۼӵ��������ͬһ��ֵ)
-------------------------------------------------------------------------------------------------------------*/
s32 IncrementPI_Ctrl(s16 ref,s16 fdb,S_INC_PI_PARA_T *ps)
{
   s32 err = 0;     

   err = ref - fdb; //Ŀ��ֵ-��ǰֵ:�ó�����ƫ��
   ps->out += (ps->K1*err + ps->K2*ps->lastErr);//��һ�����ֵ=k1*����ƫ��+k2*��һ��ƫ��
   ps->lastErr = err; //��������ƫ��

   if(ps->out > ps->outMax)      { ps->out = ps->outMax; }
   else if(ps->out < ps->outMin) { ps->out = ps->outMin; }  

   return(ps->out>>16);
}
#if 0
//����ʽPI������ʼ��
void SpeedIncPI_Init(void)               
{
   g_sSpeedIncPI.K1 = SPEED_K1_Q16;          //K1 = Kp*(1+Ki)           
   g_sSpeedIncPI.K2 = SPEED_K2_Q16 ;         //K2 = -Kp
   g_sSpeedIncPI.lastErr = SPEED_LastErr;    //��һ��ƫ��ֵ
   g_sSpeedIncPI.out = SPEED_OUT;            //��һ�����ֵ  
   g_sSpeedIncPI.outMin = SPEED_OutMin_Q16;  //��С�޷�
   g_sSpeedIncPI.outMax = SPEED_OutMax_Q16;  //����޷�
}
#endif 

//����ʽPI������ʼ��
void SpeedIncPI_Init(void)            
{
   //g_sMotor.u16TargetSpeed = (u16)(64)*(g_sMotor.u8MotorRecvBuf[2]&0x3f);  
   
   g_sMotor.u16TargetSpeed =3100 ;  
   if(g_sMotor.u16TargetSpeed >= 1856) //>=29RPM*64���ٱ� �������ʱKP=0.05    
   {
         g_sSpeedIncPI.K1 = (int32_t)(65536.0L*0.01*(1.0+0.2));  //K1 = Kp*(1+Ki)         
	  g_sSpeedIncPI.K2 = (int32_t)(-65536.0L*0.01);           //K2 = -Kp
   }

   
   
   g_sSpeedIncPI.lastErr = SPEED_LastErr;                        //��һ��ƫ��ֵ   
   g_sSpeedIncPI.out     = SPEED_OUT;                            //��һ�����ֵ 
   g_sSpeedIncPI.outMin  = (int32_t)(65536.0L*DUTY_15);         //��С�޷�
   g_sSpeedIncPI.outMax  = (int32_t)(65536.0L*DUTY_FULL);       //����޷�
 
   g_sMotor.PIRefSpeedUpValue = 20;          //�����ٶȲ���ֵ3000*0.01   
   g_sSpeedIncPI.u8RefSpeedTimeValue = 1;    //PI�����ڸ���ʱ���׼ֵ(10ms)  
}






















