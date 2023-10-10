
#include "MainTask.h"   
#include "SubTask.h"
void Main_MotorTask(void) //子循环任务,1ms周期      
{
   u16 cnt=0;
   static u16 brake_cnt=0;
   switch(g_eSysState)   
   {                     //自检状态(停机关驱动);待机状态(停机关驱动);故障状态(停机关驱动)    
      case cSTA_SELFCHK:                              
	  case cSTA_WAIT:     
	  case cSTA_ERR:     BLDC_StopMotor();         break;                      
	  case cSTA_RUN:     MotorWalkDisp_InRunSta(); break; //运行(待机动作+自举动作+定位动作+运行动作)   
	  case cSTA_STOP:    BLDC_ConsumeBrake();      break; //停机刹车动作          
	  default:break;    
   } 

   switch(g_eSysState)           
   {
      case cSTA_SELFCHK:     
	  	   if(g_sFlagErr.all)  
		   { 
		       g_eSysState = cSTA_ERR; 
			   RstVarBeforInErrSta(); 
		    }
	  	   else                
		   { 
		       g_eSysState = cSTA_WAIT; 
		   } 
		   break;
		   
	  case cSTA_WAIT:  //其余在串口接收中断中改变状态机(实时性高)
	 #if 0
	         g_eSysState = cSTA_RUN; 
	         RstVarBeforInRunSta();
						  
			 g_sMotor.eDir = eCCW;
			break;
	#endif 
	  case cSTA_RUN:
	  	   if(g_sFlagErr.all)  
		   	{ 
		   	    g_eSysState = cSTA_ERR; 
				RstVarBeforInErrSta(); 
			}
	  	   break;
		   
	  case cSTA_STOP:
	  	    if(g_sFlagErr.all)
            { 
                g_eSysState = cSTA_ERR; //有故障切换为故障状态   
				RstVarBeforInErrSta(); 
			}                       
            else 
            {   
              // BLDC_ConsumeBrake();  
              // if(++g_sMotor.u16BrakeTime >= 4000)//刹车200ms转待机状态   
               if(++brake_cnt >= 4000)//刹车200ms转待机状态               
               { 
                   brake_cnt=0;
                   //s_u8BrkOnceFlag=0;
                 //  LampOff;
                   g_sMotor.u16BrakeTime = 0; 
				   g_eSysState = cSTA_WAIT; 
			   }   
            }
	  	    break;
	  case cSTA_ERR: //短路故障;过流故障;MOS过温故障;母线电压欠压故障     
	  	                  
			break;
      default: break;    
   }


}


void RstVarBeforInErrSta(void) //进入故障状态前需清相关变量
{
   g_sBusVdc.sum = 0;
   g_sBusVdc.cnt = 0;
   g_sBusIdc.sum = 0;
   g_sBusIdc.cnt = 0;  
   g_sMosTmp.sum = 0;
   g_sMosTmp.cnt = 0;
   g_sMotor.u8MotorErrStaTxCnt = 0;
}

void MotorWalkDisp_InRunSta(void)  //工作状态下的电机处理
{
   switch(g_sMotor.eAct)    
   {
      case eSTANDBY_ACT:   Motor_Standby_Act();  break; //待机动作                      
	  
	  case eRUN_ACT:       Motor_Run_Act();      break; //运行动作    
      
	
	  default:             break;
   }
}

void Motor_Standby_Act(void) //串口中断接到主机命令进入这个函数。        
{  
   static uint8_t HOff_flag = 0;

   if(HOff_flag == 0)
   {
        BLDC_ConsumeBrake(); // 刹车自举
        HOff_flag = 1;
   }
   Motor_StateCnt(&g_sMotor.uStaCnt.u8Standby);       //待机状态计数      
   if(g_sMotor.uStaCnt.u8Standby >= cSTANDBY_1ms_CNT) //待机状态计时到       
   {
       HOff_flag=0;//自举时间到
     #if 1
      #if  1//正式板
      if(TESTBIT(g_sMotor.u8MotorRecvBuf[2],7)) { g_sMotor.eDir = eCCW; }         
	  else                                      { g_sMotor.eDir = eCW; }
	  #endif
  #endif 
     #if  0//正式板
      if(TESTBIT(g_sMotor.u8MotorRecvBuf[2],7)) { g_sMotor.eDir = eCW; }         
	  else                                      { g_sMotor.eDir = eCCW; }
	  #endif
	  g_sDrive.u16CurrPwmDuty = 0;   
	  g_sDrive.u16TargetPwmDuty = 0;    
	  g_sBusVdc.sum = 0;                         //清AD采样相关变量
      g_sBusVdc.cnt = 0;
	  g_sBusIdc.sum = 0;
      g_sBusIdc.cnt = 0;
	  g_sMosTmp.sum = 0;
      g_sMosTmp.cnt = 0;
	  SpeedIncPI_Init();                         //PI参数初始化 
	  g_sMotor.sSpeedFilt.u8Cnt = 0;             //清速度滤波计数器
	  g_sMotor.sSpeedFilt.u32Sum = 0;            //清速度滤波累加和 
	  g_sMotor.u16Speed = 0;                     //电机速度清0  
	  g_aTzc[0] = 0;                             //速度计算:过零计数器清0   
	  g_aTzc[1] = 0;
	  g_aTzc[2] = 0;   
      R_TAU0_Channel0_Start();                    //开启定时器0-CH0(换相间隔时间)  
      g_u16LastZeroTimerVal = _FFFF_TAU_TDR00_VALUE; //上次过零定时器时间值初始化  
      
      g_sMotor.u16RefSpeed    = 1500;                //给定PI环目标速度设为1000RPM(在子任务中稳速目标值改变时此值被更新)
      g_sMotor.u16TargetSpeed = 1500;
      
      PwmDrv_PwmOutDisable(); //工作前先关六管,防止直通
     
     
	  uHallValue = Hall_Get();                    //read hall value
	  if((uHallValue==0)||(uHallValue==7))
	  {
	      return;
	  }
	  MotorPosition=GetMotorPosition(uHallValue);
      
      PwmDrv_PwmOutDisable();
	  HALL_INT_ON();		                      //enable hall interrupt
     
	  CurrentDuty = DUTY_8;    //2998/24=125
	  MotorDirection = g_sMotor.eDir;
	  PWM_DutyUpdata(CurrentDuty);
	  Cap_PreCharge(MotorPosition);
	  DelayXus(10);				
	  Commutate_Phase(MotorPosition);
	  R_TMRD0_Start();
	  LampFlash();
     if(++block_cnt>500)
     {
         block_cnt=0;
         g_sFlagErr.Bits.bMotorBlock=1;
     }
	  uHallValueExpect = Expect_Hall_Value(MotorPosition);
	  g_sMotor.eAct = eRUN_ACT;                 //切换
	  
   }
}

//尝试修改PI环,启动2S内PI环增速慢,2s以后增速快
void Motor_Run_Act(void) //运行动作执行                 
{   
   g_sMotor.u16TargetSpeed = 3100; 
   BLDC_u8ZeroCrossTimeToSpeed(&g_sMotor,g_aTzc); //得出实时转速                                                  
   g_sDrive.u16TargetPwmDuty = (u16)IncrementPI_Ctrl(g_sMotor.u16RefSpeed,g_sMotor.u16Speed,&g_sSpeedIncPI);// 速度增量式PI
   g_sDrive.u16CurrPwmDuty = g_sDrive.u16TargetPwmDuty;   
   if(++g_sSpeedIncPI.timePIUpGiveSpeed >= 10)  
   {
         g_sSpeedIncPI.timePIUpGiveSpeed = 0;           
	   g_sMotor.u16RefSpeed = MotorSpeedStepCtrl(g_sMotor.u16TargetSpeed,g_sMotor.u16RefSpeed); //给定速度输入到PI环:当前转速逐步到目标转速 
   }  
   if(++block_cnt>500)
   {
       block_cnt=0;
       g_sFlagErr.Bits.bMotorBlock=1;
   }
   PWM_DutyUpdata(g_sDrive.u16CurrPwmDuty); 

   
   if(g_sMotor.uStaCnt.u8Run >= cRUN_1ms_CNT)    
   {
      g_sMotor.uStaCnt.u8Run = cRUN_1ms_CNT;  
      //Wait停机条件
   }
}

void Motor_Stop_Act(void)     //停机动作执行
{
  ;

}

void Motor_Brake_Act(void)     //刹车动作执行
{
  ;

}

void Motor_Error_Act(void)      //故障动作执行
{
   ;
}

void Motor_StateCnt(u8 *pStatCnt) //状态计数
{
   if(++(*pStatCnt) > 250)
   { *pStatCnt = 250; }
}

void Task_DisableIntDependOnMotorStat(void) //关闭关键中断
{
   R_TMRD0_Stop();          //关闭PWM上升中断 TMRD
   R_ADC_Stop();            //关闭ADC中断(由PWM上升沿触发)
   R_TAU0_Channel0_Stop();  //关闭CH0换相时间监控 CH1延时换相设定
   R_TAU0_Channel1_Stop();
}

//获得机器坡度
void GetMotionGrade(void)
{
   if(TESTBIT(g_sMotor.u8MotorRecvBuf[3],7))  //下坡:协议定义高位为1表示负数  
   {
      g_sObstacl.Grade = -((u16)(256)*(g_sMotor.u8MotorRecvBuf[3]&0x7f)+g_sMotor.u8MotorRecvBuf[4]);
   }   
   else                                       //上坡
   { 
      g_sObstacl.Grade = (u16)(256)*g_sMotor.u8MotorRecvBuf[3]+g_sMotor.u8MotorRecvBuf[4]; //获得机器坡度
   }
}












