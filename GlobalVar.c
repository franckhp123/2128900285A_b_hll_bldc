/************************************************************
   includes
************************************************************/
#include "macrodriver.h"
#include "userdefine.h"
#include "BLDC.h"

U_EVENT_TRIG_FLAG_T   g_sFlagEve;        //�¼�����λ��
U_ERROR_FLAG_T        g_sFlagErr;        //����λ��   
//U_ERROR_T             g_sFlagError;      //����λ��          
S_FILT_DATA_T         g_sBusVdc;         //ĸ�ߵ�ѹ
S_FILT_DATA_T         g_sBusIdc;         //ĸ�ߵ���
S_FILT_DATA_T         g_sMosTmp;         //MOS�¶�
E_SYS_STA_T           g_eSysState;       //ϵͳ����״̬
S_MOTOR_T             g_sMotor;          //�����ر���
S_DRIVE_T             g_sDrive;          //������ر���
S_PHASE_AD_CHANNEL_T  g_sPhaseAdChannel; //��ͨ��/�����Ƶ�ѹͨ��  
S_FILT_DATA_T         g_sPeakIdc;        //��ֵ����



E_ADC_CHANNEL_T eaAdFifoChannelBuf[3];   //AD FIFOͨ������  


//
FNP_VOID_T  g_fnpaCommutationTab[7]={0,0,0,0,0,0,0}; //�������ָ������
u8          g_u8CommAdcIndex=0;         //����ͨ������ֵ     
u8          g_u8FluxBemfStat=0;         //flux bemf�л�״̬
u8          g_u8FluxToBemfFlag=0;       //����ͨ���л�
u8          g_u8PwmToADelayTimeFlag=0;  //tmrd��ʱ����   
u16         g_u16LastBemfAd=0;          //��һ�η��綯��ֵ
u8          g_u8CommutationCnt=0;       //�������
u8          g_u16TimeOutCnt=0;          //���೬ʱ����
u8          g_u8NoAdcStartFlag=0;       //δִ���껻�༴ʹ����tmrdҲ����������ADC
u8          g_u8AdcIndex = 0;           //PWM-on AD��������  
u16         g_u16ADCBuffer[12]={0};     //ADC buf  
u16         g_u16L_dIs_dt=0;            //fluֵ   
u8          g_bCommutationOK=0;
u8          g_aNextPhase[7]={0,3,6,2,5,1,4};   //��һ����λ���  
u8          g_u16BlockCnt=0;               //��ת����
u16         u16TimerRegBuf = 0;            //���ζ�ʱ��ֵ
u16         u16LastZeroCrossTimerReg = 0;  //���ι��㶨ʱ��ֵ

u16         g_aTzc[3]={0,0,0};             //����ʱ������    
//u16         g_aTzc[6]={0,0,0,0,0,0};             //����ʱ������


u8          g_u8ZeroCrosscnt=0;            //���綯�ƹ������
u16         g_u16ZeroCompareVal=0;         //���綯�ƹ���Ƚ�ֵ    

u16         g_u16LastZeroTimerVal=0;       //��һ�ι��㶨ʱ��ֵ  
u16         g_u16NowZeroTimerVal=0;        //���ι��㶨ʱ��ֵ

U_BEMF_AD_RES_T  uBemfAdRes={0,0,0,0,0,0,0,0}; //��ռ�ձ���AD FIFO�������(�й���ͨ��)     
E_ADC_CHANNEL_T  eaComAdChannel[4] = {MosTmp_ADCH,IAvr_ADCH,BusVdc_ADCH,UVW_CMM_ADCH}; //����ͨ������
//E_ADC_CHANNEL_T  eaComAdChannel[2] = {MosTmp_ADCH,IAvr_ADCH}; //����ͨ������

U_FLUX_AD_RES_T  uFluxAdRes={0,0,0,0}; //��ռ�ձ���AD FIFO�������(�ʹ��������������)
S_FLUX_VAR_T     uFluxVar={0,0,0};     //����������ر���ʵ��ֵ

//S_INC_PI_PARA_T  g_sSpeedIncPI={0,0,0,0,0,0}; //�ٶ�����ʽPI       
S_INC_PI_PARA_T  g_sSpeedIncPI; //�ٶ�����ʽPI 
PID_LocTypeDef   g_sSpeedLocPi; //�ٶ�ֱ��ʽPI

S_OBSTACL_T      g_sObstacl;    //���Ͻṹ��      

//u8 MosOverTmpErrCnt=0; //mos���¹��ϼ�����

volatile u8 phaseSetor = 0;
   


//Debug ����
//u8 d_u8UartRecvFinish=0;
//u8 d_RecvBuf=0;

uint8_t uHallValue;
//const uint8_t HallTable[6]={1,3,2,6,4,5};
uint8_t MotorPosition;
uint8_t uHallValueExpect;
uint8_t HallAry[200];
uint8_t MotorRunningFlag;
uint8_t uTimeCntBlock;
uint8_t MotorDirection;
volatile uint8_t hall_port_value=0;      //HALL�˿�ֵ
uint16_t VrAdValue;
uint16_t TargetDuty;
uint16_t CurrentDuty;
uint8_t DischargeReq;

uint8_t mu8TestKey = 0;
uint16_t gu16OffDelay = 0;
uint8_t Flag_1ms=0;
u8 uart1_test_data=0;
 uint8_t s_u8BrkOnceFlag = 0;
 uint16_t  block_cnt=0;

#if 0
//����-----------------------------------
uint8_t     u8Flag_1ms = 0;




















uint8_t     g_u8LedShowMode=0;       // ����������ʾģʽ ��ͬ��ѹ��Χ��ʾ�Ƹ���
uint8_t     g_u8MosCheckCnt=0;       //mos�Լ����
uint8_t     g_u8ReadFlashMosCheck=0; //��ȡflash�м�¼MOS�Լ���









S_FILT_DATA_T   g_sBusVdc;  //ĸ�ߵ�ѹ
S_FILT_DATA_T   g_sVR;      //�����ź�
S_FILT_DATA_T   g_sBatTemp; //����¶�
S_FILT_DATA_T   g_sMosTemp; //MOS�¶�

//����ר��
fsl_descriptor_t my_fsl_descriptor_t;
fsl_write_t      my_fsl_write_t;
uint8_t          data_buff[4]={0,0,0,0};
uint32_t         Parameter=0;
uint16_t         BlankAddress=0;




#endif

















