
#ifndef __BLDC_H
#define __BLDC_H

#include "iodefine.h"
#include "macrodriver.h" 
#include "userdefine.h"
#include "PublicWare.h"
#include "adc.h"
#include "tau.h"
#include "tmrd.h"


//���Ͷ���
typedef struct
{
   u8 u8Standby;   // 0 ����
   u8 u8Boot;      // 1 �Ծ�
   u8 u8Position;  // 2 ��λ
   u8 u8Run;       // 3 �ջ�����
   u8 u8Stop;      // 4 ͣ��
   u8 u8Brake;     // 5 ɲ��
   u8 u8Error;     // 6 ����
}U_MOTOR_STA_T; //�������״̬����

typedef struct
{
   u16 bStandbyFinish   :1;  // 0  ����״̬���,�����������״̬
   u16 bBootFinsih      :1;  // 1  �Ծ�״̬���,�����������״̬
   u16 bPositionFinish  :1;  // 2  ��λ�Ƕ�״̬���,�����������״̬
   u16 bStartFinish     :1;  // 3  ����״̬���,�����������״̬
   u16 bRunFinish       :1;  // 4  �ջ�����״̬���,�����������״̬
   u16 bStopFinish      :1;  // 5  ͣ��״̬���,�����������״̬
   u16 bBrakeFinish     :1;  // 6  ɲ��״̬���,�����������״̬
   u16 bErrorFinish     :1;  // 7  ����״̬���,�����������״̬
   u16 bAlarmFinish     :1;  // 8  �澯״̬���,�����������״̬
   u16 bPowerOffFinish  :1;  // 9  �ػ�����״̬���,�����������״̬
   u16 reserved10       :1;  // 10 Ԥ��
   u16 reserved11		:1;  // 11 Ԥ��
   u16 reserved12		:1;  // 12 Ԥ��
   u16 reserved13		:1;  // 13 Ԥ��
   u16 reserved14		:1;  // 14 Ԥ��
   u16 reserved15		:1;  // 15 Ԥ��
}U_MOTOR_STAT_FLAG_T; //���״̬�л��¼���־����

typedef struct  //���״̬�л��¼�����λ
{
   u16 bInsertImpulse    :1;  // 0  ��Ҫע�������־
   u16 bFluxStartRead    :1;  // 1  ��ʼ��ȡ������־
   u16 bFluxToBemf       :1;  // 2  �ôӴ����л������ٱ�־
   u16 bDisBemfCalcu     :1;  // 3  ��ֹ�����������־
   u16 bBemfRising       :1;  // 4  ������������־
   u16 bBemfFalling      :1;  // 5  �������½���־
   u16 bGetBemfZeroCross :1;  // 6  �ѻ�ù�����־
   u16 bDelayTime        :1;  // 7  ����ɹ�
   u16 bfluxCommuSucess  :1;  // 8  ��������ɹ���־                 
   u16 bRecvFinish       :1;  // 9  ���ݽ�����ɱ�־
   u16 bCrash            :1;  // 10 ��ײ��־
   u16 reserved11        :1;  // 11 Ԥ��
   u16 reserved12        :1;  // 12 Ԥ��
   u16 reserved13        :1;  // 13 Ԥ��
   u16 reserved14        :1;  // 14 Ԥ��
   u16 reserved15        :1;  // 15 Ԥ��
}U_MOTOR_WORK_FLAG_T; //���������־����

typedef enum
{
   eSTANDBY_ACT,   // 0 ͣ��
   eBOOT_ACT,      // 1 �Ծ�
   ePOSITION_ACT,  // 2 ��λ�Ƕ�
   eSTART_ACT,     // 3 ����
   eRUN_ACT,       // 4 ����
   eSTOP_ACT,      // 5 ͣ��
   eBRAKE_ACT,     // 6 ɲ��
   eERROR_ACT,     // 7 ����
}E_MOTOR_STAT_T;

typedef enum
{
   eNULL_PHASE = 0, //��Ч��λ
   eUV = 1,
   eWU = 2,
   eWV = 3,
   eVW = 4,
   eUW = 5,
   eVU = 6,
}E_MOTOR_PHASE_T; //BLDC��λ

typedef enum  //CCW��CW����:�ӵ���������������ƽ�е��ӽ�ȥ��,���������
{
   eCW,   //��ת
   eCCW,  //��ת
   eMID   //�м�
}E_MOTOR_DIR_T;  //�����ת����

typedef struct
{
  u8   u8SampCnt; // ��������
  u8   u8Shift;   // ��λ����
  u8   u8Cnt;     // �ۼӼ���
  u16  u16Max;    // �޷������ֵ
  u16  u16Min;    // �޷�����Сֵ
  u32  u32Sum;    // �޷����ۼ�ֵ
  s32  i32Max;    // �з������ֵ
  s32  i32Min;    // �з�����Сֵ
  s32  i32Sum;    // �з����ۼ�ֵ
}S_EXTREMUM_MEAN_FILT_T; // ȥ��ֵƽ���˲��ṹ��  

typedef struct
{
   U_MOTOR_STA_T           uStaCnt;        //״̬ʱ�������  
   U_MOTOR_STAT_FLAG_T     uStatFlag;      //״̬��־
   U_MOTOR_WORK_FLAG_T     uWorkFlag;      //������־
   E_MOTOR_DIR_T           eDir;           //����ת��
   E_MOTOR_DIR_T           eDirBuf;        //����ת�򻺴�
   E_MOTOR_DIR_T           eTargetDir;     //Ŀ��ת��
   E_MOTOR_STAT_T          eAct;           //��ǰ����״̬
   uint8_t                 u8Phase;        //��ǰ��λ
   S_EXTREMUM_MEAN_FILT_T  sSpeedFilt;     //����ٶ�ȥ��ֵƽ���˲�     
   u16                     u16Speed;       //��ǰת��
   u16                     u16RefSpeed;    //����ת��(�ٶȻ�����)
   u16                     u16TargetSpeed; //Ŀ��ת��(�û�����)
   u8                      u8TimeCntForceChgPhase; //ǿ�ƻ���ʱ��
   u8                      u8TimeCntFluxToForce;   //������ǿ��ʱ�������
   u8                      u8MotorRecvBuf[6];      //�����������ݻ���
   u8                      u8MotorSendBuf[4];      //�����������ݻ���
   u8                      RecvCnt;                //�����������ݼ�����
   u16                     u16BrakeTime;            //ɲ��ʱ��
   u16                     u16TimeInterPI;         //����PI��ʱ��
   u16                     u16ZeroCurAD;           //���������AD
   u16                     u16CurrADBuf[4];        //��������AD
   u8                      u8MotorStatus;          //���״̬
   u8                      u8MotorErrStaTxCnt;     //�������״̬���ʹ���
   u16                     PIRefSpeedUpValue;      //PI�������ٶȲ���ֵ  
   u8                      u8StartChagedPhaseCnt;  //����������� 
   u8                      u8PWMPerionIntCnt;      //PWM�����жϼ�����  
   u8                      u8WorkModeInRun;        //�������ģʽ  
   u16                     u16TimeUpInRun;         //����ģʽ�µ����������ʱ��(��PI����)
}S_MOTOR_T;  //����ṹ��

//ADͨ��ö��    
typedef enum
{
   ADC_CH0  = _00_AD_INPUT_CHANNEL_0,  //AD0   BEMFV
   ADC_CH1  = _01_AD_INPUT_CHANNEL_1,  //AD1   UVW-COMM
   ADC_CH2  = _02_AD_INPUT_CHANNEL_2,  //AD2   BUSVDC
   ADC_CH3  = _03_AD_INPUT_CHANNEL_3,  //AD3   ADS
   ADC_CH16 = _10_AD_INPUT_CHANNEL_16, //AD16 
   ADC_CH17 = _11_AD_INPUT_CHANNEL_17, //AD17  MOSTMP
   ADC_CH18 = _12_AD_INPUT_CHANNEL_18, //AD18  BEMFW
   ADC_CH19 = _13_AD_INPUT_CHANNEL_19, //AD19  BEMFU
   ADC_PGAO = _14_AD_INPUT_PGAO,       //ADPGAO  
}E_ADC_CHANNEL_T; //ADCͨ��

typedef struct
{
   E_ADC_CHANNEL_T eaConduction[6]; //λ�úŶ�Ӧ�ĵ�ͨ��ͨ����
   E_ADC_CHANNEL_T eaBemf[6];       //λ�úŶ�Ӧ�ķ�����ͨ����
}S_PHASE_AD_CHANNEL_T; //BLDC��λADͨ���ṹ������        

typedef struct
{
   //u16 pwmDuty;             //��ǰռ�ձ�
   //u16 u16RefPwmDuty;       //����ռ�ձ�
   u16 u16CurrPwmDuty;     //��ǰռ�ձ�
   u16 u16CurrPwmDutyBuf;  //��ǰռ�ձȻ���(����PWM�ж���)
   u16 u16TargetPwmDuty;   //Ŀ��ռ�ձ�
}S_DRIVE_T;//�����ṹ��(����ģʽռ�ձȵ�λ��)

typedef struct
{
   u32 u16Conduction;     //��ͨ���ѹ
   u32 u16Bemf;           //���������ѹ
   u32 u16PhaseCurr;      //�����
}S_FLUX_VAR_T; //��������ṹ���������

typedef enum
{
   eSTAG0_1stINT,  // 0 �׶�0 ��1���ж�   
   eSTAG0_2stINT,  // 1 �׶�0 ��2���ж�
   eSTAG1_1stINT,  // 2 �׶�1 ��1���ж�
   eSTAG1_2stINT,  // 3 �׶�1 ��2���ж�
   eSTAG2_1stINT,  // 4 �׶�2 ��1���ж�
   eSTAG2_2stINT,  // 5 �׶�2 ��2���ж�
}E_MOTOR_RUN_SUB_STAT_T; //���������״̬

//�������
#define FLUX_COMMUATION_ON_CNT  3     //�׶�0 ��������ɹ�����ֵ(�ﵽ���ֵ���ӽ׶�0�л����׶�1)
#define STAG0_FLUX_TIMEOUT_CNT  500   //�׶�0 �������೬ʱʱ��ֵ(*PWM����US) 20=1ms
#define STAG1_FLUX_TIMEOUT_CNT  250   //�׶�1 �������೬ʱʱ��ֵ(*PWM����US)

//ǿ���������
#define FORCE_COMMUATION_ON_CNT  3         //�׶�0 ǿ������ɹ�����ֵ(�ﵽ���ֵ���ӽ׶�0�л����׶�1)
#define STAG0_FORCE_TIMEOUT_CNT  500       //�׶�0 ǿ�����೬ʱʱ��ֵ(*PWM����US) 20=1ms
#define STAG1_FORCE_TIMEOUT_CNT  250       //�׶�1 ǿ�����೬ʱʱ��ֵ(*PWM����US)
#define FORCE_TO_BEMF_ZERO_CROSS_OK_CNT 6  //ǿ���׶�������N�ι������ɹ����ǿ���л���������(��ֵ����<6)


//���������
#define FLUX_TO_BEMF_ZERO_CROSS_OK_CNT 3 //�����׶�������N�ι������ɹ���Ӵ����л���������(��ֵ����<3)


//�������� 
void UH_VL(void);
void WH_VL(void);
void UH_WL(void);
void WH_UL(void);
void VH_WL(void);
void VH_UL(void);
void PwrDrv_UV_On(E_MOTOR_DIR_T eMotorDir);
void PwrDrv_UW_On(E_MOTOR_DIR_T eMotorDir);
void PwrDrv_VW_On(E_MOTOR_DIR_T eMotorDir);
void PwrDrv_VU_On(E_MOTOR_DIR_T eMotorDir);
void PwrDrv_WU_On(E_MOTOR_DIR_T eMotorDir);
void PwrDrv_WV_On(E_MOTOR_DIR_T eMotorDir);
void NullPhase(void);
void UH_VL_HPwmLON(void);
void UH_WL_HPwmLON(void);
void VH_WL_HPwmLON(void);
void VH_UL_HPwmLON(void);
void WH_UL_HPwmLON(void);
void WH_VL_HPwmLON(void);
void BLDC_ConsumeBrake(void); 
void BLDC_StopMotor(void);
void PWM_DutyUpdata(u16 duty);  
void PwmDrv_PwmOutDisable(void);//��6��  
void BLDC_CommutationTableLoad(FNP_VOID_T fnpaCommutationTab[],E_MOTOR_DIR_T eMotorDir);//���뻻���
void BLDC_PhaseConducting(const FNP_VOID_T fnpaCommutationTab[],u8 u8Phase); //���ݳ�ʼλ�õó���һ����ͨ��
void BLDC_PhaseAdChannelConfig(E_MOTOR_DIR_T eMotorDir,S_PHASE_AD_CHANNEL_T * psPhaseAdChannel);
void BLDC_u8ImpulsePosition_2Phase3Dir_Fix(E_MOTOR_DIR_T eMotorDir); //���嶨λ(��ʼλ�ü��)���ർͨ,3����
u8 BLDC_u8ImpulsePosition_2Phase6Dir_Fix(E_MOTOR_DIR_T eMotorDir,u16 invaildThrval);//���嶨λ(��ʼλ�ü��)���ർͨ,6����
u8 Func_u8JudgeOneDataLessthanThrVal(const s16 a [ ],u8 ln,s16 thrval);//�ж��������Ƿ���һ����С�ڹ涨��ֵ
u8 Func_u8FindMaxDataPosition_DirectCompare(const s16 a[],u8 ln);//��ֱ�ӱȽϷ�ʽѰ��һά�����е����ֵ��λ��
void BLDC_FluxCalcuCommutation(S_MOTOR_T *psMotor,const U_FLUX_AD_RES_T *puFluxAdres);
u16 u16FluxNullCalcu(const S_FLUX_VAR_T *psFluxVar,uint16_t L_dIs_dt); //��Ч��λ,�ռ���
u16 BLDC_u16FluxFormulaCalcu(const S_FLUX_VAR_T *psFluxVar,u16 L_dIs_dt);
u16 BLDC_u16FluxFormulaCalcu2(const S_FLUX_VAR_T * psFluxVar,u16 L_dIs_dt);
u8 BLDC_u8BemfZeroCrossDetect_LowDuty(S_MOTOR_T * psMotor,uint16_t u16BusVdcHalfAd,uint16_t u16BemfAd);
u8 BLDC_u8BemfZeroCrossDetect_MidDuty(S_MOTOR_T *psMotor,u16 BusVdcHalfAd,u16 BemfAd);
u8 BLDC_u8BemfZeroCrossDetect_MidHighDuty(S_MOTOR_T *psMotor,u16 u16BusVdcHalfAd,u16 aAdcRes[ ]);
u8 BLDC_u8BemfZeroCrossDetect_HighDuty(S_MOTOR_T * psMotor,u16 u16BusVdcHalfAd,u16 aAdcRes[ ]);
u8 BLDC_u8ZeroCrossTimeToSpeed(S_MOTOR_T *psMotor,u16 aZeroCrossTime[]); 
void BLDC_ForceChangePhase(S_MOTOR_T *psMotor);
void HallISR(void);
uint8_t Hall_Get(void);
uint8_t GetMotorPosition(uint8_t hallvalue);
void Commutate_Phase(uint8_t Position);
void Self_Commutate_Phase(uint8_t Position);
void Motor_Stop(void);
void Motor_Brake(void);
void Cap_PreCharge(uint8_t Position);
uint8_t Expect_Hall_Value(uint8_t Position);
void Normal_Change_Phase(void);
void Pwm_Update(void);
uint16_t PwmDutyAdjStepCtrl(uint16_t targetAdjPwmDuty, uint16_t currAdjPwmDuty);
void PWM_DutyUpdata(u16 duty); //4799    
extern uint8_t int_flag;

#define   CW     0
#define   CCW    1
#define AH_IO	0b00100000
#define	BH_IO	0b00001000
#define CH_IO	0b00000100

//#define AL_IO	0b00000001
//#define BL_IO	0b00000010
//#define CL_IO	0b00010000

#define AL_IO	0b00010000
#define BL_IO	0b00000010
#define CL_IO	0b00000001
#define AH_PWM	0b11111101
#define	BH_PWM	0b11101111
#define CH_PWM	0b11011111

//#define AH_PWM	0b11111011
//#define BH_PWM	0b11110111
//#define CH_PWM	0b11011111



#define AL_PWM	0b11110111
#define BL_PWM	0b10111111
#define CL_PWM	0b01111111

/* 10K Frequency PWM macro define */
/* PWM duty set for open loop speed */

#define	PWM_ADJUST_LOW	(PWM_MAX/20)   //%5
#define	PWM_ADJUST_HIGH	(PWM_MAX*80L/100)  //80%    2158
#define  TRUE		1
#define  FALSE		0


#endif

#define PWM_DELTAP	(PWM_MAX/1000)  
#define PWM_DELTAN	(PWM_MAX/10)
#define DUTY_STEP	(PWM_MAX/500)
#define PWM_ZERO (0) //

/* Hall ON OFF */
#define	HALL_INT_ON() 	R_INTC0_Start();R_INTC4_Start();R_INTC3_Start()
#define	HALL_INT_OFF() 	R_INTC0_Stop();R_INTC4_Stop();R_INTC3_Stop()
void DelayXus(uint16_t us);

#define PWM_PERIOD  4799

void ClosePwmAndIo(void);


#define PWM_MAX (PWM_PERIOD)       //2698

#define DUTY_FULL               		PWM_MAX//4799//TRDGRA0 // ��ռ�ձ�
#define DUTY_MIN                        ((uint16_t)(DUTY_FULL * 0.1))//7.2   // ��Сռ�ձ�

#define DUTY_8                       	((uint16_t)(DUTY_FULL * 0.08))
#define DUTY_10                      	((uint16_t)(DUTY_FULL * 0.10))
#define DUTY_15                      	((uint16_t)(DUTY_FULL * 0.15))
#define DUTY_18                      	((uint16_t)(DUTY_FULL * 0.18))
#define DUTY_20                      	((uint16_t)(DUTY_FULL * 0.20))
#define DUTY_25                      	((uint16_t)(DUTY_FULL * 0.25))
#define DUTY_30                      	((uint16_t)(DUTY_FULL * 0.30))
#define DUTY_35                      	((uint16_t)(DUTY_FULL * 0.35))
#define DUTY_40                      	((uint16_t)(DUTY_FULL * 0.40))
#define DUTY_45                      	((uint16_t)(DUTY_FULL * 0.45))
#define DUTY_50                      	((uint16_t)(DUTY_FULL * 0.50))
#define DUTY_51                      	((uint16_t)(DUTY_FULL * 0.51))
#define DUTY_60                      	((uint16_t)(DUTY_FULL * 0.60))
#define DUTY_70                      	((uint16_t)(DUTY_FULL * 0.70))
#define DUTY_80                      	((uint16_t)(DUTY_FULL * 0.80))
#define DUTY_85                      	((uint16_t)(DUTY_FULL * 0.85))
#define DUTY_90                      	((uint16_t)(DUTY_FULL * 0.90))
#define DUTY_95                         ((uint16_t)(DUTY_FULL * 0.95))
#define DUTY_100                      	((uint16_t)(DUTY_FULL))  //4799




#define PWM_ZERO (0) //































