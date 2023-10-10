
#include "macrodriver.h"
#include "tmrd.h"
#include "userdefine.h"

/***********************************************************************************************************************
* Function Name: R_TMRD0_Create
* Description  : This function initializes the TMRD0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#if 1

/***********************************************************************************************************************
* Function Name: R_TMRD0_Create
* Description  : This function initializes the TMRD0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

void R_TMRD0_Create(void)    
{
    TRD0EN = 1U;    //�����ṩ��ʱ��RD����ʱ��  

	//CSEL1=1�ں�TRDGRA1�Ĵ����Ƚ�ƥ��󻹼�������;CSEL0=1�ں�TRDGRA0�Ĵ����Ƚ�ƥ��󻹼�������;	
    TRDSTR |= _08_TMRD_TRD1_COUNT_CONTINUES | _04_TMRD_TRD0_COUNT_CONTINUES;
	
    //TSTART1=0:TRD1ֹͣ����;TSTART0=0:TRD0ֹͣ����
    TRDSTR &= (uint8_t)~(_02_TMRD_TRD1_COUNT_START | _01_TMRD_TRD0_COUNT_START);  

    /* intpregbitset INTTRD0 disable */
    TRDMK0 = 1U;    //��ֹTRD0�ж� INTTRD0
    TRDIF0 = 0U;    //��TRD0�жϱ�־λ

    /* intpregbitset INTTRD1 disable */
    TRDMK1 = 1U;    //��ֹTRD1�ж� INTTRD1
    TRDIF1 = 0U;    //��TRD0�жϱ�־λ
		
    TRDPR10 = 0U;   //����INTTRD0���ȼ�Ϊ1    
    TRDPR00 = 1U;  
	

//ELC�¼�����0(ELCOBE0)����ǿ�ƽ�ֹ��ʱ��RD���������; ELC�¼�����1(ELCOBE1)����ǿ�ƽ�ֹ��ʱ��RD���������;
    TRDELC = _02_TMRD0_CUTOFF_ENABLED | _20_TMRD1_CUTOFF_ENABLED;   
 	
    /* regset TRDMR ContentOnly=[4-7] */    
//TRDGRD1��TRDGRB1��buf;TRDGRC1��TRDGRA1��buf;TRDGRD0��TRDGRB0��buf;TRDGRC0��TRDGRA0��buf;TRD0��TRD1��������
    TRDMR = _80_TMRD_TRDGRD1_BUFFER | _40_TMRD_TRDGRC1_BUFFER | _20_TMRD_TRDGRD0_BUFFER | _10_TMRD_TRDGRC0_BUFFER;

//0b0000 1101	ѡ��λͬ��PWMģʽ;�����ƽ���:��ʼ���L,�ߵ�ƽ��Ч;�����ƽ���:��ʼ���L,�ߵ�ƽ��Ч
    TRDFCR |= _01_TMRD_TRANSFER_RESET_SYNCHRONOUS | _04_TMRD_NORMAL_PHASE_LEVEl_LH | _08_TMRD_COUNTER_PHASE_LEVEl_LH;
  

//�����ֹ(TRDIOA0����ΪI/O);�����ֹ(TRDIOC0����ΪI/O);
//TRDIOB0(WH)�������;TRDIOD0(WL)�������;TRDIOA1(VH)�������;  
//TRDIOB1(UH)�������;TRDIOC1(VL)�������;TRDIOD1(UL)�������;  
	TRDOER1 = _01_TMRD_TRDIOA0_OUTPUT_DISABLE | _00_TMRD_TRDIOB0_OUTPUT_ENABLE | _04_TMRD_TRDIOC0_OUTPUT_DISABLE | 
              _00_TMRD_TRDIOD0_OUTPUT_ENABLE | _00_TMRD_TRDIOA1_OUTPUT_ENABLE | _00_TMRD_TRDIOB1_OUTPUT_ENABLE | 
              _00_TMRD_TRDIOC1_OUTPUT_ENABLE | _00_TMRD_TRDIOD1_OUTPUT_ENABLE;

//��ʱ�� RD ������ƼĴ���:��ʼ���L��ƽ
    TRDOCR = _00_TMRD_TRDIOC0_INITIAL_OUTPUT_L;    

//��ʱ��RD0�����˲�������ѡ��Ĵ�������:��ֹǿ�ƽ�ֹ
    TRDDF0 = _00_TMRD_TRDIOB_FORCEDCUTOFF_DISABLE | _00_TMRD_TRDIOD_FORCEDCUTOFF_DISABLE;
//��ʱ��RD1�����˲�������ѡ��Ĵ�������:��ֹǿ�ƽ�ֹ
    TRDDF1 = _00_TMRD_TRDIOA_FORCEDCUTOFF_DISABLE | _00_TMRD_TRDIOB_FORCEDCUTOFF_DISABLE | 
             _00_TMRD_TRDIOC_FORCEDCUTOFF_DISABLE | _00_TMRD_TRDIOD_FORCEDCUTOFF_DISABLE;

#if 0
    /* regset TRDDF0 */
    TRDDF0 = _20_TMRD_TRDIOB_LOW_OUTPUT | _00_TMRD_TRDIOC_FORCEDCUTOFF_DISABLE | _02_TMRD_TRDIOD_LOW_OUTPUT;
    /* regset TRDDF1 */
    TRDDF1 = _80_TMRD_TRDIOA_LOW_OUTPUT | _20_TMRD_TRDIOB_LOW_OUTPUT | _08_TMRD_TRDIOC_LOW_OUTPUT | 
             _02_TMRD_TRDIOD_LOW_OUTPUT;
#endif	
    /* regset TRDCR0 */   
	
//0b0010 0000 TRD0�ں�TRDGRA0�Ĵ����Ƚ�ƥ��ʱ���TRD0�Ĵ���;����Դѡ��fclk(24M)��fHOCO(48M)��FRQSEL4=1ѡfHOCO
    TRDCR0 = _20_TMRD_COUNTER_CLEAR_TRDGRA | _00_TMRD_INETNAL_CLOCK_FCLK_FHOCO;    
	
//��ʱ��RD0�ж�����Ĵ���:0b0000 0010 IMIEB=1������IMFBλ�������ж�(IMIB)  
    TRDIER0 = _02_TMRD_IMIB_ENABLE;  
    /* regset TRDIER1 */
    /* regset TRD0 */
   
#if 1
#if 0
    TRDGRA0 = _12BF_TMRD_TDRGRA0_VALUE;  //TRDGRA0=0x12bf=4799->fpwm=(1/48M)*(4799+1)=100us=10K  
    TRDGRB0 = _095F_TMRD_TDRGRB0_VALUE;
    TRDGRC0 = _12BF_TMRD_TDRGRA0_VALUE;
    TRDGRD0 = _095F_TMRD_TDRGRD0_VALUE;
    TRDGRA1 = _095F_TMRD_TDRGRA1_VALUE;
    TRDGRB1 = _095F_TMRD_TDRGRB1_VALUE;
    TRDGRC1 = _095F_TMRD_TDRGRC1_VALUE;
    TRDGRD1 = _095F_TMRD_TDRGRD1_VALUE;       
#endif	
#if 1
              
               TRDGRA0 = _12BF_TMRD_TDRGRA0_VALUE;  //TRDGRA0=0x12bf=4799->fpwm=(1/48M)*(4799+1)=100us=10K  
               TRDGRB0 = 0x10df;
               TRDGRC0 = _12BF_TMRD_TDRGRA0_VALUE;
               TRDGRD0 = 0x10df;
               TRDGRA1 = 0x10df;
               TRDGRB1 = 0x10df;
               TRDGRC1 = 0x10df;
               TRDGRD1 = 0x10df; 

#endif 
    


    /* Set TRDIOB0 pin */
    POM1 &= 0xDFU;      //0b1101 1111��ͨ���ģʽ P1.5(WH)      
    P1 &= 0xDFU;        //P1.5=0
    PM1 &= 0xDFU;       //P1.5����� 
    /* Set TRDIOD0 pin */
    P1 &= 0xEFU;        //P1.4=0
    PM1 &= 0xEFU;       //P1.4(WL)����� 
    /* Set TRDIOA1 pin */
    P1 &= 0xF7U;        //P1.3=0  
    PM1 &= 0xF7U;       //P1.3(VH)����� 
    /* Set TRDIOB1 pin */
    P1 &= 0xFBU;        //P1.2=0 
    PM1 &= 0xFBU;       //P1.2(UH)����� 
    /* Set TRDIOC1 pin */
    P1 &= 0xFDU;        //P1.1=0 
    PM1 &= 0xFDU;       //P1.1(VL)�����
    /* Set TRDIOD1 pin */
    POM1 &= 0xFEU;      //P1.0(UL)��ͨ���ģʽ               
    P1 &= 0xFEU;        //P1.0=0 
    PM1 &= 0xFEU;       //P1.0�����  
    #endif 
}

/***********************************************************************************************************************
* Function Name: R_TMRD0_Start
* Description  : This function starts TMRD0 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TMRD0_Start(void)
{
   volatile uint8_t trdsr_dummy;
   TRDIF0 = 0U;           //��INTTRD0�жϱ�־  
   trdsr_dummy = TRDSR0;  //��д0ǰ�ȶ�TRDSR0
   TRDSR0 = 0x00U;        //��TRD0ÿ���ж������־λ
	
   trdsr_dummy = TRDSR1;  //��д0ǰ�ȶ�TRDSR1  
   TRDSR1 = 0x00U;        //��TRD1ÿ���ж������־λ
   TRDMK0 = 0U;           //ʹ���ж�INTTRD0:��ʱ��RD0��TRDGRA0(�趨PWM����)�Ƚ�ƥ��ʱ(PWM�����ж�)
     
   TRDSTR |= _04_TMRD_TRD0_COUNT_CONTINUES; //CSEL0=1:TRD0������TRDGRA0ƥ��ʱ��������  
   TRDSTR |= _01_TMRD_TRD0_COUNT_START;     //TSTART0=1:TRD0��ʼ����  
}
/***********************************************************************************************************************
* Function Name: R_TMRD0_Stop
* Description  : This function stops TMRD0 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_TMRD0_Stop(void)   
{
   volatile uint8_t trdsr_dummy;
   TRDSTR |= _04_TMRD_TRD0_COUNT_CONTINUES;//CSEL0=1:RD0��TRDGRA0�Ĵ����Ƚ�ƥ��󻹼�������;
   TRDSTR &= (uint8_t)~_01_TMRD_TRD0_COUNT_START;//TSTART0=0:TRD0ֹͣ����
	
   TRDMK0 = 1U;          //��ֹTRD0 INTTRD0(TRD0������TRDGRA0ƥ��ʱ��������)�ж�
   TRDIF0 = 0U;          //��TRD0�жϱ�־λ
   trdsr_dummy = TRDSR0; //��дTRDSR0 0ǰ�ȶ�TRDSR0
   TRDSR0 = 0x00U;       //��TRD0ÿ���ж������־λ(OVF,IMFD,IMFC,IMFB,IMFA)
   trdsr_dummy = TRDSR1; //��дTRDSR1 0ǰ�ȶ�TRDSR1
   TRDSR1 = 0x00U;       //��TRD0ÿ���ж������־λ(OVF,IMFD,IMFC,IMFB,IMFA)
}

#endif 







