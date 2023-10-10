
#include "macrodriver.h"
#include "sau.h"
#include "userdefine.h"
void R_SAU0_Create(void)   
{
   SAU0EN = 1U;    //�����ṩ����ʱ��     
   NOP();
   NOP();
   NOP();
   NOP();
//����ʱ��ѡ��Ĵ���:���ô��ڿ�ѡ�����������ʱ��CK00��CK01;CK00=24M/2^2=6M;CK01=24M/2^2=6M;  
  SPS0 = _0020_SAU_CK01_fCLK_2 | _0002_SAU_CK00_fCLK_2;
  
//����ʱ��ѡ��Ĵ���:���ô��ڿ�ѡ�����������ʱ��CK00��CK01;CK00=24M/2^4=1.5M;CK01=24M/2^1=12M;   
  //SPS0 = _0040_SAU_CK01_fCLK_4 | _0001_SAU_CK00_fCLK_1;

#if 0
//����ʱ��ѡ��Ĵ���:���ô��ڿ�ѡ�����������ʱ��CK00��CK01;CK00=24M/2^3=3M;CK01=24M/2^3=3M;
//SPS0 = _0000_SAU_CK01_fCLK_0 | _0000_SAU_CK00_fCLK_0; //fclk(24M)
   SPS0 = _0030_SAU_CK01_fCLK_3 | _0003_SAU_CK00_fCLK_3;
#endif

   R_UART0_Create();
   //R_SAU0_Create_UserInit();
}
/***********************************************************************************************************************
* Function Name: R_UART0_Create
* Description  : This function initializes the UART0 module.
                 ����ͨ��0:UART0-Tx
                 ����ͨ��1:UART0-Rx
                 ����ͨ��2:UART1-Tx
                 ����ͨ��3:UART1-Rx
* Arguments    : None
* Return Value : None  
***********************************************************************************************************************/
void R_UART0_Create(void)
{
//����ͨ��ֹͣ�Ĵ���,ST00=1:��SEmn��0ת�Ƶ�ͨ�Ŵ���״̬��,ST00�Զ���0     
   ST0 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON; //ֹͣͨ��0(UART0����)��ͨ��1(UART0����)

   STMK0 = 1U;     //��ֹINTST0�жϴ���(UART0���ͽ����򻺳������ж�)  
   STIF0 = 0U;     //��INTST0�жϱ�־
    
   SRMK0 = 1U;     //��ֹINTSR0(UART0���ս���)�жϴ���
   SRIF0 = 0U;     //��INTSR0�жϱ�־
    
   SREMK0 = 1U;    //��ֹINTSRE0���մ����ж�(����UART0 ���յ�ͨ�Ŵ���)
   SREIF0 = 0U;    //��INTSRE0�жϱ�־
    
   SRPR10 = 1U;    //����UART0�����ж�(INTSR0)���ȼ�3   
   SRPR00 = 1U;

   STPR10 = 1U;    //����UART0�����ж�(INTST0)���ȼ�3
   STPR00 = 1U;

/* ����ͨ��0 UART0���� */	
   //SIR00 = _0004_SAU_SIRMN_FECTMN|_0002_SAU_SIRMN_PECTMN|_0001_SAU_SIRMN_OVCTMN;//���б�־��������Ĵ���:֡�����־,��żУ������־,��������־
//����ģʽ�Ĵ���SMR00  
//CKS00=0����ʱ��ѡ��CK00(3M); CCS00=0:����ʱ��ѡ��Ϊcks00ָ����ʱ��fMCK�����ⲿ����ʱ��;
//MD002 MD001=01:UARTģʽ; MD000(�ж�Դѡ��) 0:ѡ���ͽ����жϷǻ��������ж�
#if 1   
   SMR00 = _0020_SMR00_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
           _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
#endif
#if 0
   SMR00 = _0020_SMR00_DEFAULT_VALUE | _8000_SAU_CLOCK_SELECT_CK01 | _0000_SAU_CLOCK_MODE_CKS | 
           _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
#endif
//����ͨ�������趨�Ĵ��� SCR00
//TXE00 RXE00=10:ֻ���з���;DAP00 CKP00=00��UARTģʽ������Ϊ00;EOC=0:��ֹ���������ж�INTSRE0
//PTC001 PTC000=00:����żУ��λ����������żУ��λ����;DIR=1:���ȷ���/���յ�λ;
//SLC001 SLC000=01:1λֹͣλ;DLS001 DLS000=11:8λ���ݳ���
   SCR00 = _0004_SCR00_DEFAULT_VALUE | _8000_SAU_TRANSMISSION | _0000_SAU_TIMING_1 | _0000_SAU_INTSRE_MASK | 
            _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
/* �������ݼĴ��� SDR00 */
//������={����ͨ��������ʱ��(fMCK=3M)Ƶ��}/(SDRmn[15:9]+1)/2[bps]    
//SDR00 = _5C00_SAU0_CH0_BAUDRATE_DIVISOR;     
//SDR00 = _9A00_SAU0_CH0_BAUDRATE_DIVISOR; //9A(1001 1010)->SDR00[15:9]=0b1001101=77->3M/(77+1)/2=19230bps   
   SDR00 = _9A00_SAU0_CH0_BAUDRATE_DIVISOR; //9A(1001 1010)->SDR00[15:9]=0b1001101=77->6M/(77+1)/2=38461bps
                                            //9A(1001 1010)->SDR00[15:9]=0b1001101=77->12M/(77+1)/2=76923bps    
/* ����ͨ��1 UART0���� */	 
   NFEN0 |= _01_SAU_RXD0_FILTER_ON; //SNFEN00=1,����RxD0����ʱ,��λǿ����1,ʹ��RxD0���ŵ������˲���
//���б�־��������Ĵ���:֡�����־,��żУ������־,��������־
   SIR01 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN  | _0001_SAU_SIRMN_OVCTMN ;
   SMR01 = _0020_SMR01_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
		   _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
   SCR01 = _0004_SCR01_DEFAULT_VALUE | _4000_SAU_RECEPTION | _0000_SAU_TIMING_1 | _0400_SAU_INTSRE_ENABLE | 
		   _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
   SDR01 = _9A00_SAU0_CH1_BAUDRATE_DIVISOR;

/* ����UART0���(ͨ��0 Tx) */   
   SO0 |= _0001_SAU_CH0_DATA_OUTPUT_1;             //����ͨ��0(UART0)���ֵΪ1
   SOL0 &= (uint16_t)~_0001_SAU_CHANNEL0_INVERTED; //����ͨ��0(UART0)����ֱ�����,�Ƿ������
   SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;            //������ͨ��0(UART0-Tx)���

   PM5 |= 0x01U;	   //UART0-RX P50:�����  
   PM5 &= 0xFDU;	   //UART0-TX P51:����� 
   P5 |= 0x02U; 	   //UART0-TX P51=1  
}
/***********************************************************************************************************************
* Function Name: R_UART0_Start
* Description  : This function starts the UART0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART0_Start(void)
{
   SO0  |= _0001_SAU_CH0_DATA_OUTPUT_1;  //����ͨ��0(UART0)���ֵΪ1
   SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;  //������ͨ��0(UART0-Tx)���
   SS0  |= _0002_SAU_CH1_START_TRG_ON | _0001_SAU_CH0_START_TRG_ON; //������ʼ:ʹSEmnΪ1��SS00/01 �Զ���0
   
   STIF0 = 0U;     //��INTST0(UART0��������ж�)�жϱ�־
   SRIF0 = 0U;     //��INTSR0(UART0��������ж�)�жϱ�־ 
   SREIF0 = 0U;     //��INTSR0(UART0��������ж�)�жϱ�־ 
   
   //STMK0 = 0U;     //����UART0�����ж�  
   STMK0 = 1U;     //��ֹUART0�����ж�
   SRMK0  = 0U;     //����UART0�����ж�  
   SREMK0 = 0U;     //����UART0�����ж�  

   //TXD0 = 0x86;
   STIF0 = 1;      //�ô���0������ɱ�־     
}
/***********************************************************************************************************************
* Function Name: R_UART0_Stop
* Description  : This function stops the UART0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_UART0_Stop(void)  
{
   STMK0 = 1U;  //��ֹUART0�����ж�   
   SRMK0 = 1U;  //��ֹUART0�����ж�
   ST0 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON; //ֹͣͨ��0(UART0����)��ͨ��1(UART0����)
   SOE0 &= (uint16_t)~_0001_SAU_CH0_OUTPUT_ENABLE;               //��ֹͨ��0(UART0-RX)���
   STIF0 = 0U;  //��UART0������ɱ�־
   SRIF0 = 0U;  //��UART0������ɱ�־
}

void UartSendData(u8 tdata)
{
   while(!STIF0); //�ȴ��ϴ����ݷ������
   TXD0 = tdata;  //װ�ط�������   
   STIF0 = 0;  
}



/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
