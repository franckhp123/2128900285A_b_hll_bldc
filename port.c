
#include "macrodriver.h"
#include "port.h"
#include "userdefine.h"
#if 0
void R_PORT_Create(void)  
{
//�˿����ֵ����Pxx
   P1 = _40_Pn6_OUTPUT_1; //lamp  s
//�˿�����ģʽ����(PIMxx)0:ͨ�������뻺���� 1:TTL���뻺����  
   	
//�˿����ģʽ���üĴ���(0:�������,1:��©���)  	
    /* regset POM6 regmode */
    /* regset POM7 regmode */
    /* regset POM13 regmode */
    /* regset PMC1 regmode */
    /* regset PMC3 regmode */
    /* regset PMC6 regmode */
    /* regset PMC7 regmode */
    /* regset PMC13 regmode */  
    /* regset ADPC */
    /* regset ADPC */
    /* regset ADPC */
    /* regset ADPC */
    /* regset ADPC */


/* 
   PM1 �˿�����/������� 0:���; 1:���� 
   
   P1.7:��; P1.6:LED; P1.5:WH; P1.4:WL; P1.3:VH; P1.2:UH; P1.1:VL; P1.0:UL;
   P3.3:��;  P3.1:��; P3.0:��;
   P5.5:��;  P5.1:Tx; P5.0:Rx;  
   P6.6:��;  P6.2:��; P6.1:��;  P6.0:��;
   P7.7:��;  P7.0:��;      
*/
 PM1 = _01_PMn0_MODE_INPUT|_02_PMn1_MODE_INPUT|_04_PMn2_MODE_INPUT|_08_PMn3_MODE_INPUT|
       _10_PMn4_MODE_INPUT|_20_PMn5_MODE_INPUT|_00_PMn6_MODE_OUTPUT|_80_PMn7_MODE_INPUT;;    

//�˿�ģʽ���ƼĴ���PMCxx,0:��������/��� 1:ģ������(��AD������)  

   P1_bit.no6 = 0;   //P1.7���0--Debug
   PM1_bit.no6 = 0;  //P1.7����Ϊ�����--Debug   
   
}
#endif
#if 1
void R_PORT_Create(void)
{
    P0 = _02_Pn1_OUTPUT_1;
    P1 = _00_Pn7_OUTPUT_0 | _40_Pn6_OUTPUT_1;
    P3 = _00_Pn1_OUTPUT_0;
    P6 = _02_Pn1_OUTPUT_1 | _01_Pn0_OUTPUT_1;
    P7 = _01_Pn0_OUTPUT_1;
    P14 = _00_Pn7_OUTPUT_0;
    POM1 = _00_POMn7_NCH_OFF;
    PMC0 = _FC_PMC0_DEFAULT_VALUE | _00_PMCn1_NOT_USE;
    PMC14 = _7F_PMC14_DEFAULT_VALUE | _00_PMCn7_NOT_USE;
    PM0 = _FC_PM0_DEFAULT_VALUE | _00_PMn1_MODE_OUTPUT;
    PM1 = _00_PMn7_MODE_OUTPUT | _00_PMn6_MODE_OUTPUT;
    PM3 = _FC_PM3_DEFAULT_VALUE | _00_PMn1_MODE_OUTPUT;
    PM6 = _F8_PM6_DEFAULT_VALUE | _04_PMn2_MODE_INPUT | _00_PMn1_MODE_OUTPUT | _00_PMn0_MODE_OUTPUT;
    PM7 = _FE_PM7_DEFAULT_VALUE | _00_PMn0_MODE_OUTPUT;
    PM14 = _7F_PM14_DEFAULT_VALUE | _00_PMn7_MODE_OUTPUT;
}
#endif 
