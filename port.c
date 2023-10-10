
#include "macrodriver.h"
#include "port.h"
#include "userdefine.h"
#if 0
void R_PORT_Create(void)  
{
//端口输出值配置Pxx
   P1 = _40_Pn6_OUTPUT_1; //lamp  s
//端口输入模式配置(PIMxx)0:通常的输入缓冲器 1:TTL输入缓冲器  
   	
//端口输出模式配置寄存器(0:常规输出,1:开漏输出)  	
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
   PM1 端口输入/输出配置 0:输出; 1:输入 
   
   P1.7:无; P1.6:LED; P1.5:WH; P1.4:WL; P1.3:VH; P1.2:UH; P1.1:VL; P1.0:UL;
   P3.3:无;  P3.1:无; P3.0:无;
   P5.5:无;  P5.1:Tx; P5.0:Rx;  
   P6.6:无;  P6.2:无; P6.1:无;  P6.0:无;
   P7.7:无;  P7.0:无;      
*/
 PM1 = _01_PMn0_MODE_INPUT|_02_PMn1_MODE_INPUT|_04_PMn2_MODE_INPUT|_08_PMn3_MODE_INPUT|
       _10_PMn4_MODE_INPUT|_20_PMn5_MODE_INPUT|_00_PMn6_MODE_OUTPUT|_80_PMn7_MODE_INPUT;;    

//端口模式控制寄存器PMCxx,0:数字输入/输出 1:模拟输入(在AD处配置)  

   P1_bit.no6 = 0;   //P1.7输出0--Debug
   PM1_bit.no6 = 0;  //P1.7设置为输出口--Debug   
   
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
