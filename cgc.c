
#include "macrodriver.h"
#include "cgc.h"
#include "userdefine.h"


void R_CGC_Create(void)   
{
   CMC = _00_CGC_HISYS_PORT | _00_CGC_SYSOSC_UNDER10M; //P121,P122为端口模式,非外部振荡器引脚

//设置fmx:时钟运行状态控制寄存器CSC.MSTOP 高速系统时钟的运行控制位
   MSTOP = 1U;     //X1振荡电路停止

//设置fmain:系统时钟控制寄存器CKC.MCM0
   MCM0 = 0U;     //选择高速内部振荡器时钟(fIH=24M）作为主系统时钟(fMAIN）
    
//能选择低速内部振荡器作为12位间隔定时器的运行时钟;能选择低速内部振荡器作为定时器 RJ 的计数源
   OSMC = _10_CGC_IT_TMRJ_CLK_FIL;

//设置fIH:时钟运行状态控制寄存器CSC.HIOSTOP 高速内部振荡器运行
   HIOSTOP = 0U;   //高速内部振荡器运行
}













