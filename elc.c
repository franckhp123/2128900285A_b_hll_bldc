
#include "macrodriver.h"
#include "elc.h"
#include "userdefine.h"

/***********************************************************************************************************************
* Function Name: R_ELC_Create
* Description  : This function initializes the ELC module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_ELC_Create(void)
{
    /* regset ELSELR[!FChannel] */
    /* regset ELSELR[!FChannel] */
    /* regset ELSELR[!FChannel] */
    /* regset ELSELR[!FChannel] */
    /* regset ELSELR[!FChannel] */  
    /* regset ELSELR17 */
//比较器检测0,链接到定时器RD0,脉冲输出的强制截止:在发生ELC事件2个或者3个定时器RD的运行时钟周期后变为强制截止状态	
    ELSELR17 = _05_ELC_EVENT_LINK_TMRD0; 
}
/***********************************************************************************************************************
* Function Name: R_ELC_Stop
* Description  : This function stops the ELC event resources.
* Arguments    : event -
*                    event resources to be stoped (bit n for ELSELRn)
* Return Value : None
***********************************************************************************************************************/
void R_ELC_Stop(uint32_t event)
{
    volatile uint32_t   w_count;
    volatile uint8_t  * sfr_addr;

    sfr_addr = &ELSELR00;
    
    for (w_count = 0U; w_count < ELC_DESTINATION_COUNT; w_count++) 
    {
        if (0x1U == ((event >> w_count) & 0x1U))
        {
            *sfr_addr = _00_ELC_EVENT_LINK_OFF;
        }
        
        sfr_addr++;
    }
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
