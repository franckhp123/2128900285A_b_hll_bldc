/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2014, 2018 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_intp.c
* Version      : Code Generator for RL78/G1G V1.01.01.03 [24 May 2018]
* Device(s)    : R5F11EBA
* Tool-Chain   : CCRL
* Description  : This file implements device driver for INTP module.
* Creation Date: 2020/11/10
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "macrodriver.h"
#include "r_cg_intp.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "userdefine.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_INTC_Create
* Description  : This function initializes INTP module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_INTC_Create(void)
    {
    /* intpregbitset INTP0 disable */
    PMK0 = 1U;      /* disable INTP0 interrupt */
    PIF0 = 0U;      /* clear INTP0 interrupt flag */
    /* intpregbitset INTP1 disable */
    PMK1 = 1U;      /* disable INTP1 interrupt */
    PIF1 = 0U;      /* clear INTP1 interrupt flag */
    /* intpregbitset INTP2 disable */
    PMK2 = 1U;      /* disable INTP2 interrupt */
    PIF2 = 0U;      /* clear INTP2 interrupt flag */
    /* intpregbitset INTP3 disable */
    PMK3 = 1U;      /* disable INTP3 interrupt */
    PIF3 = 0U;      /* clear INTP3 interrupt flag */
    /* intpregbitset INTP4 disable */
    PMK4 = 1U;      /* disable INTP4 interrupt */
    PIF4 = 0U;      /* clear INTP4 interrupt flag */
    /* intpregbitset INTP5 disable */
    PMK5 = 1U;      /* disable INTP5 interrupt */
    PIF5 = 0U;      /* clear INTP5 interrupt flag */
    /* intpregbitset INTP0 INTP=PR */
    /* Set INTP0 high priority */
    PPR10 = 0U;
    PPR00 = 0U;
    /* intpregbitset INTP1 INTP=PR */
    /* intpregbitset INTP2 INTP=PR */
    /* intpregbitset INTP3 INTP=PR */
    /* Set INTP3 high priority */
    PPR13 = 0U;
    PPR03 = 0U;
    /* intpregbitset INTP4 INTP=PR */
    /* Set INTP4 high priority */
    PPR14 = 0U;
    PPR04 = 0U;
    /* intpregbitset INTP5 INTP=PR */
    /* regset EGN0 */
    EGN0 = _10_INTP4_EDGE_FALLING_OR_BOTH | _08_INTP3_EDGE_FALLING_OR_BOTH | _01_INTP0_EDGE_FALLING_OR_BOTH;
    /* regset EGP0 */
    EGP0 = _10_INTP4_EDGE_RISING_OR_BOTH | _08_INTP3_EDGE_RISING_OR_BOTH | _01_INTP0_EDGE_RISING_OR_BOTH;
    /* Set INTP0 pin */
    /* P137 is input only, so it's unnecessary to set */
    /* Set INTP3 pin */
    PM3 |= 0x01U;       /* set bit0 to 1 */
    /* Set INTP4 pin */
    PM3 |= 0x02U;       /* set bit1 to 1 */

  //  R_INTC_Create_UserInit();
}


/***********************************************************************************************************************
* Function Name: R_INTC1_Start
* Description  : This function clears INTP1 interrupt flag and enables interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
	void R_INTC0_Start(void)
	{
		/* intpregbitset INTP0 enable */
		PIF0 = 0U;		/* clear INTP0 interrupt flag */
		PMK0 = 0U;		/* enable INTP0 interrupt */
	}
	/***********************************************************************************************************************
	* Function Name: R_INTC0_Stop
	* Description  : This function disables INTP0 interrupt and clears interrupt flag.
	* Arguments    : None
	* Return Value : None
	***********************************************************************************************************************/
	void R_INTC0_Stop(void)
	{
		/* intpregbitset INTP0 disable */
		PMK0 = 1U;		/* disable INTP0 interrupt */
		PIF0 = 0U;		/* clear INTP0 interrupt flag */
	}
	/***********************************************************************************************************************
	* Function Name: R_INTC1_Start
	* Description  : This function clears INTP1 interrupt flag and enables interrupt.
	* Arguments    : None
	* Return Value : None
	***********************************************************************************************************************/
	void R_INTC1_Start(void)
	{
	}
	/***********************************************************************************************************************
	* Function Name: R_INTC1_Stop
	* Description  : This function disables INTP1 interrupt and clears interrupt flag.
	* Arguments    : None
	* Return Value : None
	***********************************************************************************************************************/
	void R_INTC1_Stop(void)
	{
	}
	/***********************************************************************************************************************
	* Function Name: R_INTC2_Start
	* Description  : This function clears INTP2 interrupt flag and enables interrupt.
	* Arguments    : None
	* Return Value : None
	***********************************************************************************************************************/
	void R_INTC2_Start(void)
	{
	}
	/***********************************************************************************************************************
	* Function Name: R_INTC2_Stop
	* Description  : This function disables INTP2 interrupt and clears interrupt flag.
	* Arguments    : None
	* Return Value : None
	***********************************************************************************************************************/
	void R_INTC2_Stop(void)
	{
	}
	/***********************************************************************************************************************
	* Function Name: R_INTC3_Start
	* Description  : This function clears INTP3 interrupt flag and enables interrupt.
	* Arguments    : None
	* Return Value : None
	***********************************************************************************************************************/
	void R_INTC3_Start(void)
	{
		/* intpregbitset INTP3 enable */
		PIF3 = 0U;		/* clear INTP3 interrupt flag */
		PMK3 = 0U;		/* enable INTP3 interrupt */
	}
	/***********************************************************************************************************************
	* Function Name: R_INTC3_Stop
	* Description  : This function disables INTP3 interrupt and clears interrupt flag.
	* Arguments    : None
	* Return Value : None
	***********************************************************************************************************************/
	void R_INTC3_Stop(void)
	{
		/* intpregbitset INTP3 disable */
		PMK3 = 1U;		/* disable INTP3 interrupt */
		PIF3 = 0U;		/* clear INTP3 interrupt flag */
	}
	/***********************************************************************************************************************
	* Function Name: R_INTC4_Start
	* Description  : This function clears INTP4 interrupt flag and enables interrupt.
	* Arguments    : None
	* Return Value : None
	***********************************************************************************************************************/
	void R_INTC4_Start(void)
	{
		/* intpregbitset INTP4 enable */
		PIF4 = 0U;		/* clear INTP4 interrupt flag */
		PMK4 = 0U;		/* enable INTP4 interrupt */
	}
	/***********************************************************************************************************************
	* Function Name: R_INTC4_Stop
	* Description  : This function disables INTP4 interrupt and clears interrupt flag.
	* Arguments    : None
	* Return Value : None
	***********************************************************************************************************************/
	void R_INTC4_Stop(void)
	{
		/* intpregbitset INTP4 disable */
		PMK4 = 1U;		/* disable INTP4 interrupt */
		PIF4 = 0U;		/* clear INTP4 interrupt flag */
	}
	/***********************************************************************************************************************
	* Function Name: R_INTC5_Start
	* Description  : This function clears INTP5 interrupt flag and enables interrupt.
	* Arguments    : None
	* Return Value : None
	***********************************************************************************************************************/
	void R_INTC5_Start(void)
	{
	}
	/***********************************************************************************************************************
	* Function Name: R_INTC5_Stop
	* Description  : This function disables INTP5 interrupt and clears interrupt flag.
	* Arguments    : None
	* Return Value : None
	***********************************************************************************************************************/
	void R_INTC5_Stop(void)
	{
	}


/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
