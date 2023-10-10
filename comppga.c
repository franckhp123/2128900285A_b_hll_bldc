
#include "macrodriver.h"
#include "comppga.h"
#include "userdefine.h"   

/***********************************************************************************************************************
* Function Name: R_COMPPGA_Create
* Description  : This function initializes the comparator module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
    void R_COMPPGA_Create(void)
    {
        /* regbitset OACMPEN=1 */
        OACMPEN = 1U;   /* enables input clock supply */
        /* regbitset C0ENB=0 */
        C0ENB = 0U;     /* comparator 0 operation disable */
        /* regbitset C1ENB=0 */
        C1ENB = 0U;     /* comparator 1 operation disable */
        /* intpregbitset INTCMP0 disable */
        CMPMK0 = 1U;    /* disable INTCMP0 interrupt */
        CMPIF0 = 0U;    /* clear INTCMP0 interrupt flag */
        /* intpregbitset INTCMP1 disable */
        CMPMK1 = 1U;    /* disable INTCMP1 interrupt */
        CMPIF1 = 0U;    /* clear INTCMP1 interrupt flag */
        /* Set CMP1P pin */
        PMC12 |= 0x01U;     /* set bit0 to 1 */
        PM12 |= 0x01U;      /* set bit0 to 1 */
        /* Set CMP0P pin */
   //     PMC0 |= 0x01U;      /* set bit0 to 1 */
    //    PM0 |= 0x01U;       /* set bit0 to 1 */
        /* regset PGACTL */
        /* regset C0RVM */
        C0RVM = _00_C0RVM_VALUE;
        /* regset C1RVM */
        C1RVM = 220;            //    _D1_C1RVM_VALUE;
        /* regset CVRCTL */
        CVRCTL = _00_CMP1P_INPUT_COMP1 | _20_COMP1_REFVOLTAGE_ENABLE | _00_VSS_INTERVOLTAGE_PGARESISTOR_COMP1 | 
                 _00_CMP0P_INPUT_COMP0 | _02_COMP0_REFVOLTAGE_ENABLE | _00_VDD_INTERNAL_REFERVOL;
        /* regset OPHS */
        /* regset COMPFIR */
        COMPFIR = _00_COMP1_ONE_EDGE_INTERRUPT | _00_COMP1_RISING_INTERRUPT | _00_COMP0_ONE_EDGE_INTERRUPT | 
                  _00_COMP0_RISING_INTERRUPT;
        /* regset COMPOCR */
        COMPOCR = _00_COMP1_OUTPUT_NORMAL | _10_COMP1_INTERRPUT_ENABLE | _04_COMP0_OUTPUT_INVERTED | 
                  _00_COMP0_INTERRUPT_DISABLE;
        /* intpregbitset INTCMP0 INTP=PR */
        /* intpregbitset INTCMP1 INTP=PR */
        /* Set INTCMP1 high priority */
        CMPPR11 = 0U;
        CMPPR01 = 0U;
    
    //    R_COMPPGA_Create_UserInit();
    }
    /***********************************************************************************************************************
    * Function Name: R_COMPPGA_Set_PowerOff
    * Description  : This function stops the clock supplied for comparator and PGA.
    * Arguments    : None
    * Return Value : None
    ***********************************************************************************************************************/
    void R_COMPPGA_Set_PowerOff(void)
    {
        /* regbitset OACMPEN=0 */
        OACMPEN = 0U;   /* stops input clock supply */
    }
    /***********************************************************************************************************************
    * Function Name: R_COMP0_Start
    * Description  : This function starts the comparator 0.
    * Arguments    : None
    * Return Value : None
    ***********************************************************************************************************************/
    void R_COMP0_Start(void)
    {
        volatile uint16_t w_count;
    
        /* regbitset C0ENB=1 */
        C0ENB = 1U;     /* comparator 0 operation enable */
    
        /* Change the waiting time according to the system */
        for (w_count = 0U; w_count < COMP_WAIT_TIME; w_count++)
        {
            NOP();  
        }
        /* regset C0OE=1 opmode=byvalue */
        COMPOCR |= _02_COMP0_OUTPUT_ENABLE;
    }
    /***********************************************************************************************************************
    * Function Name: R_COMP0_Stop
    * Description  : This function stops the comparator 0.
    * Arguments    : None
    * Return Value : None
    ***********************************************************************************************************************/
    void R_COMP0_Stop(void)
    {
        /* regset C0OE=0 opmode=byvalue */
        COMPOCR &= (uint8_t)~_02_COMP0_OUTPUT_ENABLE;
        /* regbitset C0ENB=0 */
        C0ENB = 0U;     /* comparator 0 operation disable */
    }
    
    /***********************************************************************************************************************
    * Function Name: R_COMP1_Start
    * Description  : This function starts the comparator 1.
    * Arguments    : None
    * Return Value : None
    ***********************************************************************************************************************/
    void R_COMP1_Start(void)
    {
        volatile uint16_t w_count;
        
        /* regbitset C1ENB=1 */
        C1ENB = 1U;     /* comparator 1 operation enable */
        
        /* Change the waiting time according to the system */
        for (w_count = 0U; w_count < COMP_WAIT_TIME; w_count++)
        {
            NOP();  
        }
        /* regset C1OE=1 opmode=byvalue */
        COMPOCR |= _20_COMP1_OUTPUT_ENABLE;
    
        /* intpregbitset INTCMP1 enable */
        CMPIF1 = 0U;    /* clear INTCMP1 interrupt flag */
        CMPMK1 = 0U;    /* enable INTCMP1 interrupt */
    }
    /***********************************************************************************************************************
    * Function Name: R_COMP1_Stop
    * Description  : This function stops the comparator 1.
    * Arguments    : None
    * Return Value : None
    ***********************************************************************************************************************/
    void R_COMP1_Stop(void)
    {
        /* regset C1OE=0 opmode=byvalue */
        COMPOCR &= (uint8_t)~_20_COMP1_OUTPUT_ENABLE;
        /* regbitset C1ENB=0 */
        C1ENB = 0U;     /* comparator 1 operation disable */
        /* intpregbitset INTCMP1 disable */
        CMPMK1 = 1U;    /* disable INTCMP1 interrupt */
        CMPIF1 = 0U;    /* clear INTCMP1 interrupt flag */
    }
    /***********************************************************************************************************************
    * Function Name: R_PGA_Start
    * Description  : This function starts the  programmable gain amplifier.
    * Arguments    : None
    * Return Value : None
    ***********************************************************************************************************************/
    void R_PGA_Start(void)
    {
        /* regbitset PGAEN=1 */
        PGAEN = 1U;     /* PGA operation enabled */
    }
    /***********************************************************************************************************************
    * Function Name: R_PGA_Stop
    * Description  : This function stops the  programmable gain amplifier.
    * Arguments    : None
    * Return Value : None
    ***********************************************************************************************************************/
    void R_PGA_Stop(void)
    {
        /* regbitset PGAEN=0 */
        PGAEN = 0U;     /* PGA operation stopped */
    }
    /***********************************************************************************************************************
    * Function Name: R_PWMOPT_Start
    * Description  : This function starts the PWM operation.
    * Arguments    : None
    * Return Value : None
    ***********************************************************************************************************************/
    void R_PWMOPT_Start(void)
    {
        /* regbitset PWMOPEN=1 */
        PWMOPEN = 1U;   /* enables input clock supply */
        /* regset OPMR */
        OPMR = _02_OVERCURRENT_INDUCE_MODE;
    }
    /***********************************************************************************************************************
    * Function Name: R_PWMOPT_Stop
    * Description  : This function stops the PWM operation.
    * Arguments    : None
    * Return Value : None
    ***********************************************************************************************************************/
    void R_PWMOPT_Stop(void)
    {
        /* regbitset PWMOPEN=0 */
        PWMOPEN = 0U;   /* stops input clock supply */
    }



