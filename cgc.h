
#ifndef __CGC_H
#define __CGC_H

#include "macrodriver.h"
#include "userdefine.h"

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/*
    Clock operation mode control register (CMC) 
*/
/* Control of X1 clock oscillation frequency (AMPH) */
#define _00_CGC_SYSOSC_UNDER10M           (0x00U) /* 1MHz <= fX <= 10MHz */
#define _01_CGC_SYSOSC_OVER10M            (0x01U) /* 10MHz < fX <= 20MHz */
/* High-speed system clock pin operation mode (EXCLK,OSCSEL) */
#define _00_CGC_HISYS_PORT                (0x00U) /* X1, X2 as I/O port */
#define _40_CGC_HISYS_OSC                 (0x40U) /* X1, X2 as crystal/ceramic resonator connection */
#define _80_CGC_HISYS_PORT1               (0x80U) /* X1, X2 as I/O port */
#define _C0_CGC_HISYS_EXT                 (0xC0U) /* X1 as I/O port, X2 as external clock input */
#define _C0_CGC_HISYS_PIN                 (0xC0U) /* X1, X2 pin setting */

/*
    System clock control register (CKC) 
*/
/* Main system clock (fMAIN) operation control (MCM0) */
#define _00_CGC_MAINCLK_SELFOCO           (0x00U) /* selects the high-speed on-chip oscillator clock (fIH) as the main system clock (fMAIN) */
#define _10_CGC_MAINCLK_SELHISYS          (0x10U) /* selects the high-speed system clock (fMX) as the main system clock (fMAIN) */
/* Status of Main system clock (fMAIN) (MCS) */
#define _00_CGC_MAINCLK_FOCO              (0x00U) /* High-speed on-chip oscillator clock (fIH) */
#define _20_CGC_MAINCLK_HISYS             (0x20U) /* high-speed system clock (fMX) */

/*
    Clock operation status control register (CSC) 
*/
/* High-speed on-chip oscillator clock operation control (HIOSTOP) */
#define _00_CGC_HIO_OPER                  (0x00U) /* high-speed on-chip oscillator operating */
#define _01_CGC_HIO_STOP                  (0x01U) /* high-speed on-chip oscillator stopped */
/* High-speed system clock operation control (MSTOP) */
#define _00_CGC_HISYS_OPER                (0x00U) /* X1 oscillator/external clock operating */
#define _80_CGC_HISYS_STOP                (0x80U) /* X1 oscillator/external clock stopped */

/*
    Oscillation stabilization time counter status register (OSTC) 
*/
/* oscillation stabilization time status (MOST8,MOST9,MOST10,MOST11,MOST13,MOST15,MOST17,MOST18) */
#define _00_CGC_OSCSTAB_STA0              (0x00U) /* 2^8/fX max. */
#define _80_CGC_OSCSTAB_STA8              (0x80U) /* 2^8/fX */
#define _C0_CGC_OSCSTAB_STA9              (0xC0U) /* 2^9/fX */
#define _E0_CGC_OSCSTAB_STA10             (0xE0U) /* 2^10/fX */
#define _F0_CGC_OSCSTAB_STA11             (0xF0U) /* 2^11/fX */
#define _F8_CGC_OSCSTAB_STA13             (0xF8U) /* 2^13/fX */
#define _FC_CGC_OSCSTAB_STA15             (0xFCU) /* 2^15/fX */
#define _FE_CGC_OSCSTAB_STA17             (0xFEU) /* 2^17/fX */
#define _FF_CGC_OSCSTAB_STA18             (0xFFU) /* 2^18/fX */

/*
    Oscillation stabilization time select register (OSTS) 
*/
/* oscillation stabilization time selection (OSTS2,OSTS1,OSTS0) */
#define _00_CGC_OSCSTAB_SEL8              (0x00U) /* 2^8/fX */
#define _01_CGC_OSCSTAB_SEL9              (0x01U) /* 2^9/fX */
#define _02_CGC_OSCSTAB_SEL10             (0x02U) /* 2^10/fX */
#define _03_CGC_OSCSTAB_SEL11             (0x03U) /* 2^11/fX */
#define _04_CGC_OSCSTAB_SEL13             (0x04U) /* 2^13/fX */
#define _05_CGC_OSCSTAB_SEL15             (0x05U) /* 2^15/fX */
#define _06_CGC_OSCSTAB_SEL17             (0x06U) /* 2^17/fX */
#define _07_CGC_OSCSTAB_SEL18             (0x07U) /* 2^18/fX */

/*
    Operation speed mode control register (OSMC) 
*/
/* Selection of low-speed on-chip oscillator as 12-bit interval timer operating clock or timer RJ count source (WUTMMCK0) */
#define _00_CGC_IT_TMRJ_CLK_STOP          (0x00U) /* cannot be selected */
#define _10_CGC_IT_TMRJ_CLK_FIL           (0x10U) /* can be selected */



/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
typedef enum
{
    HIOCLK,
    SYSX1CLK, 
    SYSEXTCLK,
} clock_mode_t;

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void R_CGC_Create(void);
void R_CGC_Get_ResetSource(void);
/* Start user code for function. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#endif
