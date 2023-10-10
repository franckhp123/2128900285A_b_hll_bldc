
/***********************************************************************************************************************
* File Name    : r_cg_elc.h
* Version      : Applilet4 for R7F0C008/009 V1.01.00.01 [30 Nov 2017]
* Device(s)    : R7F0C009B
* Tool-Chain   : CCRL
* Description  : This file implements device driver for ELC module.
* Creation Date: 2018/12/13
***********************************************************************************************************************/
#ifndef __ELC_H
#define __ELC_H

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/*
    Event output destination select register n (ELSELRn) 
*/
/* Event output destination select register n (ELSELn2 - ELSELn0) */
#define _00_ELC_EVENT_LINK_OFF             (0x00U) /* event link disabled */
#define _01_ELC_EVENT_LINK_AD              (0x01U) /* select operation of peripheral function to link A/D converter */
#define _02_ELC_EVENT_LINK_TAU0_0          (0x02U) /* select operation of peripheral function to link Timer input of timer array unit 0 channel 0 */
#define _03_ELC_EVENT_LINK_TAU0_1          (0x03U) /* select operation of peripheral function to link Timer input of timer array unit 0 channel 1 */
#define _04_ELC_EVENT_LINK_TMRJ0           (0x04U) /* select operation of peripheral function to link Timer RJ0 */
#define _05_ELC_EVENT_LINK_TMRD0           (0x05U) /* select operation of peripheral function to link Timer RD0 */
#define _06_ELC_EVENT_LINK_TMRD1           (0x06U) /* select operation of peripheral function to link Timer RD1 */


/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define ELC_DESTINATION_COUNT              (0x13U)

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void R_ELC_Create(void);
void R_ELC_Stop(uint32_t event);

#endif
