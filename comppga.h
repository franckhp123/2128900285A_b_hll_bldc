
#ifndef __COMP_H
#define __COMP_H

#include "macrodriver.h"
#include "GlobalVar.h"
#include "tau.h"
#include "userdefine.h"

/***********************************************************************************************************************
Macro definitions (Register bit)
/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIESREGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
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
* Copyright (C) 2015, 2017 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_comppga.h
* Version      : Applilet4 for R7F0C008/009 V1.01.00.01 [30 Nov 2017]
* Device(s)    : R7F0C009B
* Tool-Chain   : CCRL
* Description  : This file implements device driver for COMP module.
* Creation Date: 2021/6/19 ?????¨´
***********************************************************************************************************************/
#ifndef COMP_H
#define COMP_H
/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/*
    Comparator Mode Setting Register (COMPMDR) 
*/
/* Comparator 1 monitor flag bit (C1MON) */
#define _00_COMP1_FLAG_REFERENCE_0                         (0x00U) /* CMP1 less than comparator 1 reference voltage */
#define _80_COMP1_FLAG_REFERENCE_1                         (0x80U) /* CMP1 more than comparator 1 reference voltage */
/* Comparator 1 operation enable bit (C1ENB) */
#define _00_COMP1_OPERATION_DISABLED                       (0x00U) /* comparator 1 operation disable */
#define _10_COMP1_OPERATION_ENABLED                        (0x10U) /* comparator 1 operation enable */
/* Comparator 0 monitor flag bit (C0MON) */
#define _00_COMP0_FLAG_REFERENCE_0                         (0x00U) /* CMP0 less than comparator 0 reference voltage */
#define _08_COMP0_FLAG_REFERENCE_1                         (0x08U) /* CMP0 more than comparator 0 reference voltage */
/* Comparator 0 operation enable bit (C0ENB) */
#define _00_COMP0_OPERATION_DISABLED                       (0x00U) /* comparator 0 operation disable */
#define _01_COMP0_OPERATION_ENABLED                        (0x01U) /* comparator 0 operation enable */

/*
    Comparator Filter Control Register (COMPFIR) 
*/
/* Comparator 1 edge detectioin selection (C1EDG) */
#define _00_COMP1_ONE_EDGE_INTERRUPT                       (0x00U) /* interrupt request by comparator 1 one-edge detection */
#define _80_COMP1_BOTH_EDGES_INTERRUPT                     (0x80U) /* interrupt request by comparator 1 both-edge detection */
/* Comparator 1 edges polarity switching (C1EPO) */
#define _00_COMP1_RISING_INTERRUPT                         (0x00U) /* interrupt request at comparator 1 rising edge */
#define _40_COMP1_FALLING_INTERRUPT                        (0x40U) /* interrupt request at comparator 1 falling edge */
/* Comparator 1 filter selection (C1FCK) */
#define _00_COMP1_FILTER_NONE                              (0x00U) /* disable filter */
#define _10_COMP1_FILTER_FCLK                              (0x10U) /* sampling at fCLK */
#define _20_COMP1_FILTER_FCLK_8                            (0x20U) /* sampling at fCLK/8 */
#define _30_COMP1_FILTER_FCLK_32                           (0x30U) /* sampling at fCLK/32 */
/* Comparator 0 edge detectioin selection (C0EDG) */
#define _00_COMP0_ONE_EDGE_INTERRUPT                       (0x00U) /* interrupt request by comparator 0 one-edge detection */
#define _08_COMP0_BOTH_EDGES_INTERRUPT                     (0x08U) /* interrupt request by comparator 0 both-edge detection */
/* Comparator 0 edges polarity switching (C0EPO) */
#define _00_COMP0_RISING_INTERRUPT                         (0x00U) /* interrupt request at comparator 0 rising edge */
#define _04_COMP0_FALLING_INTERRUPT                        (0x04U) /* interrupt request at comparator 0 falling edge */
/* Comparator 0 Filter Selection (C0FCK) */
#define _00_COMP0_FILTER_NONE                              (0x00U) /* disable filter */
#define _01_COMP0_FILTER_FCLK                              (0x01U) /* sampling at fCLK */
#define _02_COMP0_FILTER_FCLK_8                            (0x02U) /* sampling at fCLK/8 */
#define _03_COMP0_FILTER_FCLK_32                           (0x03U) /* sampling at fCLK/32 */

/*
    Comparator Output Control Register (COMPOCR) 
*/
/* Comparator 1 output polarity selection (C1OP) */
#define _00_COMP1_OUTPUT_NORMAL                            (0x00U) /* VCOUT1 normal output */
#define _40_COMP1_OUTPUT_INVERTED                          (0x40U) /* VCOUT1 inverted output */
/* Comparator 1 output enable (C1OE) */
#define _00_COMP1_OUTPUT_DISABLE                           (0x00U) /* comparator 1 output stopped */
#define _20_COMP1_OUTPUT_ENABLE                            (0x20U) /* comparator 1 output enabled */
/* Comparator 1 interrupt request enable (C1IE) */
#define _00_COMP1_INTERRUPT_DISABLE                        (0x00U) /* disbale VCOUT1 interrupt request */
#define _10_COMP1_INTERRPUT_ENABLE                         (0x10U) /* enable VCOUT1 interrupt request */
/* Comparator 0 output polarity selection (C0OP) */
#define _00_COMP0_OUTPUT_NORMAL                            (0x00U) /* VCOUT0 normal output */
#define _04_COMP0_OUTPUT_INVERTED                          (0x04U) /* VCOUT0 inverted output */
/* Comparator 0 output enable (C0OE) */
#define _00_COMP0_OUTPUT_DISABLE                           (0x00U) /* comparator 0 output stopped */
#define _02_COMP0_OUTPUT_ENABLE                            (0x02U) /* comparator 0 output enabled */
/* Comparator 0 interrupt request enable (C0IE) */
#define _00_COMP0_INTERRUPT_DISABLE                        (0x00U) /* disbale VCOUT0 interrupt request */
#define _01_COMP0_INTERRPUT_ENABLE                         (0x01U) /* enable VCOUT0 interrupt request */

/*
    Comparator internal reference voltage control register (CVRCTL) 
*/
/* Positive-side input of comparator 1 selection (CMPSEL1) */
#define _00_CMP1P_INPUT_COMP1                              (0x00U) /* CMP1P pin is selected as the positive-side input of comparator 1 */
#define _40_PGA_INPUT_COMP1                                (0x40U) /* PGA output is selected as the positive-side input of comparator 1 */
/* Operation of internal reference voltage 1 selection (CVRE1) */
#define _00_COMP1_REFVOLTAGE_DISABLE                       (0x00U) /* operation of internal reference voltage 1 stopped */
#define _20_COMP1_REFVOLTAGE_ENABLE                        (0x20U) /* operation of internal reference voltage 1 enabled */
/* GND of the internal reference voltage and the GND of the PGA feedback resistor selection  (CVRVS1) */
#define _00_VSS_INTERVOLTAGE_PGARESISTOR_COMP1             (0x00U) /* VSS is selected as the GND of the internal reference voltage 
                                                                      			and the GND of the PGA feedback resistor for comparator 1 */
#define _10_AVREFM_INTERVOLTAGE_PGARESISTOR_COMP1          (0x10U) /* AVREFM is selected as the GND of the internal reference voltage
                                                                      			and the GND of the PGA feedback resistor for comparator 1 */
/* Positive-side input of comparator 0 selection (CMPSEL0) */
#define _00_CMP0P_INPUT_COMP0                              (0x00U) /* CMP0P pin is selected as the positive-side input of comparator 0 */
#define _04_PGA_INPUT_COMP0                                (0x04U) /* PGA output is selected as the positive-side input of comparator 0 */
/* Operation of internal reference voltage 0 selection (CVRE0) */
#define _00_COMP0_REFVOLTAGE_DISABLE                       (0x00U) /* operation of internal reference voltage 0 stopped */
#define _02_COMP0_REFVOLTAGE_ENABLE                        (0x02U) /* operation of internal reference voltage 0 enabled */
/* Power supply of internal reference voltage selection (CVRVS0) */
#define _00_VDD_INTERNAL_REFERVOL                          (0x00U) /* VDD is selected as the power supply of the internal reference voltage */
#define _01_AVREFP_INTERNAL_REFERVOL                       (0x01U) /* AVREFP is selected as the power supply of the internal reference voltage */

/*
    PGA control register (PGACTL) 
*/
/* PGA operation selection (PGAEN) */
#define _00_PGA_OPERATION_STOP                             (0x00U) /* PGA operation stopped */
#define _80_PGA_OPERATION_ENABLE                           (0x80U) /* PGA operation enabled */
/* Selected PGA amplification (PGAVG1,PGAVG0) */
#define _00_PGA_AMPLIFICATION_X4                           (0x00U) /* *4 is selected as the gain */
#define _01_PGA_AMPLIFICATION_X8                           (0x01U) /* *8 is selected as the gain */
#define _02_PGA_AMPLIFICATION_X16                          (0x02U) /* *16 is selected as the gain */
#define _03_PGA_AMPLIFICATION_X32                          (0x03U) /* *32 is selected as the gain */

/*
    PWM option mode register (OPMR) 
*/
/* PWM option operating mode (HDM) */
#define _00_TWOSTAGE_OVERCURRENT_MODE                      (0x00U) /* two-stage overcurrent detection mode */
#define _02_OVERCURRENT_INDUCE_MODE                        (0x02U) /* overcurrent/induced current detection mode */

/*
    PWM option status register (OPSR) 
*/
/* Hi-Z control signal operating state (HZOF) */
#define _00_NORMAL_TIMER_OUTPUT                            (0x00U) /* normal timer output */
#define _04_HIZ_OUTPUT                                     (0x04U) /* Hi-Z output state */
/* On-chip comparator 1 output signal state (HZIF1) */
#define _00_COMP1_OUTPUT_L0W                               (0x00U) /* on-chip comparator 1 output signal is low level */
#define _02_COMP1_OUTPUT_HIGH                              (0x02U) /* on-chip comparator 1 output signal is high level */
/* On-chip comparator 0 output signal state (HZIF0) */
#define _00_COMP0_OUTPUT_L0W                               (0x00U) /* on-chip comparator 0 output signal is low level */
#define _01_COMP0_OUTPUT_HIGH                              (0x01U) /* on-chip comparator 0 output signal is high level */

/*
    PWM option Hi-Z start trigger register (OPHS) 
*/
/* Hi-Z control signal of on-chip comparator output (OPHS0) */
#define _00_HIZ_SIGNAL_INVALID                             (0x00U) /* invalid */
#define _01_HIZ_SIGNAL_HIGHLEVEL                           (0x01U) /* Hi-Z control signal is high level */

/*
    PWM option Hi-Z stop trigger register (OPHT) 
*/
/* Hi-Z cancel control of on-chip comparator output (OPHT0) */
#define _00_HIZ_CANCEL_INVALID                             (0x00U) /* invalid */
#define _01_HIZ_OUTPUT_CANCEL                              (0x01U) /* Hi-Z of the 6-phase output is canceled in
                                                                      			synchronization with the PWM period  */


/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define COMP_WAIT_TIME                                 (0x10U)   /* wait time for 3us */
#define _00_C0RVM_VALUE                                    (0x00U) /* comparator 0 internal reference voltage selection */
#define _D1_C1RVM_VALUE                                    (0xD1U) /* Comparator 1 internal reference voltage selection */

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void R_COMPPGA_Create(void);
void R_COMPPGA_Set_PowerOff(void);
void R_COMP0_Start(void);
void R_COMP0_Stop(void);
void R_COMP1_Start(void);
void R_COMP1_Stop(void);
void R_PGA_Start(void);
void R_PGA_Stop(void);
void R_PWMOPT_Start(void);
void R_PWMOPT_Stop(void);
void R_COMPPGA_Create_UserInit(void);


/* Start user code for function. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#endif

/* Start user code for function. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#endif
