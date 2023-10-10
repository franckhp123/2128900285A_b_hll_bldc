
#ifndef __TMRD_H
#define __TMRD_H

/*
    Timer RD ELC Register (TRDELC) 
*/
/* ELC event input 0 select for timer RD input capture D0 (ELCICE0) */
#define _00_TMRD0_INPUTCAPTURE                       (0x00U) /* TRDIOD0 input capture is selected */
#define _01_TMRD0_ELC                                (0x01U) /* event input 0 from ELC is selected */
/* ELC event input 0 enable for timer RD pulse output forced cutoff (ELCOBE0) */
#define _00_TMRD0_CUTOFF_DISABLED                    (0x00U) /* forced cutoff is disabled */
#define _02_TMRD0_CUTOFF_ENABLED                     (0x02U) /* forced cutoff is enabled */
/* ELC event input 1 select for timer RD input capture D1 (ELCICE1) */
#define _00_TMRD1_INPUTCAPTURE                       (0x00U) /* TRDIOD1 input capture is selected */
#define _10_TMRD1_ELC                                (0x10U) /* event input 1 from ELC is selected */
/* ELC event input 1 enable for timer RD pulse output forced cutoff (ELCOBE1) */
#define _00_TMRD1_CUTOFF_DISABLED                    (0x00U) /* forced cutoff is disabled */
#define _20_TMRD1_CUTOFF_ENABLED                     (0x20U) /* forced cutoff is enabled */

/*
    Timer RD PWM function select register (TRDPMR) 
*/
/* PWM function of TRDIOB0 select (TRDPWMB0) */
#define _00_TMRD_TRDIOB0_TIMER_MODE                  (0x00U) /* input capture function or output compare function */
#define _01_TMRD_TRDIOB0_PWM_MODE                    (0x01U) /* PWM function */
/* PWM function of TRDIOC0 select (TRDPWMC0) */
#define _00_TMRD_TRDIOC0_TIMER_MODE                  (0x00U) /* input capture function or output compare function */
#define _02_TMRD_TRDIOC0_PWM_MODE                    (0x02U) /* PWM function */
/* PWM function of TRDIOD0 select (TRDPWMD0) */
#define _00_TMRD_TRDIOD0_TIMER_MODE                  (0x00U) /* input capture function or output compare function */
#define _04_TMRD_TRDIOD0_PWM_MODE                    (0x04U) /* PWM function */
/* PWM function of TRDIOB1 select (TRDPWMB1) */
#define _00_TMRD_TRDIOB1_TIMER_MODE                  (0x00U) /* input capture function or output compare function */
#define _10_TMRD_TRDIOB1_PWM_MODE                    (0x10U) /* PWM function */
/* PWM function of TRDIOC1 select (TRDPWMC1) */
#define _00_TMRD_TRDIOC1_TIMER_MODE                  (0x00U) /* input capture function or output compare function */
#define _20_TMRD_TRDIOC1_PWM_MODE                    (0x20U) /* PWM function */
/* PWM function of TRDIOD1 select (TRDPWMD1) */
#define _00_TMRD_TRDIOD1_TIMER_MODE                  (0x00U) /* input capture function or output compare function */
#define _40_TMRD_TRDIOD1_PWM_MODE                    (0x40U) /* PWM function */

/*
    Timer RD function control register (TRDFCR) 
*/
/* Combination mode select (CMD1,CMD0) */
#define _01_TMRD_TRANSFER_RESET_SYNCHRONOUS          (0x01U) /* reset synchronous PWM mode */
#define _02_TMRD_TRANSFER_TMRD1_UNDERFLOW            (0x02U) /* transfer from the buffer register to the general register when TRD1 underflows */
#define _03_TMRD_TRANSFER_TMRD0_MATCH                (0x03U) /* transfer from the buffer register to the general register at compare match between registers TRD0 and TRDGRA0 */
/* Phase output level select (OLS0) */
#define _00_TMRD_NORMAL_PHASE_LEVEl_HL               (0x00U) /* high initial output and low active level */
#define _04_TMRD_NORMAL_PHASE_LEVEl_LH               (0x04U) /* low initial output and high active level */
/* Counter-phase output level select (OLS1) */
#define _00_TMRD_COUNTER_PHASE_LEVEl_HL              (0x00U) /* high initial output and low active level */
#define _08_TMRD_COUNTER_PHASE_LEVEl_LH              (0x08U) /* low initial output and high active level */
/* External clock input select (STCLK) */
#define _00_TMRD_EXTERNAL_CLOCK_DISABLE              (0x00U) /* external clock input disabled */
#define _40_TMRD_EXTERNAL_CLOCK_ENABLE               (0x40U) /* external clock input enabled */
/* PWM3 mode select (PWM3) */
#define _00_TMRD_PWM3_MODE                           (0x00U) /* in PWM3 mode */
#define _80_TMRD_OTHER_MODE                          (0x80U) /* in the timer mode */

/*
    Timer RD output master enable register 1 (TRDOER1) 
*/
/* TRDIOA0 output disable (EA0) */
#define _00_TMRD_TRDIOA0_OUTPUT_ENABLE               (0x00U) /* output enabled */
#define _01_TMRD_TRDIOA0_OUTPUT_DISABLE              (0x01U) /* output disabled */
/* TRDIOB0 output disable (EB0) */
#define _00_TMRD_TRDIOB0_OUTPUT_ENABLE               (0x00U) /* output enabled */
#define _02_TMRD_TRDIOB0_OUTPUT_DISABLE              (0x02U) /* output disabled */
/* TRDIOC0 output disable (EC0) */
#define _00_TMRD_TRDIOC0_OUTPUT_ENABLE               (0x00U) /* output enabled */
#define _04_TMRD_TRDIOC0_OUTPUT_DISABLE              (0x04U) /* output disabled */
/* TRDIOD0 output disable (ED0) */
#define _00_TMRD_TRDIOD0_OUTPUT_ENABLE               (0x00U) /* output enabled */
#define _08_TMRD_TRDIOD0_OUTPUT_DISABLE              (0x08U) /* output disabled */
/* TRDIOA1 output disable (EA1) */
#define _00_TMRD_TRDIOA1_OUTPUT_ENABLE               (0x00U) /* output enabled */
#define _10_TMRD_TRDIOA1_OUTPUT_DISABLE              (0x10U) /* output disabled */
/* TRDIOB1 output disable (EB1) */
#define _00_TMRD_TRDIOB1_OUTPUT_ENABLE               (0x00U) /* output enabled */
#define _20_TMRD_TRDIOB1_OUTPUT_DISABLE              (0x20U) /* output disabled */
/* TRDIOC1 output disable (EC1) */
#define _00_TMRD_TRDIOC1_OUTPUT_ENABLE               (0x00U) /* output enabled */
#define _40_TMRD_TRDIOC1_OUTPUT_DISABLE              (0x40U) /* output disabled */
/* TRDIOD1 output disable (ED1) */
#define _00_TMRD_TRDIOD1_OUTPUT_ENABLE               (0x00U) /* output enabled */
#define _80_TMRD_TRDIOD1_OUTPUT_DISABLE              (0x80U) /* output disabled */

/*
    Timer RD output master enable register 2 (TRDOER2) 
*/
/* Forced cutoff flag (TRDSHUTS) */
#define _00_TMRD_INTERCEPTION_STOP                   (0x00U) /* not forcibly cut off */
#define _01_TMRD_INTERCEPTION                        (0x01U) /* forcibly cut off */
/* INTP0 of pulse output forced cutoff signal input enabled (TRDPTO) */
#define _00_TMRD_INTP0_CUTOFF_DISABLE                (0x00U) /* pulse output forced cutoff input disabled */
#define _80_TMRD_INTP0_CUTOFF_ENABLE                 (0x80U) /* pulse output forced cutoff input enabled */

/*
    Timer RD output control register (TRDOCR) 
*/
/* TRDIOA0 initial output level select (TOA0) */
#define _00_TMRD_TRDIOA0_INITIAL_OUTPUT_L            (0x00U) /* initial output is low */
#define _01_TMRD_TRDIOA0_INITIAL_OUTPUT_H            (0x01U) /* initial output is high */
/* TRDIOB0 initial output level select (TOB0) */
#define _00_TMRD_TRDIOB0_INITIAL_OUTPUT_L            (0x00U) /* initial output is low */
#define _02_TMRD_TRDIOB0_INITIAL_OUTPUT_H            (0x02U) /* initial output is high */
#define _00_TMRD_TRDIOB0_INITIAL_OUTPUT_NOT_ACTIVE   (0x00U) /* initial output isn't active level */
#define _02_TMRD_TRDIOB0_INITIAL_OUTPUT_ACTIVE       (0x02U) /* initial output is active level */
/* TRDIOC0 initial output level select (TOC0) */
#define _00_TMRD_TRDIOC0_INITIAL_OUTPUT_L            (0x00U) /* initial output is low */
#define _04_TMRD_TRDIOC0_INITIAL_OUTPUT_H            (0x04U) /* initial output is high */
#define _00_TMRD_TRDIOC0_INITIAL_OUTPUT_NOT_ACTIVE   (0x00U) /* initial output isn't active level */
#define _04_TMRD_TRDIOC0_INITIAL_OUTPUT_ACTIVE       (0x04U) /* initial output is active level */
/* TRDIOD0 initial output level select (TOD0) */
#define _00_TMRD_TRDIOD0_INITIAL_OUTPUT_L            (0x00U) /* initial output is low */
#define _08_TMRD_TRDIOD0_INITIAL_OUTPUT_H            (0x08U) /* initial output is high */
#define _00_TMRD_TRDIOD0_INITIAL_OUTPUT_NOT_ACTIVE   (0x00U) /* initial output isn't active level */
#define _08_TMRD_TRDIOD0_INITIAL_OUTPUT_ACTIVE       (0x08U) /* initial output is active level */
/* TRDIOA1 initial output level select (TOA1) */
#define _00_TMRD_TRDIOA1_INITIAL_OUTPUT_L            (0x00U) /* initial output is low */
#define _10_TMRD_TRDIOA1_INITIAL_OUTPUT_H            (0x10U) /* initial output is high */
/* TRDIOB1 initial output level select (TOB1) */
#define _00_TMRD_TRDIOB1_INITIAL_OUTPUT_L            (0x00U) /* initial output is low */
#define _20_TMRD_TRDIOB1_INITIAL_OUTPUT_H            (0x20U) /* initial output is high */
#define _00_TMRD_TRDIOB1_INITIAL_OUTPUT_NOT_ACTIVE   (0x00U) /* initial output isn't active level */
#define _20_TMRD_TRDIOB1_INITIAL_OUTPUT_ACTIVE       (0x20U) /* initial output is active level */
/* TRDIOC1 initial output level select (TOC1) */
#define _00_TMRD_TRDIOC1_INITIAL_OUTPUT_L            (0x00U) /* initial output is low */
#define _40_TMRD_TRDIOC1_INITIAL_OUTPUT_H            (0x40U) /* initial output is high */
#define _00_TMRD_TRDIOC1_INITIAL_OUTPUT_NOT_ACTIVE   (0x00U) /* initial output isn't active level */
#define _40_TMRD_TRDIOC1_INITIAL_OUTPUT_ACTIVE       (0x40U) /* initial output is active level */
/* TRDIOD1 initial output level select (TOD1) */
#define _00_TMRD_TRDIOD1_INITIAL_OUTPUT_L            (0x00U) /* initial output is low */
#define _80_TMRD_TRDIOD1_INITIAL_OUTPUT_H            (0x80U) /* initial output is high */
#define _00_TMRD_TRDIOD1_INITIAL_OUTPUT_NOT_ACTIVE   (0x00U) /* initial output isn't active level */
#define _80_TMRD_TRDIOD1_INITIAL_OUTPUT_ACTIVE       (0x80U) /* initial output is active level */

/*
    Timer RD status register 0 (TRDSR0) 
*/
/* Input capture/compare match flag A (IMFA) */
#define _00_TMRD0_INTA_FLAG_CLEAR                    (0x00U) /* interrupt A not generate */
#define _01_TMRD0_INTA_GENERATE_FLAG                 (0x01U) /* interrupt A generate */
/* Input capture/compare match flag B (IMFB) */
#define _00_TMRD0_INTB_FLAG_CLEAR                    (0x00U) /* interrupt B not generate */
#define _02_TMRD0_INTB_GENERATE_FLAG                 (0x02U) /* interrupt B generate */
/* Input capture/compare match flag C (IMFC) */
#define _00_TMRD0_INTC_FLAG_CLEAR                    (0x00U) /* interrupt C not generate */
#define _04_TMRD0_INTC_GENERATE_FLAG                 (0x04U) /* interrupt C generate */
/* Input capture/compare match flag D (IMFD) */
#define _00_TMRD0_INTD_FLAG_CLEAR                    (0x00U) /* interrupt D not generate */
#define _08_TMRD0_INTD_GENERATE_FLAG                 (0x08U) /* interrupt D generate */
/* Overflow flag (OVF) */
#define _00_TMRD0_INTOV_FLAG_CLEAR                   (0x00U) /* interrupt overflow not generate */
#define _10_TMRD0_INTOV_GENERATE_FLAG                (0x10U) /* interrupt overflow generate */

/*
    Timer RD status register 1 (TRDSR1) 
*/
/* Input capture/compare match flag A (IMFA) */
#define _00_TMRD1_INTA_FLAG_CLEAR                    (0x00U) /* interrupt A not generate */
#define _01_TMRD1_INTA_GENERATE_FLAG                 (0x01U) /* interrupt A generate */
/* Input capture/compare match flag B (IMFB) */
#define _00_TMRD1_INTB_FLAG_CLEAR                    (0x00U) /* interrupt B not generate */
#define _02_TMRD1_INTB_GENERATE_FLAG                 (0x02U) /* interrupt B generate */
/* Input capture/compare match flag C (IMFC) */
#define _00_TMRD1_INTC_FLAG_CLEAR                    (0x00U) /* interrupt C not generate */
#define _04_TMRD1_INTC_GENERATE_FLAG                 (0x04U) /* interrupt C generate */
/* INput capture/compare match flag D (IMFD) */
#define _00_TMRD1_INTD_FLAG_CLEAR                    (0x00U) /* interrupt D not generate */
#define _08_TMRD1_INTD_GENERATE_FLAG                 (0x08U) /* interrupt D generate */
/* Overflow flag (OVF) */
#define _00_TMRD1_INTOV_FLAG_CLEAR                   (0x00U) /* interrupt overflow not generate */
#define _10_TMRD1_INTOV_GENERATE_FLAG                (0x10U) /* interrupt overflow generate */
/* Underflow flag (UDF) */
#define _00_TMRD1_INTUN_FLAG_CLEAR                   (0x00U) /* interrupt underflow not generate */
#define _20_TMRD1_INTUN_GENERATE_FLAG                (0x20U) /* interrupt underflow generate */

/*
    Timer RD start register (TRDSTR) 
*/
/* TRD0 count start flag (TSTART0) */
#define _00_TMRD_TRD0_COUNT_STOP                     (0x00U) /* TRD0 Count stops */
#define _01_TMRD_TRD0_COUNT_START                    (0x01U) /* TRD0 Count starts */
/* TRD1 count start flag (TSTART1) */
#define _00_TMRD_TRD1_COUNT_STOP                     (0x00U) /* TRD1 Count stops */
#define _02_TMRD_TRD1_COUNT_START                    (0x02U) /* TRD1 Count starts */
/* TRD0 count operation select (CSEL0) */
#define _00_TMRD_TRD0_COUNT_STOP_MATCH               (0x00U) /* count stops at compare match with TRDGRA0 register */
#define _04_TMRD_TRD0_COUNT_CONTINUES                (0x04U) /* count continues after compare match with TRDGRA0 register */
/* TRD1 count operation select (CSEL1) */
#define _00_TMRD_TRD1_COUNT_STOP_MATCH               (0x00U) /* count stops at compare match with TRDGRA1 register */
#define _08_TMRD_TRD1_COUNT_CONTINUES                (0x08U) /* count continues after compare match with TRDGRA1 register */

/*
    Timer RD mode register (TRDMR) 
*/
/* Timer RD synchronous (TRDSYNC) */
#define _00_TMRD_INDEPENDENTLY                       (0x00U) /* TRD0 and TRD1 operate independently */
#define _01_TMRD_SYNCHRONOUSLY                       (0x01U) /* TRD0 and TRD1 operate synchronously */
/* TRDGRC0 register function select (TRDBFC0) */
#define _00_TMRD_TRDGRC0_GENERAL                     (0x00U) /* general register */
#define _10_TMRD_TRDGRC0_BUFFER                      (0x10U) /* buffer register for TRDGRA0 register */
/* TRDGRD0 register function select (TRDBFD0) */
#define _00_TMRD_TRDGRD0_GENERAL                     (0x00U) /* general register */
#define _20_TMRD_TRDGRD0_BUFFER                      (0x20U) /* buffer register for TRDGRB0 register */
/* TRDGRC1 register function select (TRDBFC1) */
#define _00_TMRD_TRDGRC1_GENERAL                     (0x00U) /* general register */
#define _40_TMRD_TRDGRC1_BUFFER                      (0x40U) /* buffer register for TRDGRA1 register */
/* TRDGRD1 register function select (TRDBFD1) */
#define _00_TMRD_TRDGRD1_GENERAL                     (0x00U) /* general register */
#define _80_TMRD_TRDGRD1_BUFFER                      (0x80U) /* buffer register for TRDGRB1 register */

/*
    Timer RD digital filter function select register m (TRDDFm) 
*/
/* TRDIOAm pin digital filter function select (DFA) */
#define _00_TMRD_TRDIOA_DIGITAL_FILTER_DISABLE       (0x00U) /* digital filter function disabled */
#define _01_TMRD_TRDIOA_DIGITAL_FILTER_ENABLE        (0x01U) /* digital filter function enabled */
/* TRDIOBm pin digital filter function select (DFB) */
#define _00_TMRD_TRDIOB_DIGITAL_FILTER_DISABLE       (0x00U) /* digital filter function disabled */
#define _02_TMRD_TRDIOB_DIGITAL_FILTER_ENABLE        (0x02U) /* digital filter function enabled */
/* TRDIOD pin pulse forced cutoff control (DFB,DFA) */
#define _00_TMRD_TRDIOD_FORCEDCUTOFF_DISABLE         (0x00U) /* forced cutoff disabled */
#define _01_TMRD_TRDIOD_HIGHIMPEDANCE_OUTPUT         (0x01U) /* high-impedance output */
#define _02_TMRD_TRDIOD_LOW_OUTPUT                   (0x02U) /* low output */
#define _03_TMRD_TRDIOD_HIGH_OUTPUT                  (0x03U) /* high output */
/* TRDIOCm pin digital filter function select (DFC) */
#define _00_TMRD_TRDIOC_DIGITAL_FILTER_DISABLE       (0x00U) /* digital filter function disabled */
#define _04_TMRD_TRDIOC_DIGITAL_FILTER_ENABLE        (0x04U) /* digital filter function enabled */
/* TRDIODm pin digital filter function select (DFD) */
#define _00_TMRD_TRDIOD_DIGITAL_FILTER_DISABLE       (0x00U) /* digital filter function disabled */
#define _08_TMRD_TRDIOD_DIGITAL_FILTER_ENABLE        (0x08U) /* digital filter function enabled */
/* TRDIOC pin pulse forced cutoff control (DFD,DFC) */
#define _00_TMRD_TRDIOC_FORCEDCUTOFF_DISABLE         (0x00U) /* forced cutoff disabled */
#define _04_TMRD_TRDIOC_HIGHIMPEDANCE_OUTPUT         (0x04U) /* high-impedance output */
#define _08_TMRD_TRDIOC_LOW_OUTPUT                   (0x08U) /* low output */
#define _0C_TMRD_TRDIOC_HIGH_OUTPUT                  (0x0CU) /* high output */
/* TRDIOB pin pulse forced cutoff control (PENB1,PENB0) */
#define _00_TMRD_TRDIOB_FORCEDCUTOFF_DISABLE         (0x00U) /* forced cutoff disabled */
#define _10_TMRD_TRDIOB_HIGHIMPEDANCE_OUTPUT         (0x10U) /* high-impedance output */
#define _20_TMRD_TRDIOB_LOW_OUTPUT                   (0x20U) /* low output */
#define _30_TMRD_TRDIOB_HIGH_OUTPUT                  (0x30U) /* high output */
/* Clock select for digital filter function (DFCK1,DFCK0) */
#define _00_TMRD_DIGITAL_FILTER_CLOCK_F32            (0x00U) /* fCLK/32 */
#define _40_TMRD_DIGITAL_FILTER_CLOCK_F8             (0x40U) /* fCLK/8 */
#define _80_TMRD_DIGITAL_FILTER_CLOCK_F1             (0x80U) /* fCLK */
#define _C0_TMRD_DIGITAL_FILTER_CLOCK_SOURCE         (0xC0U) /* count source */
/* TRDIOA pin pulse forced cutoff control (DFCK1,DFCK0) */
#define _00_TMRD_TRDIOA_FORCEDCUTOFF_DISABLE         (0x00U) /* forced cutoff disabled */
#define _40_TMRD_TRDIOA_HIGHIMPEDANCE_OUTPUT         (0x40U) /* high-impedance output */
#define _80_TMRD_TRDIOA_LOW_OUTPUT                   (0x80U) /* low output */
#define _C0_TMRD_TRDIOA_HIGH_OUTPUT                  (0xC0U) /* high output */

/*
    Timer RD control register m (TRDCRm) 
*/
/* count source select (TCK2,TCK1,TCK0) */
#define _00_TMRD_INETNAL_CLOCK_FCLK_FHOCO            (0x00U) /* fCLK, fHOCO */
#define _01_TMRD_INETNAL_CLOCK_FCLK2                 (0x01U) /* fCLK/2 */
#define _02_TMRD_INETNAL_CLOCK_FCLK4                 (0x02U) /* fCLK/4 */
#define _03_TMRD_INETNAL_CLOCK_FCLK8                 (0x03U) /* fCLK/8 */
#define _04_TMRD_INETNAL_CLOCK_FCLK32                (0x04U) /* fCLK/32 */
#define _05_TMRD_EXTERNAL_CLOCK_TRDCLK               (0x05U) /* TRDCLK input */
/* external clock edge select (CKEG1,CKEG0) */
#define _00_TMRD_EXTERNAL_CLOCK_EDGE_RISING          (0x00U) /* count at the rising edge */
#define _08_TMRD_EXTERNAL_CLOCK_EDGE_FALLING         (0x08U) /* count at the falling edge */
#define _10_TMRD_EXTERNAL_CLOCK_EDGE_BOTH            (0x10U) /* count at both edges */
/* counter clear select (CCLR2,CCLR1,CCLR0) */
#define _00_TMRD_COUNTER_CLEAR_DISABLE               (0x00U) /* clear disabled (free-running operation) */
#define _20_TMRD_COUNTER_CLEAR_TRDGRA                (0x20U) /* clear by input capture/compare match with TRDGRAm */
#define _40_TMRD_COUNTER_CLEAR_TRDGRB                (0x40U) /* clear by input capture/compare match with TRDGRBm */
#define _60_TMRD_COUNTER_CLEAR_SYNCHRONOUS           (0x60U) /* synchronous clear */
#define _A0_TMRD_COUNTER_CLEAR_TRDGRC                (0xA0U) /* clear by input capture/compare match with TRDGRCm */
#define _C0_TMRD_COUNTER_CLEAR_TRDGRD                (0xC0U) /* clear by input capture/compare match with TRDGRDm */

/*
    Timer RD I/O control register Am (TRDIORAm) 
*/
/* TRDGRA control (IOA1,IOA0) */
#define _00_TMRD_TRDGRA_CAPTURE_RISING               (0x00U) /* input capture to TRDGRAm at the rising edge */
#define _01_TMRD_TRDGRA_CAPTURE_FALLING              (0x01U) /* input capture to TRDGRAm at the falling edge */
#define _02_TMRD_TRDGRA_CAPTURE_BOTH                 (0x02U) /* input capture to TRDGRAm at both edges */
#define _00_TMRD_TRDGRA_COMPARE_OUTPUT_DISABLE       (0x00U) /* pin output by compare match is disabled */
#define _01_TMRD_TRDGRA_COMPARE_OUTPUT_LOW           (0x01U) /* low output by compare match with TRDGRAm */
#define _02_TMRD_TRDGRA_COMPARE_OUTPUT_HIGH          (0x02U) /* high output by compare match with TRDGRAm */
#define _03_TMRD_TRDGRA_COMPARE_OUTPUT_TOGGLE        (0x03U) /* toggle output by compare match with TRDGRAm */
/* TRDGRA mode select (IOA2) */
#define _00_TMRD_TRDGRA_COMPARE                      (0x00U) /* output compare function */
#define _04_TMRD_TRDGRA_CAPTURE                      (0x04U) /* input capture function */
/* TRDGRB control (IOB1,IOB0) */
#define _00_TMRD_TRDGRB_CAPTURE_RISING               (0x00U) /* input capture to TRDGRBm at the rising edge */
#define _10_TMRD_TRDGRB_CAPTURE_FALLING              (0x10U) /* input capture to TRDGRBm at the falling edge */
#define _20_TMRD_TRDGRB_CAPTURE_BOTH                 (0x20U) /* input capture to TRDGRBm at both edges */
#define _00_TMRD_TRDGRB_COMPARE_OUTPUT_DISABLE       (0x00U) /* pin output by compare match is disabled */
#define _10_TMRD_TRDGRB_COMPARE_OUTPUT_LOW           (0x10U) /* low output by compare match with TRDGRBm */
#define _20_TMRD_TRDGRB_COMPARE_OUTPUT_HIGH          (0x20U) /* high output by compare match with TRDGRBm */
#define _30_TMRD_TRDGRB_COMPARE_OUTPUT_TOGGLE        (0x30U) /* toggle output by compare match with TRDGRBm */
/* TRDGRB mode select (IOB2) */
#define _00_TMRD_TRDGRB_COMPARE                      (0x00U) /* output compare function */
#define _40_TMRD_TRDGRB_CAPTURE                      (0x40U) /* input capture function */

/*
    Timer RD I/O control register Cm (TRDIORCm) 
*/
/* TRDGRC control (IOC1,IOC0) */
#define _00_TMRD_TRDGRC_CAPTURE_RISING               (0x00U) /* input capture to TRDGRCm at the rising edge */
#define _01_TMRD_TRDGRC_CAPTURE_FALLING              (0x01U) /* input capture to TRDGRCm at the falling edge */
#define _02_TMRD_TRDGRC_CAPTURE_BOTH                 (0x02U) /* input capture to TRDGRCm at both edges */
#define _00_TMRD_TRDGRC_COMPARE_OUTPUT_DISABLE       (0x00U) /* pin output by compare match is disabled */
#define _01_TMRD_TRDGRC_COMPARE_OUTPUT_LOW           (0x01U) /* low output by compare match with TRDGRCm */
#define _02_TMRD_TRDGRC_COMPARE_OUTPUT_HIGH          (0x02U) /* high output by compare match with TRDGRCm */
#define _03_TMRD_TRDGRC_COMPARE_OUTPUT_TOGGLE        (0x03U) /* toggle output by compare match with TRDGRCm */
/* TRDGRD control (IOD1,IOD0) */
#define _00_TMRD_TRDGRD_CAPTURE_RISING               (0x00U) /* input capture to TRDGRDm at the rising edge */
#define _10_TMRD_TRDGRD_CAPTURE_FALLING              (0x10U) /* input capture to TRDGRDm at the falling edge */
#define _20_TMRD_TRDGRD_CAPTURE_BOTH                 (0x20U) /* input capture to TRDGRDm at both edges */
#define _00_TMRD_TRDGRD_COMPARE_OUTPUT_DISABLE       (0x00U) /* pin output by compare match is disabled */
#define _10_TMRD_TRDGRD_COMPARE_OUTPUT_LOW           (0x10U) /* low output by compare match with TRDGRDm */
#define _20_TMRD_TRDGRD_COMPARE_OUTPUT_HIGH          (0x20U) /* high output by compare match with TRDGRDm */
#define _30_TMRD_TRDGRD_COMPARE_OUTPUT_TOGGLE        (0x30U) /* toggle output by compare match with TRDGRDm */
/* TRDGRC mode select (IOC2) */
#define _00_TMRD_TRDGRC_COMPARE                      (0x00U) /* output compare function */
#define _04_TMRD_TRDGRC_CAPTURE                      (0x04U) /* input capture function */
/* TRDGRC register function select (IOC3) */
#define _00_TMRD_TRDGRC_OUTPUT_REGISTER              (0x00U) /* TRDIOA output register */
#define _08_TMRD_TRDGRC_GENERAL_BUFFER_REGISTER      (0x08U) /* general register or buffer register */
/* TRDGRD mode select (IOD2) */
#define _00_TMRD_TRDGRD_COMPARE                      (0x00U) /* output compare function */
#define _40_TMRD_TRDGRD_CAPTURE                      (0x40U) /* input capture function */
/* TRDGRD register function select (IOD3) */
#define _00_TMRD_TRDGRD_OUTPUT_REGISTER              (0x00U) /* TRDIOB output register */
#define _80_TMRD_TRDGRD_GENERAL_BUFFER_REGISTER      (0x80U) /* general register or buffer register */

/*
    Timer RD interrupt enable register m (TRDIERm) 
*/
/* Input capture/compare match interrupt enable A (IMIEA) */
#define _00_TMRD_IMIA_DISABLE                        (0x00U) /* interrupt (IMIA) by the IMFA bit is disabled */
#define _01_TMRD_IMIA_ENABLE                         (0x01U) /* interrupt (IMIA) by the IMFA bit is enabled */
/* Input capture/compare match interrupt enable B (IMIEB) */
#define _00_TMRD_IMIB_DISABLE                        (0x00U) /* interrupt (IMIB) by the IMFB bit is disabled */
#define _02_TMRD_IMIB_ENABLE                         (0x02U) /* interrupt (IMIB) by the IMFB bit is enabled */
/* Input capture/compare match interrupt enable C (IMIEC) */
#define _00_TMRD_IMIC_DISABLE                        (0x00U) /* interrupt (IMIC) by the IMFC bit is disabled */
#define _04_TMRD_IMIC_ENABLE                         (0x04U) /* interrupt (IMIC) by the IMFC bit is enabled */
/* Input capture/compare match interrupt enable D (IMIED) */
#define _00_TMRD_IMID_DISABLE                        (0x00U) /* interrupt (IMID) by the IMFD bit is disabled */
#define _08_TMRD_IMID_ENABLE                         (0x08U) /* interrupt (IMID) by the IMFD bit is enabled */
/* Overflow/underflow interrupt enable (OVIE) */
#define _00_TMRD_OVIE_DISABLE                        (0x00U) /* interrupt (OVI) by bits OVF and UDF disabled */
#define _10_TMRD_OVIE_ENABLE                         (0x10U) /* interrupt (OVI) by bits OVF and UDF enabled */

/*
    Timer RD PWM function output level control register m (TRDPOCRm) 
*/
/* PWM function output level control B (POLB) */
#define _00_TRDIOB_OUTPUT_ACTIVE_L                   (0x00U) /* TRDIOBm output level is low active */
#define _01_TRDIOB_OUTPUT_ACTIVE_H                   (0x01U) /* TRDIOBm output level is high active */
/* PWM function output level control C (POLC) */
#define _00_TRDIOC_OUTPUT_ACTIVE_L                   (0x00U) /* TRDIOCm output level is low active */
#define _02_TRDIOC_OUTPUT_ACTIVE_H                   (0x02U) /* TRDIOCm output level is high active */
/* PWM function output level control D (POLD) */
#define _00_TRDIOD_OUTPUT_ACTIVE_L                   (0x00U) /* TRDIODm output level is low active */
#define _04_TRDIOD_OUTPUT_ACTIVE_H                   (0x04U) /* TRDIODm output level is high active */


/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define _0C5F_TMRD_TDRGRA0_VALUE                         (0x0C5FU) /* Timer RD general register A0 */
#define _0B62_TMRD_TDRGRA1_VALUE                         (0x0B62U) /* Timer RD general register A1 */
#define _0B62_TMRD_TDRGRB0_VALUE                         (0x0B62U) /* Timer RD general register B0 */
#define _0B62_TMRD_TDRGRB1_VALUE                         (0x0B62U) /* Timer RD general register B1 */
#define _0C5F_TMRD_TDRGRC0_VALUE                         (0x0C5FU) /* Timer RD general register C0 */
#define _0B62_TMRD_TDRGRC1_VALUE                         (0x0B62U) /* Timer RD general register C1 */
#define _0B62_TMRD_TDRGRD0_VALUE                         (0x0B62U) /* Timer RD general register D0 */
#define _0B62_TMRD_TDRGRD1_VALUE                         (0x0B62U) /* Timer RD general register D1 */ 

#define _12BF_TMRD_TDRGRA0_VALUE                         (0x12BFU) /* Timer RD general register A0 */
#define _095F_TMRD_TDRGRA1_VALUE                         (0x095FU) /* Timer RD general register A1 */
#define _095F_TMRD_TDRGRB0_VALUE                         (0x095FU) /* Timer RD general register B0 */
#define _095F_TMRD_TDRGRB1_VALUE                         (0x095FU) /* Timer RD general register B1 */
#define _12BF_TMRD_TDRGRC0_VALUE                         (0x12BFU) /* Timer RD general register C0 */
#define _095F_TMRD_TDRGRC1_VALUE                         (0x095FU) /* Timer RD general register C1 */
#define _095F_TMRD_TDRGRD0_VALUE                         (0x095FU) /* Timer RD general register D0 */
#define _095F_TMRD_TDRGRD1_VALUE                         (0x095FU) /* Timer RD general register D1 */

#define _0000_TMRD_TDR0_VALUE                            (0x0000U) /* Timer RD count 0 */
#define _0BB6_TMRD_TDRGRA0_VALUE                         (0x0BB6U) /* Timer RD general register A0 */
#define _0BB6_TMRD_TDRGRA1_VALUE                         (0x0BB6U) /* Timer RD general register A1 */
#define _0BB6_TMRD_TDRGRB0_VALUE                         (0x0BB6U) /* Timer RD general register B0 */
#define _0BB6_TMRD_TDRGRB1_VALUE                         (0x0BB6U) /* Timer RD general register B1 */
#define _0BB6_TMRD_TDRGRC1_VALUE                         (0x0BB6U) /* Timer RD general register C1 */
#define _0BB6_TMRD_TDRGRD0_VALUE                         (0x0BB6U) /* Timer RD general register D0 */
#define _0BB6_TMRD_TDRGRD1_VALUE                         (0x0BB6U) /* Timer RD general register D1 */

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
typedef enum
{
    TMCHANNELA,
    TMCHANNELB,
    TMCHANNELC,
    TMCHANNELD,
    TMCHANNELELC
} tmrd_channel_t;

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void R_TMRD0_Create(void);
void R_TMRD0_Start(void);
void R_TMRD0_Stop(void);
/* Start user code for function. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#endif
