
#ifndef __TAU_H
#define __TAU_H   

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/*
    Timer Clock Select Register m (TPSm) 
*/
/* Selection of operation clock (PRSm03,PRSm02,PRSm01,PRSm00) */
#define _0000_TAU_CKM0_fCLK_0                 (0x0000U) /* ckm0 - fCLK */
#define _0001_TAU_CKM0_fCLK_1                 (0x0001U) /* ckm0 - fCLK/2^1 */
#define _0002_TAU_CKM0_fCLK_2                 (0x0002U) /* ckm0 - fCLK/2^2 */
#define _0003_TAU_CKM0_fCLK_3                 (0x0003U) /* ckm0 - fCLK/2^3 */
#define _0004_TAU_CKM0_fCLK_4                 (0x0004U) /* ckm0 - fCLK/2^4 */
#define _0005_TAU_CKM0_fCLK_5                 (0x0005U) /* ckm0 - fCLK/2^5 */
#define _0006_TAU_CKM0_fCLK_6                 (0x0006U) /* ckm0 - fCLK/2^6 */
#define _0007_TAU_CKM0_fCLK_7                 (0x0007U) /* ckm0 - fCLK/2^7 */
#define _0008_TAU_CKM0_fCLK_8                 (0x0008U) /* ckm0 - fCLK/2^8 */
#define _0009_TAU_CKM0_fCLK_9                 (0x0009U) /* ckm0 - fCLK/2^9 */
#define _000A_TAU_CKM0_fCLK_10                (0x000AU) /* ckm0 - fCLK/2^10 */
#define _000B_TAU_CKM0_fCLK_11                (0x000BU) /* ckm0 - fCLK/2^11 */
#define _000C_TAU_CKM0_fCLK_12                (0x000CU) /* ckm0 - fCLK/2^12 */
#define _000D_TAU_CKM0_fCLK_13                (0x000DU) /* ckm0 - fCLK/2^13 */
#define _000E_TAU_CKM0_fCLK_14                (0x000EU) /* ckm0 - fCLK/2^14 */
#define _000F_TAU_CKM0_fCLK_15                (0x000FU) /* ckm0 - fCLK/2^15 */
/* Selection of operation clock (PRSm13,PRSm12,PRSm11,PRSm10) */
#define _0000_TAU_CKM1_fCLK_0                 (0x0000U) /* ckm1 - fCLK */
#define _0010_TAU_CKM1_fCLK_1                 (0x0010U) /* ckm1 - fCLK/2^1 */
#define _0020_TAU_CKM1_fCLK_2                 (0x0020U) /* ckm1 - fCLK/2^2 */
#define _0030_TAU_CKM1_fCLK_3                 (0x0030U) /* ckm1 - fCLK/2^3 */
#define _0040_TAU_CKM1_fCLK_4                 (0x0040U) /* ckm1 - fCLK/2^4 */
#define _0050_TAU_CKM1_fCLK_5                 (0x0050U) /* ckm1 - fCLK/2^5 */
#define _0060_TAU_CKM1_fCLK_6                 (0x0060U) /* ckm1 - fCLK/2^6 */
#define _0070_TAU_CKM1_fCLK_7                 (0x0070U) /* ckm1 - fCLK/2^7 */
#define _0080_TAU_CKM1_fCLK_8                 (0x0080U) /* ckm1 - fCLK/2^8 */
#define _0090_TAU_CKM1_fCLK_9                 (0x0090U) /* ckm1 - fCLK/2^9 */
#define _00A0_TAU_CKM1_fCLK_10                (0x00A0U) /* ckm1 - fCLK/2^10 */
#define _00B0_TAU_CKM1_fCLK_11                (0x00B0U) /* ckm1 - fCLK/2^11 */
#define _00C0_TAU_CKM1_fCLK_12                (0x00C0U) /* ckm1 - fCLK/2^12 */
#define _00D0_TAU_CKM1_fCLK_13                (0x00D0U) /* ckm1 - fCLK/2^13 */
#define _00E0_TAU_CKM1_fCLK_14                (0x00E0U) /* ckm1 - fCLK/2^14 */
#define _00F0_TAU_CKM1_fCLK_15                (0x00F0U) /* ckm1 - fCLK/2^15 */
/* Selection of operation clock (PRSm21,PRSm20) */
#define _0000_TAU_CKM2_fCLK_1                 (0x0000U) /* ckm2 - fCLK/2^1 */
#define _0100_TAU_CKM2_fCLK_2                 (0x0100U) /* ckm2 - fCLK/2^2 */
#define _0200_TAU_CKM2_fCLK_4                 (0x0200U) /* ckm2 - fCLK/2^4 */
#define _0300_TAU_CKM2_fCLK_6                 (0x0300U) /* ckm2 - fCLK/2^6 */
/* Selection of operation clock (PRSm31,PRSm30) */
#define _0000_TAU_CKM3_fCLK_8                 (0x0000U) /* ckm3 - fCLK/2^8 */
#define _1000_TAU_CKM3_fCLK_10                (0x1000U) /* ckm3 - fCLK/2^10 */
#define _2000_TAU_CKM3_fCLK_12                (0x2000U) /* ckm3 - fCLK/2^12 */
#define _3000_TAU_CKM3_fCLK_14                (0x3000U) /* ckm3 - fCLK/2^14 */

/*
    Timer Channel Start Register m (TSm) 
*/
/* Operation enable (start) trigger of channel 3 higher 8 bits (TSHm3) */
#define _0000_TAU_CH3_H8_START_TRG_OFF        (0x0000U) /* no trigger operation */
#define _0800_TAU_CH3_H8_START_TRG_ON         (0x0800U) /* operation is enabled (start trigger is generated) */
/* Operation enable (start) trigger of channel 1 higher 8 bits (TSHm1) */
#define _0000_TAU_CH1_H8_START_TRG_OFF        (0x0000U) /* no trigger operation */
#define _0200_TAU_CH1_H8_START_TRG_ON         (0x0200U) /* operation is enabled (start trigger is generated) */
/* Operation enable (start) trigger of channel 3 (TSm3) */
#define _0000_TAU_CH3_START_TRG_OFF           (0x0000U) /* no trigger operation */
#define _0008_TAU_CH3_START_TRG_ON            (0x0008U) /* operation is enabled (start trigger is generated) */
/* Operation enable (start) trigger of channel 2 (TSm2) */
#define _0000_TAU_CH2_START_TRG_OFF           (0x0000U) /* no trigger operation */
#define _0004_TAU_CH2_START_TRG_ON            (0x0004U) /* operation is enabled (start trigger is generated) */
/* Operation enable (start) trigger of channel 1 (TSm1) */
#define _0000_TAU_CH1_START_TRG_OFF           (0x0000U) /* no trigger operation */
#define _0002_TAU_CH1_START_TRG_ON            (0x0002U) /* operation is enabled (start trigger is generated) */
/* Operation enable (start) trigger of channel 0 (TSm0) */
#define _0000_TAU_CH0_START_TRG_OFF           (0x0000U) /* no trigger operation */
#define _0001_TAU_CH0_START_TRG_ON            (0x0001U) /* operation is enabled (start trigger is generated) */

/*
    Timer Channel Stop Register m (TTm) 
*/
/* Operation stop trigger of channel 3 higher 8 bits (TTHm3) */
#define _0000_TAU_CH3_H8_STOP_TRG_OFF         (0x0000U) /* no trigger operation */
#define _0800_TAU_CH3_H8_STOP_TRG_ON          (0x0800U) /* operation is stopped (stop trigger is generated) */
/* Operation stop trigger of channel 1 higher 8 bits (TTHm1) */
#define _0000_TAU_CH1_H8_STOP_TRG_OFF         (0x0000U) /* no trigger operation */
#define _0200_TAU_CH1_H8_STOP_TRG_ON          (0x0200U) /* operation is stopped (stop trigger is generated) */
/* Operation stop trigger of channel 3 (TTm3) */
#define _0000_TAU_CH3_STOP_TRG_OFF            (0x0000U) /* no trigger operation */
#define _0008_TAU_CH3_STOP_TRG_ON             (0x0008U) /* operation is stopped (stop trigger is generated) */
/* Operation stop trigger of channel 2 (TTm2) */
#define _0000_TAU_CH2_STOP_TRG_OFF            (0x0000U) /* no trigger operation */
#define _0004_TAU_CH2_STOP_TRG_ON             (0x0004U) /* operation is stopped (stop trigger is generated) */
/* Operation stop trigger of channel 1 (TTm1) */
#define _0000_TAU_CH1_STOP_TRG_OFF            (0x0000U) /* no trigger operation */
#define _0002_TAU_CH1_STOP_TRG_ON             (0x0002U) /* operation is stopped (stop trigger is generated) */
/* Operation stop trigger of channel 0 (TTm0) */
#define _0000_TAU_CH0_STOP_TRG_OFF            (0x0000U) /* no trigger operation */
#define _0001_TAU_CH0_STOP_TRG_ON             (0x0001U) /* operation is stopped (stop trigger is generated) */

/*
    Timer Mode Register mn (TMRmn) 
*/
/* Selection of macro clock (MCK) of channel n (CKSmn1,CKSmn0) */
#define _0000_TAU_CLOCK_SELECT_CKM0           (0x0000U) /* operation clock CK0 set by PRS register */
#define _8000_TAU_CLOCK_SELECT_CKM1           (0x8000U) /* operation clock CK1 set by PRS register */
#define _4000_TAU_CLOCK_SELECT_CKM2           (0x4000U) /* operation clock CK2 set by PRS register */
#define _C000_TAU_CLOCK_SELECT_CKM3           (0xC000U) /* operation clock CK3 set by PRS register */
/* Selection of count clock (CCK) of channel n (CCSmn) */
#define _0000_TAU_CLOCK_MODE_CKS              (0x0000U) /* macro clock MCK specified by CKSmn bit */
#define _1000_TAU_CLOCK_MODE_TIMN             (0x1000U) /* valid edge of input signal input from TImn pin */
/* Operation explanation of channel 1 or 3 (SPLITmn) */
#define _0000_TAU_16BITS_MODE                 (0x0000U) /* operates as 16 bits timer */
#define _0800_TAU_8BITS_MODE                  (0x0800U) /* operates as 8 bits timer */
/* Selection of slave/master of channel n (MASTERmn) */
#define _0000_TAU_COMBINATION_SLAVE           (0x0000U) /* operates as slave channel */
#define _0800_TAU_COMBINATION_MASTER          (0x0800U) /* operates as master channel */
/* Setting of start trigger or capture trigger of channel n (STSmn2,STSmn1,STSmn0) */
#define _0000_TAU_TRIGGER_SOFTWARE            (0x0000U) /* only software trigger start is valid */
#define _0100_TAU_TRIGGER_TIMN_VALID          (0x0100U) /* Valid edge of the TImn pin input is used as both the start trigger and capture trigger */
#define _0200_TAU_TRIGGER_TIMN_BOTH           (0x0200U) /* Both the edges of the TImn pin input are used as a start trigger and a capture trigger */
#define _0400_TAU_TRIGGER_MASTER_INT          (0x0400U) /* interrupt signal of the master channel is used */
/* Selection of TImn pin input valid edge (CISmn1,CISmn0) */
#define _0000_TAU_TIMN_EDGE_FALLING           (0x0000U) /* falling edge */
#define _0040_TAU_TIMN_EDGE_RISING            (0x0040U) /* rising edge */
#define _0080_TAU_TIMN_EDGE_BOTH_LOW          (0x0080U) /* both edges (when low-level width is measured) */
#define _00C0_TAU_TIMN_EDGE_BOTH_HIGH         (0x00C0U) /* both edges (when high-level width is measured) */
/* Operation mode of channel n (MDmn3,MDmn2,MDmn1) */
#define _0000_TAU_MODE_INTERVAL_TIMER         (0x0000U) /* interval timer mode */
#define _0004_TAU_MODE_CAPTURE                (0x0004U) /* capture mode */
#define _0006_TAU_MODE_EVENT_COUNT            (0x0006U) /* event counter mode */
#define _0008_TAU_MODE_ONE_COUNT              (0x0008U) /* one count mode */
#define _000C_TAU_MODE_HIGHLOW_MEASURE        (0x000CU) /* high-/low-level width measurement mode */
/* Setting of starting counting and interrupt (MDmn0) */
#define _0000_TAU_START_INT_UNUSED            (0x0000U) /* interrupt is not generated when counting is started */
#define _0001_TAU_START_INT_USED              (0x0001U) /* interrupt is generated when counting is started */
/* Operation mode of channel n (MDmn3,MDmn2,MDmn1,MDmn0) */
#define _0001_TAU_MODE_PWM_MASTER             (0x0001U) /* PWM function (master channel) mode */
#define _0009_TAU_MODE_PWM_SLAVE              (0x0009U) /* PWM function (slave channel) mode */
#define _0008_TAU_MODE_ONESHOT                (0x0008U) /* one-shot pulse output mode */

/*
    Timer Output Mode Register m (TOMm) 
*/
/* Control of timer output mode of channel 3 (TOMm3) */
#define _0000_TAU_CH3_OUTPUT_TOGGLE           (0x0000U) /* toggle operation mode */
#define _0008_TAU_CH3_OUTPUT_COMBIN           (0x0008U) /* combination operation mode */
/* Control of timer output mode of channel 2 (TOMm2) */
#define _0000_TAU_CH2_OUTPUT_TOGGLE           (0x0000U) /* toggle operation mode */
#define _0004_TAU_CH2_OUTPUT_COMBIN           (0x0004U) /* combination operation mode */
/* Control of timer output mode of channel 1 (TOMm1) */
#define _0000_TAU_CH1_OUTPUT_TOGGLE           (0x0000U) /* toggle operation mode */
#define _0002_TAU_CH1_OUTPUT_COMBIN           (0x0002U) /* combination operation mode */

/*
    Timer Output Level Register 0 (TOLm) 
*/
/* Control of timer output level of channel 3 (TOLm3) */
#define _0000_TAU_CH3_OUTPUT_LEVEL_H          (0x0000U) /* positive logic output (active-high) */
#define _0008_TAU_CH3_OUTPUT_LEVEL_L          (0x0008U) /* inverted output (active-low) */
/* Control of timer output level of channel 2 (TOLm2) */
#define _0000_TAU_CH2_OUTPUT_LEVEL_H          (0x0000U) /* positive logic output (active-high) */
#define _0004_TAU_CH2_OUTPUT_LEVEL_L          (0x0004U) /* inverted output (active-low) */
/* Control of timer output level of channel 1 (TOLm1) */
#define _0000_TAU_CH1_OUTPUT_LEVEL_H          (0x0000U) /* positive logic output (active-high) */
#define _0002_TAU_CH1_OUTPUT_LEVEL_L          (0x0002U) /* inverted output (active-low) */

/*
    Timer Output Register m (TOm) 
*/
/* Timer output of channel 3 (TOm3) */
#define _0000_TAU_CH3_OUTPUT_VALUE_0          (0x0000U) /* timer output value is "0" */
#define _0008_TAU_CH3_OUTPUT_VALUE_1          (0x0008U) /* timer output value is "1" */
/* Timer output of channel 2 (TOm2) */
#define _0000_TAU_CH2_OUTPUT_VALUE_0          (0x0000U) /* timer output value is "0" */
#define _0004_TAU_CH2_OUTPUT_VALUE_1          (0x0004U) /* timer output value is "1" */
/* Timer output of channel 1 (TOm1) */
#define _0000_TAU_CH1_OUTPUT_VALUE_0          (0x0000U) /* timer output value is "0" */
#define _0002_TAU_CH1_OUTPUT_VALUE_1          (0x0002U) /* timer output value is "1" */
/* Timer output of channel 0 (TOm0) */
#define _0000_TAU_CH0_OUTPUT_VALUE_0          (0x0000U) /* timer output value is "0" */
#define _0001_TAU_CH0_OUTPUT_VALUE_1          (0x0001U) /* timer output value is "1" */

/*
    Timer Output Enable Register m (TOEm) 
*/
/* Timer output enable/disable of channel 3 (TOEm3) */
#define _0000_TAU_CH3_OUTPUT_DISABLE          (0x0000U) /* the TOm0 operation stopped by count operation */
#define _0008_TAU_CH3_OUTPUT_ENABLE           (0x0008U) /* the TOm0 operation enabled by count operation */
/* Timer output enable/disable of channel 2 (TOEm2) */
#define _0000_TAU_CH2_OUTPUT_DISABLE          (0x0000U) /* the TOm0 operation stopped by count operation */
#define _0004_TAU_CH2_OUTPUT_ENABLE           (0x0004U) /* the TOm0 operation enabled by count operation */
/* Timer output enable/disable of channel 1 (TOEm1) */
#define _0000_TAU_CH1_OUTPUT_DISABLE          (0x0000U) /* the TOm0 operation stopped by count operation */
#define _0002_TAU_CH1_OUTPUT_ENABLE           (0x0002U) /* the TOm0 operation enabled by count operation */
/* Timer output enable/disable of channel 0 (TOEm0) */
#define _0000_TAU_CH0_OUTPUT_DISABLE          (0x0000U) /* the TOm0 operation stopped by count operation */
#define _0001_TAU_CH0_OUTPUT_ENABLE           (0x0001U) /* the TOm0 operation enabled by count operation */

/*
    Timer Input Select Register m (TIS0) 
*/
/* Selection of timer input used with channel 0 (TIS04) */
#define _00_TAU_CH0_INPUT_TI00                (0x00U) /* input signal of timer input pin (TI00) */
#define _10_TAU_CH0_INPUT_ELC                 (0x10U) /* event input signal from ELC */
/* Selection of timer input used with channel 1 (TIS02,TIS01,TIS00) */
#define _00_TAU_CH1_INPUT_TI01                (0x00U) /* input signal of timer input pin (TI01) */
#define _01_TAU_CH1_INPUT_ELC                 (0x01U) /* event input signal from ELC */
#define _02_TAU_CH1_INPUT_TI01                (0x02U) /* input signal of timer input pin (TI01) */
#define _03_TAU_CH1_INPUT_TI01                (0x03U) /* input signal of timer input pin (TI01) */
#define _04_TAU_CH1_INPUT_fIL                 (0x04U) /* low-speed on-chip oscillator clock (fIL) */

/*
    Noise Filter Enable Register 1  (NFEN1) 
*/
/* Enable/disable using noise filter of TI03 pin input signal (TNFEN03) */
#define _00_TAU_CH3_NOISE_OFF                 (0x00U) /* noise filter OFF */
#define _08_TAU_CH3_NOISE_ON                  (0x08U) /* noise filter ON */
/* Enable/disable using noise filter of TI02 pin input signal (TNFEN02) */
#define _00_TAU_CH2_NOISE_OFF                 (0x00U) /* noise filter OFF */
#define _04_TAU_CH2_NOISE_ON                  (0x04U) /* noise filter ON */
/* Enable/disable using noise filter of TI01 pin input signal (TNFEN01) */
#define _00_TAU_CH1_NOISE_OFF                 (0x00U) /* noise filter OFF */
#define _02_TAU_CH1_NOISE_ON                  (0x02U) /* noise filter ON */
/* Enable/disable using noise filter of TI00 pin input signal (TNFEN00) */
#define _00_TAU_CH0_NOISE_OFF                 (0x00U) /* noise filter OFF */
#define _01_TAU_CH0_NOISE_ON                  (0x01U) /* noise filter ON */

/*
    Timer Status Register mn (TSRmn)
*/
/* Counter overflow status of channel n (OVF) */
#define _0000_TAU_OVERFLOW_NOT_OCCURS         (0x0000U) /* overflow does not occur */
#define _0001_TAU_OVERFLOW_OCCURS             (0x0001U) /* overflow occurs */

/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define _FFFF_TAU_TDR00_VALUE     (0xFFFFU)
#define _FFFF_TAU_TDR01_VALUE     (0xFFFFU)
#define _FFFF_TAU_TDR02_VALUE     (0xFFFFU)
#define _FFFF_TAU_TDR03_VALUE     (0xEA5FU)
/* Clock divisor for TAU0 channel 0 */
#define TAU0_CHANNEL0_DIVISOR                 (16U)      /* fCLK/2^4 */
/* Clock divisor for TAU0 channel 1 */
#define TAU0_CHANNEL1_DIVISOR                 (16U)      /* fCLK/2^4 */
/* Clock divisor for TAU0 channel 2 */
#define TAU0_CHANNEL2_DIVISOR                 (16U)      /* fCLK/2^4 */
/* Clock divisor for TAU0 channel 3 */
#define TAU0_CHANNEL3_DIVISOR                 (4U)      /* fCLK/2^2 */

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
//中断向量号INTTM00(以下宏为定时器T0-4通道间隔定时中断)
#define  DISABLE_T0CHAN_0_INT        TMMK00 = 1U 
#define  ENABLE_T0CHAN_0_INT         TMMK00 = 0U
#define  CLR_T0CHAN_0_INTFLAG        TMIF00 = 0U
#define  SET_T0CHAN_0_INTPRIO0       TMPR100 = 0;TMPR000 = 0 //设置通道0优先级0   
#define  SET_T0CHAN_0_INTPRIO1       TMPR100 = 0;TMPR000 = 1 //设置通道0优先级1
#define  SET_T0CHAN_0_INTPRIO2       TMPR100 = 1;TMPR000 = 0 //设置通道0优先级2
#define  SET_T0CHAN_0_INTPRIO3       TMPR100 = 1;TMPR000 = 1 //设置通道0优先级3

#define  DISABLE_T0CHAN_1_INT        TMMK01  = 1U 
#define  ENABLE_T0CHAN_1_INT         TMMK01  = 0U
#define  CLR_T0CHAN_1_INTFLAG        TMIF01  = 0U
#define  DISABLE_T0CHAN_1_HIGH8_INT  TMMK01H = 1U
#define  ENABLE_T0CHAN_1_HIGH8_INT   TMMK01H = 0U
#define  CLR_T0CHAN_1_HIGH8_INTFLAG  TMIF01H = 0U
#define  SET_T0CHAN_1_INTPRIO0       TMPR101 = 0;TMPR001 = 0 //设置通道1优先级0
#define  SET_T0CHAN_1_INTPRIO1       TMPR101 = 0;TMPR001 = 1 //设置通道1优先级1
#define  SET_T0CHAN_1_INTPRIO2       TMPR101 = 1;TMPR001 = 0 //设置通道1优先级2
#define  SET_T0CHAN_1_INTPRIO3       TMPR101 = 1;TMPR001 = 1 //设置通道1优先级3

#define  DISABLE_T0CHAN_2_INT        TMMK02 = 1U 
#define  ENABLE_T0CHAN_2_INT         TMMK02 = 0U
#define  CLR_T0CHAN_2_INTFLAG        TMIF02 = 0U
#define  SET_T0CHAN_2_INTPRIO0       TMPR102 = 0;TMPR002 = 0 //设置通道2优先级0
#define  SET_T0CHAN_2_INTPRIO1       TMPR102 = 0;TMPR002 = 1 //设置通道2优先级1
#define  SET_T0CHAN_2_INTPRIO2       TMPR102 = 1;TMPR002 = 0 //设置通道2优先级2
#define  SET_T0CHAN_2_INTPRIO3       TMPR102 = 1;TMPR002 = 1 //设置通道2优先级3

#define  DISABLE_T0CHAN_3_INT        TMMK03 = 1U 
#define  ENABLE_T0CHAN_3_INT         TMMK03 = 0U
#define  CLR_T0CHAN_3_INTFLAG        TMIF03 = 0U
#define  DISABLE_T0CHAN_3_HIGH8_INT  TMMK03H = 1U
#define  ENABLE_T0CHAN_3_HIGH8_INT   TMMK03H = 0U
#define  CLR_T0CHAN_3_HIGH8_INTFLAG  TMIF03H = 0U
#define  SET_T0CHAN_3_INTPRIO0       TMPR103 = 0;TMPR003 = 0 //设置通道3优先级0
#define  SET_T0CHAN_3_INTPRIO1       TMPR103 = 0;TMPR003 = 1 //设置通道3优先级1
#define  SET_T0CHAN_3_INTPRIO2       TMPR103 = 1;TMPR003 = 0 //设置通道3优先级2
#define  SET_T0CHAN_3_INTPRIO3       TMPR103 = 1;TMPR003 = 1 //设置通道3优先级3

#define  TIMEFLAG_T0CHAN_3           TMKAIF//T0通道3定时溢出标志


/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void R_TAU0_Create(void);
void R_TAU0_Channel0_Start(void);
void R_TAU0_Channel0_Stop(void);
void R_TAU0_Channel1_Start(void);
void R_TAU0_Channel1_Stop(void);
void R_TAU0_Channel2_Start(void);
void R_TAU0_Channel2_Stop(void);
void R_TAU0_Channel3_Start(void);
void R_TAU0_Channel3_Stop(void);

#endif
