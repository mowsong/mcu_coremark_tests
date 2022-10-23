#include "sh32f2053.h"
#include "sh32f2053_periph_init.h"

/*=====<<IC:SH32F2053 PACKAGE:LQFP64>>=====*/
#if USE_STD_LIBRARY

void GPIO_DevInit(void)
{
    /*=====<<GPIO_DevInit WIZARD CODE START>>=====*/
	RCC_AHBPeriphClockOnOff(RCC_AHB_GPIO,ON);

	//PA.00: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, XTAL2
	//PA.01: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, XTAL1
	//PA.02: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, JTRST
	//PA.03: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, JTDI
	//PA.04: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, JTMS_SWDIO
	//PA.05: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, JTCK_SWCLK
	//PA.06: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, JTDO_SWO
	//PA.11: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, RXD1
	//PA.12: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, TXD1
	GPIOA->ODR = 0x0;
	GPIOA->MODER = 0x0;
	GPIOA_CFG->OTYPER = 0x0;
	GPIOA_CFG->ODRVR.V32 = 0x0;
	GPIOA_CFG->PUPDR.V32 = 0x0;
	GPIOA_CFG->AFRL.V32 = 0xF0000000;
	GPIOA_CFG->AFRH.V32 = 0xFFF77FFF;
	GPIOA_CFG->TTLEN.V32 = 0x0;
	GPIOA_CFG->LCKR.V32 = 0x0;

	//PB.00: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.01: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.02: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.03: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.04: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.07: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.08: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.09: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.10: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.11: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.12: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.13: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.14: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PB.15: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	GPIOB->ODR = 0x0;
	GPIOB->MODER = 0x0;
	GPIOB_CFG->OTYPER = 0x0;
	GPIOB_CFG->ODRVR.V32 = 0x0;
	GPIOB_CFG->PUPDR.V32 = 0x0;
	GPIOB_CFG->AFRL.V32 = 0xFF00000;
	GPIOB_CFG->AFRH.V32 = 0x0;
	GPIOB_CFG->TTLEN.V32 = 0x0;
	GPIOB_CFG->LCKR.V32 = 0x0;

	//PC.00: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.01: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.02: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.03: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.04: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.05: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.06: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.07: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.08: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.09: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.10: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.11: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.12: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.13: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.14: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PC.15: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	GPIOC->ODR = 0x0;
	GPIOC->MODER = 0x0;
	GPIOC_CFG->OTYPER = 0x0;
	GPIOC_CFG->ODRVR.V32 = 0x0;
	GPIOC_CFG->PUPDR.V32 = 0x0;
	GPIOC_CFG->AFRL.V32 = 0x0;
	GPIOC_CFG->AFRH.V32 = 0x0;
	GPIOC_CFG->TTLEN.V32 = 0x0;
	GPIOC_CFG->LCKR.V32 = 0x0;

	//PD.00: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PD.07: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PD.08: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PD.09: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PD.10: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PD.15: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	GPIOD->ODR = 0x0;
	GPIOD->MODER = 0x0;
	GPIOD_CFG->OTYPER = 0x0;
	GPIOD_CFG->ODRVR.V32 = 0x0;
	GPIOD_CFG->PUPDR.V32 = 0x0;
	GPIOD_CFG->AFRL.V32 = 0xFFFFFF0;
	GPIOD_CFG->AFRH.V32 = 0xFFFF000;
	GPIOD_CFG->TTLEN.V32 = 0x0;
	GPIOD_CFG->LCKR.V32 = 0x0;

	//PE.04: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PE.05: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PE.06: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PE.07: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PE.14: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PE.08: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PE.00: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PE.01: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PE.02: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PE.03: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PE.12: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	//PE.13: IN,     , Push Pull,  Drive Normal,  No Pull,      CMOS, Unlock, Init:0, IO
	GPIOE->ODR = 0x0;
	GPIOE->MODER = 0x0;
	GPIOE_CFG->OTYPER = 0x0;
	GPIOE_CFG->ODRVR.V32 = 0x0;
	GPIOE_CFG->PUPDR.V32 = 0x0;
	GPIOE_CFG->AFRL.V32 = 0x0;
	GPIOE_CFG->AFRH.V32 = 0xF000FFF0;
	GPIOE_CFG->TTLEN.V32 = 0x0;
	GPIOE_CFG->LCKR.V32 = 0x0;

    /*=====<<GPIO_DevInit WIZARD CODE END>>=====*/
}


#ifdef _MODULE_MCM

void MCM1_DevInit(void)
{
    
    /*=====<<MCM1_DevInit WIZARD CODE START>>=====*/
	const MCM_TimeBaseInitTypeDef  mcmInit = {
		MCM_CounterMode_Edge,    /*.CounterMode*/
		0,                     /*.Prescaler*/
		0                      /*.Period*/
	};

	const MCM_PWMInitTypeDef  pwmInit_0 = {
		MCM_PWMMode_Complementary, /*.PWMMode*/
		MCM_PWM_Symmetry,          /*.PWMSymmetry*/ 
		MCM_DutyArea_1,            /*.DutyArea*/
		MCM_DutyPolarity_High,     /*.DutyPolarity*/
		(MCM_DutyActivePoint_Now), /*.DutyActivePoint*/
		0                        /*.DutyValue*/
	};

	const MCM_PWMInitTypeDef  pwmInit_01 = {
		MCM_PWMMode_Complementary, /*.PWMMode*/
		MCM_PWM_Symmetry,          /*.PWMSymmetry*/ 
		MCM_DutyArea_1,            /*.DutyArea*/
		MCM_DutyPolarity_High,     /*.DutyPolarity*/
		(MCM_DutyActivePoint_Now), /*.DutyActivePoint*/
		0                        /*.DutyValue*/
	};

	const MCM_PWMInitTypeDef  pwmInit_1 = {
		MCM_PWMMode_Complementary, /*.PWMMode*/
		MCM_PWM_Symmetry,          /*.PWMSymmetry*/ 
		MCM_DutyArea_1,            /*.DutyArea*/
		MCM_DutyPolarity_High,     /*.DutyPolarity*/
		(MCM_DutyActivePoint_Now), /*.DutyActivePoint*/
		0                        /*.DutyValue*/
	};

	const MCM_PWMInitTypeDef  pwmInit_11 = {
		MCM_PWMMode_Complementary, /*.PWMMode*/
		MCM_PWM_Symmetry,          /*.PWMSymmetry*/ 
		MCM_DutyArea_1,            /*.DutyArea*/
		MCM_DutyPolarity_High,     /*.DutyPolarity*/
		(MCM_DutyActivePoint_Now), /*.DutyActivePoint*/
		0                        /*.DutyValue*/
	};

	const MCM_PWMInitTypeDef  pwmInit_2 = {
		MCM_PWMMode_Complementary, /*.PWMMode*/
		MCM_PWM_Symmetry,          /*.PWMSymmetry*/ 
		MCM_DutyArea_1,            /*.DutyArea*/
		MCM_DutyPolarity_High,     /*.DutyPolarity*/
		(MCM_DutyActivePoint_Now), /*.DutyActivePoint*/
		0                        /*.DutyValue*/
	};

	const MCM_PWMInitTypeDef  pwmInit_21 = {
		MCM_PWMMode_Complementary, /*.PWMMode*/
		MCM_PWM_Symmetry,          /*.PWMSymmetry*/ 
		MCM_DutyArea_1,            /*.DutyArea*/
		MCM_DutyPolarity_High,     /*.DutyPolarity*/
		(MCM_DutyActivePoint_Now), /*.DutyActivePoint*/
		0                        /*.DutyValue*/
	};

	const MCM_SCInitTypeDef  sc1Init = {
		MCM_SC_InputLevel_High,  /*.SC_InputLevel*/
		0,    /*.SC_ProtectPWM*/
		MCM_SC_Filter_None,    /*.SC_Filter*/
		MCM_SC_ProtectTime_600CLK     /*.SC_ProtectTime*/
	};

	const MCM_SCInitTypeDef  sc2Init = {
		MCM_SC_InputLevel_High,  /*.SC_InputLevel*/
		0,    /*.SC_ProtectPWM*/
		MCM_SC_Filter_None,    /*.SC_Filter*/
		MCM_SC_ProtectTime_600CLK     /*.SC_ProtectTime*/
	};

	const MCM_SCInitTypeDef  sc3Init = {
		MCM_SC_InputLevel_High,  /*.SC_InputLevel*/
		0,    /*.SC_ProtectPWM*/
		MCM_SC_Filter_None,    /*.SC_Filter*/
		MCM_SC_ProtectTime_600CLK     /*.SC_ProtectTime*/
	};


	/* ---- Open Clock Gate --------- */
	RCC_APB2PeriphClockOnOff(RCC_APB2_MCM1,ON);

	/* ---- PWM Time Initial --------- */
	MCM_TimeBaseInit(MCM1,(MCM_TimeBaseInitTypeDef*)&mcmInit);
	MCM_PWM0Init(MCM1,(MCM_PWMInitTypeDef*)&pwmInit_0);
	MCM_PWM01Init(MCM1,(MCM_PWMInitTypeDef*)&pwmInit_01);
	MCM_PWM1Init(MCM1,(MCM_PWMInitTypeDef*)&pwmInit_1);
	MCM_PWM11Init(MCM1,(MCM_PWMInitTypeDef*)&pwmInit_11);
	MCM_PWM2Init(MCM1,(MCM_PWMInitTypeDef*)&pwmInit_2);
	MCM_PWM21Init(MCM1,(MCM_PWMInitTypeDef*)&pwmInit_21);

	MCM_PWM0DeadTimeConfig(MCM1,0,0);
	MCM_PWM1DeadTimeConfig(MCM1,0,0);
	MCM_PWM2DeadTimeConfig(MCM1,0,0);

	MCM_SetCounter(MCM1,0);

	/* ---- Post Period Scale --------- */
	MCM_IntEventFlagDivConfig(MCM1,MCM_INT_EVENT_FLAG_DIV_1);


	/* ---- PWM Out Initial --------- */
	MCM_ManualPWMOutOnOff(MCM1,MCM_MANUAL_PWM0,OFF);
	MCM_ManualPWMOutOnOff(MCM1,MCM_MANUAL_PWM01,OFF);
	MCM_ManualPWMOutOnOff(MCM1,MCM_MANUAL_PWM1,OFF);
	MCM_ManualPWMOutOnOff(MCM1,MCM_MANUAL_PWM11,OFF);
	MCM_ManualPWMOutOnOff(MCM1,MCM_MANUAL_PWM2,OFF);
	MCM_ManualPWMOutOnOff(MCM1,MCM_MANUAL_PWM21,OFF);

	/* ---- Interrupts --------- */

	/* ---- DMA --------- */

	/* ---- ADC --------- */
	MCM_ManualPWMOutRegSync(MCM1,MCM_MannualActiveTiming_Now);
	MCM_SetADCTriggerValue1(MCM1,0);
	MCM_SetADCTriggerValue2(MCM1,0);
	MCM_SetADCTriggerValue3(MCM1,0);
	MCM_SetADCTriggerValue4(MCM1,0);
	MCM_REG_UNLOCK(MCM1);
	MCM1->PWMCON2.V32 = 0x0;
	MCM_REG_LOCK(MCM1);

	/* ---- SAT --------- */
	MCM_DutySaturationMinMaxValue(MCM1,0x0,0x0);
	MCM_DutySaturationConfig(MCM1,MCM_MinSelect_None,MCM_MaxSelect_None);
	MCM_DutySaturationOnOff(MCM1,OFF);

	/* ---- OSL --------- */
	MCM_ShortCircuitProtectOnOff(MCM1,OFF);
	MCM_ShortCircuitOutPutLevel(MCM1,MCM_ShortCircuit_PWM0Pin,MCM_ShortCircuit_Level_Low);
	MCM_ShortCircuitOutPutLevel(MCM1,MCM_ShortCircuit_PWM01Pin,MCM_ShortCircuit_Level_Low);
	MCM_ShortCircuitOutPutLevel(MCM1,MCM_ShortCircuit_PWM1Pin,MCM_ShortCircuit_Level_Low);
	MCM_ShortCircuitOutPutLevel(MCM1,MCM_ShortCircuit_PWM11Pin,MCM_ShortCircuit_Level_Low);
	MCM_ShortCircuitOutPutLevel(MCM1,MCM_ShortCircuit_PWM2Pin,MCM_ShortCircuit_Level_Low);
	MCM_ShortCircuitOutPutLevel(MCM1,MCM_ShortCircuit_PWM21Pin,MCM_ShortCircuit_Level_Low);

	/* ---- SHIFT --------- */
	MCM_PhaseShiftConfig(MCM1,MCM_PS_MaxToMin_PWMD210,0x0,0x0);
	MCM_PhaseShiftOnOff(MCM1,OFF);

	/* ---- SC --------- */
	MCM_SC1Init(MCM1,(MCM_SCInitTypeDef*)&sc1Init);
	MCM_SC2Init(MCM1,(MCM_SCInitTypeDef*)&sc2Init);
	MCM_SC3Init(MCM1,(MCM_SCInitTypeDef*)&sc3Init);
	MCM_SCOnOff(MCM1,MCM_SC1,OFF);
	MCM_SCOnOff(MCM1,MCM_SC2,OFF);
	MCM_SCOnOff(MCM1,MCM_SC3,OFF);

	/* ---- FLT --------- */
	MCM_FLT1SourceConfig(MCM1,MCM_FLT1_Source_COMP1);
	MCM_FLT2Config(MCM1,MCM_FLT2_Filter_None,MCM_FLT2_ActiveLevel_High);
	MCM_FLTConfig(MCM1,MCM_FLT_DETECT_Latch,MCM_FLT_PWMx_PinStatus_Hiz,MCM_FLT_PWMx_PinStatus_Hiz);
	MCM_FLTOnOff(MCM1,MCM_FLT_COMP,OFF);
	MCM_FLTOnOff(MCM1,MCM_FLT_Pin,OFF);
	MCM_FLTWriteLock(MCM1,ENABLE);

	/* ---- MCM OSC Stop Detect On / Off --------- */
	MCM_OSCStopDetectOnOff(MCM1,OFF);

	/* ---- MCM On / Off --------- */
	MCM_OnOff(MCM1,OFF);
    /*=====<<MCM1_DEvInit WIZARD CODE END>>=====*/        
}

#endif /*_MODULE_MCM*/

#endif /*USE_STD_LIBRARY*/



void Peripherals_Init(void)
{
#if USE_STD_LIBRARY
    
    /*=====<<Peripherals_Init WIZARD CODE START>>=====*/
	GPIO_DevInit();
    /*=====<<Peripherals_Init WIZARD CODE END>>=====*/            
    
#endif /*USE_STD_LIBRARY*/    
}
