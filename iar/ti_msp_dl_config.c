/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G351X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gMOTORBackup;
DL_TimerA_backupConfig gSERVO1_2Backup;
DL_TimerG_backupConfig gSERVO5_6Backup;
DL_TimerG_backupConfig gSERVO7_8Backup;
DL_TimerG_backupConfig gEncoder_2Backup;
DL_SPI_backupConfig gLCDBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_MOTOR_init();
    SYSCFG_DL_SERVO1_2_init();
    SYSCFG_DL_SERVO3_4_init();
    SYSCFG_DL_SERVO5_6_init();
    SYSCFG_DL_SERVO7_8_init();
    SYSCFG_DL_Encoder_1_init();
    SYSCFG_DL_Encoder_2_init();
    SYSCFG_DL_MPU6050_init();
    SYSCFG_DL_UART_6_init();
    SYSCFG_DL_LCD_init();
    SYSCFG_DL_A1_init();
    SYSCFG_DL_SYSTICK_init();
    /* Ensure backup structures have no valid state */
	gMOTORBackup.backupRdy 	= false;
	gSERVO1_2Backup.backupRdy 	= false;
	gSERVO5_6Backup.backupRdy 	= false;
	gSERVO7_8Backup.backupRdy 	= false;
	gEncoder_2Backup.backupRdy 	= false;

	gLCDBackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(MOTOR_INST, &gMOTORBackup);
	retStatus &= DL_TimerA_saveConfiguration(SERVO1_2_INST, &gSERVO1_2Backup);
	retStatus &= DL_TimerG_saveConfiguration(SERVO5_6_INST, &gSERVO5_6Backup);
	retStatus &= DL_TimerG_saveConfiguration(SERVO7_8_INST, &gSERVO7_8Backup);
	retStatus &= DL_TimerG_saveConfiguration(Encoder_2_INST, &gEncoder_2Backup);
	retStatus &= DL_SPI_saveConfiguration(LCD_INST, &gLCDBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(MOTOR_INST, &gMOTORBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(SERVO1_2_INST, &gSERVO1_2Backup, false);
	retStatus &= DL_TimerG_restoreConfiguration(SERVO5_6_INST, &gSERVO5_6Backup, false);
	retStatus &= DL_TimerG_restoreConfiguration(SERVO7_8_INST, &gSERVO7_8Backup, false);
	retStatus &= DL_TimerG_restoreConfiguration(Encoder_2_INST, &gEncoder_2Backup, false);
	retStatus &= DL_SPI_restoreConfiguration(LCD_INST, &gLCDBackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_GPIO_reset(GPIOC);
    DL_TimerA_reset(MOTOR_INST);
    DL_TimerA_reset(SERVO1_2_INST);
    DL_TimerG_reset(SERVO3_4_INST);
    DL_TimerG_reset(SERVO5_6_INST);
    DL_TimerG_reset(SERVO7_8_INST);
    DL_TimerG_reset(Encoder_1_INST);
    DL_TimerG_reset(Encoder_2_INST);
    DL_I2C_reset(MPU6050_INST);
    DL_UART_Main_reset(UART_6_INST);
    DL_SPI_reset(LCD_INST);
    DL_ADC12_reset(A1_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_GPIO_enablePower(GPIOC);
    DL_TimerA_enablePower(MOTOR_INST);
    DL_TimerA_enablePower(SERVO1_2_INST);
    DL_TimerG_enablePower(SERVO3_4_INST);
    DL_TimerG_enablePower(SERVO5_6_INST);
    DL_TimerG_enablePower(SERVO7_8_INST);
    DL_TimerG_enablePower(Encoder_1_INST);
    DL_TimerG_enablePower(Encoder_2_INST);
    DL_I2C_enablePower(MPU6050_INST);
    DL_UART_Main_enablePower(UART_6_INST);
    DL_SPI_enablePower(LCD_INST);
    DL_ADC12_enablePower(A1_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXOUT_IOMUX);

    DL_GPIO_initPeripheralOutputFunction(GPIO_MOTOR_C0_IOMUX,GPIO_MOTOR_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MOTOR_C0_PORT, GPIO_MOTOR_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_MOTOR_C1_IOMUX,GPIO_MOTOR_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MOTOR_C1_PORT, GPIO_MOTOR_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_MOTOR_C2_IOMUX,GPIO_MOTOR_C2_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MOTOR_C2_PORT, GPIO_MOTOR_C2_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_MOTOR_C3_IOMUX,GPIO_MOTOR_C3_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MOTOR_C3_PORT, GPIO_MOTOR_C3_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SERVO1_2_C0_IOMUX,GPIO_SERVO1_2_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SERVO1_2_C0_PORT, GPIO_SERVO1_2_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SERVO1_2_C1_IOMUX,GPIO_SERVO1_2_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SERVO1_2_C1_PORT, GPIO_SERVO1_2_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SERVO3_4_C0_IOMUX,GPIO_SERVO3_4_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SERVO3_4_C0_PORT, GPIO_SERVO3_4_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SERVO3_4_C1_IOMUX,GPIO_SERVO3_4_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SERVO3_4_C1_PORT, GPIO_SERVO3_4_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SERVO5_6_C0_IOMUX,GPIO_SERVO5_6_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SERVO5_6_C0_PORT, GPIO_SERVO5_6_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SERVO5_6_C1_IOMUX,GPIO_SERVO5_6_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SERVO5_6_C1_PORT, GPIO_SERVO5_6_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SERVO7_8_C0_IOMUX,GPIO_SERVO7_8_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SERVO7_8_C0_PORT, GPIO_SERVO7_8_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SERVO7_8_C1_IOMUX,GPIO_SERVO7_8_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SERVO7_8_C1_PORT, GPIO_SERVO7_8_C1_PIN);

    DL_GPIO_initPeripheralInputFunction(GPIO_Encoder_1_PHA_IOMUX,GPIO_Encoder_1_PHA_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_Encoder_1_PHB_IOMUX,GPIO_Encoder_1_PHB_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_Encoder_2_PHA_IOMUX,GPIO_Encoder_2_PHA_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_Encoder_2_PHB_IOMUX,GPIO_Encoder_2_PHB_IOMUX_FUNC);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MPU6050_IOMUX_SDA,
        GPIO_MPU6050_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MPU6050_IOMUX_SCL,
        GPIO_MPU6050_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_MPU6050_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_MPU6050_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_6_IOMUX_TX, GPIO_UART_6_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_6_IOMUX_RX, GPIO_UART_6_IOMUX_RX_FUNC);

    DL_GPIO_initPeripheralInputFunction(
        GPIO_LCD_IOMUX_POCI, GPIO_LCD_IOMUX_POCI_FUNC);
    
	DL_GPIO_initPeripheralOutputFunction(
		 GPIO_LCD_IOMUX_SCLK, GPIO_LCD_IOMUX_SCLK_FUNC);
	DL_GPIO_initPeripheralOutputFunction(
		 GPIO_LCD_IOMUX_PICO, GPIO_LCD_IOMUX_PICO_FUNC);

    DL_GPIO_initDigitalOutput(PORTB_LED_IOMUX);

    DL_GPIO_initDigitalOutput(PORTB_CCD_CLK1_IOMUX);

    DL_GPIO_initDigitalOutput(PORTB_SLEEP_IOMUX);

    DL_GPIO_initDigitalOutput(PORTB_LCD_CS_IOMUX);

    DL_GPIO_initDigitalOutput(PORTB_LCD_DC_IOMUX);

    DL_GPIO_initDigitalInputFeatures(PORTA_S0_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(PORTA_PHA0_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(PORTA_PHB0_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(PORTA_PUSH_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(PORTA_BUZZ_IOMUX);

    DL_GPIO_initDigitalInputFeatures(PORTA_S2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(PORTA_FAULT_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(PORTA_LCD_RST_IOMUX);

    DL_GPIO_initDigitalOutput(PORTA_LCD_BL_IOMUX);

    DL_GPIO_initDigitalOutput(PORTC_CCD_SI1_IOMUX);

    DL_GPIO_initDigitalOutput(PORTC_CCD_SI2_IOMUX);

    DL_GPIO_initDigitalOutput(PORTC_CCD_CLK2_IOMUX);

    DL_GPIO_initDigitalInputFeatures(PORTC_S1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(PORTC_IMU_INT_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(PORTA_PORT, PORTA_BUZZ_PIN |
		PORTA_LCD_RST_PIN |
		PORTA_LCD_BL_PIN);
    DL_GPIO_enableOutput(PORTA_PORT, PORTA_BUZZ_PIN |
		PORTA_LCD_RST_PIN |
		PORTA_LCD_BL_PIN);
    DL_GPIO_setLowerPinsPolarity(PORTA_PORT, DL_GPIO_PIN_14_EDGE_FALL |
		DL_GPIO_PIN_15_EDGE_FALL);
    DL_GPIO_clearInterruptStatus(PORTA_PORT, PORTA_PHA0_PIN |
		PORTA_PHB0_PIN);
    DL_GPIO_enableInterrupt(PORTA_PORT, PORTA_PHA0_PIN |
		PORTA_PHB0_PIN);
    DL_GPIO_clearPins(PORTB_PORT, PORTB_LED_PIN |
		PORTB_CCD_CLK1_PIN |
		PORTB_SLEEP_PIN |
		PORTB_LCD_CS_PIN |
		PORTB_LCD_DC_PIN);
    DL_GPIO_enableOutput(PORTB_PORT, PORTB_LED_PIN |
		PORTB_CCD_CLK1_PIN |
		PORTB_SLEEP_PIN |
		PORTB_LCD_CS_PIN |
		PORTB_LCD_DC_PIN);
    DL_GPIO_clearPins(PORTC_PORT, PORTC_CCD_SI1_PIN |
		PORTC_CCD_SI2_PIN |
		PORTC_CCD_CLK2_PIN);
    DL_GPIO_enableOutput(PORTC_PORT, PORTC_CCD_SI1_PIN |
		PORTC_CCD_SI2_PIN |
		PORTC_CCD_CLK2_PIN);
    DL_GPIO_setLowerPinsPolarity(PORTC_PORT, DL_GPIO_PIN_8_EDGE_FALL);
    DL_GPIO_clearInterruptStatus(PORTC_PORT, PORTC_IMU_INT_PIN);
    DL_GPIO_enableInterrupt(PORTC_PORT, PORTC_IMU_INT_PIN);

}


static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ,
	.rDivClk2x              = 1,
	.rDivClk1               = 0,
	.rDivClk0               = 0,
	.enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
	.enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
	.enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
	.sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK0,
	.sysPLLRef              = DL_SYSCTL_SYSPLL_REF_HFCLK,
	.qDiv                   = 3,
	.pDiv                   = DL_SYSCTL_SYSPLL_PDIV_1
};
static const DL_SYSCTL_LFCLKConfig gLFCLKConfig = {
    .lowCap   = false,
    .monitor  = false,
    .xt1Drive = DL_SYSCTL_LFXT_DRIVE_STRENGTH_HIGHEST,
};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setHFCLKSourceHFXTParams(DL_SYSCTL_HFXT_RANGE_32_48_MHZ,16, false);
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_enableMFCLK();
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
    /* INT_GROUP1 Priority */
    NVIC_SetPriority(GPIOC_INT_IRQn, 1);

}


/*
 * Timer clock configuration to be sourced by  / 8 (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1000000 Hz = 10000000 Hz / (8 * (9 + 1))
 */
static const DL_TimerA_ClockConfig gMOTORClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 9U
};

static const DL_TimerA_PWMConfig gMOTORConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 100,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_MOTOR_init(void) {

    DL_TimerA_setClockConfig(
        MOTOR_INST, (DL_TimerA_ClockConfig *) &gMOTORClockConfig);

    DL_TimerA_initPWMMode(
        MOTOR_INST, (DL_TimerA_PWMConfig *) &gMOTORConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(MOTOR_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(MOTOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(MOTOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(MOTOR_INST, 100, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(MOTOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(MOTOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(MOTOR_INST, 100, DL_TIMER_CC_1_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(MOTOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_2_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(MOTOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_2_INDEX);
    DL_TimerA_setCaptureCompareValue(MOTOR_INST, 100, DL_TIMER_CC_2_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(MOTOR_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_3_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(MOTOR_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_3_INDEX);
    DL_TimerA_setCaptureCompareValue(MOTOR_INST, 100, DL_TIMER_CC_3_INDEX);

    DL_TimerA_enableClock(MOTOR_INST);


    
    DL_TimerA_setCCPDirection(MOTOR_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT | DL_TIMER_CC2_OUTPUT | DL_TIMER_CC3_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 8 (10000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   100000 Hz = 10000000 Hz / (8 * (99 + 1))
 */
static const DL_TimerA_ClockConfig gSERVO1_2ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 99U
};

static const DL_TimerA_PWMConfig gSERVO1_2Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 2000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_SERVO1_2_init(void) {

    DL_TimerA_setClockConfig(
        SERVO1_2_INST, (DL_TimerA_ClockConfig *) &gSERVO1_2ClockConfig);

    DL_TimerA_initPWMMode(
        SERVO1_2_INST, (DL_TimerA_PWMConfig *) &gSERVO1_2Config);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(SERVO1_2_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(SERVO1_2_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(SERVO1_2_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(SERVO1_2_INST, 2000, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(SERVO1_2_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(SERVO1_2_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(SERVO1_2_INST, 2000, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(SERVO1_2_INST);


    
    DL_TimerA_setCCPDirection(SERVO1_2_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   80000000 Hz = 80000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gSERVO3_4ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gSERVO3_4Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 2000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_SERVO3_4_init(void) {

    DL_TimerG_setClockConfig(
        SERVO3_4_INST, (DL_TimerG_ClockConfig *) &gSERVO3_4ClockConfig);

    DL_TimerG_initPWMMode(
        SERVO3_4_INST, (DL_TimerG_PWMConfig *) &gSERVO3_4Config);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(SERVO3_4_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(SERVO3_4_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SERVO3_4_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(SERVO3_4_INST, 2000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(SERVO3_4_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SERVO3_4_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(SERVO3_4_INST, 2000, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(SERVO3_4_INST);


    
    DL_TimerG_setCCPDirection(SERVO3_4_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   80000000 Hz = 80000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gSERVO5_6ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gSERVO5_6Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 2000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_SERVO5_6_init(void) {

    DL_TimerG_setClockConfig(
        SERVO5_6_INST, (DL_TimerG_ClockConfig *) &gSERVO5_6ClockConfig);

    DL_TimerG_initPWMMode(
        SERVO5_6_INST, (DL_TimerG_PWMConfig *) &gSERVO5_6Config);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(SERVO5_6_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(SERVO5_6_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SERVO5_6_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(SERVO5_6_INST, 2000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(SERVO5_6_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SERVO5_6_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(SERVO5_6_INST, 2000, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(SERVO5_6_INST);


    
    DL_TimerG_setCCPDirection(SERVO5_6_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   80000000 Hz = 80000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gSERVO7_8ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gSERVO7_8Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 2000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_SERVO7_8_init(void) {

    DL_TimerG_setClockConfig(
        SERVO7_8_INST, (DL_TimerG_ClockConfig *) &gSERVO7_8ClockConfig);

    DL_TimerG_initPWMMode(
        SERVO7_8_INST, (DL_TimerG_PWMConfig *) &gSERVO7_8Config);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(SERVO7_8_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(SERVO7_8_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SERVO7_8_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(SERVO7_8_INST, 2000, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(SERVO7_8_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SERVO7_8_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(SERVO7_8_INST, 2000, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(SERVO7_8_INST);


    
    DL_TimerG_setCCPDirection(SERVO7_8_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}


static const DL_TimerG_ClockConfig gEncoder_1ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};


SYSCONFIG_WEAK void SYSCFG_DL_Encoder_1_init(void) {

    DL_TimerG_setClockConfig(
        Encoder_1_INST, (DL_TimerG_ClockConfig *) &gEncoder_1ClockConfig);

    DL_TimerG_configQEI(Encoder_1_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(Encoder_1_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(Encoder_1_INST, 65535);
    DL_TimerG_enableClock(Encoder_1_INST);
    DL_TimerG_startCounter(Encoder_1_INST);
}
static const DL_TimerG_ClockConfig gEncoder_2ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};


SYSCONFIG_WEAK void SYSCFG_DL_Encoder_2_init(void) {

    DL_TimerG_setClockConfig(
        Encoder_2_INST, (DL_TimerG_ClockConfig *) &gEncoder_2ClockConfig);

    DL_TimerG_configQEI(Encoder_2_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(Encoder_2_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(Encoder_2_INST, 65535);
    DL_TimerG_enableClock(Encoder_2_INST);
    DL_TimerG_startCounter(Encoder_2_INST);
}


static const DL_I2C_ClockConfig gMPU6050ClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_MPU6050_init(void) {

    DL_I2C_setClockConfig(MPU6050_INST,
        (DL_I2C_ClockConfig *) &gMPU6050ClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(MPU6050_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(MPU6050_INST);




}

static const DL_UART_Main_ClockConfig gUART_6ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_6Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_6_init(void)
{
    DL_UART_Main_setClockConfig(UART_6_INST, (DL_UART_Main_ClockConfig *) &gUART_6ClockConfig);

    DL_UART_Main_init(UART_6_INST, (DL_UART_Main_Config *) &gUART_6Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(UART_6_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_6_INST, UART_6_IBRD_80_MHZ_115200_BAUD, UART_6_FBRD_80_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_6_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(UART_6_INST_INT_IRQN, 2);


    DL_UART_Main_enable(UART_6_INST);
}

static const DL_SPI_Config gLCD_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA1,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gLCD_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_LCD_init(void) {
    DL_SPI_setClockConfig(LCD_INST, (DL_SPI_ClockConfig *) &gLCD_clockConfig);

    DL_SPI_init(LCD_INST, (DL_SPI_Config *) &gLCD_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     10000000 = (80000000)/((1 + 3) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(LCD_INST, 3);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(LCD_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);

    /* Enable module */
    DL_SPI_enable(LCD_INST);
}

/* A1 Initialization */
static const DL_ADC12_ClockConfig gA1ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_A1_init(void)
{
    DL_ADC12_setClockConfig(A1_INST, (DL_ADC12_ClockConfig *) &gA1ClockConfig);

    DL_ADC12_initSeqSample(A1_INST,
        DL_ADC12_REPEAT_MODE_DISABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SEQ_START_ADDR_00, DL_ADC12_SEQ_END_ADDR_03, DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(A1_INST, A1_ADCMEM_CCD1_AO,
        DL_ADC12_INPUT_CHAN_5, DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(A1_INST, A1_ADCMEM_CCD2_AO,
        DL_ADC12_INPUT_CHAN_6, DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(A1_INST, A1_ADCMEM_CCD3_AO,
        DL_ADC12_INPUT_CHAN_4, DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_configConversionMem(A1_INST, A1_ADCMEM_CCD4_AO,
        DL_ADC12_INPUT_CHAN_2, DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_enableConversions(A1_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSTICK_init(void)
{
    /*
     * Initializes the SysTick period to 1.00 ms,
     * enables the interrupt, and starts the SysTick Timer
     */
    DL_SYSTICK_config(80000);
}

