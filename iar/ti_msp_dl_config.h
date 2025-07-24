/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G351X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G351X
#define CONFIG_MSPM0G3519

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define GPIO_LFXT_PORT                                                     GPIOA
#define GPIO_LFXIN_PIN                                             DL_GPIO_PIN_3
#define GPIO_LFXIN_IOMUX                                          (IOMUX_PINCM8)
#define GPIO_LFXOUT_PIN                                            DL_GPIO_PIN_4
#define GPIO_LFXOUT_IOMUX                                         (IOMUX_PINCM9)
#define CPUCLK_FREQ                                                     80000000



/* Defines for MOTOR */
#define MOTOR_INST                                                         TIMA0
#define MOTOR_INST_IRQHandler                                   TIMA0_IRQHandler
#define MOTOR_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define MOTOR_INST_CLK_FREQ                                              1000000
/* GPIO defines for channel 0 */
#define GPIO_MOTOR_C0_PORT                                                 GPIOB
#define GPIO_MOTOR_C0_PIN                                          DL_GPIO_PIN_3
#define GPIO_MOTOR_C0_IOMUX                                      (IOMUX_PINCM16)
#define GPIO_MOTOR_C0_IOMUX_FUNC                     IOMUX_PINCM16_PF_TIMA0_CCP0
#define GPIO_MOTOR_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_MOTOR_C1_PORT                                                 GPIOB
#define GPIO_MOTOR_C1_PIN                                          DL_GPIO_PIN_4
#define GPIO_MOTOR_C1_IOMUX                                      (IOMUX_PINCM17)
#define GPIO_MOTOR_C1_IOMUX_FUNC                     IOMUX_PINCM17_PF_TIMA0_CCP1
#define GPIO_MOTOR_C1_IDX                                    DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_MOTOR_C2_PORT                                                 GPIOB
#define GPIO_MOTOR_C2_PIN                                          DL_GPIO_PIN_0
#define GPIO_MOTOR_C2_IOMUX                                      (IOMUX_PINCM12)
#define GPIO_MOTOR_C2_IOMUX_FUNC                     IOMUX_PINCM12_PF_TIMA0_CCP2
#define GPIO_MOTOR_C2_IDX                                    DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_MOTOR_C3_PORT                                                 GPIOB
#define GPIO_MOTOR_C3_PIN                                          DL_GPIO_PIN_2
#define GPIO_MOTOR_C3_IOMUX                                      (IOMUX_PINCM15)
#define GPIO_MOTOR_C3_IOMUX_FUNC                     IOMUX_PINCM15_PF_TIMA0_CCP3
#define GPIO_MOTOR_C3_IDX                                    DL_TIMER_CC_3_INDEX

/* Defines for SERVO1_2 */
#define SERVO1_2_INST                                                      TIMA1
#define SERVO1_2_INST_IRQHandler                                TIMA1_IRQHandler
#define SERVO1_2_INST_INT_IRQN                                  (TIMA1_INT_IRQn)
#define SERVO1_2_INST_CLK_FREQ                                            100000
/* GPIO defines for channel 0 */
#define GPIO_SERVO1_2_C0_PORT                                              GPIOA
#define GPIO_SERVO1_2_C0_PIN                                      DL_GPIO_PIN_28
#define GPIO_SERVO1_2_C0_IOMUX                                    (IOMUX_PINCM3)
#define GPIO_SERVO1_2_C0_IOMUX_FUNC                   IOMUX_PINCM3_PF_TIMA1_CCP0
#define GPIO_SERVO1_2_C0_IDX                                 DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_SERVO1_2_C1_PORT                                              GPIOA
#define GPIO_SERVO1_2_C1_PIN                                      DL_GPIO_PIN_31
#define GPIO_SERVO1_2_C1_IOMUX                                    (IOMUX_PINCM6)
#define GPIO_SERVO1_2_C1_IOMUX_FUNC                   IOMUX_PINCM6_PF_TIMA1_CCP1
#define GPIO_SERVO1_2_C1_IDX                                 DL_TIMER_CC_1_INDEX

/* Defines for SERVO3_4 */
#define SERVO3_4_INST                                                     TIMG12
#define SERVO3_4_INST_IRQHandler                               TIMG12_IRQHandler
#define SERVO3_4_INST_INT_IRQN                                 (TIMG12_INT_IRQn)
#define SERVO3_4_INST_CLK_FREQ                                          80000000
/* GPIO defines for channel 0 */
#define GPIO_SERVO3_4_C0_PORT                                              GPIOA
#define GPIO_SERVO3_4_C0_PIN                                       DL_GPIO_PIN_0
#define GPIO_SERVO3_4_C0_IOMUX                                    (IOMUX_PINCM1)
#define GPIO_SERVO3_4_C0_IOMUX_FUNC                  IOMUX_PINCM1_PF_TIMG12_CCP0
#define GPIO_SERVO3_4_C0_IDX                                 DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_SERVO3_4_C1_PORT                                              GPIOA
#define GPIO_SERVO3_4_C1_PIN                                       DL_GPIO_PIN_1
#define GPIO_SERVO3_4_C1_IOMUX                                    (IOMUX_PINCM2)
#define GPIO_SERVO3_4_C1_IOMUX_FUNC                  IOMUX_PINCM2_PF_TIMG12_CCP1
#define GPIO_SERVO3_4_C1_IDX                                 DL_TIMER_CC_1_INDEX

/* Defines for SERVO5_6 */
#define SERVO5_6_INST                                                      TIMG7
#define SERVO5_6_INST_IRQHandler                                TIMG7_IRQHandler
#define SERVO5_6_INST_INT_IRQN                                  (TIMG7_INT_IRQn)
#define SERVO5_6_INST_CLK_FREQ                                          80000000
/* GPIO defines for channel 0 */
#define GPIO_SERVO5_6_C0_PORT                                              GPIOA
#define GPIO_SERVO5_6_C0_PIN                                      DL_GPIO_PIN_26
#define GPIO_SERVO5_6_C0_IOMUX                                   (IOMUX_PINCM59)
#define GPIO_SERVO5_6_C0_IOMUX_FUNC                  IOMUX_PINCM59_PF_TIMG7_CCP0
#define GPIO_SERVO5_6_C0_IDX                                 DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_SERVO5_6_C1_PORT                                              GPIOA
#define GPIO_SERVO5_6_C1_PIN                                      DL_GPIO_PIN_27
#define GPIO_SERVO5_6_C1_IOMUX                                   (IOMUX_PINCM60)
#define GPIO_SERVO5_6_C1_IOMUX_FUNC                  IOMUX_PINCM60_PF_TIMG7_CCP1
#define GPIO_SERVO5_6_C1_IDX                                 DL_TIMER_CC_1_INDEX

/* Defines for SERVO7_8 */
#define SERVO7_8_INST                                                      TIMG6
#define SERVO7_8_INST_IRQHandler                                TIMG6_IRQHandler
#define SERVO7_8_INST_INT_IRQN                                  (TIMG6_INT_IRQn)
#define SERVO7_8_INST_CLK_FREQ                                          80000000
/* GPIO defines for channel 0 */
#define GPIO_SERVO7_8_C0_PORT                                              GPIOB
#define GPIO_SERVO7_8_C0_PIN                                      DL_GPIO_PIN_26
#define GPIO_SERVO7_8_C0_IOMUX                                   (IOMUX_PINCM57)
#define GPIO_SERVO7_8_C0_IOMUX_FUNC                  IOMUX_PINCM57_PF_TIMG6_CCP0
#define GPIO_SERVO7_8_C0_IDX                                 DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_SERVO7_8_C1_PORT                                              GPIOB
#define GPIO_SERVO7_8_C1_PIN                                      DL_GPIO_PIN_27
#define GPIO_SERVO7_8_C1_IOMUX                                   (IOMUX_PINCM58)
#define GPIO_SERVO7_8_C1_IOMUX_FUNC                  IOMUX_PINCM58_PF_TIMG6_CCP1
#define GPIO_SERVO7_8_C1_IDX                                 DL_TIMER_CC_1_INDEX




/* Defines for Encoder_1 */
#define Encoder_1_INST                                                     TIMG9
#define Encoder_1_INST_IRQHandler                               TIMG9_IRQHandler
#define Encoder_1_INST_INT_IRQN                                 (TIMG9_INT_IRQn)
/* Pin configuration defines for Encoder_1 PHA Pin */
#define GPIO_Encoder_1_PHA_PORT                                            GPIOB
#define GPIO_Encoder_1_PHA_PIN                                     DL_GPIO_PIN_7
#define GPIO_Encoder_1_PHA_IOMUX                                 (IOMUX_PINCM24)
#define GPIO_Encoder_1_PHA_IOMUX_FUNC                IOMUX_PINCM24_PF_TIMG9_CCP0
/* Pin configuration defines for Encoder_1 PHB Pin */
#define GPIO_Encoder_1_PHB_PORT                                            GPIOB
#define GPIO_Encoder_1_PHB_PIN                                     DL_GPIO_PIN_9
#define GPIO_Encoder_1_PHB_IOMUX                                 (IOMUX_PINCM26)
#define GPIO_Encoder_1_PHB_IOMUX_FUNC                IOMUX_PINCM26_PF_TIMG9_CCP1

/* Defines for Encoder_2 */
#define Encoder_2_INST                                                     TIMG8
#define Encoder_2_INST_IRQHandler                               TIMG8_IRQHandler
#define Encoder_2_INST_INT_IRQN                                 (TIMG8_INT_IRQn)
/* Pin configuration defines for Encoder_2 PHA Pin */
#define GPIO_Encoder_2_PHA_PORT                                            GPIOB
#define GPIO_Encoder_2_PHA_PIN                                    DL_GPIO_PIN_15
#define GPIO_Encoder_2_PHA_IOMUX                                 (IOMUX_PINCM32)
#define GPIO_Encoder_2_PHA_IOMUX_FUNC                IOMUX_PINCM32_PF_TIMG8_CCP0
/* Pin configuration defines for Encoder_2 PHB Pin */
#define GPIO_Encoder_2_PHB_PORT                                            GPIOB
#define GPIO_Encoder_2_PHB_PIN                                    DL_GPIO_PIN_16
#define GPIO_Encoder_2_PHB_IOMUX                                 (IOMUX_PINCM33)
#define GPIO_Encoder_2_PHB_IOMUX_FUNC                IOMUX_PINCM33_PF_TIMG8_CCP1



/* Defines for MPU6050 */
#define MPU6050_INST                                                        I2C0
#define MPU6050_INST_IRQHandler                                  I2C0_IRQHandler
#define MPU6050_INST_INT_IRQN                                      I2C0_INT_IRQn
#define GPIO_MPU6050_SDA_PORT                                              GPIOB
#define GPIO_MPU6050_SDA_PIN                                      DL_GPIO_PIN_22
#define GPIO_MPU6050_IOMUX_SDA                                   (IOMUX_PINCM50)
#define GPIO_MPU6050_IOMUX_SDA_FUNC                    IOMUX_PINCM50_PF_I2C0_SDA
#define GPIO_MPU6050_SCL_PORT                                              GPIOB
#define GPIO_MPU6050_SCL_PIN                                      DL_GPIO_PIN_21
#define GPIO_MPU6050_IOMUX_SCL                                   (IOMUX_PINCM49)
#define GPIO_MPU6050_IOMUX_SCL_FUNC                    IOMUX_PINCM49_PF_I2C0_SCL


/* Defines for UART_6 */
#define UART_6_INST                                                        UART6
#define UART_6_INST_FREQUENCY                                           80000000
#define UART_6_INST_IRQHandler                                  UART6_IRQHandler
#define UART_6_INST_INT_IRQN                                      UART6_INT_IRQn
#define GPIO_UART_6_RX_PORT                                                GPIOC
#define GPIO_UART_6_TX_PORT                                                GPIOC
#define GPIO_UART_6_RX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_6_TX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_6_IOMUX_RX                                     (IOMUX_PINCM88)
#define GPIO_UART_6_IOMUX_TX                                     (IOMUX_PINCM89)
#define GPIO_UART_6_IOMUX_RX_FUNC                      IOMUX_PINCM88_PF_UART6_RX
#define GPIO_UART_6_IOMUX_TX_FUNC                      IOMUX_PINCM89_PF_UART6_TX
#define UART_6_BAUD_RATE                                                (115200)
#define UART_6_IBRD_80_MHZ_115200_BAUD                                      (43)
#define UART_6_FBRD_80_MHZ_115200_BAUD                                      (26)




/* Defines for LCD */
#define LCD_INST                                                           SPI1
#define LCD_INST_IRQHandler                                     SPI1_IRQHandler
#define LCD_INST_INT_IRQN                                         SPI1_INT_IRQn
#define GPIO_LCD_PICO_PORT                                                GPIOB
#define GPIO_LCD_PICO_PIN                                        DL_GPIO_PIN_30
#define GPIO_LCD_IOMUX_PICO                                     (IOMUX_PINCM67)
#define GPIO_LCD_IOMUX_PICO_FUNC                     IOMUX_PINCM67_PF_SPI1_PICO
#define GPIO_LCD_POCI_PORT                                                GPIOB
#define GPIO_LCD_POCI_PIN                                        DL_GPIO_PIN_14
#define GPIO_LCD_IOMUX_POCI                                     (IOMUX_PINCM31)
#define GPIO_LCD_IOMUX_POCI_FUNC                     IOMUX_PINCM31_PF_SPI1_POCI
/* GPIO configuration for LCD */
#define GPIO_LCD_SCLK_PORT                                                GPIOB
#define GPIO_LCD_SCLK_PIN                                        DL_GPIO_PIN_31
#define GPIO_LCD_IOMUX_SCLK                                     (IOMUX_PINCM68)
#define GPIO_LCD_IOMUX_SCLK_FUNC                     IOMUX_PINCM68_PF_SPI1_SCLK



/* Defines for A1 */
#define A1_INST                                                             ADC1
#define A1_INST_IRQHandler                                       ADC1_IRQHandler
#define A1_INST_INT_IRQN                                         (ADC1_INT_IRQn)
#define A1_ADCMEM_CCD1_AO                                     DL_ADC12_MEM_IDX_0
#define A1_ADCMEM_CCD1_AO_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define A1_ADCMEM_CCD2_AO                                     DL_ADC12_MEM_IDX_1
#define A1_ADCMEM_CCD2_AO_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define A1_ADCMEM_CCD3_AO                                     DL_ADC12_MEM_IDX_2
#define A1_ADCMEM_CCD3_AO_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define A1_ADCMEM_CCD4_AO                                     DL_ADC12_MEM_IDX_3
#define A1_ADCMEM_CCD4_AO_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA_VSSA
#define GPIO_A1_C5_PORT                                                    GPIOB
#define GPIO_A1_C5_PIN                                            DL_GPIO_PIN_18
#define GPIO_A1_C6_PORT                                                    GPIOB
#define GPIO_A1_C6_PIN                                            DL_GPIO_PIN_19
#define GPIO_A1_C4_PORT                                                    GPIOB
#define GPIO_A1_C4_PIN                                            DL_GPIO_PIN_17
#define GPIO_A1_C2_PORT                                                    GPIOA
#define GPIO_A1_C2_PIN                                            DL_GPIO_PIN_17



/* Port definition for Pin Group PORTB */
#define PORTB_PORT                                                       (GPIOB)

/* Defines for LED: GPIOB.5 with pinCMx 18 on package pin 26 */
#define PORTB_LED_PIN                                            (DL_GPIO_PIN_5)
#define PORTB_LED_IOMUX                                          (IOMUX_PINCM18)
/* Defines for CCD_CLK1: GPIOB.20 with pinCMx 48 on package pin 82 */
#define PORTB_CCD_CLK1_PIN                                      (DL_GPIO_PIN_20)
#define PORTB_CCD_CLK1_IOMUX                                     (IOMUX_PINCM48)
/* Defines for SLEEP: GPIOB.1 with pinCMx 13 on package pin 21 */
#define PORTB_SLEEP_PIN                                          (DL_GPIO_PIN_1)
#define PORTB_SLEEP_IOMUX                                        (IOMUX_PINCM13)
/* Defines for LCD_CS: GPIOB.28 with pinCMx 65 on package pin 29 */
#define PORTB_LCD_CS_PIN                                        (DL_GPIO_PIN_28)
#define PORTB_LCD_CS_IOMUX                                       (IOMUX_PINCM65)
/* Defines for LCD_DC: GPIOB.29 with pinCMx 66 on package pin 30 */
#define PORTB_LCD_DC_PIN                                        (DL_GPIO_PIN_29)
#define PORTB_LCD_DC_IOMUX                                       (IOMUX_PINCM66)
/* Port definition for Pin Group PORTA */
#define PORTA_PORT                                                       (GPIOA)

/* Defines for S0: GPIOA.18 with pinCMx 40 on package pin 70 */
#define PORTA_S0_PIN                                            (DL_GPIO_PIN_18)
#define PORTA_S0_IOMUX                                           (IOMUX_PINCM40)
/* Defines for PHA0: GPIOA.14 with pinCMx 36 on package pin 53 */
// pins affected by this interrupt request:["PHA0","PHB0"]
#define PORTA_INT_IRQN                                          (GPIOA_INT_IRQn)
#define PORTA_INT_IIDX                          (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define PORTA_PHA0_IIDX                                     (DL_GPIO_IIDX_DIO14)
#define PORTA_PHA0_PIN                                          (DL_GPIO_PIN_14)
#define PORTA_PHA0_IOMUX                                         (IOMUX_PINCM36)
/* Defines for PHB0: GPIOA.15 with pinCMx 37 on package pin 54 */
#define PORTA_PHB0_IIDX                                     (DL_GPIO_IIDX_DIO15)
#define PORTA_PHB0_PIN                                          (DL_GPIO_PIN_15)
#define PORTA_PHB0_IOMUX                                         (IOMUX_PINCM37)
/* Defines for PUSH: GPIOA.12 with pinCMx 34 on package pin 51 */
#define PORTA_PUSH_PIN                                          (DL_GPIO_PIN_12)
#define PORTA_PUSH_IOMUX                                         (IOMUX_PINCM34)
/* Defines for BUZZ: GPIOA.13 with pinCMx 35 on package pin 52 */
#define PORTA_BUZZ_PIN                                          (DL_GPIO_PIN_13)
#define PORTA_BUZZ_IOMUX                                         (IOMUX_PINCM35)
/* Defines for S2: GPIOA.16 with pinCMx 38 on package pin 55 */
#define PORTA_S2_PIN                                            (DL_GPIO_PIN_16)
#define PORTA_S2_IOMUX                                           (IOMUX_PINCM38)
/* Defines for FAULT: GPIOA.7 with pinCMx 14 on package pin 22 */
#define PORTA_FAULT_PIN                                          (DL_GPIO_PIN_7)
#define PORTA_FAULT_IOMUX                                        (IOMUX_PINCM14)
/* Defines for LCD_RST: GPIOA.8 with pinCMx 19 on package pin 27 */
#define PORTA_LCD_RST_PIN                                        (DL_GPIO_PIN_8)
#define PORTA_LCD_RST_IOMUX                                      (IOMUX_PINCM19)
/* Defines for LCD_BL: GPIOA.9 with pinCMx 20 on package pin 28 */
#define PORTA_LCD_BL_PIN                                         (DL_GPIO_PIN_9)
#define PORTA_LCD_BL_IOMUX                                       (IOMUX_PINCM20)
/* Port definition for Pin Group PORTC */
#define PORTC_PORT                                                       (GPIOC)

/* Defines for CCD_SI1: GPIOC.9 with pinCMx 87 on package pin 81 */
#define PORTC_CCD_SI1_PIN                                        (DL_GPIO_PIN_9)
#define PORTC_CCD_SI1_IOMUX                                      (IOMUX_PINCM87)
/* Defines for CCD_SI2: GPIOC.4 with pinCMx 78 on package pin 67 */
#define PORTC_CCD_SI2_PIN                                        (DL_GPIO_PIN_4)
#define PORTC_CCD_SI2_IOMUX                                      (IOMUX_PINCM78)
/* Defines for CCD_CLK2: GPIOC.5 with pinCMx 79 on package pin 68 */
#define PORTC_CCD_CLK2_PIN                                       (DL_GPIO_PIN_5)
#define PORTC_CCD_CLK2_IOMUX                                     (IOMUX_PINCM79)
/* Defines for S1: GPIOC.0 with pinCMx 74 on package pin 56 */
#define PORTC_S1_PIN                                             (DL_GPIO_PIN_0)
#define PORTC_S1_IOMUX                                           (IOMUX_PINCM74)
/* Defines for IMU_INT: GPIOC.8 with pinCMx 86 on package pin 80 */
// pins affected by this interrupt request:["IMU_INT"]
#define PORTC_INT_IRQN                                          (GPIOC_INT_IRQn)
#define PORTC_INT_IIDX                          (DL_INTERRUPT_GROUP1_IIDX_GPIOC)
#define PORTC_IMU_INT_IIDX                                   (DL_GPIO_IIDX_DIO8)
#define PORTC_IMU_INT_PIN                                        (DL_GPIO_PIN_8)
#define PORTC_IMU_INT_IOMUX                                      (IOMUX_PINCM86)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_MOTOR_init(void);
void SYSCFG_DL_SERVO1_2_init(void);
void SYSCFG_DL_SERVO3_4_init(void);
void SYSCFG_DL_SERVO5_6_init(void);
void SYSCFG_DL_SERVO7_8_init(void);
void SYSCFG_DL_Encoder_1_init(void);
void SYSCFG_DL_Encoder_2_init(void);
void SYSCFG_DL_MPU6050_init(void);
void SYSCFG_DL_UART_6_init(void);
void SYSCFG_DL_LCD_init(void);
void SYSCFG_DL_A1_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
