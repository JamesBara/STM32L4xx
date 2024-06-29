/******************************************************************************
* @file     startup_ARMCM4.c
* @brief    CMSIS-Core(M) Device Startup File for a Cortex-M4 Device
* @version  V2.0.3
* @date     31. March 2020
******************************************************************************/
/*
* Copyright (c) 2009-2020 Arm Limited. All rights reserved.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the License); you may
* not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an AS IS BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/


#include "stm32l4xx.h"

/*----------------------------------------------------------------------------
External References
*----------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;

extern __NO_RETURN void __PROGRAM_START(void);

/*----------------------------------------------------------------------------
Internal References
*----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler  (void);
void Default_Handler(void);

/*----------------------------------------------------------------------------
Exception / Interrupt Handler
*----------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((naked));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));
void WWDG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PVD_PVM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_TAMP_STAMP_CSS_LSE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_WKUP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_CH1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_CH2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_CH3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_CH4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_CH5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_CH6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_CH7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC1_2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if !defined(STM32L412) && !defined(STM32L422)
void CAN1_TX_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_RX0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_RX1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CAN1_SCE_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
void EXTI9_5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_TIM15_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_UP_TIM16_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_TRG_COM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
void TIM3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
void I2C1_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if !defined(STM32L432) && !defined(STM32L442)
void I2C2_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
void SPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if !defined(STM32L432) && !defined(STM32L442)
void SPI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
void USART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if !defined(STM32L432) && !defined(STM32L442)
void USART3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
void EXTI15_10_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_ALARM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
void SDMMC1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
void SPI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
void UART4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
void TIM6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
void TIM7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
void DMA2_CH1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_CH2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_CH3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_CH4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_CH5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
void DFSDM1_FLT0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DFSDM1_FLT1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif	
void COMP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPTIM1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPTIM2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if !defined(STM32L431) && !defined(STM32L451)
void USB_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
void DMA2_CH6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA2_CH7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void LPUART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void QUADSPI_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C3_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C3_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if !defined(STM32L412) && !defined(STM32L422)
void SAI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
void SWPMI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
void TSC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if defined(STM32L433) || defined(STM32L443)
void LCD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
#if defined(STM32L422) || defined(STM32L442) || defined(STM32L443) || defined(STM32L462)
void AES_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
void RNG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FPU_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CRS_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
void I2C4_EV_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C4_ER_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
#endif
/*----------------------------------------------------------------------------
Exception / Interrupt Vector table
*----------------------------------------------------------------------------*/

#if defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

extern const VECTOR_TABLE_Type __VECTOR_TABLE[101];
const VECTOR_TABLE_Type __VECTOR_TABLE[101] __VECTOR_TABLE_ATTRIBUTE = {
    (VECTOR_TABLE_Type)(&__INITIAL_SP),       /*     Initial Stack Pointer */
    Reset_Handler,                            /*     Reset Handler */
    NMI_Handler,                              /* -14 NMI Handler */
    HardFault_Handler,                        /* -13 Hard Fault Handler */
    MemManage_Handler,                        /* -12 MPU Fault Handler */
    BusFault_Handler,                         /* -11 Bus Fault Handler */
    UsageFault_Handler,                       /* -10 Usage Fault Handler */
    0,                                        /*     Reserved */
    0,                                        /*     Reserved */
    0,                                        /*     Reserved */
    0,                                        /*     Reserved */
    SVC_Handler,                              /*  -5 SVC Handler */
    DebugMon_Handler,                         /*  -4 Debug Monitor Handler */
    0,                                        /*     Reserved */
    PendSV_Handler,                           /*  -2 PendSV Handler */
    SysTick_Handler,                          /*  -1 SysTick Handler */
    WWDG_IRQHandler,
    PVD_PVM_IRQHandler,
    RTC_TAMP_STAMP_CSS_LSE_IRQHandler,
    RTC_WKUP_IRQHandler,
    FLASH_IRQHandler,
    RCC_IRQHandler,
    EXTI0_IRQHandler,
    EXTI1_IRQHandler,
    EXTI2_IRQHandler,
    EXTI3_IRQHandler,
    EXTI4_IRQHandler,
    DMA1_CH1_IRQHandler,
    DMA1_CH2_IRQHandler,
    DMA1_CH3_IRQHandler,
    DMA1_CH4_IRQHandler,
    DMA1_CH5_IRQHandler,
    DMA1_CH6_IRQHandler,
    DMA1_CH7_IRQHandler,
    ADC1_2_IRQHandler,
#if !defined(STM32L412) && !defined(STM32L422)
    CAN1_TX_IRQHandler,
    CAN1_RX0_IRQHandler,
    CAN1_RX1_IRQHandler,
    CAN1_SCE_IRQHandler,
#else
    0,
    0,
    0,
    0,
#endif
    EXTI9_5_IRQHandler,
    TIM1_BRK_TIM15_IRQHandler,
    TIM1_UP_TIM16_IRQHandler,
    TIM1_TRG_COM_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
    TIM3_IRQHandler,
#else
    0,
#endif        
    0,
    I2C1_EV_IRQHandler,
    I2C1_ER_IRQHandler,
#if !defined(STM32L432) && !defined(STM32L442)
    I2C2_EV_IRQHandler,
    I2C2_ER_IRQHandler,
#else
    0,
    0,
#endif    
    SPI1_IRQHandler,
#if !defined(STM32L432) && !defined(STM32L442)
    SPI2_IRQHandler,
#else
    0,
#endif
    USART1_IRQHandler,
    USART2_IRQHandler,
#if !defined(STM32L432) && !defined(STM32L442)    
    USART3_IRQHandler,
#else
    0,
#endif    
    EXTI15_10_IRQHandler,
    RTC_ALARM_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
    SDMMC1_IRQHandler,
#else
    0,
#endif    
    0,
    SPI3_IRQHandler,
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
    UART4_IRQHandler,
#else
    0,
#endif
    0,
    TIM6_IRQHandler,
#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
    TIM7_IRQHandler,
#else
    0,
#endif
    DMA2_CH1_IRQHandler,
    DMA2_CH2_IRQHandler,
    DMA2_CH3_IRQHandler,
    DMA2_CH4_IRQHandler,
    DMA2_CH5_IRQHandler,
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
    DFSDM1_FLT0_IRQHandler,
    DFSDM1_FLT1_IRQHandler,
#else
    0,
    0,
#endif        
    0,
    COMP_IRQHandler,
    LPTIM1_IRQHandler,
    LPTIM2_IRQHandler,
#if !defined(STM32L431) && !defined(STM32L451)
    USB_IRQHandler,
#else
    0,
#endif        
    DMA2_CH6_IRQHandler,
    DMA2_CH7_IRQHandler,
    LPUART1_IRQHandler,
    QUADSPI_IRQHandler,
    I2C3_EV_IRQHandler,
    I2C3_ER_IRQHandler,
#if !defined(STM32L412) && !defined(STM32L422)
    SAI1_IRQHandler,
#else
    0,
#endif        
    0,
#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
    SWPMI1_IRQHandler,
#else    
    0,
#endif    
    TSC_IRQHandler,
#if defined(STM32L433) || defined(STM32L443)
    LCD_IRQHandler,
#else    
    0,
#endif
#if defined(STM32L422) || defined(STM32L442) || defined(STM32L443) || defined(STM32L462)    
    AES_IRQHandler,
#else    
    0,
#endif
    RNG_IRQHandler,
    FPU_IRQHandler,
    CRS_IRQHandler,
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462) 
    I2C4_EV_IRQHandler,
    I2C4_ER_IRQHandler
#else
    0,
    0
#endif    
};

#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif

/*----------------------------------------------------------------------------
Reset Handler called on controller reset
*----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void)
{
    SystemInit();                             /* CMSIS System Initialization */
    __PROGRAM_START();                        /* Enter PreMain (C library entry point) */
}


#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

/*----------------------------------------------------------------------------
Hard Fault Handler
*----------------------------------------------------------------------------*/
void HardFault_Handler(void)
{

#ifdef NDEBUG
    NVIC_SystemReset();
#else
    __BKPT(0);
#endif /*NDEBUG*/

    while(1);
}

/*----------------------------------------------------------------------------
Default Handler for Exceptions / Interrupts
*----------------------------------------------------------------------------*/
void Default_Handler(void)
{
    while(1);
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic pop
#endif