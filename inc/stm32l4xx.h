#ifndef STM32L4XX_H
#define STM32L4XX_H

#ifdef __cplusplus
extern "C" {
#endif 

#ifndef __GNUC__
	#error Unsupported compiler. Please use GCC.
#endif

#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L431) && !defined(STM32L432) && !defined(STM32L433) && !defined(STM32L442) && !defined(STM32L443) && !defined(STM32L451) && !defined(STM32L452) && !defined(STM32L462)
	#error "Unsupported device."
#endif

typedef enum
{
	NMI_IRQn = -14,
	HardFault_IRQn = -13,
	MemManage_IRQn = -12,
	BusFault_IRQn = -11,
	UsageFault_IRQn = -10,
	SVC_IRQn = -5,
	Debug_Monitor_IRQn = -4,
	PendSV_IRQn = -2,
	SysTick_IRQn = -1,
	WWDG_IRQn = 0,
	PVD_PVM_IRQn = 1,
	RTC_TAMP_IRQn = 2,
	RTC_STAMP_IRQn = RTC_TAMP_IRQn,
	CSS_LSE_IRQn = RTC_TAMP_IRQn,
	RTC_WKUP_IRQn = 3,
	FLASH_IRQn = 4,
	RCC_IRQn = 5,
	EXTI0_IRQn = 6,
	EXTI1_IRQn = 7,
	EXTI2_IRQn = 8,
	EXTI3_IRQn = 9,
	EXTI4_IRQn = 10,
	DMA1_CH1_IRQn = 11,
	DMA1_CH2_IRQn = 12,
	DMA1_CH3_IRQn = 13,
	DMA1_CH4_IRQn = 14,
	DMA1_CH5_IRQn = 15,
	DMA1_CH6_IRQn = 16,
	DMA1_CH7_IRQn = 17,
	ADC1_2_IRQn = 18,
#if !defined(STM32L412) && !defined(STM32L422)
	CAN1_TX_IRQn = 19,
	CAN1_RX0_IRQn = 20,
	CAN1_RX1_IRQn = 21,
	CAN1_SCE_IRQn = 22,
#endif
	EXTI9_5_IRQn = 23,
	TIM1_BRK_IRQn = 24,	
	TIM15_IRQn = TIM1_BRK_IRQn,
	TIM1_UP_IRQn = 25,
	TIM16_IRQn = TIM1_UP_IRQn,
	TIM1_TRG_COM_IRQn = 26,
	TIM1_CC_IRQn = 27,
	TIM2_IRQn = 28,
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
	TIM3_IRQn = 29,
#endif
	I2C1_EV_IRQn = 31,
	I2C1_ER_IRQn = 32,
#if !defined(STM32L432) && !defined(STM32L442)
	I2C2_EV_IRQn = 33,
	I2C2_ER_IRQn = 34,
#endif
	SPI1_IRQn = 35,
#if !defined(STM32L432) && !defined(STM32L442)
	SPI2_IRQn = 36,
#endif
	USART1_IRQn = 37,
	USART2_IRQn = 38,
#if !defined(STM32L432) && !defined(STM32L442)
	USART3_IRQn = 39,
#endif
	EXTI15_10_IRQn = 40,
	RTC_ALARM_IRQn = 41,
#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
	SDMMC1_IRQn = 49,
#endif
	SPI3_IRQn = 51,
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
	UART4_IRQn = 52,
#endif
	TIM6_IRQn = 54,
#if !defined(STM32L412) && !defined(STM32L422)
	DACUNDER_IRQn = TIM6_IRQn,
#endif
#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
	TIM7_IRQn = 55,
#endif
	DMA2_CH1_IRQn = 56,
	DMA2_CH2_IRQn = 57,
	DMA2_CH3_IRQn = 58,
	DMA2_CH4_IRQn = 59,
	DMA2_CH5_IRQn = 60,
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
	DFSDM1_FLT0_IRQn = 61,
	DFSDM1_FLT1_IRQn = 62,
#endif
	COMP_IRQn = 64,
	LPTIM1_IRQn = 65,
	LPTIM2_IRQn = 66,
#if !defined(STM32L431) && !defined(STM32L451)
	USB_IRQn = 67,
#endif
	DMA2_CH6_IRQn = 68,
	DMA2_CH7_IRQn = 69,
	LPUART1_IRQn = 70,
	QUADSPI_IRQn = 71,
	I2C3_EV_IRQn = 72,
	I2C3_ER_IRQn = 73,
#if !defined(STM32L412) && !defined(STM32L422)
	SAI1_IRQn = 74,
#endif
#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
	SWPMI1_IRQn = 76,
#endif
	TSC_IRQn = 77,
#if defined(STM32L433) || defined(STM32L443)
	LCD_IRQn = 78,
#endif
#if defined(STM32L422) || defined(STM32L442) || defined(STM32L443) || defined(STM32L462)
	AES_IRQn = 79,
#endif
	RNG_IRQn = 80,
	FPU_IRQn = 81,
	CRS_IRQn = 82,
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
	I2C4_EV_IRQn = 83,
	I2C4_ER_IRQn = 84
#endif
}IRQn_Type;

/* ========================================================================= */
/* ============      Processor and Core Peripheral Section      ============ */
/* ========================================================================= */

/* ================ Start of section using anonymous unions ================ */
#if defined (__GNUC__)
/* anonymous unions are enabled by default */
#else
	#warning Not supported compiler type
#endif


/* ================    Configuration of Core Peripherals    ================ */
#define __CM4_REV               0x0001U  /* Core Revision r0p1 */
#define __Vendor_SysTickConfig  0U       /* Set to 1 if different SysTick Config is used */
#define __NVIC_PRIO_BITS        4U       /* Number of Bits used for Priority Levels */
#define __VTOR_PRESENT          1U       /* Set to 1 if VTOR is present */
#define __MPU_PRESENT           1U       /* Set to 1 if MPU is present */
#define __FPU_PRESENT           1U       /* Set to 1 if FPU is present */
#define __FPU_DP                0U       /* Set to 1 if FPU is double precision FPU (default is single precision FPU) */
#define __DSP_PRESENT           0U       /* Set to 1 if DSP extension are present */
#define __SAUREGION_PRESENT     0U       /* Set to 1 if SAU regions are present */
#define __PMU_PRESENT           0U       /* Set to 1 if PMU is present */
#define __PMU_NUM_EVENTCNT      0U       /* Set number of PMU Event Counters */
#define __ICACHE_PRESENT        1U       /* Set to 1 if I-Cache is present */
#define __DCACHE_PRESENT        1U       /* Set to 1 if D-Cache is present */
#define __DTCM_PRESENT          0U       /* Set to 1 if DTCM is present */

#include <stdint.h>
#include "core_cm4.h"
#include "system_stm32l4xx.h" /* System Header */

/* ========================================================================= */
/* ============       Device Specific Peripheral Section        ============ */
/* ========================================================================= */

/* ========================================================================= */
/* ============                      FLASH                      ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t ACR; /*!< FLASH access control register. */
	__OM uint32_t PDKEYR; /*!< FLASH power down key register. */
	__OM uint32_t KEYR; /*!< FLASH key register. */
	__OM uint32_t OPTKEYR; /*!< FLASH option key register. */
	__IOM uint32_t SR; /*!< FLASH status register. */
	__IOM uint32_t CR; /*!< FLASH control register. */
	__IOM uint32_t ECCR; /*!< FLASH ECC register. */
	__IOM uint32_t OPTR; /*!< FLASH option register. */
	__IOM uint32_t PCROP1SR; /*!< FLASH PCROP start address register. */
	__IOM uint32_t PCROP1ER; /*!< FLASH PCROP end address register. */
	__IOM uint32_t WRP1AR; /*!< FLASH WRP area A address register. */
	__IOM uint32_t WRP1BR; /*!< FLASH WRP area B address register. */
}STM32L4xx_FLASH_TypeDef;

/*ACR register*/
#define FLASH_ACR_SLEEP_PD_Pos 14U
#define FLASH_ACR_SLEEP_PD_Msk (0x1UL << FLASH_ACR_SLEEP_PD_Pos)
#define FLASH_ACR_SLEEP_PD FLASH_ACR_SLEEP_PD_Msk

#define FLASH_ACR_RUN_PD_Pos 13U
#define FLASH_ACR_RUN_PD_Msk (0x1UL << FLASH_ACR_RUN_PD_Pos)
#define FLASH_ACR_RUN_PD FLASH_ACR_RUN_PD_Msk

#define FLASH_ACR_DCRST_Pos 12U
#define FLASH_ACR_DCRST_Msk (0x1UL << FLASH_ACR_DCRST_Pos)
#define FLASH_ACR_DCRST FLASH_ACR_DCRST_Msk

#define FLASH_ACR_ICRST_Pos 11U
#define FLASH_ACR_ICRST_Msk (0x1UL << FLASH_ACR_ICRST_Pos)
#define FLASH_ACR_ICRST FLASH_ACR_ICRST_Msk

#define FLASH_ACR_DCEN_Pos 10U
#define FLASH_ACR_DCEN_Msk (0x1UL << FLASH_ACR_DCEN_Pos)
#define FLASH_ACR_DCEN FLASH_ACR_DCEN_Msk

#define FLASH_ACR_ICEN_Pos 9U
#define FLASH_ACR_ICEN_Msk (0x1UL << FLASH_ACR_ICEN_Pos)
#define FLASH_ACR_ICEN FLASH_ACR_ICEN_Msk

#define FLASH_ACR_PRFTEN_Pos 8U
#define FLASH_ACR_PRFTEN_Msk (0x1UL << FLASH_ACR_PRFTEN_Pos)
#define FLASH_ACR_PRFTEN FLASH_ACR_PRFTEN_Msk

#define FLASH_ACR_LATENCY_Pos 0U
#define FLASH_ACR_LATENCY_Msk (0x7UL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY FLASH_ACR_LATENCY_Msk
#define FLASH_ACR_LATENCY_WAIT_STATE_0 (0x0UL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY_WAIT_STATE_1 (0x1UL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY_WAIT_STATE_2 (0x2UL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY_WAIT_STATE_3 (0x3UL << FLASH_ACR_LATENCY_Pos)
#define FLASH_ACR_LATENCY_WAIT_STATE_4 (0x4UL << FLASH_ACR_LATENCY_Pos)

/*PDKEYR register*/
#define FLASH_PDKEYR_PDKEY1 (0x04152637UL)
#define FLASH_PDKEYR_PDKEY2 (0xFAFBFCFDUL)

/*KEYR register*/
#define FLASH_KEYR_KEY1 (0x45670123UL)
#define FLASH_KEYR_KEY2 (0xCDEF89ABUL)

/*OPTKEYR register*/
#define FLASH_OPTKEYR_KEY1 (0x08192A3BUL)
#define FLASH_OPTKEYR_KEY2 (0x4C5D6E7FUL)

/*SR register*/
#define FLASH_SR_PEMPTY_Pos 17U
#define FLASH_SR_PEMPTY_Msk (0x1UL << FLASH_SR_PEMPTY_Pos)
#define FLASH_SR_PEMPTY FLASH_SR_PEMPTY_Msk

#define FLASH_SR_BSY_Pos 16U
#define FLASH_SR_BSY_Msk (0x1UL << FLASH_SR_BSY_Pos)
#define FLASH_SR_BSY FLASH_SR_BSY_Msk

#define FLASH_SR_OPTVERR_Pos 15U
#define FLASH_SR_OPTVERR_Msk (0x1UL << FLASH_SR_OPTVERR_Pos)
#define FLASH_SR_OPTVERR FLASH_SR_OPTVERR_Msk

#define FLASH_SR_RDERR_Pos 14U
#define FLASH_SR_RDERR_Msk (0x1UL << FLASH_SR_RDERR_Pos)
#define FLASH_SR_RDERR FLASH_SR_RDERR_Msk

#define FLASH_SR_FASTERR_Pos 9U
#define FLASH_SR_FASTERR_Msk (0x1UL << FLASH_SR_FASTERR_Pos)
#define FLASH_SR_FASTERR FLASH_SR_FASTERR_Msk

#define FLASH_SR_MISERR_Pos 8U
#define FLASH_SR_MISERR_Msk (0x1UL << FLASH_SR_MISERR_Pos)
#define FLASH_SR_MISERR FLASH_SR_MISERR_Msk

#define FLASH_SR_PGSERR_Pos 7U
#define FLASH_SR_PGSERR_Msk (0x1UL << FLASH_SR_PGSERR_Pos)
#define FLASH_SR_PGSERR FLASH_SR_PGSERR_Msk

#define FLASH_SR_SIZERR_Pos 6U
#define FLASH_SR_SIZERR_Msk (0x1UL << FLASH_SR_SIZERR_Pos)
#define FLASH_SR_SIZERR FLASH_SR_SIZERR_Msk

#define FLASH_SR_PGAERR_Pos 5U
#define FLASH_SR_PGAERR_Msk (0x1UL << FLASH_SR_PGAERR_Pos)
#define FLASH_SR_PGAERR FLASH_SR_PGAERR_Msk

#define FLASH_SR_WRPERR_Pos 4U
#define FLASH_SR_WRPERR_Msk (0x1UL << FLASH_SR_WRPERR_Pos)
#define FLASH_SR_WRPERR FLASH_SR_WRPERR_Msk

#define FLASH_SR_PROGERR_Pos 3U
#define FLASH_SR_PROGERR_Msk (0x1UL << FLASH_SR_PROGERR_Pos)
#define FLASH_SR_PROGERR FLASH_SR_PROGERR_Msk

#define FLASH_SR_OPERR_Pos 1U
#define FLASH_SR_OPERR_Msk (0x1UL << FLASH_SR_OPERR_Pos)
#define FLASH_SR_OPERR FLASH_SR_OPERR_Msk

#define FLASH_SR_EOP_Pos 0U
#define FLASH_SR_EOP_Msk (0x1UL << FLASH_SR_EOP_Pos)
#define FLASH_SR_EOP FLASH_SR_EOP_Msk

/*CR register*/
#define FLASH_CR_LOCK_Pos 31U
#define FLASH_CR_LOCK_Msk (0x1UL << FLASH_CR_LOCK_Pos)
#define FLASH_CR_LOCK FLASH_CR_LOCK_Msk

#define FLASH_CR_OPTLOCK_Pos 30U
#define FLASH_CR_OPTLOCK_Msk (0x1UL << FLASH_CR_OPTLOCK_Pos)
#define FLASH_CR_OPTLOCK FLASH_CR_OPTLOCK_Msk

#define FLASH_CR_OBL_LAUNCH_Pos 27U
#define FLASH_CR_OBL_LAUNCH_Msk (0x1UL << FLASH_CR_OBL_LAUNCH_Pos)
#define FLASH_CR_OBL_LAUNCH FLASH_CR_OBL_LAUNCH_Msk

#define FLASH_CR_RDERRIE_Pos 26U
#define FLASH_CR_RDERRIE_Msk (0x1UL << FLASH_CR_RDERRIE_Pos)
#define FLASH_CR_RDERRIE FLASH_CR_RDERRIE_Msk

#define FLASH_CR_ERRIE_Pos 25U
#define FLASH_CR_ERRIE_Msk (0x1UL << FLASH_CR_ERRIE_Pos)
#define FLASH_CR_ERRIE FLASH_CR_ERRIE_Msk

#define FLASH_CR_EOPIE_Pos 24U
#define FLASH_CR_EOPIE_Msk (0x1UL << FLASH_CR_EOPIE_Pos)
#define FLASH_CR_EOPIE FLASH_CR_EOPIE_Msk

#define FLASH_CR_FSTPG_Pos 18U
#define FLASH_CR_FSTPG_Msk (0x1UL << FLASH_CR_FSTPG_Pos)
#define FLASH_CR_FSTPG FLASH_CR_FSTPG_Msk

#define FLASH_CR_OPTSTRT_Pos 17U
#define FLASH_CR_OPTSTRT_Msk (0x1UL << FLASH_CR_OPTSTRT_Pos)
#define FLASH_CR_OPTSTRT FLASH_CR_OPTSTRT_Msk

#define FLASH_CR_STRT_Pos 16U
#define FLASH_CR_STRT_Msk (0x1UL << FLASH_CR_STRT_Pos)
#define FLASH_CR_STRT FLASH_CR_STRT_Msk

#define FLASH_CR_PNB_Pos 3U
#define FLASH_CR_PNB_Msk (0xFFUL << FLASH_CR_PNB_Pos)
#define FLASH_CR_PNB FLASH_CR_PNB_Msk

#define FLASH_CR_MER1_Pos 2U
#define FLASH_CR_MER1_Msk (0x1UL << FLASH_CR_MER1_Pos)
#define FLASH_CR_MER1 FLASH_CR_MER1_Msk

#define FLASH_CR_PER_Pos 1U
#define FLASH_CR_PER_Msk (0x1UL << FLASH_CR_PER_Pos)
#define FLASH_CR_PER FLASH_CR_PER_Msk

#define FLASH_CR_PG_Pos 0U
#define FLASH_CR_PG_Msk (0x1UL << FLASH_CR_PG_Pos)
#define FLASH_CR_PG FLASH_CR_PG_Msk

/*ECCR register*/
#define FLASH_ECCR_ECCD_Pos 31U
#define FLASH_ECCR_ECCD_Msk (0x1UL << FLASH_ECCR_ECCD_Pos)
#define FLASH_ECCR_ECCD FLASH_ECCR_ECCD_Msk

#define FLASH_ECCR_ECCC_Pos 30U
#define FLASH_ECCR_ECCC_Msk (0x1UL << FLASH_ECCR_ECCC_Pos)
#define FLASH_ECCR_ECCC FLASH_ECCR_ECCC_Msk

#define FLASH_ECCR_ECCIE_Pos 24U
#define FLASH_ECCR_ECCIE_Msk  (0x1UL << FLASH_ECCR_ECCIE_Pos)
#define FLASH_ECCR_ECCIE FLASH_ECCR_ECCIE_Msk

#define FLASH_ECCR_SYSF_ECC_Pos 20U
#define FLASH_ECCR_SYSF_ECC_Msk (0x1UL << FLASH_ECCR_SYSF_ECC_Pos)
#define FLASH_ECCR_SYSF_ECC FLASH_ECCR_SYSF_ECC_Msk

#define FLASH_ECCR_ADDR_ECC_Pos 0U
#define FLASH_ECCR_ADDR_ECC_Msk  (0x7FFFFUL << FLASH_ECCR_ADDR_ECC_Pos)
#define FLASH_ECCR_ADDR_ECC FLASH_ECCR_ADDR_ECC_Msk

/*OTPR register*/
#define FLASH_OPTR_nBOOT0_Pos 27U
#define FLASH_OPTR_nBOOT0_Msk (0x1UL << FLASH_OPTR_nBOOT0_Pos)
#define FLASH_OPTR_nBOOT0 FLASH_OPTR_nBOOT0_Msk

#define FLASH_OPTR_nSWBOOT0_Pos 26U
#define FLASH_OPTR_nSWBOOT0_Msk (0x1UL << FLASH_OPTR_nSWBOOT0_Pos)
#define FLASH_OPTR_nSWBOOT0 FLASH_OPTR_nSWBOOT0_Msk

#define FLASH_OPTR_SRAM2_RST_Pos 25U
#define FLASH_OPTR_SRAM2_RST_Msk (0x1UL << FLASH_OPTR_SRAM2_RST_Pos)
#define FLASH_OPTR_SRAM2_RST FLASH_OPTR_SRAM2_RST_Msk

#define FLASH_OPTR_SRAM2_PE_Pos 24U
#define FLASH_OPTR_SRAM2_PE_Msk (0x1UL << FLASH_OPTR_SRAM2_PE_Pos)
#define FLASH_OPTR_SRAM2_PE FLASH_OPTR_SRAM2_PE_Msk

#define FLASH_OPTR_nBOOT1_Pos 23U
#define FLASH_OPTR_nBOOT1_Msk (0x1UL << FLASH_OPTR_nBOOT1_Pos)
#define FLASH_OPTR_nBOOT1 FLASH_OPTR_nBOOT1_Msk

#define FLASH_OPTR_WWDG_SW_Pos 19U
#define FLASH_OPTR_WWDG_SW_Msk (0x1UL << FLASH_OPTR_WWDG_SW_Pos)
#define FLASH_OPTR_WWDG_SW FLASH_OPTR_WWDG_SW_Msk

#define FLASH_OPTR_IWDG_STDBY_Pos 18U
#define FLASH_OPTR_IWDG_STDBY_Msk (0x1UL << FLASH_OPTR_IWDG_STDBY_Pos)
#define FLASH_OPTR_IWDG_STDBY FLASH_OPTR_IWDG_STDBY_Msk

#define FLASH_OPTR_IWDG_STOP_Pos 17U
#define FLASH_OPTR_IWDG_STOP_Msk (0x1UL << FLASH_OPTR_IWDG_STOP_Pos)
#define FLASH_OPTR_IWDG_STOP FLASH_OPTR_IWDG_STOP_Msk

#define FLASH_OPTR_IWDG_SW_Pos 16U
#define FLASH_OPTR_IWDG_SW_Msk (0x1UL << FLASH_OPTR_IWDG_SW_Pos)
#define FLASH_OPTR_IWDG_SW FLASH_OPTR_IWDG_SW_Msk

#define FLASH_OPTR_nRST_SHDW_Pos 14U
#define FLASH_OPTR_nRST_SHDW_Msk (0x1UL << FLASH_OPTR_nRST_SHDW_Pos)
#define FLASH_OPTR_nRST_SHDW FLASH_OPTR_nRST_SHDW_Msk

#define FLASH_OPTR_nRST_STDBY_Pos 13U
#define FLASH_OPTR_nRST_STDBY_Msk (0x1UL << FLASH_OPTR_nRST_STDBY_Pos)
#define FLASH_OPTR_nRST_STDBY FLASH_OPTR_nRST_STDBY_Msk

#define FLASH_OPTR_nRST_STOP_Pos 12U
#define FLASH_OPTR_nRST_STOP_Msk (0x1UL << FLASH_OPTR_nRST_STOP_Pos)
#define FLASH_OPTR_nRST_STOP FLASH_OPTR_nRST_STOP_Msk

#define FLASH_OPTR_BOR_LEV_Pos 8U
#define FLASH_OPTR_BOR_LEV_Msk (0x7UL << FLASH_OPTR_BOR_LEV_Pos)
#define FLASH_OPTR_BOR_LEV FLASH_OPTR_BOR_LEV_Msk
#define FLASH_OPTR_BOR_LEV_0 (0x0UL << FLASH_OPTR_BOR_LEV_Pos)
#define FLASH_OPTR_BOR_LEV_1 (0x1UL << FLASH_OPTR_BOR_LEV_Pos)
#define FLASH_OPTR_BOR_LEV_2 (0x2UL << FLASH_OPTR_BOR_LEV_Pos)
#define FLASH_OPTR_BOR_LEV_3 (0x3UL << FLASH_OPTR_BOR_LEV_Pos)
#define FLASH_OPTR_BOR_LEV_4 (0x4UL << FLASH_OPTR_BOR_LEV_Pos)

#define FLASH_OPTR_RDP_Pos 0U
#define FLASH_OPTR_RDP_Msk (0xFFUL << FLASH_OPTR_RDP_Pos)
#define FLASH_OPTR_RDP FLASH_OPTR_RDP_Msk
#define FLASH_OPTR_RDP_LEVEL_0 (0xAAUL << FLASH_OPTR_RDP_Pos) 
#define FLASH_OPTR_RDP_LEVEL_2 (0xCCUL << FLASH_OPTR_RDP_Pos)

/*PCROP1SR register*/
#define FLASH_PCROP1SR_STRT_Pos 0U
#define FLASH_PCROP1SR_STRT_Msk (0xFFFFUL << FLASH_PCROP1SR_STRT_Pos)
#define FLASH_PCROP1SR_STRT FLASH_PCROP1SR_STRT_Msk

/*PCROP1ER register*/
#define FLASH_PCROP1ER_RDP_Pos 31U
#define FLASH_PCROP1ER_RDP_Msk (0x1UL << FLASH_PCROP1ER_RDP_Pos)
#define FLASH_PCROP1ER_RDP FLASH_PCROP1ER_RDP_Msk

#define FLASH_PCROP1ER_END_Pos 0U
#define FLASH_PCROP1ER_END_Msk (0xFFFFUL << FLASH_PCROP1ER_END_Pos)
#define FLASH_PCROP1ER_END FLASH_PCROP1ER_END_Msk

/*WRP1AR register*/
#define FLASH_WRP1AR_END_Pos 16U
#define FLASH_WRP1AR_END_Msk (0xFFUL << FLASH_WRP1AR_END_Pos)
#define FLASH_WRP1AR_END FLASH_WRP1AR_END_Msk

#define FLASH_WRP1AR_STRT_Pos 0U
#define FLASH_WRP1AR_STRT_Msk (0xFFUL << FLASH_WRP1AR_STRT_Pos)
#define FLASH_WRP1AR_STRT FLASH_WRP1AR_STRT_Msk

/*WRP1BR register*/
#define FLASH_WRP1BR_END_Pos 16U
#define FLASH_WRP1BR_END_Msk (0xFFUL << FLASH_WRP1BR_END_Pos)
#define FLASH_WRP1BR_END FLASH_WRP1BR_END_Msk

#define FLASH_WRP1BR_STRT_Pos 0U
#define FLASH_WRP1BR_STRT_Msk (0xFFUL << FLASH_WRP1BR_STRT_Pos)
#define FLASH_WRP1BR_STRT FLASH_WRP1BR_STRT_Msk

/* ========================================================================= */
/* ============                    FIREWALL                     ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CSSA; /*!< FIREWALL Code Segment Start Address register. */
	__IOM uint32_t CSL; /*!< FIREWALL Code Segment Length register. */
	__IOM uint32_t NVDSSA; /*!< FIREWALL None Volatile Data Segment Start Address register. */
	__IOM uint32_t NVDSL; /*!< FIREWALL None Volatile Data Segment Length register. */
	__IOM uint32_t VDSSA; /*!< FIREWALL Volatile Data Segment Start Address register. */
	__IOM uint32_t VDSL; /*!< FIREWALL Volatile Data Segment Length register. */
	uint32_t RESERVED[2];
	__IOM uint32_t CR; /*!< FIREWALL Configuration register. */
}STM32L4xx_FW_TypeDef;

/*CSSA register*/
#define FW_CSSA_ADD_Pos 8U
#define FW_CSSA_ADD_Msk (0xFFFFUL << FW_CSSA_ADD_Pos)
#define FW_CSSA_ADD FW_CSSA_ADD_Msk

/*CSL register*/
#define FW_CSL_LENG_Pos 8U
#define FW_CSL_LENG_Msk (0x3FFFUL << FW_CSL_LENG_Pos)
#define FW_CSL_LENG FW_CSL_LENG_Msk

/*NVDSSA register*/
#define FW_NVDSSA_ADD_Pos 8U
#define FW_NVDSSA_ADD_Msk (0xFFFFUL << FW_NVDSSA_ADD_Pos)
#define FW_NVDSSA_ADD FW_NVDSSA_ADD_Msk

/*NVDSL register*/
#define FW_NVDSL_LENG_Pos 8U
#define FW_NVDSL_LENG_Msk (0x3FFFUL << FW_NVDSL_LENG_Pos)
#define FW_NVDSL_LENG FW_NVDSL_LENG_Msk

/*VDSSA register*/
#define FW_VDSSA_ADD_Pos 6U
#define FW_VDSSA_ADD_Msk (0x7FFUL << FW_VDSSA_ADD_Pos)
#define FW_VDSSA_ADD FW_VDSSA_ADD_Msk

/*VDSL register*/
#define FW_VDSL_LENG_Pos 6U
#define FW_VDSL_LENG_Msk (0x7FFUL << FW_VDSL_LENG_Pos)
#define FW_VDSL_LENG FW_VDSL_LENG_Msk

/*CR register*/
#define FW_CR_VDE_Pos 2U
#define FW_CR_VDE_Msk (0x1UL << FW_CR_VDE_Pos)
#define FW_CR_VDE FW_CR_VDE_Msk

#define FW_CR_VDS_Pos 1U
#define FW_CR_VDS_Msk (0x1UL << FW_CR_VDS_Pos)
#define FW_CR_VDS FW_CR_VDS_Msk 

#define FW_CR_FPA_Pos 0U
#define FW_CR_FPA_Msk (0x1UL << FW_CR_FPA_Pos)
#define FW_CR_FPA FW_CR_FPA_Msk

/* ========================================================================= */
/* ============                       PWR                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR1; /*!< Power Control Register 1. */
	__IOM uint32_t CR2; /*!< Power Control Register 2. */
	__IOM uint32_t CR3; /*!< Power Control Register 3. */
	__IOM uint32_t CR4; /*!< Power Control Register 4. */
	__IM uint32_t SR1; /*!< Power Status Register 1. */
	__IM uint32_t SR2; /*!< Power Status Register 2. */
	__OM uint32_t SCR; /*!< Power Status Clear Register. */
	uint32_t RESERVED0;
	__IOM uint32_t PUCRA; /*!< Power Pull-Up Control Register Port A. */
	__IOM uint32_t PDCRA; /*!< Power Pull-Down Control Register Port A. */
	__IOM uint32_t PUCRB; /*!< Power Pull-Down Control Register Port B. */
	__IOM uint32_t PDCRB; /*!< Power Pull-Up Control Register Port B. */
	__IOM uint32_t PUCRC; /*!< Power Pull-Up Control Register Port C. */
	__IOM uint32_t PDCRC; /*!< Power Pull-Down Control Register Port C. */
	__IOM uint32_t PUCRD; /*!< Power Pull-Up Control Register Port D. */
	__IOM uint32_t PDCRD; /*!< Power Pull-Down Control Register Port D. */
	__IOM uint32_t PUCRE; /*!< Power Pull-Up Control Register Port A. */
	__IOM uint32_t PDCRE; /*!< Power Pull-Down Control Register Port D. */
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	uint32_t RESERVED3;
	uint32_t RESERVED4;
	__IOM uint32_t PUCRH; /*!< Power Pull-Up Control Register Port H. */
	__IOM uint32_t PDCRH; /*!< Power Pull-Down Control Register Port H. */
}STM32L4xx_PWR_TypeDef;

/*CR1 register*/
#define PWR_CR1_LPR_Pos 14U
#define PWR_CR1_LPR_Msk (0x1UL << PWR_CR1_LPR_Pos)
#define PWR_CR1_LPR PWR_CR1_LPR_Msk

#define PWR_CR1_VOS_Pos 9U
#define PWR_CR1_VOS_Msk (0x3UL << PWR_CR1_VOS_Pos)
#define PWR_CR1_VOS PWR_CR1_VOS_Msk
#define PWR_CR1_VOS_RANGE_1 (0x1UL << PWR_CR1_VOS_Pos)
#define PWR_CR1_VOS_RANGE_2 (0x2UL << PWR_CR1_VOS_Pos)

#define PWR_CR1_DBP_Pos 8U
#define PWR_CR1_DBP_Msk (0x1UL << PWR_CR1_DBP_Pos)
#define PWR_CR1_DBP PWR_CR1_DBP_Msk

#define PWR_CR1_LPMS_Pos 0U
#define PWR_CR1_LPMS_Msk (0x7UL << PWR_CR1_LPMS_Pos)
#define PWR_CR1_LPMS PWR_CR1_LPMS_Msk
#define PWR_CR1_LPMS_STOP_0 (0x0UL << PWR_CR1_LPMS_Pos)
#define PWR_CR1_LPMS_STOP_1 (0x1UL << PWR_CR1_LPMS_Pos)
#define PWR_CR1_LPMS_STOP_2 (0x2UL << PWR_CR1_LPMS_Pos)
#define PWR_CR1_LPMS_STANDBY (0x3UL << PWR_CR1_LPMS_Pos)
#define PWR_CR1_LPMS_SHUTDOWN (0x4UL << PWR_CR1_LPMS_Pos)

/*CR2 register*/
#if defined(STM32L412) || defined(STM32L422) || defined(STM32L432) || defined(STM32L433) ||defined(STM32L442) || defined(STM32L443) ||defined(STM32L452) || defined(STM32L462)
#define PWR_CR2_USV_Pos 10U
#define PWR_CR2_USV_Msk (0x1UL << PWR_CR2_USV_Pos)
#define PWR_CR2_USV PWR_CR2_USV_Msk
#endif

#define PWR_CR2_PVME4_Pos 7U
#define PWR_CR2_PVME4_Msk (0x1UL << PWR_CR2_PVME4_Pos)
#define PWR_CR2_PVME4 PWR_CR2_PVME4_Msk

#define PWR_CR2_PVME3_Pos 6U
#define PWR_CR2_PVME3_Msk (0x1UL << PWR_CR2_PVME3_Pos)
#define PWR_CR2_PVME3 PWR_CR2_PVME3_Msk

#if defined(STM32L412) || defined(STM32L422) || defined(STM32L432) || defined(STM32L433) ||defined(STM32L442) || defined(STM32L443) ||defined(STM32L452) || defined(STM32L462)
#define PWR_CR2_PVME1_Pos 4U
#define PWR_CR2_PVME1_Msk (0x1UL << PWR_CR2_PVME1_Pos)
#define PWR_CR2_PVME1 PWR_CR2_PVME1_Msk
#endif

#define PWR_CR2_PLS_Pos 1U
#define PWR_CR2_PLS_Msk (0x7UL << PWR_CR2_PLS_Pos)
#define PWR_CR2_PLS PWR_CR2_PLS_Msk
#define PWR_CR2_PLS_PVD_0 (0x0UL << PWR_CR2_PLS_Pos)
#define PWR_CR2_PLS_PVD_1 (0x1UL << PWR_CR2_PLS_Pos)
#define PWR_CR2_PLS_PVD_2 (0x2UL << PWR_CR2_PLS_Pos)
#define PWR_CR2_PLS_PVD_3 (0x3UL << PWR_CR2_PLS_Pos)
#define PWR_CR2_PLS_PVD_4 (0x4UL << PWR_CR2_PLS_Pos)
#define PWR_CR2_PLS_PVD_5 (0x5UL << PWR_CR2_PLS_Pos)
#define PWR_CR2_PLS_PVD_6 (0x6UL << PWR_CR2_PLS_Pos)
#define PWR_CR2_PLS_PVD_7 (0x7UL << PWR_CR2_PLS_Pos)

#define PWR_CR2_PVDE_Pos 0U
#define PWR_CR2_PVDE_Msk (0x1UL << PWR_CR2_PVDE_Pos)
#define PWR_CR2_PVDE PWR_CR2_PVDE_Msk

/*CR3 register*/
#define PWR_CR3_EIWUL_Pos 15U
#define PWR_CR3_EIWUL_Msk (0x1UL << PWR_CR3_EIWUL_Pos)
#define PWR_CR3_EIWUL PWR_CR3_EIWUL_Msk

#if defined(STM32L412) || defined(STM32L422)
#define PWR_CR3_ENULP_Pos 11U
#define PWR_CR3_ENULP_Msk (0x1UL << PWR_CR3_ENULP_Pos)
#define PWR_CR3_ENULP PWR_CR3_ENULP_Msk
#endif

#define PWR_CR3_APC_Pos 10U
#define PWR_CR3_APC_Msk (0x1UL << PWR_CR3_APC_Pos)
#define PWR_CR3_APC PWR_CR3_APC_Msk

#define PWR_CR3_RRS_Pos 8U
#define PWR_CR3_RRS_Msk (0x1UL << PWR_CR3_RRS_Pos)
#define PWR_CR3_RRS PWR_CR3_RRS_Msk

#define PWR_CR3_EWUP5_Pos 4U
#define PWR_CR3_EWUP5_Msk (0x1UL << PWR_CR3_EWUP5_Pos)
#define PWR_CR3_EWUP5 PWR_CR3_EWUP5_Msk

#define PWR_CR3_EWUP4_Pos 3U
#define PWR_CR3_EWUP4_Msk (0x1UL << PWR_CR3_EWUP4_Pos)
#define PWR_CR3_EWUP4 PWR_CR3_EWUP4_Msk

#define PWR_CR3_EWUP3_Pos 2U
#define PWR_CR3_EWUP3_Msk (0x1UL << PWR_CR3_EWUP3_Pos)
#define PWR_CR3_EWUP3 PWR_CR3_EWUP3_Msk

#define PWR_CR3_EWUP2_Pos 1U
#define PWR_CR3_EWUP2_Msk (0x1UL << PWR_CR3_EWUP2_Pos)
#define PWR_CR3_EWUP2 PWR_CR3_EWUP2_Msk

#define PWR_CR3_EWUP1_Pos 0U
#define PWR_CR3_EWUP1_Msk (0x1UL << PWR_CR3_EWUP1_Pos)
#define PWR_CR3_EWUP1 PWR_CR3_EWUP1_Msk

/*CR4 register*/
#if defined(STM32L412) || defined(STM32L422)
#define PWR_CR4_EXT_SMPS_ON_Pos 13U
#define PWR_CR4_EXT_SMPS_ON_Msk (0x1UL << PWR_CR4_EXT_SMPS_ON_Pos)
#define PWR_CR4_EXT_SMPS_ON PWR_CR4_EXT_SMPS_ON_Msk
#endif

#define PWR_CR4_VBRS_Pos 9U
#define PWR_CR4_VBRS_Msk (0x1UL << PWR_CR4_VBRS_Pos)
#define PWR_CR4_VBRS PWR_CR4_VBRS_Msk 

#define PWR_CR4_VBE_Pos 8U
#define PWR_CR4_VBE_Msk (0x1UL << PWR_CR4_VBE_Pos)
#define PWR_CR4_VBE PWR_CR4_VBE_Msk

#define PWR_CR4_WP5_Pos 4U
#define PWR_CR4_WP5_Msk (0x1UL << PWR_CR4_WP5_Pos)
#define PWR_CR4_WP5 PWR_CR4_WP5_Msk

#define PWR_CR4_WP4_Pos 3U
#define PWR_CR4_WP4_Msk (0x1UL << PWR_CR4_WP4_Pos)
#define PWR_CR4_WP4 PWR_CR4_WP4_Msk

#define PWR_CR4_WP3_Pos 2U
#define PWR_CR4_WP3_Msk (0x1UL << PWR_CR4_WP3_Pos)
#define PWR_CR4_WP3 PWR_CR4_WP3_Msk

#define PWR_CR4_WP2_Pos 1U
#define PWR_CR4_WP2_Msk (0x1UL << PWR_CR4_WP2_Pos)
#define PWR_CR4_WP2 PWR_CR4_WP2_Msk

#define PWR_CR4_WP1_Pos 0U
#define PWR_CR4_WP1_Msk (0x1UL << PWR_CR4_WP1_Pos)
#define PWR_CR4_WP1 PWR_CR4_WP1_Msk

/*PWR_SR1 register*/
#define PWR_SR1_WUFI_Pos 15U
#define PWR_SR1_WUFI_Msk (0x1UL << PWR_SR1_WUFI_Pos)
#define PWR_SR1_WUFI PWR_SR1_WUFI_Msk

#if defined(STM32L412) || defined(STM32L422)
#define PWR_SR1_EXT_SMPS_RDY_Pos 13U
#define PWR_SR1_EXT_SMPS_RDY_Msk (0x1UL << PWR_SR1_EXT_SMPS_RDY_Pos)
#define PWR_SR1_EXT_SMPS_RDY PWR_SR1_EXT_SMPS_RDY_Msk
#endif

#define PWR_SR1_SBF_Pos 8U
#define PWR_SR1_SBF_Msk (0x1UL << PWR_SR1_SBF_Pos)
#define PWR_SR1_SBF PWR_SR1_SBF_Msk

#define PWR_SR1_WUF5_Pos 4U
#define PWR_SR1_WUF5_Msk (0x1UL << PWR_SR1_WUF5_Pos)
#define PWR_SR1_WUF5 PWR_SR1_WUF5_Msk

#define PWR_SR1_WUF4_Pos 3U
#define PWR_SR1_WUF4_Msk (0x1UL << PWR_SR1_WUF4_Pos)
#define PWR_SR1_WUF4 PWR_SR1_WUF4_Msk

#define PWR_SR1_WUF3_Pos 2U
#define PWR_SR1_WUF3_Msk (0x1UL << PWR_SR1_WUF3_Pos)
#define PWR_SR1_WUF3 PWR_SR1_WUF3_Msk

#define PWR_SR1_WUF2_Pos 1U
#define PWR_SR1_WUF2_Msk (0x1UL << PWR_SR1_WUF2_Pos)
#define PWR_SR1_WUF2 PWR_SR1_WUF2_Msk

#define PWR_SR1_WUF1_Pos 0U
#define PWR_SR1_WUF1_Msk (0x1UL << PWR_SR1_WUF1_Pos)
#define PWR_SR1_WUF1 PWR_SR1_WUF1_Msk

/*PWR_SR2 register*/
#define PWR_SR2_PVMO4_Pos 15U
#define PWR_SR2_PVMO4_Msk (0x1UL << PWR_SR2_PVMO4_Pos)
#define PWR_SR2_PVMO4 PWR_SR2_PVMO4_Msk

#define PWR_SR2_PVMO3_Pos 14U
#define PWR_SR2_PVMO3_Msk (0x1UL << PWR_SR2_PVMO3_Pos)
#define PWR_SR2_PVMO3 PWR_SR2_PVMO3_Msk

#if defined(STM32L412) || defined(STM32L422) || defined(STM32L432) || defined(STM32L433) ||defined(STM32L442) || defined(STM32L443) ||defined(STM32L452) || defined(STM32L462)
#define PWR_SR2_PVMO1_Pos 12U
#define PWR_SR2_PVMO1_Msk (0x1UL << PWR_SR2_PVMO1_Pos)
#define PWR_SR2_PVMO1 PWR_SR2_PVMO1_Msk
#endif

#define PWR_SR2_PVDO_Pos 11U
#define PWR_SR2_PVDO_Msk (0x1UL << PWR_SR2_PVDO_Pos)
#define PWR_SR2_PVDO PWR_SR2_PVDO_Msk

#define PWR_SR2_VOSF_Pos 10U
#define PWR_SR2_VOSF_Msk (0x1UL << PWR_SR2_VOSF_Pos)
#define PWR_SR2_VOSF PWR_SR2_VOSF_Msk

#define PWR_SR2_REGLPF_Pos 9U
#define PWR_SR2_REGLPF_Msk (0x1UL << PWR_SR2_REGLPF_Pos)
#define PWR_SR2_REGLPF PWR_SR2_REGLPF_Msk

#define PWR_SR2_REGLPS_Pos 8U
#define PWR_SR2_REGLPS_Msk (0x1UL << PWR_SR2_REGLPS_Pos)
#define PWR_SR2_REGLPS PWR_SR2_REGLPS_Msk

/*PWR_SCR register*/
#define PWR_SCR_CSBF_Pos 8U
#define PWR_SCR_CSBF_Msk (0x1UL << PWR_SCR_CSBF_Pos)
#define PWR_SCR_CSBF PWR_SCR_CSBF_Msk

#define PWR_SCR_CWUF5_Pos 4U
#define PWR_SCR_CWUF5_Msk (0x1UL << PWR_SCR_CWUF5_Pos)
#define PWR_SCR_CWUF5 PWR_SCR_CWUF5_Msk

#define PWR_SCR_CWUF4_Pos 3U
#define PWR_SCR_CWUF4_Msk (0x1UL << PWR_SCR_CWUF4_Pos)
#define PWR_SCR_CWUF4 PWR_SCR_CWUF4_Msk

#define PWR_SCR_CWUF3_Pos 2U
#define PWR_SCR_CWUF3_Msk (0x1UL << PWR_SCR_CWUF3_Pos)
#define PWR_SCR_CWUF3 PWR_SCR_CWUF3_Msk

#define PWR_SCR_CWUF2_Pos 1U
#define PWR_SCR_CWUF2_Msk (0x1UL << PWR_SCR_CWUF2_Pos)
#define PWR_SCR_CWUF2 PWR_SCR_CWUF2_Msk

#define PWR_SCR_CWUF1_Pos 1U
#define PWR_SCR_CWUF1_Msk (0x1UL << PWR_SCR_CWUF1_Pos)
#define PWR_SCR_CWUF1 PWR_SCR_CWUF1_Msk

/*PWR_PUCRA register*/
#define PWR_PUCRA_PU15_Pos 15U
#define PWR_PUCRA_PU15_Msk (0x1UL << PWR_PUCRA_PU15_Pos)
#define PWR_PUCRA_PU15 PWR_PUCRA_PU15_Msk

#define PWR_PUCRA_PU13_Pos 13U
#define PWR_PUCRA_PU13_Msk (0x1UL << PWR_PUCRA_PU13_Pos)
#define PWR_PUCRA_PU13 PWR_PUCRA_PU13_Msk

#define PWR_PUCRA_PU12_Pos 12U
#define PWR_PUCRA_PU12_Msk (0x1UL << PWR_PUCRA_PU12_Pos)
#define PWR_PUCRA_PU12 PWR_PUCRA_PU12_Msk

#define PWR_PUCRA_PU11_Pos 11U
#define PWR_PUCRA_PU11_Msk (0x1UL << PWR_PUCRA_PU11_Pos)
#define PWR_PUCRA_PU11 PWR_PUCRA_PU11_Msk

#define PWR_PUCRA_PU10_Pos 10U
#define PWR_PUCRA_PU10_Msk (0x1UL << PWR_PUCRA_PU10_Pos)
#define PWR_PUCRA_PU10 PWR_PUCRA_PU10_Msk

#define PWR_PUCRA_PU9_Pos 9U
#define PWR_PUCRA_PU9_Msk (0x1UL << PWR_PUCRA_PU9_Pos)
#define PWR_PUCRA_PU9 PWR_PUCRA_PU9_Msk

#define PWR_PUCRA_PU8_Pos 8U
#define PWR_PUCRA_PU8_Msk (0x1UL << PWR_PUCRA_PU8_Pos)
#define PWR_PUCRA_PU8 PWR_PUCRA_PU8_Msk

#define PWR_PUCRA_PU7_Pos 7U
#define PWR_PUCRA_PU7_Msk (0x1UL << PWR_PUCRA_PU7_Pos)
#define PWR_PUCRA_PU7 PWR_PUCRA_PU7_Msk

#define PWR_PUCRA_PU6_Pos 6U
#define PWR_PUCRA_PU6_Msk (0x1UL << PWR_PUCRA_PU6_Pos)
#define PWR_PUCRA_PU6 PWR_PUCRA_PU6_Msk

#define PWR_PUCRA_PU5_Pos 5U
#define PWR_PUCRA_PU5_Msk (0x1UL << PWR_PUCRA_PU5_Pos)
#define PWR_PUCRA_PU5 PWR_PUCRA_PU5_Msk

#define PWR_PUCRA_PU4_Pos 4U
#define PWR_PUCRA_PU4_Msk (0x1UL << PWR_PUCRA_PU4_Pos)
#define PWR_PUCRA_PU4 PWR_PUCRA_PU4_Msk

#define PWR_PUCRA_PU3_Pos 3U
#define PWR_PUCRA_PU3_Msk (0x1UL << PWR_PUCRA_PU3_Pos)
#define PWR_PUCRA_PU3 PWR_PUCRA_PU3_Msk

#define PWR_PUCRA_PU2_Pos 2U
#define PWR_PUCRA_PU2_Msk (0x1UL << PWR_PUCRA_PU2_Pos)
#define PWR_PUCRA_PU2 PWR_PUCRA_PU2_Msk

#define PWR_PUCRA_PU1_Pos 1U
#define PWR_PUCRA_PU1_Msk (0x1UL << PWR_PUCRA_PU1_Pos)
#define PWR_PUCRA_PU1 PWR_PUCRA_PU1_Msk

#define PWR_PUCRA_PU0_Pos 0U
#define PWR_PUCRA_PU0_Msk (0x1UL << PWR_PUCRA_PU0_Pos)
#define PWR_PUCRA_PU0 PWR_PUCRA_PU0_Msk

/*PWR_PDCRA register*/
#define PWR_PDCRA_PD14_Pos 14U
#define PWR_PDCRA_PD14_Msk (0x1UL << PWR_PDCRA_PD14_Pos)
#define PWR_PDCRA_PD14 PWR_PDCRA_PD14_Msk

#define PWR_PDCRA_PD12_Pos 12U
#define PWR_PDCRA_PD12_Msk (0x1UL << PWR_PDCRA_PD12_Pos)
#define PWR_PDCRA_PD12 PWR_PDCRA_PD12_Msk

#define PWR_PDCRA_PD11_Pos 11U
#define PWR_PDCRA_PD11_Msk (0x1UL << PWR_PDCRA_PD11_Pos)
#define PWR_PDCRA_PD11 PWR_PDCRA_PD11_Msk

#define PWR_PDCRA_PD10_Pos 10U
#define PWR_PDCRA_PD10_Msk (0x1UL << PWR_PDCRA_PD10_Pos)
#define PWR_PDCRA_PD10 PWR_PDCRA_PD10_Msk

#define PWR_PDCRA_PD9_Pos 9U
#define PWR_PDCRA_PD9_Msk (0x1UL << PWR_PDCRA_PD9_Pos)
#define PWR_PDCRA_PD9 PWR_PDCRA_PD9_Msk

#define PWR_PDCRA_PD8_Pos 8U
#define PWR_PDCRA_PD8_Msk (0x1UL << PWR_PDCRA_PD8_Pos)
#define PWR_PDCRA_PD8 PWR_PDCRA_PD8_Msk

#define PWR_PDCRA_PD7_Pos 7U
#define PWR_PDCRA_PD7_Msk (0x1UL << PWR_PDCRA_PD7_Pos)
#define PWR_PDCRA_PD7 PWR_PDCRA_PD7_Msk

#define PWR_PDCRA_PD6_Pos 6U
#define PWR_PDCRA_PD6_Msk (0x1UL << PWR_PDCRA_PD6_Pos)
#define PWR_PDCRA_PD6 PWR_PDCRA_PD6_Msk

#define PWR_PDCRA_PD5_Pos 5U
#define PWR_PDCRA_PD5_Msk (0x1UL << PWR_PDCRA_PD5_Pos)
#define PWR_PDCRA_PD5 PWR_PDCRA_PD5_Msk

#define PWR_PDCRA_PD4_Pos 4U
#define PWR_PDCRA_PD4_Msk (0x1UL << PWR_PDCRA_PD4_Pos)
#define PWR_PDCRA_PD4 PWR_PDCRA_PD4_Msk

#define PWR_PDCRA_PD3_Pos 3U
#define PWR_PDCRA_PD3_Msk (0x1UL << PWR_PDCRA_PD3_Pos)
#define PWR_PDCRA_PD3 PWR_PDCRA_PD3_Msk

#define PWR_PDCRA_PD2_Pos 2U
#define PWR_PDCRA_PD2_Msk (0x1UL << PWR_PDCRA_PD2_Pos)
#define PWR_PDCRA_PD2 PWR_PDCRA_PD2_Msk

#define PWR_PDCRA_PD1_Pos 1U
#define PWR_PDCRA_PD1_Msk (0x1UL << PWR_PDCRA_PD1_Pos)
#define PWR_PDCRA_PD1 PWR_PDCRA_PD1_Msk

#define PWR_PDCRA_PD0_Pos 0U
#define PWR_PDCRA_PD0_Msk (0x1UL << PWR_PDCRA_PD0_Pos)
#define PWR_PDCRA_PD0 PWR_PDCRA_PD0_Msk

/*PWR_PUCRB register*/
#define PWR_PUCRB_PU15_Pos 15U
#define PWR_PUCRB_PU15_Msk (0x1UL << PWR_PUCRB_PU15_Pos)
#define PWR_PUCRB_PU15 PWR_PUCRB_PU15_Msk

#define PWR_PUCRB_PU14_Pos 14U
#define PWR_PUCRB_PU14_Msk (0x1UL << PWR_PUCRB_PU14_Pos)
#define PWR_PUCRB_PU14 PWR_PUCRB_PU14_Msk

#define PWR_PUCRB_PU13_Pos 13U
#define PWR_PUCRB_PU13_Msk (0x1UL << PWR_PUCRB_PU13_Pos)
#define PWR_PUCRB_PU13 PWR_PUCRB_PU13_Msk

#define PWR_PUCRB_PU12_Pos 12U
#define PWR_PUCRB_PU12_Msk (0x1UL << PWR_PUCRB_PU12_Pos)
#define PWR_PUCRB_PU12 PWR_PUCRB_PU12_Msk

#define PWR_PUCRB_PU11_Pos 11U
#define PWR_PUCRB_PU11_Msk (0x1UL << PWR_PUCRB_PU11_Pos)
#define PWR_PUCRB_PU11 PWR_PUCRB_PU11_Msk

#define PWR_PUCRB_PU10_Pos 10U
#define PWR_PUCRB_PU10_Msk (0x1UL << PWR_PUCRB_PU10_Pos)
#define PWR_PUCRB_PU10 PWR_PUCRB_PU10_Msk

#define PWR_PUCRB_PU9_Pos 9U
#define PWR_PUCRB_PU9_Msk (0x1UL << PWR_PUCRB_PU9_Pos)
#define PWR_PUCRB_PU9 PWR_PUCRB_PU9_Msk

#define PWR_PUCRB_PU8_Pos 8U
#define PWR_PUCRB_PU8_Msk (0x1UL << PWR_PUCRB_PU8_Pos)
#define PWR_PUCRB_PU8 PWR_PUCRB_PU8_Msk

#define PWR_PUCRB_PU7_Pos 7U
#define PWR_PUCRB_PU7_Msk (0x1UL << PWR_PUCRB_PU7_Pos)
#define PWR_PUCRB_PU7 PWR_PUCRB_PU7_Msk

#define PWR_PUCRB_PU6_Pos 6U
#define PWR_PUCRB_PU6_Msk (0x1UL << PWR_PUCRB_PU6_Pos)
#define PWR_PUCRB_PU6 PWR_PUCRB_PU6_Msk

#define PWR_PUCRB_PU5_Pos 5U
#define PWR_PUCRB_PU5_Msk (0x1UL << PWR_PUCRB_PU5_Pos)
#define PWR_PUCRB_PU5 PWR_PUCRB_PU5_Msk

#define PWR_PUCRB_PU4_Pos 4U
#define PWR_PUCRB_PU4_Msk (0x1UL << PWR_PUCRB_PU4_Pos)
#define PWR_PUCRB_PU4 PWR_PUCRB_PU4_Msk

#define PWR_PUCRB_PU3_Pos 3U
#define PWR_PUCRB_PU3_Msk (0x1UL << PWR_PUCRB_PU3_Pos)
#define PWR_PUCRB_PU3 PWR_PUCRB_PU3_Msk

#define PWR_PUCRB_PU2_Pos 2U
#define PWR_PUCRB_PU2_Msk (0x1UL << PWR_PUCRB_PU2_Pos)
#define PWR_PUCRB_PU2 PWR_PUCRB_PU2_Msk

#define PWR_PUCRB_PU1_Pos 1U
#define PWR_PUCRB_PU1_Msk (0x1UL << PWR_PUCRB_PU1_Pos)
#define PWR_PUCRB_PU1 PWR_PUCRB_PU1_Msk

#define PWR_PUCRB_PU0_Pos 0U
#define PWR_PUCRB_PU0_Msk (0x1UL << PWR_PUCRB_PU0_Pos)
#define PWR_PUCRB_PU0 PWR_PUCRB_PU0_Msk

/*PWR_PDCRB register*/
#define PWR_PDCRB_PD15_Pos 15U
#define PWR_PDCRB_PD15_Msk (0x1UL << PWR_PDCRB_PD15_Pos)
#define PWR_PDCRB_PD15 PWR_PDCRB_PD15_Msk

#define PWR_PDCRB_PD14_Pos 14U
#define PWR_PDCRB_PD14_Msk (0x1UL << PWR_PDCRB_PD14_Pos)
#define PWR_PDCRB_PD14 PWR_PDCRB_PD14_Msk

#define PWR_PDCRB_PD13_Pos 13U
#define PWR_PDCRB_PD13_Msk (0x1UL << PWR_PDCRB_PD13_Pos)
#define PWR_PDCRB_PD13 PWR_PDCRB_PD13_Msk

#define PWR_PDCRB_PD12_Pos 12U
#define PWR_PDCRB_PD12_Msk (0x1UL << PWR_PDCRB_PD12_Pos)
#define PWR_PDCRB_PD12 PWR_PDCRB_PD12_Msk

#define PWR_PDCRB_PD11_Pos 11U
#define PWR_PDCRB_PD11_Msk (0x1UL << PWR_PDCRB_PD11_Pos)
#define PWR_PDCRB_PD11 PWR_PDCRB_PD11_Msk

#define PWR_PDCRB_PD10_Pos 10U
#define PWR_PDCRB_PD10_Msk (0x1UL << PWR_PDCRB_PD10_Pos)
#define PWR_PDCRB_PD10 PWR_PDCRB_PD10_Msk

#define PWR_PDCRB_PD9_Pos 9U
#define PWR_PDCRB_PD9_Msk (0x1UL << PWR_PDCRB_PD9_Pos)
#define PWR_PDCRB_PD9 PWR_PDCRB_PD9_Msk

#define PWR_PDCRB_PD8_Pos 8U
#define PWR_PDCRB_PD8_Msk (0x1UL << PWR_PDCRB_PD8_Pos)
#define PWR_PDCRB_PD8 PWR_PDCRB_PD8_Msk

#define PWR_PDCRB_PD7_Pos 7U
#define PWR_PDCRB_PD7_Msk (0x1UL << PWR_PDCRB_PD7_Pos)
#define PWR_PDCRB_PD7 PWR_PDCRB_PD7_Msk

#define PWR_PDCRB_PD6_Pos 6U
#define PWR_PDCRB_PD6_Msk (0x1UL << PWR_PDCRB_PD6_Pos)
#define PWR_PDCRB_PD6 PWR_PDCRB_PD6_Msk

#define PWR_PDCRB_PD5_Pos 5U
#define PWR_PDCRB_PD5_Msk (0x1UL << PWR_PDCRB_PD5_Pos)
#define PWR_PDCRB_PD5 PWR_PDCRB_PD5_Msk

#define PWR_PDCRB_PD4_Pos 4U
#define PWR_PDCRB_PD4_Msk (0x1UL << PWR_PDCRB_PD4_Pos)
#define PWR_PDCRB_PD4 PWR_PDCRB_PD4_Msk

#define PWR_PDCRB_PD3_Pos 3U
#define PWR_PDCRB_PD3_Msk (0x1UL << PWR_PDCRB_PD3_Pos)
#define PWR_PDCRB_PD3 PWR_PDCRB_PD3_Msk

#define PWR_PDCRB_PD2_Pos 2U
#define PWR_PDCRB_PD2_Msk (0x1UL << PWR_PDCRB_PD2_Pos)
#define PWR_PDCRB_PD2 PWR_PDCRB_PD2_Msk

#define PWR_PDCRB_PD1_Pos 1U
#define PWR_PDCRB_PD1_Msk (0x1UL << PWR_PDCRB_PD1_Pos)
#define PWR_PDCRB_PD1 PWR_PDCRB_PD1_Msk

#define PWR_PDCRB_PD0_Pos 0U
#define PWR_PDCRB_PD0_Msk (0x1UL << PWR_PDCRB_PD0_Pos)
#define PWR_PDCRB_PD0 PWR_PDCRB_PD0_Msk

/*PWR_PUCRC register*/
#define PWR_PUCRC_PU15_Pos 15U
#define PWR_PUCRC_PU15_Msk (0x1UL << PWR_PUCRC_PU15_Pos)
#define PWR_PUCRC_PU15 PWR_PUCRC_PU15_Msk

#define PWR_PUCRC_PU14_Pos 14U
#define PWR_PUCRC_PU14_Msk (0x1UL << PWR_PUCRC_PU14_Pos)
#define PWR_PUCRC_PU14 PWR_PUCRC_PU14_Msk

#define PWR_PUCRC_PU13_Pos 13U
#define PWR_PUCRC_PU13_Msk (0x1UL << PWR_PUCRC_PU13_Pos)
#define PWR_PUCRC_PU13 PWR_PUCRC_PU13_Msk

#define PWR_PUCRC_PU12_Pos 12U
#define PWR_PUCRC_PU12_Msk (0x1UL << PWR_PUCRC_PU12_Pos)
#define PWR_PUCRC_PU12 PWR_PUCRC_PU12_Msk

#define PWR_PUCRC_PU11_Pos 11U
#define PWR_PUCRC_PU11_Msk (0x1UL << PWR_PUCRC_PU11_Pos)
#define PWR_PUCRC_PU11 PWR_PUCRC_PU11_Msk

#define PWR_PUCRC_PU10_Pos 10U
#define PWR_PUCRC_PU10_Msk (0x1UL << PWR_PUCRC_PU10_Pos)
#define PWR_PUCRC_PU10 PWR_PUCRC_PU10_Msk

#define PWR_PUCRC_PU9_Pos 9U
#define PWR_PUCRC_PU9_Msk (0x1UL << PWR_PUCRC_PU9_Pos)
#define PWR_PUCRC_PU9 PWR_PUCRC_PU9_Msk

#define PWR_PUCRC_PU8_Pos 8U
#define PWR_PUCRC_PU8_Msk (0x1UL << PWR_PUCRC_PU8_Pos)
#define PWR_PUCRC_PU8 PWR_PUCRC_PU8_Msk

#define PWR_PUCRC_PU7_Pos 7U
#define PWR_PUCRC_PU7_Msk (0x1UL << PWR_PUCRC_PU7_Pos)
#define PWR_PUCRC_PU7 PWR_PUCRC_PU7_Msk

#define PWR_PUCRC_PU6_Pos 6U
#define PWR_PUCRC_PU6_Msk (0x1UL << PWR_PUCRC_PU6_Pos)
#define PWR_PUCRC_PU6 PWR_PUCRC_PU6_Msk

#define PWR_PUCRC_PU5_Pos 5U
#define PWR_PUCRC_PU5_Msk (0x1UL << PWR_PUCRC_PU5_Pos)
#define PWR_PUCRC_PU5 PWR_PUCRC_PU5_Msk

#define PWR_PUCRC_PU4_Pos 4U
#define PWR_PUCRC_PU4_Msk (0x1UL << PWR_PUCRC_PU4_Pos)
#define PWR_PUCRC_PU4 PWR_PUCRC_PU4_Msk

#define PWR_PUCRC_PU3_Pos 3U
#define PWR_PUCRC_PU3_Msk (0x1UL << PWR_PUCRC_PU3_Pos)
#define PWR_PUCRC_PU3 PWR_PUCRC_PU3_Msk

#define PWR_PUCRC_PU2_Pos 2U
#define PWR_PUCRC_PU2_Msk (0x1UL << PWR_PUCRC_PU2_Pos)
#define PWR_PUCRC_PU2 PWR_PUCRC_PU2_Msk

#define PWR_PUCRC_PU1_Pos 1U
#define PWR_PUCRC_PU1_Msk (0x1UL << PWR_PUCRC_PU1_Pos)
#define PWR_PUCRC_PU1 PWR_PUCRC_PU1_Msk

#define PWR_PUCRC_PU0_Pos 0U
#define PWR_PUCRC_PU0_Msk (0x1UL << PWR_PUCRC_PU0_Pos)
#define PWR_PUCRC_PU0 PWR_PUCRC_PU0_Msk

/*PWR_PDCRC register*/
#define PWR_PDCRC_PD15_Pos 15U
#define PWR_PDCRC_PD15_Msk (0x1UL << PWR_PDCRC_PD15_Pos)
#define PWR_PDCRC_PD15 PWR_PDCRC_PD15_Msk

#define PWR_PDCRC_PD14_Pos 14U
#define PWR_PDCRC_PD14_Msk (0x1UL << PWR_PDCRC_PD14_Pos)
#define PWR_PDCRC_PD14 PWR_PDCRC_PD14_Msk

#define PWR_PDCRC_PD13_Pos 13U
#define PWR_PDCRC_PD13_Msk (0x1UL << PWR_PDCRC_PD13_Pos)
#define PWR_PDCRC_PD13 PWR_PDCRC_PD13_Msk

#define PWR_PDCRC_PD12_Pos 12U
#define PWR_PDCRC_PD12_Msk (0x1UL << PWR_PDCRC_PD12_Pos)
#define PWR_PDCRC_PD12 PWR_PDCRC_PD12_Msk

#define PWR_PDCRC_PD11_Pos 11U
#define PWR_PDCRC_PD11_Msk (0x1UL << PWR_PDCRC_PD11_Pos)
#define PWR_PDCRC_PD11 PWR_PDCRC_PD11_Msk

#define PWR_PDCRC_PD10_Pos 10U
#define PWR_PDCRC_PD10_Msk (0x1UL << PWR_PDCRC_PD10_Pos)
#define PWR_PDCRC_PD10 PWR_PDCRC_PD10_Msk

#define PWR_PDCRC_PD9_Pos 9U
#define PWR_PDCRC_PD9_Msk (0x1UL << PWR_PDCRC_PD9_Pos)
#define PWR_PDCRC_PD9 PWR_PDCRC_PD9_Msk

#define PWR_PDCRC_PD8_Pos 8U
#define PWR_PDCRC_PD8_Msk (0x1UL << PWR_PDCRC_PD8_Pos)
#define PWR_PDCRC_PD8 PWR_PDCRC_PD8_Msk

#define PWR_PDCRC_PD7_Pos 7U
#define PWR_PDCRC_PD7_Msk (0x1UL << PWR_PDCRC_PD7_Pos)
#define PWR_PDCRC_PD7 PWR_PDCRC_PD7_Msk

#define PWR_PDCRC_PD6_Pos 6U
#define PWR_PDCRC_PD6_Msk (0x1UL << PWR_PDCRC_PD6_Pos)
#define PWR_PDCRC_PD6 PWR_PDCRC_PD6_Msk

#define PWR_PDCRC_PD5_Pos 5U
#define PWR_PDCRC_PD5_Msk (0x1UL << PWR_PDCRC_PD5_Pos)
#define PWR_PDCRC_PD5 PWR_PDCRC_PD5_Msk

#define PWR_PDCRC_PD4_Pos 4U
#define PWR_PDCRC_PD4_Msk (0x1UL << PWR_PDCRC_PD4_Pos)
#define PWR_PDCRC_PD4 PWR_PDCRC_PD4_Msk

#define PWR_PDCRC_PD3_Pos 3U
#define PWR_PDCRC_PD3_Msk (0x1UL << PWR_PDCRC_PD3_Pos)
#define PWR_PDCRC_PD3 PWR_PDCRC_PD3_Msk

#define PWR_PDCRC_PD2_Pos 2U
#define PWR_PDCRC_PD2_Msk (0x1UL << PWR_PDCRC_PD2_Pos)
#define PWR_PDCRC_PD2 PWR_PDCRC_PD2_Msk

#define PWR_PDCRC_PD1_Pos 1U
#define PWR_PDCRC_PD1_Msk (0x1UL << PWR_PDCRC_PD1_Pos)
#define PWR_PDCRC_PD1 PWR_PDCRC_PD1_Msk

#define PWR_PDCRC_PD0_Pos 0U
#define PWR_PDCRC_PD0_Msk (0x1UL << PWR_PDCRC_PD0_Pos)
#define PWR_PDCRC_PD0 PWR_PDCRC_PD0_Msk

/*PWR_PUCRD register*/
#define PWR_PUCRD_PU15_Pos 15U
#define PWR_PUCRD_PU15_Msk (0x1UL << PWR_PUCRD_PU15_Pos)
#define PWR_PUCRD_PU15 PWR_PUCRD_PU15_Msk

#define PWR_PUCRD_PU14_Pos 14U
#define PWR_PUCRD_PU14_Msk (0x1UL << PWR_PUCRD_PU14_Pos)
#define PWR_PUCRD_PU14 PWR_PUCRD_PU14_Msk

#define PWR_PUCRD_PU13_Pos 13U
#define PWR_PUCRD_PU13_Msk (0x1UL << PWR_PUCRD_PU13_Pos)
#define PWR_PUCRD_PU13 PWR_PUCRD_PU13_Msk

#define PWR_PUCRD_PU12_Pos 12U
#define PWR_PUCRD_PU12_Msk (0x1UL << PWR_PUCRD_PU12_Pos)
#define PWR_PUCRD_PU12 PWR_PUCRD_PU12_Msk

#define PWR_PUCRD_PU11_Pos 11U
#define PWR_PUCRD_PU11_Msk (0x1UL << PWR_PUCRD_PU11_Pos)
#define PWR_PUCRD_PU11 PWR_PUCRD_PU11_Msk

#define PWR_PUCRD_PU10_Pos 10U
#define PWR_PUCRD_PU10_Msk (0x1UL << PWR_PUCRD_PU10_Pos)
#define PWR_PUCRD_PU10 PWR_PUCRD_PU10_Msk

#define PWR_PUCRD_PU9_Pos 9U
#define PWR_PUCRD_PU9_Msk (0x1UL << PWR_PUCRD_PU9_Pos)
#define PWR_PUCRD_PU9 PWR_PUCRD_PU9_Msk

#define PWR_PUCRD_PU8_Pos 8U
#define PWR_PUCRD_PU8_Msk (0x1UL << PWR_PUCRD_PU8_Pos)
#define PWR_PUCRD_PU8 PWR_PUCRD_PU8_Msk

#define PWR_PUCRD_PU7_Pos 7U
#define PWR_PUCRD_PU7_Msk (0x1UL << PWR_PUCRD_PU7_Pos)
#define PWR_PUCRD_PU7 PWR_PUCRD_PU7_Msk

#define PWR_PUCRD_PU6_Pos 6U
#define PWR_PUCRD_PU6_Msk (0x1UL << PWR_PUCRD_PU6_Pos)
#define PWR_PUCRD_PU6 PWR_PUCRD_PU6_Msk

#define PWR_PUCRD_PU5_Pos 5U
#define PWR_PUCRD_PU5_Msk (0x1UL << PWR_PUCRD_PU5_Pos)
#define PWR_PUCRD_PU5 PWR_PUCRD_PU5_Msk

#define PWR_PUCRD_PU4_Pos 4U
#define PWR_PUCRD_PU4_Msk (0x1UL << PWR_PUCRD_PU4_Pos)
#define PWR_PUCRD_PU4 PWR_PUCRD_PU4_Msk

#define PWR_PUCRD_PU3_Pos 3U
#define PWR_PUCRD_PU3_Msk (0x1UL << PWR_PUCRD_PU3_Pos)
#define PWR_PUCRD_PU3 PWR_PUCRD_PU3_Msk

#define PWR_PUCRD_PU2_Pos 2U
#define PWR_PUCRD_PU2_Msk (0x1UL << PWR_PUCRD_PU2_Pos)
#define PWR_PUCRD_PU2 PWR_PUCRD_PU2_Msk

#define PWR_PUCRD_PU1_Pos 1U
#define PWR_PUCRD_PU1_Msk (0x1UL << PWR_PUCRD_PU1_Pos)
#define PWR_PUCRD_PU1 PWR_PUCRD_PU1_Msk

#define PWR_PUCRD_PU0_Pos 0U
#define PWR_PUCRD_PU0_Msk (0x1UL << PWR_PUCRD_PU0_Pos)
#define PWR_PUCRD_PU0 PWR_PUCRD_PU0_Msk

/*PWR_PDCRD register*/
#define PWR_PDCRD_PD15_Pos 15U
#define PWR_PDCRD_PD15_Msk (0x1UL << PWR_PDCRD_PD15_Pos)
#define PWR_PDCRD_PD15 PWR_PDCRD_PD15_Msk

#define PWR_PDCRD_PD14_Pos 14U
#define PWR_PDCRD_PD14_Msk (0x1UL << PWR_PDCRD_PD14_Pos)
#define PWR_PDCRD_PD14 PWR_PDCRD_PD14_Msk

#define PWR_PDCRD_PD13_Pos 13U
#define PWR_PDCRD_PD13_Msk (0x1UL << PWR_PDCRD_PD13_Pos)
#define PWR_PDCRD_PD13 PWR_PDCRD_PD13_Msk

#define PWR_PDCRD_PD12_Pos 12U
#define PWR_PDCRD_PD12_Msk (0x1UL << PWR_PDCRD_PD12_Pos)
#define PWR_PDCRD_PD12 PWR_PDCRD_PD12_Msk

#define PWR_PDCRD_PD11_Pos 11U
#define PWR_PDCRD_PD11_Msk (0x1UL << PWR_PDCRD_PD11_Pos)
#define PWR_PDCRD_PD11 PWR_PDCRD_PD11_Msk

#define PWR_PDCRD_PD10_Pos 10U
#define PWR_PDCRD_PD10_Msk (0x1UL << PWR_PDCRD_PD10_Pos)
#define PWR_PDCRD_PD10 PWR_PDCRD_PD10_Msk

#define PWR_PDCRD_PD9_Pos 9U
#define PWR_PDCRD_PD9_Msk (0x1UL << PWR_PDCRD_PD9_Pos)
#define PWR_PDCRD_PD9 PWR_PDCRD_PD9_Msk

#define PWR_PDCRD_PD8_Pos 8U
#define PWR_PDCRD_PD8_Msk (0x1UL << PWR_PDCRD_PD8_Pos)
#define PWR_PDCRD_PD8 PWR_PDCRD_PD8_Msk

#define PWR_PDCRD_PD7_Pos 7U
#define PWR_PDCRD_PD7_Msk (0x1UL << PWR_PDCRD_PD7_Pos)
#define PWR_PDCRD_PD7 PWR_PDCRD_PD7_Msk

#define PWR_PDCRD_PD6_Pos 6U
#define PWR_PDCRD_PD6_Msk (0x1UL << PWR_PDCRD_PD6_Pos)
#define PWR_PDCRD_PD6 PWR_PDCRD_PD6_Msk

#define PWR_PDCRD_PD5_Pos 5U
#define PWR_PDCRD_PD5_Msk (0x1UL << PWR_PDCRD_PD5_Pos)
#define PWR_PDCRD_PD5 PWR_PDCRD_PD5_Msk

#define PWR_PDCRD_PD4_Pos 4U
#define PWR_PDCRD_PD4_Msk (0x1UL << PWR_PDCRD_PD4_Pos)
#define PWR_PDCRD_PD4 PWR_PDCRD_PD4_Msk

#define PWR_PDCRD_PD3_Pos 3U
#define PWR_PDCRD_PD3_Msk (0x1UL << PWR_PDCRD_PD3_Pos)
#define PWR_PDCRD_PD3 PWR_PDCRD_PD3_Msk

#define PWR_PDCRD_PD2_Pos 2U
#define PWR_PDCRD_PD2_Msk (0x1UL << PWR_PDCRD_PD2_Pos)
#define PWR_PDCRD_PD2 PWR_PDCRD_PD2_Msk

#define PWR_PDCRD_PD1_Pos 1U
#define PWR_PDCRD_PD1_Msk (0x1UL << PWR_PDCRD_PD1_Pos)
#define PWR_PDCRD_PD1 PWR_PDCRD_PD1_Msk

#define PWR_PDCRD_PD0_Pos 0U
#define PWR_PDCRD_PD0_Msk (0x1UL << PWR_PDCRD_PD0_Pos)
#define PWR_PDCRD_PD0 PWR_PDCRD_PD0_Msk

/*PWR_PUCRE register*/
#define PWR_PUCRE_PU15_Pos 15U
#define PWR_PUCRE_PU15_Msk (0x1UL << PWR_PUCRE_PU15_Pos)
#define PWR_PUCRE_PU15 PWR_PUCRE_PU15_Msk

#define PWR_PUCRE_PU14_Pos 14U
#define PWR_PUCRE_PU14_Msk (0x1UL << PWR_PUCRE_PU14_Pos)
#define PWR_PUCRE_PU14 PWR_PUCRE_PU14_Msk

#define PWR_PUCRE_PU13_Pos 13U
#define PWR_PUCRE_PU13_Msk (0x1UL << PWR_PUCRE_PU13_Pos)
#define PWR_PUCRE_PU13 PWR_PUCRE_PU13_Msk

#define PWR_PUCRE_PU12_Pos 12U
#define PWR_PUCRE_PU12_Msk (0x1UL << PWR_PUCRE_PU12_Pos)
#define PWR_PUCRE_PU12 PWR_PUCRE_PU12_Msk

#define PWR_PUCRE_PU11_Pos 11U
#define PWR_PUCRE_PU11_Msk (0x1UL << PWR_PUCRE_PU11_Pos)
#define PWR_PUCRE_PU11 PWR_PUCRE_PU11_Msk

#define PWR_PUCRE_PU10_Pos 10U
#define PWR_PUCRE_PU10_Msk (0x1UL << PWR_PUCRE_PU10_Pos)
#define PWR_PUCRE_PU10 PWR_PUCRE_PU10_Msk

#define PWR_PUCRE_PU9_Pos 9U
#define PWR_PUCRE_PU9_Msk (0x1UL << PWR_PUCRE_PU9_Pos)
#define PWR_PUCRE_PU9 PWR_PUCRE_PU9_Msk

#define PWR_PUCRE_PU8_Pos 8U
#define PWR_PUCRE_PU8_Msk (0x1UL << PWR_PUCRE_PU8_Pos)
#define PWR_PUCRE_PU8 PWR_PUCRE_PU8_Msk

#define PWR_PUCRE_PU7_Pos 7U
#define PWR_PUCRE_PU7_Msk (0x1UL << PWR_PUCRE_PU7_Pos)
#define PWR_PUCRE_PU7 PWR_PUCRE_PU7_Msk

#define PWR_PUCRE_PU6_Pos 6U
#define PWR_PUCRE_PU6_Msk (0x1UL << PWR_PUCRE_PU6_Pos)
#define PWR_PUCRE_PU6 PWR_PUCRE_PU6_Msk

#define PWR_PUCRE_PU5_Pos 5U
#define PWR_PUCRE_PU5_Msk (0x1UL << PWR_PUCRE_PU5_Pos)
#define PWR_PUCRE_PU5 PWR_PUCRE_PU5_Msk

#define PWR_PUCRE_PU4_Pos 4U
#define PWR_PUCRE_PU4_Msk (0x1UL << PWR_PUCRE_PU4_Pos)
#define PWR_PUCRE_PU4 PWR_PUCRE_PU4_Msk

#define PWR_PUCRE_PU3_Pos 3U
#define PWR_PUCRE_PU3_Msk (0x1UL << PWR_PUCRE_PU3_Pos)
#define PWR_PUCRE_PU3 PWR_PUCRE_PU3_Msk

#define PWR_PUCRE_PU2_Pos 2U
#define PWR_PUCRE_PU2_Msk (0x1UL << PWR_PUCRE_PU2_Pos)
#define PWR_PUCRE_PU2 PWR_PUCRE_PU2_Msk

#define PWR_PUCRE_PU1_Pos 1U
#define PWR_PUCRE_PU1_Msk (0x1UL << PWR_PUCRE_PU1_Pos)
#define PWR_PUCRE_PU1 PWR_PUCRE_PU1_Msk

#define PWR_PUCRE_PU0_Pos 0U
#define PWR_PUCRE_PU0_Msk (0x1UL << PWR_PUCRE_PU0_Pos)
#define PWR_PUCRE_PU0 PWR_PUCRE_PU0_Msk

/*PWR_PDCRE register*/
#define PWR_PDCRE_PD15_Pos 15U
#define PWR_PDCRE_PD15_Msk (0x1UL << PWR_PDCRE_PD15_Pos)
#define PWR_PDCRE_PD15 PWR_PDCRE_PD15_Msk

#define PWR_PDCRE_PD14_Pos 14U
#define PWR_PDCRE_PD14_Msk (0x1UL << PWR_PDCRE_PD14_Pos)
#define PWR_PDCRE_PD14 PWR_PDCRE_PD14_Msk

#define PWR_PDCRE_PD13_Pos 13U
#define PWR_PDCRE_PD13_Msk (0x1UL << PWR_PDCRE_PD13_Pos)
#define PWR_PDCRE_PD13 PWR_PDCRE_PD13_Msk

#define PWR_PDCRE_PD12_Pos 12U
#define PWR_PDCRE_PD12_Msk (0x1UL << PWR_PDCRE_PD12_Pos)
#define PWR_PDCRE_PD12 PWR_PDCRE_PD12_Msk

#define PWR_PDCRE_PD11_Pos 11U
#define PWR_PDCRE_PD11_Msk (0x1UL << PWR_PDCRE_PD11_Pos)
#define PWR_PDCRE_PD11 PWR_PDCRE_PD11_Msk

#define PWR_PDCRE_PD10_Pos 10U
#define PWR_PDCRE_PD10_Msk (0x1UL << PWR_PDCRE_PD10_Pos)
#define PWR_PDCRE_PD10 PWR_PDCRE_PD10_Msk

#define PWR_PDCRE_PD9_Pos 9U
#define PWR_PDCRE_PD9_Msk (0x1UL << PWR_PDCRE_PD9_Pos)
#define PWR_PDCRE_PD9 PWR_PDCRE_PD9_Msk

#define PWR_PDCRE_PD8_Pos 8U
#define PWR_PDCRE_PD8_Msk (0x1UL << PWR_PDCRE_PD8_Pos)
#define PWR_PDCRE_PD8 PWR_PDCRE_PD8_Msk

#define PWR_PDCRE_PD7_Pos 7U
#define PWR_PDCRE_PD7_Msk (0x1UL << PWR_PDCRE_PD7_Pos)
#define PWR_PDCRE_PD7 PWR_PDCRE_PD7_Msk

#define PWR_PDCRE_PD6_Pos 6U
#define PWR_PDCRE_PD6_Msk (0x1UL << PWR_PDCRE_PD6_Pos)
#define PWR_PDCRE_PD6 PWR_PDCRE_PD6_Msk

#define PWR_PDCRE_PD5_Pos 5U
#define PWR_PDCRE_PD5_Msk (0x1UL << PWR_PDCRE_PD5_Pos)
#define PWR_PDCRE_PD5 PWR_PDCRE_PD5_Msk

#define PWR_PDCRE_PD4_Pos 4U
#define PWR_PDCRE_PD4_Msk (0x1UL << PWR_PDCRE_PD4_Pos)
#define PWR_PDCRE_PD4 PWR_PDCRE_PD4_Msk

#define PWR_PDCRE_PD3_Pos 3U
#define PWR_PDCRE_PD3_Msk (0x1UL << PWR_PDCRE_PD3_Pos)
#define PWR_PDCRE_PD3 PWR_PDCRE_PD3_Msk

#define PWR_PDCRE_PD2_Pos 2U
#define PWR_PDCRE_PD2_Msk (0x1UL << PWR_PDCRE_PD2_Pos)
#define PWR_PDCRE_PD2 PWR_PDCRE_PD2_Msk

#define PWR_PDCRE_PD1_Pos 1U
#define PWR_PDCRE_PD1_Msk (0x1UL << PWR_PDCRE_PD1_Pos)
#define PWR_PDCRE_PD1 PWR_PDCRE_PD1_Msk

#define PWR_PDCRE_PD0_Pos 0U
#define PWR_PDCRE_PD0_Msk (0x1UL << PWR_PDCRE_PD0_Pos)
#define PWR_PDCRE_PD0 PWR_PDCRE_PD0_Msk

/*PWR_PUCRH register*/
#define PWR_PUCRH_PU3_Pos 3U
#define PWR_PUCRH_PU3_Msk (0x1UL << PWR_PUCRH_PU3_Pos)
#define PWR_PUCRH_PU3 PWR_PUCRH_PU3_Msk

#define PWR_PUCRH_PU1_Pos 1U
#define PWR_PUCRH_PU1_Msk (0x1UL << PWR_PUCRH_PU1_Pos)
#define PWR_PUCRH_PU1 PWR_PUCRH_PU1_Msk

#define PWR_PUCRH_PU0_Pos 0U
#define PWR_PUCRH_PU0_Msk (0x1UL << PWR_PUCRH_PU0_Pos)
#define PWR_PUCRH_PU0 PWR_PUCRH_PU0_Msk

/*PWR_PDCRH register*/
#define PWR_PDCRH_PD3_Pos 3U
#define PWR_PDCRH_PD3_Msk (0x1UL << PWR_PDCRH_PD3_Pos)
#define PWR_PDCRH_PD3 PWR_PDCRH_PD3_Msk

#define PWR_PDCRH_PD1_Pos 1U
#define PWR_PDCRH_PD1_Msk (0x1UL << PWR_PDCRH_PD1_Pos)
#define PWR_PDCRH_PD1 PWR_PDCRH_PD1_Msk

#define PWR_PDCRH_PD0_Pos 0U
#define PWR_PDCRH_PD0_Msk (0x1UL << PWR_PDCRH_PD0_Pos)
#define PWR_PDCRH_PD0 PWR_PDCRH_PD0_Msk

/* ========================================================================= */
/* ============                       RCC                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR; /*!< Clock Control Register. */
	__IOM uint32_t ICSCR; /*!< Internal Clock Sources Calibration Register. */
	__IOM uint32_t CFGR; /*!< Clock Configuration Register. */
	__IOM uint32_t PLLCFGR; /*!< PLL Configuration Register. */
#if !defined(STM32L412) && !defined(STM32L422)
	__IOM uint32_t PLLSAI1CFGR; /*!< PLLSAI1 Configuration Register. */
#else
	uint32_t RESERVED0;
#endif
	uint32_t RESERVED1;
	__IOM uint32_t CIER; /*!< Clock Interrupt Enable Register. */
	__IM uint32_t CIFR; /*!< Clock Interrupt Flag Register. */
	__OM uint32_t CICR; /*!< Clock Interrupt Clear Register. */
	uint32_t RESERVED2;
	__IOM uint32_t AHB1RSTR; /*!< AHB1 Peripheral Reset Register. */
	__IOM uint32_t AHB2RSTR; /*!< AHB2 Peripheral Reset Register. */
	__IOM uint32_t AHB3RSTR; /*!< AHB3 Peripheral Reset Register. */
	uint32_t RESERVED3;
	__IOM uint32_t APB1RSTR1; /*!< APB1 Peripheral Reset Register 1. */
	__IOM uint32_t APB1RSTR2; /*!< APB1 Peripheral Reset Register 2. */
	__IOM uint32_t APB2RSTR; /*!< APB2 Peripheral Reset Register. */
	uint32_t RESERVED4;
	__IOM uint32_t AHB1ENR; /*!< AHB1 Peripheral Clock Enable Register. */
	__IOM uint32_t AHB2ENR; /*!< AHB2 Peripheral Clock Enable Register. */
	__IOM uint32_t AHB3ENR; /*!< AHB3 Peripheral Clock Enable Register. */
	uint32_t RESERVED5;
	__IOM uint32_t APB1ENR1; /*!< APB1 Peripheral Clock Enable Register 1. */
	__IOM uint32_t APB1ENR2; /*!< APB1 Peripheral Clock Enable Register 2. */
	__IOM uint32_t APB2ENR; /*!< APB2 Peripheral Clock Enable Register. */
	uint32_t RESERVED6;
	__IOM uint32_t AHB1SMENR; /*!< AHB1 Peripheral Clock Enable in Sleep and Stop Modes Register. */
	__IOM uint32_t AHB2SMENR; /*!< AHB2 Peripheral Clock Enable in Sleep and Stop Modes Register. */
	__IOM uint32_t AHB3SMENR; /*!< AHB3 Peripheral Clock Enable in Sleep and Stop Modes Register. */
	uint32_t RESERVED7;
	__IOM uint32_t APB1SMENR1; /*!< APB1 Peripheral Clock Enable in Sleep and Stop Modes Register 1. */
	__IOM uint32_t APB1SMENR2; /*!< APB1 Peripheral Clock Enable in Sleep and Stop Modes Register 2. */
	__IOM uint32_t APB2SMENR; /*!< APB2 Peripheral Clock Enable in Sleep and Stop Modes Register. */
	uint32_t RESERVED8;
	__IOM uint32_t CCIPR; /*!< Peripherals Independent Clock Configuration Register. */
	uint32_t RESERVED9;
	__IOM uint32_t BDCR; /*!< Backup Domain Control Register. */
	__IOM uint32_t CSR; /*!< Control/Status Register. */
	__IOM uint32_t CRRCR; /*!< Clock Recovery RC Register. */
	__IOM uint32_t CCIPR2; /*!< Peripherals Independent Clock Configuration Register 2. */	
}STM32L4xx_RCC_TypeDef;

/*CR register*/
#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_CR_PLLSAIRDY_Pos 27U
#define RCC_CR_PLLSAIRDY_Msk (0x1UL << RCC_CR_PLLSAIRDY_Pos)
#define RCC_CR_PLLSAIRDY RCC_CR_PLLSAIRDY_Msk
#endif

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_CR_PLLSAION_Pos 26U
#define RCC_CR_PLLSAION_Msk (0x1UL << RCC_CR_PLLSAION_Pos)
#define RCC_CR_PLLSAION RCC_CR_PLLSAION_Msk
#endif

#define RCC_CR_PLLRDY_Pos 25U
#define RCC_CR_PLLRDY_Msk (0x1UL << RCC_CR_PLLRDY_Pos)
#define RCC_CR_PLLRDY RCC_CR_PLLRDY_Msk

#define RCC_CR_PLLON_Pos 24U
#define RCC_CR_PLLON_Msk (0x1UL << RCC_CR_PLLON_Pos)
#define RCC_CR_PLLON RCC_CR_PLLON_Msk

#define RCC_CR_CSSON_Pos 19U
#define RCC_CR_CSSON_Msk (0x1UL << RCC_CR_CSSON_Pos)
#define RCC_CR_CSSON RCC_CR_CSSON_Msk

#define RCC_CR_HSEBYP_Pos 18U
#define RCC_CR_HSEBYP_Msk (0x1UL << RCC_CR_HSEBYP_Pos)
#define RCC_CR_HSEBYP RCC_CR_HSEBYP_Msk

#define RCC_CR_HSERDY_Pos 17U
#define RCC_CR_HSERDY_Msk (0x1UL << RCC_CR_HSERDY_Pos)
#define RCC_CR_HSERDY RCC_CR_HSERDY_Msk

#define RCC_CR_HSEON_Pos 16U
#define RCC_CR_HSEON_Msk (0x1UL << RCC_CR_HSEON_Pos)
#define RCC_CR_HSEON RCC_CR_HSEON_Msk

#define RCC_CR_HSIASFS_Pos 11U
#define RCC_CR_HSIASFS_Msk (0x1UL << RCC_CR_HSIASFS_Pos)
#define RCC_CR_HSIASFS RCC_CR_HSIASFS_Msk

#define RCC_CR_HSIRDY_Pos 10U
#define RCC_CR_HSIRDY_Msk (0x1UL << RCC_CR_HSIRDY_Pos)
#define RCC_CR_HSIRDY RCC_CR_HSIRDY_Msk

#define RCC_CR_HSIKERON_Pos 9U
#define RCC_CR_HSIKERON_Msk (0x1UL << RCC_CR_HSIKERON_Pos)
#define RCC_CR_HSIKERON RCC_CR_HSIKERON_Msk

#define RCC_CR_HSION_Pos 8U
#define RCC_CR_HSION_Msk (0x1UL << RCC_CR_HSION_Pos)
#define RCC_CR_HSION RCC_CR_HSION_Msk

#define RCC_CR_MSIRANGE_Pos 4U
#define RCC_CR_MSIRANGE_Msk (0xFUL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE RCC_CR_MSIRANGE_Msk
#define RCC_CR_MSIRANGE_0 (0x0UL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE_1 (0x1UL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE_2 (0x2UL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE_3 (0x3UL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE_4 (0x4UL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE_5 (0x5UL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE_6 (0x6UL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE_7 (0x7UL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE_8 (0x8UL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE_9 (0x9UL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE_10 (0xAUL << RCC_CR_MSIRANGE_Pos)
#define RCC_CR_MSIRANGE_11 (0xBUL << RCC_CR_MSIRANGE_Pos)

#define RCC_CR_MSIRGSEL_Pos 3U
#define RCC_CR_MSIRGSEL_Msk (0x1UL << RCC_CR_MSIRGSEL_Pos)
#define RCC_CR_MSIRGSEL RCC_CR_MSIRGSEL_Msk

#define RCC_CR_MSIPLLEN_Pos 2U
#define RCC_CR_MSIPLLEN_Msk (0x1UL << RCC_CR_MSIPLLEN_Pos)
#define RCC_CR_MSIPLLEN RCC_CR_MSIPLLEN_Msk

#define RCC_CR_MSIRDY_Pos 1U
#define RCC_CR_MSIRDY_Msk (0x1UL << RCC_CR_MSIRDY_Pos)
#define RCC_CR_MSIRDY RCC_CR_MSIRDY_Msk

#define RCC_CR_MSION_Pos 0U
#define RCC_CR_MSION_Msk (0x1UL << RCC_CR_MSION_Pos)
#define RCC_CR_MSION RCC_CR_MSION_Msk

/*ICSCR register*/
#define RCC_ICSCR_HSITRIM_Pos 24U
#define RCC_ICSCR_HSITRIM_Msk (0x7FUL << RCC_ICSCR_HSITRIM_Pos)
#define RCC_ICSCR_HSITRIM RCC_ICSCR_HSITRIM_Msk

#define RCC_ICSCR_HSICAL_Pos 16U
#define RCC_ICSCR_HSICAL_Msk (0xFFUL << RCC_ICSCR_HSICAL_Pos)
#define RCC_ICSCR_HSICAL RCC_ICSCR_HSICAL_Msk

#define RCC_ICSCR_MSITRIM_Pos 8U
#define RCC_ICSCR_MSITRIM_Msk (0xFFUL << RCC_ICSCR_MSITRIM_Pos) 
#define RCC_ICSCR_MSITRIM RCC_ICSCR_MSITRIM_Msk

#define RCC_ICSCR_MSICAL_Pos 0U
#define RCC_ICSCR_MSICAL_Msk (0xFFUL << RCC_ICSCR_MSICAL_Pos)
#define RCC_ICSCR_MSICAL RCC_ICSCR_MSICAL_Msk

/*CFGR register*/
#define RCC_CFGR_MCOPRE_Pos 28U
#define RCC_CFGR_MCOPRE_Msk (0x7UL << RCC_CFGR_MCOPRE_Pos)
#define RCC_CFGR_MCOPRE RCC_CFGR_MCOPRE_Msk
#define RCC_CFGR_MCOPRE_DIV_1 (0x0UL << RCC_CFGR_MCOPRE_Pos)
#define RCC_CFGR_MCOPRE_DIV_2 (0x1UL << RCC_CFGR_MCOPRE_Pos)
#define RCC_CFGR_MCOPRE_DIV_4 (0x2UL << RCC_CFGR_MCOPRE_Pos)
#define RCC_CFGR_MCOPRE_DIV_8 (0x3UL << RCC_CFGR_MCOPRE_Pos)
#define RCC_CFGR_MCOPRE_DIV_16 (0x4UL << RCC_CFGR_MCOPRE_Pos)

#define RCC_CFGR_MCOSEL_Pos 24U
#define RCC_CFGR_MCOSEL_Msk (0xFUL << RCC_CFGR_MCOSEL_Pos)
#define RCC_CFGR_MCOSEL RCC_CFGR_MCOSEL_Msk
#define RCC_CFGR_MCOSEL_DISABLED (0x0UL << RCC_CFGR_MCOSEL_Pos)
#define RCC_CFGR_MCOSEL_SYSCLK (0x1UL << RCC_CFGR_MCOSEL_Pos)
#define RCC_CFGR_MCOSEL_MSI (0x2UL << RCC_CFGR_MCOSEL_Pos)
#define RCC_CFGR_MCOSEL_HSI16 (0x3UL << RCC_CFGR_MCOSEL_Pos)
#define RCC_CFGR_MCOSEL_HSE (0x4UL << RCC_CFGR_MCOSEL_Pos)
#define RCC_CFGR_MCOSEL_MAIN_PLL (0x5UL << RCC_CFGR_MCOSEL_Pos)
#define RCC_CFGR_MCOSEL_LSI (0x6UL << RCC_CFGR_MCOSEL_Pos)
#define RCC_CFGR_MCOSEL_LSE (0x7UL << RCC_CFGR_MCOSEL_Pos)
#define RCC_CFGR_MCOSEL_HSI48 (0x8UL << RCC_CFGR_MCOSEL_Pos)

#define RCC_CFGR_STOPWUCK_Pos 15U
#define RCC_CFGR_STOPWUCK_Msk (0x1UL << RCC_CFGR_STOPWUCK_Pos)
#define RCC_CFGR_STOPWUCK RCC_CFGR_STOPWUCK_Msk

#define RCC_CFGR_PPRE2_Pos 11U
#define RCC_CFGR_PPRE2_Msk (0x7UL << RCC_CFGR_PPRE2_Pos)
#define RCC_CFGR_PPRE2 RCC_CFGR_PPRE2_Msk
#define RCC_CFGR_PPRE2_DIV_1 (0x0UL << RCC_CFGR_PPRE2_Pos)
#define RCC_CFGR_PPRE2_DIV_2 (0x4UL << RCC_CFGR_PPRE2_Pos)
#define RCC_CFGR_PPRE2_DIV_4 (0x5UL << RCC_CFGR_PPRE2_Pos)
#define RCC_CFGR_PPRE2_DIV_8 (0x6UL << RCC_CFGR_PPRE2_Pos)
#define RCC_CFGR_PPRE2_DIV_16 (0x7UL << RCC_CFGR_PPRE2_Pos)

#define RCC_CFGR_PPRE1_Pos 8U
#define RCC_CFGR_PPRE1_Msk (0x7UL << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE1 RCC_CFGR_PPRE1_Msk
#define RCC_CFGR_PPRE1_DIV_1 (0x0UL << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE1_DIV_2 (0x4UL << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE1_DIV_4 (0x5UL << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE1_DIV_8 (0x6UL << RCC_CFGR_PPRE1_Pos)
#define RCC_CFGR_PPRE1_DIV_16 (0x7UL << RCC_CFGR_PPRE1_Pos)

#define RCC_CFGR_HPRE_Pos 4U
#define RCC_CFGR_HPRE_Msk (0xFUL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE RCC_CFGR_HPRE_Msk
#define RCC_CFGR_HPRE_DIV_1 (0x0UL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_DIV_2 (0x8UL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_DIV_4 (0x9UL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_DIV_8 (0xAUL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_DIV_16 (0xBUL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_DIV_64 (0xCUL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_DIV_128 (0xDUL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_DIV_256 (0xEUL << RCC_CFGR_HPRE_Pos)
#define RCC_CFGR_HPRE_DIV_512 (0xFUL << RCC_CFGR_HPRE_Pos)

#define RCC_CFGR_SWS_Pos 2U
#define RCC_CFGR_SWS_Msk (0x3UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS RCC_CFGR_SWS_Msk
#define RCC_CFGR_SWS_MSI (0x0UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS_HSI16 (0x1UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS_HSE (0x2UL << RCC_CFGR_SWS_Pos)
#define RCC_CFGR_SWS_PLL (0x3UL << RCC_CFGR_SWS_Pos)

#define RCC_CFGR_SW_Pos 0U
#define RCC_CFGR_SW_Msk (0x3UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW RCC_CFGR_SW_Msk
#define RCC_CFGR_SW_MSI (0x0UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_HSI16 (0x1UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_HSE (0x2UL << RCC_CFGR_SW_Pos)
#define RCC_CFGR_SW_PLL (0x3UL << RCC_CFGR_SW_Pos)

/*PLLCGFR register*/
#define RCC_PLLCFGR_PLLPDIV_Pos 27U
#define RCC_PLLCFGR_PLLPDIV_Msk (0x1FUL << RCC_PLLCFGR_PLLPDIV_Pos)
#define RCC_PLLCFGR_PLLPDIV RCC_PLLCFGR_PLLPDIV_Msk

#define RCC_PLLCFGR_PLLR_Pos 25U
#define RCC_PLLCFGR_PLLR_Msk (0x3UL << RCC_PLLCFGR_PLLR_Pos)
#define RCC_PLLCFGR_PLLR RCC_PLLCFGR_PLLR_Msk
#define RCC_PLLCFGR_PLLR_DIV_2 (0x0UL << RCC_PLLCFGR_PLLR_Pos)
#define RCC_PLLCFGR_PLLR_DIV_4 (0x1UL << RCC_PLLCFGR_PLLR_Pos)
#define RCC_PLLCFGR_PLLR_DIV_6 (0x2UL << RCC_PLLCFGR_PLLR_Pos)
#define RCC_PLLCFGR_PLLR_DIV_8 (0x3UL << RCC_PLLCFGR_PLLR_Pos)

#define RCC_PLLCFGR_PLLREN_Pos 24U
#define RCC_PLLCFGR_PLLREN_Msk (0x1UL << RCC_PLLCFGR_PLLREN_Pos)
#define RCC_PLLCFGR_PLLREN RCC_PLLCFGR_PLLREN_Msk

#define RCC_PLLCFGR_PLLQ_Pos 21U
#define RCC_PLLCFGR_PLLQ_Msk (0x3UL << RCC_PLLCFGR_PLLQ_Pos)
#define RCC_PLLCFGR_PLLQ RCC_PLLCFGR_PLLQ_Msk
#define RCC_PLLCFGR_PLLQ_DIV_2 (0x0UL << RCC_PLLCFGR_PLLQ_Pos)
#define RCC_PLLCFGR_PLLQ_DIV_4 (0x1UL << RCC_PLLCFGR_PLLQ_Pos)
#define RCC_PLLCFGR_PLLQ_DIV_6 (0x2UL << RCC_PLLCFGR_PLLQ_Pos)
#define RCC_PLLCFGR_PLLQ_DIV_8 (0x3UL << RCC_PLLCFGR_PLLQ_Pos)

#define RCC_PLLCFGR_PLLQEN_Pos 20U
#define RCC_PLLCFGR_PLLQEN_Msk (0x1UL << RCC_PLLCFGR_PLLQEN_Pos)
#define RCC_PLLCFGR_PLLQEN RCC_PLLCFGR_PLLQEN_Msk

#define RCC_PLLCFGR_PLLP_Pos 17U
#define RCC_PLLCFGR_PLLP_Msk (0x1UL << RCC_PLLCFGR_PLLP_Pos)
#define RCC_PLLCFGR_PLLP RCC_PLLCFGR_PLLP_Msk 

#define RCC_PLLCFGR_PLLPEN_Pos 16U
#define RCC_PLLCFGR_PLLPEN_Msk (0x1UL << RCC_PLLCFGR_PLLPEN_Pos)
#define RCC_PLLCFGR_PLLPEN RCC_PLLCFGR_PLLPEN_Msk

#define RCC_PLLCFGR_PLLN_Pos 8U
#define RCC_PLLCFGR_PLLN_Msk (0x7FUL << RCC_PLLCFGR_PLLN_Pos)
#define RCC_PLLCFGR_PLLN RCC_PLLCFGR_PLLN_Msk

#define RCC_PLLCFGR_PLLM_Pos 4U
#define RCC_PLLCFGR_PLLM_Msk (0x7UL << RCC_PLLCFGR_PLLM_Pos)
#define RCC_PLLCFGR_PLLM RCC_PLLCFGR_PLLM_Msk
#define RCC_PLLCFGR_PLLM_DIV_1 (0x0UL << RCC_PLLCFGR_PLLM_Pos)
#define RCC_PLLCFGR_PLLM_DIV_2 (0x1UL << RCC_PLLCFGR_PLLM_Pos)
#define RCC_PLLCFGR_PLLM_DIV_3 (0x2UL << RCC_PLLCFGR_PLLM_Pos)
#define RCC_PLLCFGR_PLLM_DIV_4 (0x3UL << RCC_PLLCFGR_PLLM_Pos)
#define RCC_PLLCFGR_PLLM_DIV_5 (0x4UL << RCC_PLLCFGR_PLLM_Pos)
#define RCC_PLLCFGR_PLLM_DIV_6 (0x5UL << RCC_PLLCFGR_PLLM_Pos)
#define RCC_PLLCFGR_PLLM_DIV_7 (0x6UL << RCC_PLLCFGR_PLLM_Pos)
#define RCC_PLLCFGR_PLLM_DIV_8 (0x7UL << RCC_PLLCFGR_PLLM_Pos)

#define RCC_PLLCFGR_PLLSRC_Pos 0U
#define RCC_PLLCFGR_PLLSRC_Msk (0x3UL << RCC_PLLCFGR_PLLSRC_Pos)
#define RCC_PLLCFGR_PLLSRC RCC_PLLCFGR_PLLSRC_Msk
#define RCC_PLLCFGR_PLLSRC_MSI (0x1UL << RCC_PLLCFGR_PLLSRC_Pos)
#define RCC_PLLCFGR_PLLSRC_HSI16 (0x2UL << RCC_PLLCFGR_PLLSRC_Pos)
#define RCC_PLLCFGR_PLLSRC_HSE (0x3UL << RCC_PLLCFGR_PLLSRC_Pos)

/*PLLSAI1 register*/
#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_PLLSAI1CFGR_PLLSAI1PDIV_Pos 27U
#define RCC_PLLSAI1CFGR_PLLSAI1PDIV_Msk (0x1FUL << RCC_PLLSAI1CFGR_PLLSAI1PDIV_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1PDIV RCC_PLLSAI1CFGR_PLLSAI1PDIV_Msk

#define RCC_PLLSAI1CFGR_PLLSAI1R_Pos 25U
#define RCC_PLLSAI1CFGR_PLLSAI1R_Msk (0x3UL << RCC_PLLSAI1CFGR_PLLSAI1R_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1R RCC_PLLSAI1CFGR_PLLSAI1R_Msk
#define RCC_PLLSAI1CFGR_PLLSAI1R_DIV_2 (0x0UL << RCC_PLLSAI1CFGR_PLLSAI1R_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1R_DIV_4 (0x1UL << RCC_PLLSAI1CFGR_PLLSAI1R_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1R_DIV_6 (0x2UL << RCC_PLLSAI1CFGR_PLLSAI1R_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1R_DIV_8 (0x3UL << RCC_PLLSAI1CFGR_PLLSAI1R_Pos)

#define RCC_PLLSAI1CFGR_PLLSAI1REN_Pos 24U
#define RCC_PLLSAI1CFGR_PLLSAI1REN_Msk (0x1UL << RCC_PLLSAI1CFGR_PLLSAI1REN_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1REN RCC_PLLSAI1CFGR_PLLSAI1REN_Msk

#define RCC_PLLSAI1CFGR_PLLSAI1Q_Pos 21U
#define RCC_PLLSAI1CFGR_PLLSAI1Q_Msk (0x3UL << RCC_PLLSAI1CFGR_PLLSAI1Q_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1Q RCC_PLLSAI1CFGR_PLLSAI1Q_Msk
#define RCC_PLLSAI1CFGR_PLLSAI1Q_DIV_2 (0x0UL << RCC_PLLSAI1CFGR_PLLSAI1Q_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1Q_DIV_4 (0x1UL << RCC_PLLSAI1CFGR_PLLSAI1Q_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1Q_DIV_6 (0x2UL << RCC_PLLSAI1CFGR_PLLSAI1Q_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1Q_DIV_8 (0x3UL << RCC_PLLSAI1CFGR_PLLSAI1Q_Pos)

#define RCC_PLLSAI1CFGR_PLLSAI1QEN_Pos 20U
#define RCC_PLLSAI1CFGR_PLLSAI1QEN_Msk (0x1UL << RCC_PLLSAI1CFGR_PLLSAI1QEN_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1QEN RCC_PLLSAI1CFGR_PLLSAI1QEN_Msk

#define RCC_PLLSAI1CFGR_PLLSAI1P_Pos 17U
#define RCC_PLLSAI1CFGR_PLLSAI1P_Msk (0x1UL << RCC_PLLSAI1CFGR_PLLSAI1P_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1P RCC_PLLSAI1CFGR_PLLSAI1P_Msk

#define RCC_PLLSAI1CFGR_PLLSAI1PEN_Pos 16U
#define RCC_PLLSAI1CFGR_PLLSAI1PEN_Msk (0x1UL << RCC_PLLSAI1CFGR_PLLSAI1PEN_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1PEN RCC_PLLSAI1CFGR_PLLSAI1PEN_Msk

#define RCC_PLLSAI1CFGR_PLLSAI1N_Pos 8U
#define RCC_PLLSAI1CFGR_PLLSAI1N_Msk (0x7FUL << RCC_PLLSAI1CFGR_PLLSAI1N_Pos)
#define RCC_PLLSAI1CFGR_PLLSAI1N RCC_PLLSAI1CFGR_PLLSAI1N_Msk
#endif

/*CIER register*/
#define RCC_CIER_HSI48RDYIE_Pos 10U
#define RCC_CIER_HSI48RDYIE_Msk (0x1UL << RCC_CIER_HSI48RDYIE_Pos)
#define RCC_CIER_HSI48RDYIE RCC_CIER_HSI48RDYIE_Msk

#define RCC_CIER_LSECSSIE_Pos 9U
#define RCC_CIER_LSECSSIE_Msk (0x1UL << RCC_CIER_LSECSSIE_Pos)
#define RCC_CIER_LSECSSIE RCC_CIER_LSECSSIE_Msk

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_CIER_PLLSAI1RDYIE_Pos 6U
#define RCC_CIER_PLLSAI1RDYIE_Msk (0x1UL << RCC_CIER_PLLSAI1RDYIE_Pos)
#define RCC_CIER_PLLSAI1RDYIE RCC_CIER_PLLSAI1RDYIE_Msk
#endif

#define RCC_CIER_PLLRDYIE_Pos 5U
#define RCC_CIER_PLLRDYIE_Msk (0x1UL << RCC_CIER_PLLRDYIE_Pos)
#define RCC_CIER_PLLRDYIE RCC_CIER_PLLRDYIE_Msk

#define RCC_CIER_HSERDYIE_Pos 4U
#define RCC_CIER_HSERDYIE_Msk (0x1UL << RCC_CIER_HSERDYIE_Pos)
#define RCC_CIER_HSERDYIE RCC_CIER_HSERDYIE_Msk

#define RCC_CIER_HSIRDYIE_Pos 3U
#define RCC_CIER_HSIRDYIE_Msk (0x1UL << RCC_CIER_HSIRDYIE_Pos)
#define RCC_CIER_HSIRDYIE RCC_CIER_HSIRDYIE_Msk

#define RCC_CIER_MSIRDYIE_Pos 2U
#define RCC_CIER_MSIRDYIE_Msk (0x1UL << RCC_CIER_MSIRDYIE_Pos)
#define RCC_CIER_MSIRDYIE RCC_CIER_MSIRDYIE_Msk

#define RCC_CIER_LSERDYIE_Pos 1U
#define RCC_CIER_LSERDYIE_Msk (0x1UL << RCC_CIER_LSERDYIE_Pos)
#define RCC_CIER_LSERDYIE RCC_CIER_LSERDYIE_Msk

#define RCC_CIER_LSIRDYIE_Pos 0U
#define RCC_CIER_LSIRDYIE_Msk (0x1UL << RCC_CIER_LSIRDYIE_Pos)
#define RCC_CIER_LSIRDYIE RCC_CIER_LSIRDYIE_Msk

/*CIFR register*/
#define RCC_CIFR_HSI48RDYF_Pos 10U
#define RCC_CIFR_HSI48RDYF_Msk (0x1UL << RCC_CIFR_HSI48RDYF_Pos)
#define RCC_CIFR_HSI48RDYF RCC_CIFR_HSI48RDYF_Msk

#define RCC_CIFR_LSECSSF_Pos 9U
#define RCC_CIFR_LSECSSF_Msk (0x1UL << RCC_CIFR_LSECSSF_Pos)
#define RCC_CIFR_LSECSSF RCC_CIFR_LSECSSF_Msk

#define RCC_CIFR_CSSF_Pos 8U
#define RCC_CIFR_CSSF_Msk (0x1UL << RCC_CIFR_CSSF_Pos)
#define RCC_CIFR_CSSF RCC_CIFR_CSSF_Msk

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_CIFR_PLLSAI1RDYF_Pos 6U
#define RCC_CIFR_PLLSA1RDYF_Msk (0x1UL << RCC_CIFR_PLLSAI1RDYF_Pos)
#define RCC_CIFR_PLLSA1RDYF RCC_CIFR_PLLSA1RDYF_Msk
#endif

#define RCC_CIFR_PLLRDYF_Pos 5U
#define RCC_CIFR_PLLRDYF_Msk (0x1UL << RCC_CIFR_PLLRDYF_Pos)
#define RCC_CIFR_PLLRDYF RCC_CIFR_PLLRDYF_Msk

#define RCC_CIFR_HSERDYF_Pos 4U
#define RCC_CIFR_HSERDYF_Msk (0x1UL << RCC_CIFR_HSERDYF_Pos)
#define RCC_CIFR_HSERDYF RCC_CIFR_HSERDYF_Msk

#define RCC_CIFR_HSIRDYF_Pos 3U
#define RCC_CIFR_HSIRDYF_Msk (0x1UL << RCC_CIFR_HSIRDYF_Pos)
#define RCC_CIFR_HSIRDYF RCC_CIFR_HSIRDYF_Msk

#define RCC_CIFR_MSIRDYF_Pos 2U
#define RCC_CIFR_MSIRDYF_Msk (0x1UL << RCC_CIFR_MSIRDYF_Pos)
#define RCC_CIFR_MSIRDYF RCC_CIFR_MSIRDYF_Msk

#define RCC_CIFR_LSERDYF_Pos 1U
#define RCC_CIFR_LSERDYF_Msk (0x1UL << RCC_CIFR_LSERDYF_Pos)
#define RCC_CIFR_LSERDYF RCC_CIFR_LSERDYF_Msk

#define RCC_CIFR_LSIRDYF_Pos 0U
#define RCC_CIFR_LSIRDYF_Msk (0x1UL << RCC_CIFR_LSIRDYF_Pos)
#define RCC_CIFR_LSIRDYF RCC_CIFR_LSIRDYF_Msk

/*CICR register*/
#define RCC_CICR_HSI48RDYC_Pos 10U
#define RCC_CICR_HSI48RDYC_Msk (0x1UL << RCC_CICR_HSI48RDYC_Pos)
#define RCC_CICR_HSI48RDYC RCC_CICR_HSI48RDYC_Msk

#define RCC_CICR_LSECSSC_Pos 9U
#define RCC_CICR_LSECSSC_Msk (0x1UL << RCC_CICR_LSECSSC_Pos)
#define RCC_CICR_LSECSSC RCC_CICR_LSECSSC_Msk

#define RCC_CICR_CSSC_Pos 8U
#define RCC_CICR_CSSC_Msk (0x1UL << RCC_CICR_CSSC_Pos)
#define RCC_CICR_CSSC RCC_CICR_CSSC_Msk

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_CICR_PLLSAI1RDYC_Pos 6U
#define RCC_CICR_PLLSAI1RDYC_Msk (0x1UL << RCC_CICR_PLLSAI1RDYC_Pos)
#define RCC_CICR_PLLSAI1RDYC RCC_CICR_PLLSAI1RDYC_Msk
#endif

#define RCC_CICR_PLLRDYC_Pos 5U
#define RCC_CICR_PLLRDYC_Msk (0x1UL << RCC_CICR_PLLRDYC_Pos)
#define RCC_CICR_PLLRDYC RCC_CICR_PLLRDYC_Msk

#define RCC_CICR_HSERDYC_Pos 4U
#define RCC_CICR_HSERDYC_Msk (0x1UL << RCC_CICR_HSERDYC_Pos)
#define RCC_CICR_HSERDYC RCC_CICR_HSERDYC_Msk

#define RCC_CICR_HSIRDYC_Pos 3U
#define RCC_CICR_HSIRDYC_Msk (0x1UL << RCC_CICR_HSIRDYC_Pos)
#define RCC_CICR_HSIRDYC RCC_CICR_HSIRDYC_Msk

#define RCC_CICR_MSIRDYC_Pos 2U
#define RCC_CICR_MSIRDYC_Msk (0x1UL << RCC_CICR_MSIRDYC_Pos)
#define RCC_CICR_MSIRDYC RCC_CICR_MSIRDYC_Msk

#define RCC_CICR_LSERDYC_Pos 1U
#define RCC_CICR_LSERDYC_Msk (0x1UL << RCC_CICR_LSERDYC_Pos)
#define RCC_CICR_LSERDYC RCC_CICR_LSERDYC_Msk

#define RCC_CICR_LSIRDYC_Pos 0U
#define RCC_CICR_LSIRDYC_Msk (0x1UL << RCC_CICR_LSIRDYC_Pos)
#define RCC_CICR_LSIRDYC RCC_CICR_LSIRDYC_Msk

/*AHB1RSTR register*/
#define RCC_AHB1RSTR_TSCRST_Pos 16U
#define RCC_AHB1RSTR_TSCRST_Msk (0x1UL << RCC_AHB1RSTR_TSCRST_Pos)
#define RCC_AHB1RSTR_TSCRST RCC_AHB1RSTR_TSCRST_Msk

#define RCC_AHB1RSTR_CRCRST_Pos 12U
#define RCC_AHB1RSTR_CRCRST_Msk (0x1UL << RCC_AHB1RSTR_CRCRST_Pos)
#define RCC_AHB1RSTR_CRCRST RCC_AHB1RSTR_CRCRST_Msk

#define RCC_AHB1RSTR_FLASHRST_Pos 8U
#define RCC_AHB1RSTR_FLASHRST_Msk (0x1UL << RCC_AHB1RSTR_FLASHRST_Pos)
#define RCC_AHB1RSTR_FLASHRST RCC_AHB1RSTR_FLASHRST_Msk

#define RCC_AHB1RSTR_DMA2RST_Pos 1U
#define RCC_AHB1RSTR_DMA2RST_Msk (0x1UL << RCC_AHB1RSTR_DMA2RST_Pos)
#define RCC_AHB1RSTR_DMA2RST RCC_AHB1RSTR_DMA2RST_Msk

#define RCC_AHB1RSTR_DMA1RST_Pos 0U
#define RCC_AHB1RSTR_DMA1RST_Msk (0x1UL << RCC_AHB1RSTR_DMA1RST_Pos)
#define RCC_AHB1RSTR_DMA1RST RCC_AHB1RSTR_DMA1RST_Msk

/*AHB2RSTR register*/
#define RCC_AHB2RSTR_RNGRST_Pos 18U
#define RCC_AHB2RSTR_RNGRST_Msk (0x1UL << RCC_AHB2RSTR_RNGRST_Pos)
#define RCC_AHB2RSTR_RNGRST RCC_AHB2RSTR_RNGRST_Msk

#if defined(STM32L422) || defined(STM32L442) || defined(STM32L443) || defined(STM32L462)
#define RCC_AHB2RSTR_AESRST_Pos 16U
#define RCC_AHB2RSTR_AESRST_Msk (0x1UL << RCC_AHB2RSTR_AESRST_Pos)
#define RCC_AHB2RSTR_AESRST RCC_AHB2RSTR_AESRST_Msk
#endif

#define RCC_AHB2RSTR_ADCRST_Pos 13U
#define RCC_AHB2RSTR_ADCRST_Msk (0x1UL << RCC_AHB2RSTR_ADCRST_Pos)
#define RCC_AHB2RSTR_ADCRST RCC_AHB2RSTR_ADCRST_Msk 

#define RCC_AHB2RSTR_GPIOHRST_Pos 7U
#define RCC_AHB2RSTR_GPIOHRST_Msk (0x1UL << RCC_AHB2RSTR_GPIOHRST_Pos)
#define RCC_AHB2RSTR_GPIOHRST RCC_AHB2RSTR_GPIOHRST_Msk

#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
#define RCC_AHB2RSTR_GPIOERST_Pos 4U
#define RCC_AHB2RSTR_GPIOERST_Msk (0x1UL << RCC_AHB2RSTR_GPIOERST_Pos)
#define RCC_AHB2RSTR_GPIOERST RCC_AHB2RSTR_GPIOERST_Msk
#endif

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_AHB2RSTR_GPIODRST_Pos 3U
#define RCC_AHB2RSTR_GPIODRST_Msk (0x1UL << RCC_AHB2RSTR_GPIODRST_Pos)
#define RCC_AHB2RSTR_GPIODRST RCC_AHB2RSTR_GPIODRST_Msk
#endif

#define RCC_AHB2RSTR_GPIOCRST_Pos 2U
#define RCC_AHB2RSTR_GPIOCRST_Msk (0x1UL << RCC_AHB2RSTR_GPIOCRST_Pos)
#define RCC_AHB2RSTR_GPIOCRST RCC_AHB2RSTR_GPIOCRST_Msk

#define RCC_AHB2RSTR_GPIOBRST_Pos 1U
#define RCC_AHB2RSTR_GPIOBRST_Msk (0x1UL << RCC_AHB2RSTR_GPIOBRST_Pos)
#define RCC_AHB2RSTR_GPIOBRST RCC_AHB2RSTR_GPIOBRST_Msk

#define RCC_AHB2RSTR_GPIOARST_Pos 0U
#define RCC_AHB2RSTR_GPIOARST_Msk (0x1UL << RCC_AHB2RSTR_GPIOARST_Pos)
#define RCC_AHB2RSTR_GPIOARST RCC_AHB2RSTR_GPIOARST_Msk

/*AHB3RSTR register*/
#define RCC_AHB3RSTR_QSPIRST_Pos 8U
#define RCC_AHB3RSTR_QSPIRST_Msk (0x1UL << RCC_AHB3RSTR_QSPIRST_Pos)
#define RCC_AHB3RSTR_QSPIRST RCC_AHB3RSTR_QSPIRST_Msk

/*APB1RSTR1 register*/
#define RCC_APB1RSTR1_LPTIM1RST_Pos 31U
#define RCC_APB1RSTR1_LPTIM1RST_Msk (0x1UL << RCC_APB1RSTR1_LPTIM1RST_Pos)
#define RCC_APB1RSTR1_LPTIM1RST RCC_APB1RSTR1_LPTIM1RST_Msk

#define RCC_APB1RSTR1_OPAMPRST_Pos 30U
#define RCC_APB1RSTR1_OPAMPRST_Msk (0x1UL << RCC_APB1RSTR1_OPAMPRST_Pos)
#define RCC_APB1RSTR1_OPAMPRST RCC_APB1RSTR1_OPAMPRST_Msk

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_APB1RSTR1_DAC1RST_Pos 29U
#define RCC_APB1RSTR1_DAC1RST_Msk (0x1UL << RCC_APB1RSTR1_DAC1RST_Pos)
#define RCC_APB1RSTR1_DAC1RST RCC_APB1RSTR1_DAC1RST_Msk
#endif

#define RCC_APB1RSTR1_PWRRST_Pos 28U
#define RCC_APB1RSTR1_PWRRST_Msk (0x1UL << RCC_APB1RSTR1_PWRRST_Pos)
#define RCC_APB1RSTR1_PWRRST RCC_APB1RSTR1_PWRRST_Msk

#if !defined(STM32L431) && !defined(STM32L451)
#define RCC_APB1RSTR1_USBFSRST_Pos 26U
#define RCC_APB1RSTR1_USBFSRST_Msk (0x1UL << RCC_APB1RSTR1_USBFSRST_Pos)
#define RCC_APB1RSTR1_USBFSRST RCC_APB1RSTR1_USBFSRST_Msk
#endif

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_APB1RSTR1_CAN1RST_Pos 25U
#define RCC_APB1RSTR1_CAN1RST_Msk (0x1UL << RCC_APB1RSTR1_CAN1RST_Pos)
#define RCC_APB1RSTR1_CAN1RST RCC_APB1RSTR1_CAN1RST_Msk
#endif

#define RCC_APB1RSTR1_CRSRST_Pos 24U
#define RCC_APB1RSTR1_CRSRST_Msk (0x1UL << RCC_APB1RSTR1_CRSRST_Pos)
#define RCC_APB1RSTR1_CRSRST RCC_APB1RSTR1_CRSRST_Msk

#define RCC_APB1RSTR1_I2C3RST_Pos 23U
#define RCC_APB1RSTR1_I2C3RST_Msk (0x1UL << RCC_APB1RSTR1_I2C3RST_Pos)
#define RCC_APB1RSTR1_I2C3RST RCC_APB1RSTR1_I2C3RST_Msk

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_APB1RSTR1_I2C2RST_Pos 22U
#define RCC_APB1RSTR1_I2C2RST_Msk (0x1UL << RCC_APB1RSTR1_I2C2RST_Pos)
#define RCC_APB1RSTR1_I2C2RST RCC_APB1RSTR1_I2C2RST_Msk
#endif

#define RCC_APB1RSTR1_I2C1RST_Pos 21U
#define RCC_APB1RSTR1_I2C1RST_Msk (0x1UL << RCC_APB1RSTR1_I2C1RST_Pos)
#define RCC_APB1RSTR1_I2C1RST RCC_APB1RSTR1_I2C1RST_Msk

#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define RCC_APB1RSTR1_UART4RST_Pos 19U
#define RCC_APB1RSTR1_UART4RST_Msk (0x1UL << RCC_APB1RSTR1_UART4RST_Pos)
#define RCC_APB1RSTR1_UART4RST RCC_APB1RSTR1_UART4RST_Msk
#endif

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_APB1RSTR1_USART3RST_Pos 18U
#define RCC_APB1RSTR1_USART3RST_Msk (0x1UL << RCC_APB1RSTR1_USART3RST_Pos)
#define RCC_APB1RSTR1_USART3RST RCC_APB1RSTR1_USART3RST_Msk
#endif

#define RCC_APB1RSTR1_USART2RST_Pos 17U
#define RCC_APB1RSTR1_USART2RST_Msk (0x1UL << RCC_APB1RSTR1_USART2RST_Pos)
#define RCC_APB1RSTR1_USART2RST RCC_APB1RSTR1_USART2RST_Msk

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_APB1RSTR1_SPI3RST_Pos 15U
#define RCC_APB1RSTR1_SPI3RST_Msk (0x1UL << RCC_APB1RSTR1_SPI3RST_Pos)
#define RCC_APB1RSTR1_SPI3RST RCC_APB1RSTR1_SPI3RST_Msk
#endif

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_APB1RSTR1_SPI2RST_Pos 14U
#define RCC_APB1RSTR1_SPI2RST_Msk (0x1UL << RCC_APB1RSTR1_SPI2RST_Pos)
#define RCC_APB1RSTR1_SPI2RST RCC_APB1RSTR1_SPI2RST_Msk
#endif

#if defined(STM32L433) || defined(STM32L443)
#define RCC_APB1RSTR1_LCDRST_Pos 9U
#define RCC_APB1RSTR1_LCDRST_Msk (0x1UL << RCC_APB1RSTR1_LCDRST_Pos)
#define RCC_APB1RSTR1_LCDRST RCC_APB1RSTR1_LCDRST_Msk
#endif

#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
#define RCC_APB1RSTR1_TIM7RST_Pos 5U
#define RCC_APB1RSTR1_TIM7RST_Msk (0x1UL << RCC_APB1RSTR1_TIM7RST_Pos)
#define RCC_APB1RSTR1_TIM7RST RCC_APB1RSTR1_TIM7RST_Msk
#endif

#define RCC_APB1RSTR1_TIM6RST_Pos 4U
#define RCC_APB1RSTR1_TIM6RST_Msk (0x1UL << RCC_APB1RSTR1_TIM6RST_Pos)
#define RCC_APB1RSTR1_TIM6RST RCC_APB1RSTR1_TIM6RST_Msk

#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define RCC_APB1RSTR1_TIM3RST_Pos 1U
#define RCC_APB1RSTR1_TIM3RST_Msk (0x1UL << RCC_APB1RSTR1_TIM3RST_Pos)
#define RCC_APB1RSTR1_TIM3RST RCC_APB1RSTR1_TIM3RST_Msk
#endif

#define RCC_APB1RSTR1_TIM2RST_Pos 0U
#define RCC_APB1RSTR1_TIM2RST_Msk (0x1UL << RCC_APB1RSTR1_TIM2RST_Pos)
#define RCC_APB1RSTR1_TIM2RST RCC_APB1RSTR1_TIM2RST_Msk

/*APB1RSTR2 register*/
#define RCC_APB1RSTR2_LPTIM2RST_Pos 5U
#define RCC_APB1RSTR2_LPTIM2RST_Msk (0x1UL << RCC_APB1RSTR2_LPTIM2RST_Pos)
#define RCC_APB1RSTR2_LPTIM2RST RCC_APB1RSTR2_LPTIM2RST_Msk

#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
#define RCC_APB1RSTR2_SWPMI1RST_Pos 2U
#define RCC_APB1RSTR2_SWPMI1RST_Msk (0x1UL << RCC_APB1RSTR2_SWPMI1RST_Pos)
#define RCC_APB1RSTR2_SWPMI1RST RCC_APB1RSTR2_SWPMI1RST_Msk
#endif

#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define RCC_APB1RSTR2_I2C4RST_Pos 1U
#define RCC_APB1RSTR2_I2C4RST_Msk (0x1UL << RCC_APB1RSTR2_I2C4RST_Pos)
#define RCC_APB1RSTR2_I2C4RST RCC_APB1RSTR2_I2C4RST_Msk
#endif

#define RCC_APB1RSTR2_LPUART1RST_Pos 0U
#define RCC_APB1RSTR2_LPUART1RST_Msk (0x1UL << RCC_APB1RSTR2_LPUART1RST_Pos)
#define RCC_APB1RSTR2_LPUART1RST RCC_APB1RSTR2_LPUART1RST_Msk

/*APB2RSTR register*/
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define RCC_APB2RSTR_DFSDM1RST_Pos 24U
#define RCC_APB2RSTR_DFSDM1RST_Msk (0x1UL << RCC_APB2RSTR_DFSDM1RST_Pos)
#define RCC_APB2RSTR_DFSDM1RST RCC_APB2RSTR_DFSDM1RST_Msk
#endif

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_APB2RSTR_SAI1RST_Pos 21U
#define RCC_APB2RSTR_SAI1RST_Msk (0x1UL << RCC_APB2RSTR_SAI1RST_Pos)
#define RCC_APB2RSTR_SAI1RST RCC_APB2RSTR_SAI1RST_Msk
#endif

#define RCC_APB2RSTR_TIM16RST_Pos 17U
#define RCC_APB2RSTR_TIM16RST_Msk (0x1UL << RCC_APB2RSTR_TIM16RST_Pos)
#define RCC_APB2RSTR_TIM16RST RCC_APB2RSTR_TIM16RST_Msk

#define RCC_APB2RSTR_TIM15RST_Pos 16U
#define RCC_APB2RSTR_TIM15RST_Msk (0x1UL << RCC_APB2RSTR_TIM15RST_Pos)
#define RCC_APB2RSTR_TIM15RST RCC_APB2RSTR_TIM15RST_Msk

#define RCC_APB2RSTR_USART1RST_Pos 14U
#define RCC_APB2RSTR_USART1RST_Msk (0x1UL << RCC_APB2RSTR_USART1RST_Pos)
#define RCC_APB2RSTR_USART1RST RCC_APB2RSTR_USART1RST_Msk

#define RCC_APB2RSTR_SPI1RST_Pos 12U
#define RCC_APB2RSTR_SPI1RST_Msk (0x1UL << RCC_APB2RSTR_SPI1RST_Pos)
#define RCC_APB2RSTR_SPI1RST RCC_APB2RSTR_SPI1RST_Msk

#define RCC_APB2RSTR_TIM1RST_Pos 11U
#define RCC_APB2RSTR_TIM1RST_Msk (0x1UL << RCC_APB2RSTR_TIM1RST_Pos)
#define RCC_APB2RSTR_TIM1RST RCC_APB2RSTR_TIM1RST_Msk

#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
#define RCC_APB2RSTR_SDMMC1RST_Pos 10U
#define RCC_APB2RSTR_SDMMC1RST_Msk (0x1UL << RCC_APB2RSTR_SDMMC1RST_Pos)
#define RCC_APB2RSTR_SDMMC1RST RCC_APB2RSTR_SDMMC1RST_Msk
#endif

#define RCC_APB2RSTR_SYSCFGRST_Pos 0U
#define RCC_APB2RSTR_SYSCFGRST_Msk (0x1UL << RCC_APB2RSTR_SYSCFGRST_Pos)
#define RCC_APB2RSTR_SYSCFGRST RCC_APB2RSTR_SYSCFGRST_Msk

/*AHB1ENR register*/
#define RCC_AHB1ENR_TSCEN_Pos 16U
#define RCC_AHB1ENR_TSCEN_Msk (0x1UL << RCC_AHB1ENR_TSCEN_Pos)
#define RCC_AHB1ENR_TSCEN RCC_AHB1ENR_TSCEN_Msk

#define RCC_AHB1ENR_CRCEN_Pos 12U
#define RCC_AHB1ENR_CRCEN_Msk (0x1UL << RCC_AHB1ENR_CRCEN_Pos)
#define RCC_AHB1ENR_CRCEN RCC_AHB1ENR_CRCEN_Msk

#define RCC_AHB1ENR_FLASHEN_Pos 8U
#define RCC_AHB1ENR_FLASHEN_Msk (0x1UL << RCC_AHB1ENR_FLASHEN_Pos)
#define RCC_AHB1ENR_FLASHEN RCC_AHB1ENR_FLASHEN_Msk

#define RCC_AHB1ENR_DMA2EN_Pos 1U
#define RCC_AHB1ENR_DMA2EN_Msk (0x1UL << RCC_AHB1ENR_DMA2EN_Pos)
#define RCC_AHB1ENR_DMA2EN RCC_AHB1ENR_DMA2EN_Msk

#define RCC_AHB1ENR_DMA1EN_Pos 0U
#define RCC_AHB1ENR_DMA1EN_Msk (0x1UL << RCC_AHB1ENR_DMA1EN_Pos)
#define RCC_AHB1ENR_DMA1EN RCC_AHB1ENR_DMA1EN_Msk

/*AHB2ENR register*/
#define RCC_AHB2ENR_RNGEN_Pos 18U
#define RCC_AHB2ENR_RNGEN_Msk (0x1UL << RCC_AHB2ENR_RNGEN_Pos)
#define RCC_AHB2ENR_RNGEN RCC_AHB2ENR_RNGEN_Msk

#if defined(STM32L422) || defined(STM32L442) || defined(STM32L443) || defined(STM32L462)
#define RCC_AHB2ENR_AESEN_Pos 16U
#define RCC_AHB2ENR_AESEN_Msk (0x1UL << RCC_AHB2ENR_AESEN_Pos)
#define RCC_AHB2ENR_AESEN RCC_AHB2ENR_AESEN_Msk
#endif

#define RCC_AHB2ENR_ADCEN_Pos 13U
#define RCC_AHB2ENR_ADCEN_Msk (0x1UL << RCC_AHB2ENR_ADCEN_Pos)
#define RCC_AHB2ENR_ADCEN RCC_AHB2ENR_ADCEN_Msk

#define RCC_AHB2ENR_GPIOHEN_Pos 7U
#define RCC_AHB2ENR_GPIOHEN_Msk (0x1UL << RCC_AHB2ENR_GPIOHEN_Pos)
#define RCC_AHB2ENR_GPIOHEN RCC_AHB2ENR_GPIOHEN_Msk

#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
#define RCC_AHB2ENR_GPIOEEN_Pos 4U
#define RCC_AHB2ENR_GPIOEEN_Msk (0x1UL << RCC_AHB2ENR_GPIOEEN_Pos)
#define RCC_AHB2ENR_GPIOEEN RCC_AHB2ENR_GPIOEEN_Msk
#endif

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_AHB2ENR_GPIODEN_Pos 3U
#define RCC_AHB2ENR_GPIODEN_Msk (0x1UL << RCC_AHB2ENR_GPIODEN_Pos)
#define RCC_AHB2ENR_GPIODEN RCC_AHB2ENR_GPIODEN_Msk
#endif

#define RCC_AHB2ENR_GPIOCEN_Pos 2U
#define RCC_AHB2ENR_GPIOCEN_Msk (0x1UL << RCC_AHB2ENR_GPIOCEN_Pos)
#define RCC_AHB2ENR_GPIOCEN RCC_AHB2ENR_GPIOCEN_Msk

#define RCC_AHB2ENR_GPIOBEN_Pos 1U
#define RCC_AHB2ENR_GPIOBEN_Msk (0x1UL << RCC_AHB2ENR_GPIOBEN_Pos)
#define RCC_AHB2ENR_GPIOBEN RCC_AHB2ENR_GPIOBEN_Msk

#define RCC_AHB2ENR_GPIOAEN_Pos 0U
#define RCC_AHB2ENR_GPIOAEN_Msk (0x1UL << RCC_AHB2ENR_GPIOAEN_Pos)
#define RCC_AHB2ENR_GPIOAEN RCC_AHB2ENR_GPIOAEN_Msk

/*AHB3ENR register*/
#define RCC_AHB3ENR_QSPIEN_Pos 8U
#define RCC_AHB3ENR_QSPIEN_Msk (0x1UL << RCC_AHB3ENR_QSPIEN_Pos)
#define RCC_AHB3ENR_QSPIEN RCC_AHB3ENR_QSPIEN_Msk

/*APB1ENR1 register*/
#define RCC_APB1ENR1_LPTIM1EN_Pos 31U
#define RCC_APB1ENR1_LPTIM1EN_Msk (0x1UL << RCC_APB1ENR1_LPTIM1EN_Pos)
#define RCC_APB1ENR1_LPTIM1EN RCC_APB1ENR1_LPTIM1EN_Msk

#define RCC_APB1ENR1_OPAMPEN_Pos 30U
#define RCC_APB1ENR1_OPAMPEN_Msk (0x1UL << RCC_APB1ENR1_OPAMPEN_Pos)
#define RCC_APB1ENR1_OPAMPEN RCC_APB1ENR1_OPAMPEN_Msk

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_APB1ENR1_DAC1EN_Pos 29U
#define RCC_APB1ENR1_DAC1EN_Msk (0x1UL << RCC_APB1ENR1_DAC1EN_Pos)
#define RCC_APB1ENR1_DAC1EN RCC_APB1ENR1_DAC1EN_Msk
#endif

#define RCC_APB1ENR1_PWREN_Pos 28U
#define RCC_APB1ENR1_PWREN_Msk (0x1UL << RCC_APB1ENR1_PWREN_Pos)
#define RCC_APB1ENR1_PWREN RCC_APB1ENR1_PWREN_Msk

#if !defined(STM32L431) && !defined(STM32L451)
#define RCC_APB1ENR1_USBFSEN_Pos 26U
#define RCC_APB1ENR1_USBFSEN_Msk (0x1UL << RCC_APB1ENR1_USBFSEN_Pos)
#define RCC_APB1ENR1_USBFSEN RCC_APB1ENR1_USBFSEN_Msk
#endif

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_APB1ENR1_CAN1EN_Pos 25U
#define RCC_APB1ENR1_CAN1EN_Msk (0x1UL << RCC_APB1ENR1_CAN1EN_Pos)
#define RCC_APB1ENR1_CAN1EN RCC_APB1ENR1_CAN1EN_Msk
#endif

#define RCC_APB1ENR1_CRSEN_Pos 24U
#define RCC_APB1ENR1_CRSEN_Msk (0x1UL << RCC_APB1ENR1_CRSEN_Pos)
#define RCC_APB1ENR1_CRSEN RCC_APB1ENR1_CRSEN_Msk

#define RCC_APB1ENR1_I2C3EN_Pos 23U
#define RCC_APB1ENR1_I2C3EN_Msk (0x1UL << RCC_APB1ENR1_I2C3EN_Pos)
#define RCC_APB1ENR1_I2C3EN RCC_APB1ENR1_I2C3EN_Msk

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_APB1ENR1_I2C2EN_Pos 22U
#define RCC_APB1ENR1_I2C2EN_Msk (0x1UL << RCC_APB1ENR1_I2C2EN_Pos)
#define RCC_APB1ENR1_I2C2EN RCC_APB1ENR1_I2C2EN_Msk
#endif

#define RCC_APB1ENR1_I2C1EN_Pos 21U
#define RCC_APB1ENR1_I2C1EN_Msk (0x1UL << RCC_APB1ENR1_I2C1EN_Pos)
#define RCC_APB1ENR1_I2C1EN RCC_APB1ENR1_I2C1EN_Msk

#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define RCC_APB1ENR1_UART4EN_Pos 19U
#define RCC_APB1ENR1_UART4EN_Msk (0x1UL << RCC_APB1ENR1_UART4EN_Pos)
#define RCC_APB1ENR1_UART4EN RCC_APB1ENR1_UART4EN_Msk
#endif

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_APB1ENR1_USART3EN_Pos 18U
#define RCC_APB1ENR1_USART3EN_Msk (0x1UL << RCC_APB1ENR1_USART3EN_Pos)
#define RCC_APB1ENR1_USART3EN RCC_APB1ENR1_USART3EN_Msk
#endif

#define RCC_APB1ENR1_USART2EN_Pos 17U
#define RCC_APB1ENR1_USART2EN_Msk (0x1UL << RCC_APB1ENR1_USART2EN_Pos)
#define RCC_APB1ENR1_USART2EN RCC_APB1ENR1_USART2EN_Msk

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_APB1ENR1_SPI3EN_Pos 15U
#define RCC_APB1ENR1_SPI3EN_Msk (0x1UL << RCC_APB1ENR1_SPI3EN_Pos)
#define RCC_APB1ENR1_SPI3EN RCC_APB1ENR1_SPI3EN_Msk
#endif

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_APB1ENR1_SPI2EN_Pos 14U
#define RCC_APB1ENR1_SPI2EN_Msk (0x1UL << RCC_APB1ENR1_SPI2EN_Pos)
#define RCC_APB1ENR1_SPI2EN RCC_APB1ENR1_SPI2EN_Msk
#endif

#define RCC_APB1ENR1_WWDGEN_Pos 11U
#define RCC_APB1ENR1_WWDGEN_Msk (0x1UL << RCC_APB1ENR1_WWDGEN_Pos)
#define RCC_APB1ENR1_WWDGEN RCC_APB1ENR1_WWDGEN_Msk

#define RCC_APB1ENR1_RTCAPBEN_Pos 10U
#define RCC_APB1ENR1_RTCAPBEN_Msk (0x1UL << RCC_APB1ENR1_RTCAPBEN_Pos)
#define RCC_APB1ENR1_RTCAPBEN RCC_APB1ENR1_RTCAPBEN_Msk

#if defined(STM32L433) || defined(STM32L443)
#define RCC_APB1ENR1_LCDEN_Pos 9U
#define RCC_APB1ENR1_LCDEN_Msk (0x1UL << RCC_APB1ENR1_LCDEN_Pos)
#define RCC_APB1ENR1_LCDEN RCC_APB1ENR1_LCDEN_Msk
#endif

#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
#define RCC_APB1ENR1_TIM7EN_Pos 5U
#define RCC_APB1ENR1_TIM7EN_Msk (0x1UL << RCC_APB1ENR1_TIM7EN_Pos)
#define RCC_APB1ENR1_TIM7EN RCC_APB1ENR1_TIM7EN_Msk
#endif

#define RCC_APB1ENR1_TIM6EN_Pos 4U
#define RCC_APB1ENR1_TIM6EN_Msk (0x1UL << RCC_APB1ENR1_TIM6EN_Pos)
#define RCC_APB1ENR1_TIM6EN RCC_APB1ENR1_TIM6EN_Msk

#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define RCC_APB1ENR1_TIM3EN_Pos 1U
#define RCC_APB1ENR1_TIM3EN_Msk (0x1UL << RCC_APB1ENR1_TIM3EN_Pos)
#define RCC_APB1ENR1_TIM3EN RCC_APB1ENR1_TIM3EN_Msk
#endif

#define RCC_APB1ENR1_TIM2EN_Pos 0U
#define RCC_APB1ENR1_TIM2EN_Msk (0x1UL << RCC_APB1ENR1_TIM2EN_Pos)
#define RCC_APB1ENR1_TIM2EN RCC_APB1ENR1_TIM2EN_Msk

/*APB1ENR2 register*/
#define RCC_APB1ENR2_LPTIM2EN_Pos 5U
#define RCC_APB1ENR2_LPTIM2EN_Msk (0x1UL << RCC_APB1ENR2_LPTIM2EN_Pos)
#define RCC_APB1ENR2_LPTIM2EN RCC_APB1ENR2_LPTIM2EN_Msk

#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
#define RCC_APB1ENR2_SWPMI1EN_Pos 2U
#define RCC_APB1ENR2_SWPMI1EN_Msk (0x1UL << RCC_APB1ENR2_SWPMI1EN_Pos)
#define RCC_APB1ENR2_SWPMI1EN RCC_APB1ENR2_SWPMI1EN_Msk
#endif

#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define RCC_APB1ENR2_I2C4EN_Pos 1U
#define RCC_APB1ENR2_I2C4EN_Msk (0x1UL << RCC_APB1ENR2_I2C4EN_Pos)
#define RCC_APB1ENR2_I2C4EN RCC_APB1ENR2_I2C4EN_Msk
#endif

#define RCC_APB1ENR2_LPUART1EN_Pos 0U
#define RCC_APB1ENR2_LPUART1EN_Msk (0x1UL << RCC_APB1ENR2_LPUART1EN_Pos)
#define RCC_APB1ENR2_LPUART1EN RCC_APB1ENR2_LPUART1EN_Msk

/*APB2ENR register*/
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define RCC_APB2ENR_DFSDM1EN_Pos 24U
#define RCC_APB2ENR_DFSDM1EN_Msk (0x1UL << RCC_APB2ENR_DFSDM1EN_Pos)
#define RCC_APB2ENR_DFSDM1EN RCC_APB2ENR_DFSDM1EN_Msk
#endif

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_APB2ENR_SAI1EN_Pos 21U
#define RCC_APB2ENR_SAI1EN_Msk (0x1UL << RCC_APB2ENR_SAI1EN_Pos)
#define RCC_APB2ENR_SAI1EN RCC_APB2ENR_SAI1EN_Msk
#endif

#define RCC_APB2ENR_TIM16EN_Pos 17U
#define RCC_APB2ENR_TIM16EN_Msk (0x1UL << RCC_APB2ENR_TIM16EN_Pos)
#define RCC_APB2ENR_TIM16EN RCC_APB2ENR_TIM16EN_Msk

#define RCC_APB2ENR_TIM15EN_Pos 16U
#define RCC_APB2ENR_TIM15EN_Msk (0x1UL << RCC_APB2ENR_TIM15EN_Pos)
#define RCC_APB2ENR_TIM15EN RCC_APB2ENR_TIM15EN_Msk 

#define RCC_APB2ENR_USART1EN_Pos 14U
#define RCC_APB2ENR_USART1EN_Msk (0x1UL << RCC_APB2ENR_USART1EN_Pos)
#define RCC_APB2ENR_USART1EN RCC_APB2ENR_USART1EN_Msk

#define RCC_APB2ENR_SPI1EN_Pos 12U
#define RCC_APB2ENR_SPI1EN_Msk (0x1UL << RCC_APB2ENR_SPI1EN_Pos)
#define RCC_APB2ENR_SPI1EN RCC_APB2ENR_SPI1EN_Msk

#define RCC_APB2ENR_TIM1EN_Pos 11U
#define RCC_APB2ENR_TIM1EN_Msk (0x1UL << RCC_APB2ENR_TIM1EN_Pos)
#define RCC_APB2ENR_TIM1EN RCC_APB2ENR_TIM1EN_Msk

#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
#define RCC_APB2ENR_SDMMC1EN_Pos 10U
#define RCC_APB2ENR_SDMMC1EN_Msk (0x1UL << RCC_APB2ENR_SDMMC1EN_Pos)
#define RCC_APB2ENR_SDMMC1EN RCC_APB2ENR_SDMMC1EN_Msk
#endif

#define RCC_APB2ENR_FWEN_Pos 7U
#define RCC_APB2ENR_FWEN_Msk (0x1UL << RCC_APB2ENR_FWEN_Pos)
#define RCC_APB2ENR_FWEN RCC_APB2ENR_FWEN_Msk

#define RCC_APB2ENR_SYSCFGEN_Pos 0U
#define RCC_APB2ENR_SYSCFGEN_Msk (0x1UL << RCC_APB2ENR_SYSCFGEN_Pos)
#define RCC_APB2ENR_SYSCFGEN RCC_APB2ENR_SYSCFGEN_Msk

/*AHB1SMENR register*/
#define RCC_AHB1SMENR_TSCSMEN_Pos 16U
#define RCC_AHB1SMENR_TSCSMEN_Msk (0x1UL << RCC_AHB1SMENR_TSCSMEN_Pos)
#define RCC_AHB1SMENR_TSCSMEN RCC_AHB1SMENR_TSCSMEN_Msk

#define RCC_AHB1SMENR_CRCSMEN_Pos 12U
#define RCC_AHB1SMENR_CRCSMEN_Msk (0x1UL << RCC_AHB1SMENR_CRCSMEN_Pos)
#define RCC_AHB1SMENR_CRCSMEN RCC_AHB1SMENR_CRCSMEN_Msk

#define RCC_AHB1SMENR_SRAM1SMEN_Pos 9U
#define RCC_AHB1SMENR_SRAM1SMEN_Msk (0x1UL << RCC_AHB1SMENR_SRAM1SMEN_Pos)
#define RCC_AHB1SMENR_SRAM1SMEN RCC_AHB1SMENR_SRAM1SMEN_Msk

#define RCC_AHB1SMENR_FLASHSMEN_Pos 8U
#define RCC_AHB1SMENR_FLASHSMEN_Msk (0x1UL << RCC_AHB1SMENR_FLASHSMEN_Pos)
#define RCC_AHB1SMENR_FLASHSMEN RCC_AHB1SMENR_FLASHSMEN_Msk

#define RCC_AHB1SMENR_DMA2SMEN_Pos 1U
#define RCC_AHB1SMENR_DMA2SMEN_Msk (0x1UL << RCC_AHB1SMENR_DMA2SMEN_Pos)
#define RCC_AHB1SMENR_DMA2SMEN RCC_AHB1SMENR_DMA2SMEN_Msk 

#define RCC_AHB1SMENR_DMA1SMEN_Pos 0U
#define RCC_AHB1SMENR_DMA1SMEN_Msk (0x1UL << RCC_AHB1SMENR_DMA1SMEN_Pos)
#define RCC_AHB1SMENR_DMA1SMEN RCC_AHB1SMENR_DMA1SMEN_Msk

/*AHB2SMENR register*/
#define RCC_AHB2SMENR_RNGSMEN_Pos 18U
#define RCC_AHB2SMENR_RNGSMEN_Msk (0x1UL << RCC_AHB2SMENR_RNGSMEN_Pos)
#define RCC_AHB2SMENR_RNGSMEN RCC_AHB2SMENR_RNGSMEN_Msk

#if defined(STM32L422) || defined(STM32L442) || defined(STM32L443) || defined(STM32L462)
#define RCC_AHB2SMENR_AESSMEN_Pos 16U
#define RCC_AHB2SMENR_AESSMEN_Msk (0x1UL << RCC_AHB2SMENR_AESSMEN_Pos)
#defined RCC_AHB2SMENR_AESSMEN RCC_AHB2SMENR_AESSMEN_Msk
#endif

#define RCC_AHB2SMENR_ADCSMEN_Pos 13U
#define RCC_AHB2SMENR_ADCSMEN_Msk (0x1UL << RCC_AHB2SMENR_ADCSMEN_Pos)
#define RCC_AHB2SMENR_ADCSMEN RCC_AHB2SMENR_ADCSMEN_Msk

#define RCC_AHB2SMENR_SRAM2SMEN_Pos 9U
#define RCC_AHB2SMENR_SRAM2SMEN_Msk (0x1UL << RCC_AHB2SMENR_SRAM2SMEN_Pos)
#define RCC_AHB2SMENR_SRAM2SMEN RCC_AHB2SMENR_SRAM2SMEN_Msk

#define RCC_AHB2SMENR_GPIOHSMEN_Pos 7U
#define RCC_AHB2SMENR_GPIOHSMEN_Msk (0x1UL << RCC_AHB2SMENR_GPIOHSMEN_Pos)
#define RCC_AHB2SMENR_GPIOHSMEN RCC_AHB2SMENR_GPIOHSMEN_Msk

#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
#define RCC_AHB2SMENR_GPIOESMEN_Pos 4U
#define RCC_AHB2SMENR_GPIOESMEN_Msk (0x1UL << RCC_AHB2SMENR_GPIOESMEN_Pos)
#define RCC_AHB2SMENR_GPIOESMEN RCC_AHB2SMENR_GPIOESMEN_Msk
#endif

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_AHB2SMENR_GPIODSMEN_Pos 3U
#define RCC_AHB2SMENR_GPIODSMEN_Msk (0x1UL << RCC_AHB2SMENR_GPIODSMEN_Pos)
#define RCC_AHB2SMENR_GPIODSMEN RCC_AHB2SMENR_GPIODSMEN_Msk
#endif

#define RCC_AHB2SMENR_GPIOCSMEN_Pos 2U
#define RCC_AHB2SMENR_GPIOCSMEN_Msk (0x1UL << RCC_AHB2SMENR_GPIOCSMEN_Pos)
#define RCC_AHB2SMENR_GPIOCSMEN RCC_AHB2SMENR_GPIOCSMEN_Msk

#define RCC_AHB2SMENR_GPIOBSMEN_Pos 1U
#define RCC_AHB2SMENR_GPIOBSMEN_Msk (0x1UL << RCC_AHB2SMENR_GPIOBSMEN_Pos)
#define RCC_AHB2SMENR_GPIOBSMEN RCC_AHB2SMENR_GPIOBSMEN_Msk

#define RCC_AHB2SMENR_GPIOASMEN_Pos 0U
#define RCC_AHB2SMENR_GPIOASMEN_Msk (0x1UL << RCC_AHB2SMENR_GPIOASMEN_Pos)
#define RCC_AHB2SMENR_GPIOASMEN RCC_AHB2SMENR_GPIOASMEN_Msk

/*AHB3SMENR register*/
#define RCC_AHB3SMENR_QSPISMEN_Pos 8U
#define RCC_AHB3SMENR_QSPISMEN_Msk (0x1UL << RCC_AHB3SMENR_QSPISMEN_Pos)
#define RCC_AHB3SMENR_QSPISMEN RCC_AHB3SMENR_QSPISMEN_Msk

/*APB1SMENR1 register*/
#define RCC_APB1SMENR1_LPTIM1SMEN_Pos 31U
#define RCC_APB1SMENR1_LPTIM1SMEN_Msk (0x1UL << RCC_APB1SMENR1_LPTIM1SMEN_Pos)
#define RCC_APB1SMENR1_LPTIM1SMEN RCC_APB1SMENR1_LPTIM1SMEN_Msk

#define RCC_APB1SMENR1_OPAMPSMEN_Pos 30U
#define RCC_APB1SMENR1_OPAMPSMEN_Msk (0x1UL << RCC_APB1SMENR1_OPAMPSMEN_Pos)
#define RCC_APB1SMENR1_OPAMPSMEN RCC_APB1SMENR1_OPAMPSMEN_Msk

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_APB1SMENR1_DAC1SMEN_Pos 29U
#define RCC_APB1SMENR1_DAC1SMEN_Msk (0x1UL << RCC_APB1SMENR1_DAC1SMEN_Pos)
#define RCC_APB1SMENR1_DAC1SMEN RCC_APB1SMENR1_DAC1SMEN_Msk
#endif

#define RCC_APB1SMENR1_PWRSMEN_Pos 28U
#define RCC_APB1SMENR1_PWRSMEN_Msk (0x1UL << RCC_APB1SMENR1_PWRSMEN_Pos)
#define RCC_APB1SMENR1_PWRSMEN RCC_APB1SMENR1_PWRSMEN_Msk

#if !defined(STM32L431) && !defined(STM32L451)
#define RCC_APB1SMENR1_USBFSSMEN_Pos 26U
#define RCC_APB1SMENR1_USBFSSMEN_Msk (0x1UL << RCC_APB1SMENR1_USBFSSMEN_Pos)
#define RCC_APB1SMENR1_USBFSSMEN RCC_APB1SMENR1_USBFSSMEN_Msk
#endif

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_APB1SMENR1_CAN1SMEN_Pos 25U
#define RCC_APB1SMENR1_CAN1SMEN_Msk (0x1UL << RCC_APB1SMENR1_CAN1SMEN_Pos)
#define RCC_APB1SMENR1_CAN1SMEN RCC_APB1SMENR1_CAN1SMEN_Msk
#endif

#define RCC_APB1SMENR1_CRSSMEN_Pos 24U
#define RCC_APB1SMENR1_CRSSMEN_Msk (0x1UL << RCC_APB1SMENR1_CRSSMEN_Pos)
#define RCC_APB1SMENR1_CRSSMEN RCC_APB1SMENR1_CRSSMEN_Msk

#define RCC_APB1SMENR1_I2C3SMEN_Pos 23U
#define RCC_APB1SMENR1_I2C3SMEN_Msk (0x1UL << RCC_APB1SMENR1_I2C3SMEN_Pos)
#define RCC_APB1SMENR1_I2C3SMEN RCC_APB1SMENR1_I2C3SMEN_Msk

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_APB1SMENR1_I2C2SMEN_Pos 22U
#define RCC_APB1SMENR1_I2C2SMEN_Msk (0x1UL << RCC_APB1SMENR1_I2C2SMEN_Pos)
#define RCC_APB1SMENR1_I2C2SMEN RCC_APB1SMENR1_I2C2SMEN_Msk
#endif

#define RCC_APB1SMENR1_I2C1SMEN_Pos 21U
#define RCC_APB1SMENR1_I2C1SMEN_Msk (0x1UL << RCC_APB1SMENR1_I2C1SMEN_Pos)
#define RCC_APB1SMENR1_I2C1SMEN RCC_APB1SMENR1_I2C1SMEN_Msk

#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define RCC_APB1SMENR1_UART4SMEN_Pos 19U
#define RCC_APB1SMENR1_UART4SMEN_Msk (0x1UL << RCC_APB1SMENR1_UART4SMEN_Pos)
#define RCC_APB1SMENR1_UART4SMEN RCC_APB1SMENR1_UART4SMEN_Msk
#endif

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_APB1SMENR1_USART3SMEN_Pos 18U
#define RCC_APB1SMENR1_USART3SMEN_Msk (0x1UL << RCC_APB1SMENR1_USART3SMEN_Pos)
#define RCC_APB1SMENR1_USART3SMEN RCC_APB1SMENR1_USART3SMEN_Msk
#endif

#define RCC_APB1SMENR1_USART2SMEN_Pos 17U
#define RCC_APB1SMENR1_USART2SMEN_Msk (0x1UL << RCC_APB1SMENR1_USART2SMEN_Pos)
#define RCC_APB1SMENR1_USART2SMEN RCC_APB1SMENR1_USART2SMEN_Msk

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_APB1SMENR1_SPI3SMEN_Pos 15U
#define RCC_APB1SMENR1_SPI3SMEN_Msk (0x1UL << RCC_APB1SMENR1_SPI3SMEN_Pos)
#define RCC_APB1SMENR1_SPI3SMEN RCC_APB1SMENR1_SPI3SMEN_Msk
#endif

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_APB1SMENR1_SPI2SMEN_Pos 14U
#define RCC_APB1SMENR1_SPI2SMEN_Msk (0x1UL << RCC_APB1SMENR1_SPI2SMEN_Pos)
#define RCC_APB1SMENR1_SPI2SMEN RCC_APB1SMENR1_SPI2SMEN_Msk
#endif

#define RCC_APB1SMENR1_WWDGSMEN_Pos 11U
#define RCC_APB1SMENR1_WWDGSMEN_Msk (0x1UL << RCC_APB1SMENR1_WWDGSMEN_Pos)
#define RCC_APB1SMENR1_WWDGSMEN RCC_APB1SMENR1_WWDGSMEN_Msk

#define RCC_APB1SMENR1_RTCAPBSMEN_Pos 10U
#define RCC_APB1SMENR1_RTCAPBSMEN_Msk (0x1UL << RCC_APB1SMENR1_RTCAPBSMEN_Pos)
#define RCC_APB1SMENR1_RTCAPBSMEN RCC_APB1SMENR1_RTCAPBSMEN_Msk

#if defined(STM32L433) || defined(STM32L443)
#define RCC_APB1SMENR1_LCDSMEN_Pos 9U
#define RCC_APB1SMENR1_LCDSMEN_Msk (0x1UL << RCC_APB1SMENR1_LCDSMEN_Pos)
#define RCC_APB1SMENR1_LCDSMEN RCC_APB1SMENR1_LCDSMEN_Msk
#endif

#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
#define RCC_APB1SMENR1_TIM7SMEN_Pos 5U
#define RCC_APB1SMENR1_TIM7SMEN_Msk (0x1UL << RCC_APB1SMENR1_TIM7SMEN_Pos)
#define RCC_APB1SMENR1_TIM7SMEN RCC_APB1SMENR1_TIM7SMEN_Msk
#endif

#define RCC_APB1SMENR1_TIM6SMEN_Pos 4U
#define RCC_APB1SMENR1_TIM6SMEN_Msk (0x1UL << RCC_APB1SMENR1_TIM6SMEN_Pos)
#define RCC_APB1SMENR1_TIM6SMEN RCC_APB1SMENR1_TIM6SMEN_Msk

#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define RCC_APB1SMENR1_TIM3SMEN_Pos 1U
#define RCC_APB1SMENR1_TIM3SMEN_Msk (0x1UL << RCC_APB1SMENR1_TIM3SMEN_Pos)
#define RCC_APB1SMENR1_TIM3SMEN RCC_APB1SMENR1_TIM3SMEN_Msk
#endif

#define RCC_APB1SMENR1_TIM2SMEN_Pos 0U
#define RCC_APB1SMENR1_TIM2SMEN_Msk (0x1UL << RCC_APB1SMENR1_TIM2SMEN_Pos)
#define RCC_APB1SMENR1_TIM2SMEN RCC_APB1SMENR1_TIM2SMEN_Msk

/*APB1SMENR2 register*/
#define RCC_APB1SMENR2_LPTIM2SMEN_Pos 5U
#define RCC_APB1SMENR2_LPTIM2SMEN_Msk (0x1UL << RCC_APB1SMENR2_LPTIM2SMEN_Pos)
#define RCC_APB1SMENR2_LPTIM2SMEN RCC_APB1SMENR2_LPTIM2SMEN_Msk

#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
#define RCC_APB1SMENR2_SWPMI1SMEN_Pos 2U
#define RCC_APB1SMENR2_SWPMI1SMEN_Msk (0x1UL << RCC_APB1SMENR2_SWPMI1SMEN_Pos)
#define RCC_APB1SMENR2_SWPMI1SMEN RCC_APB1SMENR2_SWPMI1SMEN_Msk
#endif

#define RCC_APB1SMENR2_LPUART1SMEN_Pos 0U
#define RCC_APB1SMENR2_LPUART1SMEN_Msk (0x1UL << RCC_APB1SMENR2_LPUART1SMEN_Pos)
#define RCC_APB1SMENR2_LPUART1SMEN RCC_APB1SMENR2_LPUART1SMEN_Msk

/*APB2SMENR register*/
#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_APB2SMENR_SAI1SMEN_Pos 21U
#define RCC_APB2SMENR_SAI1SMEN_Msk (0x1UL << RCC_APB2SMENR_SAI1SMEN_Pos)
#define RCC_APB2SMENR_SAI1SMEN RCC_APB2SMENR_SAI1SMEN_Msk
#endif

#define RCC_APB2SMENR_TIM16SMEN_Pos 17U
#define RCC_APB2SMENR_TIM16SMEN_Msk (0x1UL << RCC_APB2SMENR_TIM16SMEN_Pos)
#define RCC_APB2SMENR_TIM16SMEN RCC_APB2SMENR_TIM16SMEN_Msk

#define RCC_APB2SMENR_TIM15SMEN_Pos 16U
#define RCC_APB2SMENR_TIM15SMEN_Msk (0x1UL << RCC_APB2SMENR_TIM15SMEN_Pos)
#define RCC_APB2SMENR_TIM15SMEN RCC_APB2SMENR_TIM15SMEN_Msk

#define RCC_APB2SMENR_USART1SMEN_Pos 14U
#define RCC_APB2SMENR_USART1SMEN_Msk (0x1UL << RCC_APB2SMENR_USART1SMEN_Pos)
#define RCC_APB2SMENR_USART1SMEN RCC_APB2SMENR_USART1SMEN_Msk

#define RCC_APB2SMENR_SPI1SMEN_Pos 12U
#define RCC_APB2SMENR_SPI1SMEN_Msk (0x1UL << RCC_APB2SMENR_SPI1SMEN_Pos)
#define RCC_APB2SMENR_SPI1SMEN RCC_APB2SMENR_SPI1SMEN_Msk

#define RCC_APB2SMENR_TIM1SMEN_Pos 11U
#define RCC_APB2SMENR_TIM1SMEN_Msk (0x1UL << RCC_APB2SMENR_TIM1SMEN_Pos)
#define RCC_APB2SMENR_TIM1SMEN RCC_APB2SMENR_TIM1SMEN_Msk

#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
#define RCC_APB2SMENR_SDMMC1SMEN_Pos 10U
#define RCC_APB2SMENR_SDMMC1SMEN_Msk (0x1UL << RCC_APB2SMENR_SDMMC1SMEN_Pos)
#define RCC_APB2SMENR_SDMMC1SMEN RCC_APB2SMENR_SDMMC1SMEN_Msk
#endif

#define RCC_APB2SMENR_SYSCFGSMEN_Pos 0U
#define RCC_APB2SMENR_SYSCFGSMEN_Msk (0x1UL << RCC_APB2SMENR_SYSCFGSMEN_Pos)
#define RCC_APB2SMENR_SYSCFGSMEN RCC_APB2SMENR_SYSCFGSMEN_Msk

/*CCIPR register*/
#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
#define RCC_CCIPR_SWPMI1SEL_Pos 30U
#define RCC_CCIPR_SWPMI1SEL_Msk (0x1U << RCC_CCIPR_SWPMI1SEL_Pos)
#define RCC_CCIPR_SWPMI1SEL RCC_CCIPR_SWPMI1SEL_Msk
#endif

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_CCIPR_ADCSEL_Pos 28U
#define RCC_CCIPR_ADCSEL_Msk (0x3UL << RCC_CCIPR_ADCSEL_Pos)
#define RCC_CCIPR_ADCSEL RCC_CCIPR_ADCSEL_Msk
#define RCC_CCIPR_ADCSEL_NONE (0x0UL << RCC_CCIPR_ADCSEL_Pos) 
#define RCC_CCIPR_ADCSEL_PLLSAIR (0x1UL << RCC_CCIPR_ADCSEL_Pos) 
#define RCC_CCIPR_ADCSEL_SYSCLK (0x3UL << RCC_CCIPR_ADCSEL_Pos) 
#endif

#define RCC_CCIPR_CLK48SEL_Pos 26U
#define RCC_CCIPR_CLK48SEL_Msk (0x3UL << RCC_CCIPR_CLK48SEL_Pos)
#define RCC_CCIPR_CLK48SEL RCC_CCIPR_CLK48SEL_Msk
#define RCC_CCIPR_CLK48SEL_HSI48 (0x0UL << RCC_CCIPR_CLK48SEL_Pos)
#define RCC_CCIPR_CLK48SEL_PLLSAIQ (0x1UL << RCC_CCIPR_CLK48SEL_Pos)
#define RCC_CCIPR_CLK48SEL_PLLQ (0x2UL << RCC_CCIPR_CLK48SEL_Pos)
#define RCC_CCIPR_CLK48SEL_MSI (0x3UL << RCC_CCIPR_CLK48SEL_Pos)

#if !defined(STM32L412) && !defined(STM32L422)
#define RCC_CCIPR_SAI1SEL_Pos 22U
#define RCC_CCIPR_SAI1SEL_Msk (0x3UL << RCC_CCIPR_SAI1SEL_Pos)
#define RCC_CCIPR_SAI1SEL RCC_CCIPR_SAI1SEL_Msk
#define RCC_CCIPR_SAI1SEL_PLLSAI1P (0x0UL << RCC_CCIPR_SAI1SEL_Pos)
#define RCC_CCIPR_SAI1SEL_PLLP (0x2UL << RCC_CCIPR_SAI1SEL_Pos)
#define RCC_CCIPR_SAI1SEL_EXTCLK (0x3UL << RCC_CCIPR_SAI1SEL_Pos)
#endif

#define RCC_CCIPR_LPTIM2SEL_Pos 20U
#define RCC_CCIPR_LPTIM2SEL_Msk (0x3UL << RCC_CCIPR_LPTIM2SEL_Pos)
#define RCC_CCIPR_LPTIM2SEL RCC_CCIPR_LPTIM2SEL_Msk
#define RCC_CCIPR_LPTIM2SEL_PCLK (0x0UL << RCC_CCIPR_LPTIM2SEL_Pos)
#define RCC_CCIPR_LPTIM2SEL_LSI (0x1UL << RCC_CCIPR_LPTIM2SEL_Pos)
#define RCC_CCIPR_LPTIM2SEL_HSI16 (0x2UL << RCC_CCIPR_LPTIM2SEL_Pos)
#define RCC_CCIPR_LPTIM2SEL_LSE (0x3UL << RCC_CCIPR_LPTIM2SEL_Pos)

#define RCC_CCIPR_LPTIM1SEL_Pos 18U
#define RCC_CCIPR_LPTIM1SEL_Msk (0x3UL << RCC_CCIPR_LPTIM1SEL_Pos)
#define RCC_CCIPR_LPTIM1SEL RCC_CCIPR_LPTIM1SEL_Msk
#define RCC_CCIPR_LPTIM1SEL_PCLK (0x0UL << RCC_CCIPR_LPTIM1SEL_Pos)
#define RCC_CCIPR_LPTIM1SEL_LSI (0x1UL << RCC_CCIPR_LPTIM1SEL_Pos)
#define RCC_CCIPR_LPTIM1SEL_HSI16 (0x2UL << RCC_CCIPR_LPTIM1SEL_Pos)
#define RCC_CCIPR_LPTIM1SEL_LSE (0x3UL << RCC_CCIPR_LPTIM1SEL_Pos)

#define RCC_CCIPR_I2C3SEL_Pos 16U
#define RCC_CCIPR_I2C3SEL_Msk (0x3UL << RCC_CCIPR_I2C3SEL_Pos)
#define RCC_CCIPR_I2C3SEL RCC_CCIPR_I2C3SEL_Msk
#define RCC_CCIPR_I2C3SEL_PCLK (0x0UL << RCC_CCIPR_I2C3SEL_Pos)
#define RCC_CCIPR_I2C3SEL_SYSCLK (0x1UL << RCC_CCIPR_I2C3SEL_Pos)
#define RCC_CCIPR_I2C3SEL_HSI16 (0x2UL << RCC_CCIPR_I2C3SEL_Pos)

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_CCIPR_I2C2SEL_Pos 14U
#define RCC_CCIPR_I2C2SEL_Msk (0x3UL << RCC_CCIPR_I2C2SEL_Pos)
#define RCC_CCIPR_I2C2SEL RCC_CCIPR_I2C2SEL_Msk
#define RCC_CCIPR_I2C2SEL_PCLK (0x0UL << RCC_CCIPR_I2C2SEL_Pos)
#define RCC_CCIPR_I2C2SEL_SYSCLK (0x1UL << RCC_CCIPR_I2C2SEL_Pos)
#define RCC_CCIPR_I2C2SEL_HSI16 (0x2UL << RCC_CCIPR_I2C2SEL_Pos)
#endif

#define RCC_CCIPR_I2C1SEL_Pos 12U
#define RCC_CCIPR_I2C1SEL_Msk (0x3UL << RCC_CCIPR_I2C1SEL_Pos)
#define RCC_CCIPR_I2C1SEL RCC_CCIPR_I2C1SEL_Msk
#define RCC_CCIPR_I2C1SEL_PCLK (0x0UL << RCC_CCIPR_I2C1SEL_Pos)
#define RCC_CCIPR_I2C1SEL_SYSCLK (0x1UL << RCC_CCIPR_I2C1SEL_Pos)
#define RCC_CCIPR_I2C1SEL_HSI16 (0x2UL << RCC_CCIPR_I2C1SEL_Pos)

#define RCC_CCIPR_LPUART1SEL_Pos 10U
#define RCC_CCIPR_LPUART1SEL_Msk (0x3UL << RCC_CCIPR_LPUART1SEL_Pos)
#define RCC_CCIPR_LPUART1SEL RCC_CCIPR_LPUART1SEL_Msk
#define RCC_CCIPR_LPUART1SEL_PCLK (0x0UL << RCC_CCIPR_LPUART1SEL_Pos)
#define RCC_CCIPR_LPUART1SEL_SYSCLK (0x1UL << RCC_CCIPR_LPUART1SEL_Pos)
#define RCC_CCIPR_LPUART1SEL_HSI16 (0x2UL << RCC_CCIPR_LPUART1SEL_Pos)
#define RCC_CCIPR_LPUART1SEL_LSE (0x3UL << RCC_CCIPR_LPUART1SEL_Pos)

#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define RCC_CCIPR_UART4SEL_Pos 6U
#define RCC_CCIPR_UART4SEL_Msk (0x3UL << RCC_CCIPR_UART4SEL_Pos)
#define RCC_CCIPR_UART4SEL RCC_CCIPR_UART4SEL_Msk
#define RCC_CCIPR_UART4SEL_PCLK (0x0UL << RCC_CCIPR_UART4SEL_Pos)
#define RCC_CCIPR_UART4SEL_SYSCLK (0x1UL << RCC_CCIPR_UART4SEL_Pos)
#define RCC_CCIPR_UART4SEL_HSI16 (0x2UL << RCC_CCIPR_UART4SEL_Pos)
#define RCC_CCIPR_UART4SEL_LSE (0x3UL << RCC_CCIPR_UART4SEL_Pos)
#endif

#if !defined(STM32L432) && !defined(STM32L442)
#define RCC_CCIPR_USART3SEL_Pos 4U
#define RCC_CCIPR_USART3SEL_Msk (0x3UL << RCC_CCIPR_USART3SEL_Pos)
#define RCC_CCIPR_USART3SEL RCC_CCIPR_USART3SEL_Msk
#define RCC_CCIPR_USART3SEL_PCLK (0x0UL << RCC_CCIPR_USART3SEL_Pos)
#define RCC_CCIPR_USART3SEL_SYSCLK (0x1UL << RCC_CCIPR_USART3SEL_Pos)
#define RCC_CCIPR_USART3SEL_HSI16 (0x2UL << RCC_CCIPR_USART3SEL_Pos)
#define RCC_CCIPR_USART3SEL_LSE (0x3UL << RCC_CCIPR_USART3SEL_Pos)
#endif

#define RCC_CCIPR_USART2SEL_Pos 2U
#define RCC_CCIPR_USART2SEL_Msk (0x3UL << RCC_CCIPR_USART2SEL_Pos)
#define RCC_CCIPR_USART2SEL RCC_CCIPR_USART2SEL_Msk
#define RCC_CCIPR_USART2SEL_PCLK (0x0UL << RCC_CCIPR_USART2SEL_Pos)
#define RCC_CCIPR_USART2SEL_SYSCLK (0x1UL << RCC_CCIPR_USART2SEL_Pos)
#define RCC_CCIPR_USART2SEL_HSI16 (0x2UL << RCC_CCIPR_USART2SEL_Pos)
#define RCC_CCIPR_USART2SEL_LSE (0x3UL << RCC_CCIPR_USART2SEL_Pos)

#define RCC_CCIPR_USART1SEL_Pos 0U
#define RCC_CCIPR_USART1SEL_Msk (0x3UL << RCC_CCIPR_USART1SEL_Pos)
#define RCC_CCIPR_USART1SEL RCC_CCIPR_USART1SEL_Msk
#define RCC_CCIPR_USART1SEL_PCLK (0x0UL << RCC_CCIPR_USART1SEL_Pos)
#define RCC_CCIPR_USART1SEL_SYSCLK (0x1UL << RCC_CCIPR_USART1SEL_Pos)
#define RCC_CCIPR_USART1SEL_HSI16 (0x2UL << RCC_CCIPR_USART1SEL_Pos)
#define RCC_CCIPR_USART1SEL_LSE (0x3UL << RCC_CCIPR_USART1SEL_Pos)

/*BDCR register*/
#define RCC_BDCR_LSCOSEL_Pos 25U
#define RCC_BDCR_LSCOSEL_Msk (0x1UL << RCC_BDCR_LSCOSEL_Pos)
#define RCC_BDCR_LSCOSEL RCC_BDCR_LSCOSEL_Msk

#define RCC_BDCR_LSCOEN_Pos 24U
#define RCC_BDCR_LSCOEN_Msk (0x1UL << RCC_BDCR_LSCOEN_Pos)
#define RCC_BDCR_LSCOEN RCC_BDCR_LSCOEN_Msk

#define RCC_BDCR_BDRST_Pos 16U
#define RCC_BDCR_BDRST_Msk (0x1UL << RCC_BDCR_BDRST_Pos)
#define RCC_BDCR_BDRST RCC_BDCR_BDRST_Msk

#define RCC_BDCR_RTCEN_Pos 15U
#define RCC_BDCR_RTCEN_Msk (0x1UL << RCC_BDCR_RTCEN_Pos)
#define RCC_BDCR_RTCEN RCC_BDCR_RTCEN_Msk

#define RCC_BDCR_RTCSEL_Pos 8U
#define RCC_BDCR_RTCSEL_Msk (0x3UL << RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL RCC_BDCR_RTCSEL_Msk
#define RCC_BDCR_RTCSEL_NONE (0x0UL << RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_LSE (0x1UL << RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_LSI (0x2UL << RCC_BDCR_RTCSEL_Pos)
#define RCC_BDCR_RTCSEL_HSE (0x3UL << RCC_BDCR_RTCSEL_Pos)

#if defined(STM32L412) || defined(STM32L422)
#define RCC_BDCR_LSESYSDIS_Pos 7U
#define RCC_BDCR_LSESYSDIS_Msk (0x1UL << RCC_BDCR_LSESYSDIS_Pos)
#define RCC_BDCR_LSESYSDIS RCC_BDCR_LSESYSDIS_Msk
#endif

#define RCC_BDCR_LSECSSD_Pos 6U
#define RCC_BDCR_LSECSSD_Msk (0x1UL << RCC_BDCR_LSECSSD_Pos)
#define RCC_BDCR_LSECSSD RCC_BDCR_LSECSSD_Msk

#define RCC_BDCR_LSECSSON_Pos 5U
#define RCC_BDCR_LSECSSON_Msk (0x1UL << RCC_BDCR_LSECSSON_Pos)
#define RCC_BDCR_LSECSSON RCC_BDCR_LSECSSON_Msk

#define RCC_BDCR_LSEDRV_Pos 3U
#define RCC_BDCR_LSEDRV_Msk (0x3UL << RCC_BDCR_LSEDRV_Pos)
#define RCC_BDCR_LSEDRV RCC_BDCR_LSEDRV_Msk
#define RCC_BDCR_LSEDRV_LOWER (0x0UL << RCC_BDCR_LSEDRV_Pos)
#define RCC_BDCR_LSEDRV_MEDIUM_LOW (0x1UL << RCC_BDCR_LSEDRV_Pos)
#define RCC_BDCR_LSEDRV_MEDIUM_HIGH (0x2UL << RCC_BDCR_LSEDRV_Pos)
#define RCC_BDCR_LSEDRV_HIGHER (0x3UL << RCC_BDCR_LSEDRV_Pos)

#define RCC_BDCR_LSEBYP_Pos 2U
#define RCC_BDCR_LSEBYP_Msk (0x1UL << RCC_BDCR_LSEBYP_Pos)
#define RCC_BDCR_LSEBYP RCC_BDCR_LSEBYP_Msk

#define RCC_BDCR_LSERDY_Pos 1U
#define RCC_BDCR_LSERDY_Msk (0x1UL << RCC_BDCR_LSERDY_Pos)
#define RCC_BDCR_LSERDY RCC_BDCR_LSERDY_Msk

#define RCC_BDCR_LSEON_Pos 0U
#define RCC_BDCR_LSEON_Msk (0x1UL << RCC_BDCR_LSEON_Pos)
#define RCC_BDCR_LSEON RCC_BDCR_LSEON_Msk

/*CSR register*/
#define RCC_CSR_LPWRRSTF_Pos 31U
#define RCC_CSR_LPWRRSTF_Msk (0x1UL << RCC_CSR_LPWRRSTF_Pos)
#define RCC_CSR_LPWRRSTF RCC_CSR_LPWRRSTF_Msk

#define RCC_CSR_WWDGRSTF_Pos 30U
#define RCC_CSR_WWDGRSTF_Msk (0x1UL << RCC_CSR_WWDGRSTF_Pos)
#define RCC_CSR_WWDGRSTF RCC_CSR_WWDGRSTF_Msk

#define RCC_CSR_IWDGRSTF_Pos 29U
#define RCC_CSR_IWDGRSTF_Msk (0x1UL << RCC_CSR_IWDGRSTF_Pos)
#define RCC_CSR_IWDGRSTF RCC_CSR_IWDGRSTF_Msk

#define RCC_CSR_SFTRSTF_Pos 28U
#define RCC_CSR_SFTRSTF_Msk (0x1UL << RCC_CSR_SFTRSTF_Pos)
#define RCC_CSR_SFTRSTF RCC_CSR_SFTRSTF_Msk

#define RCC_CSR_BORRSTF_Pos 27U
#define RCC_CSR_BORRSTF_Msk (0x1UL << RCC_CSR_BORRSTF_Pos)
#define RCC_CSR_BORRSTF RCC_CSR_BORRSTF_Msk

#define RCC_CSR_PINRSTF_Pos 26U
#define RCC_CSR_PINRSTF_Msk (0x1UL << RCC_CSR_PINRSTF_Pos)
#define RCC_CSR_PINRSTF RCC_CSR_PINRSTF_Msk

#define RCC_CSR_OBLRSTF_Pos 25U
#define RCC_CSR_OBLRSTF_Msk (0x1UL << RCC_CSR_OBLRSTF_Pos)
#define RCC_CSR_OBLRSTF RCC_CSR_OBLRSTF_Msk

#define RCC_CSR_FWRSTF_Pos 24U
#define RCC_CSR_FWRSTF_Msk (0x1UL << RCC_CSR_FWRSTF_Pos)
#define RCC_CSR_FWRSTF RCC_CSR_FWRSTF_Msk

#define RCC_CSR_RMVF_Pos 23U
#define RCC_CSR_RMVF_Msk (0x1UL << RCC_CSR_RMVF_Pos)
#define RCC_CSR_RMVF RCC_CSR_RMVF_Msk

#define RCC_CSR_MSISRANGE_Pos 8U
#define RCC_CSR_MSISRANGE_Msk (0xFUL << RCC_CSR_MSISRANGE_Pos)
#define RCC_CSR_MSISRANGE RCC_CSR_MSISRANGE_Msk
#define RCC_CSR_MSISRANGE_4 (0x4UL << RCC_CSR_MSISRANGE_Pos)
#define RCC_CSR_MSISRANGE_5 (0x5UL << RCC_CSR_MSISRANGE_Pos)
#define RCC_CSR_MSISRANGE_6 (0x6UL << RCC_CSR_MSISRANGE_Pos)
#define RCC_CSR_MSISRANGE_7 (0x7UL << RCC_CSR_MSISRANGE_Pos)

#if defined(STM32L412) || defined(STM32L422)
#define RCC_CSR_LSIPREDIV_Pos 4U
#define RCC_CSR_LSIPREDIV_Msk (0x1UL << RCC_CSR_LSIPREDIV_Pos)
#define RCC_CSR_LSIPREDIV RCC_CSR_LSIPREDIV_Msk
#endif

#define RCC_CSR_LSIRDY_Pos 1U
#define RCC_CSR_LSIRDY_Msk (0x1UL << RCC_CSR_LSIRDY_Pos)
#define RCC_CSR_LSIRDY RCC_CSR_LSIRDY_Msk

#define RCC_CSR_LSION_Pos 0U
#define RCC_CSR_LSION_Msk (0x1UL << RCC_CSR_LSION_Pos)
#define RCC_CSR_LSION RCC_CSR_LSION_Msk

/*CRRCR register*/
#define RCC_CRRCR_HSI48CAL_Pos 7U
#define RCC_CRRCR_HSI48CAL_Msk (0x1FFU << RCC_CRRCR_HSI48CAL_Pos)
#define RCC_CRRCR_HSI48CAL RCC_CRRCR_HSI48CAL_Msk

#define RCC_CRRCR_HSI48RDY_Pos 1U
#define RCC_CRRCR_HSI48RDY_Msk (0x1U << RCC_CRRCR_HSI48RDY_Pos)
#define RCC_CRRCR_HSI48RDY RCC_CRRCR_HSI48RDY_Msk

#define RCC_CRRCR_HSI48ON_Pos 0U
#define RCC_CRRCR_HSI48ON_Msk (0x1U << RCC_CRRCR_HSI48ON_Pos)
#define RCC_CRRCR_HSI48ON RCC_CRRCR_HSI48ON_Msk

/*CCIPR2 register*/
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define RCC_CCIPR2_I2C4SEL_Pos 0U
#define RCC_CCIPR2_I2C4SEL_Msk (0x3U << RCC_CCIPR2_I2C4SEL_Pos)
#define RCC_CCIPR2_I2C4SEL RCC_CCIPR2_I2C4SEL_Msk
#define RCC_CCIPR2_I2C4SEL_VAL(bits) (((bits) & 0x3UL) << RCC_CCIPR2_I2C4SEL_Pos) /*!< Allowed values 0-3.*/
#endif

/* ========================================================================= */
/* ============                       CRS                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR; /*!< CRS Control Register. */
	__IOM uint32_t CFGR; /*!< CRS Configuration Register. */
	__IM uint32_t ISR; /*!< CRS Interrupt and Status Register. */
	__IOM uint32_t ICR; /*!< CRS Interrupt flag Clear Register. */
}STM32L4xx_CRS_TypeDef;

/*CR register*/
#define CRS_CR_TRIM_Pos 8U
#define CRS_CR_TRIM_Msk (0x3FUL << CRS_CR_TRIM_Pos)
#define CRS_CR_TRIM CRS_CR_TRIM_Msk

#define CRS_CR_SWSYNC_Pos 7U
#define CRS_CR_SWSYNC_Msk (0x1UL << CRS_CR_SWSYNC_Pos)
#define CRS_CR_SWSYNC CRS_CR_SWSYNC_Msk

#define CRS_CR_AUTOTRIMEN_Pos 6U
#define CRS_CR_AUTOTRIMEN_Msk (0x1UL << CRS_CR_AUTOTRIMEN_Pos)
#define CRS_CR_AUTOTRIMEN CRS_CR_AUTOTRIMEN_Msk

#define CRS_CR_CEN_Pos 5U
#define CRS_CR_CEN_Msk (0x1UL << CRS_CR_CEN_Pos)
#define CRS_CR_CEN CRS_CR_CEN_Msk

#define CRS_CR_ESYNCIE_Pos 3U
#define CRS_CR_ESYNCIE_Msk (0x1UL << CRS_CR_ESYNCIE_Pos)
#define CRS_CR_ESYNCIE CRS_CR_ESYNCIE_Msk

#define CRS_CR_ERRIE_Pos 2U
#define CRS_CR_ERRIE_Msk (0x1UL << CRS_CR_ERRIE_Pos)
#define CRS_CR_ERRIE CRS_CR_ERRIE_Msk

#define CRS_CR_SYNCWARNIE_Pos 1U
#define CRS_CR_SYNCWARNIE_Msk (0x1UL << CRS_CR_SYNCWARNIE_Pos)
#define CRS_CR_SYNCWARNIE CRS_CR_SYNCWARNIE_Msk

#define CRS_CR_SYNCOKIE_Pos 0U
#define CRS_CR_SYNCOKIE_Msk (0x1UL << CRS_CR_SYNCOKIE_Pos)
#define CRS_CR_SYNCOKIE CRS_CR_SYNCOKIE_Msk

/*CFGR register*/
#define CRS_CFGR_SYNCPOL_Pos 31U
#define CRS_CFGR_SYNCPOL_Msk (0x1UL << CRS_CFGR_SYNCPOL_Pos)
#define CRS_CFGR_SYNCPOL CRS_CFGR_SYNCPOL_Msk

#define CRS_CFGR_SYNCSRC_Pos 28U
#define CRS_CFGR_SYNCSRC_Msk (0x3UL << CRS_CFGR_SYNCSRC_Pos)
#define CRS_CFGR_SYNCSRC CRS_CFGR_SYNCSRC_Msk
#define CRS_CFGR_SYNCSRC_GPIO (0x0UL << CRS_CFGR_SYNCSRC_Pos)
#define CRS_CFGR_SYNCSRC_LSE (0x1UL << CRS_CFGR_SYNCSRC_Pos)
#define CRS_CFGR_SYNCSRC_USB_SOF (0x2UL << CRS_CFGR_SYNCSRC_Pos)

#define CRS_CFGR_SYNCDIV_Pos 24U
#define CRS_CFGR_SYNCDIV_Msk (0x7UL << CRS_CFGR_SYNCDIV_Pos)
#define CRS_CFGR_SYNCDIV CRS_CFGR_SYNCDIV_Msk
#define CRS_CFGR_SYNCDIV_1 (0x0UL << CRS_CFGR_SYNCDIV_Pos)
#define CRS_CFGR_SYNCDIV_2 (0x1UL << CRS_CFGR_SYNCDIV_Pos)
#define CRS_CFGR_SYNCDIV_4 (0x2UL << CRS_CFGR_SYNCDIV_Pos)
#define CRS_CFGR_SYNCDIV_8 (0x3UL << CRS_CFGR_SYNCDIV_Pos)
#define CRS_CFGR_SYNCDIV_16 (0x4UL << CRS_CFGR_SYNCDIV_Pos)
#define CRS_CFGR_SYNCDIV_32 (0x5UL << CRS_CFGR_SYNCDIV_Pos)
#define CRS_CFGR_SYNCDIV_64 (0x6UL << CRS_CFGR_SYNCDIV_Pos)
#define CRS_CFGR_SYNCDIV_128 (0x7UL << CRS_CFGR_SYNCDIV_Pos)

#define CRS_CFGR_FELIM_Pos 16U
#define CRS_CFGR_FELIM_Msk (0xFFUL << CRS_CFGR_FELIM_Pos)
#define CRS_CFGR_FELIM CRS_CFGR_FELIM_Msk

#define CRS_CFGR_RELOAD_Pos 0U
#define CRS_CFGR_RELOAD_Msk (0xFFFFUL << CRS_CFGR_RELOAD_Pos)
#define CRS_CFGR_RELOAD CRS_CFGR_RELOAD_Msk

/*ISR register*/
#define CRS_ISR_FECAP_Pos 16U
#define CRS_ISR_FECAP_Msk (0xFFFFUL << CRS_ISR_FECAP_Pos)
#define CRS_ISR_FECAP CRS_ISR_FECAP_Msk

#define CRS_ISR_FEDIR_Pos 15U
#define CRS_ISR_FEDIR_Msk (0x1UL << CRS_ISR_FEDIR_Pos)
#define CRS_ISR_FEDIR CRS_ISR_FEDIR_Msk

#define CRS_ISR_TRIMOVF_Pos 10U
#define CRS_ISR_TRIMOVF_Msk (0x1UL << CRS_ISR_TRIMOVF_Pos)
#define CRS_ISR_TRIMOVF CRS_ISR_TRIMOVF_Msk

#define CRS_ISR_SYNCMISS_Pos 9U
#define CRS_ISR_SYNCMISS_Msk (0x1UL << CRS_ISR_SYNCMISS_Pos)
#define CRS_ISR_SYNCMISS CRS_ISR_SYNCMISS_Msk

#define CRS_ISR_SYNCERR_Pos 8U
#define CRS_ISR_SYNCERR_Msk (0x1UL << CRS_ISR_SYNCERR_Pos)
#define CRS_ISR_SYNCERR CRS_ISR_SYNCERR_Msk

#define CRS_ISR_ESYNCF_Pos 3U
#define CRS_ISR_ESYNCF_Msk (0x1UL << CRS_ISR_ESYNCF_Pos)
#define CRS_ISR_ESYNCF CRS_ISR_ESYNCF_Msk

#define CRS_ISR_ERRF_Pos 2U
#define CRS_ISR_ERRF_Msk (0x1UL << CRS_ISR_ERRF_Pos)
#define CRS_ISR_ERRF CRS_ISR_ERRF_Msk

#define CRS_ISR_SYNCWARNF_Pos 1U
#define CRS_ISR_SYNCWARNF_Msk (0x1UL << CRS_ISR_SYNCWARNF_Pos)
#define CRS_ISR_SYNCWARNF CRS_ISR_SYNCWARNF_Msk

#define CRS_ISR_SYNCOKF_Pos 0U
#define CRS_ISR_SYNCOKF_Msk (0x1UL << CRS_ISR_SYNCOKF_Pos)
#define CRS_ISR_SYNCOKF CRS_ISR_SYNCOKF_Msk

/*ICR register*/
#define CRS_ICR_ESYNCC_Pos 3U
#define CRS_ICR_ESYNCC_Msk (0x1UL << CRS_ICR_ESYNCC_Pos)
#define CRS_ICR_ESYNCC CRS_ICR_ESYNCC_Msk

#define CRS_ICR_ERRC_Pos 2U
#define CRS_ICR_ERRC_Msk (0x1UL << CRS_ICR_ERRC_Pos)
#define CRS_ICR_ERRC CRS_ICR_ERRC_Msk

#define CRS_ICR_SYNCWARNC_Pos 1U
#define CRS_ICR_SYNCWARNC_Msk (0x1UL << CRS_ICR_SYNCWARNC_Pos)
#define CRS_ICR_SYNCWARNC CRS_ICR_SYNCWARNC_Msk

#define CRS_ICR_SYNCOKC_Pos 0U
#define CRS_ICR_SYNCOKC_Msk (0x1UL << CRS_ICR_SYNCOKC_Pos)
#define CRS_ICR_SYNCOKC CRS_ICR_SYNCOKC_Msk

/* ========================================================================= */
/* ============                       GPIO                      ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t MODER; /*!< GPIO Port Mode Register. */
	__IOM uint32_t OTYPER; /*!< GPIO Port Output Type Register. */
	__IOM uint32_t OSPEEDR; /*!< GPIO Port Output Speed Register. */
	__IOM uint32_t PUPDR; /*!< GPIO Port Pull-Up/Pull-Down Register. */
	__IM uint32_t IDR; /*!< GPIO Port Input Data Register. */
	__IOM uint32_t ODR; /*!< GPIO Port Output Data Register. */
	__OM uint32_t BSRR; /*!< GPIO Port Bit Set/Reset Register. */
	__IOM uint32_t LCKR; /*!< GPIO Port Configuration Lock Register. */
	__IOM uint32_t AFRL; /*!< GPIO Port Alternate Function Low Register. */
	__IOM uint32_t AFRH; /*!< GPIO Port Alternate Function High Register. */
	__IOM uint32_t BRR; /*!< GPIO Port Bit Reset Register. */
}STM32L4xx_GPIO_TypeDef;

/*MODER register*/
#define GPIO_MODER_MODE15_Pos 30U
#define GPIO_MODER_MODE15_Msk (0x3U << GPIO_MODER_MODE15_Pos)
#define GPIO_MODER_MODE15 GPIO_MODER_MODE15_Msk
#define GPIO_MODER_MODE15_INPUT (0x0U << GPIO_MODER_MODE15_Pos)
#define GPIO_MODER_MODE15_OUTPUT (0x1U << GPIO_MODER_MODE15_Pos)
#define GPIO_MODER_MODE15_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE15_Pos)
#define GPIO_MODER_MODE15_ANALOG (0x3U << GPIO_MODER_MODE15_Pos)

#define GPIO_MODER_MODE14_Pos 28U
#define GPIO_MODER_MODE14_Msk (0x3U << GPIO_MODER_MODE14_Pos)
#define GPIO_MODER_MODE14 GPIO_MODER_MODE14_Msk
#define GPIO_MODER_MODE14_INPUT (0x0U << GPIO_MODER_MODE14_Pos)
#define GPIO_MODER_MODE14_OUTPUT (0x1U << GPIO_MODER_MODE14_Pos)
#define GPIO_MODER_MODE14_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE14_Pos)
#define GPIO_MODER_MODE14_ANALOG (0x3U << GPIO_MODER_MODE14_Pos)

#define GPIO_MODER_MODE13_Pos 26U
#define GPIO_MODER_MODE13_Msk (0x3U << GPIO_MODER_MODE13_Pos)
#define GPIO_MODER_MODE13 GPIO_MODER_MODE13_Msk
#define GPIO_MODER_MODE13_INPUT (0x0U << GPIO_MODER_MODE13_Pos)
#define GPIO_MODER_MODE13_OUTPUT (0x1U << GPIO_MODER_MODE13_Pos)
#define GPIO_MODER_MODE13_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE13_Pos)
#define GPIO_MODER_MODE13_ANALOG (0x3U << GPIO_MODER_MODE13_Pos)

#define GPIO_MODER_MODE12_Pos 24U
#define GPIO_MODER_MODE12_Msk (0x3U << GPIO_MODER_MODE12_Pos)
#define GPIO_MODER_MODE12 GPIO_MODER_MODE12_Msk
#define GPIO_MODER_MODE12_INPUT (0x0U << GPIO_MODER_MODE12_Pos)
#define GPIO_MODER_MODE12_OUTPUT (0x1U << GPIO_MODER_MODE12_Pos)
#define GPIO_MODER_MODE12_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE12_Pos)
#define GPIO_MODER_MODE12_ANALOG (0x3U << GPIO_MODER_MODE12_Pos)

#define GPIO_MODER_MODE11_Pos 22U
#define GPIO_MODER_MODE11_Msk (0x3U << GPIO_MODER_MODE11_Pos)
#define GPIO_MODER_MODE11 GPIO_MODER_MODE11_Msk
#define GPIO_MODER_MODE11_INPUT (0x0U << GPIO_MODER_MODE11_Pos)
#define GPIO_MODER_MODE11_OUTPUT (0x1U << GPIO_MODER_MODE11_Pos)
#define GPIO_MODER_MODE11_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE11_Pos)
#define GPIO_MODER_MODE11_ANALOG (0x3U << GPIO_MODER_MODE11_Pos)

#define GPIO_MODER_MODE10_Pos 20U
#define GPIO_MODER_MODE10_Msk (0x3U << GPIO_MODER_MODE10_Pos)
#define GPIO_MODER_MODE10 GPIO_MODER_MODE10_Msk
#define GPIO_MODER_MODE10_INPUT (0x0U << GPIO_MODER_MODE10_Pos)
#define GPIO_MODER_MODE10_OUTPUT (0x1U << GPIO_MODER_MODE10_Pos)
#define GPIO_MODER_MODE10_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE10_Pos)
#define GPIO_MODER_MODE10_ANALOG (0x3U << GPIO_MODER_MODE10_Pos)

#define GPIO_MODER_MODE9_Pos 18U
#define GPIO_MODER_MODE9_Msk (0x3U << GPIO_MODER_MODE9_Pos)
#define GPIO_MODER_MODE9 GPIO_MODER_MODE9_Msk
#define GPIO_MODER_MODE9_INPUT (0x0U << GPIO_MODER_MODE9_Pos)
#define GPIO_MODER_MODE9_OUTPUT (0x1U << GPIO_MODER_MODE9_Pos)
#define GPIO_MODER_MODE9_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE9_Pos)
#define GPIO_MODER_MODE9_ANALOG (0x3U << GPIO_MODER_MODE9_Pos)

#define GPIO_MODER_MODE8_Pos 16U
#define GPIO_MODER_MODE8_Msk (0x3U << GPIO_MODER_MODE8_Pos)
#define GPIO_MODER_MODE8 GPIO_MODER_MODE8_Msk
#define GPIO_MODER_MODE8_INPUT (0x0U << GPIO_MODER_MODE8_Pos)
#define GPIO_MODER_MODE8_OUTPUT (0x1U << GPIO_MODER_MODE8_Pos)
#define GPIO_MODER_MODE8_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE8_Pos)
#define GPIO_MODER_MODE8_ANALOG (0x3U << GPIO_MODER_MODE8_Pos)

#define GPIO_MODER_MODE7_Pos 14U
#define GPIO_MODER_MODE7_Msk (0x3U << GPIO_MODER_MODE7_Pos)
#define GPIO_MODER_MODE7 GPIO_MODER_MODE7_Msk
#define GPIO_MODER_MODE7_INPUT (0x0U << GPIO_MODER_MODE7_Pos)
#define GPIO_MODER_MODE7_OUTPUT (0x1U << GPIO_MODER_MODE7_Pos)
#define GPIO_MODER_MODE7_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE7_Pos)
#define GPIO_MODER_MODE7_ANALOG (0x3U << GPIO_MODER_MODE7_Pos)

#define GPIO_MODER_MODE6_Pos 12U
#define GPIO_MODER_MODE6_Msk (0x3U << GPIO_MODER_MODE6_Pos)
#define GPIO_MODER_MODE6 GPIO_MODER_MODE6_Msk
#define GPIO_MODER_MODE6_INPUT (0x0U << GPIO_MODER_MODE6_Pos)
#define GPIO_MODER_MODE6_OUTPUT (0x1U << GPIO_MODER_MODE6_Pos)
#define GPIO_MODER_MODE6_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE6_Pos)
#define GPIO_MODER_MODE6_ANALOG (0x3U << GPIO_MODER_MODE6_Pos)

#define GPIO_MODER_MODE5_Pos 10U
#define GPIO_MODER_MODE5_Msk (0x3U << GPIO_MODER_MODE5_Pos)
#define GPIO_MODER_MODE5 GPIO_MODER_MODE5_Msk
#define GPIO_MODER_MODE5_INPUT (0x0U << GPIO_MODER_MODE5_Pos)
#define GPIO_MODER_MODE5_OUTPUT (0x1U << GPIO_MODER_MODE5_Pos)
#define GPIO_MODER_MODE5_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE5_Pos)
#define GPIO_MODER_MODE5_ANALOG (0x3U << GPIO_MODER_MODE5_Pos)

#define GPIO_MODER_MODE4_Pos 8U
#define GPIO_MODER_MODE4_Msk (0x3U << GPIO_MODER_MODE4_Pos)
#define GPIO_MODER_MODE4 GPIO_MODER_MODE4_Msk
#define GPIO_MODER_MODE4_INPUT (0x0U << GPIO_MODER_MODE4_Pos)
#define GPIO_MODER_MODE4_OUTPUT (0x1U << GPIO_MODER_MODE4_Pos)
#define GPIO_MODER_MODE4_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE4_Pos)
#define GPIO_MODER_MODE4_ANALOG (0x3U << GPIO_MODER_MODE4_Pos)

#define GPIO_MODER_MODE3_Pos 6U
#define GPIO_MODER_MODE3_Msk (0x3U << GPIO_MODER_MODE3_Pos)
#define GPIO_MODER_MODE3 GPIO_MODER_MODE3_Msk
#define GPIO_MODER_MODE3_INPUT (0x0U << GPIO_MODER_MODE3_Pos)
#define GPIO_MODER_MODE3_OUTPUT (0x1U << GPIO_MODER_MODE3_Pos)
#define GPIO_MODER_MODE3_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE3_Pos)
#define GPIO_MODER_MODE3_ANALOG (0x3U << GPIO_MODER_MODE3_Pos)

#define GPIO_MODER_MODE2_Pos 4U
#define GPIO_MODER_MODE2_Msk (0x3U << GPIO_MODER_MODE2_Pos)
#define GPIO_MODER_MODE2 GPIO_MODER_MODE2_Msk
#define GPIO_MODER_MODE2_INPUT (0x0U << GPIO_MODER_MODE2_Pos)
#define GPIO_MODER_MODE2_OUTPUT (0x1U << GPIO_MODER_MODE2_Pos)
#define GPIO_MODER_MODE2_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE2_Pos)
#define GPIO_MODER_MODE2_ANALOG (0x3U << GPIO_MODER_MODE2_Pos)

#define GPIO_MODER_MODE1_Pos 2U
#define GPIO_MODER_MODE1_Msk (0x3U << GPIO_MODER_MODE1_Pos)
#define GPIO_MODER_MODE1 GPIO_MODER_MODE1_Msk
#define GPIO_MODER_MODE1_INPUT (0x0U << GPIO_MODER_MODE1_Pos)
#define GPIO_MODER_MODE1_OUTPUT (0x1U << GPIO_MODER_MODE1_Pos)
#define GPIO_MODER_MODE1_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE1_Pos)
#define GPIO_MODER_MODE1_ANALOG (0x3U << GPIO_MODER_MODE1_Pos)

#define GPIO_MODER_MODE0_Pos 0U
#define GPIO_MODER_MODE0_Msk (0x3U << GPIO_MODER_MODE0_Pos)
#define GPIO_MODER_MODE0 GPIO_MODER_MODE0_Msk
#define GPIO_MODER_MODE0_INPUT (0x0U << GPIO_MODER_MODE0_Pos)
#define GPIO_MODER_MODE0_OUTPUT (0x1U << GPIO_MODER_MODE0_Pos)
#define GPIO_MODER_MODE0_ALTERNATE_FUNCTION (0x2U << GPIO_MODER_MODE0_Pos)
#define GPIO_MODER_MODE0_ANALOG (0x3U << GPIO_MODER_MODE0_Pos)

/*OTYPER register*/
#define GPIO_OTYPER_OT15_Pos 15U
#define GPIO_OTYPER_OT15_Msk (0x1U << GPIO_OTYPER_OT15_Pos)
#define GPIO_OTYPER_OT15 GPIO_OTYPER_OT15_Msk

#define GPIO_OTYPER_OT14_Pos 14U
#define GPIO_OTYPER_OT14_Msk (0x1U << GPIO_OTYPER_OT14_Pos)
#define GPIO_OTYPER_OT14 GPIO_OTYPER_OT14_Msk

#define GPIO_OTYPER_OT13_Pos 13U
#define GPIO_OTYPER_OT13_Msk (0x1U << GPIO_OTYPER_OT13_Pos)
#define GPIO_OTYPER_OT13 GPIO_OTYPER_OT13_Msk

#define GPIO_OTYPER_OT12_Pos 12U
#define GPIO_OTYPER_OT12_Msk (0x1U << GPIO_OTYPER_OT12_Pos)
#define GPIO_OTYPER_OT12 GPIO_OTYPER_OT12_Msk

#define GPIO_OTYPER_OT11_Pos 11U
#define GPIO_OTYPER_OT11_Msk (0x1U << GPIO_OTYPER_OT11_Pos)
#define GPIO_OTYPER_OT11 GPIO_OTYPER_OT11_Msk

#define GPIO_OTYPER_OT10_Pos 10U
#define GPIO_OTYPER_OT10_Msk (0x1U << GPIO_OTYPER_OT10_Pos)
#define GPIO_OTYPER_OT10 GPIO_OTYPER_OT10_Msk

#define GPIO_OTYPER_OT9_Pos 9U
#define GPIO_OTYPER_OT9_Msk (0x1U << GPIO_OTYPER_OT9_Pos)
#define GPIO_OTYPER_OT9 GPIO_OTYPER_OT9_Msk

#define GPIO_OTYPER_OT8_Pos 8U
#define GPIO_OTYPER_OT8_Msk (0x1U << GPIO_OTYPER_OT8_Pos)
#define GPIO_OTYPER_OT8 GPIO_OTYPER_OT8_Msk

#define GPIO_OTYPER_OT7_Pos 7U
#define GPIO_OTYPER_OT7_Msk (0x1U << GPIO_OTYPER_OT7_Pos)
#define GPIO_OTYPER_OT7 GPIO_OTYPER_OT7_Msk

#define GPIO_OTYPER_OT6_Pos 6U
#define GPIO_OTYPER_OT6_Msk (0x1U << GPIO_OTYPER_OT6_Pos)
#define GPIO_OTYPER_OT6 GPIO_OTYPER_OT6_Msk

#define GPIO_OTYPER_OT5_Pos 5U
#define GPIO_OTYPER_OT5_Msk (0x1U << GPIO_OTYPER_OT5_Pos)
#define GPIO_OTYPER_OT5 GPIO_OTYPER_OT5_Msk

#define GPIO_OTYPER_OT4_Pos 4U
#define GPIO_OTYPER_OT4_Msk (0x1U << GPIO_OTYPER_OT4_Pos)
#define GPIO_OTYPER_OT4 GPIO_OTYPER_OT4_Msk

#define GPIO_OTYPER_OT3_Pos 3U
#define GPIO_OTYPER_OT3_Msk (0x1U << GPIO_OTYPER_OT3_Pos)
#define GPIO_OTYPER_OT3 GPIO_OTYPER_OT3_Msk

#define GPIO_OTYPER_OT2_Pos 2U
#define GPIO_OTYPER_OT2_Msk (0x1U << GPIO_OTYPER_OT2_Pos)
#define GPIO_OTYPER_OT2 GPIO_OTYPER_OT2_Msk

#define GPIO_OTYPER_OT1_Pos 1U
#define GPIO_OTYPER_OT1_Msk (0x1U << GPIO_OTYPER_OT1_Pos)
#define GPIO_OTYPER_OT1 GPIO_OTYPER_OT1_Msk

#define GPIO_OTYPER_OT0_Pos 0U
#define GPIO_OTYPER_OT0_Msk (0x1U << GPIO_OTYPER_OT0_Pos)
#define GPIO_OTYPER_OT0 GPIO_OTYPER_OT0_Msk

/*OSPEEDR register*/
#define GPIO_OSPEEDR_OSPEED15_Pos 30U
#define GPIO_OSPEEDR_OSPEED15_Msk (0x3U << GPIO_OSPEEDR_OSPEED15_Pos)
#define GPIO_OSPEEDR_OSPEED15 GPIO_OSPEEDR_OSPEED15_Msk
#define GPIO_OSPEEDR_OSPEED15_LOW (0x0U << GPIO_OSPEEDR_OSPEED15_Pos)
#define GPIO_OSPEEDR_OSPEED15_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED15_Pos)
#define GPIO_OSPEEDR_OSPEED15_HIGH (0x2U << GPIO_OSPEEDR_OSPEED15_Pos)
#define GPIO_OSPEEDR_OSPEED15_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED15_Pos)

#define GPIO_OSPEEDR_OSPEED14_Pos 28U
#define GPIO_OSPEEDR_OSPEED14_Msk (0x3U << GPIO_OSPEEDR_OSPEED14_Pos)
#define GPIO_OSPEEDR_OSPEED14 GPIO_OSPEEDR_OSPEED14_Msk
#define GPIO_OSPEEDR_OSPEED14_LOW (0x0U << GPIO_OSPEEDR_OSPEED14_Pos)
#define GPIO_OSPEEDR_OSPEED14_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED14_Pos)
#define GPIO_OSPEEDR_OSPEED14_HIGH (0x2U << GPIO_OSPEEDR_OSPEED14_Pos)
#define GPIO_OSPEEDR_OSPEED14_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED14_Pos)

#define GPIO_OSPEEDR_OSPEED13_Pos 26U
#define GPIO_OSPEEDR_OSPEED13_Msk (0x3U << GPIO_OSPEEDR_OSPEED13_Pos)
#define GPIO_OSPEEDR_OSPEED13 GPIO_OSPEEDR_OSPEED13_Msk
#define GPIO_OSPEEDR_OSPEED13_LOW (0x0U << GPIO_OSPEEDR_OSPEED13_Pos)
#define GPIO_OSPEEDR_OSPEED13_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED13_Pos)
#define GPIO_OSPEEDR_OSPEED13_HIGH (0x2U << GPIO_OSPEEDR_OSPEED13_Pos)
#define GPIO_OSPEEDR_OSPEED13_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED13_Pos)

#define GPIO_OSPEEDR_OSPEED12_Pos 24U
#define GPIO_OSPEEDR_OSPEED12_Msk (0x3U << GPIO_OSPEEDR_OSPEED12_Pos)
#define GPIO_OSPEEDR_OSPEED12 GPIO_OSPEEDR_OSPEED12_Msk
#define GPIO_OSPEEDR_OSPEED12_LOW (0x0U << GPIO_OSPEEDR_OSPEED12_Pos)
#define GPIO_OSPEEDR_OSPEED12_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED12_Pos)
#define GPIO_OSPEEDR_OSPEED12_HIGH (0x2U << GPIO_OSPEEDR_OSPEED12_Pos)
#define GPIO_OSPEEDR_OSPEED12_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED12_Pos)

#define GPIO_OSPEEDR_OSPEED11_Pos 22U
#define GPIO_OSPEEDR_OSPEED11_Msk (0x3U << GPIO_OSPEEDR_OSPEED11_Pos)
#define GPIO_OSPEEDR_OSPEED11 GPIO_OSPEEDR_OSPEED11_Msk
#define GPIO_OSPEEDR_OSPEED11_LOW (0x0U << GPIO_OSPEEDR_OSPEED11_Pos)
#define GPIO_OSPEEDR_OSPEED11_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED11_Pos)
#define GPIO_OSPEEDR_OSPEED11_HIGH (0x2U << GPIO_OSPEEDR_OSPEED11_Pos)
#define GPIO_OSPEEDR_OSPEED11_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED11_Pos)

#define GPIO_OSPEEDR_OSPEED10_Pos 20U
#define GPIO_OSPEEDR_OSPEED10_Msk (0x3U << GPIO_OSPEEDR_OSPEED10_Pos)
#define GPIO_OSPEEDR_OSPEED10 GPIO_OSPEEDR_OSPEED10_Msk
#define GPIO_OSPEEDR_OSPEED10_LOW (0x0U << GPIO_OSPEEDR_OSPEED10_Pos)
#define GPIO_OSPEEDR_OSPEED10_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED10_Pos)
#define GPIO_OSPEEDR_OSPEED10_HIGH (0x2U << GPIO_OSPEEDR_OSPEED10_Pos)
#define GPIO_OSPEEDR_OSPEED10_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED10_Pos)

#define GPIO_OSPEEDR_OSPEED9_Pos 18U
#define GPIO_OSPEEDR_OSPEED9_Msk (0x3U << GPIO_OSPEEDR_OSPEED9_Pos)
#define GPIO_OSPEEDR_OSPEED9 GPIO_OSPEEDR_OSPEED9_Msk
#define GPIO_OSPEEDR_OSPEED9_LOW (0x0U << GPIO_OSPEEDR_OSPEED9_Pos)
#define GPIO_OSPEEDR_OSPEED9_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED9_Pos)
#define GPIO_OSPEEDR_OSPEED9_HIGH (0x2U << GPIO_OSPEEDR_OSPEED9_Pos)
#define GPIO_OSPEEDR_OSPEED9_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED9_Pos)

#define GPIO_OSPEEDR_OSPEED8_Pos 16U
#define GPIO_OSPEEDR_OSPEED8_Msk (0x3U << GPIO_OSPEEDR_OSPEED8_Pos)
#define GPIO_OSPEEDR_OSPEED8 GPIO_OSPEEDR_OSPEED8_Msk
#define GPIO_OSPEEDR_OSPEED8_LOW (0x0U << GPIO_OSPEEDR_OSPEED8_Pos)
#define GPIO_OSPEEDR_OSPEED8_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED8_Pos)
#define GPIO_OSPEEDR_OSPEED8_HIGH (0x2U << GPIO_OSPEEDR_OSPEED8_Pos)
#define GPIO_OSPEEDR_OSPEED8_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED8_Pos)

#define GPIO_OSPEEDR_OSPEED7_Pos 14U
#define GPIO_OSPEEDR_OSPEED7_Msk (0x3U << GPIO_OSPEEDR_OSPEED7_Pos)
#define GPIO_OSPEEDR_OSPEED7 GPIO_OSPEEDR_OSPEED7_Msk
#define GPIO_OSPEEDR_OSPEED7_LOW (0x0U << GPIO_OSPEEDR_OSPEED7_Pos)
#define GPIO_OSPEEDR_OSPEED7_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED7_Pos)
#define GPIO_OSPEEDR_OSPEED7_HIGH (0x2U << GPIO_OSPEEDR_OSPEED7_Pos)
#define GPIO_OSPEEDR_OSPEED7_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED7_Pos)

#define GPIO_OSPEEDR_OSPEED6_Pos 12U
#define GPIO_OSPEEDR_OSPEED6_Msk (0x3U << GPIO_OSPEEDR_OSPEED6_Pos)
#define GPIO_OSPEEDR_OSPEED6 GPIO_OSPEEDR_OSPEED6_Msk
#define GPIO_OSPEEDR_OSPEED6_LOW (0x0U << GPIO_OSPEEDR_OSPEED6_Pos)
#define GPIO_OSPEEDR_OSPEED6_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED6_Pos)
#define GPIO_OSPEEDR_OSPEED6_HIGH (0x2U << GPIO_OSPEEDR_OSPEED6_Pos)
#define GPIO_OSPEEDR_OSPEED6_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED6_Pos)

#define GPIO_OSPEEDR_OSPEED5_Pos 10U
#define GPIO_OSPEEDR_OSPEED5_Msk (0x3U << GPIO_OSPEEDR_OSPEED5_Pos)
#define GPIO_OSPEEDR_OSPEED5 GPIO_OSPEEDR_OSPEED5_Msk
#define GPIO_OSPEEDR_OSPEED5_LOW (0x0U << GPIO_OSPEEDR_OSPEED5_Pos)
#define GPIO_OSPEEDR_OSPEED5_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED5_Pos)
#define GPIO_OSPEEDR_OSPEED5_HIGH (0x2U << GPIO_OSPEEDR_OSPEED5_Pos)
#define GPIO_OSPEEDR_OSPEED5_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED5_Pos)

#define GPIO_OSPEEDR_OSPEED4_Pos 8U
#define GPIO_OSPEEDR_OSPEED4_Msk (0x3U << GPIO_OSPEEDR_OSPEED4_Pos)
#define GPIO_OSPEEDR_OSPEED4 GPIO_OSPEEDR_OSPEED4_Msk
#define GPIO_OSPEEDR_OSPEED4_LOW (0x0U << GPIO_OSPEEDR_OSPEED4_Pos)
#define GPIO_OSPEEDR_OSPEED4_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED4_Pos)
#define GPIO_OSPEEDR_OSPEED4_HIGH (0x2U << GPIO_OSPEEDR_OSPEED4_Pos)
#define GPIO_OSPEEDR_OSPEED4_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED4_Pos)

#define GPIO_OSPEEDR_OSPEED3_Pos 6U
#define GPIO_OSPEEDR_OSPEED3_Msk (0x3U << GPIO_OSPEEDR_OSPEED3_Pos)
#define GPIO_OSPEEDR_OSPEED3 GPIO_OSPEEDR_OSPEED3_Msk
#define GPIO_OSPEEDR_OSPEED3_LOW (0x0U << GPIO_OSPEEDR_OSPEED3_Pos)
#define GPIO_OSPEEDR_OSPEED3_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED3_Pos)
#define GPIO_OSPEEDR_OSPEED3_HIGH (0x2U << GPIO_OSPEEDR_OSPEED3_Pos)
#define GPIO_OSPEEDR_OSPEED3_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED3_Pos)

#define GPIO_OSPEEDR_OSPEED2_Pos 4U
#define GPIO_OSPEEDR_OSPEED2_Msk (0x3U << GPIO_OSPEEDR_OSPEED2_Pos)
#define GPIO_OSPEEDR_OSPEED2 GPIO_OSPEEDR_OSPEED2_Msk
#define GPIO_OSPEEDR_OSPEED2_LOW (0x0U << GPIO_OSPEEDR_OSPEED2_Pos)
#define GPIO_OSPEEDR_OSPEED2_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED2_Pos)
#define GPIO_OSPEEDR_OSPEED2_HIGH (0x2U << GPIO_OSPEEDR_OSPEED2_Pos)
#define GPIO_OSPEEDR_OSPEED2_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED2_Pos)

#define GPIO_OSPEEDR_OSPEED1_Pos 2U
#define GPIO_OSPEEDR_OSPEED1_Msk (0x3U << GPIO_OSPEEDR_OSPEED1_Pos)
#define GPIO_OSPEEDR_OSPEED1 GPIO_OSPEEDR_OSPEED1_Msk
#define GPIO_OSPEEDR_OSPEED1_LOW (0x0U << GPIO_OSPEEDR_OSPEED1_Pos)
#define GPIO_OSPEEDR_OSPEED1_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED1_Pos)
#define GPIO_OSPEEDR_OSPEED1_HIGH (0x2U << GPIO_OSPEEDR_OSPEED1_Pos)
#define GPIO_OSPEEDR_OSPEED1_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED1_Pos)

#define GPIO_OSPEEDR_OSPEED0_Pos 0U
#define GPIO_OSPEEDR_OSPEED0_Msk (0x3U << GPIO_OSPEEDR_OSPEED0_Pos)
#define GPIO_OSPEEDR_OSPEED0 GPIO_OSPEEDR_OSPEED0_Msk
#define GPIO_OSPEEDR_OSPEED0_LOW (0x0U << GPIO_OSPEEDR_OSPEED0_Pos)
#define GPIO_OSPEEDR_OSPEED0_MEDIUM (0x1U << GPIO_OSPEEDR_OSPEED0_Pos)
#define GPIO_OSPEEDR_OSPEED0_HIGH (0x2U << GPIO_OSPEEDR_OSPEED0_Pos)
#define GPIO_OSPEEDR_OSPEED0_VERY_HIGH (0x3U << GPIO_OSPEEDR_OSPEED0_Pos)

/*PUPDR register*/
#define GPIO_PUPDR_PUPD15_Pos 30U
#define GPIO_PUPDR_PUPD15_Msk (0x3U << GPIO_PUPDR_PUPD15_Pos)
#define GPIO_PUPDR_PUPD15 GPIO_PUPDR_PUPD15_Msk
#define GPIO_PUPDR_PUPD15_NONE (0x0U << GPIO_PUPDR_PUPD15_Pos)
#define GPIO_PUPDR_PUPD15_PULL_UP (0x1U << GPIO_PUPDR_PUPD15_Pos)
#define GPIO_PUPDR_PUPD15_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD15_Pos)

#define GPIO_PUPDR_PUPD14_Pos 28U
#define GPIO_PUPDR_PUPD14_Msk (0x3U << GPIO_PUPDR_PUPD14_Pos)
#define GPIO_PUPDR_PUPD14 GPIO_PUPDR_PUPD14_Msk
#define GPIO_PUPDR_PUPD14_NONE (0x0U << GPIO_PUPDR_PUPD14_Pos)
#define GPIO_PUPDR_PUPD14_PULL_UP (0x1U << GPIO_PUPDR_PUPD14_Pos)
#define GPIO_PUPDR_PUPD14_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD14_Pos)

#define GPIO_PUPDR_PUPD13_Pos 26U
#define GPIO_PUPDR_PUPD13_Msk (0x3U << GPIO_PUPDR_PUPD13_Pos)
#define GPIO_PUPDR_PUPD13 GPIO_PUPDR_PUPD13_Msk
#define GPIO_PUPDR_PUPD13_NONE (0x0U << GPIO_PUPDR_PUPD13_Pos)
#define GPIO_PUPDR_PUPD13_PULL_UP (0x1U << GPIO_PUPDR_PUPD13_Pos)
#define GPIO_PUPDR_PUPD13_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD13_Pos)

#define GPIO_PUPDR_PUPD12_Pos 24U
#define GPIO_PUPDR_PUPD12_Msk (0x3U << GPIO_PUPDR_PUPD12_Pos)
#define GPIO_PUPDR_PUPD12 GPIO_PUPDR_PUPD12_Msk
#define GPIO_PUPDR_PUPD12_NONE (0x0U << GPIO_PUPDR_PUPD12_Pos)
#define GPIO_PUPDR_PUPD12_PULL_UP (0x1U << GPIO_PUPDR_PUPD12_Pos)
#define GPIO_PUPDR_PUPD12_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD12_Pos)

#define GPIO_PUPDR_PUPD11_Pos 22U
#define GPIO_PUPDR_PUPD11_Msk (0x3U << GPIO_PUPDR_PUPD11_Pos)
#define GPIO_PUPDR_PUPD11 GPIO_PUPDR_PUPD11_Msk
#define GPIO_PUPDR_PUPD11_NONE (0x0U << GPIO_PUPDR_PUPD11_Pos)
#define GPIO_PUPDR_PUPD11_PULL_UP (0x1U << GPIO_PUPDR_PUPD11_Pos)
#define GPIO_PUPDR_PUPD11_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD11_Pos)

#define GPIO_PUPDR_PUPD10_Pos 20U
#define GPIO_PUPDR_PUPD10_Msk (0x3U << GPIO_PUPDR_PUPD10_Pos)
#define GPIO_PUPDR_PUPD10 GPIO_PUPDR_PUPD10_Msk
#define GPIO_PUPDR_PUPD10_NONE (0x0U << GPIO_PUPDR_PUPD10_Pos)
#define GPIO_PUPDR_PUPD10_PULL_UP (0x1U << GPIO_PUPDR_PUPD10_Pos)
#define GPIO_PUPDR_PUPD10_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD10_Pos)

#define GPIO_PUPDR_PUPD9_Pos 18U
#define GPIO_PUPDR_PUPD9_Msk (0x3U << GPIO_PUPDR_PUPD9_Pos)
#define GPIO_PUPDR_PUPD9 GPIO_PUPDR_PUPD9_Msk
#define GPIO_PUPDR_PUPD9_NONE (0x0U << GPIO_PUPDR_PUPD9_Pos)
#define GPIO_PUPDR_PUPD9_PULL_UP (0x1U << GPIO_PUPDR_PUPD9_Pos)
#define GPIO_PUPDR_PUPD9_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD9_Pos)

#define GPIO_PUPDR_PUPD8_Pos 16U
#define GPIO_PUPDR_PUPD8_Msk (0x3U << GPIO_PUPDR_PUPD8_Pos)
#define GPIO_PUPDR_PUPD8 GPIO_PUPDR_PUPD8_Msk
#define GPIO_PUPDR_PUPD8_NONE (0x0U << GPIO_PUPDR_PUPD8_Pos)
#define GPIO_PUPDR_PUPD8_PULL_UP (0x1U << GPIO_PUPDR_PUPD8_Pos)
#define GPIO_PUPDR_PUPD8_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD8_Pos)

#define GPIO_PUPDR_PUPD7_Pos 14U
#define GPIO_PUPDR_PUPD7_Msk (0x3U << GPIO_PUPDR_PUPD7_Pos)
#define GPIO_PUPDR_PUPD7 GPIO_PUPDR_PUPD7_Msk
#define GPIO_PUPDR_PUPD7_NONE (0x0U << GPIO_PUPDR_PUPD7_Pos)
#define GPIO_PUPDR_PUPD7_PULL_UP (0x1U << GPIO_PUPDR_PUPD7_Pos)
#define GPIO_PUPDR_PUPD7_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD7_Pos)

#define GPIO_PUPDR_PUPD6_Pos 12U
#define GPIO_PUPDR_PUPD6_Msk (0x3U << GPIO_PUPDR_PUPD6_Pos)
#define GPIO_PUPDR_PUPD6 GPIO_PUPDR_PUPD6_Msk
#define GPIO_PUPDR_PUPD6_NONE (0x0U << GPIO_PUPDR_PUPD6_Pos)
#define GPIO_PUPDR_PUPD6_PULL_UP (0x1U << GPIO_PUPDR_PUPD6_Pos)
#define GPIO_PUPDR_PUPD6_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD6_Pos)

#define GPIO_PUPDR_PUPD5_Pos 10U
#define GPIO_PUPDR_PUPD5_Msk (0x3U << GPIO_PUPDR_PUPD5_Pos)
#define GPIO_PUPDR_PUPD5 GPIO_PUPDR_PUPD5_Msk
#define GPIO_PUPDR_PUPD5_NONE (0x0U << GPIO_PUPDR_PUPD5_Pos)
#define GPIO_PUPDR_PUPD5_PULL_UP (0x1U << GPIO_PUPDR_PUPD5_Pos)
#define GPIO_PUPDR_PUPD5_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD5_Pos)

#define GPIO_PUPDR_PUPD4_Pos 8U
#define GPIO_PUPDR_PUPD4_Msk (0x3U << GPIO_PUPDR_PUPD4_Pos)
#define GPIO_PUPDR_PUPD4 GPIO_PUPDR_PUPD4_Msk
#define GPIO_PUPDR_PUPD4_NONE (0x0U << GPIO_PUPDR_PUPD4_Pos)
#define GPIO_PUPDR_PUPD4_PULL_UP (0x1U << GPIO_PUPDR_PUPD4_Pos)
#define GPIO_PUPDR_PUPD4_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD4_Pos)

#define GPIO_PUPDR_PUPD3_Pos 6U
#define GPIO_PUPDR_PUPD3_Msk (0x3U << GPIO_PUPDR_PUPD3_Pos)
#define GPIO_PUPDR_PUPD3 GPIO_PUPDR_PUPD3_Msk
#define GPIO_PUPDR_PUPD3_NONE (0x0U << GPIO_PUPDR_PUPD3_Pos)
#define GPIO_PUPDR_PUPD3_PULL_UP (0x1U << GPIO_PUPDR_PUPD3_Pos)
#define GPIO_PUPDR_PUPD3_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD3_Pos)

#define GPIO_PUPDR_PUPD2_Pos 4U
#define GPIO_PUPDR_PUPD2_Msk (0x3U << GPIO_PUPDR_PUPD2_Pos)
#define GPIO_PUPDR_PUPD2 GPIO_PUPDR_PUPD2_Msk
#define GPIO_PUPDR_PUPD2_NONE (0x0U << GPIO_PUPDR_PUPD2_Pos)
#define GPIO_PUPDR_PUPD2_PULL_UP (0x1U << GPIO_PUPDR_PUPD2_Pos)
#define GPIO_PUPDR_PUPD2_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD2_Pos)

#define GPIO_PUPDR_PUPD1_Pos 2U
#define GPIO_PUPDR_PUPD1_Msk (0x3U << GPIO_PUPDR_PUPD1_Pos)
#define GPIO_PUPDR_PUPD1 GPIO_PUPDR_PUPD1_Msk
#define GPIO_PUPDR_PUPD1_NONE (0x0U << GPIO_PUPDR_PUPD1_Pos)
#define GPIO_PUPDR_PUPD1_PULL_UP (0x1U << GPIO_PUPDR_PUPD1_Pos)
#define GPIO_PUPDR_PUPD1_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD1_Pos)

#define GPIO_PUPDR_PUPD0_Pos 0U
#define GPIO_PUPDR_PUPD0_Msk (0x3U << GPIO_PUPDR_PUPD0_Pos)
#define GPIO_PUPDR_PUPD0 GPIO_PUPDR_PUPD0_Msk
#define GPIO_PUPDR_PUPD0_NONE (0x0U << GPIO_PUPDR_PUPD0_Pos)
#define GPIO_PUPDR_PUPD0_PULL_UP (0x1U << GPIO_PUPDR_PUPD0_Pos)
#define GPIO_PUPDR_PUPD0_PULL_DOWN (0x2U << GPIO_PUPDR_PUPD0_Pos)

/*IDR register*/
#define GPIO_IDR_ID15_Pos 15U
#define GPIO_IDR_ID15_Msk (0x1U << GPIO_IDR_ID15_Pos)
#define GPIO_IDR_ID15 GPIO_IDR_ID15_Msk

#define GPIO_IDR_ID14_Pos 14U
#define GPIO_IDR_ID14_Msk (0x1U << GPIO_IDR_ID14_Pos)
#define GPIO_IDR_ID14 GPIO_IDR_ID14_Msk

#define GPIO_IDR_ID13_Pos 13U
#define GPIO_IDR_ID13_Msk (0x1U << GPIO_IDR_ID13_Pos)
#define GPIO_IDR_ID13 GPIO_IDR_ID13_Msk

#define GPIO_IDR_ID12_Pos 12U
#define GPIO_IDR_ID12_Msk (0x1U << GPIO_IDR_ID12_Pos)
#define GPIO_IDR_ID12 GPIO_IDR_ID12_Msk

#define GPIO_IDR_ID11_Pos 11U
#define GPIO_IDR_ID11_Msk (0x1U << GPIO_IDR_ID11_Pos)
#define GPIO_IDR_ID11 GPIO_IDR_ID11_Msk

#define GPIO_IDR_ID10_Pos 10U
#define GPIO_IDR_ID10_Msk (0x1U << GPIO_IDR_ID10_Pos)
#define GPIO_IDR_ID10 GPIO_IDR_ID10_Msk

#define GPIO_IDR_ID9_Pos 9U
#define GPIO_IDR_ID9_Msk (0x1U << GPIO_IDR_ID9_Pos)
#define GPIO_IDR_ID9 GPIO_IDR_ID9_Msk

#define GPIO_IDR_ID8_Pos 8U
#define GPIO_IDR_ID8_Msk (0x1U << GPIO_IDR_ID8_Pos)
#define GPIO_IDR_ID8 GPIO_IDR_ID8_Msk

#define GPIO_IDR_ID7_Pos 7U
#define GPIO_IDR_ID7_Msk (0x1U << GPIO_IDR_ID7_Pos)
#define GPIO_IDR_ID7 GPIO_IDR_ID7_Msk

#define GPIO_IDR_ID6_Pos 6U
#define GPIO_IDR_ID6_Msk (0x1U << GPIO_IDR_ID6_Pos)
#define GPIO_IDR_ID6 GPIO_IDR_ID6_Msk

#define GPIO_IDR_ID5_Pos 5U
#define GPIO_IDR_ID5_Msk (0x1U << GPIO_IDR_ID5_Pos)
#define GPIO_IDR_ID5 GPIO_IDR_ID5_Msk

#define GPIO_IDR_ID4_Pos 4U
#define GPIO_IDR_ID4_Msk (0x1U << GPIO_IDR_ID4_Pos)
#define GPIO_IDR_ID4 GPIO_IDR_ID4_Msk

#define GPIO_IDR_ID3_Pos 3U
#define GPIO_IDR_ID3_Msk (0x1U << GPIO_IDR_ID3_Pos)
#define GPIO_IDR_ID3 GPIO_IDR_ID3_Msk

#define GPIO_IDR_ID2_Pos 2U
#define GPIO_IDR_ID2_Msk (0x1U << GPIO_IDR_ID2_Pos)
#define GPIO_IDR_ID2 GPIO_IDR_ID2_Msk

#define GPIO_IDR_ID1_Pos 1U
#define GPIO_IDR_ID1_Msk (0x1U << GPIO_IDR_ID1_Pos)
#define GPIO_IDR_ID1 GPIO_IDR_ID1_Msk

#define GPIO_IDR_ID0_Pos 0U
#define GPIO_IDR_ID0_Msk (0x1U << GPIO_IDR_ID0_Pos)
#define GPIO_IDR_ID0 GPIO_IDR_ID0_Msk

/*ODR register*/
#define GPIO_ODR_OD15_Pos 15U
#define GPIO_ODR_OD15_Msk (0x1U << GPIO_ODR_OD15_Pos)
#define GPIO_ODR_OD15 GPIO_ODR_OD15_Msk

#define GPIO_ODR_OD14_Pos 14U
#define GPIO_ODR_OD14_Msk (0x1U << GPIO_ODR_OD14_Pos)
#define GPIO_ODR_OD14 GPIO_ODR_OD14_Msk

#define GPIO_ODR_OD13_Pos 13U
#define GPIO_ODR_OD13_Msk (0x1U << GPIO_ODR_OD13_Pos)
#define GPIO_ODR_OD13 GPIO_ODR_OD13_Msk

#define GPIO_ODR_OD12_Pos 12U
#define GPIO_ODR_OD12_Msk (0x1U << GPIO_ODR_OD12_Pos)
#define GPIO_ODR_OD12 GPIO_ODR_OD12_Msk

#define GPIO_ODR_OD11_Pos 11U
#define GPIO_ODR_OD11_Msk (0x1U << GPIO_ODR_OD11_Pos)
#define GPIO_ODR_OD11 GPIO_ODR_OD11_Msk

#define GPIO_ODR_OD10_Pos 10U
#define GPIO_ODR_OD10_Msk (0x1U << GPIO_ODR_OD10_Pos)
#define GPIO_ODR_OD10 GPIO_ODR_OD10_Msk

#define GPIO_ODR_OD9_Pos 9U
#define GPIO_ODR_OD9_Msk (0x1U << GPIO_ODR_OD9_Pos)
#define GPIO_ODR_OD9 GPIO_ODR_OD9_Msk

#define GPIO_ODR_OD8_Pos 8U
#define GPIO_ODR_OD8_Msk (0x1U << GPIO_ODR_OD8_Pos)
#define GPIO_ODR_OD8 GPIO_ODR_OD8_Msk

#define GPIO_ODR_OD7_Pos 7U
#define GPIO_ODR_OD7_Msk (0x1U << GPIO_ODR_OD7_Pos)
#define GPIO_ODR_OD7 GPIO_ODR_OD7_Msk

#define GPIO_ODR_OD6_Pos 6U
#define GPIO_ODR_OD6_Msk (0x1U << GPIO_ODR_OD6_Pos)
#define GPIO_ODR_OD6 GPIO_ODR_OD6_Msk

#define GPIO_ODR_OD5_Pos 5U
#define GPIO_ODR_OD5_Msk (0x1U << GPIO_ODR_OD5_Pos)
#define GPIO_ODR_OD5 GPIO_ODR_OD5_Msk

#define GPIO_ODR_OD4_Pos 4U
#define GPIO_ODR_OD4_Msk (0x1U << GPIO_ODR_OD4_Pos)
#define GPIO_ODR_OD4 GPIO_ODR_OD4_Msk

#define GPIO_ODR_OD3_Pos 3U
#define GPIO_ODR_OD3_Msk (0x1U << GPIO_ODR_OD3_Pos)
#define GPIO_ODR_OD3 GPIO_ODR_OD3_Msk

#define GPIO_ODR_OD2_Pos 2U
#define GPIO_ODR_OD2_Msk (0x1U << GPIO_ODR_OD2_Pos)
#define GPIO_ODR_OD2 GPIO_ODR_OD2_Msk

#define GPIO_ODR_OD1_Pos 1U
#define GPIO_ODR_OD1_Msk (0x1U << GPIO_ODR_OD1_Pos)
#define GPIO_ODR_OD1 GPIO_ODR_OD1_Msk

#define GPIO_ODR_OD0_Pos 0U
#define GPIO_ODR_OD0_Msk (0x1U << GPIO_ODR_OD0_Pos)
#define GPIO_ODR_OD0 GPIO_ODR_OD0_Msk

/*BSRR register*/
#define GPIO_BSRR_BR15_Pos 31U
#define GPIO_BSRR_BR15_Msk (0x1U << GPIO_BSRR_BR15_Pos)
#define GPIO_BSRR_BR15 GPIO_BSRR_BR15_Msk

#define GPIO_BSRR_BR14_Pos 30U
#define GPIO_BSRR_BR14_Msk (0x1U << GPIO_BSRR_BR14_Pos)
#define GPIO_BSRR_BR14 GPIO_BSRR_BR14_Msk

#define GPIO_BSRR_BR13_Pos 29U
#define GPIO_BSRR_BR13_Msk (0x1U << GPIO_BSRR_BR13_Pos)
#define GPIO_BSRR_BR13 GPIO_BSRR_BR13_Msk

#define GPIO_BSRR_BR12_Pos 28U
#define GPIO_BSRR_BR12_Msk (0x1U << GPIO_BSRR_BR12_Pos)
#define GPIO_BSRR_BR12 GPIO_BSRR_BR12_Msk

#define GPIO_BSRR_BR11_Pos 27U
#define GPIO_BSRR_BR11_Msk (0x1U << GPIO_BSRR_BR11_Pos)
#define GPIO_BSRR_BR11 GPIO_BSRR_BR11_Msk

#define GPIO_BSRR_BR10_Pos 26U
#define GPIO_BSRR_BR10_Msk (0x1U << GPIO_BSRR_BR10_Pos)
#define GPIO_BSRR_BR10 GPIO_BSRR_BR10_Msk

#define GPIO_BSRR_BR9_Pos 25U
#define GPIO_BSRR_BR9_Msk (0x1U << GPIO_BSRR_BR9_Pos)
#define GPIO_BSRR_BR9 GPIO_BSRR_BR9_Msk

#define GPIO_BSRR_BR8_Pos 24U
#define GPIO_BSRR_BR8_Msk (0x1U << GPIO_BSRR_BR8_Pos)
#define GPIO_BSRR_BR8 GPIO_BSRR_BR8_Msk

#define GPIO_BSRR_BR7_Pos 23U
#define GPIO_BSRR_BR7_Msk (0x1U << GPIO_BSRR_BR7_Pos)
#define GPIO_BSRR_BR7 GPIO_BSRR_BR7_Msk

#define GPIO_BSRR_BR6_Pos 22U
#define GPIO_BSRR_BR6_Msk (0x1U << GPIO_BSRR_BR6_Pos)
#define GPIO_BSRR_BR6 GPIO_BSRR_BR6_Msk

#define GPIO_BSRR_BR5_Pos 21U
#define GPIO_BSRR_BR5_Msk (0x1U << GPIO_BSRR_BR5_Pos)
#define GPIO_BSRR_BR5 GPIO_BSRR_BR5_Msk

#define GPIO_BSRR_BR4_Pos 20U
#define GPIO_BSRR_BR4_Msk (0x1U << GPIO_BSRR_BR4_Pos)
#define GPIO_BSRR_BR4 GPIO_BSRR_BR4_Msk

#define GPIO_BSRR_BR3_Pos 19U
#define GPIO_BSRR_BR3_Msk (0x1U << GPIO_BSRR_BR3_Pos)
#define GPIO_BSRR_BR3 GPIO_BSRR_BR3_Msk

#define GPIO_BSRR_BR2_Pos 18U
#define GPIO_BSRR_BR2_Msk (0x1U << GPIO_BSRR_BR2_Pos)
#define GPIO_BSRR_BR2 GPIO_BSRR_BR2_Msk

#define GPIO_BSRR_BR1_Pos 17U
#define GPIO_BSRR_BR1_Msk (0x1U << GPIO_BSRR_BR1_Pos)
#define GPIO_BSRR_BR1 GPIO_BSRR_BR1_Msk

#define GPIO_BSRR_BR0_Pos 16U
#define GPIO_BSRR_BR0_Msk (0x1U << GPIO_BSRR_BR0_Pos)
#define GPIO_BSRR_BR0 GPIO_BSRR_BR0_Msk

#define GPIO_BSRR_BS15_Pos 15U
#define GPIO_BSRR_BS15_Msk (0x1U << GPIO_BSRR_BS15_Pos)
#define GPIO_BSRR_BS15 GPIO_BSRR_BS15_Msk

#define GPIO_BSRR_BS14_Pos 14U
#define GPIO_BSRR_BS14_Msk (0x1U << GPIO_BSRR_BS14_Pos)
#define GPIO_BSRR_BS14 GPIO_BSRR_BS14_Msk

#define GPIO_BSRR_BS13_Pos 13U
#define GPIO_BSRR_BS13_Msk (0x1U << GPIO_BSRR_BS13_Pos)
#define GPIO_BSRR_BS13 GPIO_BSRR_BS13_Msk

#define GPIO_BSRR_BS12_Pos 12U
#define GPIO_BSRR_BS12_Msk (0x1U << GPIO_BSRR_BS12_Pos)
#define GPIO_BSRR_BS12 GPIO_BSRR_BS12_Msk

#define GPIO_BSRR_BS11_Pos 11U
#define GPIO_BSRR_BS11_Msk (0x1U << GPIO_BSRR_BS11_Pos)
#define GPIO_BSRR_BS11 GPIO_BSRR_BS11_Msk

#define GPIO_BSRR_BS10_Pos 10U
#define GPIO_BSRR_BS10_Msk (0x1U << GPIO_BSRR_BS10_Pos)
#define GPIO_BSRR_BS10 GPIO_BSRR_BS10_Msk

#define GPIO_BSRR_BS9_Pos 9U
#define GPIO_BSRR_BS9_Msk (0x1U << GPIO_BSRR_BS9_Pos)
#define GPIO_BSRR_BS9 GPIO_BSRR_BS9_Msk

#define GPIO_BSRR_BS8_Pos 8U
#define GPIO_BSRR_BS8_Msk (0x1U << GPIO_BSRR_BS8_Pos)
#define GPIO_BSRR_BS8 GPIO_BSRR_BS8_Msk

#define GPIO_BSRR_BS7_Pos 7U
#define GPIO_BSRR_BS7_Msk (0x1U << GPIO_BSRR_BS7_Pos)
#define GPIO_BSRR_BS7 GPIO_BSRR_BS7_Msk

#define GPIO_BSRR_BS6_Pos 6U
#define GPIO_BSRR_BS6_Msk (0x1U << GPIO_BSRR_BS6_Pos)
#define GPIO_BSRR_BS6 GPIO_BSRR_BS6_Msk

#define GPIO_BSRR_BS5_Pos 5U
#define GPIO_BSRR_BS5_Msk (0x1U << GPIO_BSRR_BS5_Pos)
#define GPIO_BSRR_BS5 GPIO_BSRR_BS5_Msk

#define GPIO_BSRR_BS4_Pos 4U
#define GPIO_BSRR_BS4_Msk (0x1U << GPIO_BSRR_BS4_Pos)
#define GPIO_BSRR_BS4 GPIO_BSRR_BS4_Msk

#define GPIO_BSRR_BS3_Pos 3U
#define GPIO_BSRR_BS3_Msk (0x1U << GPIO_BSRR_BS3_Pos)
#define GPIO_BSRR_BS3 GPIO_BSRR_BS3_Msk

#define GPIO_BSRR_BS2_Pos 2U
#define GPIO_BSRR_BS2_Msk (0x1U << GPIO_BSRR_BS2_Pos)
#define GPIO_BSRR_BS2 GPIO_BSRR_BS2_Msk

#define GPIO_BSRR_BS1_Pos 1U
#define GPIO_BSRR_BS1_Msk (0x1U << GPIO_BSRR_BS1_Pos)
#define GPIO_BSRR_BS1 GPIO_BSRR_BS1_Msk

#define GPIO_BSRR_BS0_Pos 0U
#define GPIO_BSRR_BS0_Msk (0x1U << GPIO_BSRR_BS0_Pos)
#define GPIO_BSRR_BS0 GPIO_BSRR_BS0_Msk

/*LCKR register*/
#define GPIO_LCKR_LCKK_Pos 16U
#define GPIO_LCKR_LCKK_Msk (0x1U << GPIO_LCKR_LCKK_Pos)
#define GPIO_LCKR_LCKK GPIO_LCKR_LCKK_Msk

#define GPIO_LCKR_LCK15_Pos 15U
#define GPIO_LCKR_LCK15_Msk (0x1U << GPIO_LCKR_LCK15_Pos)
#define GPIO_LCKR_LCK15 GPIO_LCKR_LCK15_Msk

#define GPIO_LCKR_LCK14_Pos 14U
#define GPIO_LCKR_LCK14_Msk (0x1U << GPIO_LCKR_LCK14_Pos)
#define GPIO_LCKR_LCK14 GPIO_LCKR_LCK14_Msk

#define GPIO_LCKR_LCK13_Pos 13U
#define GPIO_LCKR_LCK13_Msk (0x1U << GPIO_LCKR_LCK13_Pos)
#define GPIO_LCKR_LCK13 GPIO_LCKR_LCK13_Msk

#define GPIO_LCKR_LCK12_Pos 12U
#define GPIO_LCKR_LCK12_Msk (0x1U << GPIO_LCKR_LCK12_Pos)
#define GPIO_LCKR_LCK12 GPIO_LCKR_LCK12_Msk

#define GPIO_LCKR_LCK11_Pos 11U
#define GPIO_LCKR_LCK11_Msk (0x1U << GPIO_LCKR_LCK11_Pos)
#define GPIO_LCKR_LCK11 GPIO_LCKR_LCK11_Msk

#define GPIO_LCKR_LCK10_Pos 10U
#define GPIO_LCKR_LCK10_Msk (0x1U << GPIO_LCKR_LCK10_Pos)
#define GPIO_LCKR_LCK10 GPIO_LCKR_LCK10_Msk

#define GPIO_LCKR_LCK9_Pos 9U
#define GPIO_LCKR_LCK9_Msk (0x1U << GPIO_LCKR_LCK9_Pos)
#define GPIO_LCKR_LCK9 GPIO_LCKR_LCK9_Msk

#define GPIO_LCKR_LCK8_Pos 8U
#define GPIO_LCKR_LCK8_Msk (0x1U << GPIO_LCKR_LCK8_Pos)
#define GPIO_LCKR_LCK8 GPIO_LCKR_LCK8_Msk

#define GPIO_LCKR_LCK7_Pos 7U
#define GPIO_LCKR_LCK7_Msk (0x1U << GPIO_LCKR_LCK7_Pos)
#define GPIO_LCKR_LCK7 GPIO_LCKR_LCK7_Msk

#define GPIO_LCKR_LCK6_Pos 6U
#define GPIO_LCKR_LCK6_Msk (0x1U << GPIO_LCKR_LCK6_Pos)
#define GPIO_LCKR_LCK6 GPIO_LCKR_LCK6_Msk

#define GPIO_LCKR_LCK5_Pos 5U
#define GPIO_LCKR_LCK5_Msk (0x1U << GPIO_LCKR_LCK5_Pos)
#define GPIO_LCKR_LCK5 GPIO_LCKR_LCK5_Msk

#define GPIO_LCKR_LCK4_Pos 4U
#define GPIO_LCKR_LCK4_Msk (0x1U << GPIO_LCKR_LCK4_Pos)
#define GPIO_LCKR_LCK4 GPIO_LCKR_LCK4_Msk

#define GPIO_LCKR_LCK3_Pos 3U
#define GPIO_LCKR_LCK3_Msk (0x1U << GPIO_LCKR_LCK3_Pos)
#define GPIO_LCKR_LCK3 GPIO_LCKR_LCK3_Msk

#define GPIO_LCKR_LCK2_Pos 2U
#define GPIO_LCKR_LCK2_Msk (0x1U << GPIO_LCKR_LCK2_Pos)
#define GPIO_LCKR_LCK2 GPIO_LCKR_LCK2_Msk

#define GPIO_LCKR_LCK1_Pos 1U
#define GPIO_LCKR_LCK1_Msk (0x1U << GPIO_LCKR_LCK1_Pos)
#define GPIO_LCKR_LCK1 GPIO_LCKR_LCK1_Msk

#define GPIO_LCKR_LCK0_Pos 0U
#define GPIO_LCKR_LCK0_Msk (0x1U << GPIO_LCKR_LCK0_Pos)
#define GPIO_LCKR_LCK0 GPIO_LCKR_LCK0_Msk

/*AFRL register*/
#define GPIO_AFRL_AFSEL7_Pos 28U
#define GPIO_AFRL_AFSEL7_Msk (0xFU << GPIO_AFRL_AFSEL7_Pos)
#define GPIO_AFRL_AFSEL7 GPIO_AFRL_AFSEL7_Msk

#define GPIO_AFRL_AFSEL6_Pos 24U
#define GPIO_AFRL_AFSEL6_Msk (0xFU << GPIO_AFRL_AFSEL6_Pos)
#define GPIO_AFRL_AFSEL6 GPIO_AFRL_AFSEL6_Msk

#define GPIO_AFRL_AFSEL5_Pos 20U
#define GPIO_AFRL_AFSEL5_Msk (0xFU << GPIO_AFRL_AFSEL5_Pos)
#define GPIO_AFRL_AFSEL5 GPIO_AFRL_AFSEL5_Msk

#define GPIO_AFRL_AFSEL4_Pos 16U
#define GPIO_AFRL_AFSEL4_Msk (0xFU << GPIO_AFRL_AFSEL4_Pos)
#define GPIO_AFRL_AFSEL4 GPIO_AFRL_AFSEL4_Msk

#define GPIO_AFRL_AFSEL3_Pos 12U
#define GPIO_AFRL_AFSEL3_Msk (0xFU << GPIO_AFRL_AFSEL3_Pos)
#define GPIO_AFRL_AFSEL3 GPIO_AFRL_AFSEL3_Msk

#define GPIO_AFRL_AFSEL2_Pos 8U
#define GPIO_AFRL_AFSEL2_Msk (0xFU << GPIO_AFRL_AFSEL2_Pos)
#define GPIO_AFRL_AFSEL2 GPIO_AFRL_AFSEL2_Msk

#define GPIO_AFRL_AFSEL1_Pos 4U
#define GPIO_AFRL_AFSEL1_Msk (0xFU << GPIO_AFRL_AFSEL1_Pos)
#define GPIO_AFRL_AFSEL1 GPIO_AFRL_AFSEL1_Msk

#define GPIO_AFRL_AFSEL0_Pos 0U
#define GPIO_AFRL_AFSEL0_Msk (0xFU << GPIO_AFRL_AFSEL0_Pos)
#define GPIO_AFRL_AFSEL0 GPIO_AFRL_AFSEL0_Msk

/*AFRH register*/
#define GPIO_AFRH_AFSEL15_Pos 28U
#define GPIO_AFRH_AFSEL15_Msk (0xFU << GPIO_AFRH_AFSEL15_Pos)
#define GPIO_AFRH_AFSEL15 GPIO_AFRH_AFSEL15_Msk

#define GPIO_AFRH_AFSEL14_Pos 24U
#define GPIO_AFRH_AFSEL14_Msk (0xFU << GPIO_AFRH_AFSEL14_Pos)
#define GPIO_AFRH_AFSEL14 GPIO_AFRH_AFSEL14_Msk

#define GPIO_AFRH_AFSEL13_Pos 20U
#define GPIO_AFRH_AFSEL13_Msk (0xFU << GPIO_AFRH_AFSEL13_Pos)
#define GPIO_AFRH_AFSEL13 GPIO_AFRH_AFSEL13_Msk

#define GPIO_AFRH_AFSEL12_Pos 16U
#define GPIO_AFRH_AFSEL12_Msk (0xFU << GPIO_AFRH_AFSEL12_Pos)
#define GPIO_AFRH_AFSEL12 GPIO_AFRH_AFSEL12_Msk

#define GPIO_AFRH_AFSEL11_Pos 12U
#define GPIO_AFRH_AFSEL11_Msk (0xFU << GPIO_AFRH_AFSEL11_Pos)
#define GPIO_AFRH_AFSEL11 GPIO_AFRH_AFSEL11_Msk

#define GPIO_AFRH_AFSEL10_Pos 8U
#define GPIO_AFRH_AFSEL10_Msk (0xFU << GPIO_AFRH_AFSEL10_Pos)
#define GPIO_AFRH_AFSEL10 GPIO_AFRH_AFSEL10_Msk

#define GPIO_AFRH_AFSEL9_Pos 4U
#define GPIO_AFRH_AFSEL9_Msk (0xFU << GPIO_AFRH_AFSEL9_Pos)
#define GPIO_AFRH_AFSEL9 GPIO_AFRH_AFSEL9_Msk

#define GPIO_AFRH_AFSEL8_Pos 0U
#define GPIO_AFRH_AFSEL8_Msk (0xFU << GPIO_AFRH_AFSEL8_Pos)
#define GPIO_AFRH_AFSEL8 GPIO_AFRH_AFSEL8_Msk

/*BRR register*/
#define GPIO_BRR_BR15_Pos 15U
#define GPIO_BRR_BR15_Msk (0x1U << GPIO_BRR_BR15_Pos)
#define GPIO_BRR_BR15 GPIO_BRR_BR15_Msk

#define GPIO_BRR_BR14_Pos 14U
#define GPIO_BRR_BR14_Msk (0x1U << GPIO_BRR_BR14_Pos)
#define GPIO_BRR_BR14 GPIO_BRR_BR14_Msk

#define GPIO_BRR_BR13_Pos 13U
#define GPIO_BRR_BR13_Msk (0x1U << GPIO_BRR_BR13_Pos)
#define GPIO_BRR_BR13 GPIO_BRR_BR13_Msk

#define GPIO_BRR_BR12_Pos 12U
#define GPIO_BRR_BR12_Msk (0x1U << GPIO_BRR_BR12_Pos)
#define GPIO_BRR_BR12 GPIO_BRR_BR12_Msk

#define GPIO_BRR_BR11_Pos 11U
#define GPIO_BRR_BR11_Msk (0x1U << GPIO_BRR_BR11_Pos)
#define GPIO_BRR_BR11 GPIO_BRR_BR11_Msk

#define GPIO_BRR_BR10_Pos 10U
#define GPIO_BRR_BR10_Msk (0x1U << GPIO_BRR_BR10_Pos)
#define GPIO_BRR_BR10 GPIO_BRR_BR10_Msk

#define GPIO_BRR_BR9_Pos 9U
#define GPIO_BRR_BR9_Msk (0x1U << GPIO_BRR_BR9_Pos)
#define GPIO_BRR_BR9 GPIO_BRR_BR9_Msk

#define GPIO_BRR_BR8_Pos 8U
#define GPIO_BRR_BR8_Msk (0x1U << GPIO_BRR_BR8_Pos)
#define GPIO_BRR_BR8 GPIO_BRR_BR8_Msk

#define GPIO_BRR_BR7_Pos 7U
#define GPIO_BRR_BR7_Msk (0x1U << GPIO_BRR_BR7_Pos)
#define GPIO_BRR_BR7 GPIO_BRR_BR7_Msk

#define GPIO_BRR_BR6_Pos 6U
#define GPIO_BRR_BR6_Msk (0x1U << GPIO_BRR_BR6_Pos)
#define GPIO_BRR_BR6 GPIO_BRR_BR6_Msk

#define GPIO_BRR_BR5_Pos 5U
#define GPIO_BRR_BR5_Msk (0x1U << GPIO_BRR_BR5_Pos)
#define GPIO_BRR_BR5 GPIO_BRR_BR5_Msk

#define GPIO_BRR_BR4_Pos 4U
#define GPIO_BRR_BR4_Msk (0x1U << GPIO_BRR_BR4_Pos)
#define GPIO_BRR_BR4 GPIO_BRR_BR4_Msk

#define GPIO_BRR_BR3_Pos 3U
#define GPIO_BRR_BR3_Msk (0x1U << GPIO_BRR_BR3_Pos)
#define GPIO_BRR_BR3 GPIO_BRR_BR3_Msk

#define GPIO_BRR_BR2_Pos 2U
#define GPIO_BRR_BR2_Msk (0x1U << GPIO_BRR_BR2_Pos)
#define GPIO_BRR_BR2 GPIO_BRR_BR2_Msk

#define GPIO_BRR_BR1_Pos 1U
#define GPIO_BRR_BR1_Msk (0x1U << GPIO_BRR_BR1_Pos)
#define GPIO_BRR_BR1 GPIO_BRR_BR1_Msk

#define GPIO_BRR_BR0_Pos 0U
#define GPIO_BRR_BR0_Msk (0x1U << GPIO_BRR_BR0_Pos)
#define GPIO_BRR_BR0 GPIO_BRR_BR0_Msk

/* ========================================================================= */
/* ============                      SYSCFG                     ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t MEMRMP; /*!< SYSCFG Memory Remap Register. */
	__IOM uint32_t CFGR1; /*!< SYSCFG Configuration Register 1. */
	__IOM uint32_t EXTICR1; /*!< SYSCFG External Interrupt Configuration Register 1. */
	__IOM uint32_t EXTICR2; /*!< SYSCFG External Interrupt Configuration Register 2. */
	__IOM uint32_t EXTICR3; /*!< SYSCFG External Interrupt Configuration Register 3. */
	__IOM uint32_t EXTICR4; /*!< SYSCFG External Interrupt Configuration Register 4. */
	__IOM uint32_t SCSR; /*!< SYSCFG SRAM2 Control and Status Register. */
	__IOM uint32_t CFGR2; /*!< SYSCFG Configuration Register 2. */
	__OM uint32_t SWPR; /*!< SYSCFG SRAM2 Write Protection Register. */
	__OM uint32_t SKR; /*!< SYSCFG SRAM2 Key Register. */
}STM32L4xx_SYSCFG_TypeDef;

/*MEMRMP register*/
#define SYSCFG_MEMRMP_MEM_MODE_Pos 0U
#define SYSCFG_MEMRMP_MEM_MODE_Msk (0x7U << SYSCFG_MEMRMP_MEM_MODE_Pos)
#define SYSCFG_MEMRMP_MEM_MODE SYSCFG_MEMRMP_MEM_MODE_Msk
#define SYSCFG_MEMRMP_MEM_MODE_MAIN_FLASH (0x0U << SYSCFG_MEMRMP_MEM_MODE_Pos)
#define SYSCFG_MEMRMP_MEM_MODE_SYSTEM_FLASH (0x1U << SYSCFG_MEMRMP_MEM_MODE_Pos)
#define SYSCFG_MEMRMP_MEM_MODE_SRAM1 (0x3U << SYSCFG_MEMRMP_MEM_MODE_Pos)
#define SYSCFG_MEMRMP_MEM_MODE_QUADSPI (0x6U << SYSCFG_MEMRMP_MEM_MODE_Pos)

/*CFGR1 register*/
#define SYSCFG_CFGR1_FPU_IE_5_Pos 31U
#define SYSCFG_CFGR1_FPU_IE_5_Msk (0x1UL << SYSCFG_CFGR1_FPU_IE_5_Pos)
#define SYSCFG_CFGR1_FPU_IE_5 SYSCFG_CFGR1_FPU_IE_5_Msk

#define SYSCFG_CFGR1_FPU_IE_4_Pos 30U
#define SYSCFG_CFGR1_FPU_IE_4_Msk (0x1UL << SYSCFG_CFGR1_FPU_IE_4_Pos)
#define SYSCFG_CFGR1_FPU_IE_4 SYSCFG_CFGR1_FPU_IE_5_Msk

#define SYSCFG_CFGR1_FPU_IE_3_Pos 29U
#define SYSCFG_CFGR1_FPU_IE_3_Msk (0x1UL << SYSCFG_CFGR1_FPU_IE_3_Pos)
#define SYSCFG_CFGR1_FPU_IE_3 SYSCFG_CFGR1_FPU_IE_3_Msk

#define SYSCFG_CFGR1_FPU_IE_2_Pos 28U
#define SYSCFG_CFGR1_FPU_IE_2_Msk (0x1UL << SYSCFG_CFGR1_FPU_IE_2_Pos)
#define SYSCFG_CFGR1_FPU_IE_2 SYSCFG_CFGR1_FPU_IE_2_Msk

#define SYSCFG_CFGR1_FPU_IE_1_Pos 27U
#define SYSCFG_CFGR1_FPU_IE_1_Msk (0x1UL << SYSCFG_CFGR1_FPU_IE_1_Pos)
#define SYSCFG_CFGR1_FPU_IE_1 SYSCFG_CFGR1_FPU_IE_1_Msk

#define SYSCFG_CFGR1_FPU_IE_0_Pos 26U
#define SYSCFG_CFGR1_FPU_IE_0_Msk (0x1UL << SYSCFG_CFGR1_FPU_IE_0_Pos)
#define SYSCFG_CFGR1_FPU_IE_0 SYSCFG_CFGR1_FPU_IE_0_Msk

#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define SYSCFG_CFGR1_I2C4_FMP_Pos 23U
#define SYSCFG_CFGR1_I2C4_FMP_Msk (0x1UL << SYSCFG_CFGR1_I2C4_FMP_Pos)
#define SYSCFG_CFGR1_I2C4_FMP SYSCFG_CFGR1_I2C4_FMP_Msk
#endif

#define SYSCFG_CFGR1_I2C3_FMP_Pos 22U
#define SYSCFG_CFGR1_I2C3_FMP_Msk (0x1UL << SYSCFG_CFGR1_I2C3_FMP_Pos)
#define SYSCFG_CFGR1_I2C3_FMP SYSCFG_CFGR1_I2C3_FMP_Msk

#if !defined(STM32L432) && !defined(STM32L442)
#define SYSCFG_CFGR1_I2C2_FMP_Pos 21U
#define SYSCFG_CFGR1_I2C2_FMP_Msk (0x1UL << SYSCFG_CFGR1_I2C2_FMP_Pos)
#define SYSCFG_CFGR1_I2C2_FMP SYSCFG_CFGR1_I2C2_FMP_Msk
#endif

#define SYSCFG_CFGR1_I2C1_FMP_Pos 20U
#define SYSCFG_CFGR1_I2C1_FMP_Msk (0x1UL << SYSCFG_CFGR1_I2C1_FMP_Pos)
#define SYSCFG_CFGR1_I2C1_FMP SYSCFG_CFGR1_I2C1_FMP_Msk

#define SYSCFG_CFGR1_I2C_PB9_FMP_Pos 19U
#define SYSCFG_CFGR1_I2C_PB9_FMP_Msk (0x1UL << SYSCFG_CFGR1_I2C_PB9_FMP_Pos)
#define SYSCFG_CFGR1_I2C_PB9_FMP SYSCFG_CFGR1_I2C_PB9_FMP_Msk

#define SYSCFG_CFGR1_I2C_PB8_FMP_Pos 18U
#define SYSCFG_CFGR1_I2C_PB8_FMP_Msk (0x1UL << SYSCFG_CFGR1_I2C_PB8_FMP_Pos)
#define SYSCFG_CFGR1_I2C_PB8_FMP SYSCFG_CFGR1_I2C_PB8_FMP_Msk

#define SYSCFG_CFGR1_I2C_PB7_FMP_Pos 17U
#define SYSCFG_CFGR1_I2C_PB7_FMP_Msk (0x1UL << SYSCFG_CFGR1_I2C_PB7_FMP_Pos)
#define SYSCFG_CFGR1_I2C_PB7_FMP SYSCFG_CFGR1_I2C_PB7_FMP_Msk

#define SYSCFG_CFGR1_I2C_PB6_FMP_Pos 16U
#define SYSCFG_CFGR1_I2C_PB6_FMP_Msk (0x1UL << SYSCFG_CFGR1_I2C_PB6_FMP_Pos)

#define SYSCFG_CFGR1_BOOSTEN_Pos 8U
#define SYSCFG_CFGR1_BOOSTEN_Msk (0x1UL << SYSCFG_CFGR1_BOOSTEN_Pos)
#define SYSCFG_CFGR1_BOOSTEN SYSCFG_CFGR1_BOOSTEN_Msk

#define SYSCFG_CFGR1_FWDIS_Pos 0U
#define SYSCFG_CFGR1_FWDIS_Msk (0x1UL << SYSCFG_CFGR1_FWDIS_Pos)
#define SYSCFG_CFGR1_FWDIS SYSCFG_CFGR1_FWDIS_Msk

/*EXTICR1 register*/
#define SYSCFG_EXTICR1_EXTI3_Pos 12U
#define SYSCFG_EXTICR1_EXTI3_Msk (0x7UL << SYSCFG_EXTICR1_EXTI3_Pos)
#define SYSCFG_EXTICR1_EXTI3 SYSCFG_EXTICR1_EXTI3_Msk
#define SYSCFG_EXTICR1_EXTI3_PA3 (0x0UL << SYSCFG_EXTICR1_EXTI3_Pos)
#define SYSCFG_EXTICR1_EXTI3_PB3 (0x1UL << SYSCFG_EXTICR1_EXTI3_Pos)
#define SYSCFG_EXTICR1_EXTI3_PC3 (0x2UL << SYSCFG_EXTICR1_EXTI3_Pos)
#define SYSCFG_EXTICR1_EXTI3_PD3 (0x3UL << SYSCFG_EXTICR1_EXTI3_Pos)
#define SYSCFG_EXTICR1_EXTI3_PE3 (0x4UL << SYSCFG_EXTICR1_EXTI3_Pos)
#define SYSCFG_EXTICR1_EXTI3_PH3 (0x7UL << SYSCFG_EXTICR1_EXTI3_Pos)

#define SYSCFG_EXTICR1_EXTI2_Pos 8U
#define SYSCFG_EXTICR1_EXTI2_Msk (0x7UL << SYSCFG_EXTICR1_EXTI2_Pos)
#define SYSCFG_EXTICR1_EXTI2 SYSCFG_EXTICR1_EXTI2_Msk
#define SYSCFG_EXTICR1_EXTI2_PA2 (0x0UL << SYSCFG_EXTICR1_EXTI2_Pos)
#define SYSCFG_EXTICR1_EXTI2_PB2 (0x1UL << SYSCFG_EXTICR1_EXTI2_Pos)
#define SYSCFG_EXTICR1_EXTI2_PC2 (0x2UL << SYSCFG_EXTICR1_EXTI2_Pos)
#define SYSCFG_EXTICR1_EXTI2_PD2 (0x3UL << SYSCFG_EXTICR1_EXTI2_Pos)
#define SYSCFG_EXTICR1_EXTI2_PE2 (0x4UL << SYSCFG_EXTICR1_EXTI2_Pos)

#define SYSCFG_EXTICR1_EXTI1_Pos 4U
#define SYSCFG_EXTICR1_EXTI1_Msk (0x7UL << SYSCFG_EXTICR1_EXTI1_Pos)
#define SYSCFG_EXTICR1_EXTI1 SYSCFG_EXTICR1_EXTI2_Msk
#define SYSCFG_EXTICR1_EXTI1_PA1 (0x0UL << SYSCFG_EXTICR1_EXTI1_Pos)
#define SYSCFG_EXTICR1_EXTI1_PB1 (0x1UL << SYSCFG_EXTICR1_EXTI1_Pos)
#define SYSCFG_EXTICR1_EXTI1_PC1 (0x2UL << SYSCFG_EXTICR1_EXTI1_Pos)
#define SYSCFG_EXTICR1_EXTI1_PD1 (0x3UL << SYSCFG_EXTICR1_EXTI1_Pos)
#define SYSCFG_EXTICR1_EXTI1_PE1 (0x4UL << SYSCFG_EXTICR1_EXTI1_Pos)
#define SYSCFG_EXTICR1_EXTI1_PH1 (0x7UL << SYSCFG_EXTICR1_EXTI1_Pos)

#define SYSCFG_EXTICR1_EXTI0_Pos 0U
#define SYSCFG_EXTICR1_EXTI0_Msk (0x7UL << SYSCFG_EXTICR1_EXTI0_Pos)
#define SYSCFG_EXTICR1_EXTI0 SYSCFG_EXTICR1_EXTI0_Msk
#define SYSCFG_EXTICR1_EXTI0_PA0 (0x0UL << SYSCFG_EXTICR1_EXTI0_Pos)
#define SYSCFG_EXTICR1_EXTI0_PB0 (0x1UL << SYSCFG_EXTICR1_EXTI0_Pos)
#define SYSCFG_EXTICR1_EXTI0_PC0 (0x2UL << SYSCFG_EXTICR1_EXTI0_Pos)
#define SYSCFG_EXTICR1_EXTI0_PD0 (0x3UL << SYSCFG_EXTICR1_EXTI0_Pos)
#define SYSCFG_EXTICR1_EXTI0_PE0 (0x4UL << SYSCFG_EXTICR1_EXTI0_Pos)
#define SYSCFG_EXTICR1_EXTI0_PH0 (0x7UL << SYSCFG_EXTICR1_EXTI0_Pos)

/*EXTICR2 register*/
#define SYSCFG_EXTICR2_EXTI7_Pos 12U
#define SYSCFG_EXTICR2_EXTI7_Msk (0x7UL << SYSCFG_EXTICR2_EXTI7_Pos)
#define SYSCFG_EXTICR2_EXTI7 SYSCFG_EXTICR2_EXTI7_Msk
#define SYSCFG_EXTICR2_EXTI7_PA7 (0x0UL << SYSCFG_EXTICR2_EXTI7_Pos)
#define SYSCFG_EXTICR2_EXTI7_PB7 (0x1UL << SYSCFG_EXTICR2_EXTI7_Pos)
#define SYSCFG_EXTICR2_EXTI7_PC7 (0x2UL << SYSCFG_EXTICR2_EXTI7_Pos)
#define SYSCFG_EXTICR2_EXTI7_PD7 (0x3UL << SYSCFG_EXTICR2_EXTI7_Pos)
#define SYSCFG_EXTICR2_EXTI7_PE7 (0x4UL << SYSCFG_EXTICR2_EXTI7_Pos)

#define SYSCFG_EXTICR2_EXTI6_Pos 8U
#define SYSCFG_EXTICR2_EXTI6_Msk (0x7UL << SYSCFG_EXTICR2_EXTI6_Pos)
#define SYSCFG_EXTICR2_EXTI6 SYSCFG_EXTICR2_EXTI6_Msk
#define SYSCFG_EXTICR2_EXTI6_PA6 (0x0UL << SYSCFG_EXTICR2_EXTI6_Pos)
#define SYSCFG_EXTICR2_EXTI6_PB6 (0x1UL << SYSCFG_EXTICR2_EXTI6_Pos)
#define SYSCFG_EXTICR2_EXTI6_PC6 (0x2UL << SYSCFG_EXTICR2_EXTI6_Pos)
#define SYSCFG_EXTICR2_EXTI6_PD6 (0x3UL << SYSCFG_EXTICR2_EXTI6_Pos)
#define SYSCFG_EXTICR2_EXTI6_PE6 (0x4UL << SYSCFG_EXTICR2_EXTI6_Pos)

#define SYSCFG_EXTICR2_EXTI5_Pos 4U
#define SYSCFG_EXTICR2_EXTI5_Msk (0x7UL << SYSCFG_EXTICR2_EXTI5_Pos)
#define SYSCFG_EXTICR2_EXTI5 SYSCFG_EXTICR2_EXTI5_Msk
#define SYSCFG_EXTICR2_EXTI5_PA5 (0x0UL << SYSCFG_EXTICR2_EXTI5_Pos)
#define SYSCFG_EXTICR2_EXTI5_PB5 (0x1UL << SYSCFG_EXTICR2_EXTI5_Pos)
#define SYSCFG_EXTICR2_EXTI5_PC5 (0x2UL << SYSCFG_EXTICR2_EXTI5_Pos)
#define SYSCFG_EXTICR2_EXTI5_PD5 (0x3UL << SYSCFG_EXTICR2_EXTI5_Pos)
#define SYSCFG_EXTICR2_EXTI5_PE5 (0x4UL << SYSCFG_EXTICR2_EXTI5_Pos)

#define SYSCFG_EXTICR2_EXTI4_Pos 0U
#define SYSCFG_EXTICR2_EXTI4_Msk (0x7UL << SYSCFG_EXTICR2_EXTI4_Pos)
#define SYSCFG_EXTICR2_EXTI4 SYSCFG_EXTICR2_EXTI4_Msk
#define SYSCFG_EXTICR2_EXTI4_PA4 (0x0UL << SYSCFG_EXTICR2_EXTI4_Pos)
#define SYSCFG_EXTICR2_EXTI4_PB4 (0x1UL << SYSCFG_EXTICR2_EXTI4_Pos)
#define SYSCFG_EXTICR2_EXTI4_PC4 (0x2UL << SYSCFG_EXTICR2_EXTI4_Pos)
#define SYSCFG_EXTICR2_EXTI4_PD4 (0x3UL << SYSCFG_EXTICR2_EXTI4_Pos)
#define SYSCFG_EXTICR2_EXTI4_PE4 (0x4UL << SYSCFG_EXTICR2_EXTI4_Pos)

/*EXTICR3 register*/
#define SYSCFG_EXTICR3_EXTI11_Pos 12U
#define SYSCFG_EXTICR3_EXTI11_Msk (0x7UL << SYSCFG_EXTICR3_EXTI11_Pos)
#define SYSCFG_EXTICR3_EXTI11 SYSCFG_EXTICR3_EXTI11_Msk
#define SYSCFG_EXTICR3_EXTI11_PA11 (0x0UL << SYSCFG_EXTICR3_EXTI11_Pos)
#define SYSCFG_EXTICR3_EXTI11_PB11 (0x1UL << SYSCFG_EXTICR3_EXTI11_Pos)
#define SYSCFG_EXTICR3_EXTI11_PC11 (0x2UL << SYSCFG_EXTICR3_EXTI11_Pos)
#define SYSCFG_EXTICR3_EXTI11_PD11 (0x3UL << SYSCFG_EXTICR3_EXTI11_Pos)
#define SYSCFG_EXTICR3_EXTI11_PE11 (0x4UL << SYSCFG_EXTICR3_EXTI11_Pos)

#define SYSCFG_EXTICR3_EXTI10_Pos 8U
#define SYSCFG_EXTICR3_EXTI10_Msk (0x7UL << SYSCFG_EXTICR3_EXTI10_Pos)
#define SYSCFG_EXTICR3_EXTI10 SYSCFG_EXTICR3_EXTI10_Msk
#define SYSCFG_EXTICR3_EXTI10_PA10 (0x0UL << SYSCFG_EXTICR3_EXTI10_Pos)
#define SYSCFG_EXTICR3_EXTI10_PB10 (0x1UL << SYSCFG_EXTICR3_EXTI10_Pos)
#define SYSCFG_EXTICR3_EXTI10_PC10 (0x2UL << SYSCFG_EXTICR3_EXTI10_Pos)
#define SYSCFG_EXTICR3_EXTI10_PD10 (0x3UL << SYSCFG_EXTICR3_EXTI10_Pos)
#define SYSCFG_EXTICR3_EXTI10_PE10 (0x4UL << SYSCFG_EXTICR3_EXTI10_Pos)

#define SYSCFG_EXTICR3_EXTI9_Pos 4U
#define SYSCFG_EXTICR3_EXTI9_Msk (0x7UL << SYSCFG_EXTICR3_EXTI9_Pos)
#define SYSCFG_EXTICR3_EXTI9 SYSCFG_EXTICR3_EXTI9_Msk
#define SYSCFG_EXTICR3_EXTI9_PA9 (0x0UL << SYSCFG_EXTICR3_EXTI9_Pos)
#define SYSCFG_EXTICR3_EXTI9_PB9 (0x1UL << SYSCFG_EXTICR3_EXTI9_Pos)
#define SYSCFG_EXTICR3_EXTI9_PC9 (0x2UL << SYSCFG_EXTICR3_EXTI9_Pos)
#define SYSCFG_EXTICR3_EXTI9_PD9 (0x3UL << SYSCFG_EXTICR3_EXTI9_Pos)
#define SYSCFG_EXTICR3_EXTI9_PE9 (0x4UL << SYSCFG_EXTICR3_EXTI9_Pos)

#define SYSCFG_EXTICR3_EXTI8_Pos 0U
#define SYSCFG_EXTICR3_EXTI8_Msk (0x7UL << SYSCFG_EXTICR3_EXTI8_Pos)
#define SYSCFG_EXTICR3_EXTI8 SYSCFG_EXTICR3_EXTI8_Msk
#define SYSCFG_EXTICR3_EXTI8_PA8 (0x0UL << SYSCFG_EXTICR3_EXTI8_Pos)
#define SYSCFG_EXTICR3_EXTI8_PB8 (0x1UL << SYSCFG_EXTICR3_EXTI8_Pos)
#define SYSCFG_EXTICR3_EXTI8_PC8 (0x2UL << SYSCFG_EXTICR3_EXTI8_Pos)
#define SYSCFG_EXTICR3_EXTI8_PD8 (0x3UL << SYSCFG_EXTICR3_EXTI8_Pos)
#define SYSCFG_EXTICR3_EXTI8_PE8 (0x4UL << SYSCFG_EXTICR3_EXTI8_Pos)

/*EXTICR4 register*/
#define SYSCFG_EXTICR4_EXTI15_Pos 12U
#define SYSCFG_EXTICR4_EXTI15_Msk (0x7UL << SYSCFG_EXTICR4_EXTI15_Pos)
#define SYSCFG_EXTICR4_EXTI15 SYSCFG_EXTICR4_EXTI15_Msk
#define SYSCFG_EXTICR4_EXTI15_PA15 (0x0UL << SYSCFG_EXTICR4_EXTI15_Pos)
#define SYSCFG_EXTICR4_EXTI15_PB15 (0x1UL << SYSCFG_EXTICR4_EXTI15_Pos)
#define SYSCFG_EXTICR4_EXTI15_PC15 (0x2UL << SYSCFG_EXTICR4_EXTI15_Pos)
#define SYSCFG_EXTICR4_EXTI15_PD15 (0x3UL << SYSCFG_EXTICR4_EXTI15_Pos)
#define SYSCFG_EXTICR4_EXTI15_PE15 (0x4UL << SYSCFG_EXTICR4_EXTI15_Pos)

#define SYSCFG_EXTICR4_EXTI14_Pos 8U
#define SYSCFG_EXTICR4_EXTI14_Msk (0x7UL << SYSCFG_EXTICR4_EXTI14_Pos)
#define SYSCFG_EXTICR4_EXTI14 SYSCFG_EXTICR4_EXTI14_Msk
#define SYSCFG_EXTICR4_EXTI14_PA14 (0x0UL << SYSCFG_EXTICR4_EXTI14_Pos)
#define SYSCFG_EXTICR4_EXTI14_PB14 (0x1UL << SYSCFG_EXTICR4_EXTI14_Pos)
#define SYSCFG_EXTICR4_EXTI14_PC14 (0x2UL << SYSCFG_EXTICR4_EXTI14_Pos)
#define SYSCFG_EXTICR4_EXTI14_PD14 (0x3UL << SYSCFG_EXTICR4_EXTI14_Pos)
#define SYSCFG_EXTICR4_EXTI14_PE14 (0x4UL << SYSCFG_EXTICR4_EXTI14_Pos)

#define SYSCFG_EXTICR4_EXTI13_Pos 4U
#define SYSCFG_EXTICR4_EXTI13_Msk (0x7UL << SYSCFG_EXTICR4_EXTI13_Pos)
#define SYSCFG_EXTICR4_EXTI13 SYSCFG_EXTICR4_EXTI13_Msk
#define SYSCFG_EXTICR4_EXTI13_PA13 (0x0UL << SYSCFG_EXTICR4_EXTI13_Pos)
#define SYSCFG_EXTICR4_EXTI13_PB13 (0x1UL << SYSCFG_EXTICR4_EXTI13_Pos)
#define SYSCFG_EXTICR4_EXTI13_PC13 (0x2UL << SYSCFG_EXTICR4_EXTI13_Pos)
#define SYSCFG_EXTICR4_EXTI13_PD13 (0x3UL << SYSCFG_EXTICR4_EXTI13_Pos)
#define SYSCFG_EXTICR4_EXTI13_PE13 (0x4UL << SYSCFG_EXTICR4_EXTI13_Pos)

#define SYSCFG_EXTICR4_EXTI12_Pos 0U
#define SYSCFG_EXTICR4_EXTI12_Msk (0x7UL << SYSCFG_EXTICR4_EXTI12_Pos)
#define SYSCFG_EXTICR4_EXTI12 SYSCFG_EXTICR4_EXTI12_Msk
#define SYSCFG_EXTICR4_EXTI12_PA12 (0x0UL << SYSCFG_EXTICR4_EXTI12_Pos)
#define SYSCFG_EXTICR4_EXTI12_PB12 (0x1UL << SYSCFG_EXTICR4_EXTI12_Pos)
#define SYSCFG_EXTICR4_EXTI12_PC12 (0x2UL << SYSCFG_EXTICR4_EXTI12_Pos)
#define SYSCFG_EXTICR4_EXTI12_PD12 (0x3UL << SYSCFG_EXTICR4_EXTI12_Pos)
#define SYSCFG_EXTICR4_EXTI12_PE12 (0x4UL << SYSCFG_EXTICR4_EXTI12_Pos)

/*SCSR register*/
#define SYSCFG_SCSR_SRAM2BSY_Pos 1U
#define SYSCFG_SCSR_SRAM2BSY_Msk (0x1UL << SYSCFG_SCSR_SRAM2BSY_Pos)
#define SYSCFG_SCSR_SRAM2BSY SYSCFG_SCSR_SRAM2BSY_Msk

#define SYSCFG_SCSR_SRAM2ER_Pos 0U
#define SYSCFG_SCSR_SRAM2ER_Msk (0x1UL << SYSCFG_SCSR_SRAM2ER_Pos)
#define SYSCFG_SCSR_SRAM2ER SYSCFG_SCSR_SRAM2ER_Msk

/*CFGR2 register*/
#define SYSCFG_CFGR2_SPF_Pos 8U
#define SYSCFG_CFGR2_SPF_Msk (0x1UL << SYSCFG_CFGR2_SPF_Pos)
#define SYSCFG_CFGR2_SPF SYSCFG_CFGR2_SPF_Msk

#define SYSCFG_CFGR2_ECCL_Pos 3U
#define SYSCFG_CFGR2_ECCL_Msk (0x1UL << SYSCFG_CFGR2_ECCL_Pos)
#define SYSCFG_CFGR2_ECCL SYSCFG_CFGR2_ECCL_Msk

#define SYSCFG_CFGR2_PVDL_Pos 2U
#define SYSCFG_CFGR2_PVDL_Msk (0x1UL << SYSCFG_CFGR2_PVDL_Pos)
#define SYSCFG_CFGR2_PVDL SYSCFG_CFGR2_PVDL_Msk

#define SYSCFG_CFGR2_SPL_Pos 1U
#define SYSCFG_CFGR2_SPL_Msk (0x1UL << SYSCFG_CFGR2_SPL_Pos)
#define SYSCFG_CFGR2_SPL SYSCFG_CFGR2_SPL_Msk

#define SYSCFG_CFGR2_CLL_Pos 0U
#define SYSCFG_CFGR2_CLL_Msk (0x1UL << SYSCFG_CFGR2_CLL_Pos)
#define SYSCFG_CFGR2_CLL SYSCFG_CFGR2_CLL_Msk

/*SWPR register*/
#define SYSCFG_SWPR_P31WP_Pos 31U
#define SYSCFG_SWPR_P31WP_Msk (0x1UL << SYSCFG_SWPR_P31WP_Pos)
#define SYSCFG_SWPR_P31WP SYSCFG_SWPR_P31WP_Msk

#define SYSCFG_SWPR_P30WP_Pos 30U
#define SYSCFG_SWPR_P30WP_Msk (0x1UL << SYSCFG_SWPR_P30WP_Pos)
#define SYSCFG_SWPR_P30WP SYSCFG_SWPR_P30WP_Msk

#define SYSCFG_SWPR_P29WP_Pos 29U
#define SYSCFG_SWPR_P29WP_Msk (0x1UL << SYSCFG_SWPR_P29WP_Pos)
#define SYSCFG_SWPR_P29WP SYSCFG_SWPR_P29WP_Msk

#define SYSCFG_SWPR_P28WP_Pos 28U
#define SYSCFG_SWPR_P28WP_Msk (0x1UL << SYSCFG_SWPR_P28WP_Pos)
#define SYSCFG_SWPR_P28WP SYSCFG_SWPR_P28WP_Msk

#define SYSCFG_SWPR_P27WP_Pos 27U
#define SYSCFG_SWPR_P27WP_Msk (0x1UL << SYSCFG_SWPR_P27WP_Pos)
#define SYSCFG_SWPR_P27WP SYSCFG_SWPR_P27WP_Msk

#define SYSCFG_SWPR_P26WP_Pos 26U
#define SYSCFG_SWPR_P26WP_Msk (0x1UL << SYSCFG_SWPR_P26WP_Pos)
#define SYSCFG_SWPR_P26WP SYSCFG_SWPR_P26WP_Msk

#define SYSCFG_SWPR_P25WP_Pos 25U
#define SYSCFG_SWPR_P25WP_Msk (0x1UL << SYSCFG_SWPR_P25WP_Pos)
#define SYSCFG_SWPR_P25WP SYSCFG_SWPR_P25WP_Msk

#define SYSCFG_SWPR_P24WP_Pos 24U
#define SYSCFG_SWPR_P24WP_Msk (0x1UL << SYSCFG_SWPR_P24WP_Pos)
#define SYSCFG_SWPR_P24WP SYSCFG_SWPR_P24WP_Msk

#define SYSCFG_SWPR_P23WP_Pos 23U
#define SYSCFG_SWPR_P23WP_Msk (0x1UL << SYSCFG_SWPR_P23WP_Pos)
#define SYSCFG_SWPR_P23WP SYSCFG_SWPR_P23WP_Msk

#define SYSCFG_SWPR_P22WP_Pos 22U
#define SYSCFG_SWPR_P22WP_Msk (0x1UL << SYSCFG_SWPR_P22WP_Pos)
#define SYSCFG_SWPR_P22WP SYSCFG_SWPR_P22WP_Msk

#define SYSCFG_SWPR_P21WP_Pos 21U
#define SYSCFG_SWPR_P21WP_Msk (0x1UL << SYSCFG_SWPR_P21WP_Pos)
#define SYSCFG_SWPR_P21WP SYSCFG_SWPR_P21WP_Msk

#define SYSCFG_SWPR_P20WP_Pos 20U
#define SYSCFG_SWPR_P20WP_Msk (0x1UL << SYSCFG_SWPR_P20WP_Pos)
#define SYSCFG_SWPR_P20WP SYSCFG_SWPR_P20WP_Msk

#define SYSCFG_SWPR_P19WP_Pos 19U
#define SYSCFG_SWPR_P19WP_Msk (0x1UL << SYSCFG_SWPR_P19WP_Pos)
#define SYSCFG_SWPR_P19WP SYSCFG_SWPR_P19WP_Msk

#define SYSCFG_SWPR_P18WP_Pos 18U
#define SYSCFG_SWPR_P18WP_Msk (0x1UL << SYSCFG_SWPR_P18WP_Pos)
#define SYSCFG_SWPR_P18WP SYSCFG_SWPR_P18WP_Msk

#define SYSCFG_SWPR_P17WP_Pos 17U
#define SYSCFG_SWPR_P17WP_Msk (0x1UL << SYSCFG_SWPR_P17WP_Pos)
#define SYSCFG_SWPR_P17WP SYSCFG_SWPR_P17WP_Msk

#define SYSCFG_SWPR_P16WP_Pos 16U
#define SYSCFG_SWPR_P16WP_Msk (0x1UL << SYSCFG_SWPR_P16WP_Pos)
#define SYSCFG_SWPR_P16WP SYSCFG_SWPR_P16WP_Msk

#define SYSCFG_SWPR_P15WP_Pos 15U
#define SYSCFG_SWPR_P15WP_Msk (0x1UL << SYSCFG_SWPR_P15WP_Pos)
#define SYSCFG_SWPR_P15WP SYSCFG_SWPR_P15WP_Msk

#define SYSCFG_SWPR_P14WP_Pos 14U
#define SYSCFG_SWPR_P14WP_Msk (0x1UL << SYSCFG_SWPR_P14WP_Pos)
#define SYSCFG_SWPR_P14WP SYSCFG_SWPR_P14WP_Msk

#define SYSCFG_SWPR_P13WP_Pos 13U
#define SYSCFG_SWPR_P13WP_Msk (0x1UL << SYSCFG_SWPR_P13WP_Pos)
#define SYSCFG_SWPR_P13WP SYSCFG_SWPR_P13WP_Msk

#define SYSCFG_SWPR_P12WP_Pos 12U
#define SYSCFG_SWPR_P12WP_Msk (0x1UL << SYSCFG_SWPR_P12WP_Pos)
#define SYSCFG_SWPR_P12WP SYSCFG_SWPR_P12WP_Msk

#define SYSCFG_SWPR_P11WP_Pos 11U
#define SYSCFG_SWPR_P11WP_Msk (0x1UL << SYSCFG_SWPR_P11WP_Pos)
#define SYSCFG_SWPR_P11WP SYSCFG_SWPR_P11WP_Msk

#define SYSCFG_SWPR_P10WP_Pos 10U
#define SYSCFG_SWPR_P10WP_Msk (0x1UL << SYSCFG_SWPR_P10WP_Pos)
#define SYSCFG_SWPR_P10WP SYSCFG_SWPR_P10WP_Msk

#define SYSCFG_SWPR_P9WP_Pos 9U
#define SYSCFG_SWPR_P9WP_Msk (0x1UL << SYSCFG_SWPR_P9WP_Pos)
#define SYSCFG_SWPR_P9WP SYSCFG_SWPR_P9WP_Msk

#define SYSCFG_SWPR_P8WP_Pos 8U
#define SYSCFG_SWPR_P8WP_Msk (0x1UL << SYSCFG_SWPR_P8WP_Pos)
#define SYSCFG_SWPR_P8WP SYSCFG_SWPR_P8WP_Msk

#define SYSCFG_SWPR_P7WP_Pos 7U
#define SYSCFG_SWPR_P7WP_Msk (0x1UL << SYSCFG_SWPR_P7WP_Pos)
#define SYSCFG_SWPR_P7WP SYSCFG_SWPR_P7WP_Msk

#define SYSCFG_SWPR_P6WP_Pos 6U
#define SYSCFG_SWPR_P6WP_Msk (0x1UL << SYSCFG_SWPR_P6WP_Pos)
#define SYSCFG_SWPR_P6WP SYSCFG_SWPR_P6WP_Msk

#define SYSCFG_SWPR_P5WP_Pos 5U
#define SYSCFG_SWPR_P5WP_Msk (0x1UL << SYSCFG_SWPR_P5WP_Pos)
#define SYSCFG_SWPR_P5WP SYSCFG_SWPR_P5WP_Msk

#define SYSCFG_SWPR_P4WP_Pos 4U
#define SYSCFG_SWPR_P4WP_Msk (0x1UL << SYSCFG_SWPR_P4WP_Pos)
#define SYSCFG_SWPR_P4WP SYSCFG_SWPR_P4WP_Msk

#define SYSCFG_SWPR_P3WP_Pos 3U
#define SYSCFG_SWPR_P3WP_Msk (0x1UL << SYSCFG_SWPR_P3WP_Pos)
#define SYSCFG_SWPR_P3WP SYSCFG_SWPR_P3WP_Msk

#define SYSCFG_SWPR_P2WP_Pos 2U
#define SYSCFG_SWPR_P2WP_Msk (0x1UL << SYSCFG_SWPR_P2WP_Pos)
#define SYSCFG_SWPR_P2WP SYSCFG_SWPR_P2WP_Msk

#define SYSCFG_SWPR_P1WP_Pos 1U
#define SYSCFG_SWPR_P1WP_Msk (0x1UL << SYSCFG_SWPR_P1WP_Pos)
#define SYSCFG_SWPR_P1WP SYSCFG_SWPR_P1WP_Msk

#define SYSCFG_SWPR_P0WP_Pos 0U
#define SYSCFG_SWPR_P0WP_Msk (0x1UL << SYSCFG_SWPR_P0WP_Pos)
#define SYSCFG_SWPR_P0WP SYSCFG_SWPR_P0WP_Msk

/*SKR register*/
#define SYSCFG_SKR_KEY1 0xC1UL 
#define SYSCFG_SKR_KEY2 0x53UL

/* ========================================================================= */
/* ============                       DMA                       ============ */
/* ========================================================================= */
typedef struct
{
	__IM uint32_t ISR; /*!< DMA Interrupt Status Register. */
	__OM uint32_t IFCR; /*!< DMA Interrupt Flag Clear Register. */ 
	__IOM uint32_t CCR1; /*!< DMA Channel 1 Configuration Register. */
	__IOM uint32_t CNDTR1; /*!< DMA Channel 1 Number of Data to Transfer Register. */
	__IOM uint32_t CPAR1; /*!< DMA Channel 1 Peripheral Address Register. */
	__IOM uint32_t CMAR1; /*!< DMA Channel 1 Memory Address Register. */
	uint32_t RESERVED0;
	__IOM uint32_t CCR2; /*!< DMA Channel 2 Configuration Register. */
	__IOM uint32_t CNDTR2; /*!< DMA Channel 2 Number of Data to Transfer Register. */
	__IOM uint32_t CPAR2; /*!< DMA Channel 2 Peripheral Address Register. */
	__IOM uint32_t CMAR2; /*!< DMA Channel 2 Memory Address Register. */
	uint32_t RESERVED1;
	__IOM uint32_t CCR3; /*!< DMA Channel 3 Configuration Register. */
	__IOM uint32_t CNDTR3; /*!< DMA Channel 3 Number of Data to Transfer Register. */
	__IOM uint32_t CPAR3; /*!< DMA Channel 3 Peripheral Address Register. */
	__IOM uint32_t CMAR3; /*!< DMA Channel 3 Memory Address Register. */
	uint32_t RESERVED2;
	__IOM uint32_t CCR4; /*!< DMA Channel 4 Configuration Register. */
	__IOM uint32_t CNDTR4; /*!< DMA Channel 4 Number of Data to Transfer Register. */ 
	__IOM uint32_t CPAR4; /*!< DMA Channel 4 Peripheral Address Register. */
	__IOM uint32_t CMAR4; /*!< DMA Channel 4 Memory Address Register. */
	uint32_t RESERVED3;
	__IOM uint32_t CCR5; /*!< DMA Channel 5 Configuration Register. */
	__IOM uint32_t CNDTR5; /*!< DMA Channel 5 Number of Data to Transfer Register. */
	__IOM uint32_t CPAR5; /*!< DMA Channel 5 Peripheral Address Register. */
	__IOM uint32_t CMAR5; /*!< DMA Channel 5 Memory Address Register. */
	uint32_t RESERVED4;
	__IOM uint32_t CCR6; /*!< DMA Channel 6 Configuration Register. */
	__IOM uint32_t CNDTR6; /*!< DMA Channel 6 Number of Data to Transfer Register. */
	__IOM uint32_t CPAR6; /*!< DMA Channel 6 Peripheral Address Register. */
	__IOM uint32_t CMAR6; /*!< DMA Channel 6 Memory Address Register. */
	uint32_t RESERVED5;
	__IOM uint32_t CCR7; /*!< DMA Channel 7 Configuration Register. */
	__IOM uint32_t CNDTR7; /*!< DMA Channel 7 Number of Data to Transfer Register. */
	__IOM uint32_t CPAR7; /*!< DMA Channel 7 Peripheral Address Register. */
	__IOM uint32_t CMAR7; /*!< DMA Channel 7 Memory Address Register. */
	uint32_t RESERVED6;
	uint32_t RESERVED7[5];
	__IOM uint32_t CSELR; /*!< DMA Channel Selection Register. */
}STM32L4xx_DMA_TypeDef;

/*ISR register*/
#define DMA_ISR_TEIF7_Pos 27U
#define DMA_ISR_TEIF7_Msk (0x1UL << DMA_ISR_TEIF7_Pos)
#define DMA_ISR_TEIF7 DMA_ISR_TEIF7_Msk

#define DMA_ISR_HTIF7_Pos 26U
#define DMA_ISR_HTIF7_Msk (0x1UL << DMA_ISR_HTIF7_Pos)
#define DMA_ISR_HTIF7 DMA_ISR_HTIF7_Msk

#define DMA_ISR_TCIF7_Pos 25U
#define DMA_ISR_TCIF7_Msk (0x1UL << DMA_ISR_TCIF7_Pos)
#define DMA_ISR_TCIF7 DMA_ISR_TCIF7_Msk

#define DMA_ISR_GIF7_Pos 24U
#define DMA_ISR_GIF7_Msk (0x1UL << DMA_ISR_GIF7_Pos)
#define DMA_ISR_GIF7 DMA_ISR_GIF7_Msk

#define DMA_ISR_TEIF6_Pos 23U
#define DMA_ISR_TEIF6_Msk (0x1UL << DMA_ISR_TEIF6_Pos)
#define DMA_ISR_TEIF6 DMA_ISR_TEIF6_Msk

#define DMA_ISR_HTIF6_Pos 22U
#define DMA_ISR_HTIF6_Msk (0x1UL << DMA_ISR_HTIF6_Pos)
#define DMA_ISR_HTIF6 DMA_ISR_HTIF6_Msk

#define DMA_ISR_TCIF6_Pos 21U
#define DMA_ISR_TCIF6_Msk (0x1UL << DMA_ISR_TCIF6_Pos)
#define DMA_ISR_TCIF6 DMA_ISR_TCIF6_Msk

#define DMA_ISR_GIF6_Pos 20U
#define DMA_ISR_GIF6_Msk (0x1UL << DMA_ISR_GIF6_Pos)
#define DMA_ISR_GIF6 DMA_ISR_GIF6_Msk

#define DMA_ISR_TEIF5_Pos 19U
#define DMA_ISR_TEIF5_Msk (0x1UL << DMA_ISR_TEIF5_Pos)
#define DMA_ISR_TEIF5 DMA_ISR_TEIF5_Msk

#define DMA_ISR_HTIF5_Pos 18U
#define DMA_ISR_HTIF5_Msk (0x1UL << DMA_ISR_HTIF5_Pos)
#define DMA_ISR_HTIF5 DMA_ISR_HTIF7_Msk

#define DMA_ISR_TCIF5_Pos 17U
#define DMA_ISR_TCIF5_Msk (0x1UL << DMA_ISR_TCIF5_Pos)
#define DMA_ISR_TCIF5 DMA_ISR_TCIF5_Msk

#define DMA_ISR_GIF5_Pos 16U
#define DMA_ISR_GIF5_Msk (0x1UL << DMA_ISR_GIF5_Pos)
#define DMA_ISR_GIF5 DMA_ISR_GIF5_Msk

#define DMA_ISR_TEIF4_Pos 15U
#define DMA_ISR_TEIF4_Msk (0x1UL << DMA_ISR_TEIF4_Pos)
#define DMA_ISR_TEIF4 DMA_ISR_TEIF4_Msk

#define DMA_ISR_HTIF4_Pos 14U
#define DMA_ISR_HTIF4_Msk (0x1UL << DMA_ISR_HTIF4_Pos)
#define DMA_ISR_HTIF4 DMA_ISR_HTIF4_Msk

#define DMA_ISR_TCIF4_Pos 13U
#define DMA_ISR_TCIF4_Msk (0x1UL << DMA_ISR_TCIF4_Pos)
#define DMA_ISR_TCIF4 DMA_ISR_TCIF4_Msk

#define DMA_ISR_GIF4_Pos 12U
#define DMA_ISR_GIF4_Msk (0x1UL << DMA_ISR_GIF4_Pos)
#define DMA_ISR_GIF4 DMA_ISR_GIF4_Msk

#define DMA_ISR_TEIF3_Pos 11U
#define DMA_ISR_TEIF3_Msk (0x1UL << DMA_ISR_TEIF3_Pos)
#define DMA_ISR_TEIF3 DMA_ISR_TEIF3_Msk

#define DMA_ISR_HTIF3_Pos 10U
#define DMA_ISR_HTIF3_Msk (0x1UL << DMA_ISR_HTIF3_Pos)
#define DMA_ISR_HTIF3 DMA_ISR_HTIF3_Msk

#define DMA_ISR_TCIF3_Pos 9U
#define DMA_ISR_TCIF3_Msk (0x1UL << DMA_ISR_TCIF3_Pos)
#define DMA_ISR_TCIF3 DMA_ISR_TCIF3_Msk

#define DMA_ISR_GIF3_Pos 8U
#define DMA_ISR_GIF3_Msk (0x1UL << DMA_ISR_GIF3_Pos)
#define DMA_ISR_GIF3 DMA_ISR_GIF3_Msk

#define DMA_ISR_TEIF2_Pos 7U
#define DMA_ISR_TEIF2_Msk (0x1UL << DMA_ISR_TEIF2_Pos)
#define DMA_ISR_TEIF2 DMA_ISR_TEIF2_Msk

#define DMA_ISR_HTIF2_Pos 6U
#define DMA_ISR_HTIF2_Msk (0x1UL << DMA_ISR_HTIF2_Pos)
#define DMA_ISR_HTIF2 DMA_ISR_HTIF2_Msk

#define DMA_ISR_TCIF2_Pos 5U
#define DMA_ISR_TCIF2_Msk (0x1UL << DMA_ISR_TCIF2_Pos)
#define DMA_ISR_TCIF2 DMA_ISR_TCIF2_Msk

#define DMA_ISR_GIF2_Pos 4U
#define DMA_ISR_GIF2_Msk (0x1UL << DMA_ISR_GIF2_Pos)
#define DMA_ISR_GIF2 DMA_ISR_GIF2_Msk

#define DMA_ISR_TEIF1_Pos 3U
#define DMA_ISR_TEIF1_Msk (0x1UL << DMA_ISR_TEIF1_Pos)
#define DMA_ISR_TEIF1 DMA_ISR_TEIF1_Msk

#define DMA_ISR_HTIF1_Pos 2U
#define DMA_ISR_HTIF1_Msk (0x1UL << DMA_ISR_HTIF1_Pos)
#define DMA_ISR_HTIF1 DMA_ISR_HTIF1_Msk

#define DMA_ISR_TCIF1_Pos 1U
#define DMA_ISR_TCIF1_Msk (0x1UL << DMA_ISR_TCIF1_Pos)
#define DMA_ISR_TCIF1 DMA_ISR_TCIF1_Msk

#define DMA_ISR_GIF1_Pos 0U
#define DMA_ISR_GIF1_Msk (0x1UL << DMA_ISR_GIF1_Pos)
#define DMA_ISR_GIF1 DMA_ISR_GIF1_Msk

/*IFCR register*/
#define DMA_IFCR_CTEIF7_Pos 27U
#define DMA_IFCR_CTEIF7_Msk (0x1UL << DMA_IFCR_CTEIF7_Pos)
#define DMA_IFCR_CTEIF7 DMA_IFCR_CTEIF7_Msk

#define DMA_IFCR_CHTIF7_Pos 26U
#define DMA_IFCR_CHTIF7_Msk (0x1UL << DMA_IFCR_CHTIF7_Pos)
#define DMA_IFCR_CHTIF7 DMA_IFCR_CHTIF7_Msk

#define DMA_IFCR_CTCIF7_Pos 25U
#define DMA_IFCR_CTCIF7_Msk (0x1UL << DMA_IFCR_CTCIF7_Pos)
#define DMA_IFCR_CTCIF7 DMA_IFCR_CTCIF7_Msk

#define DMA_IFCR_CGIF7_Pos 24U
#define DMA_IFCR_CGIF7_Msk (0x1UL << DMA_IFCR_CGIF7_Pos)
#define DMA_IFCR_CGIF7 DMA_IFCR_CGIF7_Msk

#define DMA_IFCR_CTEIF6_Pos 23U
#define DMA_IFCR_CTEIF6_Msk (0x1UL << DMA_IFCR_CTEIF6_Pos)
#define DMA_IFCR_CTEIF6 DMA_IFCR_CTEIF6_Msk

#define DMA_IFCR_CHTIF6_Pos 22U
#define DMA_IFCR_CHTIF6_Msk (0x1UL << DMA_IFCR_CHTIF6_Pos)
#define DMA_IFCR_CHTIF6 DMA_IFCR_CHTIF6_Msk

#define DMA_IFCR_CTCIF6_Pos 21U
#define DMA_IFCR_CTCIF6_Msk (0x1UL << DMA_IFCR_CTCIF6_Pos)
#define DMA_IFCR_CTCIF6 DMA_IFCR_CTCIF6_Msk

#define DMA_IFCR_CGIF6_Pos 20U
#define DMA_IFCR_CGIF6_Msk (0x1UL << DMA_IFCR_CGIF6_Pos)
#define DMA_IFCR_CGIF6 DMA_IFCR_CGIF6_Msk

#define DMA_IFCR_CTEIF5_Pos 19U
#define DMA_IFCR_CTEIF5_Msk (0x1UL << DMA_IFCR_CTEIF5_Pos)
#define DMA_IFCR_CTEIF5 DMA_IFCR_CTEIF5_Msk

#define DMA_IFCR_CHTIF5_Pos 18U
#define DMA_IFCR_CHTIF5_Msk (0x1UL << DMA_IFCR_CHTIF5_Pos)
#define DMA_IFCR_CHTIF5 DMA_IFCR_HTIF5_Msk

#define DMA_IFCR_CTCIF5_Pos 17U
#define DMA_IFCR_CTCIF5_Msk (0x1UL << DMA_IFCR_CTCIF5_Pos)
#define DMA_IFCR_CTCIF5 DMA_IFCR_CTCIF5_Msk

#define DMA_IFCR_CGIF5_Pos 16U
#define DMA_IFCR_CGIF5_Msk (0x1UL << DMA_IFCR_CGIF5_Pos)
#define DMA_IFCR_CGIF5 DMA_IFCR_CGIF5_Msk

#define DMA_IFCR_CTEIF4_Pos 15U
#define DMA_IFCR_CTEIF4_Msk (0x1UL << DMA_IFCR_CTEIF4_Pos)
#define DMA_IFCR_CTEIF4 DMA_IFCR_CTEIF4_Msk

#define DMA_IFCR_CHTIF4_Pos 14U
#define DMA_IFCR_CHTIF4_Msk (0x1UL << DMA_IFCR_CHTIF4_Pos)
#define DMA_IFCR_CHTIF4 DMA_IFCR_CHTIF4_Msk

#define DMA_IFCR_CTCIF4_Pos 13U
#define DMA_IFCR_CTCIF4_Msk (0x1UL << DMA_IFCR_CTCIF4_Pos)
#define DMA_IFCR_CTCIF4 DMA_IFCR_CTCIF4_Msk

#define DMA_IFCR_CGIF4_Pos 12U
#define DMA_IFCR_CGIF4_Msk (0x1UL << DMA_IFCR_CGIF4_Pos)
#define DMA_IFCR_CGIF4 DMA_IFCR_CGIF4_Msk

#define DMA_IFCR_CTEIF3_Pos 11U
#define DMA_IFCR_CTEIF3_Msk (0x1UL << DMA_IFCR_CTEIF3_Pos)
#define DMA_IFCR_CTEIF3 DMA_IFCR_CTEIF3_Msk

#define DMA_IFCR_CHTIF3_Pos 10U
#define DMA_IFCR_CHTIF3_Msk (0x1UL << DMA_IFCR_CHTIF3_Pos)
#define DMA_IFCR_CHTIF3 DMA_IFCR_CHTIF3_Msk

#define DMA_IFCR_CTCIF3_Pos 9U
#define DMA_IFCR_CTCIF3_Msk (0x1UL << DMA_IFCR_CTCIF3_Pos)
#define DMA_IFCR_CTCIF3 DMA_IFCR_CTCIF3_Msk

#define DMA_IFCR_CGIF3_Pos 8U
#define DMA_IFCR_CGIF3_Msk (0x1UL << DMA_IFCR_CGIF3_Pos)
#define DMA_IFCR_CGIF3 DMA_IFCR_CGIF3_Msk

#define DMA_IFCR_CTEIF2_Pos 7U
#define DMA_IFCR_CTEIF2_Msk (0x1UL << DMA_IFCR_CTEIF2_Pos)
#define DMA_IFCR_CTEIF2 DMA_IFCR_CTEIF2_Msk

#define DMA_IFCR_CHTIF2_Pos 6U
#define DMA_IFCR_CHTIF2_Msk (0x1UL << DMA_IFCR_CHTIF2_Pos)
#define DMA_IFCR_CHTIF2 DMA_IFCR_CHTIF2_Msk

#define DMA_IFCR_CTCIF2_Pos 5U
#define DMA_IFCR_CTCIF2_Msk (0x1UL << DMA_IFCR_CTCIF2_Pos)
#define DMA_IFCR_CTCIF2 DMA_IFCR_CTCIF2_Msk

#define DMA_IFCR_CGIF2_Pos 4U
#define DMA_IFCR_CGIF2_Msk (0x1UL << DMA_IFCR_CGIF2_Pos)
#define DMA_IFCR_CGIF2 DMA_IFCR_CGIF2_Msk

#define DMA_IFCR_CTEIF1_Pos 3U
#define DMA_IFCR_CTEIF1_Msk (0x1UL << DMA_IFCR_CTEIF1_Pos)
#define DMA_IFCR_CTEIF1 DMA_IFCR_CTEIF1_Msk

#define DMA_IFCR_CHTIF1_Pos 2U
#define DMA_IFCR_CHTIF1_Msk (0x1UL << DMA_IFCR_CHTIF1_Pos)
#define DMA_IFCR_CHTIF1 DMA_IFCR_CHTIF1_Msk

#define DMA_IFCR_CTCIF1_Pos 1U
#define DMA_IFCR_CTCIF1_Msk (0x1UL << DMA_IFCR_CTCIF1_Pos)
#define DMA_IFCR_CTCIF1 DMA_IFCR_CTCIF1_Msk

#define DMA_IFCR_CGIF1_Pos 0U
#define DMA_IFCR_CGIF1_Msk (0x1UL << DMA_IFCR_CGIF1_Pos)
#define DMA_IFCR_CGIF1 DMA_IFCR_CGIF1_Msk

/*CCRx register*/
#define DMA_CCR_MEM2MEM_Pos 14U
#define DMA_CCR_MEM2MEM_Msk (0x1UL << DMA_CCR_MEM2MEM_Pos)
#define DMA_CCR_MEM2MEM DMA_CCR_MEM2MEM_Msk

#define DMA_CCR_PL_Pos 12U
#define DMA_CCR_PL_Msk (0x3UL << DMA_CCR_PL_Pos)
#define DMA_CCR_PL DMA_CCR_PL_Msk

#define DMA_CCR_MSIZE_Pos 10U
#define DMA_CCR_MSIZE_Msk (0x3UL << DMA_CCR_MSIZE_Pos)
#define DMA_CCR_MSIZE DMA_CCR_MSIZE_Msk
#define DMA_CCR_MSIZE_8_BIT (0x0UL << DMA_CCR_MSIZE_Pos)
#define DMA_CCR_MSIZE_16_BIT (0x1UL << DMA_CCR_MSIZE_Pos)
#define DMA_CCR_MSIZE_32_BIT (0x2UL << DMA_CCR_MSIZE_Pos)

#define DMA_CCR_PSIZE_Pos 8U
#define DMA_CCR_PSIZE_Msk (0x3UL << DMA_CCR_PSIZE_Pos)
#define DMA_CCR_PSIZE DMA_CCR_PSIZE_Msk
#define DMA_CCR_PSIZE_8_BIT (0x0UL << DMA_CCR_PSIZE_Pos)
#define DMA_CCR_PSIZE_16_BIT (0x1UL << DMA_CCR_PSIZE_Pos)
#define DMA_CCR_PSIZE_32_BIT (0x2UL << DMA_CCR_PSIZE_Pos)

#define DMA_CCR_MINC_Pos 7U
#define DMA_CCR_MINC_Msk (0x1UL << DMA_CCR_MINC_Pos)
#define DMA_CCR_MINC DMA_CCR_MINC_Msk

#define DMA_CCR_PINC_Pos 6U
#define DMA_CCR_PINC_Msk (0x1UL << DMA_CCR_PINC_Pos)
#define DMA_CCR_PINC DMA_CCR_PINC_Msk

#define DMA_CCR_CIRC_Pos 5U
#define DMA_CCR_CIRC_Msk (0x1UL << DMA_CCR_CIRC_Pos)
#define DMA_CCR_CIRC DMA_CCR_CIRC_Msk

#define DMA_CCR_DIR_Pos 4U
#define DMA_CCR_DIR_Msk (0x1UL << DMA_CCR_DIR_Pos)
#define DMA_CCR_DIR DMA_CCR_DIR_Msk

#define DMA_CCR_TEIE_Pos 3U
#define DMA_CCR_TEIE_Msk (0x1UL << DMA_CCR_TEIE_Pos)
#define DMA_CCR_TEIE DMA_CCR_TEIE_Msk

#define DMA_CCR_HTIE_Pos 2U
#define DMA_CCR_HTIE_Msk (0x1UL << DMA_CCR_HTIE_Pos)
#define DMA_CCR_HTIE DMA_CCR_HTIE_Msk

#define DMA_CCR_TCIE_Pos 1U
#define DMA_CCR_TCIE_Msk (0x1UL << DMA_CCR_TCIE_Pos)
#define DMA_CCR_TCIE DMA_CCR_TCIE_Msk

#define DMA_CCR_EN_Pos 0U
#define DMA_CCR_EN_Msk (0x1UL << DMA_CCR_EN_Pos)
#define DMA_CCR_EN DMA_CCR_EN_Msk

/*CNDTRx register*/
#define DMA_CNDTR_NDT_Pos 0U
#define DMA_CNDTR_NDT_Msk (0xFFFFUL << DMA_CNDTR_NDT_Pos)
#define DMA_CNDTR_NDT DMA_CNDTR_NDT_Msk

/*CSELR register*/
#define DMA_CSELR_C7S_Pos 24U
#define DMA_CSELR_C7S_Msk (0xFUL << DMA_CSELR_C7S_Pos)
#define DMA_CSELR_C7S DMA_CSELR_C7S_Msk

#define DMA_CSELR_C6S_Pos 20U
#define DMA_CSELR_C6S_Msk (0xFUL << DMA_CSELR_C6S_Pos)
#define DMA_CSELR_C6S DMA_CSELR_C6S_Msk

#define DMA_CSELR_C5S_Pos 16U
#define DMA_CSELR_C5S_Msk (0xFUL << DMA_CSELR_C5S_Pos)
#define DMA_CSELR_C5S DMA_CSELR_C5S_Msk

#define DMA_CSELR_C4S_Pos 12U
#define DMA_CSELR_C4S_Msk (0xFUL << DMA_CSELR_C4S_Pos)
#define DMA_CSELR_C4S DMA_CSELR_C4S_Msk

#define DMA_CSELR_C3S_Pos 8U
#define DMA_CSELR_C3S_Msk (0xFUL << DMA_CSELR_C3S_Pos)
#define DMA_CSELR_C3S DMA_CSELR_C3S_Msk

#define DMA_CSELR_C2S_Pos 4U
#define DMA_CSELR_C2S_Msk (0xFUL << DMA_CSELR_C2S_Pos)
#define DMA_CSELR_C2S DMA_CSELR_C2S_Msk

#define DMA_CSELR_C1S_Pos 0U
#define DMA_CSELR_C1S_Msk (0xFUL << DMA_CSELR_C1S_Pos)
#define DMA_CSELR_C1S DMA_CSELR_C1S_Msk

/* ========================================================================= */
/* ============                       EXTI                      ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t IMR1; /*!< Interrupt Mask Register 1. */
	__IOM uint32_t EMR1; /*!< Event Mask Register 1. */
	__IOM uint32_t RTSR1; /*!< Rising Trigger Selection Register 1. */
	__IOM uint32_t FTSR1; /*!< Falling Trigger Selection Register 1. */
	__IOM uint32_t SWIER1; /*!< Software Interrupt Event Register 1. */
	__IOM uint32_t PR1; /*!< Pending Register 1. */
	__IOM uint32_t IMR2; /*!< Interrupt Mask Register 2. */
	__IOM uint32_t EMR2; /*!< Event Mask Register 2. */
	__IOM uint32_t RTSR2; /*!< Rising Trigger Selection Register 2. */
	__IOM uint32_t FTSR2; /*!< Falling Trigger Selection Register 2. */
	__IOM uint32_t SWIER2; /*!< Software Interrupt Event Register 2. */
	__IOM uint32_t PR2; /*!< Pending Register 2. */
}STM32L4xx_EXTI_TypeDef;

/*IMR1 register*/
#define EXTI_IMR1_IM31_Pos 31U
#define EXTI_IMR1_IM31_Msk (0x1UL << EXTI_IMR1_IM31_Pos)
#define EXTI_IMR1_IM31 EXTI_IMR1_IM31_Msk

#define EXTI_IMR1_IM28_Pos 28U
#define EXTI_IMR1_IM28_Msk (0x1UL << EXTI_IMR1_IM28_Pos)
#define EXTI_IMR1_IM28 EXTI_IMR1_IM28_Msk

#define EXTI_IMR1_IM27_Pos 27U
#define EXTI_IMR1_IM27_Msk (0x1UL << EXTI_IMR1_IM27_Pos)
#define EXTI_IMR1_IM27 EXTI_IMR1_IM27_Msk

#define EXTI_IMR1_IM26_Pos 26U
#define EXTI_IMR1_IM26_Msk (0x1UL << EXTI_IMR1_IM26_Pos)
#define EXTI_IMR1_IM26 EXTI_IMR1_IM26_Msk

#define EXTI_IMR1_IM25_Pos 25U
#define EXTI_IMR1_IM25_Msk (0x1UL << EXTI_IMR1_IM25_Pos)
#define EXTI_IMR1_IM25 EXTI_IMR1_IM25_Msk

#define EXTI_IMR1_IM24_Pos 24U
#define EXTI_IMR1_IM24_Msk (0x1UL << EXTI_IMR1_IM24_Pos)
#define EXTI_IMR1_IM24 EXTI_IMR1_IM24_Msk

#define EXTI_IMR1_IM23_Pos 23U
#define EXTI_IMR1_IM23_Msk (0x1UL << EXTI_IMR1_IM23_Pos)
#define EXTI_IMR1_IM23 EXTI_IMR1_IM23_Msk

#define EXTI_IMR1_IM22_Pos 22U
#define EXTI_IMR1_IM22_Msk (0x1UL << EXTI_IMR1_IM22_Pos)
#define EXTI_IMR1_IM22 EXTI_IMR1_IM22_Msk

#define EXTI_IMR1_IM21_Pos 21U
#define EXTI_IMR1_IM21_Msk (0x1UL << EXTI_IMR1_IM21_Pos)
#define EXTI_IMR1_IM21 EXTI_IMR1_IM21_Msk

#define EXTI_IMR1_IM20_Pos 20U
#define EXTI_IMR1_IM20_Msk (0x1UL << EXTI_IMR1_IM20_Pos)
#define EXTI_IMR1_IM20 EXTI_IMR1_IM20_Msk

#define EXTI_IMR1_IM19_Pos 19U
#define EXTI_IMR1_IM19_Msk (0x1UL << EXTI_IMR1_IM19_Pos)
#define EXTI_IMR1_IM19 EXTI_IMR1_IM19_Msk

#define EXTI_IMR1_IM18_Pos 18U
#define EXTI_IMR1_IM18_Msk (0x1UL << EXTI_IMR1_IM18_Pos)
#define EXTI_IMR1_IM18 EXTI_IMR1_IM18_Msk

#define EXTI_IMR1_IM17_Pos 17U
#define EXTI_IMR1_IM17_Msk (0x1UL << EXTI_IMR1_IM17_Pos)
#define EXTI_IMR1_IM17 EXTI_IMR1_IM17_Msk

#define EXTI_IMR1_IM16_Pos 16U
#define EXTI_IMR1_IM16_Msk (0x1UL << EXTI_IMR1_IM16_Pos)
#define EXTI_IMR1_IM16 EXTI_IMR1_IM16_Msk

#define EXTI_IMR1_IM15_Pos 15U
#define EXTI_IMR1_IM15_Msk (0x1UL << EXTI_IMR1_IM15_Pos)
#define EXTI_IMR1_IM15 EXTI_IMR1_IM15_Msk

#define EXTI_IMR1_IM14_Pos 14U
#define EXTI_IMR1_IM14_Msk (0x1UL << EXTI_IMR1_IM14_Pos)
#define EXTI_IMR1_IM14 EXTI_IMR1_IM14_Msk

#define EXTI_IMR1_IM13_Pos 13U
#define EXTI_IMR1_IM13_Msk (0x1UL << EXTI_IMR1_IM13_Pos)
#define EXTI_IMR1_IM13 EXTI_IMR1_IM13_Msk

#define EXTI_IMR1_IM12_Pos 12U
#define EXTI_IMR1_IM12_Msk (0x1UL << EXTI_IMR1_IM12_Pos)
#define EXTI_IMR1_IM12 EXTI_IMR1_IM12_Msk

#define EXTI_IMR1_IM11_Pos 11U
#define EXTI_IMR1_IM11_Msk (0x1UL << EXTI_IMR1_IM11_Pos)
#define EXTI_IMR1_IM11 EXTI_IMR1_IM11_Msk

#define EXTI_IMR1_IM10_Pos 10U
#define EXTI_IMR1_IM10_Msk (0x1UL << EXTI_IMR1_IM10_Pos)
#define EXTI_IMR1_IM10 EXTI_IMR1_IM10_Msk

#define EXTI_IMR1_IM9_Pos 9U
#define EXTI_IMR1_IM9_Msk (0x1UL << EXTI_IMR1_IM9_Pos)
#define EXTI_IMR1_IM9 EXTI_IMR1_IM9_Msk

#define EXTI_IMR1_IM8_Pos 8U
#define EXTI_IMR1_IM8_Msk (0x1UL << EXTI_IMR1_IM8_Pos)
#define EXTI_IMR1_IM8 EXTI_IMR1_IM8_Msk

#define EXTI_IMR1_IM7_Pos 7U
#define EXTI_IMR1_IM7_Msk (0x1UL << EXTI_IMR1_IM7_Pos)
#define EXTI_IMR1_IM7 EXTI_IMR1_IM7_Msk

#define EXTI_IMR1_IM6_Pos 6U
#define EXTI_IMR1_IM6_Msk (0x1UL << EXTI_IMR1_IM6_Pos)
#define EXTI_IMR1_IM6 EXTI_IMR1_IM6_Msk

#define EXTI_IMR1_IM5_Pos 5U
#define EXTI_IMR1_IM5_Msk (0x1UL << EXTI_IMR1_IM5_Pos)
#define EXTI_IMR1_IM5 EXTI_IMR1_IM5_Msk

#define EXTI_IMR1_IM4_Pos 4U
#define EXTI_IMR1_IM4_Msk (0x1UL << EXTI_IMR1_IM4_Pos)
#define EXTI_IMR1_IM4 EXTI_IMR1_IM4_Msk

#define EXTI_IMR1_IM3_Pos 3U
#define EXTI_IMR1_IM3_Msk (0x1UL << EXTI_IMR1_IM3_Pos)
#define EXTI_IMR1_IM3 EXTI_IMR1_IM3_Msk

#define EXTI_IMR1_IM2_Pos 2U
#define EXTI_IMR1_IM2_Msk (0x1UL << EXTI_IMR1_IM2_Pos)
#define EXTI_IMR1_IM2 EXTI_IMR1_IM2_Msk

#define EXTI_IMR1_IM1_Pos 1U
#define EXTI_IMR1_IM1_Msk (0x1UL << EXTI_IMR1_IM1_Pos)
#define EXTI_IMR1_IM1 EXTI_IMR1_IM1_Msk

#define EXTI_IMR1_IM0_Pos 0U
#define EXTI_IMR1_IM0_Msk (0x1UL << EXTI_IMR1_IM0_Pos)
#define EXTI_IMR1_IM0 EXTI_IMR1_IM0_Msk

/*EMR1 register*/
#define EXTI_EMR1_EM31_Pos 31U
#define EXTI_EMR1_EM31_Msk (0x1UL << EXTI_EMR1_EM31_Pos)
#define EXTI_EMR1_EM31 EXTI_EMR1_EM31_Msk

#define EXTI_EMR1_EM28_Pos 28U
#define EXTI_EMR1_EM28_Msk (0x1UL << EXTI_EMR1_EM28_Pos)
#define EXTI_EMR1_EM28 EXTI_EMR1_EM28_Msk

#define EXTI_EMR1_EM27_Pos 27U
#define EXTI_EMR1_EM27_Msk (0x1UL << EXTI_EMR1_EM27_Pos)
#define EXTI_EMR1_EM27 EXTI_EMR1_EM27_Msk

#define EXTI_EMR1_EM26_Pos 26U
#define EXTI_EMR1_EM26_Msk (0x1UL << EXTI_EMR1_EM26_Pos)
#define EXTI_EMR1_EM26 EXTI_EMR1_EM26_Msk

#define EXTI_EMR1_EM25_Pos 25U
#define EXTI_EMR1_EM25_Msk (0x1UL << EXTI_EMR1_EM25_Pos)
#define EXTI_EMR1_EM25 EXTI_EMR1_EM25_Msk

#define EXTI_EMR1_EM24_Pos 24U
#define EXTI_EMR1_EM24_Msk (0x1UL << EXTI_EMR1_EM24_Pos)
#define EXTI_EMR1_EM24 EXTI_EMR1_EM24_Msk

#define EXTI_EMR1_EM23_Pos 23U
#define EXTI_EMR1_EM23_Msk (0x1UL << EXTI_EMR1_EM23_Pos)
#define EXTI_EMR1_EM23 EXTI_EMR1_EM23_Msk

#define EXTI_EMR1_EM22_Pos 22U
#define EXTI_EMR1_EM22_Msk (0x1UL << EXTI_EMR1_EM22_Pos)
#define EXTI_EMR1_EM22 EXTI_EMR1_EM22_Msk

#define EXTI_EMR1_EM21_Pos 21U
#define EXTI_EMR1_EM21_Msk (0x1UL << EXTI_EMR1_EM21_Pos)
#define EXTI_EMR1_EM21 EXTI_EMR1_EM21_Msk

#define EXTI_EMR1_EM20_Pos 20U
#define EXTI_EMR1_EM20_Msk (0x1UL << EXTI_EMR1_EM20_Pos)
#define EXTI_EMR1_EM20 EXTI_EMR1_EM20_Msk

#define EXTI_EMR1_EM19_Pos 19U
#define EXTI_EMR1_EM19_Msk (0x1UL << EXTI_EMR1_EM19_Pos)
#define EXTI_EMR1_EM19 EXTI_EMR1_EM19_Msk

#define EXTI_EMR1_EM18_Pos 18U
#define EXTI_EMR1_EM18_Msk (0x1UL << EXTI_EMR1_EM18_Pos)
#define EXTI_EMR1_EM18 EXTI_EMR1_EM18_Msk

#define EXTI_EMR1_EM17_Pos 17U
#define EXTI_EMR1_EM17_Msk (0x1UL << EXTI_EMR1_EM17_Pos)
#define EXTI_EMR1_EM17 EXTI_EMR1_EM17_Msk

#define EXTI_EMR1_EM16_Pos 16U
#define EXTI_EMR1_EM16_Msk (0x1UL << EXTI_EMR1_EM16_Pos)
#define EXTI_EMR1_EM16 EXTI_EMR1_EM16_Msk

#define EXTI_EMR1_EM15_Pos 15U
#define EXTI_EMR1_EM15_Msk (0x1UL << EXTI_EMR1_EM15_Pos)
#define EXTI_EMR1_EM15 EXTI_EMR1_EM15_Msk

#define EXTI_EMR1_EM14_Pos 14U
#define EXTI_EMR1_EM14_Msk (0x1UL << EXTI_EMR1_EM14_Pos)
#define EXTI_EMR1_EM14 EXTI_EMR1_EM14_Msk

#define EXTI_EMR1_EM13_Pos 13U
#define EXTI_EMR1_EM13_Msk (0x1UL << EXTI_EMR1_EM13_Pos)
#define EXTI_EMR1_EM13 EXTI_EMR1_EM13_Msk

#define EXTI_EMR1_EM12_Pos 12U
#define EXTI_EMR1_EM12_Msk (0x1UL << EXTI_EMR1_EM12_Pos)
#define EXTI_EMR1_EM12 EXTI_EMR1_EM12_Msk

#define EXTI_EMR1_EM11_Pos 11U
#define EXTI_EMR1_EM11_Msk (0x1UL << EXTI_EMR1_EM11_Pos)
#define EXTI_EMR1_EM11 EXTI_EMR1_EM11_Msk

#define EXTI_EMR1_EM10_Pos 10U
#define EXTI_EMR1_EM10_Msk (0x1UL << EXTI_EMR1_EM10_Pos)
#define EXTI_EMR1_EM10 EXTI_EMR1_EM10_Msk

#define EXTI_EMR1_EM9_Pos 9U
#define EXTI_EMR1_EM9_Msk (0x1UL << EXTI_EMR1_EM9_Pos)
#define EXTI_EMR1_EM9 EXTI_EMR1_EM9_Msk

#define EXTI_EMR1_EM8_Pos 8U
#define EXTI_EMR1_EM8_Msk (0x1UL << EXTI_EMR1_EM8_Pos)
#define EXTI_EMR1_EM8 EXTI_EMR1_EM8_Msk

#define EXTI_EMR1_EM7_Pos 7U
#define EXTI_EMR1_EM7_Msk (0x1UL << EXTI_EMR1_EM7_Pos)
#define EXTI_EMR1_EM7 EXTI_EMR1_EM7_Msk

#define EXTI_EMR1_EM6_Pos 6U
#define EXTI_EMR1_EM6_Msk (0x1UL << EXTI_EMR1_EM6_Pos)
#define EXTI_EMR1_EM6 EXTI_EMR1_EM6_Msk

#define EXTI_EMR1_EM5_Pos 5U
#define EXTI_EMR1_EM5_Msk (0x1UL << EXTI_EMR1_EM5_Pos)
#define EXTI_EMR1_EM5 EXTI_EMR1_EM5_Msk

#define EXTI_EMR1_EM4_Pos 4U
#define EXTI_EMR1_EM4_Msk (0x1UL << EXTI_EMR1_EM4_Pos)
#define EXTI_EMR1_EM4 EXTI_EMR1_EM4_Msk

#define EXTI_EMR1_EM3_Pos 3U
#define EXTI_EMR1_EM3_Msk (0x1UL << EXTI_EMR1_EM3_Pos)
#define EXTI_EMR1_EM3 EXTI_EMR1_EM3_Msk

#define EXTI_EMR1_EM2_Pos 2U
#define EXTI_EMR1_EM2_Msk (0x1UL << EXTI_EMR1_EM2_Pos)
#define EXTI_EMR1_EM2 EXTI_EMR1_EM2_Msk

#define EXTI_EMR1_EM1_Pos 1U
#define EXTI_EMR1_EM1_Msk (0x1UL << EXTI_EMR1_EM1_Pos)
#define EXTI_EMR1_EM1 EXTI_EMR1_EM1_Msk

#define EXTI_EMR1_EM0_Pos 0U
#define EXTI_EMR1_EM0_Msk (0x1UL << EXTI_EMR1_EM0_Pos)
#define EXTI_EMR1_EM0 EXTI_EMR1_EM0_Msk

/*RTSR1 register*/
#define EXTI_RTSR1_RT22_Pos 22U
#define EXTI_RTSR1_RT22_Msk (0x1UL << EXTI_RTSR1_RT22_Pos)
#define EXTI_RTSR1_RT22 EXTI_RTSR1_RT22_Msk

#define EXTI_RTSR1_RT21_Pos 21U
#define EXTI_RTSR1_RT21_Msk (0x1UL << EXTI_RTSR1_RT21_Pos)
#define EXTI_RTSR1_RT21 EXTI_RTSR1_RT21_Msk

#define EXTI_RTSR1_RT20_Pos 20U
#define EXTI_RTSR1_RT20_Msk (0x1UL << EXTI_RTSR1_RT20_Pos)
#define EXTI_RTSR1_RT20 EXTI_RTSR1_RT20_Msk

#define EXTI_RTSR1_RT19_Pos 19U
#define EXTI_RTSR1_RT19_Msk (0x1UL << EXTI_RTSR1_RT19_Pos)
#define EXTI_RTSR1_RT19 EXTI_RTSR1_RT19_Msk

#define EXTI_RTSR1_RT18_Pos 18U
#define EXTI_RTSR1_RT18_Msk (0x1UL << EXTI_RTSR1_RT18_Pos)
#define EXTI_RTSR1_RT18 EXTI_RTSR1_RT18_Msk

#define EXTI_RTSR1_RT16_Pos 16U
#define EXTI_RTSR1_RT16_Msk (0x1UL << EXTI_RTSR1_RT16_Pos)
#define EXTI_RTSR1_RT16 EXTI_RTSR1_RT16_Msk

#define EXTI_RTSR1_RT15_Pos 15U
#define EXTI_RTSR1_RT15_Msk (0x1UL << EXTI_RTSR1_RT15_Pos)
#define EXTI_RTSR1_RT15 EXTI_RTSR1_RT15_Msk

#define EXTI_RTSR1_RT14_Pos 14U
#define EXTI_RTSR1_RT14_Msk (0x1UL << EXTI_RTSR1_RT14_Pos)
#define EXTI_RTSR1_RT14 EXTI_RTSR1_RT14_Msk

#define EXTI_RTSR1_RT13_Pos 13U
#define EXTI_RTSR1_RT13_Msk (0x1UL << EXTI_RTSR1_RT13_Pos)
#define EXTI_RTSR1_RT13 EXTI_RTSR1_RT13_Msk

#define EXTI_RTSR1_RT12_Pos 12U
#define EXTI_RTSR1_RT12_Msk (0x1UL << EXTI_RTSR1_RT12_Pos)
#define EXTI_RTSR1_RT12 EXTI_RTSR1_RT12_Msk

#define EXTI_RTSR1_RT11_Pos 11U
#define EXTI_RTSR1_RT11_Msk (0x1UL << EXTI_RTSR1_RT11_Pos)
#define EXTI_RTSR1_RT11 EXTI_RTSR1_RT11_Msk

#define EXTI_RTSR1_RT10_Pos 10U
#define EXTI_RTSR1_RT10_Msk (0x1UL << EXTI_RTSR1_RT10_Pos)
#define EXTI_RTSR1_RT10 EXTI_RTSR1_RT10_Msk

#define EXTI_RTSR1_RT9_Pos 9U
#define EXTI_RTSR1_RT9_Msk (0x1UL << EXTI_RTSR1_RT9_Pos)
#define EXTI_RTSR1_RT9 EXTI_RTSR1_RT9_Msk

#define EXTI_RTSR1_RT8_Pos 8U
#define EXTI_RTSR1_RT8_Msk (0x1UL << EXTI_RTSR1_RT8_Pos)
#define EXTI_RTSR1_RT8 EXTI_RTSR1_RT8_Msk

#define EXTI_RTSR1_RT7_Pos 7U
#define EXTI_RTSR1_RT7_Msk (0x1UL << EXTI_RTSR1_RT7_Pos)
#define EXTI_RTSR1_RT7 EXTI_RTSR1_RT7_Msk

#define EXTI_RTSR1_RT6_Pos 6U
#define EXTI_RTSR1_RT6_Msk (0x1UL << EXTI_RTSR1_RT6_Pos)
#define EXTI_RTSR1_RT6 EXTI_RTSR1_RT6_Msk

#define EXTI_RTSR1_RT5_Pos 5U
#define EXTI_RTSR1_RT5_Msk (0x1UL << EXTI_RTSR1_RT5_Pos)
#define EXTI_RTSR1_RT5 EXTI_RTSR1_RT5_Msk

#define EXTI_RTSR1_RT4_Pos 4U
#define EXTI_RTSR1_RT4_Msk (0x1UL << EXTI_RTSR1_RT4_Pos)
#define EXTI_RTSR1_RT4 EXTI_RTSR1_RT4_Msk

#define EXTI_RTSR1_RT3_Pos 3U
#define EXTI_RTSR1_RT3_Msk (0x1UL << EXTI_RTSR1_RT3_Pos)
#define EXTI_RTSR1_RT3 EXTI_RTSR1_RT3_Msk

#define EXTI_RTSR1_RT2_Pos 2U
#define EXTI_RTSR1_RT2_Msk (0x1UL << EXTI_RTSR1_RT2_Pos)
#define EXTI_RTSR1_RT2 EXTI_RTSR1_RT2_Msk

#define EXTI_RTSR1_RT1_Pos 1U
#define EXTI_RTSR1_RT1_Msk (0x1UL << EXTI_RTSR1_RT1_Pos)
#define EXTI_RTSR1_RT1 EXTI_RTSR1_RT1_Msk

#define EXTI_RTSR1_RT0_Pos 0U
#define EXTI_RTSR1_RT0_Msk (0x1UL << EXTI_RTSR1_RT0_Pos)
#define EXTI_RTSR1_RT0 EXTI_RTSR1_RT0_Msk

/*FTSR1 register*/
#define EXTI_FTSR1_FT22_Pos 22U
#define EXTI_FTSR1_FT22_Msk (0x1UL << EXTI_FTSR1_FT22_Pos)
#define EXTI_FTSR1_FT22 EXTI_FTSR1_FT22_Msk

#define EXTI_FTSR1_FT21_Pos 21U
#define EXTI_FTSR1_FT21_Msk (0x1UL << EXTI_FTSR1_FT21_Pos)
#define EXTI_FTSR1_FT21 EXTI_FTSR1_FT21_Msk

#define EXTI_FTSR1_FT20_Pos 20U
#define EXTI_FTSR1_FT20_Msk (0x1UL << EXTI_FTSR1_FT20_Pos)
#define EXTI_FTSR1_FT20 EXTI_FTSR1_FT20_Msk

#define EXTI_FTSR1_FT19_Pos 19U
#define EXTI_FTSR1_FT19_Msk (0x1UL << EXTI_FTSR1_FT19_Pos)
#define EXTI_FTSR1_FT19 EXTI_FTSR1_FT19_Msk

#define EXTI_FTSR1_FT18_Pos 18U
#define EXTI_FTSR1_FT18_Msk (0x1UL << EXTI_FTSR1_FT18_Pos)
#define EXTI_FTSR1_FT18 EXTI_FTSR1_FT18_Msk

#define EXTI_FTSR1_FT16_Pos 16U
#define EXTI_FTSR1_FT16_Msk (0x1UL << EXTI_FTSR1_FT16_Pos)
#define EXTI_FTSR1_FT16 EXTI_FTSR1_FT16_Msk

#define EXTI_FTSR1_FT15_Pos 15U
#define EXTI_FTSR1_FT15_Msk (0x1UL << EXTI_FTSR1_FT15_Pos)
#define EXTI_FTSR1_FT15 EXTI_FTSR1_FT15_Msk

#define EXTI_FTSR1_FT14_Pos 14U
#define EXTI_FTSR1_FT14_Msk (0x1UL << EXTI_FTSR1_FT14_Pos)
#define EXTI_FTSR1_FT14 EXTI_FTSR1_FT14_Msk

#define EXTI_FTSR1_FT13_Pos 13U
#define EXTI_FTSR1_FT13_Msk (0x1UL << EXTI_FTSR1_FT13_Pos)
#define EXTI_FTSR1_FT13 EXTI_FTSR1_FT13_Msk

#define EXTI_FTSR1_FT12_Pos 12U
#define EXTI_FTSR1_FT12_Msk (0x1UL << EXTI_FTSR1_FT12_Pos)
#define EXTI_FTSR1_FT12 EXTI_FTSR1_FT12_Msk

#define EXTI_FTSR1_FT11_Pos 11U
#define EXTI_FTSR1_FT11_Msk (0x1UL << EXTI_FTSR1_FT11_Pos)
#define EXTI_FTSR1_FT11 EXTI_FTSR1_FT11_Msk

#define EXTI_FTSR1_FT10_Pos 10U
#define EXTI_FTSR1_FT10_Msk (0x1UL << EXTI_FTSR1_FT10_Pos)
#define EXTI_FTSR1_FT10 EXTI_FTSR1_FT10_Msk

#define EXTI_FTSR1_FT9_Pos 9U
#define EXTI_FTSR1_FT9_Msk (0x1UL << EXTI_FTSR1_FT9_Pos)
#define EXTI_FTSR1_FT9 EXTI_FTSR1_FT9_Msk

#define EXTI_FTSR1_FT8_Pos 8U
#define EXTI_FTSR1_FT8_Msk (0x1UL << EXTI_FTSR1_FT8_Pos)
#define EXTI_FTSR1_FT8 EXTI_FTSR1_FT8_Msk

#define EXTI_FTSR1_FT7_Pos 7U
#define EXTI_FTSR1_FT7_Msk (0x1UL << EXTI_FTSR1_FT7_Pos)
#define EXTI_FTSR1_FT7 EXTI_FTSR1_FT7_Msk

#define EXTI_FTSR1_FT6_Pos 6U
#define EXTI_FTSR1_FT6_Msk (0x1UL << EXTI_FTSR1_FT6_Pos)
#define EXTI_FTSR1_FT6 EXTI_FTSR1_FT6_Msk

#define EXTI_FTSR1_FT5_Pos 5U
#define EXTI_FTSR1_FT5_Msk (0x1UL << EXTI_FTSR1_FT5_Pos)
#define EXTI_FTSR1_FT5 EXTI_FTSR1_FT5_Msk

#define EXTI_FTSR1_FT4_Pos 4U
#define EXTI_FTSR1_FT4_Msk (0x1UL << EXTI_FTSR1_FT4_Pos)
#define EXTI_FTSR1_FT4 EXTI_FTSR1_FT4_Msk

#define EXTI_FTSR1_FT3_Pos 3U
#define EXTI_FTSR1_FT3_Msk (0x1UL << EXTI_FTSR1_FT3_Pos)
#define EXTI_FTSR1_FT3 EXTI_FTSR1_FT3_Msk

#define EXTI_FTSR1_FT2_Pos 2U
#define EXTI_FTSR1_FT2_Msk (0x1UL << EXTI_FTSR1_FT2_Pos)
#define EXTI_FTSR1_FT2 EXTI_FTSR1_FT2_Msk

#define EXTI_FTSR1_FT1_Pos 1U
#define EXTI_FTSR1_FT1_Msk (0x1UL << EXTI_FTSR1_FT1_Pos)
#define EXTI_FTSR1_FT1 EXTI_FTSR1_FT1_Msk

#define EXTI_FTSR1_FT0_Pos 0U
#define EXTI_FTSR1_FT0_Msk (0x1UL << EXTI_FTSR1_FT0_Pos)
#define EXTI_FTSR1_FT0 EXTI_FTSR1_FT0_Msk

/*SWIER1 register*/
#define EXTI_SWIER1_SWI22_Pos 22U
#define EXTI_SWIER1_SWI22_Msk (0x1UL << EXTI_SWIER1_SWI22_Pos)
#define EXTI_SWIER1_SWI22 EXTI_SWIER1_SWI22_Msk

#define EXTI_SWIER1_SWI21_Pos 21U
#define EXTI_SWIER1_SWI21_Msk (0x1UL << EXTI_SWIER1_SWI21_Pos)
#define EXTI_SWIER1_SWI21 EXTI_SWIER1_SWI21_Msk

#define EXTI_SWIER1_SWI20_Pos 20U
#define EXTI_SWIER1_SWI20_Msk (0x1UL << EXTI_SWIER1_SWI20_Pos)
#define EXTI_SWIER1_SWI20 EXTI_SWIER1_SWI20_Msk

#define EXTI_SWIER1_SWI19_Pos 19U
#define EXTI_SWIER1_SWI19_Msk (0x1UL << EXTI_SWIER1_SWI19_Pos)
#define EXTI_SWIER1_SWI19 EXTI_SWIER1_SWI19_Msk

#define EXTI_SWIER1_SWI18_Pos 18U
#define EXTI_SWIER1_SWI18_Msk (0x1UL << EXTI_SWIER1_SWI18_Pos)
#define EXTI_SWIER1_SWI18 EXTI_SWIER1_SWI18_Msk

#define EXTI_SWIER1_SWI16_Pos 16U
#define EXTI_SWIER1_SWI16_Msk (0x1UL << EXTI_SWIER1_SWI16_Pos)
#define EXTI_SWIER1_SWI16 EXTI_SWIER1_SWI16_Msk

#define EXTI_SWIER1_SWI15_Pos 15U
#define EXTI_SWIER1_SWI15_Msk (0x1UL << EXTI_SWIER1_SWI15_Pos)
#define EXTI_SWIER1_SWI15 EXTI_SWIER1_SWI15_Msk

#define EXTI_SWIER1_SWI14_Pos 14U
#define EXTI_SWIER1_SWI14_Msk (0x1UL << EXTI_SWIER1_SWI14_Pos)
#define EXTI_SWIER1_SWI14 EXTI_SWIER1_SWI14_Msk

#define EXTI_SWIER1_SWI13_Pos 13U
#define EXTI_SWIER1_SWI13_Msk (0x1UL << EXTI_SWIER1_SWI13_Pos)
#define EXTI_SWIER1_SWI13 EXTI_SWIER1_SWI13_Msk

#define EXTI_SWIER1_SWI12_Pos 12U
#define EXTI_SWIER1_SWI12_Msk (0x1UL << EXTI_SWIER1_SWI12_Pos)
#define EXTI_SWIER1_SWI12 EXTI_SWIER1_SWI12_Msk

#define EXTI_SWIER1_SWI11_Pos 11U
#define EXTI_SWIER1_SWI11_Msk (0x1UL << EXTI_SWIER1_SWI11_Pos)
#define EXTI_SWIER1_SWI11 EXTI_SWIER1_SWI11_Msk

#define EXTI_SWIER1_SWI10_Pos 10U
#define EXTI_SWIER1_SWI10_Msk (0x1UL << EXTI_SWIER1_SWI10_Pos)
#define EXTI_SWIER1_SWI10 EXTI_SWIER1_SWI10_Msk

#define EXTI_SWIER1_SWI9_Pos 9U
#define EXTI_SWIER1_SWI9_Msk (0x1UL << EXTI_SWIER1_SWI9_Pos)
#define EXTI_SWIER1_SWI9 EXTI_SWIER1_SWI9_Msk

#define EXTI_SWIER1_SWI8_Pos 8U
#define EXTI_SWIER1_SWI8_Msk (0x1UL << EXTI_SWIER1_SWI8_Pos)
#define EXTI_SWIER1_SWI8 EXTI_SWIER1_SWI8_Msk

#define EXTI_SWIER1_SWI7_Pos 7U
#define EXTI_SWIER1_SWI7_Msk (0x1UL << EXTI_SWIER1_SWI7_Pos)
#define EXTI_SWIER1_SWI7 EXTI_SWIER1_SWI7_Msk

#define EXTI_SWIER1_SWI6_Pos 6U
#define EXTI_SWIER1_SWI6_Msk (0x1UL << EXTI_SWIER1_SWI6_Pos)
#define EXTI_SWIER1_SWI6 EXTI_SWIER1_SWI6_Msk

#define EXTI_SWIER1_SWI5_Pos 5U
#define EXTI_SWIER1_SWI5_Msk (0x1UL << EXTI_SWIER1_SWI5_Pos)
#define EXTI_SWIER1_SWI5 EXTI_SWIER1_SWI5_Msk

#define EXTI_SWIER1_SWI4_Pos 4U
#define EXTI_SWIER1_SWI4_Msk (0x1UL << EXTI_SWIER1_SWI4_Pos)
#define EXTI_SWIER1_SWI4 EXTI_SWIER1_SWI4_Msk

#define EXTI_SWIER1_SWI3_Pos 3U
#define EXTI_SWIER1_SWI3_Msk (0x1UL << EXTI_SWIER1_SWI3_Pos)
#define EXTI_SWIER1_SWI3 EXTI_SWIER1_SWI3_Msk

#define EXTI_SWIER1_SWI2_Pos 2U
#define EXTI_SWIER1_SWI2_Msk (0x1UL << EXTI_SWIER1_SWI2_Pos)
#define EXTI_SWIER1_SWI2 EXTI_SWIER1_SWI2_Msk

#define EXTI_SWIER1_SWI1_Pos 1U
#define EXTI_SWIER1_SWI1_Msk (0x1UL << EXTI_SWIER1_SWI1_Pos)
#define EXTI_SWIER1_SWI1 EXTI_SWIER1_SWI1_Msk

#define EXTI_SWIER1_SWI0_Pos 0U
#define EXTI_SWIER1_SWI0_Msk (0x1UL << EXTI_SWIER1_SWI0_Pos)
#define EXTI_SWIER1_SWI0 EXTI_SWIER1_SWI0_Msk

/*PR1 register*/
#define EXTI_PR1_PIF22_Pos 22U
#define EXTI_PR1_PIF22_Msk (0x1UL << EXTI_PR1_PIF22_Pos)
#define EXTI_PR1_PIF22 EXTI_PR1_PIF22_Msk

#define EXTI_PR1_PIF21_Pos 21U
#define EXTI_PR1_PIF21_Msk (0x1UL << EXTI_PR1_PIF21_Pos)
#define EXTI_PR1_PIF21 EXTI_PR1_PIF21_Msk

#define EXTI_PR1_PIF20_Pos 20U
#define EXTI_PR1_PIF20_Msk (0x1UL << EXTI_PR1_PIF20_Pos)
#define EXTI_PR1_PIF20 EXTI_PR1_PIF20_Msk

#define EXTI_PR1_PIF19_Pos 19U
#define EXTI_PR1_PIF19_Msk (0x1UL << EXTI_PR1_PIF19_Pos)
#define EXTI_PR1_PIF19 EXTI_PR1_PIF19_Msk

#define EXTI_PR1_PIF18_Pos 18U
#define EXTI_PR1_PIF18_Msk (0x1UL << EXTI_PR1_PIF18_Pos)
#define EXTI_PR1_PIF18 EXTI_PR1_PIF18_Msk

#define EXTI_PR1_PIF16_Pos 16U
#define EXTI_PR1_PIF16_Msk (0x1UL << EXTI_PR1_PIF16_Pos)
#define EXTI_PR1_PIF16 EXTI_PR1_PIF16_Msk

#define EXTI_PR1_PIF15_Pos 15U
#define EXTI_PR1_PIF15_Msk (0x1UL << EXTI_PR1_PIF15_Pos)
#define EXTI_PR1_PIF15 EXTI_PR1_PIF15_Msk

#define EXTI_PR1_PIF14_Pos 14U
#define EXTI_PR1_PIF14_Msk (0x1UL << EXTI_PR1_PIF14_Pos)
#define EXTI_PR1_PIF14 EXTI_PR1_PIF14_Msk

#define EXTI_PR1_PIF13_Pos 13U
#define EXTI_PR1_PIF13_Msk (0x1UL << EXTI_PR1_PIF13_Pos)
#define EXTI_PR1_PIF13 EXTI_PR1_PIF13_Msk

#define EXTI_PR1_PIF12_Pos 12U
#define EXTI_PR1_PIF12_Msk (0x1UL << EXTI_PR1_PIF12_Pos)
#define EXTI_PR1_PIF12 EXTI_PR1_PIF12_Msk

#define EXTI_PR1_PIF11_Pos 11U
#define EXTI_PR1_PIF11_Msk (0x1UL << EXTI_PR1_PIF11_Pos)
#define EXTI_PR1_PIF11 EXTI_PR1_PIF11_Msk

#define EXTI_PR1_PIF10_Pos 10U
#define EXTI_PR1_PIF10_Msk (0x1UL << EXTI_PR1_PIF10_Pos)
#define EXTI_PR1_PIF10 EXTI_PR1_PIF10_Msk

#define EXTI_PR1_PIF9_Pos 9U
#define EXTI_PR1_PIF9_Msk (0x1UL << EXTI_PR1_PIF9_Pos)
#define EXTI_PR1_PIF9 EXTI_PR1_PIF9_Msk

#define EXTI_PR1_PIF8_Pos 8U
#define EXTI_PR1_PIF8_Msk (0x1UL << EXTI_PR1_PIF8_Pos)
#define EXTI_PR1_PIF8 EXTI_PR1_PIF8_Msk

#define EXTI_PR1_PIF7_Pos 7U
#define EXTI_PR1_PIF7_Msk (0x1UL << EXTI_PR1_PIF7_Pos)
#define EXTI_PR1_PIF7 EXTI_PR1_PIF7_Msk

#define EXTI_PR1_PIF6_Pos 6U
#define EXTI_PR1_PIF6_Msk (0x1UL << EXTI_PR1_PIF6_Pos)
#define EXTI_PR1_PIF6 EXTI_PR1_PIF6_Msk

#define EXTI_PR1_PIF5_Pos 5U
#define EXTI_PR1_PIF5_Msk (0x1UL << EXTI_PR1_PIF5_Pos)
#define EXTI_PR1_PIF5 EXTI_PR1_PIF5_Msk

#define EXTI_PR1_PIF4_Pos 4U
#define EXTI_PR1_PIF4_Msk (0x1UL << EXTI_PR1_PIF4_Pos)
#define EXTI_PR1_PIF4 EXTI_PR1_PIF4_Msk

#define EXTI_PR1_PIF3_Pos 3U
#define EXTI_PR1_PIF3_Msk (0x1UL << EXTI_PR1_PIF3_Pos)
#define EXTI_PR1_PIF3 EXTI_PR1_PIF3_Msk

#define EXTI_PR1_PIF2_Pos 2U
#define EXTI_PR1_PIF2_Msk (0x1UL << EXTI_PR1_PIF2_Pos)
#define EXTI_PR1_PIF2 EXTI_PR1_PIF2_Msk

#define EXTI_PR1_PIF1_Pos 1U
#define EXTI_PR1_PIF1_Msk (0x1UL << EXTI_PR1_PIF1_Pos)
#define EXTI_PR1_PIF1 EXTI_PR1_PIF1_Msk

#define EXTI_PR1_PIF0_Pos 0U
#define EXTI_PR1_PIF0_Msk (0x1UL << EXTI_PR1_PIF0_Pos)
#define EXTI_PR1_PIF0 EXTI_PR1_PIF0_Msk

/*IMR2 register*/
#define EXTI_IMR2_IM39_Pos 7U
#define EXTI_IMR2_IM39_Msk (0x1UL << EXTI_IMR2_IM39_Pos)
#define EXTI_IMR2_IM39 EXTI_IMR2_IM39_Msk

#define EXTI_IMR2_IM38_Pos 6U
#define EXTI_IMR2_IM38_Msk (0x1UL << EXTI_IMR2_IM38_Pos)
#define EXTI_IMR2_IM38 EXTI_IMR2_IM38_Msk

#define EXTI_IMR2_IM37_Pos 5U
#define EXTI_IMR2_IM37_Msk (0x1UL << EXTI_IMR2_IM37_Pos)
#define EXTI_IMR2_IM37 EXTI_IMR2_IM37_Msk

#define EXTI_IMR2_IM36_Pos 4U
#define EXTI_IMR2_IM36_Msk (0x1UL << EXTI_IMR2_IM36_Pos)
#define EXTI_IMR2_IM36 EXTI_IMR2_IM36_Msk

#define EXTI_IMR2_IM35_Pos 3U
#define EXTI_IMR2_IM35_Msk (0x1UL << EXTI_IMR2_IM35_Pos)
#define EXTI_IMR2_IM35 EXTI_IMR2_IM35_Msk

#define EXTI_IMR2_IM34_Pos 2U
#define EXTI_IMR2_IM34_Msk (0x1UL << EXTI_IMR2_IM34_Pos)
#define EXTI_IMR2_IM34 EXTI_IMR2_IM34_Msk

#define EXTI_IMR2_IM33_Pos 1U
#define EXTI_IMR2_IM33_Msk (0x1UL << EXTI_IMR2_IM33_Pos)
#define EXTI_IMR2_IM33 EXTI_IMR2_IM33_Msk

#define EXTI_IMR2_IM32_Pos 0U
#define EXTI_IMR2_IM32_Msk (0x1UL << EXTI_IMR2_IM32_Pos)
#define EXTI_IMR2_IM32 EXTI_IMR2_IM32_Msk

/*EMR2 register*/
#define EXTI_EMR2_EM39_Pos 7U
#define EXTI_EMR2_EM39_Msk (0x1UL << EXTI_EMR2_EM39_Pos)
#define EXTI_EMR2_EM39 EXTI_EMR2_EM39_Msk

#define EXTI_EMR2_EM38_Pos 6U
#define EXTI_EMR2_EM38_Msk (0x1UL << EXTI_EMR2_EM38_Pos)
#define EXTI_EMR2_EM38 EXTI_EMR2_EM38_Msk

#define EXTI_EMR2_EM37_Pos 5U
#define EXTI_EMR2_EM37_Msk (0x1UL << EXTI_EMR2_EM37_Pos)
#define EXTI_EMR2_EM37 EXTI_EMR2_EM37_Msk

#define EXTI_EMR2_EM36_Pos 4U
#define EXTI_EMR2_EM36_Msk (0x1UL << EXTI_EMR2_EM36_Pos)
#define EXTI_EMR2_EM36 EXTI_EMR2_EM36_Msk

#define EXTI_EMR2_EM35_Pos 3U
#define EXTI_EMR2_EM35_Msk (0x1UL << EXTI_EMR2_EM35_Pos)
#define EXTI_EMR2_EM35 EXTI_EMR2_EM35_Msk

#define EXTI_EMR2_EM34_Pos 2U
#define EXTI_EMR2_EM34_Msk (0x1UL << EXTI_EMR2_EM34_Pos)
#define EXTI_EMR2_EM34 EXTI_EMR2_EM34_Msk

#define EXTI_EMR2_EM33_Pos 1U
#define EXTI_EMR2_EM33_Msk (0x1UL << EXTI_EMR2_EM33_Pos)
#define EXTI_EMR2_EM33 EXTI_EMR2_EM33_Msk

#define EXTI_EMR2_EM32_Pos 0U
#define EXTI_EMR2_EM32_Msk (0x1UL << EXTI_EMR2_EM32_Pos)
#define EXTI_EMR2_EM32 EXTI_EMR2_EM32_Msk

/*RTSR2 register*/
#define EXTI_RTSR2_RT38_Pos 6U
#define EXTI_RTSR2_RT38_Msk (0x1UL << EXTI_RTSR2_RT38_Pos)
#define EXTI_RTSR2_RT38 EXTI_RTSR2_RT38_Msk

#define EXTI_RTSR2_RT37_Pos 5U
#define EXTI_RTSR2_RT37_Msk (0x1UL << EXTI_RTSR2_RT37_Pos)
#define EXTI_RTSR2_RT37 EXTI_RTSR2_RT37_Msk

#define EXTI_RTSR2_RT36_Pos 4U
#define EXTI_RTSR2_RT36_Msk (0x1UL << EXTI_RTSR2_RT36_Pos)
#define EXTI_RTSR2_RT36 EXTI_RTSR2_RT36_Msk

#define EXTI_RTSR2_RT35_Pos 3U
#define EXTI_RTSR2_RT35_Msk (0x1UL << EXTI_RTSR2_RT35_Pos)
#define EXTI_RTSR2_RT35 EXTI_RTSR2_RT35_Msk

/*FTSR2 register*/
#define EXTI_FTSR2_FT38_Pos 6U
#define EXTI_FTSR2_FT38_Msk (0x1UL << EXTI_FTSR2_FT38_Pos)
#define EXTI_FTSR2_FT38 EXTI_FTSR2_FT38_Msk

#define EXTI_FTSR2_FT37_Pos 5U
#define EXTI_FTSR2_FT37_Msk (0x1UL << EXTI_FTSR2_FT37_Pos)
#define EXTI_FTSR2_FT37 EXTI_FTSR2_FT37_Msk

#define EXTI_FTSR2_FT36_Pos 4U
#define EXTI_FTSR2_FT36_Msk (0x1UL << EXTI_FTSR2_FT36_Pos)
#define EXTI_FTSR2_FT36 EXTI_FTSR2_FT36_Msk

#define EXTI_FTSR2_FT35_Pos 3U
#define EXTI_FTSR2_FT35_Msk (0x1UL << EXTI_FTSR2_FT35_Pos)
#define EXTI_FTSR2_FT35 EXTI_FTSR2_FT35_Msk

/*SWIER2 register*/
#define EXTI_SWIER2_SWI38_Pos 6U
#define EXTI_SWIER2_SWI38_Msk (0x1UL << EXTI_SWIER2_SWI38_Pos)
#define EXTI_SWIER2_SWI38 EXTI_SWIER2_SWI38_Msk

#define EXTI_SWIER2_SWI37_Pos 5U
#define EXTI_SWIER2_SWI37_Msk (0x1UL << EXTI_SWIER2_SWI37_Pos)
#define EXTI_SWIER2_SWI37 EXTI_SWIER2_SWI37_Msk

#define EXTI_SWIER2_SWI36_Pos 4U
#define EXTI_SWIER2_SWI36_Msk (0x1UL << EXTI_SWIER2_SWI36_Pos)
#define EXTI_SWIER2_SWI36 EXTI_SWIER2_SWI36_Msk

#define EXTI_SWIER2_SWI35_Pos 3U
#define EXTI_SWIER2_SWI35_Msk (0x1UL << EXTI_SWIER2_SWI35_Pos)
#define EXTI_SWIER2_SWI35 EXTI_SWIER2_SWI35_Msk

/*PR2 register*/
#define EXTI_PR2_PIF38_Pos 6U
#define EXTI_PR2_PIF38_Msk (0x1UL << EXTI_PR2_PIF38_Pos)
#define EXTI_PR2_PIF38 EXTI_PR2_PIF38_Msk

#define EXTI_PR2_PIF37_Pos 5U
#define EXTI_PR2_PIF37_Msk (0x1UL << EXTI_PR2_PIF37_Pos)
#define EXTI_PR2_PIF37 EXTI_PR2_PIF37_Msk

#define EXTI_PR2_PIF36_Pos 4U
#define EXTI_PR2_PIF36_Msk (0x1UL << EXTI_PR2_PIF36_Pos)
#define EXTI_PR2_PIF36 EXTI_PR2_PIF36_Msk

#define EXTI_PR2_PIF35_Pos 3U
#define EXTI_PR2_PIF35_Msk (0x1UL << EXTI_PR2_PIF35_Pos)
#define EXTI_PR2_PIF35 EXTI_PR2_PIF35_Msk

/* ========================================================================= */
/* ============                       CRC                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t DR; /*!< Data Register. */
	__IOM uint32_t IDR; /*!< Independent Data Register. */
	__IOM uint32_t CR; /*!< Control Register. */
	uint32_t RESERVED; 
	__IOM uint32_t INIT; /*!< Initial CRC value. */	
	__IOM uint32_t POL; /*!< CRC Polynomial. */
}STM32L4xx_CRC_TypeDef;

/*IDR register*/
#define CRC_IDR_Pos 0U
#define CRC_IDR_Msk (0xFFUL << CRC_IDR_Pos)
#define CRC_IDR CRC_IDR_Msk

/*CR register*/
#define CRC_CR_REV_OUT_Pos 7U
#define CRC_CR_REV_OUT_Msk (0x1UL << CRC_CR_REV_OUT_Pos)
#define CRC_CR_REV_OUT CRC_CR_REV_OUT_Msk

#define CRC_CR_REV_IN_Pos 5U
#define CRC_CR_REV_IN_Msk (0x3UL << CRC_CR_REV_IN_Pos)
#define CRC_CR_REV_IN CRC_CR_REV_IN_Msk
#define CRC_CR_REV_IN_NONE (0x0UL << CRC_CR_REV_IN_Pos)
#define CRC_CR_REV_IN_BYTE (0x1UL << CRC_CR_REV_IN_Pos)
#define CRC_CR_REV_IN_HALF_WORD (0x2UL << CRC_CR_REV_IN_Pos)
#define CRC_CR_REV_IN_WORD (0x3UL << CRC_CR_REV_IN_Pos)

#define CRC_CR_POLYSIZE_Pos 3U
#define CRC_CR_POLYSIZE_Msk (0x3U << CRC_CR_POLYSIZE_Pos)
#define CRC_CR_POLYSIZE CRC_CR_POLYSIZE_Msk
#define CRC_CR_POLYSIZE_32_BIT (0x0U << CRC_CR_POLYSIZE_Pos)
#define CRC_CR_POLYSIZE_16_BIT (0x1U << CRC_CR_POLYSIZE_Pos)
#define CRC_CR_POLYSIZE_8_BIT (0x2U << CRC_CR_POLYSIZE_Pos)
#define CRC_CR_POLYSIZE_7_BIT (0x3U << CRC_CR_POLYSIZE_Pos)

#define CRC_CR_RESET_Pos 0U
#define CRC_CR_RESET_Msk (0x1U << CRC_CR_RESET_Pos)
#define CRC_CR_RESET CRC_CR_RESET_Msk

/* ========================================================================= */
/* ============                     QUADSPI                     ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR; /*!< QUADSPI Control Register. */
	__IOM uint32_t DCR; /*!< QUADSPI Device Configuration Register. */
	__IM uint32_t SR; /*!< QUADSPI Status Register. */
	__OM uint32_t FCR; /*!< QUADSPI Flag Clear Register. */
	__IOM uint32_t DLR; /*!< QUADSPI Data Length Register. */
	__IOM uint32_t CCR; /*!< QUADSPI Communication Configuration Register. */
	__IOM uint32_t AR; /*!< QUADSPI Address Register. */
	__IOM uint32_t ABR; /*!< QUADSPI Alternate Bytes Register. */
	__IOM uint32_t DR; /*!< QUADSPI Data Register. */
	__IOM uint32_t PSMKR; /*!< QUADSPI Polling Status Mask Register. */
	__IOM uint32_t PSMAR; /*!< QUADSPI Polling Status Match Register. */
	__IOM uint32_t PIR; /*!< QUADSPI Polling Interval Register. */
	__IOM uint32_t LPTR; /*!< QUADSPI Low Power Timeout Register. */
}STM32L4xx_QUADSPI_TypeDef;

/*CR register*/
#define QUADSPI_CR_PRESCALER_Pos 24U
#define QUADSPI_CR_PRESCALER_Msk (0xFFUL << QUADSPI_CR_PRESCALER_Pos)
#define QUADSPI_CR_PRESCALER QUADSPI_CR_PRESCALER_Msk

#define QUADSPI_CR_PMM_Pos 23U
#define QUADSPI_CR_PMM_Msk (0x1UL << QUADSPI_CR_PMM_Pos)
#define QUADSPI_CR_PMM QUADSPI_CR_PMM_Msk

#define QUADSPI_CR_APMS_Pos 22U
#define QUADSPI_CR_APMS_Msk (0x1UL << QUADSPI_CR_APMS_Pos)
#define QUADSPI_CR_APMS QUADSPI_CR_APMS_Msk

#define QUADSPI_CR_TOIE_Pos 20U
#define QUADSPI_CR_TOIE_Msk (0x1UL << QUADSPI_CR_TOIE_Pos)
#define QUADSPI_CR_TOIE QUADSPI_CR_TOIE_Msk

#define QUADSPI_CR_SMIE_Pos 19U
#define QUADSPI_CR_SMIE_Msk (0x1UL << QUADSPI_CR_SMIE_Pos)
#define QUADSPI_CR_SMIE QUADSPI_CR_SMIE_Msk

#define QUADSPI_CR_FTIE_Pos 18U
#define QUADSPI_CR_FTIE_Msk (0x1UL << QUADSPI_CR_FTIE_Pos)
#define QUADSPI_CR_FTIE QUADSPI_CR_FTIE_Msk

#define QUADSPI_CR_TCIE_Pos 17U
#define QUADSPI_CR_TCIE_Msk (0x1UL << QUADSPI_CR_TCIE_Pos)
#define QUADSPI_CR_TCIE QUADSPI_CR_TCIE_Msk

#define QUADSPI_CR_TEIE_Pos 16U
#define QUADSPI_CR_TEIE_Msk (0x1UL << QUADSPI_CR_TEIE_Pos)
#define	QUADSPI_CR_TEIE QUADSPI_CR_TEIE_Msk

#define QUADSPI_CR_FTHRES_Pos 8U
#define QUADSPI_CR_FTHRES_Msk (0xFUL << QUADSPI_CR_FTHRES_Pos)
#define QUADSPI_CR_FTHRES QUADSPI_CR_FTHRES_Msk

#define QUADSPI_CR_FSEL_Pos 7U
#define QUADSPI_CR_FSEL_Msk (0x1UL << QUADSPI_CR_FSEL_Pos)
#define QUADSPI_CR_FSEL QUADSPI_CR_FSEL_Msk

#define QUADSPI_CR_DFM_Pos 6U
#define QUADSPI_CR_DFM_Msk (0x1UL << QUADSPI_CR_DFM_Pos)
#define QUADSPI_CR_DFM QUADSPI_CR_DFM_Msk

#define QUADSPI_CR_SSHIFT_Pos 4U
#define QUADSPI_CR_SSHIFT_Msk (0x1UL << QUADSPI_CR_SSHIFT_Pos)
#define QUADSPI_CR_SSHIFT QUADSPI_CR_SSHIFT_Msk

#define QUADSPI_CR_TCEN_Pos 3U
#define QUADSPI_CR_TCEN_Msk (0x1UL << QUADSPI_CR_TCEN_Pos)
#define QUADSPI_CR_TCEN QUADSPI_CR_TCEN_Msk

#define QUADSPI_CR_DMAEN_Pos 2U
#define QUADSPI_CR_DMAEN_Msk (0x1UL << QUADSPI_CR_DMAEN_Pos)
#define QUADSPI_CR_DMAEN QUADSPI_CR_DMAEN_Msk

#define QUADSPI_CR_ABORT_Pos 1U
#define QUADSPI_CR_ABORT_Msk (0x1UL << QUADSPI_CR_ABORT_Pos)
#define QUADSPI_CR_ABORT QUADSPI_CR_ABORT_Msk

#define QUADSPI_CR_EN_Pos 0U
#define QUADSPI_CR_EN_Msk (0x1UL << QUADSPI_CR_EN_Pos)
#define QUADSPI_CR_EN QUADSPI_CR_EN_Msk

/*DCR register*/
#define QUADSPI_DCR_FSIZE_Pos 16U
#define QUADSPI_DCR_FSIZE_Msk (0x1FUL << QUADSPI_DCR_FSIZE_Pos)
#define QUADSPI_DCR_FSIZE QUADSPI_DCR_FSIZE_Msk

#define QUADSPI_DCR_CSHT_Pos 8U
#define QUADSPI_DCR_CSHT_Msk (0x7UL << QUADSPI_DCR_CSHT_Pos)
#define QUADSPI_DCR_CSHT QUADSPI_DCR_CSHT_Msk

#define QUADSPI_DCR_CKMODE_Pos 0U
#define QUADSPI_DCR_CKMODE_Msk (0x1UL << QUADSPI_DCR_CKMODE_Pos)
#define QUADSPI_DCR_CKMODE QUADSPI_DCR_CKMODE_Msk

/*SR register*/
#define QUADSPI_SR_FLEVEL_Pos 8U
#define QUADSPI_SR_FLEVEL_Msk (0x1FUL << QUADSPI_SR_FLEVEL_Pos)
#define QUADSPI_SR_FLEVEL QUADSPI_SR_FLEVEL_Msk

#define QUADSPI_SR_BUSY_Pos 5U
#define QUADSPI_SR_BUSY_Msk (0x1UL << QUADSPI_SR_BUSY_Pos)
#define QUADSPI_SR_BUSY QUADSPI_SR_BUSY_Msk

#define QUADSPI_SR_TOF_Pos 4U
#define QUADSPI_SR_TOF_Msk  (0x1UL << QUADSPI_SR_TOF_Pos)
#define QUADSPI_SR_TOF QUADSPI_SR_TOF_Msk

#define QUADSPI_SR_SMF_Pos 3U
#define QUADSPI_SR_SMF_Msk (0x1UL << QUADSPI_SR_SMF_Pos)
#define QUADSPI_SR_SMF QUADSPI_SR_SMF_Msk

#define QUADSPI_SR_FTF_Pos 2U
#define QUADSPI_SR_FTF_Msk (0x1UL << QUADSPI_SR_FTF_Pos)
#define QUADSPI_SR_FTF QUADSPI_SR_FTF_Msk

#define QUADSPI_SR_TCF_Pos 1U
#define QUADSPI_SR_TCF_Msk (0x1UL << QUADSPI_SR_TCF_Pos)
#define QUADSPI_SR_TCF QUADSPI_SR_TCF_Msk

#define QUADSPI_SR_TEF_Pos 0U
#define QUADSPI_SR_TEF_Msk (0x1UL << QUADSPI_SR_TEF_Pos)
#define QUADSPI_SR_TEF QUADSPI_SR_TEF_Msk

/*FCR register*/
#define QUADSPI_FCR_CTOF_Pos 4U
#define QUADSPI_FCR_CTOF_Msk (0x1UL << QUADSPI_FCR_CTOF_Pos)
#define QUADSPI_FCR_CTOF QUADSPI_FCR_CTOF_Msk

#define QUADSPI_FCR_CSMF_Pos 3U
#define QUADSPI_FCR_CSMF_Msk (0x1UL << QUADSPI_FCR_CSMF_Pos)
#define QUADSPI_FCR_CSMF QUADSPI_FCR_CSMF_Msk

#define QUADSPI_FCR_CTCF_Pos 1U
#define QUADSPI_FCR_CTCF_Msk  (0x1UL << QUADSPI_FCR_CTCF_Pos)
#define QUADSPI_FCR_CTCF QUADSPI_FCR_CTCF_Msk

#define QUADSPI_FCR_CTEF_Pos 0U
#define QUADSPI_FCR_CTEF_Msk (0x1UL << QUADSPI_FCR_CTEF_Pos)
#define QUADSPI_FCR_CTEF QUADSPI_FCR_CTEF_Msk

/*CCR register*/
#define QUADSPI_CCR_DDRM_Pos 31U
#define QUADSPI_CCR_DDRM_Msk (0x1UL << QUADSPI_CCR_DDRM_Pos)
#define QUADSPI_CCR_DDRM QUADSPI_CCR_DDRM_Msk

#define QUADSPI_CCR_DHHC_Pos 30U
#define QUADSPI_CCR_DHHC_Msk  (0x1UL << QUADSPI_CCR_DHHC_Pos)
#define QUADSPI_CCR_DHHC QUADSPI_CCR_DHHC_Msk

#define QUADSPI_CCR_SIOO_Pos 28U
#define QUADSPI_CCR_SIOO_Msk  (0x1UL << QUADSPI_CCR_SIOO_Pos)
#define QUADSPI_CCR_SIOO QUADSPI_CCR_SIOO_Msk

#define QUADSPI_CCR_FMODE_Pos 26U
#define QUADSPI_CCR_FMODE_Msk (0x3UL << QUADSPI_CCR_FMODE_Pos)
#define QUADSPI_CCR_FMODE QUADSPI_CCR_FMODE_Msk
#define QUADSPI_CCR_FMODE_INDIRECT_WRITE (0x0UL << QUADSPI_CCR_FMODE_Pos)
#define QUADSPI_CCR_FMODE_INDIRECT_READ (0x1UL << QUADSPI_CCR_FMODE_Pos)
#define QUADSPI_CCR_FMODE_AUTOMATIC_POLLING (0x2UL << QUADSPI_CCR_FMODE_Pos)
#define QUADSPI_CCR_FMODE_MEMORY_MAPPED (0x3UL << QUADSPI_CCR_FMODE_Pos)

#define QUADSPI_CCR_DMODE_Pos 24U
#define QUADSPI_CCR_DMODE_Msk (0x3UL << QUADSPI_CCR_DMODE_Pos)
#define QUADSPI_CCR_DMODE QUADSPI_CCR_DMODE_Msk
#define QUADSPI_CCR_DMODE_NONE (0x0UL << QUADSPI_CCR_DMODE_Pos)
#define QUADSPI_CCR_DMODE_1_LINE (0x1UL << QUADSPI_CCR_DMODE_Pos)
#define QUADSPI_CCR_DMODE_2_LINES (0x2UL << QUADSPI_CCR_DMODE_Pos)
#define QUADSPI_CCR_DMODE_4_LINES (0x3UL << QUADSPI_CCR_DMODE_Pos)

#define QUADSPI_CCR_DCYC_Pos 18U
#define QUADSPI_CCR_DCYC_Msk (0x1FUL << QUADSPI_CCR_DCYC_Pos)
#define QUADSPI_CCR_DCYC QUADSPI_CCR_DCYC_Msk

#define QUADSPI_CCR_ABSIZE_Pos 16U
#define QUADSPI_CCR_ABSIZE_Msk (0x3UL << QUADSPI_CCR_ABSIZE_Pos)
#define QUADSPI_CCR_ABSIZE QUADSPI_CCR_ABSIZE_Msk
#define QUADSPI_CCR_ABSIZE_8_BIT (0x0UL << QUADSPI_CCR_ABSIZE_Pos)
#define QUADSPI_CCR_ABSIZE_16_BIT (0x1UL << QUADSPI_CCR_ABSIZE_Pos)
#define QUADSPI_CCR_ABSIZE_24_BIT (0x2UL << QUADSPI_CCR_ABSIZE_Pos)
#define QUADSPI_CCR_ABSIZE_32_BIT (0x3UL << QUADSPI_CCR_ABSIZE_Pos)

#define QUADSPI_CCR_ABMODE_Pos 14U
#define QUADSPI_CCR_ABMODE_Msk (0x3UL << QUADSPI_CCR_ABMODE_Pos)
#define QUADSPI_CCR_ABMODE QUADSPI_CCR_ABMODE_Msk
#define QUADSPI_CCR_ABMODE_NONE (0x0UL << QUADSPI_CCR_ABMODE_Pos)
#define QUADSPI_CCR_ABMODE_1_LINE (0x1UL << QUADSPI_CCR_ABMODE_Pos)
#define QUADSPI_CCR_ABMODE_2_LINES (0x2UL << QUADSPI_CCR_ABMODE_Pos)
#define QUADSPI_CCR_ABMODE_3_LINES (0x3UL << QUADSPI_CCR_ABMODE_Pos)

#define QUADSPI_CCR_ADSIZE_Pos 12U
#define QUADSPI_CCR_ADSIZE_Msk (0x3UL << QUADSPI_CCR_ADSIZE_Pos)
#define QUADSPI_CCR_ADSIZE QUADSPI_CCR_ADSIZE_Msk
#define QUADSPI_CCR_ADSIZE_8_BIT (0x0UL << QUADSPI_CCR_ADSIZE_Pos)
#define QUADSPI_CCR_ADSIZE_16_BIT (0x1UL << QUADSPI_CCR_ADSIZE_Pos)
#define QUADSPI_CCR_ADSIZE_24_BIT (0x2UL << QUADSPI_CCR_ADSIZE_Pos)
#define QUADSPI_CCR_ADSIZE_32_BIT (0x3UL << QUADSPI_CCR_ADSIZE_Pos)

#define QUADSPI_CCR_ADMODE_Pos 10U
#define QUADSPI_CCR_ADMODE_Msk (0x3UL << QUADSPI_CCR_ADMODE_Pos)
#define QUADSPI_CCR_ADMODE QUADSPI_CCR_ADMODE_Msk
#define QUADSPI_CCR_ADMODE_NONE (0x0UL << QUADSPI_CCR_ADMODE_Pos)
#define QUADSPI_CCR_ADMODE_1_LINE (0x1UL << QUADSPI_CCR_ADMODE_Pos)
#define QUADSPI_CCR_ADMODE_2_LINES (0x2UL << QUADSPI_CCR_ADMODE_Pos)
#define QUADSPI_CCR_ADMODE_3_LINES (0x3UL << QUADSPI_CCR_ADMODE_Pos)

#define QUADSPI_CCR_IMODE_Pos 8U
#define QUADSPI_CCR_IMODE_Msk (0x3UL << QUADSPI_CCR_IMODE_Pos)
#define QUADSPI_CCR_IMODE QUADSPI_CCR_IMODE_Msk
#define QUADSPI_CCR_IMODE_NONE (0x0UL << QUADSPI_CCR_IMODE_Pos)
#define QUADSPI_CCR_IMODE_1_LINE (0x1UL << QUADSPI_CCR_IMODE_Pos)
#define QUADSPI_CCR_IMODE_2_LINES (0x2UL << QUADSPI_CCR_IMODE_Pos)
#define QUADSPI_CCR_IMODE_3_LINES (0x3UL << QUADSPI_CCR_IMODE_Pos)

#define QUADSPI_CCR_INSTRUCTION_Pos 0U
#define QUADSPI_CCR_INSTRUCTION_Msk (0xFFUL << QUADSPI_CCR_INSTRUCTION_Pos)
#define QUADSPI_CCR_INSTRUCTION QUADSPI_CCR_INSTRUCTION_Msk

/*PIR register*/
#define QUADSPI_PIR_INTERVAL_Pos 0U
#define QUADSPI_PIR_INTERVAL_Msk (0xFFFFUL << QUADSPI_PIR_INTERVAL_Pos)
#define QUADSPI_PIR_INTERVAL QUADSPI_PIR_INTERVAL_Msk

/*LPTR register*/
#define QUADSPI_LPTR_TIMEOUT_Pos 0U
#define QUADSPI_LPTR_TIMEOUT_Msk (0xFFFFUL << QUADSPI_LPTR_TIMEOUT_Pos)
#define QUADSPI_LPTR_TIMEOUT QUADSPI_LPTR_TIMEOUT_Msk

/* ========================================================================= */
/* ============                       ADC                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t ISR; /*!< ADC Interrupt and Status Register. */
	__IOM uint32_t IER; /*!< ADC Interrupt Enable Register. */
	__IOM uint32_t CR; /*!< ADC Control Register. */
	__IOM uint32_t CFGR; /*!< ADC Configuration Register. */
	__IOM uint32_t CFGR2; /*!< ADC Configuration Register 2. */
	__IOM uint32_t SMPR1; /*!< ADC Sample time Register 1. */
	__IOM uint32_t SMPR2; /*!< ADC Sample time Register 2. */
	uint32_t RESERVED0;
	__IOM uint32_t TR1; /*!< ADC watchdog Threshold Register 1. */
	__IOM uint32_t TR2; /*!< ADC watchdog Threshold Register 2. */
	__IOM uint32_t TR3;	/*!< ADC watchdog Threshold Register 3. */
	uint32_t RESERVED1;
	__IOM uint32_t SQR1; /*!< ADC regular Sequence Register 1. */
	__IOM uint32_t SQR2; /*!< ADC regular Sequence Register 2. */
	__IOM uint32_t SQR3; /*!< ADC regular Sequence Register 3. */
	__IOM uint32_t SQR4; /*!< ADC regular Sequence Register 4. */
	__IM uint32_t DR; /*!< ADC regular Data Register. */
	uint32_t RESERVED2[2];
	__IOM uint32_t JSQR; /*!< ADC Injected Sequence Register. */
	uint32_t RESERVED3[4];
	__IOM uint32_t OFR1; /*!< ADC Offset 1 Register. */
	__IOM uint32_t OFR2; /*!< ADC Offset 2 Register. */
	__IOM uint32_t OFR3; /*!< ADC Offset 3 Register. */
	__IOM uint32_t OFR4; /*!< ADC Offset 4 Register. */
	uint32_t RESERVED4[4];
	__IM uint32_t JDR1; /*!< ADC Injected Channel 1 Data Register. */
	__IM uint32_t JDR2; /*!< ADC Injected Channel 2 Data Register. */
	__IM uint32_t JDR3; /*!< ADC Injected Channel 3 Data Register. */
	__IM uint32_t JDR4; /*!< ADC Injected Channel 4 Data Register. */
	uint32_t RESERVED5[4];
	__IOM uint32_t AWD2CR; /*!< ADC Analog Watchdog 2 Configuration Register. */
	__IOM uint32_t AWD3CR; /*!< ADC Analog Watchdog 3 Configuration Register. */
	uint32_t RESERVED6[2];
	__IOM uint32_t DIFSEL; /*!< ADC Differential Mode Selection Register. */
	__IOM uint32_t CALFACT; /*!< ADC Calibration Factors. */
}STM32L4xx_ADC_TypeDef;

typedef struct
{
	__IM uint32_t CSR; /*!< ADC Common Status Register. */
	uint32_t RESERVED0;
	__IOM uint32_t CCR; /*!< ADC Common Control Register. */
	__IM uint32_t CDR; /*!< ADC Common Regular Data Register for dual mode. */
}STM32L4xx_ADC_CM_TypeDef;

/*ISR register*/
#define ADC_ISR_JQOVF_Pos 10U
#define ADC_ISR_JQOVF_Msk (0x1UL << ADC_ISR_JQOVF_Pos)
#define ADC_ISR_JQOVF ADC_ISR_JQOVF_Msk

#define ADC_ISR_AWD3_Pos 9U
#define ADC_ISR_AWD3_Msk (0x1UL << ADC_ISR_AWD3_Pos)
#define ADC_ISR_AWD3 ADC_ISR_AWD3_Msk

#define ADC_ISR_AWD2_Pos 8U
#define ADC_ISR_AWD2_Msk (0x1UL << ADC_ISR_AWD2_Pos)
#define ADC_ISR_AWD2 ADC_ISR_AWD2_Msk

#define ADC_ISR_AWD1_Pos 7U
#define ADC_ISR_AWD1_Msk (0x1UL << ADC_ISR_AWD1_Pos)
#define ADC_ISR_AWD1 ADC_ISR_AWD1_Msk

#define ADC_ISR_JEOS_Pos 6U
#define ADC_ISR_JEOS_Msk (0x1UL << ADC_ISR_JEOS_Pos)
#define ADC_ISR_JEOS ADC_ISR_JEOS_Msk

#define ADC_ISR_JEOC_Pos 5U
#define ADC_ISR_JEOC_Msk (0x1UL << ADC_ISR_JEOC_Pos)
#define ADC_ISR_JEOC ADC_ISR_JEOC_Msk

#define ADC_ISR_OVR_Pos 4U
#define ADC_ISR_OVR_Msk (0x1UL << ADC_ISR_OVR_Pos)
#define ADC_ISR_OVR ADC_ISR_OVR_Msk

#define ADC_ISR_EOS_Pos 3U
#define ADC_ISR_EOS_Msk (0x1UL << ADC_ISR_EOS_Pos)
#define ADC_ISR_EOS ADC_ISR_EOS_Msk

#define ADC_ISR_EOC_Pos 2U
#define ADC_ISR_EOC_Msk (0x1UL << ADC_ISR_EOC_Pos)
#define ADC_ISR_EOC ADC_ISR_EOC_Msk

#define ADC_ISR_EOSMP_Pos 1U
#define ADC_ISR_EOSMP_Msk (0x1UL << ADC_ISR_EOSMP_Pos)
#define ADC_ISR_EOSMP ADC_ISR_EOSMP_Msk

#define ADC_ISR_ADRDY_Pos 0U
#define ADC_ISR_ADRDY_Msk (0x1UL << ADC_ISR_ADRDY_Pos)
#define ADC_ISR_ADRDY ADC_ISR_ADRDY_Msk

/*IER register*/
#define ADC_IER_JQOVFIE_Pos 10U
#define ADC_IER_JQOVFIE_Msk (0x1UL << ADC_IER_JQOVFIE_Pos)
#define ADC_IER_JQOVFIE ADC_IER_JQOVFIE_Msk

#define ADC_IER_AWD3IE_Pos 9U
#define ADC_IER_AWD3IE_Msk (0x1UL << ADC_IER_AWD3IE_Pos)
#define ADC_IER_AWD3IE ADC_IER_AWD3IE_Msk

#define ADC_IER_AWD2IE_Pos 8U
#define ADC_IER_AWD2IE_Msk (0x1UL << ADC_IER_AWD2IE_Pos)
#define ADC_IER_AWD2IE ADC_IER_AWD2IE_Msk

#define ADC_IER_AWD1IE_Pos 7U
#define ADC_IER_AWD1IE_Msk (0x1UL << ADC_IER_AWD1IE_Pos)
#define ADC_IER_AWD1IE ADC_IER_AWD1IE_Msk

#define ADC_IER_JEOSIE_Pos 6U
#define ADC_IER_JEOSIE_Msk (0x1UL << ADC_IER_JEOSIE_Pos)
#define ADC_IER_JEOSIE ADC_IER_JEOSIE_Msk

#define ADC_IER_JEOCIE_Pos 5U
#define ADC_IER_JEOCIE_Msk (0x1UL << ADC_IER_JEOCIE_Pos)
#define ADC_IER_JEOCIE ADC_IER_JEOCIE_Msk

#define ADC_IER_OVRIE_Pos 4U
#define ADC_IER_OVRIE_Msk (0x1UL << ADC_IER_OVRIE_Pos)
#define ADC_IER_OVRIE ADC_IER_OVRIE_Msk

#define ADC_IER_EOSIE_Pos 3U
#define ADC_IER_EOSIE_Msk (0x1UL << ADC_IER_EOSIE_Pos)
#define ADC_IER_EOSIE ADC_IER_EOSIE_Msk

#define ADC_IER_EOCIE_Pos 2U
#define ADC_IER_EOCIE_Msk (0x1UL << ADC_IER_EOCIE_Pos)
#define ADC_IER_EOCIE ADC_IER_EOCIE_Msk

#define ADC_IER_EOSMPIE_Pos 1U
#define ADC_IER_EOSMPIE_Msk (0x1UL << ADC_IER_EOSMPIE_Pos)
#define ADC_IER_EOSMPIE ADC_IER_EOSMPIE_Msk

#define ADC_IER_ADRDYIE_Pos 0U
#define ADC_IER_ADRDYIE_Msk (0x1UL << ADC_IER_ADRDYIE_Pos)
#define ADC_IER_ADRDYIE ADC_IER_ADRDYIE_Msk

/*CR register*/
#define ADC_CR_ADCAL_Pos 31U
#define ADC_CR_ADCAL_Msk (0x1UL << ADC_CR_ADCAL_Pos)
#define ADC_CR_ADCAL ADC_CR_ADCAL_Msk

#define ADC_CR_ADCALDIF_Pos 30U
#define ADC_CR_ADCALDIF_Msk (0x1UL << ADC_CR_ADCALDIF_Pos)
#define ADC_CR_ADCALDIF ADC_CR_ADCALDIF_Msk

#define ADC_CR_DEEPPWD_Pos 29U
#define ADC_CR_DEEPPWD_Msk (0x1UL << ADC_CR_DEEPPWD_Pos)
#define ADC_CR_DEEPPWD ADC_CR_DEEPPWD_Msk

#define ADC_CR_ADVREGEN_Pos 28U
#define ADC_CR_ADVREGEN_Msk (0x1UL << ADC_CR_ADVREGEN_Pos)
#define ADC_CR_ADVREGEN ADC_CR_ADVREGEN_Msk

#define ADC_CR_JADSTP_Pos 5U
#define ADC_CR_JADSTP_Msk (0x1UL << ADC_CR_JADSTP_Pos)
#define ADC_CR_JADSTP ADC_CR_JADSTP_Msk

#define ADC_CR_ADSTP_Pos 4U
#define ADC_CR_ADSTP_Msk (0x1UL << ADC_CR_ADSTP_Pos)
#define ADC_CR_ADSTP ADC_CR_ADSTP_Msk

#define ADC_CR_JADSTART_Pos 3U
#define ADC_CR_JADSTART_Msk (0x1UL << ADC_CR_JADSTART_Pos)
#define ADC_CR_JADSTART ADC_CR_JADSTART_Msk

#define ADC_CR_ADSTART_Pos 2U
#define ADC_CR_ADSTART_Msk (0x1UL << ADC_CR_ADSTART_Pos)
#define ADC_CR_ADSTART ADC_CR_ADSTART_Msk

#define ADC_CR_ADDIS_Pos 1U
#define ADC_CR_ADDIS_Msk (0x1UL << ADC_CR_ADDIS_Pos)
#define ADC_CR_ADDIS ADC_CR_ADDIS_Msk

#define ADC_CR_ADEN_Pos 0U
#define ADC_CR_ADEN_Msk (0x1UL << ADC_CR_ADEN_Pos)
#define ADC_CR_ADEN ADC_CR_ADEN_Msk

/*CFGR register*/
#define ADC_CFGR_JQDIS_Pos 31U
#define ADC_CFGR_JQDIS_Msk (0x1UL << ADC_CFGR_JQDIS_Pos)
#define ADC_CFGR_JQDIS ADC_CFGR_JQDIS_Msk

#define ADC_CFGR_AWD1CH_Pos 26U
#define ADC_CFGR_AWD1CH_Msk (0x1FUL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH ADC_CFGR_AWD1CH_Msk
#define ADC_CFGR_AWD1CH_0 (0x0UL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_1 (0x1UL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_2 (0x2UL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_3 (0x3UL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_4 (0x4UL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_5 (0x5UL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_6 (0x6UL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_7 (0x7UL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_8 (0x8UL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_9 (0x9UL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_10 (0xAUL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_11 (0xBUL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_12 (0xCUL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_13 (0xDUL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_14 (0xEUL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_15 (0xFUL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_16 (0x10UL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_17 (0x11UL << ADC_CFGR_AWD1CH_Pos)
#define ADC_CFGR_AWD1CH_18 (0x12UL << ADC_CFGR_AWD1CH_Pos)

#define ADC_CFGR_JAUTO_Pos 25U
#define ADC_CFGR_JAUTO_Msk (0x1UL << ADC_CFGR_JAUTO_Pos)
#define ADC_CFGR_JAUTO ADC_CFGR_JAUTO_Msk

#define ADC_CFGR_JAWD1EN_Pos 24U
#define ADC_CFGR_JAWD1EN_Msk (0x1UL << ADC_CFGR_JAWD1EN_Pos)
#define ADC_CFGR_JAWD1EN ADC_CFGR_JAWD1EN_Msk

#define ADC_CFGR_AWD1EN_Pos 23U
#define ADC_CFGR_AWD1EN_Msk (0x1UL << ADC_CFGR_AWD1EN_Pos)
#define ADC_CFGR_AWD1EN ADC_CFGR_AWD1EN_Msk

#define ADC_CFGR_AWD1SGL_Pos 22U
#define ADC_CFGR_AWD1SGL_Msk (0x1UL << ADC_CFGR_AWD1SGL_Pos)
#define ADC_CFGR_AWD1SGL ADC_CFGR_AWD1SGL_Msk

#define ADC_CFGR_JQM_Pos 21U
#define ADC_CFGR_JQM_Msk (0x1UL << ADC_CFGR_JQM_Pos)
#define ADC_CFGR_JQM ADC_CFGR_JQM_Msk

#define ADC_CFGR_JDISCEN_Pos 20U
#define ADC_CFGR_JDISCEN_Msk (0x1UL << ADC_CFGR_JDISCEN_Pos)
#define ADC_CFGR_JDISCEN ADC_CFGR_JDISCEN_Msk

#define ADC_CFGR_DISCNUM_Pos 17U
#define ADC_CFGR_DISCNUM_Msk (0x7UL << ADC_CFGR_DISCNUM_Pos)
#define ADC_CFGR_DISCNUM ADC_CFGR_DISCNUM_Msk

#define ADC_CFGR_DISCEN_Pos 16U
#define ADC_CFGR_DISCEN_Msk (0x1UL << ADC_CFGR_DISCEN_Pos)
#define ADC_CFGR_DISCEN ADC_CFGR_DISCEN_Msk

#define ADC_CFGR_AUTDLY_Pos  14U
#define ADC_CFGR_AUTDLY_Msk (0x1UL << ADC_CFGR_AUTDLY_Pos)
#define ADC_CFGR_AUTDLY ADC_CFGR_AUTDLY_Msk

#define ADC_CFGR_CONT_Pos 13U
#define ADC_CFGR_CONT_Msk (0x1UL << ADC_CFGR_CONT_Pos)
#define ADC_CFGR_CONT ADC_CFGR_CONT_Msk

#define ADC_CFGR_OVRMOD_Pos 12U
#define ADC_CFGR_OVRMOD_Msk (0x1UL << ADC_CFGR_OVRMOD_Pos)
#define ADC_CFGR_OVRMOD ADC_CFGR_OVRMOD_Msk

#define ADC_CFGR_EXTEN_Pos 10U
#define ADC_CFGR_EXTEN_Msk (0x3UL << ADC_CFGR_EXTEN_Pos)
#define ADC_CFGR_EXTEN ADC_CFGR_EXTEN_Msk
#define ADC_CFGR_EXTEN_DISABLED (0x0UL << ADC_CFGR_EXTEN_Pos)
#define ADC_CFGR_EXTEN_RISING_EDGE (0x1UL << ADC_CFGR_EXTEN_Pos)
#define ADC_CFGR_EXTEN_FALLING_EDGE (0x2UL << ADC_CFGR_EXTEN_Pos)
#define ADC_CFGR_EXTEN_RISING_FALLING_EDGE (0x3UL << ADC_CFGR_EXTEN_Pos)

#define ADC_CFGR_EXTSEL_Pos 6U
#define ADC_CFGR_EXTSEL_Msk (0xFUL << ADC_CFGR_EXTSEL_Pos)
#define ADC_CFGR_EXTSEL ADC_CFGR_EXTSEL_Msk

#define ADC_CFGR_ALIGN_Pos 5U
#define ADC_CFGR_ALIGN_Msk (0x1UL << ADC_CFGR_ALIGN_Pos)
#define ADC_CFGR_ALIGN ADC_CFGR_ALIGN_Msk

#define ADC_CFGR_RES_Pos 3U
#define ADC_CFGR_RES_Msk (0x3UL << ADC_CFGR_RES_Pos)
#define ADC_CFGR_RES ADC_CFGR_RES_Msk
#define ADC_CFGR_RES_12_BIT (0x0UL << ADC_CFGR_RES_Pos)
#define ADC_CFGR_RES_10_BIT (0x1UL << ADC_CFGR_RES_Pos)
#define ADC_CFGR_RES_8_BIT (0x2UL << ADC_CFGR_RES_Pos)
#define ADC_CFGR_RES_6_BIT (0x3UL << ADC_CFGR_RES_Pos)

#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define ADC_CFGR_DFSDMCFG_Pos 2U
#define ADC_CFGR_DFSDMCFG_Msk (0x1UL << ADC_CFGR_DFSDMCFG_Pos)
#define ADC_CFGR_DFSDMCFG ADC_CFGR_DFSDMCFG_Msk
#endif

#define ADC_CFGR_DMACFG_Pos 1U
#define ADC_CFGR_DMACFG_Msk (0x1UL << ADC_CFGR_DMACFG_Pos)
#define ADC_CFGR_DMACFG ADC_CFGR_DMACFG_Msk

#define ADC_CFGR_DMAEN_Pos 0U
#define ADC_CFGR_DMAEN_Msk (0x1UL << ADC_CFGR_DMAEN_Pos)
#define ADC_CFGR_DMAEN ADC_CFGR_DMAEN_Msk

/*CFGR2 register*/
#define ADC_CFGR2_ROVSM_Pos 10U
#define ADC_CFGR2_ROVSM_Msk (0x1UL << ADC_CFGR2_ROVSM_Pos)
#define ADC_CFGR2_ROVSM ADC_CFGR2_ROVSM_Msk

#define ADC_CFGR2_TROVS_Pos 9U
#define ADC_CFGR2_TROVS_Msk (0x1UL << ADC_CFGR2_TROVS_Pos)
#define ADC_CFGR2_TROVS ADC_CFGR2_TROVS_Msk

#define ADC_CFGR2_OVSS_Pos 5U
#define ADC_CFGR2_OVSS_Msk (0xFUL << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS ADC_CFGR2_OVSS_Msk
#define ADC_CFGR2_OVSS_0_BIT (0x0UL << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_1_BIT (0x1UL << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_2_BIT (0x2UL << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_3_BIT (0x3UL << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_4_BIT (0x4UL << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_5_BIT (0x5UL << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_6_BIT (0x6UL << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_7_BIT (0x7UL << ADC_CFGR2_OVSS_Pos)
#define ADC_CFGR2_OVSS_8_BIT (0x8UL << ADC_CFGR2_OVSS_Pos)

#define ADC_CFGR2_OVSR_Pos 2U
#define ADC_CFGR2_OVSR_Msk (0x7UL << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR ADC_CFGR2_OVSR_Msk
#define ADC_CFGR2_OVSR_2 (0x0UL << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_4x (0x1UL << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_8 (0x2UL << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_16 (0x3UL << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_32 (0x4UL << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_64 (0x5UL << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_128 (0x6UL << ADC_CFGR2_OVSR_Pos)
#define ADC_CFGR2_OVSR_256 (0x7UL << ADC_CFGR2_OVSR_Pos)

#define ADC_CFGR2_JOVSE_Pos 1U
#define ADC_CFGR2_JOVSE_Msk (0x1UL << ADC_CFGR2_JOVSE_Pos)
#define ADC_CFGR2_JOVSE ADC_CFGR2_JOVSE_Msk

#define ADC_CFGR2_ROVSE_Pos 0U
#define ADC_CFGR2_ROVSE_Msk (0x1UL << ADC_CFGR2_ROVSE_Pos)
#define ADC_CFGR2_ROVSE ADC_CFGR2_ROVSE_Msk

/*SMPR1 register*/
#define ADC_SMPR1_SMP9_Pos 27U
#define ADC_SMPR1_SMP9_Msk (0x7UL << ADC_SMPR1_SMP9_Pos)
#define ADC_SMPR1_SMP9 ADC_SMPR1_SMP9_Msk

#define ADC_SMPR1_SMP8_Pos 24U
#define ADC_SMPR1_SMP8_Msk (0x7UL << ADC_SMPR1_SMP8_Pos)
#define ADC_SMPR1_SMP8 ADC_SMPR1_SMP8_Msk

#define ADC_SMPR1_SMP7_Pos 21U
#define ADC_SMPR1_SMP7_Msk (0x7UL << ADC_SMPR1_SMP7_Pos)
#define ADC_SMPR1_SMP7 ADC_SMPR1_SMP7_Msk

#define ADC_SMPR1_SMP6_Pos 18U
#define ADC_SMPR1_SMP6_Msk (0x7UL << ADC_SMPR1_SMP6_Pos)
#define ADC_SMPR1_SMP6 ADC_SMPR1_SMP6_Msk

#define ADC_SMPR1_SMP5_Pos 15U
#define ADC_SMPR1_SMP5_Msk (0x7UL << ADC_SMPR1_SMP5_Pos)
#define ADC_SMPR1_SMP5 ADC_SMPR1_SMP5_Msk

#define ADC_SMPR1_SMP4_Pos 12U
#define ADC_SMPR1_SMP4_Msk (0x7UL << ADC_SMPR1_SMP4_Pos)
#define ADC_SMPR1_SMP4 ADC_SMPR1_SMP4_Msk

#define ADC_SMPR1_SMP3_Pos 9U
#define ADC_SMPR1_SMP3_Msk (0x7UL << ADC_SMPR1_SMP3_Pos)
#define ADC_SMPR1_SMP3 ADC_SMPR1_SMP3_Msk

#define ADC_SMPR1_SMP2_Pos 6U
#define ADC_SMPR1_SMP2_Msk (0x7UL << ADC_SMPR1_SMP2_Pos)
#define ADC_SMPR1_SMP2 ADC_SMPR1_SMP2_Msk

#define ADC_SMPR1_SMP1_Pos 3U
#define ADC_SMPR1_SMP1_Msk (0x7UL << ADC_SMPR1_SMP1_Pos)
#define ADC_SMPR1_SMP1 ADC_SMPR1_SMP1_Msk

#define ADC_SMPR1_SMP0_Pos 0U
#define ADC_SMPR1_SMP0_Msk (0x7UL << ADC_SMPR1_SMP0_Pos)
#define ADC_SMPR1_SMP0 ADC_SMPR1_SMP0_Msk

/*SMPR2 register*/
#define ADC_SMPR2_SMP18_Pos 24U
#define ADC_SMPR2_SMP18_Msk (0x7UL << ADC_SMPR2_SMP18_Pos)
#define ADC_SMPR2_SMP18 ADC_SMPR2_SMP18_Msk

#define ADC_SMPR2_SMP17_Pos 21U
#define ADC_SMPR2_SMP17_Msk (0x7UL << ADC_SMPR2_SMP17_Pos)
#define ADC_SMPR2_SMP17 ADC_SMPR2_SMP17_Msk

#define ADC_SMPR2_SMP16_Pos 18U
#define ADC_SMPR2_SMP16_Msk (0x7UL << ADC_SMPR2_SMP16_Pos)
#define ADC_SMPR2_SMP16 ADC_SMPR2_SMP16_Msk

#define ADC_SMPR2_SMP15_Pos 15U
#define ADC_SMPR2_SMP15_Msk (0x7UL << ADC_SMPR2_SMP15_Pos)
#define ADC_SMPR2_SMP15 ADC_SMPR2_SMP15_Msk

#define ADC_SMPR2_SMP14_Pos 12U
#define ADC_SMPR2_SMP14_Msk (0x7UL << ADC_SMPR2_SMP14_Pos)
#define ADC_SMPR2_SMP14 ADC_SMPR2_SMP14_Msk

#define ADC_SMPR2_SMP13_Pos 9U
#define ADC_SMPR2_SMP13_Msk (0x7UL << ADC_SMPR2_SMP13_Pos)
#define ADC_SMPR2_SMP13 ADC_SMPR2_SMP13_Msk

#define ADC_SMPR2_SMP12_Pos 6U
#define ADC_SMPR2_SMP12_Msk (0x7UL << ADC_SMPR2_SMP12_Pos)
#define ADC_SMPR2_SMP12 ADC_SMPR2_SMP12_Msk

#define ADC_SMPR2_SMP11_Pos 3U
#define ADC_SMPR2_SMP11_Msk (0x7UL << ADC_SMPR2_SMP11_Pos)
#define ADC_SMPR2_SMP11 ADC_SMPR2_SMP11_Msk

#define ADC_SMPR2_SMP10_Pos 0U
#define ADC_SMPR2_SMP10_Msk (0x7UL << ADC_SMPR2_SMP10_Pos)
#define ADC_SMPR1_SMP10 ADC_SMPR2_SMP10_Msk

/*TR1 register*/
#define ADC_TR1_HT1_Pos 16U
#define ADC_TR1_HT1_Msk (0xFFFUL << ADC_TR1_HT1_Pos)
#define ADC_TR1_HT1 ADC_TR1_HT1_Msk

#define ADC_TR1_LT1_Pos 0U
#define ADC_TR1_LT1_Msk (0xFFFUL << ADC_TR1_LT1_Pos)
#define ADC_TR1_LT1 ADC_TR1_LT1_Msk

/*TR2 register*/
#define ADC_TR2_HT2_Pos 16U
#define ADC_TR2_HT2_Msk (0xFFFUL << ADC_TR2_HT2_Pos)
#define ADC_TR2_HT2 ADC_TR2_HT2_Msk

#define ADC_TR2_LT2_Pos 0U
#define ADC_TR2_LT2_Msk (0xFFFUL << ADC_TR2_LT2_Pos)
#define ADC_TR2_LT2 ADC_TR2_LT2_Msk

/*TR3 register*/
#define ADC_TR3_HT3_Pos 16U
#define ADC_TR3_HT3_Msk (0xFFFUL << ADC_TR3_HT3_Pos)
#define ADC_TR3_HT3 ADC_TR3_HT3_Msk

#define ADC_TR3_LT3_Pos 0U
#define ADC_TR3_LT3_Msk (0xFFFUL << ADC_TR3_LT3_Pos)
#define ADC_TR3_LT3 ADC_TR3_LT3_Msk

/*SQR1 register*/
#define ADC_SQR1_SQ4_Pos 24U
#define ADC_SQR1_SQ4_Msk (0x1FUL << ADC_SQR1_SQ4_Pos)
#define ADC_SQR1_SQ4 ADC_SQR1_SQ4_Msk

#define ADC_SQR1_SQ3_Pos 18U
#define ADC_SQR1_SQ3_Msk (0x1FUL << ADC_SQR1_SQ3_Pos)
#define ADC_SQR1_SQ3 ADC_SQR1_SQ3_Msk

#define ADC_SQR1_SQ2_Pos 12U
#define ADC_SQR1_SQ2_Msk (0x1FUL << ADC_SQR1_SQ2_Pos)
#define ADC_SQR1_SQ2 ADC_SQR1_SQ2_Msk

#define ADC_SQR1_SQ1_Pos 6U
#define ADC_SQR1_SQ1_Msk (0x1FUL << ADC_SQR1_SQ1_Pos)
#define ADC_SQR1_SQ1 ADC_SQR1_SQ1_Msk

#define ADC_SQR1_L_Pos 0U
#define ADC_SQR1_L_Msk (0xFUL << ADC_SQR1_L_Pos)
#define ADC_SQR1_L ADC_SQR1_L_Msk

/*SQR2 register*/
#define ADC_SQR2_SQ9_Pos 24U
#define ADC_SQR2_SQ9_Msk (0x1FUL << ADC_SQR2_SQ9_Pos)
#define ADC_SQR2_SQ9 ADC_SQR2_SQ9_Msk

#define ADC_SQR2_SQ8_Pos 18U
#define ADC_SQR2_SQ8_Msk (0x1FUL << ADC_SQR2_SQ8_Pos)
#define ADC_SQR2_SQ8 ADC_SQR2_SQ8_Msk

#define ADC_SQR2_SQ7_Pos 12U
#define ADC_SQR2_SQ7_Msk (0x1FUL << ADC_SQR2_SQ7_Pos)
#define ADC_SQR2_SQ7 ADC_SQR2_SQ7_Msk

#define ADC_SQR2_SQ6_Pos 6U
#define ADC_SQR2_SQ6_Msk (0x1FUL << ADC_SQR2_SQ6_Pos)
#define ADC_SQR2_SQ6 ADC_SQR2_SQ6_Msk

#define ADC_SQR2_SQ5_Pos 0U
#define ADC_SQR2_SQ5_Msk (0x1FUL << ADC_SQR2_SQ5_Pos)
#define ADC_SQR2_SQ5 ADC_SQR2_SQ5_Msk

/*SQR3 register*/
#define ADC_SQR3_SQ14_Pos 24U
#define ADC_SQR3_SQ14_Msk (0x1FUL << ADC_SQR3_SQ14_Pos)
#define ADC_SQR3_SQ14 ADC_SQR3_SQ14_Msk

#define ADC_SQR3_SQ13_Pos 18U
#define ADC_SQR3_SQ13_Msk (0x1FUL << ADC_SQR3_SQ13_Pos)
#define ADC_SQR3_SQ13 ADC_SQR3_SQ13_Msk

#define ADC_SQR3_SQ12_Pos 12U
#define ADC_SQR3_SQ12_Msk (0x1FUL << ADC_SQR3_SQ12_Pos)
#define ADC_SQR3_SQ12 ADC_SQR3_SQ12_Msk

#define ADC_SQR3_SQ11_Pos 6U
#define ADC_SQR3_SQ11_Msk (0x1FUL << ADC_SQR3_SQ11_Pos)
#define ADC_SQR3_SQ11 ADC_SQR3_SQ11_Msk

#define ADC_SQR3_SQ10_Pos 0U
#define ADC_SQR3_SQ10_Msk (0x1FUL << ADC_SQR3_SQ10_Pos)
#define ADC_SQR3_SQ10 ADC_SQR3_SQ10_Msk

/*SQR4 register*/
#define ADC_SQR4_SQ16_Pos 6U
#define ADC_SQR4_SQ16_Msk (0x1FUL << ADC_SQR4_SQ16_Pos)
#define ADC_SQR4_SQ16 ADC_SQR4_SQ16_Msk

#define ADC_SQR4_SQ15_Pos 0U
#define ADC_SQR4_SQ15_Msk (0x1FUL << ADC_SQR4_SQ15_Pos)
#define ADC_SQR4_SQ15 ADC_SQR4_SQ15_Msk

/*DR register*/
#define ADC_DR_RDATA_Pos 0U
#define ADC_DR_RDATA_Msk (0xFFFFUL << ADC_DR_RDATA_Pos)
#define ADC_DR_RDATA ADC_DR_RDATA_Msk

/*JSQR register*/
#define ADC_JSQR_JSQ4_Pos 26U
#define ADC_JSQR_JSQ4_Msk (0x1FUL << ADC_JSQR_JSQ4_Pos)
#define ADC_JSQR_JSQ4 ADC_JSQR_JSQ4_Msk

#define ADC_JSQR_JSQ3_Pos 20U
#define ADC_JSQR_JSQ3_Msk (0x1FUL << ADC_JSQR_JSQ3_Pos)
#define ADC_JSQR_JSQ3 ADC_JSQR_JSQ3_Msk

#define ADC_JSQR_JSQ2_Pos 14U
#define ADC_JSQR_JSQ2_Msk (0x1FUL << ADC_JSQR_JSQ2_Pos)
#define ADC_JSQR_JSQ2 ADC_JSQR_JSQ2_Msk

#define ADC_JSQR_JSQ1_Pos 8U
#define ADC_JSQR_JSQ1_Msk (0x1FUL << ADC_JSQR_JSQ1_Pos)
#define ADC_JSQR_JSQ1 ADC_JSQR_JSQ1_Msk

#define ADC_JSQR_JEXTEN_Pos 6U
#define ADC_JSQR_JEXTEN_Msk (0x3UL << ADC_JSQR_JEXTEN_Pos)
#define ADC_JSQR_JEXTEN ADC_JSQR_JEXTEN_Msk
#define ADC_JSQR_JEXTEN_DISABLED (0x0UL << ADC_JSQR_JEXTEN_Pos)
#define ADC_JSQR_JEXTEN_RISING_EDGE (0x1UL << ADC_JSQR_JEXTEN_Pos)
#define ADC_JSQR_JEXTEN_FALLING_EDGE (0x2UL << ADC_JSQR_JEXTEN_Pos)
#define ADC_JSQR_JEXTEN_RISING_FALLING_EDGE (0x3UL << ADC_JSQR_JEXTEN_Pos)

#define ADC_JSQR_JEXTSEL_Pos 2U
#define ADC_JSQR_JEXTSEL_Msk (0xFUL << ADC_JSQR_JEXTSEL_Pos)
#define ADC_JSQR_JEXTSEL ADC_JSQR_JEXTSEL_Msk

#define ADC_JSQR_JL_Pos 0U
#define ADC_JSQR_JL_Msk (0x3UL << ADC_JSQR_JL_Pos)
#define ADC_JSQR_JL ADC_JSQR_JL_Msk
#define ADC_JSQR_JL_1 (0x0UL << ADC_JSQR_JL_Pos)
#define ADC_JSQR_JL_2 (0x1UL << ADC_JSQR_JL_Pos)
#define ADC_JSQR_JL_3 (0x2UL << ADC_JSQR_JL_Pos)
#define ADC_JSQR_JL_4 (0x3UL << ADC_JSQR_JL_Pos)

/*OFRy register*/
#define ADC_OFR_OFFSET_EN_Pos 31U
#define ADC_OFR_OFFSET_EN_Msk (0x1UL << ADC_OFR_OFFSET_EN_Pos)
#define ADC_OFR_OFFSET_EN ADC_OFR_OFFSET_EN_Msk

#define ADC_OFR_OFFSET_CH_Pos 26U
#define ADC_OFR_OFFSET_CH_Msk (0x1FUL << ADC_OFR_OFFSET_CH_Pos)
#define ADC_OFR_OFFSET_CH ADC_OFR_OFFSET_CH_Msk

#define ADC_OFR_OFFSET_Pos 0U
#define ADC_OFR_OFFSET_Msk (0xFFFUL << ADC_OFR_OFFSET_Pos)
#define ADC_OFR_OFFSET ADC_OFR_OFFSET_Msk

/*JDRy register*/
#define ADC_JDR_JDATA_Pos 0U
#define ADC_JDR_JDATA_Msk (0xFFFFUL << ADC_JDR1_JDATA_Pos)
#define ADC_JDR_JDATA ADC_JDR_JDATA_Msk

/*AWD2CR register*/
#define ADC_AWD2CR_AWD2CH_Pos 0U
#define ADC_AWD2CR_AWD2CH_Msk (0x7FFFFUL << ADC_AWD2CR_AWD2CH_Pos)
#define ADC_AWD2CR_AWD2CH ADC_AWD2CR_AWD2CH_Msk

/*AWD3CR register*/
#define ADC_AWD3CR_AWD3CH_Pos 0U
#define ADC_AWD3CR_AWD3CH_Msk (0x7FFFFUL << ADC_AWD3CR_AWD3CH_Pos)
#define ADC_AWD3CR_AWD3CH ADC_AWD3CR_AWD3CH_Msk

/*DIFSEL register*/
#define ADC_DIFSEL_Pos 0U
#define ADC_DIFSEL_Msk (0x7FFFFUL << ADC_DIFSEL_Pos)
#define ADC_DIFSEL ADC_DIFSEL_Msk

/*CALFACT register*/
#define ADC_CALFACT_D_Pos 16U
#define ADC_CALFACT_D_Msk (0x7FUL << ADC_CALFACT_D_Pos)
#define ADC_CALFACT_D ADC_CALFACT_D_Msk

#define ADC_CALFACT_S_Pos 0U
#define ADC_CALFACT_S_Msk (0x7FUL << ADC_CALFACT_S_Pos)
#define ADC_CALFACT_S ADC_CALFACT_S_Msk

/*CSR register*/
#define ADC_CSR_JQOVF_SLV_Pos 26U
#define ADC_CSR_JQOVF_SLV_Msk (0x1UL << ADC_CSR_JQOVF_SLV_Pos)
#define ADC_CSR_JQOVF_SLV ADC_CSR_JQOVF_SLV_Msk

#define ADC_CSR_AWD3_SLV_Pos 25U
#define ADC_CSR_AWD3_SLV_Msk (0x1UL << ADC_CSR_AWD3_SLV_Pos)
#define ADC_CSR_AWD3_SLV ADC_CSR_AWD3_SLV_Msk

#define ADC_CSR_AWD2_SLV_Pos 24U
#define ADC_CSR_AWD2_SLV_Msk (0x1UL << ADC_CSR_AWD2_SLV_Pos)
#define ADC_CSR_AWD2_SLV ADC_CSR_AWD2_SLV_Msk

#define ADC_CSR_AWD1_SLV_Pos 23U
#define ADC_CSR_AWD1_SLV_Msk (0x1UL << ADC_CSR_AWD1_SLV_Pos)
#define ADC_CSR_AWD1_SLV ADC_CSR_AWD1_SLV_Msk

#define ADC_CSR_JEOS_SLV_Pos 22U
#define ADC_CSR_JEOS_SLV_Msk (0x1UL << ADC_CSR_JEOS_SLV_Pos)
#define ADC_CSR_JEOS_SLV ADC_CSR_JEOS_SLV_Msk

#define ADC_CSR_JEOC_SLV_Pos 21U
#define ADC_CSR_JEOC_SLV_Msk (0x1UL << ADC_CSR_JEOC_SLV_Pos)
#define ADC_CSR_JEOC_SLV ADC_CSR_JEOC_SLV_Msk

#define ADC_CSR_OVR_SLV_Pos 20U
#define ADC_CSR_OVR_SLV_Msk (0x1UL << ADC_CSR_OVR_SLV_Pos)
#define ADC_CSR_OVR_SLV ADC_CSR_OVR_SLV_Msk

#define ADC_CSR_EOS_SLV_Pos 19U
#define ADC_CSR_EOS_SLV_Msk (0x1UL << ADC_CSR_EOS_SLV_Pos)
#define ADC_CSR_EOS_SLV ADC_CSR_EOS_SLV_Msk

#define ADC_CSR_EOC_SLV_Pos 18U
#define ADC_CSR_EOC_SLV_Msk (0x1UL << ADC_CSR_EOC_SLV_Pos)
#define ADC_CSR_EOC_SLV ADC_CSR_EOC_SLV_Msk

#define ADC_CSR_EOSMP_SLV_Pos 17U
#define ADC_CSR_EOSMP_SLV_Msk (0x1UL << ADC_CSR_EOSMP_SLV_Pos)
#define ADC_CSR_EOSMP_SLV ADC_CSR_EOSMP_SLV_Msk

#define ADC_CSR_ADRDY_SLV_Pos 16U
#define ADC_CSR_ADRDY_SLV_Msk (0x1UL << ADC_CSR_ADRDY_SLV_Pos)
#define ADC_CSR_ADRDY_SLV ADC_CSR_ADRDY_SLV_Msk

#define ADC_CSR_JQOVF_MST_Pos 10U
#define ADC_CSR_JQOVF_MST_Msk (0x1UL << ADC_CSR_JQOVF_MST_Pos)
#define ADC_CSR_JQOVF_MST ADC_CSR_JQOVF_MST_Msk

#define ADC_CSR_AWD3_MST_Pos 9U
#define ADC_CSR_AWD3_MST_Msk (0x1UL << ADC_CSR_AWD3_MST_Pos)
#define ADC_CSR_AWD3_MST ADC_CSR_AWD3_MST_Msk

#define ADC_CSR_AWD2_MST_Pos 8U
#define ADC_CSR_AWD2_MST_Msk (0x1UL << ADC_CSR_AWD2_MST_Pos)
#define ADC_CSR_AWD2_MST ADC_CSR_AWD2_MST_Msk

#define ADC_CSR_AWD1_MST_Pos 7U
#define ADC_CSR_AWD1_MST_Msk (0x1UL << ADC_CSR_AWD1_MST_Pos)
#define ADC_CSR_AWD1_MST ADC_CSR_AWD1_MST_Msk

#define ADC_CSR_JEOS_MST_Pos 6U
#define ADC_CSR_JEOS_MST_Msk (0x1UL << ADC_CSR_JEOS_MST_Pos)
#define ADC_CSR_JEOS_MST ADC_CSR_JEOS_MST_Msk

#define ADC_CSR_JEOC_MST_Pos 5U
#define ADC_CSR_JEOC_MST_Msk (0x1UL << ADC_CSR_JEOC_MST_Pos)
#define ADC_CSR_JEOC_MST ADC_CSR_JEOC_MST_Msk

#define ADC_CSR_OVR_MST_Pos 4U
#define ADC_CSR_OVR_MST_Msk (0x1UL << ADC_CSR_OVR_MST_Pos)
#define ADC_CSR_OVR_MST ADC_CSR_OVR_MST_Msk

#define ADC_CSR_EOS_MST_Pos 3U
#define ADC_CSR_EOS_MST_Msk (0x1UL << ADC_CSR_EOS_MST_Pos)
#define ADC_CSR_EOS_MST ADC_CSR_EOS_MST_Msk

#define ADC_CSR_EOC_MST_Pos 2U
#define ADC_CSR_EOC_MST_Msk (0x1UL << ADC_CSR_EOC_MST_Pos)
#define ADC_CSR_EOC_MST ADC_CSR_EOC_MST_Msk

#define ADC_CSR_EOSMP_MST_Pos 1U
#define ADC_CSR_EOSMP_MST_Msk (0x1UL << ADC_CSR_EOSMP_MST_Pos)
#define ADC_CSR_EOSMP_MST ADC_CSR_EOSMP_MST_Msk

#define ADC_CSR_ADRDY_MST_Pos 0U
#define ADC_CSR_ADRDY_MST_Msk (0x1UL << ADC_CSR_ADRDY_MST_Pos)
#define ADC_CSR_ADRDY_MST ADC_CSR_ADRDY_MST_Msk

/*CCR register*/
#define ADC_CCR_CH18SEL_Pos 24U
#define ADC_CCR_CH18SEL_Msk (0x1UL << ADC_CCR_CH18SEL_Pos)
#define ADC_CCR_CH18SEL ADC_CCR_CH18SEL_Msk

#define ADC_CCR_CH17SEL_Pos 23U
#define ADC_CCR_CH17SEL_Msk (0x1UL << ADC_CCR_CH17SEL_Pos)
#define ADC_CCR_CH17SEL ADC_CCR_CH17SEL_Msk

#define ADC_CCR_VREFEN_Pos 22U
#define ADC_CCR_VREFEN_Msk (0x1UL << ADC_CCR_VREFEN_Pos)
#define ADC_CCR_VREFEN ADC_CCR_VREFEN_Msk

#define ADC_CCR_PRESC_Pos 18U
#define ADC_CCR_PRESC_Msk (0xFUL << ADC_CCR_PRESC_Pos)
#define ADC_CCR_PRESC ADC_CCR_PRESC_Msk
#define ADC_CCR_PRESC_DIV_1 (0x0UL << ADC_CCR_PRESC_Pos)
#define ADC_CCR_PRESC_DIV_2 (0x1UL << ADC_CCR_PRESC_Pos)
#define ADC_CCR_PRESC_DIV_4 (0x2UL << ADC_CCR_PRESC_Pos)
#define ADC_CCR_PRESC_DIV_6 (0x3UL << ADC_CCR_PRESC_Pos)
#define ADC_CCR_PRESC_DIV_8 (0x4UL << ADC_CCR_PRESC_Pos)
#define ADC_CCR_PRESC_DIV_10 (0X5FUL << ADC_CCR_PRESC_Pos)
#define ADC_CCR_PRESC_DIV_12 (0x6UL << ADC_CCR_PRESC_Pos)
#define ADC_CCR_PRESC_DIV_16 (0x7UL << ADC_CCR_PRESC_Pos)
#define ADC_CCR_PRESC_DIV_32 (0x8UL << ADC_CCR_PRESC_Pos)
#define ADC_CCR_PRESC_DIV_64 (0x9UL << ADC_CCR_PRESC_Pos)
#define ADC_CCR_PRESC_DIV_128 (0xAUL << ADC_CCR_PRESC_Pos)
#define ADC_CCR_PRESC_DIV_256 (0xBUL << ADC_CCR_PRESC_Pos)

#define ADC_CCR_CKMODE_Pos 16U
#define ADC_CCR_CKMODE_Msk (0x3UL << ADC_CCR_CKMODE_Pos)
#define ADC_CCR_CKMODE ADC_CCR_CKMODE_Msk
#define ADC_CCR_CKMODE_CK_ADC (0x0UL << ADC_CCR_CKMODE_Pos)
#define ADC_CCR_CKMODE_HCLK_DIV_1 (0x1UL << ADC_CCR_CKMODE_Pos)
#define ADC_CCR_CKMODE_HCLK_DIV_2 (0x2UL << ADC_CCR_CKMODE_Pos)
#define ADC_CCR_CKMODE_HCLK_DIV_4 (0x3UL << ADC_CCR_CKMODE_Pos)

#define ADC_CCR_MDMA_Pos 14U
#define ADC_CCR_MDMA_Msk (0x3UL << ADC_CCR_MDMA_Pos)
#define ADC_CCR_MDMA ADC_CCR_MDMA_Msk
#define ADC_CCR_MDMA_DISABLED (0x0UL << ADC_CCR_MDMA_Pos)
#define ADC_CCR_MDMA_DFSDM_16_BIT (0x1UL << ADC_CCR_MDMA_Pos)
#define ADC_CCR_MDMA_10_12_BIT (0x2UL << ADC_CCR_MDMA_Pos)
#define ADC_CCR_MDMA_8_6_BIT (0x3UL << ADC_CCR_MDMA_Pos)

#define ADC_CCR_DMACFG_Pos 13U
#define ADC_CCR_DMACFG_Msk (0x1UL << ADC_CCR_DMACFG_Pos)
#define ADC_CCR_DMACFG ADC_CCR_DMACFG_Msk

#define ADC_CCR_DELAY_Pos 8U
#define ADC_CCR_DELAY_Msk (0xFUL << ADC_CCR_DELAY_Pos)
#define ADC_CCR_DELAY ADC_CCR_DELAY_Msk

#define ADC_CCR_DUAL_Pos 0U
#define ADC_CCR_DUAL_Msk (0x1FUL << ADC_CCR_DUAL_Pos)
#define ADC_CCR_DUAL ADC_CCR_DUAL_Msk
#define ADC_CCR_DUAL_INDEPENDANT (0x0UL << ADC_CCR_DUAL_Pos)
#define ADC_CCR_DUAL_REGULAR_SIM_INJECTED_SIM (0x1UL << ADC_CCR_DUAL_Pos)
#define ADC_CCR_DUAL_REGULAR_SIM_ALT_TRIGGER (0x2UL << ADC_CCR_DUAL_Pos)
#define ADC_CCR_DUAL_INTERLEAVED_INJECTED_SIM (0x3UL << ADC_CCR_DUAL_Pos)
#define ADC_CCR_DUAL_INJECTED_SIM (0x5UL << ADC_CCR_DUAL_Pos)
#define ADC_CCR_DUAL_REGULAR_SIM (0x6UL << ADC_CCR_DUAL_Pos)
#define ADC_CCR_DUAL_INTERLEAVED (0x7UL << ADC_CCR_DUAL_Pos)
#define ADC_CCR_DUAL_ALT_TRIGGER (0x9UL << ADC_CCR_DUAL_Pos)

/*CDR register*/
#define ADC_CDR_RDATA_SLV_Pos 16U
#define ADC_CDR_RDATA_SLV_Msk (0xFFFFUL << ADC_CDR_RDATA_SLV_Pos)
#define ADC_CDR_RDATA_SLV ADC_CDR_RDATA_SLV_Msk

#define ADC_CDR_RDATA_MST_Pos 0U
#define ADC_CDR_RDATA_MST_Msk (0xFFFFUL << ADC_CDR_RDATA_MST_Pos)
#define ADC_CDR_RDATA_MST ADC_CDR_RDATA_MST_Msk

#if !defined(STM32L412) && !defined(STM32L422)
/* ========================================================================= */
/* ============                       DAC                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR; /*!< DAC Control Register. */
	__OM uint32_t SWTRGR; /*!< DAC Software Trigger Register. */
	__IOM uint32_t DHR12R1; /*!< DAC channel 1 12-bit Right-aligned Data Holding Register. */
	__IOM uint32_t DHR12L1; /*!< DAC channel 1 12-bit Left-aligned Data Holding Register. */
	__IOM uint32_t DHR8R1; /*!< DAC channel 1 8-bit Right-aligned Data Holding Register. */
	__IOM uint32_t DHR12R2; /*!< DAC channel 2 12-bit Right-aligned Data Holding Register. */
	__IOM uint32_t DHR12L2; /*!< DAC channel 2 12-bit Left-aligned Data Holding Register. */
	__IOM uint32_t DHR8R2; /*!< DAC channel 2 8-bit Right-aligned Data Holding Register. */
	__IOM uint32_t DHR12RD; /*!< DAC Dual channel 12-bit Right-aligned Data Holding Register. */
	__IOM uint32_t DHR12LD; /*!< DAC Dual channel 12-bit Left-aligned Data Holding Register. */
	__IOM uint32_t DHR8RD; /*!< DAC Dual channel 8-bit Right-aligned Data Holding Register. */
	__IM uint32_t DOR1; /*!< DAC channel 1 Data Output Register. */
	__IM uint32_t DOR2; /*!< DAC channel 2 Data Output Register. */
	__IOM uint32_t SR; /*!< DAC Status Register. */
	__IOM uint32_t CCR; /*!< DAC Calibration Control Register. */
	__IOM uint32_t MCR; /*!< DAC Mode Control Register. */
	__IOM uint32_t SHSR1; /*!< DAC channel 1 Sample and Hold Sample Register. */
	__IOM uint32_t SHSR2; /*!< DAC channel 2 Sample and Hold Sample Register. */
	__IOM uint32_t SHHR; /*!< DAC Sample and Hold time Register. */
	__IOM uint32_t SHRR; /*!< DAC Sample and Hold Refresh time Register. */
}STM32L4xx_DAC_TypeDef;

/*CR register*/
#define DAC_CR_CEN2_Pos 30U
#define DAC_CR_CEN2_Msk (0x1UL << DAC_CR_CEN2_Pos)
#define DAC_CR_CEN2 DAC_CR_CEN2_Msk

#define DAC_CR_DMAUDRIE2_Pos 29U
#define DAC_CR_DMAUDRIE2_Msk (0x1UL << DAC_CR_CEN2_Pos)
#define DAC_CR_DMAUDRIE2 DAC_CR_DMAUDRIE2_Msk

#define DAC_CR_DMAEN2_Pos 28U
#define DAC_CR_DMAEN2_Msk (0x1UL << DAC_CR_DMAEN2_Pos)
#define DAC_CR_DMAEN2 DAC_CR_DMAEN2_Msk

#define DAC_CR_MAMP2_Pos 24U
#define DAC_CR_MAMP2_Msk (0xFUL << DAC_CR_MAMP2_Pos)
#define DAC_CR_MAMP2 DAC_CR_MAMP2_Msk
#define DAC_CR_MAMP2_AMPLITUDE_1 (0x0UL << DAC_CR_MAMP2_Pos)
#define DAC_CR_MAMP2_AMPLITUDE_3 (0x1UL << DAC_CR_MAMP2_Pos)
#define DAC_CR_MAMP2_AMPLITUDE_7 (0x2UL << DAC_CR_MAMP2_Pos)
#define DAC_CR_MAMP2_AMPLITUDE_15 (0x3UL << DAC_CR_MAMP2_Pos)
#define DAC_CR_MAMP2_AMPLITUDE_31 (0x4UL << DAC_CR_MAMP2_Pos)
#define DAC_CR_MAMP2_AMPLITUDE_63 (0x5UL << DAC_CR_MAMP2_Pos)
#define DAC_CR_MAMP2_AMPLITUDE_127 (0x6UL << DAC_CR_MAMP2_Pos)
#define DAC_CR_MAMP2_AMPLITUDE_255 (0x7UL << DAC_CR_MAMP2_Pos)
#define DAC_CR_MAMP2_AMPLITUDE_511 (0x8UL << DAC_CR_MAMP2_Pos)
#define DAC_CR_MAMP2_AMPLITUDE_1023 (0x9UL << DAC_CR_MAMP2_Pos)
#define DAC_CR_MAMP2_AMPLITUDE_2047 (0xAUL << DAC_CR_MAMP2_Pos)

#define DAC_CR_WAVE2_Pos 22U
#define DAC_CR_WAVE2_Msk (0x3UL << DAC_CR_WAVE2_Pos)
#define DAC_CR_WAVE2 DAC_CR_WAVE2_Msk
#define DAC_CR_WAVE2_DISABLED (0x0UL << DAC_CR_WAVE2_Pos)
#define DAC_CR_WAVE2_NOISE_ENABLED (0x1UL << DAC_CR_WAVE2_Pos)

#define DAC_CR_TSEL2_Pos 19U
#define DAC_CR_TSEL2_Msk (0x7UL << DAC_CR_TSEL2_Pos)
#define DAC_CR_TSEL2 DAC_CR_TSEL2_Msk

#define DAC_CR_TEN2_Pos 18U
#define DAC_CR_TEN2_Msk (0x1UL << DAC_CR_TEN2_Pos)
#define DAC_CR_TEN2 DAC_CR_TEN2_Msk

#define DAC_CR_EN2_Pos 16U
#define DAC_CR_EN2_Msk (0x1UL << DAC_CR_EN2_Pos)
#define DAC_CR_EN2 DAC_CR_EN2_Msk

#define DAC_CR_CEN1_Pos 14U
#define DAC_CR_CEN1_Msk (0x1UL << DAC_CR_CEN1_Pos)
#define DAC_CR_CEN1 DAC_CR_CEN1_Msk

#define DAC_CR_DMAUDRIE1_Pos 13U
#define DAC_CR_DMAUDRIE1_Msk (0x1UL << DAC_CR_DMAUDRIE1_Pos)
#define DAC_CR_DMAUDRIE1 DAC_CR_DMAUDRIE1_Msk

#define DAC_CR_DMAEN1_Pos 12U
#define DAC_CR_DMAEN1_Msk (0x1UL << DAC_CR_DMAEN1_Pos)
#define DAC_CR_DMAEN1 DAC_CR_DMAEN1_Msk

#define DAC_CR_MAMP1_Pos 8U
#define DAC_CR_MAMP1_Msk (0xFUL << DAC_CR_MAMP1_Pos)
#define DAC_CR_MAMP1 DAC_CR_MAMP1_Msk
#define DAC_CR_MAMP1_AMPLITUDE_1 (0x0UL << DAC_CR_MAMP1_Pos)
#define DAC_CR_MAMP1_AMPLITUDE_3 (0x1UL << DAC_CR_MAMP1_Pos)
#define DAC_CR_MAMP1_AMPLITUDE_7 (0x2UL << DAC_CR_MAMP1_Pos)
#define DAC_CR_MAMP1_AMPLITUDE_15 (0x3UL << DAC_CR_MAMP1_Pos)
#define DAC_CR_MAMP1_AMPLITUDE_31 (0x4UL << DAC_CR_MAMP1_Pos)
#define DAC_CR_MAMP1_AMPLITUDE_63 (0x5UL << DAC_CR_MAMP1_Pos)
#define DAC_CR_MAMP1_AMPLITUDE_127 (0x6UL << DAC_CR_MAMP1_Pos)
#define DAC_CR_MAMP1_AMPLITUDE_255 (0x7UL << DAC_CR_MAMP1_Pos)
#define DAC_CR_MAMP1_AMPLITUDE_511 (0x8UL << DAC_CR_MAMP1_Pos)
#define DAC_CR_MAMP1_AMPLITUDE_1023 (0x9UL << DAC_CR_MAMP1_Pos)
#define DAC_CR_MAMP1_AMPLITUDE_2047 (0xAUL << DAC_CR_MAMP1_Pos)

#define DAC_CR_WAVE1_Pos 6U
#define DAC_CR_WAVE1_Msk (0x3UL << DAC_CR_WAVE1_Pos)
#define DAC_CR_WAVE1 DAC_CR_WAVE1_Msk
#define DAC_CR_WAVE1_DISABLED (0x0UL << DAC_CR_WAVE1_Pos)
#define DAC_CR_WAVE1_NOISE_ENABLED (0x1UL << DAC_CR_WAVE1_Pos)

#define DAC_CR_TSEL1_Pos 3U
#define DAC_CR_TSEL1_Msk (0x7UL << DAC_CR_TSEL1_Pos)
#define DAC_CR_TSEL1 DAC_CR_TSEL1_Msk

#define DAC_CR_TEN1_Pos 2U
#define DAC_CR_TEN1_Msk (0x1UL << DAC_CR_TEN1_Pos)
#define DAC_CR_TEN1 DAC_CR_TEN1_Msk

#define DAC_CR_EN1_Pos 0U
#define DAC_CR_EN1_Msk (0x1UL << DAC_CR_EN1_Pos)
#define DAC_CR_EN1 DAC_CR_EN1_Msk

/*SWTRGR register*/
#define DAC_SWTRGR_SWTRIG2_Pos 1U
#define DAC_SWTRGR_SWTRIG2_Msk (0x1UL << DAC_SWTRGR_SWTRIG2_Pos)
#define DAC_SWTRGR_SWTRIG2 DAC_SWTRGR_SWTRIG2_Msk

#define DAC_SWTRGR_SWTRIG1_Pos 0U
#define DAC_SWTRGR_SWTRIG1_Msk (0x1UL << DAC_SWTRGR_SWTRIG1_Pos)
#define DAC_SWTRGR_SWTRIG1 DAC_SWTRGR_SWTRIG1_Msk

/*DHR12R1 register*/
#define DAC_DHR12R1_DACC1DHR_Pos 0U
#define DAC_DHR12R1_DACC1DHR_Msk (0xFFFUL << DAC_DHR12R1_DACC1DHR_Pos)
#define DAC_DHR12R1_DACC1DHR DAC_DHR12R1_DACC1DHR_Msk

/*DHR12L1 register*/
#define DAC_DHR12L1_DACC1DHR_Pos 4U
#define DAC_DHR12L1_DACC1DHR_Msk (0xFFFUL << DAC_DHR12L1_DACC1DHR_Pos)
#define DAC_DHR12L1_DACC1DHR DAC_DHR12L1_DACC1DHR_Msk

/*DHR8R1 register*/
#define DAC_DHR8R1_DACC1DHR_Pos 0U
#define DAC_DHR8R1_DACC1DHR_Msk (0xFFUL << DAC_DHR8R1_DACC1DHR_Pos)
#define DAC_DHR8R1_DACC1DHR DAC_DHR8R1_DACC1DHR_Msk

/*DHR12R2 register*/
#define DAC_DHR12R2_DACC2DHR_Pos 0U
#define DAC_DHR12R2_DACC2DHR_Msk (0xFFFUL << DAC_DHR12R2_DACC2DHR_Pos)
#define DAC_DHR12R2_DACC2DHR DAC_DHR12R2_DACC2DHR_Msk

/*DHR12L2 register*/
#define DAC_DHR12L2_DACC2DHR_Pos 4U
#define DAC_DHR12L2_DACC2DHR_Msk (0xFFFUL << DAC_DHR12L2_DACC2DHR_Pos)
#define DAC_DHR12L2_DACC2DHR DAC_DHR12L2_DACC2DHR_Msk

/*DHR8R2 register*/
#define DAC_DHR8R2_DACC2DHR_Pos 0U
#define DAC_DHR8R2_DACC2DHR_Msk (0xFFUL << DAC_DHR8R2_DACC2DHR_Pos)
#define DAC_DHR8R2_DACC2DHR DAC_DHR8R2_DACC2DHR_Msk

/*DHR12RD register*/
#define DAC_DHR12RD_DACC2DHR_Pos 16U
#define DAC_DHR12RD_DACC2DHR_Msk (0xFFFUL << DAC_DHR12RD_DACC2DHR_Pos)
#define DAC_DHR12RD_DACC2DHR DAC_DHR12RD_DACC2DHR_Msk

#define DAC_DHR12RD_DACC1DHR_Pos 0U
#define DAC_DHR12RD_DACC1DHR_Msk (0xFFFUL << DAC_DHR12RD_DACC1DHR_Pos)
#define DAC_DHR12RD_DACC1DHR DAC_DHR12RD_DACC1DHR_Msk

/*DHR12LD register*/
#define DAC_DHR12LD_DACC2DHR_Pos 20U
#define DAC_DHR12LD_DACC2DHR_Msk (0xFFFUL << DAC_DHR12LD_DACC2DHR_Pos)
#define DAC_DHR12LD_DACC2DHR DAC_DHR12LD_DACC2DHR_Msk

#define DAC_DHR12LD_DACC1DHR_Pos 4U
#define DAC_DHR12LD_DACC1DHR_Msk (0xFFFUL << DAC_DHR12LD_DACC1DHR_Pos)
#define DAC_DHR12LD_DACC1DHR DAC_DHR12LD_DACC1DHR_Msk

/*DHR8RD register*/
#define DAC_DHR8RD_DACC2DHR_Pos 8U
#define DAC_DHR8RD_DACC2DHR_Msk (0xFFUL << DAC_DHR8RD_DACC2DHR_Pos)
#define DAC_DHR8RD_DACC2DHR DAC_DHR8RD_DACC2DHR_Msk

#define DAC_DHR8RD_DACC1DHR_Pos 0U
#define DAC_DHR8RD_DACC1DHR_Msk (0xFFUL << DAC_DHR8RD_DACC1DHR_Pos)
#define DAC_DHR8RD_DACC1DHR DAC_DHR8RD_DACC1DHR_Msk

/*DOR1 register*/
#define DAC_DOR1_DACC1DOR_Pos 0U
#define DAC_DOR1_DACC1DOR_Msk (0xFFFUL << DAC_DOR1_DACC1DOR_Pos)
#define DAC_DOR1_DACC1DOR DAC_DOR1_DACC1DOR_Msk

/*DOR2 register*/
#define DAC_DOR2_DACC2DOR_Pos 0U
#define DAC_DOR2_DACC2DOR_Msk (0xFFFUL << DAC_DOR2_DACC2DOR_Pos)
#define DAC_DOR2_DACC2DOR DAC_DOR2_DACC2DOR_Msk

/*SR register*/
#define DAC_SR_BWST2_Pos 31U
#define DAC_SR_BWST2_Msk (0x1UL << DAC_SR_BWST2_Pos)
#define DAC_SR_BWST2 DAC_SR_BWST2_Msk

#define DAC_SR_CAL_FLAG2_Pos 30U
#define DAC_SR_CAL_FLAG2_Msk (0x1UL << DAC_SR_CAL_FLAG2_Pos)
#define DAC_SR_CAL_FLAG2 DAC_SR_CAL_FLAG2_Msk

#define DAC_SR_DMAUDR2_Pos 29U
#define DAC_SR_DMAUDR2_Msk (0x1UL << DAC_SR_DMAUDR2_Pos)
#define DAC_SR_DMAUDR2 DAC_SR_DMAUDR2_Msk

#define DAC_SR_BWST1_Pos 15U
#define DAC_SR_BWST1_Msk (0x1UL << DAC_SR_BWST1_Pos)
#define DAC_SR_BWST1 DAC_SR_BWST1_Msk

#define DAC_SR_CAL_FLAG1_Pos 14U
#define DAC_SR_CAL_FLAG1_Msk (0x1UL << DAC_SR_CAL_FLAG1_Pos)
#define DAC_SR_CAL_FLAG1 DAC_SR_CAL_FLAG1_Msk

#define DAC_SR_DMAUDR1_Pos 13U
#define DAC_SR_DMAUDR1_Msk (0x1UL << DAC_SR_DMAUDR1_Pos)
#define DAC_SR_DMAUDR1 DAC_SR_DMAUDR1_Msk

/*CCR register*/
#define DAC_CCR_OTRIM2_Pos 16U
#define DAC_CCR_OTRIM2_Msk (0x1FUL << DAC_CCR_OTRIM2_Pos)
#define DAC_CCR_OTRIM2 DAC_CCR_OTRIM2_Msk

#define DAC_CCR_OTRIM1_Pos 0U
#define DAC_CCR_OTRIM1_Msk (0x1FUL << DAC_CCR_OTRIM1_Pos)
#define DAC_CCR_OTRIM1 DAC_CCR_OTRIM1_Msk

/*MCR register*/
#define DAC_MCR_MODE2_Pos 16U
#define DAC_MCR_MODE2_Msk (0x7UL << DAC_MCR_MODE2_Pos)
#define DAC_MCR_MODE2 DAC_MCR_MODE2_Msk

#define DAC_MCR_MODE1_Pos 0U
#define DAC_MCR_MODE1_Msk (0x7UL << DAC_MCR_MODE1_Pos)
#define DAC_MCR_MODE1 DAC_MCR_MODE1_Msk

/*SHSR1 register*/
#define DAC_SHSR1_TSAMPLE1_Pos 0U
#define DAC_SHSR1_TSAMPLE1_Msk (0x3FFUL << DAC_SHSR1_TSAMPLE1_Pos)
#define DAC_SHSR1_TSAMPLE1 DAC_SHSR1_TSAMPLE1_Msk

/*SHSR2 register*/
#define DAC_SHSR2_TSAMPLE2_Pos 0U
#define DAC_SHSR2_TSAMPLE2_Msk (0x3FFUL << DAC_SHSR2_TSAMPLE2_Pos)
#define DAC_SHSR2_TSAMPLE2 DAC_SHSR2_TSAMPLE2_Msk

/*SHHR register*/
#define DAC_SHHR_THOLD2_Pos 16U
#define DAC_SHHR_THOLD2_Msk (0x3FFUL << DAC_SHHR_THOLD2_Pos)
#define DAC_SHHR_THOLD2 DAC_SHHR_THOLD2_Msk

#define DAC_SHHR_THOLD1_Pos 0U
#define DAC_SHHR_THOLD1_Msk (0x3FFUL << DAC_SHHR_THOLD1_Pos)
#define DAC_SHHR_THOLD1 DAC_SHHR_THOLD1_Msk

/*SHRR register*/
#define DAC_SHRR_TREFRESH2_Pos 16U
#define DAC_SHRR_TREFRESH2_Msk (0xFFUL << DAC_SHRR_TREFRESH2_Pos)
#define DAC_SHRR_TREFRESH2 DAC_SHRR_TREFRESH2_Msk

#define DAC_SHRR_TREFRESH1_Pos 0U
#define DAC_SHRR_TREFRESH1_Msk (0xFFUL << DAC_SHRR_TREFRESH1_Pos)
#define DAC_SHRR_TREFRESH1 DAC_SHRR_TREFRESH1_Msk
#endif

#if !defined(STM32L412) && !defined(STM32L422)&& !defined(STM32L432) && !defined(STM32L442)
/* ========================================================================= */
/* ============                     VREFBUF                     ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CSR; /*!< VERFBUF Control and Status Register. */
	__IOM uint32_t CCR; /*!< VERFBUF Calibration Control Register. */
}STM32L4xx_VREFBUF_TypeDef;

/*CSR register*/
#define VREFBUF_CSR_VRR_Pos 3U
#define VREFBUF_CSR_VRR_Msk (0x1UL << VREFBUF_CSR_VRR_Pos)
#define VREFBUF_CSR_VRR VREFBUF_CSR_VRR_Msk

#define VREFBUF_CSR_VRS_Pos 2U
#define VREFBUF_CSR_VRS_Msk (0x1UL << VREFBUF_CSR_VRS_Pos)
#define VREFBUF_CSR_VRS VREFBUF_CSR_VRS_Msk

#define VREFBUF_CSR_HIZ_Pos 1U
#define VREFBUF_CSR_HIZ_Msk (0x1UL << VREFBUF_CSR_HIZ_Pos)
#define VREFBUF_CSR_HIZ VREFBUF_CSR_HIZ_Msk

#define VREFBUF_CSR_ENVR_Pos 0U
#define VREFBUF_CSR_ENVR_Msk (0x1UL << VREFBUF_CSR_ENVR_Pos)
#define VREFBUF_CSR_ENVR VREFBUF_CSR_ENVR_Msk
/*CCR register*/
#define VREFBUF_CCR_TRIM_Pos 0U
#define VREFBUF_CCR_TRIM_Msk (0x3FUL << VREFBUF_CCR_TRIM_Pos)
#define VREFBUF_CCR_TRIM VREFBUF_CCR_TRIM_Msk
#endif

/* ========================================================================= */
/* ============                       COMP                      ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CSR; /*!< Comparator Control and Status Register. */
}STM32L4xx_COMP_TypeDef;

/*CSR register.*/
#define COMP_CSR_LOCK_Pos 31U
#define COMP_CSR_LOCK_Msk (0x1UL << COMP_CSR_LOCK_Pos)
#define COMP_CSR_LOCK COMP_CSR_LOCK_Msk

#define COMP_CSR_VALUE_Pos 30U
#define COMP_CSR_VALUE_Msk (0x1UL << COMP_CSR_VALUE_Pos)
#define COMP_CSR_VALUE COMP_CSR_VALUE_Msk

#define COMP_CSR_INMESEL_Pos 25U
#define COMP_CSR_INMESEL_Msk (0x3UL << COMP_CSR_INMESEL_Pos)
#define COMP_CSR_INMESEL COMP_CSR_INMESEL_Msk
#define COMP1_CSR_INMESEL_PC4 (0x0UL << COMP_CSR_INMESEL_Pos)
#define COMP1_CSR_INMESEL_PA0 (0x1UL << COMP_CSR_INMESEL_Pos)
#define COMP1_CSR_INMESEL_PA4 (0x2UL << COMP_CSR_INMESEL_Pos)
#define COMP1_CSR_INMESEL_PA5 (0x3UL << COMP_CSR_INMESEL_Pos)
#define COMP2_CSR_INMESEL_PB7 (0x0UL << COMP_CSR_INMESEL_Pos)
#define COMP2_CSR_INMESEL_PA2 (0x1UL << COMP_CSR_INMESEL_Pos)
#define COMP2_CSR_INMESEL_PA4 (0x2UL << COMP_CSR_INMESEL_Pos)
#define COMP2_CSR_INMESEL_PA5 (0x3UL << COMP_CSR_INMESEL_Pos)

#define COMP_CSR_SCALEN_Pos 23U
#define COMP_CSR_SCALEN_Msk (0x1UL << COMP_CSR_SCALEN_Pos)
#define COMP_CSR_SCALEN COMP_CSR_SCALEN_Msk

#define COMP_CSR_BRGEN_Pos 22U
#define COMP_CSR_BRGEN_Msk (0x1UL << COMP_CSR_BRGEN_Pos)
#define COMP_CSR_BRGEN COMP_CSR_BRGEN_Msk

#define COMP_CSR_BLANKING_Pos 18U
#define COMP_CSR_BLANKING_Msk (0x7UL << COMP_CSR_BLANKING_Pos)
#define COMP_CSR_BLANKING COMP_CSR_BLANKING_Msk
#define COMP1_CSR_BLANKING_NONE (0x0UL << COMP_CSR_BLANKING_Pos)
#define COMP1_CSR_BLANKING_TIM1_OC5 (0x1UL << COMP_CSR_BLANKING_Pos)
#define COMP1_CSR_BLANKING_TIM2_OC3 (0x2UL << COMP_CSR_BLANKING_Pos)
#define COMP2_CSR_BLANKING_NONE (0x0UL << COMP_CSR_BLANKING_Pos)
#define COMP2_CSR_BLANKING_TIM15_OC5 (0x4UL << COMP_CSR_BLANKING_Pos)

#define COMP_CSR_HYST_Pos 16U
#define COMP_CSR_HYST_Msk (0x3UL << COMP_CSR_HYST_Pos)
#define COMP_CSR_HYST COMP_CSR_HYST_Msk
#define COMP_CSR_HYST_NONE (0x0UL << COMP_CSR_HYST_Pos)
#define COMP_CSR_HYST_LOW (0x1UL << COMP_CSR_HYST_Pos)
#define COMP_CSR_HYST_MEDIUM (0x2UL << COMP_CSR_HYST_Pos)
#define COMP_CSR_HYST_HIGH (0x3UL << COMP_CSR_HYST_Pos)

#define COMP_CSR_POLARITY_Pos 15U
#define COMP_CSR_POLARITY_Msk (0x1UL << COMP_CSR_POLARITY_Pos)
#define COMP_CSR_POLARITY COMP_CSR_POLARITY_Msk

#define COMP2_CSR_WINMODE_Pos 9U
#define COMP2_CSR_WINMODE_Msk (0x1UL << COMP2_CSR_WINMODE_Pos)
#define COMP2_CSR_WINMODE COMP2_CSR_WINMODE_Msk

#define COMP_CSR_INPSEL_Pos 7U
#define COMP_CSR_INPSEL_Msk (0x3UL << COMP_CSR_INPSEL_Pos)
#define COMP_CSR_INPSEL COMP_CSR_INPSEL_Msk
#define COMP1_CSR_INPSEL_PC5 (0x0UL << COMP_CSR_INPSEL_Pos)
#define COMP1_CSR_INPSEL_PB2 (0x1UL << COMP_CSR_INPSEL_Pos)
#define COMP1_CSR_INPSEL_PA1 (0x2UL << COMP_CSR_INPSEL_Pos)
#define COMP2_CSR_INPSEL_PB4 (0x0UL << COMP_CSR_INPSEL_Pos)
#define COMP2_CSR_INPSEL_PB6 (0x1UL << COMP_CSR_INPSEL_Pos)
#define COMP2_CSR_INPSEL_PA3 (0x2UL << COMP_CSR_INPSEL_Pos)

#define COMP_CSR_INMSEL_Pos 4U
#define COMP_CSR_INMSEL_Msk (0x7UL << COMP_CSR_INMSEL_Pos)
#define COMP_CSR_INMSEL COMP_CSR_INMSEL_Msk
#define COMP_CSR_INMSEL_VREFINT_QUARTER (0x0UL << COMP_CSR_INMSEL_Pos)
#define COMP_CSR_INMSEL_VREFINT_HALF (0x1UL << COMP_CSR_INMSEL_Pos)
#define COMP_CSR_INMSEL_VREFINT_THREE_QUARTERS (0x2UL << COMP_CSR_INMSEL_Pos)
#define COMP_CSR_INMSEL_VREFINT (0x3UL << COMP_CSR_INMSEL_Pos)
#define COMP_CSR_INMSEL_DAC_CH1 (0x4UL << COMP_CSR_INMSEL_Pos)
#define COMP_CSR_INMSEL_DAC_CH2 (0x5UL << COMP_CSR_INMSEL_Pos)
#define COMP1_CSR_INMSEL_PB1 (0x6UL << COMP_CSR_INMSEL_Pos)
#define COMP2_CSR_INMSEL_PB3 (0x6UL << COMP_CSR_INMSEL_Pos)
#define COMP_CSR_INMSEL_INMESEL (0x7UL << COMP_CSR_INMSEL_Pos)

#define COMP_CSR_PWRMODE_Pos 2U
#define COMP_CSR_PWRMODE_Msk (0x3UL << COMP_CSR_PWRMODE_Pos)
#define COMP_CSR_PWRMODE COMP_CSR_PWRMODE_Msk 
#define COMP_CSR_PWRMODE_HIGH_SPEED (0x0UL << COMP_CSR_PWRMODE_Pos)
#define COMP_CSR_PWRMODE_ULTRA_LOW_POWER (0x3UL << COMP_CSR_PWRMODE_Pos)

#define COMP_CSR_EN_Pos 0U
#define COMP_CSR_EN_Msk (0x1UL << COMP_CSR_EN_Pos)
#define COMP_CSR_EN COMP_CSR_EN_Msk

/* ========================================================================= */
/* ============                      OPAMP                      ============ */
/* ========================================================================= */

typedef struct
{
	__IOM uint32_t CSR; /*!< OPAMP Control/Status Register. */
	__IOM uint32_t OTR; /*!< OPAMP Offset Trimming Register in normal mode. */
	__IOM uint32_t LPOTR; /*!< OPAMP Offset Trimming Register in Low Power mode. */
}STM32L4xx_OPAMP_TypeDef;

/*CSR register.*/
#define OPAMP_CSR_OPA_RANGE_Pos 31U
#define OPAMP_CSR_OPA_RANGE_Msk (0x1UL << OPAMP_CSR_OPA_RANGE_Pos)
#define OPAMP_CSR_OPA_RANGE OPAMP_CSR_OPA_RANGE_Msk

#define OPAMP_CSR_CALOUT_Pos 15U
#define OPAMP_CSR_CALOUT_Msk (0x1UL << OPAMP_CSR_CALOUT_Pos)
#define OPAMP_CSR_CALOUT OPAMP_CSR_CALOUT_Msk

#define OPAMP_CSR_USERTRIM_Pos 14U
#define OPAMP_CSR_USERTRIM_Msk (0x1UL << OPAMP_CSR_USERTRIM_Pos)
#define OPAMP_CSR_USERTRIM OPAMP_CSR_USERTRIM_Msk

#define OPAMP_CSR_CALSEL_Pos 13U
#define OPAMP_CSR_CALSEL_Msk (0x1UL << OPAMP_CSR_CALSEL_Pos)
#define OPAMP_CSR_CALSEL OPAMP_CSR_CALSEL_Msk

#define OPAMP_CSR_CALON_Pos 12U
#define OPAMP_CSR_CALON_Msk (0x1UL << OPAMP_CSR_CALON_Pos)
#define OPAMP_CSR_CALON OPAMP_CSR_CALON_Msk

#define OPAMP_CSR_VP_SEL_Pos 10U
#define OPAMP_CSR_VP_SEL_Msk (0x1UL << OPAMP_CSR_VP_SEL_Pos)
#define OPAMP_CSR_VP_SEL OPAMP_CSR_VP_SEL_Msk

#define OPAMP_CSR_VM_SEL_Pos 8U
#define OPAMP_CSR_VM_SEL_Msk (0x3UL << OPAMP_CSR_VM_SEL_Pos)
#define OPAMP_CSR_VM_SEL OPAMP_CSR_VM_SEL_Msk

#define OPAMP_CSR_PGA_GAIN_Pos 4U
#define OPAMP_CSR_PGA_GAIN_Msk (0x3UL << OPAMP_CSR_PGA_GAIN_Pos)
#define OPAMP_CSR_PGA_GAIN OPAMP_CSR_PGA_GAIN_Msk
#define OPAMP_CSR_PGA_GAIN_2 (0x0UL << OPAMP_CSR_PGA_GAIN_Pos)
#define OPAMP_CSR_PGA_GAIN_4 (0x1UL << OPAMP_CSR_PGA_GAIN_Pos)
#define OPAMP_CSR_PGA_GAIN_8 (0x2UL << OPAMP_CSR_PGA_GAIN_Pos)
#define OPAMP_CSR_PGA_GAIN_16 (0x3UL << OPAMP_CSR_PGA_GAIN_Pos)

#define OPAMP_CSR_OPAMODE_Pos 2U
#define OPAMP_CSR_OPAMODE_Msk (0x3UL << OPAMP_CSR_OPAMODE_Pos)
#define OPAMP_CSR_OPAMODE OPAMP_CSR_OPAMODE_Msk
#define OPAMP_CSR_OPAMODE_INTERNAL_PGA_DISABLE_0 (0x0UL << OPAMP_CSR_OPAMODE_Pos)
#define OPAMP_CSR_OPAMODE_INTERNAL_PGA_DISABLE_1 (0x1UL << OPAMP_CSR_OPAMODE_Pos)
#define OPAMP_CSR_OPAMODE_INTERNAL_PGA_ENABLE (0x2UL << OPAMP_CSR_OPAMODE_Pos)
#define OPAMP_CSR_OPAMODE_INTERNAL_FOLLOWER (0x3UL << OPAMP_CSR_OPAMODE_Pos)

#define OPAMP_CSR_OPALPM_Pos 1U
#define OPAMP_CSR_OPALPM_Msk (0x1UL << OPAMP_CSR_OPALPM_Pos)
#define OPAMP_CSR_OPALPM OPAMP_CSR_OPALPM_Msk

#define OPAMP_CSR_OPAEN_Pos 1U
#define OPAMP_CSR_OPAEN_Msk (0x1UL << OPAMP_CSR_OPAEN_Pos)
#define OPAMP_CSR_OPAEN OPAMP_CSR_OPAEN_Msk

/*OTR register.*/
#define OPAMP_OTR_TRIMOFFSETP_Pos 8U
#define OPAMP_OTR_TRIMOFFSETP_Msk (0x1FUL << OPAMP_OTR_TRIMOFFSETP_Pos)
#define OPAMP_OTR_TRIMOFFSETP OPAMP_OTR_TRIMOFFSETP_Msk

#define OPAMP_OTR_TRIMOFFSETN_Pos 0U
#define OPAMP_OTR_TRIMOFFSETN_Msk (0x1FUL << OPAMP_OTR_TRIMOFFSETN_Pos)
#define OPAMP_OTR_TRIMOFFSETN OPAMP_OTR_TRIMOFFSETN_Msk

/*LPOTR register.*/
#define OPAMP_LPOTR_TRIMOFFSETP_Pos 8U
#define OPAMP_LPOTR_TRIMOFFSETP_Msk (0x1FUL << OPAMP_LPOTR_TRIMOFFSETP_Pos)
#define OPAMP_LPOTR_TRIMOFFSETP OPAMP_LPOTR_TRIMOFFSETP_Msk

#define OPAMP_LPOTR_TRIMOFFSETN_Pos 0U
#define OPAMP_LPOTR_TRIMOFFSETN_Msk (0x1FUL << OPAMP_LPOTR_TRIMOFFSETN_Pos)
#define OPAMP_LPOTR_TRIMOFFSETN OPAMP_LPOTR_TRIMOFFSETN_Msk

#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
/* ========================================================================= */
/* ============                      DFSDM                      ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CFGR1; /*!< DFSDM Configuration Register 1. */
	__IOM uint32_t CFGR2; /*!< DFSDM Configuration Register 2. */
	__IOM uint32_t AWSCDR; /*!< DFSDM Analog Watchdog and Short-Circuit Detector Register. */
	__IM uint32_t WDATR; /*!< DFSDM Watchdog filter Data Register. */
	__IOM uint32_t DATINR; /*!< DFSDM Data Input Register. */
}STM32L4xx_DFSDM_CH_TypeDef;

typedef struct
{
	__IOM uint32_t CR1; /*!< DFSDM filter Control Register 1. */
	__IOM uint32_t CR2; /*!< DFSDM filter Control Register 2. */
	__IM uint32_t ISR; /*!< DFSDM filter Interrupt and Status Register. */
	__IOM uint32_t ICR; /*!< DFSDM filter Interrupt flag Clear Register. */
	__IOM uint32_t JCHGR; /*!< DFSDM filter Injected Channel Group selection Register. */
	__IOM uint32_t FCR; /*!< DFSDM Filter Control Register. */
	__IM uint32_t JDATAR; /*!< DFSDM filter Data Register for Injected group. */
	__IM uint32_t RDATAR; /*!< DFSDM filter Data Register for the Regular channel. */
	__IOM uint32_t AWHTR; /*!< DFSDM filter Analog Watchdog High Threshold Register. */
	__IOM uint32_t AWLTR; /*!< DFSDM filter Analog Watchdog Low Threshold Register. */
	__IM uint32_t AWSR; /*!< DFSDM filter Analog Watchdog Status Register. */
	__IOM uint32_t AWCFR; /*!< DFSDM filter Analog Watchdog Clear Flag Register. */
	__IM uint32_t EXMAX; /*!< DFSDM filter Extremes detector Maximum Register. */
	__IM uint32_t EXMIN; /*!< DFSDM filter Extremes detector Minimum Register. */
	__IM uint32_t CNVTIMR; /*!< DFSDM filter Conversion Time Register. */
}STM32L4xx_DFSDM_FLT_TypeDef;

/*CHyCFGR1 register.*/
#define DFSDM_CH_CFGR1_DFSDMEN_Pos 31U
#define DFSDM_CH_CFGR1_DFSDMEN_Msk (0x1UL << DFSDM_CH_CFGR1_DFSDMEN_Pos)
#define DFSDM_CH_CFGR1_DFSDMEN DFSDM_CH_CFGR1_DFSDMEN_Msk

#define DFSDM_CH_CFGR1_CKOUTSRC_Pos 30U
#define DFSDM_CH_CFGR1_CKOUTSRC_Msk (0x1UL << DFSDM_CH_CFGR1_CKOUTSRC_Pos)
#define DFSDM_CH_CFGR1_CKOUTSRC DFSDM_CH_CFGR1_CKOUTSRC_Msk

#define DFSDM_CH_CFGR1_CKOUTDIV_Pos 16U
#define DFSDM_CH_CFGR1_CKOUTDIV_Msk (0xFFUL << DFSDM_CH_CFGR1_CKOUTDIV_Pos)
#define DFSDM_CH_CFGR1_CKOUTDIV DFSDM_CH_CFGR1_CKOUTDIV_Msk

#define DFSDM_CH_CFGR1_DATPACK_Pos 14U
#define DFSDM_CH_CFGR1_DATPACK_Msk (0x3UL << DFSDM_CH_CFGR1_DATPACK_Pos)
#define DFSDM_CH_CFGR1_DATPACK DFSDM_CH_CFGR1_DATPACK_Msk
#define DFSDM_CH_CFGR1_DATPACK_STANDARD (0x0UL << DFSDM_CH_CFGR1_DATPACK_Pos)
#define DFSDM_CH_CFGR1_DATPACK_INTERLEAVED (0x1UL << DFSDM_CH_CFGR1_DATPACK_Pos)
#define DFSDM_CH_CFGR1_DATPACK_DUAL (0x2UL << DFSDM_CH_CFGR1_DATPACK_Pos)

#define DFSDM_CH_CFGR1_DATMPX_Pos 12U
#define DFSDM_CH_CFGR1_DATMPX_Msk (0x3UL << DFSDM_CH_CFGR1_DATMPX_Pos)
#define DFSDM_CH_CFGR1_DATMPX DFSDM_CH_CFGR1_DATMPX_Msk
#define DFSDM_CH_CFGR1_DATMPX_EXTERNAL (0x0UL << DFSDM_CH_CFGR1_DATMPX_Pos)
#define DFSDM_CH_CFGR1_DATMPX_INTERNAL (0x1UL << DFSDM_CH_CFGR1_DATMPX_Pos)

#define DFSDM_CH_CFGR1_CHINSEL_Pos 8U
#define DFSDM_CH_CFGR1_CHINSEL_Msk (0x1UL << DFSDM_CH_CFGR1_CHINSEL_Pos)
#define DFSDM_CH_CFGR1_CHINSEL DFSDM_CH_CFGR1_CHINSEL_Msk

#define DFSDM_CH_CFGR1_CHEN_Pos 7U
#define DFSDM_CH_CFGR1_CHEN_Msk (0x1UL << DFSDM_CH_CFGR1_CHEN_Pos)
#define DFSDM_CH_CFGR1_CHEN DFSDM_CH_CFGR1_CHEN_Msk

#define DFSDM_CH_CFGR1_CKABEN_Pos 6U
#define DFSDM_CH_CFGR1_CKABEN_Msk (0x1UL << DFSDM_CH_CFGR1_CKABEN_Pos)
#define DFSDM_CH_CFGR1_CKABEN DFSDM_CH_CFGR1_CKABEN_Msk

#define DFSDM_CH_CFGR1_SCDEN_Pos 5U
#define DFSDM_CH_CFGR1_SCDEN_Msk (0x1UL << DFSDM_CH_CFGR1_SCDEN_Pos)
#define DFSDM_CH_CFGR1_SCDEN DFSDM_CH_CFGR1_SCDEN_Msk

#define DFSDM_CH_CFGR1_SPICKSEL_Pos 2U
#define DFSDM_CH_CFGR1_SPICKSEL_Msk (0x3UL << DFSDM_CH_CFGR1_SPICKSEL_Pos)
#define DFSDM_CH_CFGR1_SPICKSEL DFSDM_CH_CFGR1_SPICKSEL_Msk

#define DFSDM_CH_CFGR1_SITP_Pos 0U
#define DFSDM_CH_CFGR1_SITP_Msk (0x3UL << DFSDM_CH_CFGR1_SITP_Pos)
#define DFSDM_CH_CFGR1_SITP DFSDM_CH_CFGR1_SITP_Msk
#define DFSDM_CH_CFGR1_SITP_SPI_RISING_EDGE (0x0UL << DFSDM_CH_CFGR1_SITP_Pos)
#define DFSDM_CH_CFGR1_SITP_SPI_FALLING_EDGE (0x1UL << DFSDM_CH_CFGR1_SITP_Pos)
#define DFSDM_CH_CFGR1_SITP_MANCHESTER_FALLING_EDGE (0x2UL << DFSDM_CH_CFGR1_SITP_Pos)
#define DFSDM_CH_CFGR1_SITP_MANCHESTER_RISING_EDGE (0x3UL << DFSDM_CH_CFGR1_SITP_Pos)

/*CHyCFGR2 register.*/
#define DFSDM_CH_CFGR2_OFFSET_Pos 8U
#define DFSDM_CH_CFGR2_OFFSET_Msk (0xFFFFFFUL << DFSDM_CH_CFGR2_OFFSET_Pos)
#define DFSDM_CH_CFGR2_OFFSET DFSDM_CH_CFGR2_OFFSET_Msk

#define DFSDM_CH_CFGR2_DTRBS_Pos 3U
#define DFSDM_CH_CFGR2_DTRBS_Msk (0x1FUL << DFSDM_CH_CFGR2_DTRBS_Pos)
#define DFSDM_CH_CFGR2_DTRBS DFSDM_CH_CFGR2_DTRBS_Msk

/*CHyAWSCDR register.*/
#define DFSDM_CH_AWSCDR_AWFORD_Pos 22U
#define DFSDM_CH_AWSCDR_AWFORD_Msk (0x3UL << DFSDM_CH_AWSCDR_AWFORD_Pos)
#define DFSDM_CH_AWSCDR_AWFORD DFSDM_CH_AWSCDR_AWFORD_Msk
#define DFSDM_CH_AWSCDR_AWFORD_FASTSINC (0x0UL << DFSDM_CH_AWSCDR_AWFORD_Pos)
#define DFSDM_CH_AWSCDR_AWFORD_SINC_1 (0x1UL << DFSDM_CH_AWSCDR_AWFORD_Pos)
#define DFSDM_CH_AWSCDR_AWFORD_SINC_2 (0x2UL << DFSDM_CH_AWSCDR_AWFORD_Pos)
#define DFSDM_CH_AWSCDR_AWFORD_SINC_3 (0x3UL << DFSDM_CH_AWSCDR_AWFORD_Pos)

#define DFSDM_CH_AWSCDR_AWFOSR_Pos 16U
#define DFSDM_CH_AWSCDR_AWFOSR_Msk (0x1FUL << DFSDM_CH_AWSCDR_AWFOSR_Pos)
#define DFSDM_CH_AWSCDR_AWFOSR DFSDM_CH_AWSCDR_AWFOSR_Msk

#define DFSDM_CH_AWSCDR_BKSCD_Pos 12U
#define DFSDM_CH_AWSCDR_BKSCD_Msk (0xFUL << DFSDM_CH_AWSCDR_BKSCD_Pos)
#define DFSDM_CH_AWSCDR_BKSCD DFSDM_CH_AWSCDR_BKSCD_Msk

#define DFSDM_CH_AWSCDR_SCDT_Pos 0U
#define DFSDM_CH_AWSCDR_SCDT_Msk (0xFFUL << DFSDM_CH_AWSCDR_SCDT_Pos)
#define DFSDM_CH_AWSCDR_SCDT DFSDM_CH_AWSCDR_SCDT_Msk

/*CHyWDATR register.*/
#define DFSDM_CH_WDATR_WDATA_Pos 0U
#define DFSDM_CH_WDATR_WDATA_Msk (0xFFFFUL << DFSDM_CH_WDATR_WDATA_Pos)
#define DFSDM_CH_WDATR_WDATA DFSDM_CH_WDATR_WDATA_Msk

/*CHyDATINR register.*/
#define DFSDM_CH_DATINR_INDAT1_Pos 16U
#define DFSDM_CH_DATINR_INDAT1_Msk (0xFFFFUL << DFSDM_CH_DATINR_INDAT1_Pos)
#define DFSDM_CH_DATINR_INDAT1 DFSDM_CH_DATINR_INDAT1_Msk

#define DFSDM_CH_DATINR_INDAT0_Pos 0U
#define DFSDM_CH_DATINR_INDAT0_Msk (0xFFFFUL << DFSDM_CH_DATINR_INDAT0_Pos)
#define DFSDM_CH_DATINR_INDAT0 DFSDM_CH_DATINR_INDAT0_Msk

/*FLTxCR1 register.*/
#define DFSDM_FLT_CR1_AWFSEL_Pos 30U
#define DFSDM_FLT_CR1_AWFSEL_Msk (0x1UL << DFSDM_FLT_CR1_AWFSEL_Pos)
#define DFSDM_FLT_CR1_AWFSEL DFSDM_FLT_CR1_AWFSEL_Msk

#define DFSDM_FLT_CR1_FAST_Pos 29U
#define DFSDM_FLT_CR1_FAST_Msk (0x1UL << DFSDM_FLT_CR1_FAST_Pos)
#define DFSDM_FLT_CR1_FAST DFSDM_FLT_CR1_FAST_Msk

#define DFSDM_FLT_CR1_RCH_Pos 24U
#define DFSDM_FLT_CR1_RCH_Msk (0x3UL << DFSDM_FLT_CR1_RCH_Pos)
#define DFSDM_FLT_CR1_RCH DFSDM_FLT_CR1_RCH_Msk

#define DFSDM_FLT_CR1_RDMAEN_Pos 21U
#define DFSDM_FLT_CR1_RDMAEN_Msk (0x1UL << DFSDM_FLT_CR1_RDMAEN_Pos)
#define DFSDM_FLT_CR1_RDMAEN DFSDM_FLT_CR1_RDMAEN_Msk

#define DFSDM_FLT_CR1_RSYNC_Pos 19U
#define DFSDM_FLT_CR1_RSYNC_Msk (0x1UL << DFSDM_FLT_CR1_RSYNC_Pos)
#define DFSDM_FLT_CR1_RSYNC DFSDM_FLT_CR1_RSYNC_Msk

#define DFSDM_FLT_CR1_RCONT_Pos 18U
#define DFSDM_FLT_CR1_RCONT_Msk (0x1UL << DFSDM_FLT_CR1_RCONT_Pos)
#define DFSDM_FLT_CR1_RCONT DFSDM_FLT_CR1_RCONT_Msk

#define DFSDM_FLT_CR1_RSWSTART_Pos 17U
#define DFSDM_FLT_CR1_RSWSTART_Msk (0x1UL << DFSDM_FLT_CR1_RSWSTART_Pos)
#define DFSDM_FLT_CR1_RSWSTART DFSDM_FLT_CR1_RSWSTART_Msk

#define DFSDM_FLT_CR1_JEXTEN_Pos 13U
#define DFSDM_FLT_CR1_JEXTEN_Msk (0x3UL << DFSDM_FLT_CR1_JEXTEN_Pos)
#define DFSDM_FLT_CR1_JEXTEN DFSDM_FLT_CR1_JEXTEN_Msk
#define DFSDM_FLT_CR1_JEXTEN_DISABLED (0x0UL << DFSDM_FLT_CR1_JEXTEN_Pos)
#define DFSDM_FLT_CR1_JEXTEN_RISING_EDGE (0x1UL << DFSDM_FLT_CR1_JEXTEN_Pos)
#define DFSDM_FLT_CR1_JEXTEN_FALLING_EDGE (0x2UL << DFSDM_FLT_CR1_JEXTEN_Pos)
#define DFSDM_FLT_CR1_JEXTEN_RISING_FALLING_EDGE (0x3UL << DFSDM_FLT_CR1_JEXTEN_Pos)

#define DFSDM_FLT_CR1_JEXTSEL_Pos 8U
#define DFSDM_FLT_CR1_JEXTSEL_Msk (0x7UL << DFSDM_FLT_CR1_JEXTSEL_Pos)
#define DFSDM_FLT_CR1_JEXTSEL DFSDM_FLT_CR1_JEXTSEL_Msk

#define DFSDM_FLT_CR1_JDMAEN_Pos 5U
#define DFSDM_FLT_CR1_JDMAEN_Msk (0x1UL << DFSDM_FLT_CR1_JDMAEN_Pos)
#define DFSDM_FLT_CR1_JDMAEN DFSDM_FLT_CR1_JDMAEN_Msk

#define DFSDM_FLT_CR1_JSCAN_Pos 4U
#define DFSDM_FLT_CR1_JSCAN_Msk (0x1UL << DFSDM_FLT_CR1_JSCAN_Pos)
#define DFSDM_FLT_CR1_JSCAN DFSDM_FLT_CR1_JSCAN_Msk

#define DFSDM_FLT_CR1_JSYNC_Pos 3U
#define DFSDM_FLT_CR1_JSYNC_Msk (0x1UL << DFSDM_FLT_CR1_JSYNC_Pos)
#define DFSDM_FLT_CR1_JSYNC DFSDM_FLT_CR1_JSYNC_Msk

#define DFSDM_FLT_CR1_JSWSTART_Pos 1U
#define DFSDM_FLT_CR1_JSWSTART_Msk (0x1UL << DFSDM_FLT_CR1_JSWSTART_Pos)
#define DFSDM_FLT_CR1_JSWSTART DFSDM_FLT_CR1_JSWSTART_Msk

#define DFSDM_FLT_CR1_DFEN_Pos 0U
#define DFSDM_FLT_CR1_DFEN_Msk (0x1UL << DFSDM_FLT_CR1_DFEN_Pos)
#define DFSDM_FLT_CR1_DFEN DFSDM_FLT_CR1_DFEN_Msk

/*FLTxCR2 register.*/
#define DFSDM_FLT_CR2_AWDCH_Pos 16U
#define DFSDM_FLT_CR2_AWDCH_Msk (0xFUL << DFSDM_FLT_CR2_AWDCH_Pos)
#define DFSDM_FLT_CR2_AWDCH DFSDM_FLT_CR2_AWDCH_Msk

#define DFSDM_FLT_CR2_EXCH_Pos 8U
#define DFSDM_FLT_CR2_EXCH_Msk (0xFUL << DFSDM_FLT_CR2_EXCH_Pos)
#define DFSDM_FLT_CR2_EXCH DFSDM_FLT_CR2_EXCH_Msk

#define DFSDM_FLT_CR2_CKABIE_Pos 6U
#define DFSDM_FLT_CR2_CKABIE_Msk (0x1UL << DFSDM_FLT_CR2_CKABIE_Pos)
#define DFSDM_FLT_CR2_CKABIE DFSDM_FLT_CR2_CKABIE_Msk

#define DFSDM_FLT_CR2_SCDIE_Pos 5U
#define DFSDM_FLT_CR2_SCDIE_Msk (0x1UL << DFSDM_FLT_CR2_SCDIE_Pos)
#define DFSDM_FLT_CR2_SCDIE DFSDM_FLT_CR2_SCDIE_Msk

#define DFSDM_FLT_CR2_AWDIE_Pos 4U
#define DFSDM_FLT_CR2_AWDIE_Msk (0x1UL << DFSDM_FLT_CR2_AWDIE_Pos)
#define DFSDM_FLT_CR2_AWDIE DFSDM_FLT_CR2_AWDIE_Msk

#define DFSDM_FLT_CR2_ROVRIE_Pos 3U
#define DFSDM_FLT_CR2_ROVRIE_Msk (0x1UL << DFSDM_FLT_CR2_ROVRIE_Pos)
#define DFSDM_FLT_CR2_ROVRIE DFSDM_FLT_CR2_ROVRIE_Msk

#define DFSDM_FLT_CR2_JOVRIE_Pos 2U
#define DFSDM_FLT_CR2_JOVRIE_Msk (0x1UL << DFSDM_FLT_CR2_JOVRIE_Pos)
#define DFSDM_FLT_CR2_JOVRIE DFSDM_FLT_CR2_JOVRIE_Msk

#define DFSDM_FLT_CR2_REOCIE_Pos 1U
#define DFSDM_FLT_CR2_REOCIE_Msk (0x1UL << DFSDM_FLT_CR2_REOCIE_Pos)
#define DFSDM_FLT_CR2_REOCIE DFSDM_FLT_CR2_REOCIE_Msk

#define DFSDM_FLT_CR2_JEOCIE_Pos 0U
#define DFSDM_FLT_CR2_JEOCIE_Msk (0x1UL << DFSDM_FLT_CR2_JEOCIE_Pos)
#define DFSDM_FLT_CR2_JEOCIE DFSDM_FLT_CR2_JEOCIE_Msk

/*FLTxISR register.*/
#define DFSDM_FLT_ISR_SCDF_Pos 24U
#define DFSDM_FLT_ISR_SCDF_Msk (0xFUL << DFSDM_FLT_ISR_SCDF_Pos)
#define DFSDM_FLT_ISR_SCDF DFSDM_FLT_ISR_SCDF_Msk

#define DFSDM_FLT_ISR_CKABF_Pos 16U
#define DFSDM_FLT_ISR_CKABF_Msk (0xFUL << DFSDM_FLT_ISR_CKABF_Pos)
#define DFSDM_FLT_ISR_CKABF DFSDM_FLT_ISR_CKABF_Msk

#define DFSDM_FLT_ISR_RCIP_Pos 14U
#define DFSDM_FLT_ISR_RCIP_Msk (0x1UL << DFSDM_FLT_ISR_RCIP_Pos)
#define DFSDM_FLT_ISR_RCIP DFSDM_FLT_ISR_RCIP_Msk

#define DFSDM_FLT_ISR_JCIP_Pos 13U
#define DFSDM_FLT_ISR_JCIP_Msk (0x1UL << DFSDM_FLT_ISR_JCIP_Pos)
#define DFSDM_FLT_ISR_JCIP DFSDM_FLT_ISR_JCIP_Msk

#define DFSDM_FLT_ISR_AWDF_Pos 4U
#define DFSDM_FLT_ISR_AWDF_Msk (0x1UL << DFSDM_FLT_ISR_AWDF_Pos)
#define DFSDM_FLT_ISR_AWDF DFSDM_FLT_ISR_AWDF_Msk

#define DFSDM_FLT_ISR_ROVRF_Pos 3U
#define DFSDM_FLT_ISR_ROVRF_Msk (0x1UL << DFSDM_FLT_ISR_ROVRF_Pos)
#define DFSDM_FLT_ISR_ROVRF DFSDM_FLT_ISR_ROVRF_Msk

#define DFSDM_FLT_ISR_JOVRF_Pos 2U
#define DFSDM_FLT_ISR_JOVRF_Msk (0x1UL << DFSDM_FLT_ISR_JOVRF_Pos)
#define DFSDM_FLT_ISR_JOVRF DFSDM_FLT_ISR_JOVRF_Msk

#define DFSDM_FLT_ISR_REOCF_Pos 1U
#define DFSDM_FLT_ISR_REOCF_Msk (0x1UL << DFSDM_FLT_ISR_REOCF_Pos)
#define DFSDM_FLT_ISR_REOCF DFSDM_FLT_ISR_REOCF_Msk

#define DFSDM_FLT_ISR_JEOCF_Pos 0U
#define DFSDM_FLT_ISR_JEOCF_Msk (0x1UL << DFSDM_FLT_ISR_JEOCF_Pos)
#define DFSDM_FLT_ISR_JEOCF DFSDM_FLT_ISR_JEOCF_Msk

/*FLTxICR register.*/
#define DFSDM_FLT_ICR_CLRSCDF_Pos 24U
#define DFSDM_FLT_ICR_CLRSCDF_Msk (0xFUL << DFSDM_FLT_ICR_CLRSCDF_Pos)
#define DFSDM_FLT_ICR_CLRSCDF DFSDM_FLT_ICR_CLRSCDF_Msk

#define DFSDM_FLT_ICR_CLRCKABF_Pos 16U
#define DFSDM_FLT_ICR_CLRCKABF_Msk (0xFUL << DFSDM_FLT_ICR_CLRCKABF_Pos)
#define DFSDM_FLT_ICR_CLRCKABF DFSDM_FLT_ICR_CLRCKABF_Msk

#define DFSDM_FLT_ICR_CLRROVRF_Pos 3U
#define DFSDM_FLT_ICR_CLRROVRF_Msk (0x1UL << DFSDM_FLT_ICR_CLRROVRF_Pos)
#define DFSDM_FLT_ICR_CLRROVRF DFSDM_FLT_ICR_CLRROVRF_Msk

#define DFSDM_FLT_ICR_CLRJOVRF_Pos 2U
#define DFSDM_FLT_ICR_CLRJOVRF_Msk (0x1UL << DFSDM_FLT_ICR_CLRJOVRF_Pos)
#define DFSDM_FLT_ICR_CLRJOVRF DFSDM_FLT_ICR_CLRJOVRF_Msk

/*FLTxJCHGR register.*/
#define DFSDM_FLT_JCHGR_JCHG_Pos 0U
#define DFSDM_FLT_JCHGR_JCHG_Msk (0xFUL << DFSDM_FLT_JCHGR_JCHG_Pos)
#define DFSDM_FLT_JCHGR_JCHG DFSDM_FLT_JCHGR_JCHG_Msk

/*FLTxFCR register.*/
#define DFSDM_FLT_FCR_FORD_Pos 29U
#define DFSDM_FLT_FCR_FORD_Msk (0x7UL << DFSDM_FLT_FCR_FORD_Pos)
#define DFSDM_FLT_FCR_FORD DFSDM_FLT_FCR_FORD_Msk

#define DFSDM_FLT_FCR_FOSR_Pos 16U
#define DFSDM_FLT_FCR_FOSR_Msk (0x3FFUL << DFSDM_FLT_FCR_FOSR_Pos)
#define DFSDM_FLT_FCR_FOSR DFSDM_FLT_FCR_FOSR_Msk

#define DFSDM_FLT_FCR_IOSR_Pos 0U
#define DFSDM_FLT_FCR_IOSR_Msk (0xFFUL << DFSDM_FLT_FCR_IOSR_Pos)
#define DFSDM_FLT_FCR_IOSR DFSDM_FLT_FCR_IOSR_Msk

/*FLTxJDATAR register.*/
#define DFSDM_FLT_JDATAR_JDATA_Pos 8U
#define DFSDM_FLT_JDATAR_JDATA_Msk (0xFFFFFFUL << DFSDM_FLT_JDATAR_JDATA_Pos)
#define DFSDM_FLT_JDATAR_JDATA DFSDM_FLT_JDATAR_JDATA_Msk

#define DFSDM_FLT_JDATAR_JDATACH_Pos 0U
#define DFSDM_FLT_JDATAR_JDATACH_Msk (0x3UL << DFSDM_FLT_JDATAR_JDATACH_Pos)
#define DFSDM_FLT_JDATAR_JDATACH DFSDM_FLT_JDATAR_JDATACH_Msk

/*FLTxRDATAR register.*/
#define DFSDM_FLT_RDATAR_RDATA_Pos 8U
#define DFSDM_FLT_RDATAR_RDATA_Msk (0xFFFFFFUL << DFSDM_FLT_RDATAR_RDATA_Pos)
#define DFSDM_FLT_RDATAR_RDATA DFSDM_FLT_RDATAR_RDATA_Msk

#define DFSDM_FLT_RDATAR_RPEND_Pos 4U
#define DFSDM_FLT_RDATAR_RPEND_Msk (0x1UL << DFSDM_FLT_RDATAR_RPEND_Pos)
#define DFSDM_FLT_RDATAR_RPEND DFSDM_FLT_RDATAR_RPEND_Msk

#define DFSDM_FLT_RDATAR_RDATACH_Pos 0U
#define DFSDM_FLT_RDATAR_RDATACH_Msk (0x1UL << DFSDM_FLT_RDATAR_RDATACH_Pos)
#define DFSDM_FLT_RDATAR_RDATACH DFSDM_FLT_RDATAR_RDATACH_Msk

/*FLTxAWHTR register.*/
#define DFSDM_FLT_AWHTR_AWHT_Pos 8U
#define DFSDM_FLT_AWHTR_AWHT_Msk (0xFFFFFFUL << DFSDM_FLT_AWHTR_AWHT_Pos)
#define DFSDM_FLT_AWHTR_AWHT DFSDM_FLT_AWHTR_AWHT_Msk

#define DFSDM_FLT_AWHTR_BKAWH_Pos 0U
#define DFSDM_FLT_AWHTR_BKAWH_Msk (0xFUL << DFSDM_FLT_AWHTR_BKAWH_Pos)
#define DFSDM_FLT_AWHTR_BKAWH DFSDM_FLT_AWHTR_BKAWH_Msk

/*FLTxAWLTR register.*/
#define DFSDM_FLT_AWLTR_AWLT_Pos 8U
#define DFSDM_FLT_AWLTR_AWLT_Msk (0xFFFFFFUL << DFSDM_FLT_AWLTR_AWLT_Pos)
#define DFSDM_FLT_AWLTR_AWLT DFSDM_FLT_AWLTR_AWLT_Msk

#define DFSDM_FLT_AWLTR_BKAWL_Pos 0U
#define DFSDM_FLT_AWLTR_BKAWL_Msk (0xFUL << DFSDM_FLT_AWLTR_BKAWL_Pos)
#define DFSDM_FLT_AWLTR_BKAWL DFSDM_FLT_AWLTR_BKAWL_Msk

/*FLTxAWSR register.*/
#define DFSDM_FLT_AWSR_AWHTF_Pos 8U
#define DFSDM_FLT_AWSR_AWHTF_Msk (0xFUL << DFSDM_FLT_AWSR_AWHTF_Pos)
#define DFSDM_FLT_AWSR_AWHTF DFSDM_FLT_AWSR_AWHTF_Msk

#define DFSDM_FLT_AWSR_AWLTF_Pos 0U
#define DFSDM_FLT_AWSR_AWLTF_Msk (0xFUL << DFSDM_FLT_AWSR_AWLTF_Pos)
#define DFSDM_FLT_AWSR_AWLTF DFSDM_FLT_AWSR_AWLTF_Msk

/*FLTxAWCFR register.*/
#define DFSDM_FLT_AWCFR_CLRAWHTF_Pos 8U
#define DFSDM_FLT_AWCFR_CLRAWHTF_Msk (0xFUL << DFSDM_FLT_AWCFR_CLRAWHTF_Pos)
#define DFSDM_FLT_AWCFR_CLRAWHTF DFSDM_FLT_AWCFR_CLRAWHTF_Msk

#define DFSDM_FLT_AWCFR_CLRAWLTF_Pos 0U
#define DFSDM_FLT_AWCFR_CLRAWLTF_Msk (0xFUL << DFSDM_FLT_AWCFR_CLRAWLTF_Pos)
#define DFSDM_FLT_AWCFR_CLRAWLTF DFSDM_FLT_AWCFR_CLRAWLTF_Msk

/*FLTxEXMAX register.*/
#define DFSDM_FLT_EXMAX_EXMAX_Pos 8U
#define DFSDM_FLT_EXMAX_EXMAX_Msk (0xFFFFFFUL << DFSDM_FLT_EXMAX_EXMAX_Pos)
#define DFSDM_FLT_EXMAX_EXMAX DFSDM_FLT_EXMAX_EXMAX_Msk

#define DFSDM_FLT_EXMAX_EXMAXCH_Pos 0U
#define DFSDM_FLT_EXMAX_EXMAXCH_Msk (0x3UL << DFSDM_FLT_EXMAX_EXMAXCH_Pos)
#define DFSDM_FLT_EXMAX_EXMAXCH DFSDM_FLT_EXMAX_EXMAXCH_Msk

/*FLTxEXMIN register.*/
#define DFSDM_FLT_EXMIN_EXMIN_Pos 8U
#define DFSDM_FLT_EXMIN_EXMIN_Msk (0xFFFFFFUL << DFSDM_FLT_EXMIN_EXMIN_Pos)
#define DFSDM_FLT_EXMIN_EXMIN DFSDM_FLT_EXMIN_EXMIN_Msk

#define DFSDM_FLT_EXMIN_EXMINCH_Pos 0U
#define DFSDM_FLT_EXMIN_EXMINCH_Msk (0x3UL << DFSDM_FLT_EXMIN_EXMINCH_Pos)
#define DFSDM_FLT_EXMIN_EXMINCH DFSDM_FLT_EXMIN_EXMINCH_Msk

/*FLTxCNVTIMR register.*/
#define DFSDM_FLT_CNVTIMR_CNVCNT_Pos 4U
#define DFSDM_FLT_CNVTIMR_CNVCNT_Msk (0xFFFFFFFFFFFFFFFUL << DFSDM_FLT_CNVTIMR_CNVCNT_Pos)
#define DFSDM_FLT_CNVTIMR_CNVCNT DFSDM_FLT_CNVTIMR_CNVCNT_Msk
#endif

#if defined(STM32L433) || defined(STM32L443)
/* ========================================================================= */
/* ============                       LCD                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR; /*!< LCD Control Register. */
	__IOM uint32_t FCR; /*!< LCD Frame Control Register. */
	__IOM uint32_t SR; /*!< LCD Status Register. */
	__OM uint32_t CLR; /*!< LCD Clear Register. */
	uint32_t RESERVED;
	__IOM uint32_t RAM[16]; /*!< LCD Display Memory. */
}STM32L4xx_LCD_TypeDef;

/*CR register.*/
#define LCD_CR_BUFEN_Pos 8U
#define LCD_CR_BUFEN_Msk (0x1UL << LCD_CR_BUFEN_Pos)
#define LCD_CR_BUFEN LCD_CR_BUFEN_Msk

#define LCD_CR_MUX_SEG_Pos 7U
#define LCD_CR_MUX_SEG_Msk (0x1UL << LCD_CR_MUX_SEG_Pos)
#define LCD_CR_MUX_SEG LCD_CR_MUX_SEG_Msk

#define LCD_CR_BIAS_Pos 5U
#define LCD_CR_BIAS_Msk (0x3UL << LCD_CR_BIAS_Pos)
#define LCD_CR_BIAS LCD_CR_BIAS_Msk
#define LCD_CR_BIAS_1_DIV_4 (0x0UL << LCD_CR_BIAS_Pos)
#define LCD_CR_BIAS_1_DIV_2 (0x1UL << LCD_CR_BIAS_Pos)
#define LCD_CR_BIAS_1_DIV_3 (0x2UL << LCD_CR_BIAS_Pos)

#define LCD_CR_DUTY_Pos 2U
#define LCD_CR_DUTY_Msk (0x7UL << LCD_CR_DUTY_Pos)
#define LCD_CR_DUTY LCD_CR_DUTY_Msk
#define LCD_CR_DUTY_STATIC (0x0UL << LCD_CR_DUTY_Pos)
#define LCD_CR_DUTY_1_DIV_2 (0x1UL << LCD_CR_DUTY_Pos)
#define LCD_CR_DUTY_1_DIV_3 (0x2UL << LCD_CR_DUTY_Pos)
#define LCD_CR_DUTY_1_DIV_4 (0x3UL << LCD_CR_DUTY_Pos)
#define LCD_CR_DUTY_1_DIV_8 (0x4UL << LCD_CR_DUTY_Pos)

#define LCD_CR_VSEL_Pos 1U
#define LCD_CR_VSEL_Msk (0x1UL << LCD_CR_VSEL_Pos)
#define LCD_CR_VSEL LCD_CR_VSEL_Msk

#define LCD_CR_LCDEN_Pos 0U
#define LCD_CR_LCDEN_Msk (0x1UL << LCD_CR_LCDEN_Pos)
#define LCD_CR_LCDEN LCD_CR_LCDEN_Msk

/*FCR register.*/
#define LCD_FCR_PS_Pos 22U
#define LCD_FCR_PS_Msk (0xFUL << LCD_FCR_PS_Pos)
#define LCD_FCR_PS LCD_FCR_PS_Msk

#define LCD_FCR_DIV_Pos 18U
#define LCD_FCR_DIV_Msk (0xFUL << LCD_FCR_DIV_Pos)
#define LCD_FCR_DIV LCD_FCR_DIV_Msk

#define LCD_FCR_BLINK_Pos 16U
#define LCD_FCR_BLINK_Msk (0x3UL << LCD_FCR_BLINK_Pos)
#define LCD_FCR_BLINK LCD_FCR_BLINK_Msk
#define LCD_FCR_BLINK_DISABLED (0x0UL << LCD_FCR_BLINK_Pos)
#define LCD_FCR_BLINK_ENABLED_SEG0_COM0 (0x1UL << LCD_FCR_BLINK_Pos)
#define LCD_FCR_BLINK_ENABLED_SEG0_ALL_COMS (0x2UL << LCD_FCR_BLINK_Pos)
#define LCD_FCR_BLINK_ENABLED_ALL_SEGS_ALL_COMS (0x3UL << LCD_FCR_BLINK_Pos)

#define LCD_FCR_BLINKF_Pos 13U
#define LCD_FCR_BLINKF_Msk (0x7UL << LCD_FCR_BLINKF_Pos)
#define LCD_FCR_BLINKF LCD_FCR_BLINKF_Msk
#define LCD_FCR_BLINKF_DIV_8 (0x0UL << LCD_FCR_BLINKF_Pos)
#define LCD_FCR_BLINKF_DIV_16 (0x1UL << LCD_FCR_BLINKF_Pos)
#define LCD_FCR_BLINKF_DIV_32 (0x2UL << LCD_FCR_BLINKF_Pos)
#define LCD_FCR_BLINKF_DIV_64 (0x3UL << LCD_FCR_BLINKF_Pos)
#define LCD_FCR_BLINKF_DIV_128 (0x4UL << LCD_FCR_BLINKF_Pos)
#define LCD_FCR_BLINKF_DIV_256 (0x5UL << LCD_FCR_BLINKF_Pos)
#define LCD_FCR_BLINKF_DIV_512 (0x6UL << LCD_FCR_BLINKF_Pos)
#define LCD_FCR_BLINKF_DIV_1024 (0x7UL << LCD_FCR_BLINKF_Pos)

#define LCD_FCR_CC_Pos 10U
#define LCD_FCR_CC_Msk (0x7UL << LCD_FCR_CC_Pos)
#define LCD_FCR_CC LCD_FCR_CC_Msk
#define LCD_FCR_CC_LCD_0 (0x0UL << LCD_FCR_CC_Pos)
#define LCD_FCR_CC_LCD_1 (0x1UL << LCD_FCR_CC_Pos)
#define LCD_FCR_CC_LCD_2 (0x2UL << LCD_FCR_CC_Pos)
#define LCD_FCR_CC_LCD_3 (0x3UL << LCD_FCR_CC_Pos)
#define LCD_FCR_CC_LCD_4 (0x4UL << LCD_FCR_CC_Pos)
#define LCD_FCR_CC_LCD_5 (0x5UL << LCD_FCR_CC_Pos)
#define LCD_FCR_CC_LCD_6 (0x6UL << LCD_FCR_CC_Pos)
#define LCD_FCR_CC_LCD_7 (0x7UL << LCD_FCR_CC_Pos)

#define LCD_FCR_DEAD_Pos 7U
#define LCD_FCR_DEAD_Msk (0x7UL << LCD_FCR_DEAD_Pos)
#define LCD_FCR_DEAD LCD_FCR_DEAD_Msk

#define LCD_FCR_PON_Pos 4U
#define LCD_FCR_PON_Msk (0x7UL << LCD_FCR_PON_Pos)
#define LCD_FCR_PON LCD_FCR_PON_Msk

#define LCD_FCR_UDDIE_Pos 3U
#define LCD_FCR_UDDIE_Msk (0x1UL << LCD_FCR_UDDIE_Pos)
#define LCD_FCR_UDDIE LCD_FCR_UDDIE_Msk

#define LCD_FCR_SOFIE_Pos 1U
#define LCD_FCR_SOFIE_Msk (0x1UL << LCD_FCR_SOFIE_Pos)
#define LCD_FCR_SOFIE LCD_FCR_SOFIE_Msk

#define LCD_FCR_HD_Pos 0U
#define LCD_FCR_HD_Msk (0x1UL << LCD_FCR_HD_Pos)
#define LCD_FCR_HD LCD_FCR_HD_Msk

/*SR register.*/
#define LCD_SR_FCRSF_Pos 5U
#define LCD_SR_FCRSF_Msk (0x1UL << LCD_SR_FCRSF_Pos)
#define LCD_SR_FCRSF LCD_SR_FCRSF_Msk

#define LCD_SR_RDY_Pos 4U
#define LCD_SR_RDY_Msk (0x1UL << LCD_SR_RDY_Pos)
#define LCD_SR_RDY LCD_SR_RDY_Msk

#define LCD_SR_UDD_Pos 3U
#define LCD_SR_UDD_Msk (0x1UL << LCD_SR_UDD_Pos)
#define LCD_SR_UDD_Msk LCD_SR_UDD

#define LCD_SR_UDR_Pos 2U
#define LCD_SR_UDR_Msk (0x1UL << LCD_SR_UDR_Pos)
#define LCD_SR_UDR LCD_SR_UDR_Msk

#define LCD_SR_SOF_Pos 1U
#define LCD_SR_SOF_Msk (0x1UL << LCD_SR_SOF_Pos)
#define LCD_SR_SOF LCD_SR_SOF_Msk

#define LCD_SR_ENS_Pos 0U
#define LCD_SR_ENS_Msk (0x1UL << LCD_SR_ENS_Pos)
#define LCD_SR_ENS LCD_SR_ENS_Msk

/*CLR register.*/
#define LCD_CLR_UDDC_Pos 3U
#define LCD_CLR_UDDC_Msk (0x1UL << LCD_CLR_UDDC_Pos)
#define LCD_CLR_UDDC LCD_CLR_UDDC_Msk

#define LCD_CLR_SOFC_Pos 1U
#define LCD_CLR_SOFC_Msk (0x1UL << LCD_CLR_SOFC_Pos)
#define LCD_CLR_SOFC LCD_CLR_SOFC_Msk
#endif

/* ========================================================================= */
/* ============                       TSC                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR; /*!< TSC Control Register. */
	__IOM uint32_t IER; /*!< TSC Interrupt Enable Register. */
	__IOM uint32_t ICR; /*!< TSC Interrupt Clear Register. */
	__IM uint32_t ISR; /*!< TSC Interrupt Status Register. */
	__IM uint32_t IOHCR; /*!< TSC I/O Hysteris Control Register. */
	uint32_t RESERVED0;
	__IOM uint32_t IOASCR; /*!< TSC I/O Analog Switch Control Register. */
	uint32_t RESERVED1;
	__IOM uint32_t IOSCR; /*!< TSC I/O Sampling Control Register. */
	uint32_t RESERVED2;
	__IOM uint32_t IOCCR; /*!< TSC I/O Channel Control Register. */
	uint32_t RESERVED3;
	__IOM uint32_t IOGCSR; /*!< TSC I/O Group Control Status Register. */
	__IM uint32_t IOG1CR; /*!< TSC I/O Group 1 Counter Register. */
	__IM uint32_t IOG2CR; /*!< TSC I/O Group 2 Counter Register. */
	__IM uint32_t IOG3CR; /*!< TSC I/O Group 3 Counter Register. */
	__IM uint32_t IOG4CR; /*!< TSC I/O Group 4 Counter Register. */
	__IM uint32_t IOG5CR; /*!< TSC I/O Group 5 Counter Register. */
	__IM uint32_t IOG6CR; /*!< TSC I/O Group 6 Counter Register. */
	__IM uint32_t IOG7CR; /*!< TSC I/O Group 7 Counter Register. */
}STM32L4xx_TSC_TypeDef;

/*CR register.*/
#define TSC_CR_CTPH_Pos 28U
#define TSC_CR_CTPH_Msk (0xFUL << TSC_CR_CTPH_Pos)
#define TSC_CR_CTPH TSC_CR_CTPH_Msk

#define TSC_CR_CTPL_Pos 24U
#define TSC_CR_CTPL_Msk (0xFUL << TSC_CR_CTPL_Pos)
#define TSC_CR_CTPL TSC_CR_CTPL_Msk

#define TSC_CR_SSD_Pos 17U
#define TSC_CR_SSD_Msk (0x7FUL << TSC_CR_SSD_Pos)
#define TSC_CR_SSD TSC_CR_SSD_Msk

#define TSC_CR_SSE_Pos 16U
#define TSC_CR_SSE_Msk (0x1UL << TSC_CR_SSE_Pos)
#define TSC_CR_SSE TSC_CR_SSE_Msk

#define TSC_CR_SSPSC_Pos 15U
#define TSC_CR_SSPSC_Msk (0x1UL << TSC_CR_SSPSC_Pos)
#define TSC_CR_SSPSC TSC_CR_SSPSC_Msk

#define TSC_CR_PGPSC_Pos 12U
#define TSC_CR_PGPSC_Msk (0x7UL << TSC_CR_PGPSC_Pos)
#define TSC_CR_PGPSC TSC_CR_PGPSC_Msk
#define TSC_CR_PGPSC_DIV_1 (0x0UL << TSC_CR_PGPSC_Pos)
#define TSC_CR_PGPSC_DIV_2 (0x1UL << TSC_CR_PGPSC_Pos)
#define TSC_CR_PGPSC_DIV_4 (0x2UL << TSC_CR_PGPSC_Pos)
#define TSC_CR_PGPSC_DIV_8 (0x3UL << TSC_CR_PGPSC_Pos)
#define TSC_CR_PGPSC_DIV_16 (0x4UL << TSC_CR_PGPSC_Pos)
#define TSC_CR_PGPSC_DIV_32 (0x5UL << TSC_CR_PGPSC_Pos)
#define TSC_CR_PGPSC_DIV_64 (0x6UL << TSC_CR_PGPSC_Pos)
#define TSC_CR_PGPSC_DIV_128 (0x7UL << TSC_CR_PGPSC_Pos)

#define TSC_CR_MCV_Pos 5U
#define TSC_CR_MCV_Msk (0x7UL << TSC_CR_MCV_Pos)
#define TSC_CR_MCV TSC_CR_MCV_Msk
#define TSC_CR_MCV_255 (0x0UL << TSC_CR_MCV_Pos)
#define TSC_CR_MCV_511 (0x1UL << TSC_CR_MCV_Pos)
#define TSC_CR_MCV_1023 (0x2UL << TSC_CR_MCV_Pos)
#define TSC_CR_MCV_2047 (0x3UL << TSC_CR_MCV_Pos)
#define TSC_CR_MCV_4095 (0x4UL << TSC_CR_MCV_Pos)
#define TSC_CR_MCV_8191 (0x5UL << TSC_CR_MCV_Pos)
#define TSC_CR_MCV_16383 (0x6UL << TSC_CR_MCV_Pos)

#define TSC_CR_IODEF_Pos 4U
#define TSC_CR_IODEF_Msk (0x1UL << TSC_CR_IODEF_Pos)
#define TSC_CR_IODEF TSC_CR_IODEF_Msk

#define TSC_CR_SYNCPOL_Pos 3U
#define TSC_CR_SYNCPOL_Msk (0x1UL << TSC_CR_SYNCPOL_Pos)
#define TSC_CR_SYNCPOL TSC_CR_SYNCPOL_Msk

#define TSC_CR_AM_Pos 2U
#define TSC_CR_AM_Msk (0x1UL << TSC_CR_AM_Pos)
#define TSC_CR_AM TSC_CR_AM_Msk

#define TSC_CR_START_Pos 1U
#define TSC_CR_START_Msk (0x1UL << TSC_CR_START_Pos)
#define TSC_CR_START TSC_CR_START_Msk

#define TSC_CR_TSCE_Pos 0U
#define TSC_CR_TSCE_Msk (0x1UL << TSC_CR_TSCE_Pos)
#define TSC_CR_TSCE TSC_CR_TSCE_Msk

/*IER register.*/
#define TSC_IER_MCEIE_Pos 1U
#define TSC_IER_MCEIE_Msk (0x1UL << TSC_IER_MCEIE_Pos)
#define TSC_IER_MCEIE TSC_IER_MCEIE_Msk

#define TSC_IER_EOAIE_Pos 0U
#define TSC_IER_EOAIE_Msk (0x1UL << TSC_IER_EOAIE_Pos)
#define TSC_IER_EOAIE TSC_IER_EOAIE_Msk

/*ICR register.*/
#define TSC_ICR_MCEIC_Pos 1U
#define TSC_ICR_MCEIC_Msk (0x1UL << TSC_ICR_MCEIC_Pos)
#define TSC_ICR_MCEIC TSC_ICR_MCEIC_Msk

#define TSC_ICR_EOAIC_Pos 0U
#define TSC_ICR_EOAIC_Msk (0x1UL << TSC_ICR_EOAIC_Pos)
#define TSC_ICR_EOAIC TSC_ICR_EOAIC_Msk

/*ISR register.*/
#define TSC_ISR_MCEF_Pos 1U
#define TSC_ISR_MCEF_Msk (0x1UL << TSC_ISR_MCEF_Pos)
#define TSC_ISR_MCEF TSC_ISR_MCEF_Msk

#define TSC_ISR_EOAF_Pos 0U
#define TSC_ISR_EOAF_Msk (0x1UL << TSC_ISR_EOAF_Pos)
#define TSC_ISR_EOAF TSC_ISR_EOAF_Msk

/*IOHCR register.*/
#define TSC_IOHCR_G7IO4_Pos 27U
#define TSC_IOHCR_G7IO4_Msk (0x1UL << TSC_IOHCR_G7IO4_Pos)
#define TSC_IOHCR_G7IO4 TSC_IOHCR_G7IO4_Msk

#define TSC_IOHCR_G7IO3_Pos 26U
#define TSC_IOHCR_G7IO3_Msk (0x1UL << TSC_IOHCR_G7IO3_Pos)
#define TSC_IOHCR_G7IO3 TSC_IOHCR_G7IO3_Msk

#define TSC_IOHCR_G7IO2_Pos 25U
#define TSC_IOHCR_G7IO2_Msk (0x1UL << TSC_IOHCR_G7IO2_Pos)
#define TSC_IOHCR_G7IO2 TSC_IOHCR_G7IO2_Msk

#define TSC_IOHCR_G7IO1_Pos 24U
#define TSC_IOHCR_G7IO1_Msk (0x1UL << TSC_IOHCR_G7IO1_Pos)
#define TSC_IOHCR_G7IO1 TSC_IOHCR_G7IO1_Msk

#define TSC_IOHCR_G6IO4_Pos 23U
#define TSC_IOHCR_G6IO4_Msk (0x1UL << TSC_IOHCR_G6IO4_Pos)
#define TSC_IOHCR_G6IO4 TSC_IOHCR_G6IO4_Msk

#define TSC_IOHCR_G6IO3_Pos 22U
#define TSC_IOHCR_G6IO3_Msk (0x1UL << TSC_IOHCR_G6IO3_Pos)
#define TSC_IOHCR_G6IO3 TSC_IOHCR_G6IO3_Msk

#define TSC_IOHCR_G6IO2_Pos 21U
#define TSC_IOHCR_G6IO2_Msk (0x1UL << TSC_IOHCR_G6IO2_Pos)
#define TSC_IOHCR_G6IO2 TSC_IOHCR_G6IO2_Msk

#define TSC_IOHCR_G6IO1_Pos 20U
#define TSC_IOHCR_G6IO1_Msk (0x1UL << TSC_IOHCR_G6IO1_Pos)
#define TSC_IOHCR_G6IO1 TSC_IOHCR_G6IO1_Msk

#define TSC_IOHCR_G5IO4_Pos 19U
#define TSC_IOHCR_G5IO4_Msk (0x1UL << TSC_IOHCR_G5IO4_Pos)
#define TSC_IOHCR_G5IO4 TSC_IOHCR_G5IO4_Msk

#define TSC_IOHCR_G5IO3_Pos 18U
#define TSC_IOHCR_G5IO3_Msk (0x1UL << TSC_IOHCR_G5IO3_Pos)
#define TSC_IOHCR_G5IO3 TSC_IOHCR_G5IO3_Msk

#define TSC_IOHCR_G5IO2_Pos 17U
#define TSC_IOHCR_G5IO2_Msk (0x1UL << TSC_IOHCR_G5IO2_Pos)
#define TSC_IOHCR_G5IO2 TSC_IOHCR_G5IO2_Msk

#define TSC_IOHCR_G5IO1_Pos 16U
#define TSC_IOHCR_G5IO1_Msk (0x1UL << TSC_IOHCR_G5IO1_Pos)
#define TSC_IOHCR_G5IO1 TSC_IOHCR_G5IO1_Msk

#define TSC_IOHCR_G4IO4_Pos 15U
#define TSC_IOHCR_G4IO4_Msk (0x1UL << TSC_IOHCR_G4IO4_Pos)
#define TSC_IOHCR_G4IO4 TSC_IOHCR_G4IO4_Msk

#define TSC_IOHCR_G4IO3_Pos 14U
#define TSC_IOHCR_G4IO3_Msk (0x1UL << TSC_IOHCR_G4IO3_Pos)
#define TSC_IOHCR_G4IO3 TSC_IOHCR_G4IO3_Msk

#define TSC_IOHCR_G4IO2_Pos 13U
#define TSC_IOHCR_G4IO2_Msk (0x1UL << TSC_IOHCR_G4IO2_Pos)
#define TSC_IOHCR_G4IO2 TSC_IOHCR_G4IO2_Msk

#define TSC_IOHCR_G4IO1_Pos 12U
#define TSC_IOHCR_G4IO1_Msk (0x1UL << TSC_IOHCR_G4IO1_Pos)
#define TSC_IOHCR_G4IO1 TSC_IOHCR_G4IO1_Msk

#define TSC_IOHCR_G3IO4_Pos 11U
#define TSC_IOHCR_G3IO4_Msk (0x1UL << TSC_IOHCR_G3IO4_Pos)
#define TSC_IOHCR_G3IO4 TSC_IOHCR_G3IO4_Msk

#define TSC_IOHCR_G3IO3_Pos 10U
#define TSC_IOHCR_G3IO3_Msk (0x1UL << TSC_IOHCR_G3IO3_Pos)
#define TSC_IOHCR_G3IO3 TSC_IOHCR_G3IO3_Msk

#define TSC_IOHCR_G3IO2_Pos 9U
#define TSC_IOHCR_G3IO2_Msk (0x1UL << TSC_IOHCR_G3IO2_Pos)
#define TSC_IOHCR_G3IO2 TSC_IOHCR_G3IO2_Msk

#define TSC_IOHCR_G3IO1_Pos 8U
#define TSC_IOHCR_G3IO1_Msk (0x1UL << TSC_IOHCR_G3IO1_Pos)
#define TSC_IOHCR_G3IO1 TSC_IOHCR_G3IO1_Msk

#define TSC_IOHCR_G2IO4_Pos 7U
#define TSC_IOHCR_G2IO4_Msk (0x1UL << TSC_IOHCR_G2IO4_Pos)
#define TSC_IOHCR_G2IO4 TSC_IOHCR_G2IO4_Msk

#define TSC_IOHCR_G2IO3_Pos 6U
#define TSC_IOHCR_G2IO3_Msk (0x1UL << TSC_IOHCR_G2IO3_Pos)
#define TSC_IOHCR_G2IO3 TSC_IOHCR_G2IO3_Msk

#define TSC_IOHCR_G2IO2_Pos 5U
#define TSC_IOHCR_G2IO2_Msk (0x1UL << TSC_IOHCR_G2IO2_Pos)
#define TSC_IOHCR_G2IO2 TSC_IOHCR_G2IO2_Msk

#define TSC_IOHCR_G2IO1_Pos 4U
#define TSC_IOHCR_G2IO1_Msk (0x1UL << TSC_IOHCR_G2IO1_Pos)
#define TSC_IOHCR_G2IO1 TSC_IOHCR_G2IO1_Msk

#define TSC_IOHCR_G1IO4_Pos 3U
#define TSC_IOHCR_G1IO4_Msk (0x1UL << TSC_IOHCR_G1IO4_Pos)
#define TSC_IOHCR_G1IO4 TSC_IOHCR_G1IO4_Msk

#define TSC_IOHCR_G1IO3_Pos 2U
#define TSC_IOHCR_G1IO3_Msk (0x1UL << TSC_IOHCR_G1IO3_Pos)
#define TSC_IOHCR_G1IO3 TSC_IOHCR_G1IO3_Msk

#define TSC_IOHCR_G1IO2_Pos 1U
#define TSC_IOHCR_G1IO2_Msk (0x1UL << TSC_IOHCR_G1IO2_Pos)
#define TSC_IOHCR_G1IO2 TSC_IOHCR_G1IO2_Msk

#define TSC_IOHCR_G1IO1_Pos 0U
#define TSC_IOHCR_G1IO1_Msk (0x1UL << TSC_IOHCR_G1IO1_Pos)
#define TSC_IOHCR_G1IO1 TSC_IOHCR_G1IO1_Msk

/*IOASCR register.*/
#define TSC_IOASCR_G7IO4_Pos 27U
#define TSC_IOASCR_G7IO4_Msk (0x1UL << TSC_IOASCR_G7IO4_Pos)
#define TSC_IOASCR_G7IO4 TSC_IOASCR_G7IO4_Msk

#define TSC_IOASCR_G7IO3_Pos 26U
#define TSC_IOASCR_G7IO3_Msk (0x1UL << TSC_IOASCR_G7IO3_Pos)
#define TSC_IOASCR_G7IO3 TSC_IOASCR_G7IO3_Msk

#define TSC_IOASCR_G7IO2_Pos 25U
#define TSC_IOASCR_G7IO2_Msk (0x1UL << TSC_IOASCR_G7IO2_Pos)
#define TSC_IOASCR_G7IO2 TSC_IOASCR_G7IO2_Msk

#define TSC_IOASCR_G7IO1_Pos 24U
#define TSC_IOASCR_G7IO1_Msk (0x1UL << TSC_IOASCR_G7IO1_Pos)
#define TSC_IOASCR_G7IO1 TSC_IOASCR_G7IO1_Msk

#define TSC_IOASCR_G6IO4_Pos 23U
#define TSC_IOASCR_G6IO4_Msk (0x1UL << TSC_IOASCR_G6IO4_Pos)
#define TSC_IOASCR_G6IO4 TSC_IOASCR_G6IO4_Msk

#define TSC_IOASCR_G6IO3_Pos 22U
#define TSC_IOASCR_G6IO3_Msk (0x1UL << TSC_IOASCR_G6IO3_Pos)
#define TSC_IOASCR_G6IO3 TSC_IOASCR_G6IO3_Msk

#define TSC_IOASCR_G6IO2_Pos 21U
#define TSC_IOASCR_G6IO2_Msk (0x1UL << TSC_IOASCR_G6IO2_Pos)
#define TSC_IOASCR_G6IO2 TSC_IOASCR_G6IO2_Msk

#define TSC_IOASCR_G6IO1_Pos 20U
#define TSC_IOASCR_G6IO1_Msk (0x1UL << TSC_IOASCR_G6IO1_Pos)
#define TSC_IOASCR_G6IO1 TSC_IOASCR_G6IO1_Msk

#define TSC_IOASCR_G5IO4_Pos 19U
#define TSC_IOASCR_G5IO4_Msk (0x1UL << TSC_IOASCR_G5IO4_Pos)
#define TSC_IOASCR_G5IO4 TSC_IOASCR_G5IO4_Msk

#define TSC_IOASCR_G5IO3_Pos 18U
#define TSC_IOASCR_G5IO3_Msk (0x1UL << TSC_IOASCR_G5IO3_Pos)
#define TSC_IOASCR_G5IO3 TSC_IOASCR_G5IO3_Msk

#define TSC_IOASCR_G5IO2_Pos 17U
#define TSC_IOASCR_G5IO2_Msk (0x1UL << TSC_IOASCR_G5IO2_Pos)
#define TSC_IOASCR_G5IO2 TSC_IOASCR_G5IO2_Msk

#define TSC_IOASCR_G5IO1_Pos 16U
#define TSC_IOASCR_G5IO1_Msk (0x1UL << TSC_IOASCR_G5IO1_Pos)
#define TSC_IOASCR_G5IO1 TSC_IOASCR_G5IO1_Msk

#define TSC_IOASCR_G4IO4_Pos 15U
#define TSC_IOASCR_G4IO4_Msk (0x1UL << TSC_IOASCR_G4IO4_Pos)
#define TSC_IOASCR_G4IO4 TSC_IOASCR_G4IO4_Msk

#define TSC_IOASCR_G4IO3_Pos 14U
#define TSC_IOASCR_G4IO3_Msk (0x1UL << TSC_IOASCR_G4IO3_Pos)
#define TSC_IOASCR_G4IO3 TSC_IOASCR_G4IO3_Msk

#define TSC_IOASCR_G4IO2_Pos 13U
#define TSC_IOASCR_G4IO2_Msk (0x1UL << TSC_IOASCR_G4IO2_Pos)
#define TSC_IOASCR_G4IO2 TSC_IOASCR_G4IO2_Msk

#define TSC_IOASCR_G4IO1_Pos 12U
#define TSC_IOASCR_G4IO1_Msk (0x1UL << TSC_IOASCR_G4IO1_Pos)
#define TSC_IOASCR_G4IO1 TSC_IOASCR_G4IO1_Msk

#define TSC_IOASCR_G3IO4_Pos 11U
#define TSC_IOASCR_G3IO4_Msk (0x1UL << TSC_IOASCR_G3IO4_Pos)
#define TSC_IOASCR_G3IO4 TSC_IOASCR_G3IO4_Msk

#define TSC_IOASCR_G3IO3_Pos 10U
#define TSC_IOASCR_G3IO3_Msk (0x1UL << TSC_IOASCR_G3IO3_Pos)
#define TSC_IOASCR_G3IO3 TSC_IOASCR_G3IO3_Msk

#define TSC_IOASCR_G3IO2_Pos 9U
#define TSC_IOASCR_G3IO2_Msk (0x1UL << TSC_IOASCR_G3IO2_Pos)
#define TSC_IOASCR_G3IO2 TSC_IOASCR_G3IO2_Msk

#define TSC_IOASCR_G3IO1_Pos 8U
#define TSC_IOASCR_G3IO1_Msk (0x1UL << TSC_IOASCR_G3IO1_Pos)
#define TSC_IOASCR_G3IO1 TSC_IOASCR_G3IO1_Msk

#define TSC_IOASCR_G2IO4_Pos 7U
#define TSC_IOASCR_G2IO4_Msk (0x1UL << TSC_IOASCR_G2IO4_Pos)
#define TSC_IOASCR_G2IO4 TSC_IOASCR_G2IO4_Msk

#define TSC_IOASCR_G2IO3_Pos 6U
#define TSC_IOASCR_G2IO3_Msk (0x1UL << TSC_IOASCR_G2IO3_Pos)
#define TSC_IOASCR_G2IO3 TSC_IOASCR_G2IO3_Msk

#define TSC_IOASCR_G2IO2_Pos 5U
#define TSC_IOASCR_G2IO2_Msk (0x1UL << TSC_IOASCR_G2IO2_Pos)
#define TSC_IOASCR_G2IO2 TSC_IOASCR_G2IO2_Msk

#define TSC_IOASCR_G2IO1_Pos 4U
#define TSC_IOASCR_G2IO1_Msk (0x1UL << TSC_IOASCR_G2IO1_Pos)
#define TSC_IOASCR_G2IO1 TSC_IOASCR_G2IO1_Msk

#define TSC_IOASCR_G1IO4_Pos 3U
#define TSC_IOASCR_G1IO4_Msk (0x1UL << TSC_IOASCR_G1IO4_Pos)
#define TSC_IOASCR_G1IO4 TSC_IOASCR_G1IO4_Msk

#define TSC_IOASCR_G1IO3_Pos 2U
#define TSC_IOASCR_G1IO3_Msk (0x1UL << TSC_IOASCR_G1IO3_Pos)
#define TSC_IOASCR_G1IO3 TSC_IOASCR_G1IO3_Msk

#define TSC_IOASCR_G1IO2_Pos 1U
#define TSC_IOASCR_G1IO2_Msk (0x1UL << TSC_IOASCR_G1IO2_Pos)
#define TSC_IOASCR_G1IO2 TSC_IOASCR_G1IO2_Msk

#define TSC_IOASCR_G1IO1_Pos 0U
#define TSC_IOASCR_G1IO1_Msk (0x1UL << TSC_IOASCR_G1IO1_Pos)
#define TSC_IOASCR_G1IO1 TSC_IOASCR_G1IO1_Msk

/*IOSCR register.*/
#define TSC_IOSCR_G7IO4_Pos 27U
#define TSC_IOSCR_G7IO4_Msk (0x1UL << TSC_IOSCR_G7IO4_Pos)
#define TSC_IOSCR_G7IO4 TSC_IOSCR_G7IO4_Msk

#define TSC_IOSCR_G7IO3_Pos 26U
#define TSC_IOSCR_G7IO3_Msk (0x1UL << TSC_IOSCR_G7IO3_Pos)
#define TSC_IOSCR_G7IO3 TSC_IOSCR_G7IO3_Msk

#define TSC_IOSCR_G7IO2_Pos 25U
#define TSC_IOSCR_G7IO2_Msk (0x1UL << TSC_IOSCR_G7IO2_Pos)
#define TSC_IOSCR_G7IO2 TSC_IOSCR_G7IO2_Msk

#define TSC_IOSCR_G7IO1_Pos 24U
#define TSC_IOSCR_G7IO1_Msk (0x1UL << TSC_IOSCR_G7IO1_Pos)
#define TSC_IOSCR_G7IO1 TSC_IOSCR_G7IO1_Msk

#define TSC_IOSCR_G6IO4_Pos 23U
#define TSC_IOSCR_G6IO4_Msk (0x1UL << TSC_IOSCR_G6IO4_Pos)
#define TSC_IOSCR_G6IO4 TSC_IOSCR_G6IO4_Msk

#define TSC_IOSCR_G6IO3_Pos 22U
#define TSC_IOSCR_G6IO3_Msk (0x1UL << TSC_IOSCR_G6IO3_Pos)
#define TSC_IOSCR_G6IO3 TSC_IOSCR_G6IO3_Msk

#define TSC_IOSCR_G6IO2_Pos 21U
#define TSC_IOSCR_G6IO2_Msk (0x1UL << TSC_IOSCR_G6IO2_Pos)
#define TSC_IOSCR_G6IO2 TSC_IOSCR_G6IO2_Msk

#define TSC_IOSCR_G6IO1_Pos 20U
#define TSC_IOSCR_G6IO1_Msk (0x1UL << TSC_IOSCR_G6IO1_Pos)
#define TSC_IOSCR_G6IO1 TSC_IOSCR_G6IO1_Msk

#define TSC_IOSCR_G5IO4_Pos 19U
#define TSC_IOSCR_G5IO4_Msk (0x1UL << TSC_IOSCR_G5IO4_Pos)
#define TSC_IOSCR_G5IO4 TSC_IOSCR_G5IO4_Msk

#define TSC_IOSCR_G5IO3_Pos 18U
#define TSC_IOSCR_G5IO3_Msk (0x1UL << TSC_IOSCR_G5IO3_Pos)
#define TSC_IOSCR_G5IO3 TSC_IOSCR_G5IO3_Msk

#define TSC_IOSCR_G5IO2_Pos 17U
#define TSC_IOSCR_G5IO2_Msk (0x1UL << TSC_IOSCR_G5IO2_Pos)
#define TSC_IOSCR_G5IO2 TSC_IOSCR_G5IO2_Msk

#define TSC_IOSCR_G5IO1_Pos 16U
#define TSC_IOSCR_G5IO1_Msk (0x1UL << TSC_IOSCR_G5IO1_Pos)
#define TSC_IOSCR_G5IO1 TSC_IOSCR_G5IO1_Msk

#define TSC_IOSCR_G4IO4_Pos 15U
#define TSC_IOSCR_G4IO4_Msk (0x1UL << TSC_IOSCR_G4IO4_Pos)
#define TSC_IOSCR_G4IO4 TSC_IOSCR_G4IO4_Msk

#define TSC_IOSCR_G4IO3_Pos 14U
#define TSC_IOSCR_G4IO3_Msk (0x1UL << TSC_IOSCR_G4IO3_Pos)
#define TSC_IOSCR_G4IO3 TSC_IOSCR_G4IO3_Msk

#define TSC_IOSCR_G4IO2_Pos 13U
#define TSC_IOSCR_G4IO2_Msk (0x1UL << TSC_IOSCR_G4IO2_Pos)
#define TSC_IOSCR_G4IO2 TSC_IOSCR_G4IO2_Msk

#define TSC_IOSCR_G4IO1_Pos 12U
#define TSC_IOSCR_G4IO1_Msk (0x1UL << TSC_IOSCR_G4IO1_Pos)
#define TSC_IOSCR_G4IO1 TSC_IOSCR_G4IO1_Msk

#define TSC_IOSCR_G3IO4_Pos 11U
#define TSC_IOSCR_G3IO4_Msk (0x1UL << TSC_IOSCR_G3IO4_Pos)
#define TSC_IOSCR_G3IO4 TSC_IOSCR_G3IO4_Msk

#define TSC_IOSCR_G3IO3_Pos 10U
#define TSC_IOSCR_G3IO3_Msk (0x1UL << TSC_IOSCR_G3IO3_Pos)
#define TSC_IOSCR_G3IO3 TSC_IOSCR_G3IO3_Msk

#define TSC_IOSCR_G3IO2_Pos 9U
#define TSC_IOSCR_G3IO2_Msk (0x1UL << TSC_IOSCR_G3IO2_Pos)
#define TSC_IOSCR_G3IO2 TSC_IOSCR_G3IO2_Msk

#define TSC_IOSCR_G3IO1_Pos 8U
#define TSC_IOSCR_G3IO1_Msk (0x1UL << TSC_IOSCR_G3IO1_Pos)
#define TSC_IOSCR_G3IO1 TSC_IOSCR_G3IO1_Msk

#define TSC_IOSCR_G2IO4_Pos 7U
#define TSC_IOSCR_G2IO4_Msk (0x1UL << TSC_IOSCR_G2IO4_Pos)
#define TSC_IOSCR_G2IO4 TSC_IOSCR_G2IO4_Msk

#define TSC_IOSCR_G2IO3_Pos 6U
#define TSC_IOSCR_G2IO3_Msk (0x1UL << TSC_IOSCR_G2IO3_Pos)
#define TSC_IOSCR_G2IO3 TSC_IOSCR_G2IO3_Msk

#define TSC_IOSCR_G2IO2_Pos 5U
#define TSC_IOSCR_G2IO2_Msk (0x1UL << TSC_IOSCR_G2IO2_Pos)
#define TSC_IOSCR_G2IO2 TSC_IOSCR_G2IO2_Msk

#define TSC_IOSCR_G2IO1_Pos 4U
#define TSC_IOSCR_G2IO1_Msk (0x1UL << TSC_IOSCR_G2IO1_Pos)
#define TSC_IOSCR_G2IO1 TSC_IOSCR_G2IO1_Msk

#define TSC_IOSCR_G1IO4_Pos 3U
#define TSC_IOSCR_G1IO4_Msk (0x1UL << TSC_IOSCR_G1IO4_Pos)
#define TSC_IOSCR_G1IO4 TSC_IOSCR_G1IO4_Msk

#define TSC_IOSCR_G1IO3_Pos 2U
#define TSC_IOSCR_G1IO3_Msk (0x1UL << TSC_IOSCR_G1IO3_Pos)
#define TSC_IOSCR_G1IO3 TSC_IOSCR_G1IO3_Msk

#define TSC_IOSCR_G1IO2_Pos 1U
#define TSC_IOSCR_G1IO2_Msk (0x1UL << TSC_IOSCR_G1IO2_Pos)
#define TSC_IOSCR_G1IO2 TSC_IOSCR_G1IO2_Msk

#define TSC_IOSCR_G1IO1_Pos 0U
#define TSC_IOSCR_G1IO1_Msk (0x1UL << TSC_IOSCR_G1IO1_Pos)
#define TSC_IOSCR_G1IO1 TSC_IOSCR_G1IO1_Msk

/*IOCCR register.*/
#define TSC_IOCCR_G7IO4_Pos 27U
#define TSC_IOCCR_G7IO4_Msk (0x1UL << TSC_IOCCR_G7IO4_Pos)
#define TSC_IOCCR_G7IO4 TSC_IOCCR_G7IO4_Msk

#define TSC_IOCCR_G7IO3_Pos 26U
#define TSC_IOCCR_G7IO3_Msk (0x1UL << TSC_IOCCR_G7IO3_Pos)
#define TSC_IOCCR_G7IO3 TSC_IOCCR_G7IO3_Msk

#define TSC_IOCCR_G7IO2_Pos 25U
#define TSC_IOCCR_G7IO2_Msk (0x1UL << TSC_IOCCR_G7IO2_Pos)
#define TSC_IOCCR_G7IO2 TSC_IOCCR_G7IO2_Msk

#define TSC_IOCCR_G7IO1_Pos 24U
#define TSC_IOCCR_G7IO1_Msk (0x1UL << TSC_IOCCR_G7IO1_Pos)
#define TSC_IOCCR_G7IO1 TSC_IOCCR_G7IO1_Msk

#define TSC_IOCCR_G6IO4_Pos 23U
#define TSC_IOCCR_G6IO4_Msk (0x1UL << TSC_IOCCR_G6IO4_Pos)
#define TSC_IOCCR_G6IO4 TSC_IOCCR_G6IO4_Msk

#define TSC_IOCCR_G6IO3_Pos 22U
#define TSC_IOCCR_G6IO3_Msk (0x1UL << TSC_IOCCR_G6IO3_Pos)
#define TSC_IOCCR_G6IO3 TSC_IOCCR_G6IO3_Msk

#define TSC_IOCCR_G6IO2_Pos 21U
#define TSC_IOCCR_G6IO2_Msk (0x1UL << TSC_IOCCR_G6IO2_Pos)
#define TSC_IOCCR_G6IO2 TSC_IOCCR_G6IO2_Msk

#define TSC_IOCCR_G6IO1_Pos 20U
#define TSC_IOCCR_G6IO1_Msk (0x1UL << TSC_IOCCR_G6IO1_Pos)
#define TSC_IOCCR_G6IO1 TSC_IOCCR_G6IO1_Msk

#define TSC_IOCCR_G5IO4_Pos 19U
#define TSC_IOCCR_G5IO4_Msk (0x1UL << TSC_IOCCR_G5IO4_Pos)
#define TSC_IOCCR_G5IO4 TSC_IOCCR_G5IO4_Msk

#define TSC_IOCCR_G5IO3_Pos 18U
#define TSC_IOCCR_G5IO3_Msk (0x1UL << TSC_IOCCR_G5IO3_Pos)
#define TSC_IOCCR_G5IO3 TSC_IOCCR_G5IO3_Msk

#define TSC_IOCCR_G5IO2_Pos 17U
#define TSC_IOCCR_G5IO2_Msk (0x1UL << TSC_IOCCR_G5IO2_Pos)
#define TSC_IOCCR_G5IO2 TSC_IOCCR_G5IO2_Msk

#define TSC_IOCCR_G5IO1_Pos 16U
#define TSC_IOCCR_G5IO1_Msk (0x1UL << TSC_IOCCR_G5IO1_Pos)
#define TSC_IOCCR_G5IO1 TSC_IOCCR_G5IO1_Msk

#define TSC_IOCCR_G4IO4_Pos 15U
#define TSC_IOCCR_G4IO4_Msk (0x1UL << TSC_IOCCR_G4IO4_Pos)
#define TSC_IOCCR_G4IO4 TSC_IOCCR_G4IO4_Msk

#define TSC_IOCCR_G4IO3_Pos 14U
#define TSC_IOCCR_G4IO3_Msk (0x1UL << TSC_IOCCR_G4IO3_Pos)
#define TSC_IOCCR_G4IO3 TSC_IOCCR_G4IO3_Msk

#define TSC_IOCCR_G4IO2_Pos 13U
#define TSC_IOCCR_G4IO2_Msk (0x1UL << TSC_IOCCR_G4IO2_Pos)
#define TSC_IOCCR_G4IO2 TSC_IOCCR_G4IO2_Msk

#define TSC_IOCCR_G4IO1_Pos 12U
#define TSC_IOCCR_G4IO1_Msk (0x1UL << TSC_IOCCR_G4IO1_Pos)
#define TSC_IOCCR_G4IO1 TSC_IOCCR_G4IO1_Msk

#define TSC_IOCCR_G3IO4_Pos 11U
#define TSC_IOCCR_G3IO4_Msk (0x1UL << TSC_IOCCR_G3IO4_Pos)
#define TSC_IOCCR_G3IO4 TSC_IOCCR_G3IO4_Msk

#define TSC_IOCCR_G3IO3_Pos 10U
#define TSC_IOCCR_G3IO3_Msk (0x1UL << TSC_IOCCR_G3IO3_Pos)
#define TSC_IOCCR_G3IO3 TSC_IOCCR_G3IO3_Msk

#define TSC_IOCCR_G3IO2_Pos 9U
#define TSC_IOCCR_G3IO2_Msk (0x1UL << TSC_IOCCR_G3IO2_Pos)
#define TSC_IOCCR_G3IO2 TSC_IOCCR_G3IO2_Msk

#define TSC_IOCCR_G3IO1_Pos 8U
#define TSC_IOCCR_G3IO1_Msk (0x1UL << TSC_IOCCR_G3IO1_Pos)
#define TSC_IOCCR_G3IO1 TSC_IOCCR_G3IO1_Msk

#define TSC_IOCCR_G2IO4_Pos 7U
#define TSC_IOCCR_G2IO4_Msk (0x1UL << TSC_IOCCR_G2IO4_Pos)
#define TSC_IOCCR_G2IO4 TSC_IOCCR_G2IO4_Msk

#define TSC_IOCCR_G2IO3_Pos 6U
#define TSC_IOCCR_G2IO3_Msk (0x1UL << TSC_IOCCR_G2IO3_Pos)
#define TSC_IOCCR_G2IO3 TSC_IOCCR_G2IO3_Msk

#define TSC_IOCCR_G2IO2_Pos 5U
#define TSC_IOCCR_G2IO2_Msk (0x1UL << TSC_IOCCR_G2IO2_Pos)
#define TSC_IOCCR_G2IO2 TSC_IOCCR_G2IO2_Msk

#define TSC_IOCCR_G2IO1_Pos 4U
#define TSC_IOCCR_G2IO1_Msk (0x1UL << TSC_IOCCR_G2IO1_Pos)
#define TSC_IOCCR_G2IO1 TSC_IOCCR_G2IO1_Msk

#define TSC_IOCCR_G1IO4_Pos 3U
#define TSC_IOCCR_G1IO4_Msk (0x1UL << TSC_IOCCR_G1IO4_Pos)
#define TSC_IOCCR_G1IO4 TSC_IOCCR_G1IO4_Msk

#define TSC_IOCCR_G1IO3_Pos 2U
#define TSC_IOCCR_G1IO3_Msk (0x1UL << TSC_IOCCR_G1IO3_Pos)
#define TSC_IOCCR_G1IO3 TSC_IOCCR_G1IO3_Msk

#define TSC_IOCCR_G1IO2_Pos 1U
#define TSC_IOCCR_G1IO2_Msk (0x1UL << TSC_IOCCR_G1IO2_Pos)
#define TSC_IOCCR_G1IO2 TSC_IOCCR_G1IO2_Msk

#define TSC_IOCCR_G1IO1_Pos 0U
#define TSC_IOCCR_G1IO1_Msk (0x1UL << TSC_IOCCR_G1IO1_Pos)
#define TSC_IOCCR_G1IO1 TSC_IOCCR_G1IO1_Msk

/*IOGCSR register.*/
#define TSC_IOGCSR_G7S_Pos 22U
#define TSC_IOGCSR_G7S_Msk (0x1UL << TSC_IOGCSR_G7S_Pos)
#define TSC_IOGCSR_G7S TSC_IOGCSR_G7S_Msk

#define TSC_IOGCSR_G6S_Pos 21U
#define TSC_IOGCSR_G6S_Msk (0x1UL << TSC_IOGCSR_G6S_Pos)
#define TSC_IOGCSR_G6S TSC_IOGCSR_G6S_Msk

#define TSC_IOGCSR_G5S_Pos 20U
#define TSC_IOGCSR_G5S_Msk (0x1UL << TSC_IOGCSR_G5S_Pos)
#define TSC_IOGCSR_G5S TSC_IOGCSR_G5S_Msk

#define TSC_IOGCSR_G4S_Pos 19U
#define TSC_IOGCSR_G4S_Msk (0x1UL << TSC_IOGCSR_G4S_Pos)
#define TSC_IOGCSR_G4S TSC_IOGCSR_G4S_Msk

#define TSC_IOGCSR_G3S_Pos 18U
#define TSC_IOGCSR_G3S_Msk (0x1UL << TSC_IOGCSR_G3S_Pos)
#define TSC_IOGCSR_G3S TSC_IOGCSR_G3S_Msk

#define TSC_IOGCSR_G2S_Pos 17U
#define TSC_IOGCSR_G2S_Msk (0x1UL << TSC_IOGCSR_G2S_Pos)
#define TSC_IOGCSR_G2S TSC_IOGCSR_G2S_Msk

#define TSC_IOGCSR_G1S_Pos 16U
#define TSC_IOGCSR_G1S_Msk (0x1UL << TSC_IOGCSR_G1S_Pos)
#define TSC_IOGCSR_G1S TSC_IOGCSR_G1S_Msk

#define TSC_IOGCSR_G7E_Pos 6U
#define TSC_IOGCSR_G7E_Msk (0x1UL << TSC_IOGCSR_G7E_Pos)
#define TSC_IOGCSR_G7E TSC_IOGCSR_G7E_Msk

#define TSC_IOGCSR_G6E_Pos 5U
#define TSC_IOGCSR_G6E_Msk (0x1UL << TSC_IOGCSR_G6E_Pos)
#define TSC_IOGCSR_G6E TSC_IOGCSR_G6E_Msk

#define TSC_IOGCSR_G5E_Pos 4U
#define TSC_IOGCSR_G5E_Msk (0x1UL << TSC_IOGCSR_G5E_Pos)
#define TSC_IOGCSR_G5E TSC_IOGCSR_G5E_Msk

#define TSC_IOGCSR_G4E_Pos 3U
#define TSC_IOGCSR_G4E_Msk (0x1UL << TSC_IOGCSR_G4E_Pos)
#define TSC_IOGCSR_G4E TSC_IOGCSR_G4E_Msk

#define TSC_IOGCSR_G3E_Pos 2U
#define TSC_IOGCSR_G3E_Msk (0x1UL << TSC_IOGCSR_G3E_Pos)
#define TSC_IOGCSR_G3E TSC_IOGCSR_G3E_Msk

#define TSC_IOGCSR_G2E_Pos 1U
#define TSC_IOGCSR_G2E_Msk (0x1UL << TSC_IOGCSR_G2E_Pos)
#define TSC_IOGCSR_G2E TSC_IOGCSR_G2E_Msk

#define TSC_IOGCSR_G1E_Pos 0U
#define TSC_IOGCSR_G1E_Msk (0x1UL << TSC_IOGCSR_G1E_Pos)
#define TSC_IOGCSR_G1E TSC_IOGCSR_G1E_Msk

/*IOGCR register.*/
#define TSC_IOGCR_CNT_Pos 0U
#define TSC_IOGCR_CNT_Msk (0x3FFFUL << TSC_IOGCR_CNT_Pos)
#define TSC_IOGCR_CNT TSC_IOGCR_CNT_Msk

/* ========================================================================= */
/* ============                       RNG                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR; /*!< RNG Control Register. */
	__IOM uint32_t SR; /*!< RNG Status Register. */
	__IM uint32_t DR; /*!< RNG Data Register. */
}STM32L4xx_RNG_TypeDef;

/*CR register.*/
#define RNG_CR_CED_Pos 5U
#define RNG_CR_CED_Msk (0x1UL << RNG_CR_CED_Pos)
#define RNG_CR_CED RNG_CR_CED_Msk

#define RNG_CR_IE_Pos 3U
#define RNG_CR_IE_Msk (0x1UL << RNG_CR_IE_Pos)
#define RNG_CR_IE RNG_CR_IE_Msk

#define RNG_CR_RNGEN_Pos 2U
#define RNG_CR_RNGEN_Msk (0x1UL << RNG_CR_RNGEN_Pos)
#define RNG_CR_RNGEN RNG_CR_RNGEN_Msk

/*SR register.*/
#define RNG_SR_SEIS_Pos 6U
#define RNG_SR_SEIS_Msk (0x1UL << RNG_SR_SEIS_Pos)
#define RNG_SR_SEIS RNG_SR_SEIS_Msk

#define RNG_SR_CEIS_Pos 5U
#define RNG_SR_CEIS_Msk (0x1UL << RNG_SR_CEIS_Pos)
#define RNG_SR_CEIS RNG_SR_CEIS_Msk

#define RNG_SR_SECS_Pos 2U
#define RNG_SR_SECS_Msk (0x1UL << RNG_SR_SECS_Pos)
#define RNG_SR_SECS RNG_SR_SECS_Msk

#define RNG_SR_CECS_Pos 1U
#define RNG_SR_CECS_Msk (0x1UL << RNG_SR_CECS_Pos)
#define RNG_SR_CECS RNG_SR_CECS_Msk

#define RNG_SR_DRDY_Pos 0U
#define RNG_SR_DRDY_Msk (0x1UL << RNG_SR_DRDY_Pos)
#define RNG_SR_DRDY RNG_SR_DRDY_Msk

#if defined(STM32L422) || defined(STM32L442) || defined(STM32L443) || defined(STM32L462)
/* ========================================================================= */
/* ============                       AES                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR; /*!< AES Control Register. */
	__IM uint32_t SR; /*!< AES Status Register. */
	__IOM uint32_t DINR; /*!< AES Data Input Register. */
	__IM uint32_t DOUTR; /*!< AES Data Output Register. */
	__IOM uint32_t KEYR0; /*!< AES Key Register 0. */
	__IOM uint32_t KEYR1; /*!< AES Key Register 1. */
	__IOM uint32_t KEYR2; /*!< AES Key Register 2. */
	__IOM uint32_t KEYR3; /*!< AES Key Register 3. */
	__IOM uint32_t IVR0; /*!< AES Initialization Vector Register 0. */
	__IOM uint32_t IVR1; /*!< AES Initialization Vector Register 1. */
	__IOM uint32_t IVR2; /*!< AES Initialization Vector Register 2. */
	__IOM uint32_t IVR3; /*!< AES Initialization Vector Register 3. */
	__IOM uint32_t KEYR4; /*!< AES Key Register 4. */
	__IOM uint32_t KEYR5; /*!< AES Key Register 5. */
	__IOM uint32_t KEYR6; /*!< AES Key Register 6. */
	__IOM uint32_t KEYR7; /*!< AES Key Register 7. */
	__IOM uint32_t SUSP0R; /*!< AES Suspend 0 Register. */
	__IOM uint32_t SUSP1R; /*!< AES Suspend 1 Register. */
	__IOM uint32_t SUSP2R; /*!< AES Suspend 2 Register. */
	__IOM uint32_t SUSP3R; /*!< AES Suspend 3 Register. */
	__IOM uint32_t SUSP4R; /*!< AES Suspend 4 Register. */
	__IOM uint32_t SUSP5R; /*!< AES Suspend 5 Register. */
	__IOM uint32_t SUSP6R; /*!< AES Suspend 6 Register. */
	__IOM uint32_t SUSP7R; /*!< AES Suspend 7 Register. */
}STM32L4xx_AES_TypeDef;

/*CR register.*/
#define AES_CR_KEYSIZE_Pos 18U
#define AES_CR_KEYSIZE_Msk (0x1UL << AES_CR_KEYSIZE_Pos)
#define AES_CR_KEYSIZE AES_CR_KEYSIZE_Msk

#define AES_CR_GCMPH_Pos 13U
#define AES_CR_GCMPH_Msk (0x3UL << AES_CR_GCMPH_Pos)
#define AES_CR_GCMPH AES_CR_GCMPH_Msk
#define AES_CR_GCMPH_INIT (0x0UL << AES_CR_GCMPH_Pos)
#define AES_CR_GCMPH_HEADER (0x1UL << AES_CR_GCMPH_Pos)
#define AES_CR_GCMPH_PAYLOAD (0x2UL << AES_CR_GCMPH_Pos)
#define AES_CR_GCMPH_FINAL (0x3UL << AES_CR_GCMPH_Pos)

#define AES_CR_DMAOUTEN_Pos 12U
#define AES_CR_DMAOUTEN_Msk (0x1UL << AES_CR_DMAOUTEN_Pos)
#define AES_CR_DMAOUTEN AES_CR_DMAOUTEN_Msk

#define AES_CR_DMAINEN_Pos 11U
#define AES_CR_DMAINEN_Msk (0x1UL << AES_CR_DMAINEN_Pos)
#define AES_CR_DMAINEN AES_CR_DMAINEN_Msk

#define AES_CR_ERRIE_Pos 10U
#define AES_CR_ERRIE_Msk (0x1UL << AES_CR_ERRIE_Pos)
#define AES_CR_ERRIE AES_CR_ERRIE_Msk

#define AES_CR_CCFIE_Pos 9U
#define AES_CR_CCFIE_Msk (0x1UL << AES_CR_CCFIE_Pos)

#define AES_CR_ERRC_Pos 8U
#define AES_CR_ERRC_Msk (0x1UL << AES_CR_ERRC_Pos)
#define AES_CR_ERRC AES_CR_ERRC_Msk

#define AES_CR_CCFC_Pos 7U
#define AES_CR_CCFC_Msk (0x1UL << AES_CR_CCFC_Pos)
#define AES_CR_CCFC AES_CR_CCFC_Msk

#define AES_CR_CHMOD2_Pos 16U
#define AES_CR_CHMOD_Pos 5U
#define AES_CR_CHMOD_Msk ((0x1UL << AES_CR_CHMOD2_Pos) | (0x3UL << AES_CR_CHMOD_Pos))
#define AES_CR_CHMOD AES_CR_CHMOD_Msk
#define AES_CR_CHMOD_ECB ((0x0UL << AES_CR_CHMOD2_Pos) | (0x0UL << AES_CR_CHMOD_Pos))
#define AES_CR_CHMOD_CBC ((0x0UL << AES_CR_CHMOD2_Pos) | (0x1UL << AES_CR_CHMOD_Pos))
#define AES_CR_CHMOD_CTR ((0x0UL << AES_CR_CHMOD2_Pos) | (0x2UL << AES_CR_CHMOD_Pos))
#define AES_CR_CHMOD_GCM_GMAC ((0x0UL << AES_CR_CHMOD2_Pos) | (0x3UL << AES_CR_CHMOD_Pos))
#define AES_CR_CHMOD_CCM ((0x1UL << AES_CR_CHMOD2_Pos) | (0x0UL << AES_CR_CHMOD_Pos))

#define AES_CR_MODE_Pos 3U
#define AES_CR_MODE_Msk (0x3UL << AES_CR_MODE_Pos)
#define AES_CR_MODE AES_CR_MODE_Msk
#define AES_CR_MODE_ENCRYPTION (0x0UL << AES_CR_MODE_Pos)
#define AES_CR_MODE_KEY_DERIVATION (0x1UL << AES_CR_MODE_Pos)
#define AES_CR_MODE_DECRYPTION (0x2UL << AES_CR_MODE_Pos)
#define AES_CR_MODE_KEY_DERIVATION_SINGLE_DECRYPTION (0x3UL << AES_CR_MODE_Pos)

#define AES_CR_DATATYPE_Pos 1U
#define AES_CR_DATATYPE_Msk (0x3UL << AES_CR_DATATYPE_Pos)
#define AES_CR_DATATYPE AES_CR_DATATYPE_Msk
#define AES_CR_DATATYPE_NONE (0x0UL << AES_CR_DATATYPE_Pos)
#define AES_CR_DATATYPE_HALFWORD (0x1UL << AES_CR_DATATYPE_Pos)
#define AES_CR_DATATYPE_BTYTE (0x2UL << AES_CR_DATATYPE_Pos)
#define AES_CR_DATATYPE_BIT (0x3UL << AES_CR_DATATYPE_Pos)

#define AES_CR_EN_Pos 0U
#define AES_CR_EN_Msk (0x1UL << AES_CR_EN_Pos)
#define AES_CR_EN AES_CR_EN_Msk

/*SR register.*/
#define AES_SR_BUSY_Pos 3U
#define AES_SR_BUSY_Msk (0x1UL << AES_SR_BUSY_Pos)
#define AES_SR_BUSY AES_SR_BUSY_Msk

#define AES_SR_WRERR_Pos 2U
#define AES_SR_WRERR_Msk (0x1UL << AES_SR_WRERR_Pos)
#define AES_SR_WRERR AES_SR_WRERR_Msk

#define AES_SR_RDERR_Pos 1U
#define AES_SR_RDERR_Msk (0x1UL << AES_SR_RDERR_Pos)
#define AES_SR_RDERR AES_SR_RDERR_Msk

#define AES_SR_CCF_Pos 0U
#define AES_SR_CCF_Msk (0x1UL << AES_SR_CCF_Pos)
#define AES_SR_CCF AES_SR_CCF_Msk
#endif

/* ========================================================================= */
/* ============                       TIM1                      ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR1; /*!< TIM1 Control Register 1. */
	__IOM uint32_t CR2; /*!< TIM1 Control Register 2. */
	__IOM uint32_t SMCR; /*!< TIM1 Slave Mode Control Register. */
	__IOM uint32_t DIER; /*!< TIM1 DMA/Interrupt Enable Register. */
	__IOM uint32_t SR; /*!< TIM1 Status Register. */
	__OM uint32_t EGR; /*!< TIM1 Event Generation Register. */
	__IOM uint32_t CCMR1; /*!< TIM1 Capture/Compare Mode Register 1. */
	__IOM uint32_t CCMR2; /*!< TIM1 Capture/Compare Mode Register 2. */
	__IOM uint32_t CCER; /*!< TIM1 Capture/Compare Enable Register. */
	__IOM uint32_t CNT; /*!< TIM1 Counter. */
	__IOM uint32_t PSC; /*!< TIM1 Prescaler. */
	__IOM uint32_t ARR; /*!< TIM1 Auto-Reload Register. */
	__IOM uint32_t RCR; /*!< TIM1 Repetition Counter Register. */
	__IOM uint32_t CCR1; /*!< TIM1 Capture/Compare Register 1. */
	__IOM uint32_t CCR2; /*!< TIM1 Capture/Compare Register 2. */
	__IOM uint32_t CCR3; /*!< TIM1 Capture/Compare Register 3. */
	__IOM uint32_t CCR4; /*!< TIM1 Capture/Compare Register 4. */
	__IOM uint32_t BDTR; /*!< TIM1 Break and Dead Time Register. */
	__IOM uint32_t DCR; /*!< TIM1 DMA Control Register. */
	__IOM uint32_t DMAR; /*!< TIM1 DMA address for full transfer. */
	__IOM uint32_t OR1; /*!< TIM1 Option Register 1. */
	__IOM uint32_t CCMR3; /*!< TIM1 Capture/Compare Mode Register 3. */
	__IOM uint32_t CCR5; /*!< TIM1 Capture/Compare Register 5. */
	__IOM uint32_t CCR6; /*!< TIM1 Capture/Compare Register 6. */
	__IOM uint32_t OR2; /*!< TIM1 Option Register 2. */
	__IOM uint32_t OR3; /*!< TIM1 Option Register 3. */
}STM32L4xx_TIM1_TypeDef;

/*CR1 register.*/
#define TIM1_CR1_UIFREMAP_Pos 11U
#define TIM1_CR1_UIFREMAP_Msk (0x1UL << TIM1_CR1_UIFREMAP_Pos)
#define TIM1_CR1_UIFREMAP TIM1_CR1_UIFREMAP_Msk

#define TIM1_CR1_CKD_Pos 8U
#define TIM1_CR1_CKD_Msk (0x3UL << TIM1_CR1_CKD_Pos)
#define TIM1_CR1_CKD TIM1_CR1_CKD_Msk
#define TIM1_CR1_CKD_MUL_1 (0x0UL << TIM1_CR1_CKD_Pos)
#define TIM1_CR1_CKD_MUL_2 (0x1UL << TIM1_CR1_CKD_Pos)
#define TIM1_CR1_CKD_MUL_4 (0x2UL << TIM1_CR1_CKD_Pos)

#define TIM1_CR1_ARPE_Pos 7U
#define TIM1_CR1_ARPE_Msk (0x1UL << TIM1_CR1_ARPE_Pos)
#define TIM1_CR1_ARPE TIM1_CR1_ARPE_Msk

#define TIM1_CR1_CMS_Pos 5U
#define TIM1_CR1_CMS_Msk (0x3UL << TIM1_CR1_CMS_Pos)
#define TIM1_CR1_CMS TIM1_CR1_CMS_Msk
#define TIM1_CR1_CMS_EDGE_ALIGNED (0x0UL << TIM1_CR1_CMS_Pos)
#define TIM1_CR1_CMS_CENTER_ALIGNED_1 (0x1UL << TIM1_CR1_CMS_Pos)
#define TIM1_CR1_CMS_CENTER_ALIGNED_2 (0x2UL << TIM1_CR1_CMS_Pos)
#define TIM1_CR1_CMS_CENTER_ALIGNED_3 (0x3UL << TIM1_CR1_CMS_Pos)

#define TIM1_CR1_DIR_Pos 4U
#define TIM1_CR1_DIR_Msk (0x1UL << TIM1_CR1_DIR_Pos)
#define TIM1_CR1_DIR TIM1_CR1_DIR_Msk

#define TIM1_CR1_OPM_Pos 3U
#define TIM1_CR1_OPM_Msk (0x1UL << TIM1_CR1_OPM_Pos)
#define TIM1_CR1_OPM TIM1_CR1_OPM_Msk

#define TIM1_CR1_URS_Pos 2U
#define TIM1_CR1_URS_Msk (0x1UL << TIM1_CR1_URS_Pos)
#define TIM1_CR1_URS TIM1_CR1_URS_Msk

#define TIM1_CR1_UDIS_Pos 1U
#define TIM1_CR1_UDIS_Msk (0x1UL << TIM1_CR1_UDIS_Pos)
#define TIM1_CR1_UDIS TIM1_CR1_UDIS_Msk

#define TIM1_CR1_CEN_Pos 0U
#define TIM1_CR1_CEN_Msk (0x1UL << TIM1_CR1_CEN_Pos)
#define TIM1_CR1_CEN TIM1_CR1_CEN_Msk

/*CR2 register.*/
#define TIM1_CR2_MMS2_Pos 20U
#define TIM1_CR2_MMS2_Msk (0xFUL << TIM1_CR2_MMS2_Pos)
#define TIM1_CR2_MMS2 TIM1_CR2_MMS2_Msk

#define TIM1_CR2_OIS6_Pos 18U
#define TIM1_CR2_OIS6_Msk (0x1UL << TIM1_CR2_OIS6_Pos)
#define TIM1_CR2_OIS6 TIM1_CR2_OIS6_Msk

#define TIM1_CR2_OIS6_Pos 18U
#define TIM1_CR2_OIS6_Msk (0x1UL << TIM1_CR2_OIS6_Pos)
#define TIM1_CR2_OIS6 TIM1_CR2_OIS6_Msk

#define TIM1_CR2_OIS5_Pos 16U
#define TIM1_CR2_OIS5_Msk (0x1UL << TIM1_CR2_OIS5_Pos)
#define TIM1_CR2_OIS5 TIM1_CR2_OIS5_Msk

#define TIM1_CR2_OIS4_Pos 14U
#define TIM1_CR2_OIS4_Msk (0x1UL << TIM1_CR2_OIS4_Pos)
#define TIM1_CR2_OIS4 TIM1_CR2_OIS4_Msk

#define TIM1_CR2_OIS3N_Pos 13U
#define TIM1_CR2_OIS3N_Msk (0x1UL << TIM1_CR2_OIS3N_Pos)
#define TIM1_CR2_OIS3N TIM1_CR2_OIS3N_Msk

#define TIM1_CR2_OIS3_Pos 12U
#define TIM1_CR2_OIS3_Msk (0x1UL << TIM1_CR2_OIS3_Pos)
#define TIM1_CR2_OIS3 TIM1_CR2_OIS3_Msk

#define TIM1_CR2_OIS2N_Pos 11U
#define TIM1_CR2_OIS2N_Msk (0x1UL << TIM1_CR2_OIS2N_Pos)
#define TIM1_CR2_OIS2N TIM1_CR2_OIS2N_Msk

#define TIM1_CR2_OIS2_Pos 10U
#define TIM1_CR2_OIS2_Msk (0x1UL << TIM1_CR2_OIS2_Pos)
#define TIM1_CR2_OIS2 TIM1_CR2_OIS2_Msk

#define TIM1_CR2_OIS1N_Pos 9U
#define TIM1_CR2_OIS1N_Msk (0x1UL << TIM1_CR2_OIS1N_Pos)
#define TIM1_CR2_OIS1N TIM1_CR2_OIS1N_Msk

#define TIM1_CR2_OIS1_Pos 8U
#define TIM1_CR2_OIS1_Msk (0x1UL << TIM1_CR2_OIS1_Pos)
#define TIM1_CR2_OIS1 TIM1_CR2_OIS1_Msk

#define TIM1_CR2_TI1S_Pos 7U
#define TIM1_CR2_TI1S_Msk (0x1UL << TIM1_CR2_TI1S_Pos)
#define TIM1_CR2_TI1S TIM1_CR2_TI1S_Msk

#define TIM1_CR2_MMS_Pos 4U
#define TIM1_CR2_MMS_Msk (0x7UL << TIM1_CR2_MMS_Pos)
#define TIM1_CR2_MMS TIM1_CR2_MMS_Msk

#define TIM1_CR2_CCDS_Pos 3U
#define TIM1_CR2_CCDS_Msk (0x1UL << TIM1_CR2_CCDS_Pos)
#define TIM1_CR2_CCDS TIM1_CR2_CCDS_Msk

#define TIM1_CR2_CCUS_Pos 2U
#define TIM1_CR2_CCUS_Msk (0x1UL << TIM1_CR2_CCUS_Pos)
#define TIM1_CR2_CCUS TIM1_CR2_CCUS_Msk

#define TIM1_CR2_CCPC_Pos 0U
#define TIM1_CR2_CCPC_Msk (0x1UL << TIM1_CR2_CCPC_Pos)
#define TIM1_CR2_CCPC TIM1_CR2_CCPC_Msk

/*SMCR register.*/
#define TIM1_SMCR_ETP_Pos 15U
#define TIM1_SMCR_ETP_Msk (0x1UL << TIM1_SMCR_ETP_Pos)
#define TIM1_SMCR_ETP TIM1_SMCR_ETP_Msk

#define TIM1_SMCR_ECE_Pos 14U
#define TIM1_SMCR_ECE_Msk (0x1UL << TIM1_SMCR_ECE_Pos)
#define TIM1_SMCR_ECE TIM1_SMCR_ECE_Msk

#define TIM1_SMCR_ETPS_Pos 12U
#define TIM1_SMCR_ETPS_Msk (0x3UL << TIM1_SMCR_ETPS_Pos)
#define TIM1_SMCR_ETPS TIM1_SMCR_ETPS_Msk
#define TIM1_SMCR_ETPS_DIV_1 (0x0UL << TIM1_SMCR_ETPS_Pos)
#define TIM1_SMCR_ETPS_DIV_2 (0x1UL << TIM1_SMCR_ETPS_Pos)
#define TIM1_SMCR_ETPS_DIV_4 (0x2UL << TIM1_SMCR_ETPS_Pos)
#define TIM1_SMCR_ETPS_DIV_8 (0x3UL << TIM1_SMCR_ETPS_Pos)

#define TIM1_SMCR_ETF_Pos 8U
#define TIM1_SMCR_ETF_Msk (0xFUL << TIM1_SMCR_ETF_Pos)
#define TIM1_SMCR_ETF TIM1_SMCR_ETF_Msk

#define TIM1_SMCR_MSM_Pos 7U
#define TIM1_SMCR_MSM_Msk (0x1UL << TIM1_SMCR_MSM_Pos)
#define TIM1_SMCR_MSM TIM1_SMCR_MSM_Msk

#define TIM1_SMCR_TS_Pos 4U
#define TIM1_SMCR_TS_Msk (0x7UL << TIM1_SMCR_TS_Pos)
#define TIM1_SMCR_TS TIM1_SMCR_TS_Msk
#define TIM1_SMCR_TS_ITR0 (0x0UL << TIM1_SMCR_TS_Pos)
#define TIM1_SMCR_TS_ITR1 (0x1UL << TIM1_SMCR_TS_Pos)
#define TIM1_SMCR_TS_TI1F_ED (0x4UL << TIM1_SMCR_TS_Pos)
#define TIM1_SMCR_TS_TI1FP1 (0x5UL << TIM1_SMCR_TS_Pos)
#define TIM1_SMCR_TS_TI2FP2 (0x6UL << TIM1_SMCR_TS_Pos)
#define TIM1_SMCR_TS_ETRF (0x7UL << TIM1_SMCR_TS_Pos)

#define TIM1_SMCR_OCCS_Pos 3U
#define TIM1_SMCR_OCCS_Msk (0x1UL << TIM1_SMCR_OCCS_Pos)
#define TIM1_SMCR_OCCS TIM1_SMCR_OCCS_Msk

#define TIM1_SMCR_SMS3_Pos 16U
#define TIM1_SMCR_SMS_Pos 0U
#define TIM1_SMCR_SMS_Msk ((0x1UL << TIM1_SMCR_SMS3_Pos)| (0x7UL << TIM1_SMCR_SMS_Pos))
#define TIM1_SMCR_SMS TIM1_SMCR_SMS_Msk
#define TIM1_SMCR_SMS_SLAVE_DISABLED ((0x0UL << TIM1_SMCR_SMS3_Pos)| (0x0UL << TIM1_SMCR_SMS_Pos))
#define TIM1_SMCR_SMS_ENCODER_1 ((0x0UL << TIM1_SMCR_SMS3_Pos)| (0x1UL << TIM1_SMCR_SMS_Pos))
#define TIM1_SMCR_SMS_ENCODER_2 ((0x0UL << TIM1_SMCR_SMS3_Pos)| (0x2UL << TIM1_SMCR_SMS_Pos))
#define TIM1_SMCR_SMS_ENCODER_3 ((0x0UL << TIM1_SMCR_SMS3_Pos)| (0x3UL << TIM1_SMCR_SMS_Pos))
#define TIM1_SMCR_SMS_RESET ((0x0UL << TIM1_SMCR_SMS3_Pos)| (0x4UL << TIM1_SMCR_SMS_Pos))
#define TIM1_SMCR_SMS_GATED ((0x0UL << TIM1_SMCR_SMS3_Pos)| (0x5UL << TIM1_SMCR_SMS_Pos))
#define TIM1_SMCR_SMS_TRIGGER ((0x0UL << TIM1_SMCR_SMS3_Pos)| (0x6UL << TIM1_SMCR_SMS_Pos))
#define TIM1_SMCR_SMS_EXTERNAL_CLOCK ((0x0UL << TIM1_SMCR_SMS3_Pos)| (0x7UL << TIM1_SMCR_SMS_Pos))
#define TIM1_SMCR_SMS_COMBINED ((0x1UL << TIM1_SMCR_SMS3_Pos)| (0x0UL << TIM1_SMCR_SMS_Pos))

/*DIER register.*/
#define TIM1_DIER_TDE_Pos 14U
#define TIM1_DIER_TDE_Msk (0x1UL << TIM1_DIER_TDE_Pos)
#define TIM1_DIER_TDE TIM1_DIER_TDE_Msk

#define TIM1_DIER_COMDE_Pos 13U
#define TIM1_DIER_COMDE_Msk (0x1UL << TIM1_DIER_COMDE_Pos)
#define TIM1_DIER_COMDE TIM1_DIER_COMDE_Msk

#define TIM1_DIER_CC4DE_Pos 12U
#define TIM1_DIER_CC4DE_Msk (0x1UL << TIM1_DIER_CC4DE_Pos)
#define TIM1_DIER_CC4DE TIM1_DIER_CC4DE_Msk

#define TIM1_DIER_CC3DE_Pos 11U
#define TIM1_DIER_CC3DE_Msk (0x1UL << TIM1_DIER_CC3DE_Pos)
#define TIM1_DIER_CC3DE TIM1_DIER_CC3DE_Msk

#define TIM1_DIER_CC2DE_Pos 10U
#define TIM1_DIER_CC2DE_Msk (0x1UL << TIM1_DIER_CC2DE_Pos)
#define TIM1_DIER_CC2DE TIM1_DIER_CC2DE_Msk

#define TIM1_DIER_CC1DE_Pos 9U
#define TIM1_DIER_CC1DE_Msk (0x1UL << TIM1_DIER_CC1DE_Pos)
#define TIM1_DIER_CC1DE TIM1_DIER_CC1DE_Msk

#define TIM1_DIER_UDE_Pos 8U
#define TIM1_DIER_UDE_Msk (0x1UL << TIM1_DIER_UDE_Pos)
#define TIM1_DIER_UDE TIM1_DIER_UDE_Msk

#define TIM1_DIER_BIE_Pos 7U
#define TIM1_DIER_BIE_Msk (0x1UL << TIM1_DIER_BIE_Pos)
#define TIM1_DIER_BIE TIM1_DIER_BIE_Msk

#define TIM1_DIER_TIE_Pos 6U
#define TIM1_DIER_TIE_Msk (0x1UL << TIM1_DIER_TIE_Pos)
#define TIM1_DIER_TIE TIM1_DIER_TIE_Msk

#define TIM1_DIER_COMIE_Pos 5U
#define TIM1_DIER_COMIE_Msk (0x1UL << TIM1_DIER_COMIE_Pos)
#define TIM1_DIER_COMIE TIM1_DIER_COMIE_Msk

#define TIM1_DIER_CC4IE_Pos 4U
#define TIM1_DIER_CC4IE_Msk (0x1UL << TIM1_DIER_CC4IE_Pos)
#define TIM1_DIER_CC4IE TIM1_DIER_CC4IE_Msk

#define TIM1_DIER_CC3IE_Pos 3U
#define TIM1_DIER_CC3IE_Msk (0x1UL << TIM1_DIER_CC3IE_Pos)
#define TIM1_DIER_CC3IE TIM1_DIER_CC3IE_Msk

#define TIM1_DIER_CC2IE_Pos 2U
#define TIM1_DIER_CC2IE_Msk (0x1UL << TIM1_DIER_CC2IE_Pos)
#define TIM1_DIER_CC2IE TIM1_DIER_CC2IE_Msk

#define TIM1_DIER_CC1IE_Pos 1U
#define TIM1_DIER_CC1IE_Msk (0x1UL << TIM1_DIER_CC1IE_Pos)
#define TIM1_DIER_CC1IE TIM1_DIER_CC1IE_Msk

#define TIM1_DIER_UIE_Pos 0U
#define TIM1_DIER_UIE_Msk (0x1UL << TIM1_DIER_UIE_Pos)
#define TIM1_DIER_UIE TIM1_DIER_UIE_Msk

/*SR register.*/
#define TIM1_SR_CC6IF_Pos 17U
#define TIM1_SR_CC6IF_Msk (0x1UL << TIM1_SR_CC6IF_Pos)
#define TIM1_SR_CC6IF TIM1_SR_CC6IF_Msk

#define TIM1_SR_CC5IF_Pos 16U
#define TIM1_SR_CC5IF_Msk (0x1UL << TIM1_SR_CC5IF_Pos)
#define TIM1_SR_CC5IF TIM1_SR_CC5IF_Msk

#define TIM1_SR_SBIF_Pos 13U
#define TIM1_SR_SBIF_Msk (0x1UL << TIM1_SR_SBIF_Pos)
#define TIM1_SR_SBIF TIM1_SR_SBIF_Msk

#define TIM1_SR_CC4OF_Pos 12U
#define TIM1_SR_CC4OF_Msk (0x1UL << TIM1_SR_CC4OF_Pos)
#define TIM1_SR_CC4OF TIM1_SR_CC4OF_Msk

#define TIM1_SR_CC3OF_Pos 11U
#define TIM1_SR_CC3OF_Msk (0x1UL << TIM1_SR_CC3OF_Pos)
#define TIM1_SR_CC3OF TIM1_SR_CC3OF_Msk

#define TIM1_SR_CC2OF_Pos 10U
#define TIM1_SR_CC2OF_Msk (0x1UL << TIM1_SR_CC2OF_Pos)
#define TIM1_SR_CC2OF TIM1_SR_CC2OF_Msk

#define TIM1_SR_CC1OF_Pos 9U
#define TIM1_SR_CC1OF_Msk (0x1UL << TIM1_SR_CC1OF_Pos)
#define TIM1_SR_CC1OF TIM1_SR_CC1OF_Msk

#define TIM1_SR_B2IF_Pos 8U
#define TIM1_SR_B2IF_Msk (0x1UL << TIM1_SR_B2IF_Pos)
#define TIM1_SR_B2IF TIM1_SR_B2IF_Msk

#define TIM1_SR_BIF_Pos 7U
#define TIM1_SR_BIF_Msk (0x1UL << TIM1_SR_BIF_Pos)
#define TIM1_SR_BIF TIM1_SR_BIF_Msk

#define TIM1_SR_TIF_Pos 6U
#define TIM1_SR_TIF_Msk (0x1UL << TIM1_SR_TIF_Pos)
#define TIM1_SR_TIF TIM1_SR_TIF_Msk

#define TIM1_SR_COMIF_Pos 5U
#define TIM1_SR_COMIF_Msk (0x1UL << TIM1_SR_COMIF_Pos)
#define TIM1_SR_COMIF TIM1_SR_COMIF_Msk

#define TIM1_SR_CC4IF_Pos 4U
#define TIM1_SR_CC4IF_Msk (0x1UL << TIM1_SR_CC4IF_Pos)
#define TIM1_SR_CC4IF TIM1_SR_CC4IF_Msk

#define TIM1_SR_CC3IF_Pos 3U
#define TIM1_SR_CC3IF_Msk (0x1UL << TIM1_SR_CC3IF_Pos)
#define TIM1_SR_CC3IF TIM1_SR_CC3IF_Msk

#define TIM1_SR_CC2IF_Pos 2U
#define TIM1_SR_CC2IF_Msk (0x1UL << TIM1_SR_CC2IF_Pos)
#define TIM1_SR_CC2IF TIM1_SR_CC2IF_Msk

#define TIM1_SR_CC1IF_Pos 1U
#define TIM1_SR_CC1IF_Msk (0x1UL << TIM1_SR_CC1IF_Pos)
#define TIM1_SR_CC1IF TIM1_SR_CC1IF_Msk

#define TIM1_SR_UIF_Pos 0U
#define TIM1_SR_UIF_Msk (0x1UL << TIM1_SR_UIF_Pos)
#define TIM1_SR_UIF TIM1_SR_UIF_Msk

/*EGR register.*/
#define TIM1_EGR_B2G_Pos 8U
#define TIM1_EGR_B2G_Msk (0x1UL << TIM1_EGR_B2G_Pos)
#define TIM1_EGR_B2G TIM1_EGR_B2G_Msk

#define TIM1_EGR_BG_Pos 7U
#define TIM1_EGR_BG_Msk (0x1UL << TIM1_EGR_BG_Pos)
#define TIM1_EGR_BG TIM1_EGR_BG_Msk

#define TIM1_EGR_TG_Pos 6U
#define TIM1_EGR_TG_Msk (0x1UL << TIM1_EGR_TG_Pos)
#define TIM1_EGR_TG TIM1_EGR_TG_Msk

#define TIM1_EGR_COMG_Pos 5U
#define TIM1_EGR_COMG_Msk (0x1UL << TIM1_EGR_COMG_Pos)
#define TIM1_EGR_COMG TIM1_EGR_COMG_Msk

#define TIM1_EGR_CC4G_Pos 4U
#define TIM1_EGR_CC4G_Msk (0x1UL << TIM1_EGR_CC4G_Pos)
#define TIM1_EGR_CC4G TIM1_EGR_CC4G_Msk

#define TIM1_EGR_CC3G_Pos 3U
#define TIM1_EGR_CC3G_Msk (0x1UL << TIM1_EGR_CC3G_Pos)
#define TIM1_EGR_CC3G TIM1_EGR_CC3G_Msk

#define TIM1_EGR_CC2G_Pos 2U
#define TIM1_EGR_CC2G_Msk (0x1UL << TIM1_EGR_CC2G_Pos)
#define TIM1_EGR_CC2G TIM1_EGR_CC2G_Msk

#define TIM1_EGR_CC1G_Pos 1U
#define TIM1_EGR_CC1G_Msk (0x1UL << TIM1_EGR_CC1G_Pos)
#define TIM1_EGR_CC1G TIM1_EGR_CC1G_Msk

#define TIM1_EGR_UG_Pos 0U
#define TIM1_EGR_UG_Msk (0x1UL << TIM1_EGR_UG_Pos)
#define TIM1_EGR_UG TIM1_EGR_UG_Msk

/*CCMR1 register.*/
/*******Channel 2.*******/
#define TIM1_CCMR1_CC2S_Pos 8U
#define TIM1_CCMR1_CC2S_Msk (0x3UL << TIM1_CCMR1_CC2S_Pos)
#define TIM1_CCMR1_CC2S TIM1_CCMR1_CC2S_Msk
#define TIM1_CCMR1_CC2S_OUTPUT (0x0UL << TIM1_CCMR1_CC2S_Pos)
#define TIM1_CCMR1_CC2S_INPUT_TI2 (0x1UL << TIM1_CCMR1_CC2S_Pos)
#define TIM1_CCMR1_CC2S_INPUT_TI1 (0x2UL << TIM1_CCMR1_CC2S_Pos)
#define TIM1_CCMR1_CC2S_INPUT_TRC (0x3UL << TIM1_CCMR1_CC2S_Pos)

/*Input Capture Mode.*/
#define TIM1_CCMR1_IC2F_Pos 12U
#define TIM1_CCMR1_IC2F_Msk (0xFUL << TIM1_CCMR1_IC2F_Pos)
#define TIM1_CCMR1_IC2F TIM1_CCMR1_IC2F_Msk

#define TIM1_CCMR1_IC2PSC_Pos 10U
#define TIM1_CCMR1_IC2PSC_Msk (0x3UL << TIM1_CCMR1_IC2PSC_Pos)
#define TIM1_CCMR1_IC2PSC TIM1_CCMR1_IC2PSC_Msk

/*Output Compare Mode.*/
#define TIM1_CCMR1_OC2CE_Pos 15U
#define TIM1_CCMR1_OC2CE_Msk (0x1UL << TIM1_CCMR1_OC2CE_Pos)
#define TIM1_CCMR1_OC2CE TIM1_CCMR1_OC2CE_Msk

#define TIM1_CCMR1_OC2M3_Pos 24U
#define TIM1_CCMR1_OC2M_Pos 12U
#define TIM1_CCMR1_OC2M_Msk ((0x1UL << TIM1_CCMR1_OC2M3_Pos) | (0x7UL << TIM1_CCMR1_OC2M_Pos))
#define TIM1_CCMR1_OC2M TIM1_CCMR1_OC2M_Msk

#define TIM1_CCMR1_OC2PE_Pos 11U
#define TIM1_CCMR1_OC2PE_Msk (0x1UL << TIM1_CCMR1_OC2PE_Pos)
#define TIM1_CCMR1_OC2PE TIM1_CCMR1_OC2PE_Msk

#define TIM1_CCMR1_OC2FE_Pos 10U
#define TIM1_CCMR1_OC2FE_Msk (0x1UL << TIM1_CCMR1_OC2FE_Pos)
#define TIM1_CCMR1_OC2FE TIM1_CCMR1_OC2FE_Msk

/*******Channel 1.*******/
#define TIM1_CCMR1_CC1S_Pos 0U
#define TIM1_CCMR1_CC1S_Msk (0x3UL << TIM1_CCMR1_CC1S_Pos)
#define TIM1_CCMR1_CC1S TIM1_CCMR1_CC1S_Msk
#define TIM1_CCMR1_CC1S_OUTPUT (0x0UL << TIM1_CCMR1_CC1S_Pos)
#define TIM1_CCMR1_CC1S_INPUT_TI2 (0x1UL << TIM1_CCMR1_CC1S_Pos)
#define TIM1_CCMR1_CC1S_INPUT_TI1 (0x2UL << TIM1_CCMR1_CC1S_Pos)
#define TIM1_CCMR1_CC1S_INPUT_TRC (0x3UL << TIM1_CCMR1_CC1S_Pos)

/*Input Capture Mode.*/
#define TIM1_CCMR1_IC1F_Pos 4U
#define TIM1_CCMR1_IC1F_Msk (0xFUL << TIM1_CCMR1_IC1F_Pos)
#define TIM1_CCMR1_IC1F TIM1_CCMR1_IC1F_Msk

#define TIM1_CCMR1_IC1PSC_Pos 2U
#define TIM1_CCMR1_IC1PSC_Msk (0x3UL << TIM1_CCMR1_IC1PSC_Pos)
#define TIM1_CCMR1_IC1PSC TIM1_CCMR1_IC1PSC_Msk

/*Output Compare Mode.*/
#define TIM1_CCMR1_OC1CE_Pos 7U
#define TIM1_CCMR1_OC1CE_Msk (0x1UL << TIM1_CCMR1_OC1CE_Pos)
#define TIM1_CCMR1_OC1CE TIM1_CCMR1_OC1CE_Msk

#define TIM1_CCMR1_OC1M3_Pos 16U
#define TIM1_CCMR1_OC1M_Pos 4U
#define TIM1_CCMR1_OC1M_Msk ((0x1UL << TIM1_CCMR1_OC1M3_Pos) | (0x7UL << TIM1_CCMR1_OC1M_Pos))
#define TIM1_CCMR1_OC1M TIM1_CCMR1_OC1M_Msk

#define TIM1_CCMR1_OC1PE_Pos 3U
#define TIM1_CCMR1_OC1PE_Msk (0x1UL << TIM1_CCMR1_OC1PE_Pos)
#define TIM1_CCMR1_OC1PE TIM1_CCMR1_OC1PE_Msk

#define TIM1_CCMR1_OC1FE_Pos 2U
#define TIM1_CCMR1_OC1FE_Msk (0x1UL << TIM1_CCMR1_OC1FE_Pos)
#define TIM1_CCMR1_OC1FE TIM1_CCMR1_OC1FE_Msk

/*CCMR2 register.*/
/*******Channel 4.*******/
#define TIM1_CCMR2_CC4S_Pos 8U
#define TIM1_CCMR2_CC4S_Msk (0x3UL << TIM1_CCMR2_CC4S_Pos)
#define TIM1_CCMR2_CC4S TIM1_CCMR2_CC4S_Msk
#define TIM1_CCMR2_CC4S_OUTPUT (0x0UL << TIM1_CCMR1_CC2S_Pos)
#define TIM1_CCMR2_CC4S_INPUT_TI2 (0x1UL << TIM1_CCMR1_CC2S_Pos)
#define TIM1_CCMR2_CC4S_INPUT_TI1 (0x2UL << TIM1_CCMR1_CC2S_Pos)
#define TIM1_CCMR2_CC4S_INPUT_TRC (0x3UL << TIM1_CCMR1_CC2S_Pos)

/*Input Capture Mode.*/
#define TIM1_CCMR2_IC4F_Pos 12U
#define TIM1_CCMR2_IC4F_Msk (0xFUL << TIM1_CCMR2_IC4F_Pos)
#define TIM1_CCMR2_IC4F TIM1_CCMR2_IC4F_Msk

#define TIM1_CCMR2_IC4PSC_Pos 10U
#define TIM1_CCMR2_IC4PSC_Msk (0x3UL << TIM1_CCMR2_IC4PSC_Pos)
#define TIM1_CCMR2_IC4PSC TIM1_CCMR2_IC4PSC_Msk

/*Output Compare Mode.*/
#define TIM1_CCMR2_OC4CE_Pos 15U
#define TIM1_CCMR2_OC4CE_Msk (0x1UL << TIM1_CCMR2_OC4CE_Pos)
#define TIM1_CCMR2_OC4CE TIM1_CCMR2_OC4CE_Msk


#define TIM1_CCMR2_OC4M3_Pos 24U
#define TIM1_CCMR2_OC4M_Pos 12U
#define TIM1_CCMR2_OC4M_Msk ((0x1UL << TIM1_CCMR2_OC4M3_Pos) | (0x7UL << TIM1_CCMR2_OC4M_Pos))
#define TIM1_CCMR2_OC4M TIM1_CCMR2_OC4M_Msk

#define TIM1_CCMR2_OC4PE_Pos 11U
#define TIM1_CCMR2_OC4PE_Msk (0x1UL << TIM1_CCMR2_OC4PE_Pos)
#define TIM1_CCMR2_OC4PE TIM1_CCMR2_OC4PE_Msk

#define TIM1_CCMR2_OC4FE_Pos 10U
#define TIM1_CCMR2_OC4FE_Msk (0x1UL << TIM1_CCMR2_OC4FE_Pos)
#define TIM1_CCMR2_OC4FE TIM1_CCMR2_OC4FE_Msk

/*******Channel 3.*******/
#define TIM1_CCMR2_CC3S_Pos 0U
#define TIM1_CCMR2_CC3S_Msk (0x3UL << TIM1_CCMR2_CC3S_Pos)
#define TIM1_CCMR2_CC3S TIM1_CCMR2_CC3S_Msk
#define TIM1_CCMR2_CC3S_OUTPUT (0x0UL << TIM1_CCMR2_CC3S_Pos)
#define TIM1_CCMR2_CC3S_INPUT_TI2 (0x1UL << TIM1_CCMR2_CC3S_Pos)
#define TIM1_CCMR2_CC3S_INPUT_TI1 (0x2UL << TIM1_CCMR2_CC3S_Pos)
#define TIM1_CCMR2_CC3S_INPUT_TRC (0x3UL << TIM1_CCMR2_CC3S_Pos)

/*Input Capture Mode.*/
#define TIM1_CCMR2_IC3F_Pos 4U
#define TIM1_CCMR2_IC3F_Msk (0xFUL << TIM1_CCMR2_IC3F_Pos)
#define TIM1_CCMR2_IC3F TIM1_CCMR2_IC3F_Msk

#define TIM1_CCMR2_IC3PSC_Pos 2U
#define TIM1_CCMR2_IC3PSC_Msk (0x3UL << TIM1_CCMR2_IC3PSC_Pos)
#define TIM1_CCMR2_IC3PSC TIM1_CCMR2_IC3PSC_Msk

/*Output Compare Mode.*/
#define TIM1_CCMR2_OC3CE_Pos 7U
#define TIM1_CCMR2_OC3CE_Msk (0x1UL << TIM1_CCMR2_OC3CE_Pos)
#define TIM1_CCMR2_OC3CE TIM1_CCMR2_OC3CE_Msk

#define TIM1_CCMR2_OC3M3_Pos 16U
#define TIM1_CCMR2_OC3M_Pos 4U
#define TIM1_CCMR2_OC3M_Msk ((0x1UL << TIM1_CCMR2_OC3M3_Pos) | (0x7UL << TIM1_CCMR2_OC3M_Pos))

#define TIM1_CCMR2_OC3PE_Pos 3U
#define TIM1_CCMR2_OC3PE_Msk (0x1UL << TIM1_CCMR2_OC3PE_Pos)

#define TIM1_CCMR2_OC3FE_Pos 2U
#define TIM1_CCMR2_OC3FE_Msk (0x1UL << TIM1_CCMR2_OC3FE_Pos)

/*CCER register.*/
#define TIM1_CCER_CC6P_Pos 21U
#define TIM1_CCER_CC6P_Msk (0x1UL << TIM1_CCER_CC6P_Pos)
#define TIM1_CCER_CC6P TIM1_CCER_CC6P_Msk

#define TIM1_CCER_CC6E_Pos 20U
#define TIM1_CCER_CC6E_Msk (0x1UL << TIM1_CCER_CC6E_Pos)
#define TIM1_CCER_CC6E TIM1_CCER_CC6E_Msk

#define TIM1_CCER_CC5P_Pos 17U
#define TIM1_CCER_CC5P_Msk (0x1UL << TIM1_CCER_CC5P_Pos)
#define TIM1_CCER_CC5P TIM1_CCER_CC5P_Msk

#define TIM1_CCER_CC5E_Pos 16U
#define TIM1_CCER_CC5E_Msk (0x1UL << TIM1_CCER_CC5E_Pos)
#define TIM1_CCER_CC5E TIM1_CCER_CC5E_Msk

#define TIM1_CCER_CC4NP_Pos 15U
#define TIM1_CCER_CC4NP_Msk (0x1UL << TIM1_CCER_CC4NP_Pos)
#define TIM1_CCER_CC4NP TIM1_CCER_CC4NP_Msk

#define TIM1_CCER_CC4P_Pos 13U
#define TIM1_CCER_CC4P_Msk (0x1UL << TIM1_CCER_CC4P_Pos)
#define TIM1_CCER_CC4P TIM1_CCER_CC4P_Msk

#define TIM1_CCER_CC4E_Pos 12U
#define TIM1_CCER_CC4E_Msk (0x1UL << TIM1_CCER_CC4E_Pos)
#define TIM1_CCER_CC4E TIM1_CCER_CC4E_Msk

#define TIM1_CCER_CC3NP_Pos 11U
#define TIM1_CCER_CC3NP_Msk (0x1UL << TIM1_CCER_CC3NP_Pos)
#define TIM1_CCER_CC3NP TIM1_CCER_CC3NP_Msk

#define TIM1_CCER_CC3NE_Pos 10U
#define TIM1_CCER_CC3NE_Msk (0x1UL << TIM1_CCER_CC3NE_Pos)
#define TIM1_CCER_CC3NE TIM1_CCER_CC3NE_Msk

#define TIM1_CCER_CC3P_Pos 9U
#define TIM1_CCER_CC3P_Msk (0x1UL << TIM1_CCER_CC3P_Pos)
#define TIM1_CCER_CC3P TIM1_CCER_CC3P_Msk

#define TIM1_CCER_CC3E_Pos 8U
#define TIM1_CCER_CC3E_Msk (0x1UL << TIM1_CCER_CC3E_Pos)
#define TIM1_CCER_CC3E TIM1_CCER_CC3E_Msk

#define TIM1_CCER_CC2NP_Pos 7U
#define TIM1_CCER_CC2NP_Msk (0x1UL << TIM1_CCER_CC2NP_Pos)
#define TIM1_CCER_CC2NP TIM1_CCER_CC2NP_Msk

#define TIM1_CCER_CC2NE_Pos 6U
#define TIM1_CCER_CC2NE_Msk (0x1UL << TIM1_CCER_CC2NE_Pos)
#define TIM1_CCER_CC2NE TIM1_CCER_CC2NE_Msk

#define TIM1_CCER_CC2P_Pos 5U
#define TIM1_CCER_CC2P_Msk (0x1UL << TIM1_CCER_CC2P_Pos)
#define TIM1_CCER_CC2P TIM1_CCER_CC2P_Msk

#define TIM1_CCER_CC2E_Pos 4U
#define TIM1_CCER_CC2E_Msk (0x1UL << TIM1_CCER_CC2E_Pos)
#define TIM1_CCER_CC2E TIM1_CCER_CC2E_Msk

#define TIM1_CCER_CC1NP_Pos 3U
#define TIM1_CCER_CC1NP_Msk (0x1UL << TIM1_CCER_CC1NP_Pos)
#define TIM1_CCER_CC1NP TIM1_CCER_CC1NP_Msk

#define TIM1_CCER_CC1NE_Pos 2U
#define TIM1_CCER_CC1NE_Msk (0x1UL << TIM1_CCER_CC1NE_Pos)
#define TIM1_CCER_CC1NE TIM1_CCER_CC1NE_Msk

#define TIM1_CCER_CC1P_Pos 1U
#define TIM1_CCER_CC1P_Msk (0x1UL << TIM1_CCER_CC1P_Pos)
#define	TIM1_CCER_CC1P TIM1_CCER_CC1P_Msk

#define TIM1_CCER_CC1E_Pos 0U
#define TIM1_CCER_CC1E_Msk (0x1UL << TIM1_CCER_CC1E_Pos)
#define TIM1_CCER_CC1E TIM1_CCER_CC1E_Msk

/*CNT register.*/
#define TIM1_CNT_UIFCPY_Pos 31U
#define TIM1_CNT_UIFCPY_Msk (0x1UL << TIM1_CNT_UIFCPY_Pos)
#define TIM1_CNT_UIFCPY TIM1_CNT_UIFCPY_Msk

#define TIM1_CNT_Pos 0U
#define TIM1_CNT_Msk (0xFFFFUL << TIM1_CNT_Pos)
#define TIM1_CNT TIM1_CNT_Msk

/*PSC register.*/
#define TIM1_PSC_Pos 0U
#define TIM1_PSC_Msk (0xFFFFUL << TIM1_PSC_Pos)
#define TIM1_PSC TIM1_PSC_Msk

/*ARR register.*/
#define TIM1_ARR_Pos 0U
#define TIM1_ARR_Msk (0xFFFFUL << TIM1_ARR_Pos)
#define TIM1_ARR TIM1_ARR_Msk

/*RCR register.*/
#define TIM1_RCR_Pos 0U
#define TIM1_RCR_Msk (0xFFFFUL << TIM1_RCR_Pos)
#define TIM1_RCR TIM1_RCR_Msk

/*CCR1 register.*/
#define TIM1_CCR1_Pos 0U
#define TIM1_CCR1_Msk (0xFFFFUL << TIM1_CCR1_Pos)
#define TIM1_CCR1 TIM1_CCR1_Msk

/*CCR2 register.*/
#define TIM1_CCR2_Pos 0U
#define TIM1_CCR2_Msk (0xFFFFUL << TIM1_CCR2_Pos)
#define TIM1_CCR2 TIM1_CCR2_Msk

/*CCR3 register.*/
#define TIM1_CCR3_Pos 0U
#define TIM1_CCR3_Msk (0xFFFFUL << TIM1_CCR3_Pos)
#define TIM1_CCR3 TIM1_CCR3_Msk

/*CCR4 register.*/
#define TIM1_CCR4_Pos 0U
#define TIM1_CCR4_Msk (0xFFFFUL << TIM_CCR4_Pos)
#define TIM1_CCR4 TIM1_CCR4_Msk

/*BDTR register.*/
#define TIM1_BDTR_BK2P_Pos 25U
#define TIM1_BDTR_BK2P_Msk (0x1UL << TIM1_BDTR_BK2P_Pos)
#define TIM1_BDTR_BK2P TIM1_BDTR_BK2P_Msk

#define TIM1_BDTR_BK2E_Pos 24U
#define TIM1_BDTR_BK2E_Msk (0x1UL << TIM1_BDTR_BK2E_Pos)
#define TIM1_BDTR_BK2E TIM1_BDTR_BK2E_Msk

#define TIM1_BDTR_BK2F_Pos 20U
#define TIM1_BDTR_BK2F_Msk (0xFUL << TIM1_BDTR_BK2F_Pos)
#define TIM1_BDTR_BK2F TIM1_BDTR_BK2F_Msk

#define TIM1_BDTR_BKF_Pos 16U
#define TIM1_BDTR_BKF_Msk (0xFUL << TIM1_BDTR_BKF_Pos)
#define TIM1_BDTR_BKF TIM1_BDTR_BKF_Msk

#define TIM1_BDTR_MOE_Pos 15U
#define TIM1_BDTR_MOE_Msk (0x1UL << TIM1_BDTR_MOE_Pos)
#define TIM1_BDTR_MOE TIM1_BDTR_MOE_Msk

#define TIM1_BDTR_AOE_Pos 14U
#define TIM1_BDTR_AOE_Msk (0x1UL << TIM1_BDTR_AOE_Pos)
#define TIM1_BDTR_AOE TIM1_BDTR_AOE_Msk

#define TIM1_BDTR_BKP_Pos 13U
#define TIM1_BDTR_BKP_Msk (0x1UL << TIM1_BDTR_BKP_Pos)
#define TIM1_BDTR_BKP TIM1_BDTR_BKP_Msk

#define TIM1_BDTR_BKE_Pos 12U
#define TIM1_BDTR_BKE_Msk (0x1UL << TIM1_BDTR_BKE_Pos)
#define TIM1_BDTR_BKE TIM1_BDTR_BKE_Msk

#define TIM1_BDTR_OSSR_Pos 11U
#define TIM1_BDTR_OSSR_Msk (0x1UL << TIM1_BDTR_OSSR_Pos)
#define TIM1_BDTR_OSSR TIM1_BDTR_OSSR_Msk

#define TIM1_BDTR_OSSI_Pos 10U
#define TIM1_BDTR_OSSI_Msk (0x1UL << TIM1_BDTR_OSSI_Pos)
#define TIM1_BDTR_OSSI TIM1_BDTR_OSSI_Msk

#define TIM1_BDTR_LOCK_Pos 8U
#define TIM1_BDTR_LOCK_Msk (0x3UL << TIM1_BDTR_LOCK_Pos)
#define TIM1_BDTR_LOCK TIM1_BDTR_LOCK_Msk

#define TIM1_BDTR_DTG_Pos 0U
#define TIM1_BDTR_DTG_Msk (0xFFUL << TIM1_BDTR_DTG_Pos)
#define TIM1_BDTR_DTG TIM1_BDTR_DTG_Msk

/*DCR register.*/
#define TIM1_DCR_DBL_Pos 8U
#define TIM1_DCR_DBL_Msk (0x1FUL << TIM1_DCR_DBL_Pos)
#define TIM1_DCR_DBL TIM1_DCR_DBL_Msk

#define TIM1_DCR_DBA_Pos 0U
#define TIM1_DCR_DBA_Msk (0x1FUL << TIM1_DCR_DBA_Pos)
#define TIM1_DCR_DBA TIM1_DCR_DBA_Msk

/*OR1 register.*/
#define TIM1_OR1_TI1_RMP_Pos 4U
#define TIM1_OR1_TI1_RMP_Msk (0x1UL << TIM1_OR1_TI1_RMP_Pos)
#define TIM1_OR1_TI1_RMP TIM1_OR1_TI1_RMP_Msk

#define TIM1_OR1_ETR_ADC1_RMP_Pos 0U
#define TIM1_OR1_ETR_ADC1_RMP_Msk (0x3UL << TIM1_OR1_ETR_ADC1_RMP_Pos)
#define TIM1_OR1_ETR_ADC1_RMP TIM1_OR1_ETR_ADC1_RMP_Msk

/*CCMR3 register.*/
/*******Channel 6.*******/
/*Output Compare Mode.*/
#define TIM1_CCMR3_OC6CE_Pos 15U
#define TIM1_CCMR3_OC6CE_Msk (0x1UL << TIM1_CCMR3_OC6CE_Pos)
#define TIM1_CCMR3_OC6CE TIM1_CCMR3_OC6CE_Msk

#define TIM1_CCMR3_OC6M3_Pos 24U
#define TIM1_CCMR3_OC6M_Pos 12U
#define TIM1_CCMR3_OC6M_Msk ((0x1UL << TIM1_CCMR3_OC6M3_Pos) | (0x7UL << TIM1_CCMR3_OC6M_Pos))
#define TIM1_CCMR3_OC6M TIM1_CCMR3_OC6M_Msk

#define TIM1_CCMR3_OC6PE_Pos 11U
#define TIM1_CCMR3_OC6PE_Msk (0x1UL << TIM1_CCMR3_OC6PE_Pos)
#define TIM1_CCMR3_OC6PE TIM1_CCMR3_OC6PE_Msk

#define TIM1_CCMR3_OC6FE_Pos 10U
#define TIM1_CCMR3_OC6FE_Msk (0x1UL << TIM1_CCMR3_OC6FE_Pos)
#define TIM1_CCMR3_OC6FE TIM1_CCMR3_OC6FE_Msk

/*******Channel 5.*******/
/*Output Compare Mode.*/
#define TIM1_CCMR3_OC5CE_Pos 7U
#define TIM1_CCMR3_OC5CE_Msk (0x1UL << TIM1_CCMR3_OC5CE_Pos)
#define TIM1_CCMR3_OC5CE TIM1_CCMR3_OC5CE_Msk

#define TIM1_CCMR3_OC5M3_Pos 16U
#define TIM1_CCMR3_OC5M_Pos 4U
#define TIM1_CCMR3_OC5M_Msk ((0x1UL << TIM1_CCMR3_OC5M3_Pos) | (0x7UL << TIM1_CCMR3_OC5M_Pos))
#define TIM1_CCMR3_OC5M TIM1_CCMR3_OC5M_Msk

#define TIM1_CCMR3_OC5PE_Pos 3U
#define TIM1_CCMR3_OC5PE_Msk (0x1UL << TIM1_CCMR3_OC5PE_Pos)
#define TIM1_CCMR3_OC5PE TIM1_CCMR3_OC5PE_Msk

#define TIM1_CCMR3_OC5FE_Pos 2U
#define TIM1_CCMR3_OC5FE_Msk (0x1UL << TIM1_CCMR3_OC5FE_Pos)
#define TIM1_CCMR3_OC5FE TIM1_CCMR3_OC5FE_Msk

/*CCR5 register.*/
#define TIM1_CCR5_GC5C3_Pos 31U
#define TIM1_CCR5_GC5C3_Msk (0x1UL << TIM1_CCR5_GC5C3_Pos)
#define TIM1_CCR5_GC5C3 TIM1_CCR5_GC5C3_Msk

#define TIM1_CCR5_GC5C2_Pos 30U
#define TIM1_CCR5_GC5C2_Msk (0x1UL << TIM1_CCR5_GC5C2_Pos)
#define TIM1_CCR5_GC5C2 TIM1_CCR5_GC5C2_Msk

#define TIM1_CCR5_GC5C1_Pos 29U
#define TIM1_CCR5_GC5C1_Msk (0x1UL << TIM1_CCR5_GC5C1_Pos)
#define TIM1_CCR5_GC5C1 TIM1_CCR5_GC5C1_Msk

#define TIM1_CCR5_Pos 0U
#define TIM1_CCR5_Msk (0xFFFFUL << TIM1_CCR5_Pos)
#define TIM1_CCR5 TIM1_CCR5_Msk

/*CCR6 register.*/
#define TIM1_CCR6_Pos 0U
#define TIM1_CCR6_Msk (0xFFFFUL << TIM1_CCR6_Pos)
#define TIM1_CCR6 TIM1_CCR6_Msk

/*OR2 register.*/
#define TIM1_OR2_ETRSEL_Pos 14U
#define TIM1_OR2_ETRSEL_Msk (0x7UL << TIM1_OR2_ETRSEL_Pos)
#define TIM1_OR2_ETRSEL TIM1_OR2_ETRSEL_Msk
#define TIM1_OR2_ETRSEL_OR1 (0x0UL << TIM1_OR2_ETRSEL_Pos)
#define TIM1_OR2_ETRSEL_COMP1 (0x1UL << TIM1_OR2_ETRSEL_Pos)
#define TIM1_OR2_ETRSEL_COMP2 (0x2UL << TIM1_OR2_ETRSEL_Pos)

#define TIM1_OR2_BKCMP2P_Pos 11U
#define TIM1_OR2_BKCMP2P_Msk (0x1UL << TIM1_OR2_BKCMP2P_Pos)
#define TIM1_OR2_BKCMP2P TIM1_OR2_BKCMP2P_Msk

#define TIM1_OR2_BKCMP1P_Pos 10U
#define TIM1_OR2_BKCMP1P_Msk (0x1UL << TIM1_OR2_BKCMP1P_Pos)
#define TIM1_OR2_BKCMP1P TIM1_OR2_BKCMP1P_Msk

#define TIM1_OR2_BKINP_Pos 9U
#define TIM1_OR2_BKINP_Msk (0x1UL << TIM1_OR2_BKINP_Pos)
#define TIM1_OR2_BKINP TIM1_OR2_BKINP_Msk

#define TIM1_OR2_BKCMP2E_Pos 2U
#define TIM1_OR2_BKCMP2E_Msk (0x1UL << TIM1_OR2_BKCMP2E_Pos)
#define TIM1_OR2_BKCMP2E TIM1_OR2_BKCMP2E_Msk

#define TIM1_OR2_BKCMP1E_Pos 1U
#define TIM1_OR2_BKCMP1E_Msk (0x1UL << TIM1_OR2_BKCMP1E_Pos)
#define TIM1_OR2_BKCMP1E TIM1_OR2_BKCMP1E_Msk

#define TIM1_OR2_BKINE_Pos 0U
#define TIM1_OR2_BKINE_Msk (0x1UL << TIM1_OR2_BKINE_Pos)
#define TIM1_OR2_BKINE TIM1_OR2_BKINE_Msk

/*OR3 register.*/
#define TIM1_OR3_BK2CMP2P_Pos 11U
#define TIM1_OR3_BK2CMP2P_Msk (0x1UL << TIM1_OR3_BK2CMP2P_Pos)
#define TIM1_OR3_BK2CMP2P TIM1_OR3_BK2CMP2P_Msk

#define TIM1_OR3_BK2CMP1P_Pos 10U
#define TIM1_OR3_BK2CMP1P_Msk (0x1UL << TIM1_OR3_BK2CMP1P_Pos)
#define TIM1_OR3_BK2CMP1P TIM1_OR3_BK2CMP1P_Msk

#define TIM1_OR3_BK2INP_Pos 9U
#define TIM1_OR3_BK2INP_Msk (0x1UL << TIM1_OR3_BK2INP_Pos)
#define TIM1_OR3_BK2INP TIM1_OR3_BK2INP_Msk

#define TIM1_OR3_BK2CMP2E_Pos 2U
#define TIM1_OR3_BK2CMP2E_Msk (0x1UL << TIM1_OR3_BK2CMP2E_Pos)
#define TIM1_OR3_BK2CMP2E TIM1_OR3_BK2CMP2E_Msk

#define TIM1_OR3_BK2CMP1E_Pos 1U
#define TIM1_OR3_BK2CMP1E_Msk (0x1UL << TIM1_OR3_BK2CMP1E_Pos)
#define TIM1_OR3_BK2CMP1E TIM1_OR3_BK2CMP1E_Msk

#define TIM1_OR3_BK2INE_Pos 0U
#define TIM1_OR3_BK2INE_Msk (0x1UL << TIM1_OR3_BK2INE_Pos)
#define TIM1_OR3_BK2INE TIM1_OR3_BK2INE_Msk

/* ========================================================================= */
/* ============                      TIM2/3                     ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR1; /*!< TIM2/3 Control Register 1. */
	__IOM uint32_t CR2; /*!< TIM2/3 Control Register 2. */
	__IOM uint32_t SMCR; /*!< TIM2/3 Slave Mode Control Register. */
	__IOM uint32_t DIER; /*!< TIM2/3 DMA/Interrupt Enable Register. */ 
	__IOM uint32_t SR; /*!< TIM2/3 Status Register. */
	__OM uint32_t EGR; /*!< TIM2/3 Event Generation Register. */
	__IOM uint32_t CCMR1; /*!< TIM2/3 Capture/Compare Mode Register 1. */
	__IOM uint32_t CCMR2; /*!< TIM2/3 Capture/Compare Mode Register 2. */
	__IOM uint32_t CCER; /*!< TIM2/3 Capture/Compare Enable Register. */
	__IOM uint32_t CNT; /*!< TIM2/3 Counter. */
	__IOM uint32_t PSC; /*!< TIM2/3 Prescaler. */
	__IOM uint32_t ARR; /*!< TIM2/3 Auto-Reload Register. */
	uint32_t RESERVED0;
	__IOM uint32_t CCR1; /*!< TIM2/3 Capture/Compare Register 1. */
	__IOM uint32_t CCR2; /*!< TIM2/3 Capture/Compare Register 2. */
	__IOM uint32_t CCR3; /*!< TIM2/3 Capture/Compare Register 3. */
	__IOM uint32_t CCR4; /*!< TIM2/3 Capture/Compare Register 4. */
	uint32_t RESERVED1;
	__IOM uint32_t DCR; /*!< TIM2/3 DMA Control Register. */
	__IOM uint32_t DMAR; /*!< TIM2/3 DMA address for full transfer. */
	__IOM uint32_t OR1; /*!< TIM2/3 Option Register 1. */
	uint32_t RESERVED2[3];
	__IOM uint32_t OR2; /*!< TIM2/3 Option Register 2. */
}STM32L4xx_TIM2_3_TypeDef;
 
/*CR1 register.*/
#define TIM2_3_CR1_UIFREMAP_Pos 11U
#define TIM2_3_CR1_UIFREMAP_Msk (0x1UL << TIM2_3_CR1_UIFREMAP_Pos)
#define TIM2_3_CR1_UIFREMAP TIM2_3_CR1_UIFREMAP_Msk

#define TIM2_3_CR1_CKD_Pos 8U
#define TIM2_3_CR1_CKD_Msk (0x3UL << TIM2_3_CR1_CKD_Pos)
#define TIM2_3_CR1_CKD TIM2_3_CR1_CKD_Msk

#define TIM2_3_CR1_ARPE_Pos 7U
#define TIM2_3_CR1_ARPE_Msk (0x1UL << TIM2_3_CR1_ARPE_Pos)
#define TIM2_3_CR1_ARPE TIM2_3_CR1_ARPE_Msk

#define TIM2_3_CR1_CMS_Pos 5U
#define TIM2_3_CR1_CMS_Msk (0x3UL << TIM2_3_CR1_CMS_Pos)
#define TIM2_3_CR1_CMS TIM2_3_CR1_CMS_Msk

#define TIM2_3_CR1_DIR_Pos 4U
#define TIM2_3_CR1_DIR_Msk (0x1UL << TIM2_3_CR1_DIR_Pos)
#define TIM2_3_CR1_DIR TIM2_3_CR1_DIR_Msk

#define TIM2_3_CR1_OPM_Pos 3U
#define TIM2_3_CR1_OPM_Msk (0x1UL << TIM2_3_CR1_OPM_Pos)
#define TIM2_3_CR1_OPM TIM2_3_CR1_OPM_Msk

#define TIM2_3_CR1_URS_Pos 2U
#define TIM2_3_CR1_URS_Msk (0x1UL << TIM2_3_CR1_URS_Pos)
#define TIM2_3_CR1_URS TIM2_3_CR1_URS_Msk

#define TIM2_3_CR1_UDIS_Pos 1U
#define TIM2_3_CR1_UDIS_Msk (0x1UL << TIM2_3_CR1_UDIS_Pos)
#define TIM2_3_CR1_UDIS TIM2_3_CR1_UDIS_Msk

#define TIM2_3_CR1_CEN_Pos 0U
#define TIM2_3_CR1_CEN_Msk (0x1UL << TIM2_3_CR1_CEN_Pos)
#define TIM2_3_CR1_CEN TIM2_3_CR1_CEN_Msk

/*CR2 register.*/
#define TIM2_3_CR2_TI1S_Pos 7U
#define TIM2_3_CR2_TI1S_Msk (0x1UL << TIM2_3_CR2_TI1S_Pos)
#define TIM2_3_CR2_TI1S TIM2_3_CR2_TI1S_Msk

#define TIM2_3_CR2_MMS_Pos 4U
#define TIM2_3_CR2_MMS_Msk (0x7UL << TIM2_3_CR2_MMS_Pos)
#define TIM2_3_CR2_MMS TIM2_3_CR2_MMS_Msk

#define TIM2_3_CR2_CCDS_Pos 3U
#define TIM2_3_CR2_CCDS_Msk (0x1UL << TIM2_3_CR2_CCDS_Pos)
#define TIM2_3_CR2_CCDS TIM2_3_CR2_CCDS_Msk

/*SMCR register.*/
#define TIM2_3_SMCR_ETP_Pos 15U
#define TIM2_3_SMCR_ETP_Msk (0x1UL << TIM2_3_SMCR_ETP_Pos)
#define TIM2_3_SMCR_ETP TIM2_3_SMCR_ETP_Msk

#define TIM2_3_SMCR_ECE_Pos 14U
#define TIM2_3_SMCR_ECE_Msk (0x1UL << TIM2_3_SMCR_ECE_Pos)
#define TIM2_3_SMCR_ECE TIM2_3_SMCR_ECE_Msk

#define TIM2_3_SMCR_ETPS_Pos 12U
#define TIM2_3_SMCR_ETPS_Msk (0x3UL << TIM2_3_SMCR_ETPS_Pos)
#define TIM2_3_SMCR_ETPS TIM2_3_SMCR_ETPS_Msk

#define TIM2_3_SMCR_ETF_Pos 8U
#define TIM2_3_SMCR_ETF_Msk (0xFUL << TIM2_3_SMCR_ETF_Pos)
#define TIM2_3_SMCR_ETF TIM2_3_SMCR_ETF_Msk

#define TIM2_3_SMCR_MSM_Pos 7U
#define TIM2_3_SMCR_MSM_Msk (0x1UL << TIM2_3_SMCR_MSM_Pos)
#define TIM2_3_SMCR_MSM TIM2_3_SMCR_MSM_Msk

#define TIM2_3_SMCR_TS_Pos 4U
#define TIM2_3_SMCR_TS_Msk (0x7UL << TIM2_3_SMCR_TS_Pos)

#define TIM2_3_SMCR_OCCS_Pos 3U
#define TIM2_3_SMCR_OCCS_Msk (0x1UL << TIM2_3_SMCR_OCCS_Pos)
#define TIM2_3_SMCR_TS TIM2_3_SMCR_TS_Msk

#define TIM2_3_SMCR_SMS3_Pos 16U
#define TIM2_3_SMCR_SMS_Pos 0U
#define TIM2_3_SMCR_SMS_Msk ((0x1UL << TIM2_3_SMCR_SMS3_Pos) | (0x7UL << TIM2_3_SMCR_SMS_Pos))

/*DIER register.*/
#define TIM2_3_DIER_TDE_Pos 14U
#define TIM2_3_DIER_TDE_Msk (0x1UL << TIM2_3_DIER_TDE_Pos)
#define TIM2_3_DIER_TDE TIM2_3_DIER_TDE_Msk

#define TIM2_3_DIER_CC4DE_Pos 12U
#define TIM2_3_DIER_CC4DE_Msk (0x1UL << TIM2_3_DIER_CC4DE_Pos)
#define TIM2_3_DIER_CC4DE TIM2_3_DIER_CC4DE_Msk

#define TIM2_3_DIER_CC3DE_Pos 11U
#define TIM2_3_DIER_CC3DE_Msk (0x1UL << TIM2_3_DIER_CC3DE_Pos)
#define TIM2_3_DIER_CC3DE TIM2_3_DIER_CC3DE_Msk

#define TIM2_3_DIER_CC2DE_Pos 10U
#define TIM2_3_DIER_CC2DE_Msk (0x1UL << TIM2_3_DIER_CC2DE_Pos)
#define TIM2_3_DIER_CC2DE TIM2_3_DIER_CC2DE_Msk

#define TIM2_3_DIER_CC1DE_Pos 9U
#define TIM2_3_DIER_CC1DE_Msk (0x1UL << TIM2_3_DIER_CC1DE_Pos)
#define TIM2_3_DIER_CC1DE TIM2_3_DIER_CC1DE_Msk

#define TIM2_3_DIER_UDE_Pos 8U
#define TIM2_3_DIER_UDE_Msk (0x1UL << TIM2_3_DIER_UDE_Pos)
#define TIM2_3_DIER_UDE TIM2_3_DIER_UDE_Msk

#define TIM2_3_DIER_TIE_Pos 6U
#define TIM2_3_DIER_TIE_Msk (0x1UL << TIM2_3_DIER_TIE_Pos)
#define TIM2_3_DIER_TIE TIM2_3_DIER_TIE_Msk

#define TIM2_3_DIER_CC4IE_Pos 4U
#define TIM2_3_DIER_CC4IE_Msk (0x1UL << TIM2_3_DIER_CC4IE_Pos)
#define TIM2_3_DIER_CC4IE TIM2_3_DIER_CC4IE_Msk

#define TIM2_3_DIER_CC3IE_Pos 3U
#define TIM2_3_DIER_CC3IE_Msk (0x1UL << TIM2_3_DIER_CC3IE_Pos)
#define TIM2_3_DIER_CC3IE TIM2_3_DIER_CC3IE_Msk

#define TIM2_3_DIER_CC2IE_Pos 2U
#define TIM2_3_DIER_CC2IE_Msk (0x1UL << TIM2_3_DIER_CC2IE_Pos)
#define TIM2_3_DIER_CC2IE TIM2_3_DIER_CC2IE_Msk

#define TIM2_3_DIER_CC1IE_Pos 1U
#define TIM2_3_DIER_CC1IE_Msk (0x1UL << TIM2_3_DIER_CC1IE_Pos)
#define TIM2_3_DIER_CC1IE TIM2_3_DIER_CC1IE_Msk

#define TIM2_3_DIER_UIE_Pos 0U
#define TIM2_3_DIER_UIE_Msk (0x1UL << TIM2_3_DIER_UIE_Pos)
#define TIM2_3_DIER_UIE TIM2_3_DIER_UIE_Msk

/*SR register.*/
#define TIM2_3_SR_CC4OF_Pos 12U
#define TIM2_3_SR_CC4OF_Msk (0x1UL << TIM2_3_SR_CC4OF_Pos)
#define TIM2_3_SR_CC4OF TIM2_3_SR_CC4OF_Msk

#define TIM2_3_SR_CC3OF_Pos 11U
#define TIM2_3_SR_CC3OF_Msk (0x1UL << TIM2_3_SR_CC3OF_Pos)
#define TIM2_3_SR_CC3OF TIM2_3_SR_CC3OF_Msk

#define TIM2_3_SR_CC2OF_Pos 10U
#define TIM2_3_SR_CC2OF_Msk (0x1UL << TIM2_3_SR_CC2OF_Pos)
#define TIM2_3_SR_CC2OF TIM2_3_SR_CC2OF_Msk

#define TIM2_3_SR_CC1OF_Pos 9U
#define TIM2_3_SR_CC1OF_Msk (0x1UL << TIM2_3_SR_CC1OF_Pos)
#define TIM2_3_SR_CC1OF TIM2_3_SR_CC1OF_Msk

#define TIM2_3_SR_TIF_Pos 6U
#define TIM2_3_SR_TIF_Msk (0x1UL << TIM2_3_SR_TIF_Pos)
#define TIM2_3_SR_TIF TIM2_3_SR_TIF_Msk

#define TIM2_3_SR_CC4IF_Pos 4U
#define TIM2_3_SR_CC4IF_Msk (0x1UL << TIM2_3_SR_CC4IF_Pos)
#define TIM2_3_SR_CC4IF TIM2_3_SR_CC4IF_Msk

#define TIM2_3_SR_CC3IF_Pos 3U
#define TIM2_3_SR_CC3IF_Msk (0x1UL << TIM2_3_SR_CC3IF_Pos)
#define TIM2_3_SR_CC3IF TIM2_3_SR_CC3IF_Msk

#define TIM2_3_SR_CC2IF_Pos 2U
#define TIM2_3_SR_CC2IF_Msk (0x1UL << TIM2_3_SR_CC2IF_Pos)
#define TIM2_3_SR_CC2IF TIM2_3_SR_CC2IF_Msk

#define TIM2_3_SR_CC1IF_Pos 1U
#define TIM2_3_SR_CC1IF_Msk (0x1UL << TIM2_3_SR_CC1IF_Pos)
#define TIM2_3_SR_CC1IF TIM2_3_SR_CC1IF_Msk

#define TIM2_3_SR_UIF_Pos 0U
#define TIM2_3_SR_UIF_Msk (0x1UL << TIM2_3_SR_UIF_Pos)
#define TIM2_3_SR_UIF TIM2_3_SR_UIF_Msk

/*EGR register.*/
#define TIM2_3_EGR_TG_Pos 6U
#define TIM2_3_EGR_TG_Msk (0x1UL << TIM2_3_EGR_TG_Pos)
#define TIM2_3_EGR_TG TIM2_3_EGR_TG_Msk

#define TIM2_3_EGR_CC4G_Pos 4U
#define TIM2_3_EGR_CC4G_Msk (0x1UL << TIM2_3_EGR_CC4G_Pos)
#define TIM2_3_EGR_CC4G TIM2_3_EGR_CC4G_Msk

#define TIM2_3_EGR_CC3G_Pos 3U
#define TIM2_3_EGR_CC3G_Msk (0x1UL << TIM2_3_EGR_CC3G_Pos)
#define TIM2_3_EGR_CC3G TIM2_3_EGR_CC3G_Msk

#define TIM2_3_EGR_CC2G_Pos 2U
#define TIM2_3_EGR_CC2G_Msk (0x1UL << TIM2_3_EGR_CC2G_Pos)
#define TIM2_3_EGR_CC2G TIM2_3_EGR_CC2G_Msk

#define TIM2_3_EGR_CC1G_Pos 1U
#define TIM2_3_EGR_CC1G_Msk (0x1UL << TIM2_3_EGR_CC1G_Pos)
#define TIM2_3_EGR_CC1G TIM2_3_EGR_CC1G_Msk

#define TIM2_3_EGR_UG_Pos 0U
#define TIM2_3_EGR_UG_Msk (0x1UL << TIM2_3_EGR_UG_Pos)
#define TIM2_3_EGR_UG  TIM2_3_EGR_UG_Msk

/*CCMR1 register.*/
/*******Channel 2*******/
#define TIM2_3_CCMR1_CC2S_Pos 8U
#define TIM2_3_CCMR1_CC2S_Msk (0x3UL << TIM2_3_CCMR1_CC2S_Pos)
#define TIM2_3_CCMR1_CC2S TIM2_3_CCMR1_CC2S_Msk

/*Input Capture Mode.*/
#define TIM2_3_CCMR1_IC2F_Pos 12U
#define TIM2_3_CCMR1_IC2F_Msk (0xFUL << TIM2_3_CCMR1_IC2F_Pos)
#define TIM2_3_CCMR1_IC2F TIM2_3_CCMR1_IC2F_Msk

#define TIM2_3_CCMR1_IC2PSC_Pos 10U
#define TIM2_3_CCMR1_IC2PSC_Msk (0x3UL << TIM2_3_CCMR1_IC2PSC_Pos)
#define TIM2_3_CCMR1_IC2PSC TIM2_3_CCMR1_IC2PSC_Msk

#define TIM2_3_CCMR1_CC2S_Pos 8U
#define TIM2_3_CCMR1_CC2S_Msk (0x3UL << TIM2_3_CCMR1_CC2S_Pos)
#define TIM2_3_CCMR1_CC2S TIM2_3_CCMR1_CC2S_Msk

/*Output Compare Mode.*/
#define TIM2_3_CCMR1_OC2CE_Pos 15U
#define TIM2_3_CCMR1_OC2CE_Msk (0x1UL << TIM2_3_CCMR1_OC2CE_Pos)

#define TIM2_3_CCMR1_OC2M3_Pos 24U
#define TIM2_3_CCMR1_OC2M_Pos 12U
#define TIM2_3_CCMR1_OC2M_Msk ((0x1UL << TIM2_3_CCMR1_OC2M3_Pos)| (0x7UL << TIM2_3_CCMR1_OC2M_Pos))

#define TIM2_3_CCMR1_OC2PE_Pos 11U
#define TIM2_3_CCMR1_OC2PE_Msk (0x1UL << TIM2_3_CCMR1_OC2PE_Pos)

#define TIM2_3_CCMR1_OC2FE_Pos 10U
#define TIM2_3_CCMR1_OC2FE_Msk (0x1UL << TIM2_3_CCMR1_OC2FE_Pos)

/*******Channel 1*******/
#define TIM2_3_CCMR1_CC1S_Pos 0U
#define TIM2_3_CCMR1_CC1S_Msk (0x3UL << TIM2_3_CCMR1_CC1S_Pos)
#define TIM2_3_CCMR1_CC1S TIM2_3_CCMR1_CC1S_Msk

/*Input Capture Mode.*/
#define TIM2_3_CCMR1_IC1F_Pos 4U
#define TIM2_3_CCMR1_IC1F_Msk (0xFUL << TIM2_3_CCMR1_IC1F_Pos)
#define TIM2_3_CCMR1_IC1F TIM2_3_CCMR1_IC1F_Msk

#define TIM2_3_CCMR1_IC1PSC_Pos 2U
#define TIM2_3_CCMR1_IC1PSC_Msk (0x3UL << TIM2_3_CCMR1_IC1PSC_Pos)
#define TIM2_3_CCMR1_IC1PSC TIM2_3_CCMR1_IC1PSC_Msk

#define TIM2_3_CCMR1_CC1S_Pos 0U
#define TIM2_3_CCMR1_CC1S_Msk (0x3UL << TIM2_3_CCMR1_CC1S_Pos)
#define TIM2_3_CCMR1_CC1S TIM2_3_CCMR1_CC1S_Msk

/*Output Compare Mode.*/
#define TIM2_3_CCMR1_OC1CE_Pos 7U
#define TIM2_3_CCMR1_OC1CE_Msk (0x1UL << TIM2_3_CCMR1_OC1CE_Pos)
#define TIM2_3_CCMR1_OC1CE TIM2_3_CCMR1_OC1CE_Msk

#define TIM2_3_CCMR1_OC1M3_Pos 16U
#define TIM2_3_CCMR1_OC1M_Pos 4U
#define TIM2_3_CCMR1_OC1M_Msk ((0x1UL << TIM2_3_CCMR1_OC1M3_Pos) | (0x7UL << TIM2_3_CCMR1_OC1M_Pos))
#define TIM2_3_CCMR1_OC1M TIM2_3_CCMR1_OC1M_Msk

#define TIM2_3_CCMR1_OC1PE_Pos 3U
#define TIM2_3_CCMR1_OC1PE_Msk (0x1UL << TIM2_3_CCMR1_OC1PE_Pos)
#define TIM2_3_CCMR1_OC1PE TIM2_3_CCMR1_OC1PE_Msk

#define TIM2_3_CCMR1_OC1FE_Pos 2U
#define TIM2_3_CCMR1_OC1FE_Msk (0x1UL << TIM2_3_CCMR1_OC1FE_Pos)
#define TIM2_3_CCMR1_OC1FE TIM2_3_CCMR1_OC1FE_Msk

/*CCMR2 register.*/
/*******Channel 4*******/
#define TIM2_3_CCMR2_CC4S_Pos 8U
#define TIM2_3_CCMR2_CC4S_Msk (0x3UL << TIM2_3_CCMR2_CC4S_Pos)
#define TIM2_3_CCMR2_CC4S TIM2_3_CCMR2_CC4S_Msk

/*Input Capture Mode.*/
#define TIM2_3_CCMR2_IC4F_Pos 12U
#define TIM2_3_CCMR2_IC4F_Msk (0xFUL << TIM2_3_CCMR2_IC4F_Pos)
#define TIM2_3_CCMR2_IC4F TIM2_3_CCMR2_IC4F_Msk

#define TIM2_3_CCMR2_IC4PSC_Pos 10U
#define TIM2_3_CCMR2_IC4PSC_Msk (0x3UL << TIM2_3_CCMR2_IC4PSC_Pos)
#define TIM2_3_CCMR2_IC4PSC TIM2_3_CCMR2_IC4PSC_Msk

#define TIM2_3_CCMR2_CC4S_Pos 8U
#define TIM2_3_CCMR2_CC4S_Msk (0x3UL << TIM2_3_CCMR2_CC4S_Pos)
#define TIM2_3_CCMR2_CC4S TIM2_3_CCMR2_CC4S_Msk

/*Output Compare Mode.*/
#define TIM2_3_CCMR2_OC4CE_Pos 15U
#define TIM2_3_CCMR2_OC4CE_Msk (0x1UL << TIM2_3_CCMR2_OC4CE_Pos)

#define TIM2_3_CCMR2_OC4M3_Pos 24U
#define TIM2_3_CCMR2_OC4M_Pos 12U
#define TIM2_3_CCMR2_OC4M_Msk ((0x1UL << TIM2_3_CCMR2_OC4M3_Pos)| (0x7UL << TIM2_3_CCMR2_OC4M_Pos))

#define TIM2_3_CCMR2_OC4PE_Pos 11U
#define TIM2_3_CCMR2_OC4PE_Msk (0x1UL << TIM2_3_CCMR2_OC4PE_Pos)

#define TIM2_3_CCMR2_OC4FE_Pos 10U
#define TIM2_3_CCMR2_OC4FE_Msk (0x1UL << TIM2_3_CCMR2_OC4FE_Pos)

/*******Channel 3*******/
#define TIM2_3_CCMR2_CC3S_Pos 0U
#define TIM2_3_CCMR2_CC3S_Msk (0x3UL << TIM2_3_CCMR2_CC3S_Pos)
#define TIM2_3_CCMR2_CC3S TIM2_3_CCMR2_CC3S_Msk

/*Input Capture Mode.*/
#define TIM2_3_CCMR2_IC3F_Pos 4U
#define TIM2_3_CCMR2_IC3F_Msk (0xFUL << TIM2_3_CCMR2_IC3F_Pos)
#define TIM2_3_CCMR2_IC3F TIM2_3_CCMR2_IC3F_Msk

#define TIM2_3_CCMR2_IC3PSC_Pos 2U
#define TIM2_3_CCMR2_IC3PSC_Msk (0x3UL << TIM2_3_CCMR2_IC3PSC_Pos)
#define TIM2_3_CCMR2_IC3PSC TIM2_3_CCMR2_IC3PSC_Msk

#define TIM2_3_CCMR2_CC3S_Pos 0U
#define TIM2_3_CCMR2_CC3S_Msk (0x3UL << TIM2_3_CCMR2_CC3S_Pos)
#define TIM2_3_CCMR2_CC3S TIM2_3_CCMR2_CC3S_Msk

/*Output Compare Mode.*/
#define TIM2_3_CCMR2_OC3CE_Pos 7U
#define TIM2_3_CCMR2_OC3CE_Msk (0x1UL << TIM2_3_CCMR2_OC3CE_Pos)
#define TIM2_3_CCMR2_OC3CE TIM2_3_CCMR2_OC3CE_Msk

#define TIM2_3_CCMR2_OC3M3_Pos 16U
#define TIM2_3_CCMR2_OC3M_Pos 4U
#define TIM2_3_CCMR2_OC3M_Msk ((0x1UL << TIM2_3_CCMR2_OC3M3_Pos) | (0x7UL << TIM2_3_CCMR2_OC3M_Pos))
#define TIM2_3_CCMR2_OC3M TIM2_3_CCMR2_OC3M_Msk

#define TIM2_3_CCMR2_OC3PE_Pos 3U
#define TIM2_3_CCMR2_OC3PE_Msk (0x1UL << TIM2_3_CCMR2_OC3PE_Pos)
#define TIM2_3_CCMR2_OC3PE TIM2_3_CCMR2_OC3PE_Msk

#define TIM2_3_CCMR2_OC3FE_Pos 2U
#define TIM2_3_CCMR2_OC3FE_Msk (0x1UL << TIM2_3_CCMR2_OC3FE_Pos)
#define TIM2_3_CCMR2_OC3FE TIM2_3_CCMR2_OC3FE_Msk

/*CCER register.*/
#define TIM2_3_CCER_CC4NP_Pos 15U
#define TIM2_3_CCER_CC4NP_Msk (0x1UL << TIM2_3_CCER_CC4NP_Pos)
#define TIM2_3_CCER_CC4NP TIM2_3_CCER_CC4NP_Msk

#define TIM2_3_CCER_CC4P_Pos 13U
#define TIM2_3_CCER_CC4P_Msk (0x1UL << TIM2_3_CCER_CC4P_Pos)
#define TIM2_3_CCER_CC4P TIM2_3_CCER_CC4P_Msk

#define TIM2_3_CCER_CC4E_Pos 12U
#define TIM2_3_CCER_CC4E_Msk (0x1UL << TIM2_3_CCER_CC4E_Pos)
#define TIM2_3_CCER_CC4E TIM2_3_CCER_CC4E_Msk

#define TIM2_3_CCER_CC3NP_Pos 11U
#define TIM2_3_CCER_CC3NP_Msk (0x1UL << TIM2_3_CCER_CC3NP_Pos)
#define TIM2_3_CCER_CC3NP TIM2_3_CCER_CC3NP_Msk

#define TIM2_3_CCER_CC3P_Pos 9U
#define TIM2_3_CCER_CC3P_Msk (0x1UL << TIM2_3_CCER_CC3P_Pos)
#define TIM2_3_CCER_CC3P TIM2_3_CCER_CC3P_Msk

#define TIM2_3_CCER_CC3E_Pos 8U
#define TIM2_3_CCER_CC3E_Msk (0x1UL << TIM2_3_CCER_CC3E_Pos)
#define TIM2_3_CCER_CC3E TIM2_3_CCER_CC3E_Msk

#define TIM2_3_CCER_CC2NP_Pos 7U
#define TIM2_3_CCER_CC2NP_Msk (0x1UL << TIM2_3_CCER_CC2NP_Pos)
#define TIM2_3_CCER_CC2NP TIM2_3_CCER_CC2NP_Msk

#define TIM2_3_CCER_CC2P_Pos 5U
#define TIM2_3_CCER_CC2P_Msk (0x1UL << TIM2_3_CCER_CC2P_Pos)
#define TIM2_3_CCER_CC2P TIM2_3_CCER_CC2P_Msk

#define TIM2_3_CCER_CC2E_Pos 4U
#define TIM2_3_CCER_CC2E_Msk (0x1UL << TIM2_3_CCER_CC2E_Pos)
#define TIM2_3_CCER_CC2E TIM2_3_CCER_CC2E_Msk

#define TIM2_3_CCER_CC1NP_Pos 3U
#define TIM2_3_CCER_CC1NP_Msk (0x1UL << TIM2_3_CCER_CC1NP_Pos)
#define TIM2_3_CCER_CC1NP TIM2_3_CCER_CC1NP_Msk

#define TIM2_3_CCER_CC1P_Pos 1U
#define TIM2_3_CCER_CC1P_Msk (0x1UL << TIM2_3_CCER_CC1P_Pos)
#define TIM2_3_CCER_CC1P TIM2_3_CCER_CC1P_Msk

#define TIM2_3_CCER_CC1E_Pos 0U
#define TIM2_3_CCER_CC1E_Msk (0x1UL << TIM2_3_CCER_CC1E_Pos)
#define TIM2_3_CCER_CC1E TIM2_3_CCER_CC1E_Msk

/*CNT register.*/
#define TIM2_3_CNT_UIFCPY_Pos 31U
#define TIM2_3_CNT_UIFCPY_Msk (0x1UL << TIM2_3_CNT_UIFCPY_Pos)
#define TIM2_3_CNT_UIFCPY TIM2_3_CNT_UIFCPY_Msk

/*PSC register.*/
#define TIM2_3_PSC_Pos 0U
#define TIM2_3_PSC_Msk (0xFFFFUL << TIM2_3_PSC_Pos)
#define TIM2_3_PSC TIM2_3_PSC_Msk

/*DCR register.*/
#define TIM2_3_DCR_DBL_Pos 8U
#define TIM2_3_DCR_DBL_Msk (0x1FUL << TIM2_3_DCR_DBL_Pos)
#define TIM2_3_DCR_DBL TIM2_3_DCR_DBL_Msk

#define TIM2_3_DCR_DBA_Pos 0U
#define TIM2_3_DCR_DBA_Msk (0x1FUL << TIM2_3_DCR_DBA_Pos)
#define TIM2_3_DCR_DBA TIM2_3_DCR_DBA_Msk

/*DMAR register.*/
#define TIM2_3_DMAR_DMAB_Pos 0U
#define TIM2_3_DMAR_DMAB_Msk (0xFFFFUL << TIM2_3_DMAR_DMAB_Pos)
#define TIM2_3_DMAR_DMAB TIM2_3_DMAR_DMAB_Msk
/*OR1 register.*/
#define TIM2_3_OR1_TI4_RMP_Pos 2U
#define TIM2_3_OR1_TI4_RMP_Msk (0x3UL << TIM2_3_OR1_TI4_RMP_Pos)
#define TIM2_3_OR1_TI4_RMP TIM2_3_OR1_TI4_RMP_Msk

#define TIM2_3_OR1_ETR1_RMP_Pos 1U
#define TIM2_3_OR1_ETR1_RMP_Msk (0x1UL << TIM2_3_OR1_ETR1_RMP_Pos)
#define TIM2_3_OR1_ETR1_RMP TIM2_3_OR1_ETR1_RMP_Msk

#define TIM2_3_OR1_ITR1_RMP_Pos 0U
#define TIM2_3_OR1_ITR1_RMP_Msk (0x1UL << TIM2_3_OR1_ITR1_RMP_Pos)
#define TIM2_3_OR1_ITR1_RMP TIM2_3_OR1_ITR1_RMP_Msk

/*OR2 register.*/
#define TIM2_3_OR2_ETRSEL2_Pos 16U
#define TIM2_3_OR2_ETRSEL2_Msk (0x1UL << TIM2_3_OR2_ETRSEL2_Pos)
#define TIM2_3_OR2_ETRSEL2 TIM2_3_OR2_ETRSEL2_Msk

#define TIM2_3_OR2_ETRSEL_Pos 14U
#define TIM2_3_OR2_ETRSEL_Msk (0x3UL << TIM2_3_OR2_ETRSEL_Pos)
#define TIM2_3_OR2_ETRSEL TIM2_3_OR2_ETRSEL_Msk

/* ========================================================================= */
/* ============                      TIM15                      ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR1; /*!< TIM15 Control Register 1. */
	__IOM uint32_t CR2; /*!< TIM15 Control Register 2. */
	__IOM uint32_t SMCR; /*!< TIM15 Slave Mode Control Register. */
	__IOM uint32_t DIER; /*!< TIM15 DMA/Interrupt Enable Register. */
	__IOM uint32_t SR; /*!< TIM15 Status Register. */
	__OM uint32_t EGR; /*!< TIM15 Event Generation Register. */
	__IOM uint32_t CCMR1; /*!< TIM15 Capture/Compare Mode Register 1. */
	uint32_t RESERVED0;
	__IOM uint32_t CCER; /*!< TIM15 Capture/Compare Enable Register. */
	__IOM uint32_t CNT; /*!< TIM15 Counter. */
	__IOM uint32_t PSC; /*!< TIM15 Prescaler. */
	__IOM uint32_t ARR; /*!< TIM15 Auto-Reload Register. */
	__IOM uint32_t RCR; /*!< TIM15 Repetition Counter Register. */
	__IOM uint32_t CCR1; /*!< TIM15 Capture/Compare Register 1. */
	__IOM uint32_t CCR2; /*!< TIM15 Capture/Compare Register 1. */
	uint32_t RESERVED1[2];
	__IOM uint32_t BDTR; /*!< TIM15 Break and Dead Time Register. */
	__IOM uint32_t DCR; /*!< TIM15 DMA Control Register. */
	__IOM uint32_t DMAR; /*!< TIM15 DMA address for full transfer. */
	__IOM uint32_t OR1; /*!< TIM15 Option Register 1. */
	uint32_t RESERVED2[3];
	__IOM uint32_t OR2; /*!< TIM15 Option Register 2. */
}STM32L4xx_TIM15_TypeDef;

/*CR1 register.*/
#define TIM15_CR1_UIFREMAP_Pos 11U
#define TIM15_CR1_UIFREMAP_Msk (0x1UL << TIM15_CR1_UIFREMAP_Pos)
#define TIM15_CR1_UIFREMAP TIM15_CR1_UIFREMAP_Msk

#define TIM15_CR1_CKD_Pos 8U
#define TIM15_CR1_CKD_Msk (0x3UL << TIM15_CR1_CKD_Pos)
#define TIM15_CR1_CKD TIM15_CR1_CKD_Msk

#define TIM15_CR1_ARPE_Pos 7U
#define TIM15_CR1_ARPE_Msk (0x1UL << TIM15_CR1_ARPE_Pos)
#define TIM15_CR1_ARPE TIM15_CR1_ARPE_Msk

#define TIM15_CR1_OPM_Pos 3U
#define TIM15_CR1_OPM_Msk (0x1UL << TIM15_CR1_OPM_Pos)
#define TIM15_CR1_OPM TIM15_CR1_OPM_Msk

#define TIM15_CR1_URS_Pos 2U
#define TIM15_CR1_URS_Msk (0x1UL << TIM15_CR1_URS_Pos)
#define TIM15_CR1_URS TIM15_CR1_URS_Msk

#define TIM15_CR1_UDIS_Pos 1U
#define TIM15_CR1_UDIS_Msk (0x1UL << TIM15_CR1_UDIS_Pos)
#define TIM15_CR1_UDIS TIM15_CR1_UDIS_Msk

#define TIM15_CR1_CEN_Pos 0U
#define TIM15_CR1_CEN_Msk (0x1UL << TIM15_CR1_CEN_Pos)
#define TIM15_CR1_CEN TIM15_CR1_CEN_Msk

/*CR2 register.*/
#define TIM15_CR2_OIS2_Pos 10U
#define TIM15_CR2_OIS2_Msk (0x1UL << TIM15_CR2_OIS2_Pos)
#define TIM15_CR2_OIS2 TIM15_CR2_OIS2_Msk

#define TIM15_CR2_OIS1N_Pos 9U
#define TIM15_CR2_OIS1N_Msk (0x1UL << TIM15_CR2_OIS1N_Pos)
#define TIM15_CR2_OIS1N TIM15_CR2_OIS1N_Msk

#define TIM15_CR2_OIS1_Pos 8U
#define TIM15_CR2_OIS1_Msk (0x1UL << TIM15_CR2_OIS1_Pos)
#define TIM15_CR2_OIS1 TIM15_CR2_OIS1_Msk

#define TIM15_CR2_TI1S_Pos 7U
#define TIM15_CR2_TI1S_Msk (0x1UL << TIM15_CR2_TI1S_Pos)
#define TIM15_CR2_TI1S TIM15_CR2_TI1S_Msk

#define TIM15_CR2_MMS_Pos 4U
#define TIM15_CR2_MMS_Msk (0x7UL << TIM15_CR2_MMS_Pos)
#define TIM15_CR2_MMS TIM15_CR2_MMS_Msk

#define TIM15_CR2_CCDS_Pos 3U
#define TIM15_CR2_CCDS_Msk (0x1UL << TIM15_CR2_CCDS_Pos)
#define TIM15_CR2_CCDS TIM15_CR2_CCDS_Msk

#define TIM15_CR2_CCUS_Pos 2U
#define TIM15_CR2_CCUS_Msk (0x1UL << TIM15_CR2_CCUS_Pos)
#define TIM15_CR2_CCUS TIM15_CR2_CCUS_Msk

#define TIM15_CR2_CCPC_Pos 0U
#define TIM15_CR2_CCPC_Msk (0x1UL << TIM15_CR2_CCPC_Pos)
#define TIM15_CR2_CCPC TIM15_CR2_CCPC_Msk

/*SMCR register.*/
#define TIM15_SMCR_MSM_Pos 7U
#define TIM15_SMCR_MSM_Msk (0x1UL << TIM15_SMCR_MSM_Pos)
#define TIM15_SMCR_MSM TIM15_SMCR_MSM_Msk

#define TIM15_SMCR_TS_Pos 4U
#define TIM15_SMCR_TS_Msk (0x7UL << TIM15_SMCR_TS_Pos)
#define TIM15_SMCR_TS TIM15_SMCR_TS_Msk

#define TIM15_SMCR_SMS3_Pos 16U
#define TIM15_SMCR_SMS_Pos 0U
#define TIM15_SMCR_SMS_Msk ((0x1UL<< TIM15_SMCR_SMS3_Pos) | (0x7UL << TIM15_SMCR_SMS_Pos))
#define TIM15_SMCR_SMS TIM15_SMCR_SMS_Msk

/*DIER register.*/
#define TIM15_DIER_TDE_Pos 14U
#define TIM15_DIER_TDE_Msk (0x1UL << TIM15_DIER_TDE_Pos)
#define TIM15_DIER_TDE TIM15_DIER_TDE_Msk

#define TIM15_DIER_COMDE_Pos 13U
#define TIM15_DIER_COMDE_Msk (0x1UL << TIM15_DIER_COMDE_Pos)
#define TIM15_DIER_COMDE TIM15_DIER_COMDE_Msk

#define TIM15_DIER_CC2DE_Pos 10U
#define TIM15_DIER_CC2DE_Msk (0x1UL << TIM15_DIER_CC2DE_Pos)
#define TIM15_DIER_CC2DE TIM15_DIER_CC2DE_Msk

#define TIM15_DIER_CC1DE_Pos 9U
#define TIM15_DIER_CC1DE_Msk (0x1UL << TIM15_DIER_CC1DE_Pos)
#define TIM15_DIER_CC1DE TIM15_DIER_CC1DE_Msk

#define TIM15_DIER_UDE_Pos 8U
#define TIM15_DIER_UDE_Msk (0x1UL << TIM15_DIER_UDE_Pos)
#define TIM15_DIER_UDE TIM15_DIER_UDE_Msk

#define TIM15_DIER_BIE_Pos 7U
#define TIM15_DIER_BIE_Msk (0x1UL << TIM15_DIER_BIE_Pos)
#define TIM15_DIER_BIE TIM15_DIER_BIE_Msk

#define TIM15_DIER_TIE_Pos 6U
#define TIM15_DIER_TIE_Msk (0x1UL << TIM15_DIER_TIE_Pos)
#define TIM15_DIER_TIE TIM15_DIER_TIE_Msk

#define TIM15_DIER_COMIE_Pos 5U
#define TIM15_DIER_COMIE_Msk (0x1UL << TIM15_DIER_COMIE_Pos)
#define TIM15_DIER_COMIE TIM15_DIER_COMIE_Msk

#define TIM15_DIER_CC2IE_Pos 2U
#define TIM15_DIER_CC2IE_Msk (0x1UL << TIM15_DIER_CC2IE_Pos)
#define TIM15_DIER_CC2IE TIM15_DIER_CC2IE_Msk

#define TIM15_DIER_CC1IE_Pos 1U
#define TIM15_DIER_CC1IE_Msk (0x1UL << TIM15_DIER_CC1IE_Pos)
#define TIM15_DIER_CC1IE TIM15_DIER_CC1IE_Msk

#define TIM15_DIER_UIE_Pos 0U
#define TIM15_DIER_UIE_Msk (0x1UL << TIM15_DIER_UIE_Pos)
#define TIM15_DIER_UIE TIM15_DIER_UIE_Msk

/*SR register.*/
#define TIM15_SR_CC2OF_Pos 10U
#define TIM15_SR_CC2OF_Msk (0x1UL << TIM15_SR_CC2OF_Pos)
#define TIM15_SR_CC2OF TIM15_SR_CC2OF_Msk

#define TIM15_SR_CC1OF_Pos 9U
#define TIM15_SR_CC1OF_Msk (0x1UL << TIM15_SR_CC1OF_Pos)
#define TIM15_SR_CC1OF TIM15_SR_CC1OF_Msk

#define TIM15_SR_BIF_Pos 7U
#define TIM15_SR_BIF_Msk (0x1UL << TIM15_SR_BIF_Pos)
#define TIM15_SR_BIF TIM15_SR_BIF_Msk

#define TIM15_SR_TIF_Pos 6U
#define TIM15_SR_TIF_Msk (0x1UL << TIM15_SR_TIF_Pos)
#define TIM15_SR_TIF TIM15_SR_TIF_Msk

#define TIM15_SR_COMIF_Pos 5U
#define TIM15_SR_COMIF_Msk (0x1UL << TIM15_SR_COMIF_Pos)
#define TIM15_SR_COMIF TIM15_SR_COMIF_Msk

#define TIM15_SR_CC2IF_Pos 2U
#define TIM15_SR_CC2IF_Msk (0x1UL << TIM15_SR_CC2IF_Pos)
#define TIM15_SR_CC2IF TIM15_SR_CC2IF_Msk

#define TIM15_SR_CC1IF_Pos 1U
#define TIM15_SR_CC1IF_Msk (0x1UL << TIM15_SR_CC1IF_Pos)
#define TIM15_SR_CC1IF TIM15_SR_CC1IF_Msk

#define TIM15_SR_UIF_Pos 0U
#define TIM15_SR_UIF_Msk (0x1UL << TIM15_SR_UIF_Pos)
#define TIM15_SR_UIF TIM15_SR_UIF_Msk

/*EGR register.*/
#define TIM15_EGR_BG_Pos 7U
#define TIM15_EGR_BG_Msk (0x1UL << TIM15_EGR_BG_Pos)
#define TIM15_EGR_BG TIM15_EGR_BG_Msk

#define TIM15_EGR_TG_Pos 6U
#define TIM15_EGR_TG_Msk (0x1UL << TIM15_EGR_TG_Pos)
#define TIM15_EGR_TG TIM15_EGR_TG_Msk

#define TIM15_EGR_COMG_Pos 5U
#define TIM15_EGR_COMG_Msk (0x1UL << TIM15_EGR_COMG_Pos)
#define TIM15_EGR_COMG TIM15_EGR_COMG_Msk

#define TIM15_EGR_CC2G_Pos 2U
#define TIM15_EGR_CC2G_Msk (0x1UL << TIM15_EGR_CC2G_Pos)
#define TIM15_EGR_CC2G TIM15_EGR_CC2G_Msk

#define TIM15_EGR_CC1G_Pos 1U
#define TIM15_EGR_CC1G_Msk (0x1UL << TIM15_EGR_CC1G_Pos)
#define TIM15_EGR_CC1G TIM15_EGR_CC1G_Msk

#define TIM15_EGR_UG_Pos 0U
#define TIM15_EGR_UG_Msk (0x1UL << TIM15_EGR_UG_Pos)
#define TIM15_EGR_UG TIM15_EGR_UG_Msk

/*CCMR1 register.*/
/*******Channel 2*******/
#define TIM15_CCMR1_CC2S_Pos 8U
#define TIM15_CCMR1_CC2S_Msk (0x3UL << TIM15_CCMR1_CC2S_Pos)

/*Input Capture Mode.*/
#define TIM15_CCMR1_IC2F_Pos 12U
#define TIM15_CCMR1_IC2F_Msk (0xFUL << TIM15_CCMR1_IC2F_Pos)

#define TIM15_CCMR1_IC2PSC_Pos 10U
#define TIM15_CCMR1_IC2PSC_Msk (0x3UL << TIM15_CCMR1_IC2PSC_Pos)

/*Output Compare Mode.*/
#define TIM15_CCMR1_OC2M3_Pos 24U
#define TIM15_CCMR1_OC2M_Pos 12U
#define TIM15_CCMR1_OC2M_Msk ((0x1UL << TIM15_CCMR1_OC2M3_Pos) | (0x7UL << TIM15_CCMR1_OC2M_Pos))

#define TIM15_CCMR1_OC2PE_Pos 11U
#define TIM15_CCMR1_OC2PE_Msk (0x1UL << TIM15_CCMR1_OC2PE_Pos)

#define TIM15_CCMR1_OC2FE_Pos 10U
#define TIM15_CCMR1_OC2FE_Msk (0x1UL << TIM15_CCMR1_OC2FE_Pos)

/*******Channel 2*******/
#define TIM15_CCMR1_CC1S_Pos 0U
#define TIM15_CCMR1_CC1S_Msk (0x3UL << TIM15_CCMR1_CC1S_Pos)

/*Input Capture Mode.*/
#define TIM15_CCMR1_IC1F_Pos 4U
#define TIM15_CCMR1_IC1F_Msk (0xFUL << TIM15_CCMR1_IC1F_Pos)

#define TIM15_CCMR1_IC1PSC_Pos 2U
#define TIM15_CCMR1_IC1PSC_Msk (0x3UL << TIM15_CCMR1_IC1PSC_Pos)

/*Output Compare Mode.*/
#define TIM15_CCMR1_OC1M3_Pos 16U
#define TIM15_CCMR1_OC1M_Pos 4U
#define TIM15_CCMR1_OC1M_Msk ((0x1UL << TIM15_CCMR1_OC1M3_Pos) | (0x7UL << TIM15_CCMR1_OC1M_Pos))

#define TIM15_CCMR1_OC1PE_Pos 3U
#define TIM15_CCMR1_OC1PE_Msk (0x1UL << TIM15_CCMR1_OC1PE_Pos)

#define TIM15_CCMR1_OC1FE_Pos 2U
#define TIM15_CCMR1_OC1FE_Msk (0x1UL << TIM15_CCMR1_OC1FE_Pos)

/*CCER register.*/
#define TIM15_CCER_CC2NP_Pos 7U
#define TIM15_CCER_CC2NP_Msk (0x1UL << TIM15_CCER_CC2NP_Pos)
#define TIM15_CCER_CC2NP TIM15_CCER_CC2NP_Msk

#define TIM15_CCER_CC2P_Pos 5U
#define TIM15_CCER_CC2P_Msk (0x1UL << TIM15_CCER_CC2P_Pos)
#define TIM15_CCER_CC2P TIM15_CCER_CC2P_Msk

#define TIM15_CCER_CC2E_Pos 4U
#define TIM15_CCER_CC2E_Msk (0x1UL << TIM15_CCER_CC2E_Pos)
#define TIM15_CCER_CC2E TIM15_CCER_CC2E_Msk

#define TIM15_CCER_CC1NP_Pos 3U
#define TIM15_CCER_CC1NP_Msk (0x1UL << TIM15_CCER_CC1NP_Pos)
#define TIM15_CCER_CC1NP TIM15_CCER_CC1NP_Msk

#define TIM15_CCER_CC1NE_Pos 2U
#define TIM15_CCER_CC1NE_Msk (0x1UL << TIM15_CCER_CC1NE_Pos)
#define TIM15_CCER_CC1NE TIM15_CCER_CC1NE_Msk

#define TIM15_CCER_CC1P_Pos 1U
#define TIM15_CCER_CC1P_Msk (0x1UL << TIM15_CCER_CC1P_Pos)
#define TIM15_CCER_CC1P TIM15_CCER_CC1P_Msk

#define TIM15_CCER_CC1E_Pos 0U
#define TIM15_CCER_CC1E_Msk (0x1UL << TIM15_CCER_CC1E_Pos)
#define TIM15_CCER_CC1E TIM15_CCER_CC1E_Msk

/*CNT register.*/
#define TIM15_CNT_UIFCPY_Pos 31U
#define TIM15_CNT_UIFCPY_Msk (0x1UL << TIM15_CNT_UIFCPY_Pos)
#define TIM15_CNT_UIFCPY TIM15_CNT_UIFCPY_Msk

#define TIM15_CNT_Pos 0U
#define TIM15_CNT_Msk (0xFFFFUL << TIM15_CNT_Pos)
#define TIM15_CNT TIM15_CNT_Msk

/*PSC register.*/
#define TIM15_PSC_Pos 0U
#define TIM15_PSC_Msk (0xFFFFUL << TIM15_PSC_Pos)
#define TIM15_PSC TIM15_PSC_Msk

/*ARR register.*/
#define TIM15_ARR_Pos 0U
#define TIM15_ARR_Msk (0xFFFFUL << TIM15_ARR_Pos)
#define TIM15_ARR TIM15_ARR_Msk

/*RCR register.*/
#define TIM15_RCR_REP_Pos 0U
#define TIM15_RCR_REP_Msk (0xFFUL << TIM15_RCR_REP_Pos)
#define TIM15_RCR_REP TIM15_RCR_REP_Msk

/*CCR1 register.*/
#define TIM15_CCR1_Pos 0U
#define TIM15_CCR1_Msk (0xFFFFUL << TIM15_CCR1_Pos)
#define TIM15_CCR1 TIM15_CCR1_Msk

/*CCR2 register.*/
#define TIM15_CCR2_Pos 0U
#define TIM15_CCR2_Msk (0xFFFFUL << TIM15_CCR2_Pos)
#define TIM15_CCR2 TIM15_CCR2_Msk

/*BDTR register.*/
#define TIM15_BDTR_MOE_Pos 15U
#define TIM15_BDTR_MOE_Msk (0x1UL << TIM15_BDTR_MOE_Pos)
#define TIM15_BDTR_MOE TIM15_BDTR_MOE_Msk

#define TIM15_BDTR_AOE_Pos 14U
#define TIM15_BDTR_AOE_Msk (0x1UL << TIM15_BDTR_AOE_Pos)
#define TIM15_BDTR_AOE TIM15_BDTR_AOE_Msk

#define TIM15_BDTR_BKP_Pos 13U
#define TIM15_BDTR_BKP_Msk (0x1UL << TIM15_BDTR_BKP_Pos)
#define TIM15_BDTR_BKP TIM15_BDTR_BKP_Msk

#define TIM15_BDTR_BKE_Pos 12U
#define TIM15_BDTR_BKE_Msk (0x1UL << TIM15_BDTR_BKE_Pos)
#define TIM15_BDTR_BKE TIM15_BDTR_BKE_Msk

#define TIM15_BDTR_OSSR_Pos 11U
#define TIM15_BDTR_OSSR_Msk (0x1UL << TIM15_BDTR_OSSR_Pos)
#define TIM15_BDTR_OSSR TIM15_BDTR_OSSR_Msk

#define TIM15_BDTR_OSSI_Pos 10U
#define TIM15_BDTR_OSSI_Msk (0x1UL << TIM15_BDTR_OSSI_Pos)
#define TIM15_BDTR_OSSI TIM15_BDTR_OSSI_Msk

#define TIM15_BDTR_LOCK_Pos 8U
#define TIM15_BDTR_LOCK_Msk (0x3UL << TIM15_BDTR_LOCK_Pos)
#define TIM15_BDTR_LOCK TIM15_BDTR_LOCK_Msk

#define TIM15_BDTR_DTG_Pos 0U
#define TIM15_BDTR_DTG_Msk (0xFFUL << TIM15_BDTR_DTG_Pos)
#define TIM15_BDTR_DTG TIM15_BDTR_DTG_Msk

/*DCR register.*/
#define TIM15_DCR_DBL_Pos 8U
#define TIM15_DCR_DBL_Msk (0x1FUL << TIM15_DCR_DBL_Pos)
#define TIM15_DCR_DBL TIM15_DCR_DBL_Msk

#define TIM15_DCR_DBA_Pos 0U
#define TIM15_DCR_DBA_Msk (0x1FUL << TIM15_DCR_DBA_Pos)
#define TIM15_DCR_DBA TIM15_DCR_DBA_Msk

/*OR1 register.*/
#define TIM15_OR1_ENCODER_MODE_Pos 1U
#define TIM15_OR1_ENCODER_MODE_Msk (0x3UL << TIM15_OR1_ENCODER_MODE_Pos)
#define TIM15_OR1_ENCODER_MODE TIM15_OR1_ENCODER_MODE_Msk

#define TIM15_OR1_TI1_RMP_Pos 0U
#define TIM15_OR1_TI1_RMP_Msk (0x1UL << TIM15_OR1_TI1_RMP_Pos)
#define TIM15_OR1_TI1_RMP TIM15_OR1_TI1_RMP_Msk

/*OR2 register.*/
#define TIM15_OR2_BKCMP2P_Pos 11U
#define TIM15_OR2_BKCMP2P_Msk (0x1UL << TIM15_OR2_BKCMP2P_Pos)
#define TIM15_OR2_BKCMP2P TIM15_OR2_BKCMP2P_Msk

#define TIM15_OR2_BKCMP1P_Pos 10U
#define TIM15_OR2_BKCMP1P_Msk (0x1UL << TIM15_OR2_BKCMP1P_Pos)
#define TIM15_OR2_BKCMP1P TIM15_OR2_BKCMP1P_Msk

#define TIM15_OR2_BKINP_Pos 9U
#define TIM15_OR2_BKINP_Msk (0x1UL << TIM15_OR2_BKINP_Pos)
#define TIM15_OR2_BKINP TIM15_OR2_BKINP_Msk

#define TIM15_OR2_BKCMP2E_Pos 2U
#define TIM15_OR2_BKCMP2E_Msk (0x1UL << TIM15_OR2_BKCMP2E_Pos)
#define TIM15_OR2_BKCMP2E TIM15_OR2_BKCMP2E_Msk

#define TIM15_OR2_BKCMP1E_Pos 1U
#define TIM15_OR2_BKCMP1E_Msk (0x1UL << TIM15_OR2_BKCMP1E_Pos)
#define TIM15_OR2_BKCMP1E TIM15_OR2_BKCMP1E_Msk

#define TIM15_OR2_BKINE_Pos 0U
#define TIM15_OR2_BKINE_Msk (0x1UL << TIM15_OR2_BKINE_Pos)
#define TIM15_OR2_BKINE TIM15_OR2_BKINE_Msk

/* ========================================================================= */
/* ============                      TIM16                      ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR1; /*!< TIM16 Control Register 1. */
	__IOM uint32_t CR2; /*!< TIM16 Control Register 2. */
	uint32_t RESERVED0;
	__IOM uint32_t DIER; /*!< TIM16 DMA/Interrupt Enable Register. */
	__IOM uint32_t SR; /*!< TIM16 Status Register. */
	__OM uint32_t EGR; /*!< TIM16 Event Generation Register. */
	__IOM uint32_t CCMR1; /*!< TIM16 Capture/Compare Mode Register 1. */
	uint32_t RESERVED1;
	__IOM uint32_t CCER; /*!< TIM16 Capture/Compare Enable Register. */
	__IOM uint32_t CNT; /*!< TIM16 Counter. */
	__IOM uint32_t PSC; /*!< TIM16 Prescaler. */
	__IOM uint32_t ARR; /*!< TIM16 Auto-Reload Register. */
	__IOM uint32_t RCR; /*!< TIM16 Repetition Counter Register. */
	__IOM uint32_t CCR1; /*!< TIM16 Capture/Compare Register 1. */
	uint32_t RESERVED2[3];
	__IOM uint32_t BDTR; /*!< TIM16 Break and Dead Time Register. */
	__IOM uint32_t DCR; /*!< TIM16 DMA Control Register. */
	__IOM uint32_t DMAR; /*!< TIM16 DMA address for full transfer. */
	__IOM uint32_t OR1; /*!< TIM16 Option Register 1. */
	uint32_t RESERVED3[3];
	__IOM uint32_t OR2; /*!< TIM16 Option Register 2. */
}STM32L4xx_TIM16_TypeDef;

/*CR1 register.*/
#define TIM16_CR1_UIFREMAP_Pos 11U
#define TIM16_CR1_UIFREMAP_Msk (0x1UL << TIM16_CR1_UIFREMAP_Pos)
#define TIM16_CR1_UIFREMAP TIM16_CR1_UIFREMAP_Msk

#define TIM16_CR1_CKD_Pos 8U
#define TIM16_CR1_CKD_Msk (0x3UL << TIM16_CR1_CKD_Pos)
#define TIM16_CR1_CKD TIM16_CR1_CKD_Msk

#define TIM16_CR1_ARPE_Pos 7U
#define TIM16_CR1_ARPE_Msk (0x1UL << TIM16_CR1_ARPE_Pos)
#define TIM16_CR1_ARPE TIM16_CR1_ARPE_Msk

#define TIM16_CR1_OPM_Pos 3U
#define TIM16_CR1_OPM_Msk (0x1UL << TIM16_CR1_OPM_Pos)
#define TIM16_CR1_OPM TIM16_CR1_OPM_Msk

#define TIM16_CR1_URS_Pos 2U
#define TIM16_CR1_URS_Msk (0x1UL << TIM16_CR1_URS_Pos)
#define TIM16_CR1_URS TIM16_CR1_URS_Msk

#define TIM16_CR1_UDIS_Pos 1U
#define TIM16_CR1_UDIS_Msk (0x1UL << TIM16_CR1_UDIS_Pos)
#define TIM16_CR1_UDIS TIM16_CR1_UDIS_Msk

#define TIM16_CR1_CEN_Pos 0U
#define TIM16_CR1_CEN_Msk (0x1UL << TIM16_CR1_CEN_Pos)
#define TIM16_CR1_CEN TIM16_CR1_CEN_Msk

/*CR2 register.*/
#define TIM16_CR2_OIS1N_Pos 9U
#define TIM16_CR2_OIS1N_Msk (0x1UL << TIM16_CR2_OIS1N_Pos)
#define TIM16_CR2_OIS1N TIM16_CR2_OIS1N_Msk

#define TIM16_CR2_OIS1_Pos 8U
#define TIM16_CR2_OIS1_Msk (0x1UL << TIM16_CR2_OIS1_Pos)
#define TIM16_CR2_OIS1 TIM16_CR2_OIS1_Msk

#define TIM16_CR2_CCDS_Pos 3U
#define TIM16_CR2_CCDS_Msk (0x1UL << TIM16_CR2_CCDS_Pos)
#define TIM16_CR2_CCDS TIM16_CR2_CCDS_Msk

#define TIM16_CR2_CCUS_Pos 2U
#define TIM16_CR2_CCUS_Msk (0x1UL << TIM16_CR2_CCUS_Pos)
#define TIM16_CR2_CCUS TIM16_CR2_CCUS_Msk

#define TIM16_CR2_CCPC_Pos 0U
#define TIM16_CR2_CCPC_Msk (0x1UL << TIM16_CR2_CCPC_Pos)
#define TIM16_CR2_CCPC TIM16_CR2_CCPC_Msk

/*DIER register.*/
#define TIM16_DIER_COMDE_Pos 13U
#define TIM16_DIER_COMDE_Msk (0x1UL << TIM16_DIER_COMDE_Pos)
#define TIM16_DIER_COMDE TIM16_DIER_COMDE_Msk

#define TIM16_DIER_CC1DE_Pos 9U
#define TIM16_DIER_CC1DE_Msk (0x1UL << TIM16_DIER_CC1DE_Pos)
#define TIM16_DIER_CC1DE TIM16_DIER_CC1DE_Msk

#define TIM16_DIER_UDE_Pos 8U
#define TIM16_DIER_UDE_Msk (0x1UL << TIM16_DIER_UDE_Pos)
#define TIM16_DIER_UDE TIM16_DIER_UDE_Msk

#define TIM16_DIER_BIE_Pos 7U
#define TIM16_DIER_BIE_Msk (0x1UL << TIM16_DIER_BIE_Pos)
#define TIM16_DIER_BIE TIM16_DIER_BIE_Msk

#define TIM16_DIER_COMIE_Pos 5U
#define TIM16_DIER_COMIE_Msk (0x1UL << TIM16_DIER_COMIE_Pos)
#define TIM16_DIER_COMIE TIM16_DIER_COMIE_Msk

#define TIM16_DIER_CC1IE_Pos 1U
#define TIM16_DIER_CC1IE_Msk (0x1UL << TIM16_DIER_CC1IE_Pos)
#define TIM16_DIER_CC1IE TIM16_DIER_CC1IE_Msk

#define TIM16_DIER_UIE_Pos 0U
#define TIM16_DIER_UIE_Msk (0x1UL << TIM16_DIER_UIE_Pos)
#define TIM16_DIER_UIE TIM16_DIER_UIE_Msk

/*SR register.*/
#define TIM16_SR_CC1OF_Pos 9U
#define TIM16_SR_CC1OF_Msk (0x1UL << TIM16_SR_CC1OF_Pos)
#define TIM16_SR_CC1OF TIM16_SR_CC1OF_Msk

#define TIM16_SR_BIF_Pos 7U
#define TIM16_SR_BIF_Msk (0x1UL << TIM16_SR_BIF_Pos)
#define TIM16_SR_BIF TIM16_SR_BIF_Msk

#define TIM16_SR_COMIF_Pos 5U
#define TIM16_SR_COMIF_Msk (0x1UL << TIM16_SR_COMIF_Pos)
#define TIM16_SR_COMIF TIM16_SR_COMIF_Msk

#define TIM16_SR_CC1IF_Pos 1U
#define TIM16_SR_CC1IF_Msk (0x1UL << TIM16_SR_CC1IF_Pos)
#define TIM16_SR_CC1IF TIM16_SR_CC1IF_Msk

#define TIM16_SR_UIF_Pos 0U
#define TIM16_SR_UIF_Msk (0x1UL << TIM16_SR_UIF_Pos)
#define TIM16_SR_UIF TIM16_SR_UIF_Msk

/*EGR register.*/
#define TIM16_EGR_BG_Pos 7U
#define TIM16_EGR_BG_Msk (0x1UL << TIM16_EGR_BG_Pos)
#define TIM16_EGR_BG TIM16_EGR_BG_Msk

#define TIM16_EGR_COMG_Pos 5U
#define TIM16_EGR_COMG_Msk (0x1UL << TIM16_EGR_COMG_Pos)
#define TIM16_EGR_COMG TIM16_EGR_COMG_Msk

#define TIM16_EGR_CC1G_Pos 1U
#define TIM16_EGR_CC1G_Msk (0x1UL << TIM16_EGR_CC1G_Pos)
#define TIM16_EGR_CC1G TIM16_EGR_CC1G_Msk

#define TIM16_EGR_UG_Pos 0U
#define TIM16_EGR_UG_Msk (0x1UL << TIM16_EGR_UG_Pos)
#define TIM16_EGR_UG TIM16_EGR_UG_Msk

/*CCMR1 register.*/
/******Channel 1.*******/
#define TIM16_CCMR1_CC1S_Pos 0U
#define TIM16_CCMR1_CC1S_Msk (0x3UL << TIM16_CCMR1_CC1S_Pos)
#define TIM16_CCMR1_CC1S TIM16_CCMR1_CC1S_Msk

/*Input Capture Mode.*/
#define TIM16_CCMR1_IC1F_Pos 4U
#define TIM16_CCMR1_IC1F_Msk (0xFUL << TIM16_CCMR1_IC1F_Pos)
#define TIM16_CCMR1_IC1F TIM16_CCMR1_IC1F_Msk

#define TIM16_CCMR1_IC1PSC_Pos 2U
#define TIM16_CCMR1_IC1PSC_Msk (0x3UL << TIM16_CCMR1_IC1PSC_Pos)
#define TIM16_CCMR1_IC1PSC TIM16_CCMR1_IC1PSC_Msk

/*Output Compare Mode.*/
#define TIM16_CCMR1_OC1M3_Pos 16U
#define TIM16_CCMR1_OC1M_Pos 4U
#define TIM16_CCMR1_OC1M_Msk ((0x1UL << TIM16_CCMR1_OC1M3_Pos) | (0x7UL << TIM16_CCMR1_OC1M_Pos))
#define TIM16_CCMR1_OC1M TIM16_CCMR1_OC1M_Msk

#define TIM16_CCMR1_OC1PE_Pos 3U
#define TIM16_CCMR1_OC1PE_Msk (0x1UL << TIM16_CCMR1_OC1PE_Pos)
#define TIM16_CCMR1_OC1PE TIM16_CCMR1_OC1PE_Msk

#define TIM16_CCMR1_OC1FE_Pos 2U
#define TIM16_CCMR1_OC1FE_Msk (0x1UL << TIM16_CCMR1_OC1FE_Pos)
#define TIM16_CCMR1_OC1FE

/*CCER register.*/
#define TIM16_CCER_CC1NP_Pos 3U
#define TIM16_CCER_CC1NP_Msk (0x1UL << TIM16_CCER_CC1NP_Pos)
#define TIM16_CCER_CC1NP TIM16_CCER_CC1NP_Msk

#define TIM16_CCER_CC1NE_Pos 2U
#define TIM16_CCER_CC1NE_Msk (0x1UL << TIM16_CCER_CC1NE_Pos)
#define TIM16_CCER_CC1NE TIM16_CCER_CC1NE_Msk

#define TIM16_CCER_CC1P_Pos 1U
#define TIM16_CCER_CC1P_Msk (0x1UL << TIM16_CCER_CC1P_Pos)
#define TIM16_CCER_CC1P TIM16_CCER_CC1P_Msk

#define TIM16_CCER_CC1E_Pos 0U
#define TIM16_CCER_CC1E_Msk (0x1UL << TIM16_CCER_CC1E_Pos)
#define TIM16_CCER_CC1E TIM16_CCER_CC1E_Msk

/*CNT register.*/
#define TIM16_CNT_UIFCPY_Pos 31U
#define TIM16_CNT_UIFCPY_Msk (0x1UL << TIM16_CNT_UIFCPY_Pos)
#define TIM16_CNT_UIFCPY TIM16_CNT_UIFCPY_Msk

#define TIM16_CNT_Pos 0U
#define TIM16_CNT_Msk (0xFFFFUL << TIM16_CNT_Pos)
#define TIM16_CNT TIM16_CNT_Msk

/*PSC register.*/
#define TIM16_PSC_Pos 0U
#define TIM16_PSC_Msk (0xFFFFUL << TIM16_PSC_Pos)
#define TIM16_PSC TIM16_PSC_Msk

/*ARR register.*/
#define TIM16_ARR_Pos 0U
#define TIM16_ARR_Msk (0xFFFFUL << TIM16_ARR_Pos)
#define TIM16_ARR TIM16_ARR_Pos

/*RCR register.*/
#define TIM16_RCR_REP_Pos 0U
#define TIM16_RCR_REP_Msk (0xFFUL << TIM16_RCR_REP_Pos)
#define TIM16_RCR_REP TIM16_RCR_REP_Msk

/*CCR1 register.*/
#define TIM16_CCR1_Pos 0U
#define TIM16_CCR1_Msk (0xFFFFUL << TIM16_CCR1_Pos)
#define TIM16_CCR1 TIM16_CCR1_Msk

/*BDTR register.*/
#define TIM16_BDTR_MOE_Pos 15U
#define TIM16_BDTR_MOE_Msk (0x1UL << TIM16_BDTR_MOE_Pos)
#define TIM16_BDTR_MOE TIM16_BDTR_MOE_Msk

#define TIM16_BDTR_AOE_Pos 14U
#define TIM16_BDTR_AOE_Msk (0x1UL << TIM16_BDTR_AOE_Pos)
#define TIM16_BDTR_AOE TIM16_BDTR_AOE_Msk

#define TIM16_BDTR_BKP_Pos 13U
#define TIM16_BDTR_BKP_Msk (0x1UL << TIM16_BDTR_BKP_Pos)
#define TIM16_BDTR_BKP TIM16_BDTR_BKP_Msk

#define TIM16_BDTR_BKE_Pos 12U
#define TIM16_BDTR_BKE_Msk (0x1UL << TIM16_BDTR_BKE_Pos)
#define TIM16_BDTR_BKE TIM16_BDTR_BKE_Msk

#define TIM16_BDTR_OSSR_Pos 11U
#define TIM16_BDTR_OSSR_Msk (0x1UL << TIM16_BDTR_OSSR_Pos)
#define TIM16_BDTR_OSSR TIM16_BDTR_OSSR_Msk

#define TIM16_BDTR_OSSI_Pos 10U
#define TIM16_BDTR_OSSI_Msk (0x1UL << TIM16_BDTR_OSSI_Pos)
#define TIM16_BDTR_OSSI TIM16_BDTR_OSSI_Msk

#define TIM16_BDTR_LOCK_Pos 8U
#define TIM16_BDTR_LOCK_Msk (0x3UL << TIM16_BDTR_LOCK_Pos)
#define TIM16_BDTR_LOCK TIM16_BDTR_LOCK_Msk

#define TIM16_BDTR_DTG_Pos 0U
#define TIM16_BDTR_DTG_Msk (0xFFUL << TIM16_BDTR_DTG_Pos)
#define TIM16_BDTR_DTG TIM16_BDTR_DTG_Msk

/*DCR register.*/
#define TIM16_DCR_DBL_Pos 8U
#define TIM16_DCR_DBL_Msk (0x1FUL << TIM16_DCR_DBL_Pos)
#define TIM16_DCR_DBL TIM16_DCR_DBL_Msk

#define TIM16_DCR_DBA_Pos 0U
#define TIM16_DCR_DBA_Msk (0x1FUL << TIM16_DCR_DBA_Pos)
#define TIM16_DCR_DBA TIM16_DCR_DBA_Msk

/*DMAR register.*/
#define TIM16_DMAR_DMAB_Pos 0U
#define TIM16_DMAR_DMAB_Msk (0xFFFFUL << TIM16_DMAR_DMAB_Pos)
#define TIM16_DMAR_DMAB TIM16_DMAR_DMAB_Msk

/*OR1 register.*/
#define TIM16_OR1_TI1_RMP_Pos 0U
#define TIM16_OR1_TI1_RMP_Msk (0x7UL << TIM16_OR1_TI1_RMP_Pos)
#define TIM16_OR1_TI1_RMP TIM16_OR1_TI1_RMP_Msk

/*OR2 register.*/
#define TIM16_OR2_BKCMP2P_Pos 11U
#define TIM16_OR2_BKCMP2P_Msk (0x1UL << TIM16_OR2_BKCMP2P_Pos)
#define TIM16_OR2_BKCMP2P TIM16_OR2_BKCMP2P_Msk

#define TIM16_OR2_BKCMP1P_Pos 10U
#define TIM16_OR2_BKCMP1P_Msk (0x1UL << TIM16_OR2_BKCMP1P_Pos)
#define TIM16_OR2_BKCMP1P TIM16_OR2_BKCMP1P_Msk

#define TIM16_OR2_BKINP_Pos 9U
#define TIM16_OR2_BKINP_Msk (0x1UL << TIM16_OR2_BKINP_Pos)
#define TIM16_OR2_BKINP TIM16_OR2_BKINP_Msk

#define TIM16_OR2_BKCMP2E_Pos 2U
#define TIM16_OR2_BKCMP2E_Msk (0x1UL << TIM16_OR2_BKCMP2E_Pos)
#define TIM16_OR2_BKCMP2E TIM16_OR2_BKCMP2E_Msk

#define TIM16_OR2_BKCMP1E_Pos 1U
#define TIM16_OR2_BKCMP1E_Msk (0x1UL << TIM16_OR2_BKCMP1E_Pos)
#define TIM16_OR2_BKCMP1E TIM16_OR2_BKCMP1E_Msk

#define TIM16_OR2_BKINE_Pos 0U
#define TIM16_OR2_BKINE_Msk (0x1UL << TIM16_OR2_BKINE_Pos)
#define TIM16_OR2_BKINE TIM16_OR2_BKINE_Msk

/* ========================================================================= */
/* ============                      TIM6/7                     ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR1; /*!< TIM6/7 Control Register 1. */
	__IOM uint32_t CR2; /*!< TIM6/7 Control Register 2. */
	uint32_t RESERVED0;
	__IOM uint32_t DIER; /*!< TIM6/7 DMA/Interrupt Enable Register. */
	__IOM uint32_t SR; /*!< TIM6/7 Status Register. */
	__OM uint32_t EGR; /*!< TIM6/7 Event Generation Register. */
	uint32_t RESERVED1[3];
	__IOM uint32_t CNT; /*!< TIM6/7 Counter. */
	__IOM uint32_t PSC; /*!< TIM6/7 Prescaler. */
	__IOM uint32_t ARR; /*!< TIM6/7 Auto-Reload Register. */
}STM32L4xx_TIM6_7_TypeDef;

/*CR1 register.*/
#define TIM6_7_CR1_UIFREMAP_Pos 11U
#define TIM6_7_CR1_UIFREMAP_Msk (0x1UL << TIM6_7_CR1_UIFREMAP_Pos)
#define TIM6_7_CR1_UIFREMAP TIM6_7_CR1_UIFREMAP_Msk

#define TIM6_7_CR1_ARPE_Pos 7U
#define TIM6_7_CR1_ARPE_Msk (0x1UL << TIM6_7_CR1_ARPE_Pos)
#define TIM6_7_CR1_ARPE TIM6_7_CR1_ARPE_Msk

#define TIM6_7_CR1_OPM_Pos 3U
#define TIM6_7_CR1_OPM_Msk (0x1UL << TIM6_7_CR1_OPM_Pos)
#define TIM6_7_CR1_OPM TIM6_7_CR1_OPM_Msk

#define TIM6_7_CR1_URS_Pos 2U
#define TIM6_7_CR1_URS_Msk (0x1UL << TIM6_7_CR1_URS_Pos)
#define TIM6_7_CR1_URS TIM6_7_CR1_URS_Msk

#define TIM6_7_CR1_UDIS_Pos 1U
#define TIM6_7_CR1_UDIS_Msk (0x1UL << TIM6_7_CR1_UDIS_Pos)
#define TIM6_7_CR1_UDIS TIM6_7_CR1_UDIS_Msk

#define TIM6_7_CR1_CEN_Pos 0U
#define TIM6_7_CR1_CEN_Msk (0x1UL << TIM6_7_CR1_CEN_Pos)
#define TIM6_7_CR1_CEN TIM6_7_CR1_CEN_Msk

/*CR2 register.*/
#define TIM6_7_CR2_MMS_Pos 4U
#define TIM6_7_CR2_MMS_Msk (0x7UL << TIM6_7_CR2_MMS_Pos)
#define TIM6_7_CR2_MMS TIM6_7_CR2_MMS_Msk

/*DIER register.*/
#define TIM6_7_DIER_UDE_Pos 8U
#define TIM6_7_DIER_UDE_Msk (0x1UL << TIM6_7_DIER_UDE_Pos)
#define TIM6_7_DIER_UDE TIM6_7_DIER_UDE_Msk

#define TIM6_7_DIER_UIE_Pos 0U
#define TIM6_7_DIER_UIE_Msk (0x1UL << TIM6_7_DIER_UIE_Pos)
#define TIM6_7_DIER_UIE TIM6_7_DIER_UIE_Msk

/*SR register.*/
#define TIM6_7_SR_UIF_Pos 0U
#define TIM6_7_SR_UIF_Msk (0x1UL << TIM6_7_SR_UIF_Pos)
#define TIM6_7_SR_UIF TIM6_7_SR_UIF_Msk

/*EGR register.*/
#define TIM6_7_EGR_UG_Pos 0U
#define TIM6_7_EGR_UG_Msk (0x1UL << TIM6_7_EGR_UG_Pos)
#define TIM6_7_EGR_UG TIM6_7_EGR_UG_Msk

/*CNT register.*/
#define TIM6_7_CNT_UIFCPY_Pos 31U
#define TIM6_7_CNT_UIFCPY_Msk (0x1UL << TIM6_7_CNT_UIFCPY_Pos)
#define TIM6_7_CNT_UIFCPY TIM6_7_CNT_UIFCPY_Msk

#define TIM6_7_CNT_Pos 0U
#define TIM6_7_CNT_Msk (0xFFFFUL << TIM6_7_CNT_Pos)
#define TIM6_7_CNT TIM6_7_CNT_Msk

/*PSC register.*/
#define TIM6_7_PSC_Pos 0U
#define TIM6_7_PSC_Msk (0xFFFFUL << TIM6_7_PSC_Pos)
#define TIM6_7_PSC TIM6_7_PSC_Msk

/*ARR register.*/
#define TIM6_7_ARR_Pos 0U
#define TIM6_7_ARR_Msk (0xFFFFUL << TIM6_7_ARR_Pos)
#define TIM6_7_ARR TIM6_7_ARR_Msk

/* ========================================================================= */
/* ============                      LPTIM                      ============ */
/* ========================================================================= */
typedef struct
{
	__IM uint32_t ISR; /*!< LPTIM Interrupt and Status Register. */
	__OM uint32_t ICR; /*!< LPTIM Interrupt Clear Register. */
	__IOM uint32_t IER; /*!< LPTIM Interrupt Enable Register. */
	__IOM uint32_t CFGR; /*!< LPTIM Configuration Register. */
	__IOM uint32_t CR; /*!< LPTIM Control Register. */
	__IOM uint32_t CMP; /*!< LPTIM Compare Register. */
	__IOM uint32_t ARR; /*!< LPTIM Auto-Reload Register. */
	__IM uint32_t CNT; /*!< LPTIM Counter. */
#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443) || defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
	__IOM uint32_t OR; /*!< LPTIM Option Register. */
#else
	uint32_t RESERVED0;
#endif
#if defined(STM32L412) || defined(STM32L422)
	__IOM uint32_t CFGR2; /*!< LPTIM Configuration Register 2. */
	__IOM uint32_t RCR; /*!< LPTIM Repetition Counter Register. */
#endif
}STM32L4xx_LPTIM_TypeDef;

/*ISR register.*/
#define LPTIM_ISR_REPOK_Pos 8U
#define LPTIM_ISR_REPOK_Msk (0x1UL << LPTIM_ISR_REPOK_Pos)
#define LPTIM_ISR_REPOK LPTIM_ISR_REPOK_Msk

#define LPTIM_ISR_UE_Pos 7U
#define LPTIM_ISR_UE_Msk (0x1UL << LPTIM_ISR_UE_Pos)
#define LPTIM_ISR_UE LPTIM_ISR_UE_Msk

#define LPTIM_ISR_DOWN_Pos 6U
#define LPTIM_ISR_DOWN_Msk (0x1UL << LPTIM_ISR_DOWN_Pos)
#define LPTIM_ISR_DOWN LPTIM_ISR_DOWN_Msk

#define LPTIM_ISR_UP_Pos 5U
#define LPTIM_ISR_UP_Msk (0x1UL << LPTIM_ISR_UP_Pos)
#define LPTIM_ISR_UP LPTIM_ISR_UP_Msk

#define LPTIM_ISR_ARROK_Pos 4U
#define LPTIM_ISR_ARROK_Msk (0x1UL << LPTIM_ISR_ARROK_Pos)
#define LPTIM_ISR_ARROK LPTIM_ISR_ARROK_Msk

#define LPTIM_ISR_CMPOK_Pos 3U
#define LPTIM_ISR_CMPOK_Msk (0x1UL << LPTIM_ISR_CMPOK_Pos)
#define LPTIM_ISR_CMPOK LPTIM_ISR_CMPOK_Msk

#define LPTIM_ISR_EXTTRIG_Pos 2U
#define LPTIM_ISR_EXTTRIG_Msk (0x1UL << LPTIM_ISR_EXTTRIG_Pos)
#define LPTIM_ISR_EXTTRIG LPTIM_ISR_EXTTRIG_Msk

#define LPTIM_ISR_ARRM_Pos 1U
#define LPTIM_ISR_ARRM_Msk (0x1UL << LPTIM_ISR_ARRM_Pos)
#define LPTIM_ISR_ARRM LPTIM_ISR_ARRM_Msk

#define LPTIM_ISR_CMPM_Pos 0U
#define LPTIM_ISR_CMPM_Msk (0x1UL << LPTIM_ISR_CMPM_Pos)
#define LPTIM_ISR_CMPM LPTIM_ISR_CMPM_Msk

/*ICR register.*/
#define LPTIM_ICR_REPOKCF_Pos 8U
#define LPTIM_ICR_REPOKCF_Msk (0x1UL << LPTIM_ICR_REPOKCF_Pos)
#define LPTIM_ICR_REPOKCF LPTIM_ICR_REPOKCF_Msk

#define LPTIM_ICR_UECF_Pos 7U
#define LPTIM_ICR_UECF_Msk (0x1UL << LPTIM_ICR_UECF_Pos)
#define LPTIM_ICR_UECF LPTIM_ICR_UECF_Msk

#define LPTIM_ICR_DOWNCF_Pos 6U
#define LPTIM_ICR_DOWNCF_Msk (0x1UL << LPTIM_ICR_DOWNCF_Pos)
#define LPTIM_ICR_DOWNCF LPTIM_ICR_DOWNCF_Msk

#define LPTIM_ICR_UPCF_Pos 5U
#define LPTIM_ICR_UPCF_Msk (0x1UL << LPTIM_ICR_UPCF_Pos)
#define LPTIM_ICR_UPCF LPTIM_ICR_UPCF_Msk

#define LPTIM_ICR_ARROKCF_Pos 4U
#define LPTIM_ICR_ARROKCF_Msk (0x1UL << LPTIM_ICR_ARROKCF_Pos)
#define LPTIM_ICR_ARROKCF LPTIM_ICR_ARROKCF_Msk

#define LPTIM_ICR_CMPOKCF_Pos 3U
#define LPTIM_ICR_CMPOKCF_Msk (0x1UL << LPTIM_ICR_CMPOKCF_Pos)
#define LPTIM_ICR_CMPOKCF LPTIM_ICR_CMPOKCF_Msk

#define LPTIM_ICR_EXTTRIGCF_Pos 2U
#define LPTIM_ICR_EXTTRIGCF_Msk (0x1UL << LPTIM_ICR_EXTTRIGCF_Pos)
#define LPTIM_ICR_EXTTRIGCF LPTIM_ICR_EXTTRIGCF_Msk

#define LPTIM_ICR_ARRMCF_Pos 1U
#define LPTIM_ICR_ARRMCF_Msk (0x1UL << LPTIM_ICR_ARRMCF_Pos)
#define LPTIM_ICR_ARRMCF LPTIM_ICR_ARRMCF_Msk

#define LPTIM_ICR_CMPMCF_Pos 0U
#define LPTIM_ICR_CMPMCF_Msk (0x1UL << LPTIM_ICR_CMPMCF_Pos)
#define LPTIM_ICR_CMPMCF LPTIM_ICR_CMPMCF_Msk

/*IER register.*/
#define LPTIM_IER_REPOKIE_Pos 8U
#define LPTIM_IER_REPOKIE_Msk (0x1UL << LPTIM_IER_REPOKIE_Pos)
#define LPTIM_IER_REPOKIE LPTIM_IER_REPOKIE_Msk

#define LPTIM_IER_UEIE_Pos 7U
#define LPTIM_IER_UEIE_Msk (0x1UL << LPTIM_IER_UEIE_Pos)
#define LPTIM_IER_UEIE LPTIM_IER_UEIE_Msk

#define LPTIM_IER_DOWNIE_Pos 6U
#define LPTIM_IER_DOWNIE_Msk (0x1UL << LPTIM_IER_DOWNIE_Pos)
#define LPTIM_IER_DOWNIE LPTIM_IER_DOWNIE_Msk

#define LPTIM_IER_UPIE_Pos 5U
#define LPTIM_IER_UPIE_Msk (0x1UL << LPTIM_IER_UPIE_Pos)
#define LPTIM_IER_UPIE LPTIM_IER_UPIE_Msk

#define LPTIM_IER_ARROKIE_Pos 4U
#define LPTIM_IER_ARROKIE_Msk (0x1UL << LPTIM_IER_ARROKIE_Pos)
#define LPTIM_IER_ARROKIE LPTIM_IER_ARROKIE_Msk

#define LPTIM_IER_CMPOKIE_Pos 3U
#define LPTIM_IER_CMPOKIE_Msk (0x1UL << LPTIM_IER_CMPOKIE_Pos)
#define LPTIM_IER_CMPOKIE LPTIM_IER_CMPOKIE_Msk

#define LPTIM_IER_EXTTRIGIE_Pos 2U
#define LPTIM_IER_EXTTRIGIE_Msk (0x1UL << LPTIM_IER_EXTTRIGIE_Pos)
#define LPTIM_IER_EXTTRIGIE LPTIM_IER_EXTTRIGIE_Msk

#define LPTIM_IER_ARRMIE_Pos 1U
#define LPTIM_IER_ARRMIE_Msk (0x1UL << LPTIM_IER_ARRMIE_Pos)
#define LPTIM_IER_ARRMIE LPTIM_IER_ARRMIE_Msk

#define LPTIM_IER_CMPMIE_Pos 0U
#define LPTIM_IER_CMPMIE_Msk (0x1UL << LPTIM_IER_CMPMIE_Pos)
#define LPTIM_IER_CMPMIE LPTIM_IER_CMPMIE_Msk

/*CFGR register.*/
#define LPTIM_CFGR_ENC_Pos 24U
#define LPTIM_CFGR_ENC_Msk (0x1UL << LPTIM_CFGR_ENC_Pos)
#define LPTIM_CFGR_ENC LPTIM_CFGR_ENC_Msk

#define LPTIM_CFGR_COUNTMODE_Pos 23U
#define LPTIM_CFGR_COUNTMODE_Msk (0x1UL << LPTIM_CFGR_COUNTMODE_Pos)
#define LPTIM_CFGR_COUNTMODE LPTIM_CFGR_COUNTMODE_Msk

#define LPTIM_CFGR_PRELOAD_Pos 22U
#define LPTIM_CFGR_PRELOAD_Msk (0x1UL << LPTIM_CFGR_PRELOAD_Pos)
#define LPTIM_CFGR_PRELOAD LPTIM_CFGR_PRELOAD_Msk

#define LPTIM_CFGR_WAVPOL_Pos 21U
#define LPTIM_CFGR_WAVPOL_Msk (0x1UL << LPTIM_CFGR_WAVPOL_Pos)
#define LPTIM_CFGR_WAVPOL LPTIM_CFGR_WAVPOL_Msk

#define LPTIM_CFGR_WAVE_Pos 20U
#define LPTIM_CFGR_WAVE_Msk (0x1UL << LPTIM_CFGR_WAVE_Pos)
#define LPTIM_CFGR_WAVE LPTIM_CFGR_WAVE_Msk

#define LPTIM_CFGR_TIMEOUT_Pos 19U
#define LPTIM_CFGR_TIMEOUT_Msk (0x1UL << LPTIM_CFGR_TIMEOUT_Pos)
#define LPTIM_CFGR_TIMEOUT LPTIM_CFGR_TIMEOUT_Msk

#define LPTIM_CFGR_TRIGEN_Pos 17U
#define LPTIM_CFGR_TRIGEN_Msk (0x3UL << LPTIM_CFGR_TRIGEN_Pos)
#define LPTIM_CFGR_TRIGEN LPTIM_CFGR_TRIGEN_Msk

#define LPTIM_CFGR_TRISEL_Pos 13U
#define LPTIM_CFGR_TRISEL_Msk (0x7UL << LPTIM_CFGR_TRISEL_Pos)
#define LPTIM_CFGR_TRISEL LPTIM_CFGR_TRISEL_Msk

#define LPTIM_CFGR_PRESC_Pos 9U
#define LPTIM_CFGR_PRESC_Msk (0x7UL << LPTIM_CFGR_PRESC_Pos)
#define LPTIM_CFGR_PRESC LPTIM_CFGR_PRESC_Msk

#define LPTIM_CFGR_TRGFLT_Pos 6U
#define LPTIM_CFGR_TRGFLT_Msk (0x3UL << LPTIM_CFGR_TRGFLT_Pos)
#define LPTIM_CFGR_TRGFLT LPTIM_CFGR_TRGFLT_Msk

#define LPTIM_CFGR_CKFLT_Pos 3U
#define LPTIM_CFGR_CKFLT_Msk (0x3UL << LPTIM_CFGR_CKFLT_Pos)
#define LPTIM_CFGR_CKFLT LPTIM_CFGR_CKFLT_Msk

#define LPTIM_CFGR_CKPOL_Pos 1U
#define LPTIM_CFGR_CKPOL_Msk (0x3UL << LPTIM_CFGR_CKPOL_Pos)
#define LPTIM_CFGR_CKPOL LPTIM_CFGR_CKPOL_Msk

#define LPTIM_CFGR_CKSEL_Pos 0U
#define LPTIM_CFGR_CKSEL_Msk (0x1UL << LPTIM_CFGR_CKSEL_Pos)
#define LPTIM_CFGR_CKSEL LPTIM_CFGR_CKSEL_Msk

/*CR register.*/
#define LPTIM_CR_RSTARE_Pos 4U
#define LPTIM_CR_RSTARE_Msk (0x1UL << LPTIM_CR_RSTARE_Pos)
#define LPTIM_CR_RSTARE LPTIM_CR_RSTARE_Msk

#define LPTIM_CR_COUNTRST_Pos 3U
#define LPTIM_CR_COUNTRST_Msk (0x1UL << LPTIM_CR_COUNTRST_Pos)
#define LPTIM_CR_COUNTRST LPTIM_CR_COUNTRST_Msk

#define LPTIM_CR_CNTSTRT_Pos 2U
#define LPTIM_CR_CNTSTRT_Msk (0x1UL << LPTIM_CR_CNTSTRT_Pos)
#define LPTIM_CR_CNTSTRT LPTIM_CR_CNTSTRT_Msk

#define LPTIM_CR_SNGSTRT_Pos 1U
#define LPTIM_CR_SNGSTRT_Msk (0x1UL << LPTIM_CR_SNGSTRT_Pos)
#define LPTIM_CR_SNGSTRT LPTIM_CR_SNGSTRT_Msk

#define LPTIM_CR_ENABLE_Pos 0U
#define LPTIM_CR_ENABLE_Msk (0x1UL << LPTIM_CR_ENABLE_Pos)
#define LPTIM_CR_ENABLE LPTIM_CR_ENABLE_Msk

/*CMP register.*/
#define LPTIM_CMP_Pos 0U
#define LPTIM_CMP_Msk (0xFFFFUL << LPTIM_CMP_Pos)
#define LPTIM_CMP LPTIM_CMP_Msk

/*ARR register.*/
#define LPTIM_ARR_Pos 0U
#define LPTIM_ARR_Msk (0xFFFFUL << LPTIM_ARR_Pos)
#define LPTIM_ARR LPTIM_ARR_Msk

/*CNT register.*/
#define LPTIM_CNT_Pos 0U
#define LPTIM_CNT_Msk (0xFFFFUL << LPTIM_CNT_Pos)
#define LPTIM_CNT LPTIM_CNT_Msk

#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443) || defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
/*OR register.*/
#define LPTIM_OR_1_Pos 1U
#define LPTIM_OR_1_Msk (0x1UL << LPTIM_OR_1_Pos)
#define LPTIM_OR_1 LPTIM_OR_1_Msk

#define LPTIM_OR_0_Pos 0U
#define LPTIM_OR_0_Msk (0x1UL << LPTIM_OR_0_Pos)
#define LPTIM_OR_0 LPTIM_OR_0_Msk
#endif

#if defined(STM32L412) || defined(STM32L422)
/*CFGR2 register.*/
#define LPTIM_CFGR2_IN2SEL_Pos 4U
#define LPTIM_CFGR2_IN2SEL_Msk (0x3UL << LPTIM_CFGR2_IN2SEL_Pos)

#define LPTIM_CFGR2_IN1SEL_Pos 0U
#define LPTIM_CFGR2_IN1SEL_Msk (0x3UL << LPTIM_CFGR2_IN1SEL_Pos)

/*RCR register.*/
#define LPTIM_RCR_REP_Pos 0U
#define LPTIM_RCR_REP_Msk (0xFFUL << LPTIM_RCR_REP_Pos)
#endif

/* ========================================================================= */
/* ============                       IWDG                       ============ */
/* ========================================================================= */
typedef struct
{
	__OM uint32_t KR; /*!< Key Register. */
	__IOM uint32_t PR; /*!< Prescaler Register. */
	__IOM uint32_t RLR; /*!< Reload Register. */
	__IM uint32_t SR; /*!< Status Register. */
	__IOM uint32_t WINR; /*!< Window Register. */
}STM32L4xx_IWDG_TypeDef;

/*KR register.*/
#define IWDG_KR_KEY_Pos 0U
#define IWDG_KR_KEY_Msk (0xFFFFUL << IWDG_KR_KEY_Pos)
#define IWDG_KR_KEY IWDG_KR_KEY_Msk
#define IWDG_KR_KEY_VALUE (0xAAAAUL << IWDG_KR_KEY_Pos)
#define IWDG_KR_KEY_ENABLE_REGISTER_ACCESS (0x5555UL << IWDG_KR_KEY_Pos)
#define IWDG_KR_KEY_START_WATCHDOG (0xCCCCUL << IWDG_KR_KEY_Pos)

/*PR register.*/
#define IWDG_PR_Pos 0U
#define IWDG_PR_Msk (0x7UL << IWDG_PR_Pos)
#define IWDG_PR IWDG_PR_Msk
#define IWDG_PR_DIV_4 (0x0UL << IWDG_PR_Pos)
#define IWDG_PR_DIV_8 (0x1UL << IWDG_PR_Pos)
#define IWDG_PR_DIV_16 (0x2UL << IWDG_PR_Pos)
#define IWDG_PR_DIV_32 (0x3UL << IWDG_PR_Pos)
#define IWDG_PR_DIV_64 (0x4UL << IWDG_PR_Pos)
#define IWDG_PR_DIV_128 (0x5UL << IWDG_PR_Pos)
#define IWDG_PR_DIV_256_1 (0x6UL << IWDG_PR_Pos)
#define IWDG_PR_DIV_256_2 (0x7UL << IWDG_PR_Pos)

/*RLR register.*/
#define IWDG_RLR_RL_Pos 0U
#define IWDG_RLR_RL_Msk (0xFFFUL << IWDG_RLR_RL_Pos)
#define IWDG_RLR_RL IWDG_RLR_RL_Msk

/*SR register.*/
#define IWDG_SR_WVU_Pos 2U
#define IWDG_SR_WVU_Msk (0x1UL << IWDG_SR_WVU_Pos)
#define IWDG_SR_WVU IWDG_SR_WVU_Msk

#define IWDG_SR_RVU_Pos 1U
#define IWDG_SR_RVU_Msk (0x1UL << IWDG_SR_RVU_Pos)
#define IWDG_SR_RVU IWDG_SR_RVU_Msk

#define IWDG_SR_PVU_Pos 0U
#define IWDG_SR_PVU_Msk (0x1UL << IWDG_SR_PVU_Pos)
#define IWDG_SR_PVU IWDG_SR_PVU_Msk

/*WINR register.*/
#define IWDG_WINR_WIN_Pos 0U
#define IWDG_WINR_WIN_Msk (0xFFFUL << IWDG_WINR_WIN_Pos)
#define IWDG_WINR_WIN IWDG_WINR_WIN_Msk

/* ========================================================================= */
/* ============                       WWDG                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR; /*!< Control Register. */
	__IOM uint32_t CFR; /*!< Configuration Register. */
	__IOM uint32_t SR; /*!< Status Register. */
}STM32L4xx_WWDG_TypeDef;

/*CR register.*/
#define WWDG_CR_WDGA_Pos 7U
#define WWDG_CR_WDGA_Msk (0x1UL << WWDG_CR_WDGA_Pos)
#define WWDG_CR_WDGA WWDG_CR_WDGA_Msk

#define WWDG_CR_T_Pos 0U
#define WWDG_CR_T_Msk (0x7FUL << WWDG_CR_T_Pos)
#define WWDG_CR_T WWDG_CR_T_Msk

/*CFR register.*/
#define WWDG_CFR_EWI_Pos 9U
#define WWDG_CFR_EWI_Msk (0x1UL << WWDG_CFR_EWI_Pos)
#define WWDG_CFR_EWI WWDG_CFR_EWI_Msk

#define WWDG_CFR_WDGTB_Pos 7U
#define WWDG_CFR_WDGTB_Msk (0x3UL << WWDG_CFR_WDGTB_Pos)
#define WWDG_CFR_WDGTB WWDG_CFR_WDGTB_Msk
#define WWDG_CFR_WDGTB_DIV_1 (0x0UL << WWDG_CFR_WDGTB_Pos)
#define WWDG_CFR_WDGTB_DIV_2 (0x1UL << WWDG_CFR_WDGTB_Pos)
#define WWDG_CFR_WDGTB_DIV_4 (0x2UL << WWDG_CFR_WDGTB_Pos)
#define WWDG_CFR_WDGTB_DIV_8 (0x3UL << WWDG_CFR_WDGTB_Pos)

#define WWDG_CFR_W_Pos 0U
#define WWDG_CFR_W_Msk (0x7FUL << WWDG_CFR_W_Pos)
#define WWDG_CFR_W WWDG_CFR_W_Msk

/*SR register.*/
#define WWDG_SR_EWIF_Pos 0U
#define WWDG_SR_EWIF_Msk (0x1UL << WWDG_SR_EWIF_Pos)
#define	 WWDG_SR_EWIF WWDG_SR_EWIF_Msk

#if defined(STM32L412) || defined(STM32L422)
/* ========================================================================= */
/* ============                       RTC                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t TR; /*!< RTC Time Register. */
	__IOM uint32_t DR; /*!< RTC Date Register. */
	__IM uint32_t SSR; /*!< RTC Sub Second Register. */
	__IOM uint32_t ICSR; /*!< RTC Initialization Control and Status Register. */
	__IOM uint32_t PRER; /*!< RTC Prescaler Register. */
	__IOM uint32_t WUTR; /*!< RTC Wakeup Timer Register. */
	__IOM uint32_t CR; /*!< RTC Control Register. */
	uint32_t RESERVED0[2];
	__OM uint32_t WPR; /*!< RTC Write Protection Register. */
	__IOM uint32_t CALR; /*!< RTC Calibration Register. */
	__OM uint32_t SHIFTR; /*!< RTC Shift control Register. */
	__IM uint32_t TSTR; /*!< RTC Timestamp Time Register. */
	__IM uint32_t TSDR; /*!< RTC Timestamp Date Register. */
	__IM uint32_t TSSSR; /*!< RTC Timestamp Sub Second Register. */
	uint32_t RESERVED1;
	__IOM uint32_t ALRMAR; /*!< RTC Alarm A Register. */
	__IOM uint32_t ALRMASSR; /*!< RTC Alarm A Sub Second Register. */
	__IOM uint32_t ALRMBR; /*!< RTC Alarm B Register. */
	__IOM uint32_t ALRMBSSR; /*!< RTC Alarm B Sub Second Register. */
	__IM uint32_t SR; /*!< RTC Status Register. */
	__IM uint32_t MISR; /*!< RTC Masked Interrupt Status Register. */
	uint32_t RESERVED2;
	__OM uint32_t SCR; /*!< RTC Status Clear Register. */
}STM32L4xx_RTC_TypeDef;

/*TR register.*/
#define RTC_TR_PM_Pos 22U
#define RTC_TR_PM_Msk (0x1UL << RTC_TR_PM_Pos)
#define RTC_TR_PM RTC_TR_PM_Msk

#define RTC_TR_HT_Pos 20U
#define RTC_TR_HT_Msk (0x3UL << RTC_TR_HT_Pos)
#define RTC_TR_HT RTC_TR_HT_Msk

#define RTC_TR_HU_Pos 16U
#define RTC_TR_HU_Msk (0xFUL << RTC_TR_HU_Pos)
#define RTC_TR_HU RTC_TR_HU_Msk

#define RTC_TR_MNT_Pos 12U
#define RTC_TR_MNT_Msk (0x7UL << RTC_TR_MNT_Pos)
#define RTC_TR_MNT RTC_TR_MNT_Msk

#define RTC_TR_MNU_Pos 7U
#define RTC_TR_MNU_Msk (0xFUL << RTC_TR_MNU_Pos)
#define RTC_TR_MNU RTC_TR_MNU_Msk

#define RTC_TR_ST_Pos 4U
#define RTC_TR_ST_Msk (0x7UL << RTC_TR_ST_Pos)
#define RTC_TR_ST RTC_TR_ST_Msk

#define RTC_TR_SU_Pos 0U
#define RTC_TR_SU_Msk (0xFUL << RTC_TR_SU_Pos)
#define RTC_TR_SU RTC_TR_SU_Msk

/*DR register.*/
#define RTC_DR_YT_Pos 20U
#define RTC_DR_YT_Msk (0xFUL << RTC_DR_YT_Pos)
#define RTC_DR_YT RTC_DR_YT_Msk

#define RTC_DR_YU_Pos 16U
#define RTC_DR_YU_Msk (0xFUL << RTC_DR_YU_Pos)
#define RTC_DR_YU RTC_DR_YU_Msk

#define RTC_DR_WDU_Pos 13U
#define RTC_DR_WDU_Msk (0x7UL << RTC_DR_WDU_Pos)
#define RTC_DR_WDU RTC_DR_WDU_Msk

#define RTC_DR_MT_Pos 12U
#define RTC_DR_MT_Msk (0x1UL << RTC_DR_MT_Pos)
#define RTC_DR_MT RTC_DR_MT_Msk

#define RTC_DR_MU_Pos 8U
#define RTC_DR_MU_Msk (0xFUL << RTC_DR_MU_Pos)
#define RTC_DR_MU RTC_DR_MU_Msk

#define RTC_DR_DT_Pos 4U
#define RTC_DR_DT_Msk (0x3UL << RTC_DR_DT_Pos)
#define RTC_DR_DT RTC_DR_DT_Msk

#define RTC_DR_DU_Pos 0U
#define RTC_DR_DU_Msk (0xFUL << RTC_DR_DU_Pos)
#define RTC_DR_DU RTC_DR_DU_Msk

/*SSR register.*/
#define RTC_SSR_SS_Pos 0U
#define RTC_SSR_SS_Msk (0xFFFFUL << RTC_SSR_SS_Pos)
#define RTC_SSR_SS RTC_SSR_SS_Msk

/*ICSR register.*/
#define RTC_ICSR_RECALPF_Pos 16U
#define RTC_ICSR_RECALPF_Msk (0x1UL << RTC_ICSR_RECALPF_Pos)
#define RTC_ICSR_RECALPF RTC_ICSR_RECALPF_Msk

#define RTC_ICSR_INIT_Pos 7U
#define RTC_ICSR_INIT_Msk (0x1UL << RTC_ICSR_INIT_Pos)
#define RTC_ICSR_INIT RTC_ICSR_INIT_Msk

#define RTC_ICSR_INITF_Pos 6U
#define RTC_ICSR_INITF_Msk (0x1UL << RTC_ICSR_INITF_Pos)
#define RTC_ICSR_INITF RTC_ICSR_INITF_Msk

#define RTC_ICSR_RSF_Pos 5U
#define RTC_ICSR_RSF_Msk (0x1UL << RTC_ICSR_RSF_Pos)
#define RTC_ICSR_RSF RTC_ICSR_RSF_Msk

#define RTC_ICSR_INITS_Pos 4U
#define RTC_ICSR_INITS_Msk (0x1UL << RTC_ICSR_INITS_Pos)
#define RTC_ICSR_INITS RTC_ICSR_INITS_Msk

#define RTC_ICSR_SHPF_Pos 3U
#define RTC_ICSR_SHPF_Msk (0x1UL << RTC_ICSR_SHPF_Pos)
#define RTC_ICSR_SHPF RTC_ICSR_SHPF_Msk

#define RTC_ICSR_WUTWF_Pos 2U
#define RTC_ICSR_WUTWF_Msk (0x1UL << RTC_ICSR_WUTWF_Pos)
#define RTC_ICSR_WUTWF RTC_ICSR_WUTWF_Msk

/*PRER register.*/
#define RTC_PRER_PREDIV_A_Pos 16U
#define RTC_PRER_PREDIV_A_Msk (0x7FUL << RTC_PRER_PREDIV_A_Pos)
#define RTC_PRER_PREDIV_A RTC_PRER_PREDIV_A_Msk

#define RTC_PRER_PREDIV_S_Pos 0U
#define RTC_PRER_PREDIV_S_Msk (0x7FFFUL << RTC_PRER_PREDIV_S_Pos)
#define RTC_PRER_PREDIV_S RTC_PRER_PREDIV_S_Msk

/*WUTR register.*/
#define RTC_WUTR_WUTOCLR_Pos 16U
#define RTC_WUTR_WUTOCLR_Msk (0xFFFFUL << RTC_WUTR_WUTOCLR_Pos)
#define RTC_WUTR_WUTOCLR RTC_WUTR_WUTOCLR_Msk

#define RTC_WUTR_WUT_Pos 0U
#define RTC_WUTR_WUT_Msk (0xFFFFUL << RTC_WUTR_WUT_Pos)
#define RTC_WUTR_WUT RTC_WUTR_WUT_Msk

/*CR register.*/
#define RTC_CR_OUT2EN_Pos 31U
#define RTC_CR_OUT2EN_Msk (0x1UL << RTC_CR_OUT2EN_Pos)
#define RTC_CR_OUT2EN RTC_CR_OUT2EN_Msk

#define RTC_CR_TAMPALRM_TYPE_Pos 30U
#define RTC_CR_TAMPALRM_TYPE_Msk (0x1UL << RTC_CR_TAMPALRM_TYPE_Pos)
#define RTC_CR_TAMPALRM_TYPE RTC_CR_TAMPALRM_TYPE_Msk

#define RTC_CR_TAMPALRM_PU_Pos 29U
#define RTC_CR_TAMPALRM_PU_Msk (0x1UL << RTC_CR_TAMPALRM_PU_Pos)
#define RTC_CR_TAMPALRM_PU RTC_CR_TAMPALRM_PU_Msk

#define RTC_CR_TAMPOE_Pos 26U
#define RTC_CR_TAMPOE_Msk (0x1UL << RTC_CR_TAMPOE_Pos)
#define RTC_CR_TAMPOE RTC_CR_TAMPOE_Msk

#define RTC_CR_TAMPTS_Pos 25U
#define RTC_CR_TAMPTS_Msk (0x1UL << RTC_CR_TAMPTS_Pos)
#define RTC_CR_TAMPTS RTC_CR_TAMPTS_Msk

#define RTC_CR_ITSE_Pos 24U
#define RTC_CR_ITSE_Msk (0x1UL << RTC_CR_ITSE_Pos)
#define RTC_CR_ITSE RTC_CR_ITSE_Msk

#define RTC_CR_COE_Pos 23U
#define RTC_CR_COE_Msk (0x1UL << RTC_CR_COE_Pos)
#define RTC_CR_COE RTC_CR_COE_Msk

#define RTC_CR_OSEL_Pos 21U
#define RTC_CR_OSEL_Msk (0x3UL << RTC_CR_OSEL_Pos)
#define RTC_CR_OSEL RTC_CR_OSEL_Msk

#define RTC_CR_POL_Pos 20U
#define RTC_CR_POL_Msk (0x1UL << RTC_CR_POL_Pos)
#define RTC_CR_POL RTC_CR_POL_Msk

#define RTC_CR_COSEL_Pos 19U
#define RTC_CR_COSEL_Msk (0x1UL << RTC_CR_COSEL_Pos)
#define RTC_CR_COSEL RTC_CR_COSEL_Msk

#define RTC_CR_BKP_Pos 18U
#define RTC_CR_BKP_Msk (0x1UL << RTC_CR_BKP_Pos)
#define RTC_CR_BKP RTC_CR_BKP_Msk

#define RTC_CR_SUB1H_Pos 17U
#define RTC_CR_SUB1H_Msk (0x1UL << RTC_CR_SUB1H_Pos)
#define RTC_CR_SUB1H RTC_CR_SUB1H_Msk

#define RTC_CR_ADD1H_Pos 16U
#define RTC_CR_ADD1H_Msk (0x1UL << RTC_CR_ADD1H_Pos)
#define RTC_CR_ADD1H RTC_CR_ADD1H_Msk

#define RTC_CR_TSIE_Pos 15U
#define RTC_CR_TSIE_Msk (0x1UL << RTC_CR_TSIE_Pos)
#define RTC_CR_TSIE RTC_CR_TSIE_Msk

#define RTC_CR_WUTIE_Pos 14U
#define RTC_CR_WUTIE_Msk (0x1UL << RTC_CR_WUTIE_Pos)
#define RTC_CR_WUTIE RTC_CR_WUTIE_Msk

#define RTC_CR_ALRBIE_Pos 13U
#define RTC_CR_ALRBIE_Msk (0x1UL << RTC_CR_ALRBIE_Pos)
#define RTC_CR_ALRBIE RTC_CR_ALRBIE_Msk

#define RTC_CR_ALRAIE_Pos 12U
#define RTC_CR_ALRAIE_Msk (0x1UL << RTC_CR_ALRAIE_Pos)
#define RTC_CR_ALRAIE RTC_CR_ALRAIE_Msk

#define RTC_CR_TSE_Pos 11U
#define RTC_CR_TSE_Msk (0x1UL << RTC_CR_TSE_Pos)
#define RTC_CR_TSE RTC_CR_TSE_Msk

#define RTC_CR_WUTE_Pos 10U
#define RTC_CR_WUTE_Msk (0x1UL << RTC_CR_WUTE_Pos)
#define RTC_CR_WUTE RTC_CR_WUTE_Msk

#define RTC_CR_ALRBE_Pos 9U
#define RTC_CR_ALRBE_Msk (0x1UL << RTC_CR_ALRBE_Pos)
#define RTC_CR_ALRBE RTC_CR_ALRBE_Msk

#define RTC_CR_ALRAE_Pos 8U
#define RTC_CR_ALRAE_Msk (0x1UL << RTC_CR_ALRAE_Pos)
#define RTC_CR_ALRAE RTC_CR_ALRAE_Msk

#define RTC_CR_FMT_Pos 6U
#define RTC_CR_FMT_Msk (0x1UL << RTC_CR_FMT_Pos)
#define RTC_CR_FMT RTC_CR_FMT_Msk

#define RTC_CR_BYPSHAD_Pos 5U
#define RTC_CR_BYPSHAD_Msk (0x1UL << RTC_CR_BYPSHAD_Pos)
#define RTC_CR_BYPSHAD RTC_CR_BYPSHAD_Msk

#define RTC_CR_REFCKON_Pos 4U
#define RTC_CR_REFCKON_Msk (0x1UL << RTC_CR_REFCKON_Pos)
#define RTC_CR_REFCKON RTC_CR_REFCKON_Msk

#define RTC_CR_TSEDGE_Pos 3U
#define RTC_CR_TSEDGE_Msk (0x1UL << RTC_CR_TSEDGE_Pos)
#define RTC_CR_TSEDGE RTC_CR_TSEDGE_Msk

#define RTC_CR_WUCKSEL_Pos 0U
#define RTC_CR_WUCKSEL_Msk (0x7UL << RTC_CR_WUCKSEL_Pos)
#define RTC_CR_WUCKSEL RTC_CR_WUCKSEL_Msk

/*WPR register.*/
#define RTC_WPR_KEY_KEY_Pos 0U
#define RTC_WPR_KEY_KEY_Msk (0xFFUL << RTC_WPR_KEY_KEY_Pos)
#define RTC_WPR_KEY_KEY RTC_WPR_KEY_KEY_Msk

/*CALR register.*/
#define RTC_CALR_CALP_Pos 15U
#define RTC_CALR_CALP_Msk (0x1UL << RTC_CALR_CALP_Pos)
#define RTC_CALR_CALP RTC_CALR_CALP_Msk

#define RTC_CALR_CALW8_Pos 14U
#define RTC_CALR_CALW8_Msk (0x1UL << RTC_CALR_CALW8_Pos)
#define RTC_CALR_CALW8 RTC_CALR_CALW8_Msk

#define RTC_CALR_CALW16_Pos 13U
#define RTC_CALR_CALW16_Msk (0x1UL << RTC_CALR_CALW16_Pos)
#define RTC_CALR_CALW16 RTC_CALR_CALW16_Msk

#define RTC_CALR_LPCAL_Pos 12U
#define RTC_CALR_LPCAL_Msk (0x1UL << RTC_CALR_LPCAL_Pos)
#define RTC_CALR_LPCAL RTC_CALR_LPCAL_Msk

#define RTC_CALR_CALM_Pos 0U
#define RTC_CALR_CALM_Msk (0x1FFUL << RTC_CALR_CALM_Pos)
#define RTC_CALR_CALM RTC_CALR_CALM_Msk

/*SHIFTR register.*/
#define RTC_SHIFTR_ADD1S_Pos 31U
#define RTC_SHIFTR_ADD1S_Msk (0x1UL << RTC_SHIFTR_ADD1S_Pos)
#define RTC_SHIFTR_ADD1S RTC_SHIFTR_ADD1S_Msk

#define RTC_SHIFTR_SUBFS_Pos 0U
#define RTC_SHIFTR_SUBFS_Msk (0x7FFFUL << RTC_SHIFTR_SUBFS_Pos)
#define RTC_SHIFTR_SUBFS RTC_SHIFTR_SUBFS_Msk

/*TSTR register.*/
#define RTC_TSTR_PM_Pos 22U
#define RTC_TSTR_PM_Msk (0x1UL << RTC_TSTR_PM_Pos)
#define RTC_TSTR_PM RTC_TSTR_PM_Msk

#define RTC_TSTR_HT_Pos 20U
#define RTC_TSTR_HT_Msk (0x3UL << RTC_TSTR_HT_Pos)
#define RTC_TSTR_HT RTC_TSTR_HT_Msk

#define RTC_TSTR_HU_Pos 16U
#define RTC_TSTR_HU_Msk (0xFUL << RTC_TSTR_HU_Pos)
#define RTC_TSTR_HU RTC_TSTR_HU_Msk

#define RTC_TSTR_MNT_Pos 12U
#define RTC_TSTR_MNT_Msk (0x7UL << RTC_TSTR_MNT_Pos)
#define RTC_TSTR_MNT RTC_TSTR_MNT_Msk

#define RTC_TSTR_MNU_Pos 8U
#define RTC_TSTR_MNU_Msk (0xFUL << RTC_TSTR_MNU_Pos)
#define RTC_TSTR_MNU RTC_TSTR_MNU_Msk

#define RTC_TSTR_ST_Pos 4U
#define RTC_TSTR_ST_Msk (0x7UL << RTC_TSTR_ST_Pos)
#define RTC_TSTR_ST RTC_TSTR_ST_Msk

#define RTC_TSTR_SU_Pos 0U
#define RTC_TSTR_SU_Msk (0xFUL << RTC_TSTR_SU_Pos)
#define RTC_TSTR_SU RTC_TSTR_SU_Msk

/*TSDR register.*/
#define RTC_TSDR_WDU_Pos 13U
#define RTC_TSDR_WDU_Msk (0x7UL << RTC_TSDR_WDU_Pos)
#define RTC_TSDR_WDU RTC_TSDR_WDU_Msk

#define RTC_TSDR_MT_Pos 12U
#define RTC_TSDR_MT_Msk (0x1UL << RTC_TSDR_MT_Pos)
#define RTC_TSDR_MT RTC_TSDR_MT_Msk

#define RTC_TSDR_MU_Pos 8U
#define RTC_TSDR_MU_Msk (0xFUL << RTC_TSDR_MU_Pos)
#define RTC_TSDR_MU RTC_TSDR_MU_Msk

#define RTC_TSDR_DT_Pos 4U
#define RTC_TSDR_DT_Msk (0x3UL << RTC_TSDR_DT_Pos)
#define RTC_TSDR_DT RTC_TSDR_DT_Msk

#define RTC_TSDR_DU_Pos 0U
#define RTC_TSDR_DU_Msk (0xFUL << RTC_TSDR_DU_Pos)
#define RTC_TSDR_DU RTC_TSDR_DU_Msk

/*TSSSR register.*/
#define RTC_TSSSR_SS_Pos 0U
#define RTC_TSSSR_SS_Msk (0xFFFFUL << RTC_TSSSR_SS_Pos)
#define RTC_TSSSR_SS RTC_TSSSR_SS_Msk

/*ALRMAR register.*/
#define RTC_ALRMAR_MSK4_Pos 31U
#define RTC_ALRMAR_MSK4_Msk (0x1UL << RTC_ALRMAR_MSK4_Pos)
#define RTC_ALRMAR_MSK4 RTC_ALRMAR_MSK4_Msk

#define RTC_ALRMAR_WDSEL_Pos 30U
#define RTC_ALRMAR_WDSEL_Msk (0x1UL << RTC_ALRMAR_WDSEL_Pos)
#define RTC_ALRMAR_WDSEL RTC_ALRMAR_WDSEL_Msk

#define RTC_ALRMAR_DT_Pos 28U
#define RTC_ALRMAR_DT_Msk (0x3UL << RTC_ALRMAR_DT_Pos)
#define RTC_ALRMAR_DT RTC_ALRMAR_DT_Msk

#define RTC_ALRMAR_DU_Pos 24U
#define RTC_ALRMAR_DU_Msk (0xFUL << RTC_ALRMAR_DU_Pos)
#define RTC_ALRMAR_DU RTC_ALRMAR_DU_Msk

#define RTC_ALRMAR_MSK3_Pos 23U
#define RTC_ALRMAR_MSK3_Msk (0x1UL << RTC_ALRMAR_MSK3_Pos)
#define RTC_ALRMAR_MSK3 RTC_ALRMAR_MSK3_Msk

#define RTC_ALRMAR_PM_Pos 22U
#define RTC_ALRMAR_PM_Msk (0x1UL << RTC_ALRMAR_PM_Pos)
#define RTC_ALRMAR_PM RTC_ALRMAR_PM_Msk

#define RTC_ALRMAR_HT_Pos 20U
#define RTC_ALRMAR_HT_Msk (0x3UL << RTC_ALRMAR_HT_Pos)
#define RTC_ALRMAR_HT RTC_ALRMAR_HT_Msk

#define RTC_ALRMAR_HU_Pos 16U
#define RTC_ALRMAR_HU_Msk (0xFUL << RTC_ALRMAR_HU_Pos)
#define RTC_ALRMAR_HU RTC_ALRMAR_HU_Msk

#define RTC_ALRMAR_MSK2_Pos 15U
#define RTC_ALRMAR_MSK2_Msk (0x1UL << RTC_ALRMAR_MSK2_Pos)
#define RTC_ALRMAR_MSK2 RTC_ALRMAR_MSK2_Msk

#define RTC_ALRMAR_MNT_Pos 12U
#define RTC_ALRMAR_MNT_Msk (0x7UL << RTC_ALRMAR_MNT_Pos)
#define RTC_ALRMAR_MNT RTC_ALRMAR_MNT_Msk

#define RTC_ALRMAR_MNU_Pos 8U
#define RTC_ALRMAR_MNU_Msk (0xFUL << RTC_ALRMAR_MNU_Pos)
#define RTC_ALRMAR_MNU RTC_ALRMAR_MNU_Msk

#define RTC_ALRMAR_MSK1_Pos 7U
#define RTC_ALRMAR_MSK1_Msk (0x1UL << RTC_ALRMAR_MSK1_Pos)
#define RTC_ALRMAR_MSK1 RTC_ALRMAR_MSK1_Msk

#define RTC_ALRMAR_ST_Pos 4U
#define RTC_ALRMAR_ST_Msk (0x7UL << RTC_ALRMAR_ST_Pos)
#define RTC_ALRMAR_ST RTC_ALRMAR_ST_Msk

#define RTC_ALRMAR_SU_Pos 0U
#define RTC_ALRMAR_SU_Msk (0xFUL << RTC_ALRMAR_SU_Pos)
#define RTC_ALRMAR_SU RTC_ALRMAR_SU_Msk

/*ALRMASSR registers.*/
#define RTC_ALRMASSR_MASKSS_Pos 24U
#define RTC_ALRMASSR_MASKSS_Msk (0xFUL << RTC_ALRMASSR_MASKSS_Pos)
#define RTC_ALRMASSR_MASKSS RTC_ALRMASSR_MASKSS_Msk

#define RTC_ALRMASSR_SS_Pos 0U
#define RTC_ALRMASSR_SS_Msk (0x7FFFUL << RTC_ALRMASSR_SS_Pos)
#define RTC_ALRMASSR_SS RTC_ALRMASSR_SS_Msk

/*ALRMBR register.*/
#define RTC_ALRMBR_MSK4_Pos 31U
#define RTC_ALRMBR_MSK4_Msk (0x1UL << RTC_ALRMBR_MSK4_Pos)
#define RTC_ALRMBR_MSK4 RTC_ALRMBR_MSK4_Msk

#define RTC_ALRMBR_WDSEL_Pos 30U
#define RTC_ALRMBR_WDSEL_Msk (0x1UL << RTC_ALRMBR_WDSEL_Pos)
#define RTC_ALRMBR_WDSEL RTC_ALRMBR_WDSEL_Msk

#define RTC_ALRMBR_DT_Pos 28U
#define RTC_ALRMBR_DT_Msk (0x3UL << RTC_ALRMBR_DT_Pos)
#define RTC_ALRMBR_DT RTC_ALRMBR_DT_Msk

#define RTC_ALRMBR_DU_Pos 24U
#define RTC_ALRMBR_DU_Msk (0xFUL << RTC_ALRMBR_DU_Pos)
#define RTC_ALRMBR_DU RTC_ALRMBR_DU_Msk

#define RTC_ALRMBR_MSK3_Pos 23U
#define RTC_ALRMBR_MSK3_Msk (0x1UL << RTC_ALRMBR_MSK3_Pos)
#define RTC_ALRMBR_MSK3 RTC_ALRMBR_MSK3_Msk

#define RTC_ALRMBR_PM_Pos 22U
#define RTC_ALRMBR_PM_Msk (0x1UL << RTC_ALRMBR_PM_Pos)
#define RTC_ALRMBR_PM RTC_ALRMBR_PM_Msk

#define RTC_ALRMBR_HT_Pos 20U
#define RTC_ALRMBR_HT_Msk (0x3UL << RTC_ALRMBR_HT_Pos)
#define RTC_ALRMBR_HT RTC_ALRMBR_HT_Msk

#define RTC_ALRMBR_HU_Pos 16U
#define RTC_ALRMBR_HU_Msk (0xFUL << RTC_ALRMBR_HU_Pos)
#define RTC_ALRMBR_HU RTC_ALRMBR_HU_Msk

#define RTC_ALRMBR_MSK2_Pos 15U
#define RTC_ALRMBR_MSK2_Msk (0x1UL << RTC_ALRMBR_MSK2_Pos)
#define RTC_ALRMBR_MSK2 RTC_ALRMBR_MSK2_Msk

#define RTC_ALRMBR_MNT_Pos 12U
#define RTC_ALRMBR_MNT_Msk (0x7UL << RTC_ALRMBR_MNT_Pos)
#define RTC_ALRMBR_MNT RTC_ALRMBR_MNT_Msk

#define RTC_ALRMBR_MNU_Pos 8U
#define RTC_ALRMBR_MNU_Msk (0xFUL << RTC_ALRMBR_MNU_Pos)
#define RTC_ALRMBR_MNU RTC_ALRMBR_MNU_Msk

#define RTC_ALRMBR_MSK1_Pos 7U
#define RTC_ALRMBR_MSK1_Msk (0x1UL << RTC_ALRMBR_MSK1_Pos)
#define RTC_ALRMBR_MSK1 RTC_ALRMBR_MSK1_Msk

#define RTC_ALRMBR_ST_Pos 4U
#define RTC_ALRMBR_ST_Msk (0x7UL << RTC_ALRMBR_ST_Pos)
#define RTC_ALRMBR_ST RTC_ALRMBR_ST_Msk

#define RTC_ALRMBR_SU_Pos 0U
#define RTC_ALRMBR_SU_Msk (0xFUL << RTC_ALRMBR_SU_Pos)
#define RTC_ALRMBR_SU RTC_ALRMBR_SU_Msk

/*ALRMBSSR registers.*/
#define RTC_ALRMBSSR_MASKSS_Pos 24U
#define RTC_ALRMBSSR_MASKSS_Msk (0xFUL << RTC_ALRMBSSR_MASKSS_Pos)
#define RTC_ALRMBSSR_MASKSS RTC_ALRMBSSR_MASKSS_Msk

#define RTC_ALRMBSSR_SS_Pos 0U
#define RTC_ALRMBSSR_SS_Msk (0x7FFFUL << RTC_ALRMBSSR_SS_Pos)
#define RTC_ALRMBSSR_SS RTC_ALRMBSSR_SS_Msk

/*SR register.*/
#define RTC_SR_ITSF_Pos 5U
#define RTC_SR_ITSF_Msk (0x1UL << RTC_SR_ITSF_Pos)
#define RTC_SR_ITSF RTC_SR_ITSF_Msk

#define RTC_SR_TSOVF_Pos 4U
#define RTC_SR_TSOVF_Msk (0x1UL << RTC_SR_TSOVF_Pos)
#define RTC_SR_TSOVF RTC_SR_TSOVF_Msk

#define RTC_SR_TSF_Pos 3U
#define RTC_SR_TSF_Msk (0x1UL << RTC_SR_TSF_Pos)
#define RTC_SR_TSF RTC_SR_TSF_Msk

#define RTC_SR_WUTF_Pos 2U
#define RTC_SR_WUTF_Msk (0x1UL << RTC_SR_WUTF_Pos)
#define RTC_SR_WUTF RTC_SR_WUTF_Msk

#define RTC_SR_ALRBF_Pos 1U
#define RTC_SR_ALRBF_Msk (0x1UL << RTC_SR_ALRBF_Pos)
#define RTC_SR_ALRBF RTC_SR_ALRBF_Msk

#define RTC_SR_ALRAF_Pos 0U
#define RTC_SR_ALRAF_Msk (0x1UL << RTC_SR_ALRAF_Pos)
#define RTC_SR_ALRAF RTC_SR_ALRAF_Msk

/*MISR register.*/
#define RTC_MISR_ITSMF_Pos 5U
#define RTC_MISR_ITSMF_Msk (0x1UL << RTC_MISR_ITSMF_Pos)
#define RTC_MISR_ITSMF RTC_MISR_ITSMF_Msk

#define RTC_MISR_TSOVMF_Pos 4U
#define RTC_MISR_TSOVMF_Msk (0x1UL << RTC_MISR_TSOVMF_Pos)
#define RTC_MISR_TSOVMF RTC_MISR_TSOVMF_Msk

#define RTC_MISR_TSMF_Pos 3U
#define RTC_MISR_TSMF_Msk (0x1UL << RTC_MISR_TSMF_Pos)
#define RTC_MISR_TSMF RTC_MISR_TSMF_Msk

#define RTC_MISR_WUTMF_Pos 2U
#define RTC_MISR_WUTMF_Msk (0x1UL << RTC_MISR_WUTMF_Pos)
#define RTC_MISR_WUTMF RTC_MISR_WUTMF_Msk

#define RTC_MISR_ALRBMF_Pos 1U
#define RTC_MISR_ALRBMF_Msk (0x1UL << RTC_MISR_ALRBMF_Pos)
#define RTC_MISR_ALRBMF RTC_MISR_ALRBMF_Msk

#define RTC_MISR_ALRAMF_Pos 0U
#define RTC_MISR_ALRAMF_Msk (0x1UL << RTC_MISR_ALRAMF_Pos)
#define RTC_MISR_ALRAMF RTC_MISR_ALRAMF_Msk

/*SCR register.*/
#define RTC_SCR_CITSF_Pos 5U
#define RTC_SCR_CITSF_Msk (0x1UL << RTC_SCR_CITSF_Pos)
#define RTC_SCR_CITSF RTC_SCR_CITSF_Msk

#define RTC_SCR_CTSOVF_Pos 4U
#define RTC_SCR_CTSOVF_Msk (0x1UL << RTC_SCR_CTSOVF_Pos)
#define RTC_SCR_CTSOVF RTC_SCR_CTSOVF_Msk

#define RTC_SCR_CTSF_Pos 3U
#define RTC_SCR_CTSF_Msk (0x1UL << RTC_SCR_TSF_Pos)
#define RTC_SCR_CTSF RTC_SCR_CTSF_Msk

#define RTC_SCR_CWUTF_Pos 2U
#define RTC_SCR_CWUTF_Msk (0x1UL << RTC_SCR_CWUTF_Pos)
#define RTC_SCR_CWUTF RTC_SCR_CWUTF_Msk

#define RTC_SCR_CALRBMF_Pos 1U
#define RTC_SCR_CALRBMF_Msk (0x1UL << RTC_SCR_CALRBF_Pos)
#define RTC_SCR_CALRBMF RTC_SCR_CALRBMF_Msk

#define RTC_SCR_CALRAF_Pos 0U
#define RTC_SCR_CALRAF_Msk (0x1UL << RTC_SCR_CALRAF_Pos)
#define RTC_SCR_CALRAF RTC_SCR_CALRAF_Msk
#endif

/* ========================================================================= */
/* ============                      TAMP                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR1; /*!< TAMP Control Register 1. */
	__IOM uint32_t CR2; /*!< TAMP Control Register 2. */
	uint32_t RESERVED0;
	__IOM uint32_t FLTCR; /*!< TAMP Filter Control Register. */
	uint32_t RESERVED1[7];
	__IOM uint32_t IER; /*!< TAMP Interrupt Enable Register. */
	__IM uint32_t SR; /*!< TAMP Status Register. */
	__IM uint32_t MISR; /*!< TAMP Masked Interrupt Status Register. */
	uint32_t RESERVED2;
	__OM uint32_t SCR; /*!< TAMP Status Clear Register. */
	uint32_t RESERVED3[192U];
	__IOM uint32_t BKP0R; /*!< TAMP Backup 0 Register. */
	__IOM uint32_t BKP1R; /*!< TAMP Backup 1 Register. */
	__IOM uint32_t BKP2R; /*!< TAMP Backup 2 Register. */
	__IOM uint32_t BKP3R; /*!< TAMP Backup 3 Register. */
	__IOM uint32_t BKP4R; /*!< TAMP Backup 4 Register. */
	__IOM uint32_t BKP5R; /*!< TAMP Backup 5 Register. */
	__IOM uint32_t BKP6R; /*!< TAMP Backup 6 Register. */
	__IOM uint32_t BKP7R; /*!< TAMP Backup 7 Register. */
	__IOM uint32_t BKP8R; /*!< TAMP Backup 8 Register. */
	__IOM uint32_t BKP9R; /*!< TAMP Backup 9 Register. */
	__IOM uint32_t BKP10R; /*!< TAMP Backup 10 Register. */
	__IOM uint32_t BKP11R; /*!< TAMP Backup 11 Register. */
	__IOM uint32_t BKP12R; /*!< TAMP Backup 12 Register. */
	__IOM uint32_t BKP13R; /*!< TAMP Backup 13 Register. */
	__IOM uint32_t BKP14R; /*!< TAMP Backup 14 Register. */
	__IOM uint32_t BKP15R; /*!< TAMP Backup 15 Register. */
	__IOM uint32_t BKP16R; /*!< TAMP Backup 16 Register. */
	__IOM uint32_t BKP17R; /*!< TAMP Backup 17 Register. */
	__IOM uint32_t BKP18R; /*!< TAMP Backup 18 Register. */
	__IOM uint32_t BKP19R; /*!< TAMP Backup 19 Register. */
	__IOM uint32_t BKP20R; /*!< TAMP Backup 20 Register. */
	__IOM uint32_t BKP21R; /*!< TAMP Backup 21 Register. */
	__IOM uint32_t BKP22R; /*!< TAMP Backup 22 Register. */
	__IOM uint32_t BKP23R; /*!< TAMP Backup 23 Register. */
	__IOM uint32_t BKP24R; /*!< TAMP Backup 24 Register. */
	__IOM uint32_t BKP25R; /*!< TAMP Backup 25 Register. */
	__IOM uint32_t BKP26R; /*!< TAMP Backup 26 Register. */
	__IOM uint32_t BKP27R; /*!< TAMP Backup 27 Register. */
	__IOM uint32_t BKP28R; /*!< TAMP Backup 28 Register. */
	__IOM uint32_t BKP29R; /*!< TAMP Backup 29 Register. */
	__IOM uint32_t BKP30R; /*!< TAMP Backup 30 Register. */
	__IOM uint32_t BKP31R; /*!< TAMP Backup 31 Register. */	
}STM32L4xx_TAMP_TypeDef;

/*CR1 register.*/
#define TAMP_CR1_TAMP2E_Pos 1U
#define TAMP_CR1_TAMP2E_Msk (0x1UL << TAMP_CR1_TAMP2E_Pos)
#define TAMP_CR1_TAMP2E TAMP_CR1_TAMP2E_Msk

#define TAMP_CR1_TAMP1E_Pos 0U
#define TAMP_CR1_TAMP1E_Msk (0x1UL << TAMP_CR1_TAMP1E_Pos)
#define TAMP_CR1_TAMP1E TAMP_CR1_TAMP1E_Msk

/*CR2 register.*/
#define TAMP_CR2_TAMP2TRG_Pos 25U
#define TAMP_CR2_TAMP2TRG_Msk (0x1UL << TAMP_CR2_TAMP2TRG_Pos)
#define TAMP_CR2_TAMP2TRG TAMP_CR2_TAMP2TRG_Msk

#define TAMP_CR2_TAMP1TRG_Pos 24U
#define TAMP_CR2_TAMP1TRG_Msk (0x1UL << TAMP_CR2_TAMP1TRG_Pos)
#define TAMP_CR2_TAMP1TRG TAMP_CR2_TAMP1TRG_Msk

#define TAMP_CR2_BKERASE_Pos 23U
#define TAMP_CR2_BKERASE_Msk (0x1UL << TAMP_CR2_BKERASE_Pos)
#define TAMP_CR2_BKERASE TAMP_CR2_BKERASE_Msk

#define TAMP_CR2_TAMP2MSK_Pos 17U
#define TAMP_CR2_TAMP2MSK_Msk (0x1UL << TAMP_CR2_TAMP2MSK_Pos)
#define TAMP_CR2_TAMP2MSK TAMP_CR2_TAMP2MSK_Msk

#define TAMP_CR2_TAMP1MSK_Pos 16U
#define TAMP_CR2_TAMP1MSK_Msk (0x1UL << TAMP_CR2_TAMP1MSK_Pos)
#define TAMP_CR2_TAMP1MSK TAMP_CR2_TAMP1MSK_Msk

#define TAMP_CR2_TAMP2NOER_Pos 1U
#define TAMP_CR2_TAMP2NOER_Msk (0x1UL << TAMP_CR2_TAMP2NOER_Pos)
#define TAMP_CR2_TAMP2NOER TAMP_CR2_TAMP2NOER_Msk

#define TAMP_CR2_TAMP1NOER_Pos 0U
#define TAMP_CR2_TAMP1NOER_Msk (0x1UL << TAMP_CR2_TAMP1NOER_Pos)
#define TAMP_CR2_TAMP1NOER TAMP_CR2_TAMP1NOER_Msk

/*FLTCR register.*/
#define TAMP_FLTCR_TAMPPUDIS_Pos 7U
#define TAMP_FLTCR_TAMPPUDIS_Msk (0x1UL << TAMP_FLTCR_TAMPPUDIS_Pos)
#define TAMP_FLTCR_TAMPPUDIS TAMP_FLTCR_TAMPPUDIS_Msk

#define TAMP_FLTCR_TAMPPRCH_Pos 5U
#define TAMP_FLTCR_TAMPPRCH_Msk (0x3UL << TAMP_FLTCR_TAMPPRCH_Pos)
#define TAMP_FLTCR_TAMPPRCH TAMP_FLTCR_TAMPPRCH_Msk

#define TAMP_FLTCR_TAMPFLT_Pos 3U
#define TAMP_FLTCR_TAMPFLT_Msk (0x3UL << TAMP_FLTCR_TAMPFLT_Pos)
#define TAMP_FLTCR_TAMPFLT TAMP_FLTCR_TAMPFLT_Msk

#define TAMP_FLTCR_TAMPFREQ_Pos 0U
#define TAMP_FLTCR_TAMPFREQ_Msk (0x7UL << TAMP_FLTCR_TAMPFREQ_Pos)
#define TAMP_FLTCR_TAMPFREQ TAMP_FLTCR_TAMPFREQ_Msk

/*IER register.*/
#define TAMP_IER_TAMP2IER_Pos 1U
#define TAMP_IER_TAMP2IER_Msk (0x1UL << TAMP_IER_TAMP2IER_Pos)
#define TAMP_IER_TAMP2IER TAMP_IER_TAMP2IER_Msk

#define TAMP_IER_TAMP1IER_Pos 0U
#define TAMP_IER_TAMP1IER_Msk (0x1UL << TAMP_IER_TAMP1IER_Pos)
#define TAMP_IER_TAMP1IER TAMP_IER_TAMP1IER_Msk

/*SR register.*/
#define TAMP_SR_TAMP2F_Pos 1U
#define TAMP_SR_TAMP2F_Msk (0x1UL << TAMP_SR_TAMP2F_Pos)
#define TAMP_SR_TAMP2F TAMP_SR_TAMP2F_Msk

#define TAMP_SR_TAMP1F_Pos 0U
#define TAMP_SR_TAMP1F_Msk (0x1UL << TAMP_SR_TAMP1F_Pos)
#define TAMP_SR_TAMP1F TAMP_SR_TAMP1F_Msk

/*MISR register.*/
#define TAMP_MISR_TAMP2MF_Pos 1U
#define TAMP_MISR_TAMP2MF_Msk (0x1UL << TAMP_MISR_TAMP2MF_Pos)
#define TAMP_MISR_TAMP2MF TAMP_MISR_TAMP2MF_Msk

#define TAMP_MISR_TAMP1MF_Pos 0U
#define TAMP_MISR_TAMP1MF_Msk (0x1UL << TAMP_MISR_TAMP1MF_Pos)
#define TAMP_MISR_TAMP1MF TAMP_MISR_TAMP1MF_Msk

/*SCR register.*/
#define TAMP_SCR_CTAMP2F_Pos 1U
#define TAMP_SCR_CTAMP2F_Msk (0x1UL << TAMP_SCR_CTAMP2F_Pos)
#define TAMP_SCR_CTAMP2F TAMP_SCR_CTAMP2F_Msk

#define TAMP_SCR_CTAMP1F_Pos 0U
#define TAMP_SCR_CTAMP1F_Msk (0x1UL << TAMP_SCR_CTAMP1F_Pos)
#define TAMP_SCR_CTAMP1F TAMP_SCR_CTAMP1F_Msk

#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
/* ========================================================================= */
/* ============                       RTC                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t TR; /*!< RTC Time Register. */
	__IOM uint32_t DR; /*!< RTC Date Register. */
	__IOM uint32_t CR; /*!< RTC Control Register. */
	__IOM uint32_t ISR; /*!< RTC Interrupt and Status Register. */
	__IOM uint32_t PRER; /*!< RTC Prescaler Register. */
	__IOM uint32_t WUTR; /*!< RTC Wakeup Timer Register. */
	uint32_t RESERVED0;
	__IOM uint32_t ALRMAR; /*!< RTC Alarm A Register. */
	__IOM uint32_t ALRMBR; /*!< RTC Alarm B Register. */
	__OM uint32_t WPR; /*!< RTC Write Protection Register. */
	__IM uint32_t SSR; /*!< RTC Sub Second Register. */
	__OM uint32_t SHIFTR; /*!< RTC Shift control Register. */
	__IM uint32_t TSTR; /*!< RTC Timestamp Time Register. */
	__IM uint32_t TSDR; /*!< RTC Timestamp Date Register. */
	__IM uint32_t TSSSR; /*!< RTC Timestamp Sub Second Register. */
	__IOM uint32_t CALR; /*!< RTC Calibration Register. */
	__IOM uint32_t TAMPCR; /*!< RTC Tamper Configuration Register. */
	__IOM uint32_t ALRMASSR; /*!< RTC Alarm A Sub Second Register. */
	__IOM uint32_t ALRMBSSR; /*!< RTC Alarm B Sub Second Register. */
	__IOM uint32_t OR; /*!< RTC Option Register. */
	__IOM uint32_t BKP0R; /*!< RTC Backup 0 Register. */
	__IOM uint32_t BKP1R; /*!< RTC Backup 1 Register. */
	__IOM uint32_t BKP2R; /*!< RTC Backup 2 Register. */
	__IOM uint32_t BKP3R; /*!< RTC Backup 3 Register. */
	__IOM uint32_t BKP4R; /*!< RTC Backup 4 Register. */
	__IOM uint32_t BKP5R; /*!< RTC Backup 5 Register. */
	__IOM uint32_t BKP6R; /*!< RTC Backup 6 Register. */
	__IOM uint32_t BKP7R; /*!< RTC Backup 7 Register. */
	__IOM uint32_t BKP8R; /*!< RTC Backup 8 Register. */
	__IOM uint32_t BKP9R; /*!< RTC Backup 9 Register. */
	__IOM uint32_t BKP10R; /*!< RTC Backup 10 Register. */
	__IOM uint32_t BKP11R; /*!< RTC Backup 11 Register. */
	__IOM uint32_t BKP12R; /*!< RTC Backup 12 Register. */
	__IOM uint32_t BKP13R; /*!< RTC Backup 13 Register. */
	__IOM uint32_t BKP14R; /*!< RTC Backup 14 Register. */
	__IOM uint32_t BKP15R; /*!< RTC Backup 15 Register. */
	__IOM uint32_t BKP16R; /*!< RTC Backup 16 Register. */
	__IOM uint32_t BKP17R; /*!< RTC Backup 17 Register. */
	__IOM uint32_t BKP18R; /*!< RTC Backup 18 Register. */
	__IOM uint32_t BKP19R; /*!< RTC Backup 19 Register. */
	__IOM uint32_t BKP20R; /*!< RTC Backup 20 Register. */
	__IOM uint32_t BKP21R; /*!< RTC Backup 21 Register. */
	__IOM uint32_t BKP22R; /*!< RTC Backup 22 Register. */
	__IOM uint32_t BKP23R; /*!< RTC Backup 23 Register. */
	__IOM uint32_t BKP24R; /*!< RTC Backup 24 Register. */
	__IOM uint32_t BKP25R; /*!< RTC Backup 25 Register. */
	__IOM uint32_t BKP26R; /*!< RTC Backup 26 Register. */
	__IOM uint32_t BKP27R; /*!< RTC Backup 27 Register. */
	__IOM uint32_t BKP28R; /*!< RTC Backup 28 Register. */
	__IOM uint32_t BKP29R; /*!< RTC Backup 29 Register. */
	__IOM uint32_t BKP30R; /*!< RTC Backup 30 Register. */
	__IOM uint32_t BKP31R; /*!< RTC Backup 31 Register. */
}STM32L4xx_RTC_TypeDef;

/*TR register.*/
#define RTC_TR_PM_Pos 22U
#define RTC_TR_PM_Msk (0x1UL << RTC_TR_PM_Pos)
#define RTC_TR_PM RTC_TR_PM_Msk

#define RTC_TR_HT_Pos 20U
#define RTC_TR_HT_Msk (0x3UL << RTC_TR_HT_Pos)
#define RTC_TR_HT RTC_TR_HT_Msk

#define RTC_TR_HU_Pos 16U
#define RTC_TR_HU_Msk (0xFUL << RTC_TR_HU_Pos)
#define RTC_TR_HU RTC_TR_HU_Msk

#define RTC_TR_MNT_Pos 12U
#define RTC_TR_MNT_Msk (0x7UL << RTC_TR_MNT_Pos)
#define RTC_TR_MNT RTC_TR_MNT_Msk

#define RTC_TR_MNU_Pos 7U
#define RTC_TR_MNU_Msk (0xFUL << RTC_TR_MNU_Pos)
#define RTC_TR_MNU RTC_TR_MNU_Msk

#define RTC_TR_ST_Pos 4U
#define RTC_TR_ST_Msk (0x7UL << RTC_TR_ST_Pos)
#define RTC_TR_ST RTC_TR_ST_Msk

#define RTC_TR_SU_Pos 0U
#define RTC_TR_SU_Msk (0xFUL << RTC_TR_SU_Pos)
#define RTC_TR_SU RTC_TR_SU_Msk

/*DR register.*/
#define RTC_DR_YT_Pos 20U
#define RTC_DR_YT_Msk (0xFUL << RTC_DR_YT_Pos)
#define RTC_DR_YT RTC_DR_YT_Msk

#define RTC_DR_YU_Pos 16U
#define RTC_DR_YU_Msk (0xFUL << RTC_DR_YU_Pos)
#define RTC_DR_YU RTC_DR_YU_Msk

#define RTC_DR_WDU_Pos 13U
#define RTC_DR_WDU_Msk (0x7UL << RTC_DR_WDU_Pos)
#define RTC_DR_WDU RTC_DR_WDU_Msk

#define RTC_DR_MT_Pos 12U
#define RTC_DR_MT_Msk (0x1UL << RTC_DR_MT_Pos)
#define RTC_DR_MT RTC_DR_MT_Msk

#define RTC_DR_MU_Pos 8U
#define RTC_DR_MU_Msk (0xFUL << RTC_DR_MU_Pos)
#define RTC_DR_MU RTC_DR_MU_Msk

#define RTC_DR_DT_Pos 4U
#define RTC_DR_DT_Msk (0x3UL << RTC_DR_DT_Pos)
#define RTC_DR_DT RTC_DR_DT_Msk

#define RTC_DR_DU_Pos 0U
#define RTC_DR_DU_Msk (0xFUL << RTC_DR_DU_Pos)
#define RTC_DR_DU RTC_DR_DU_Msk

/*CR register.*/
#define RTC_CR_ITSE_Pos 24U
#define RTC_CR_ITSE_Msk (0x1UL << RTC_CR_ITSE_Pos)
#define RTC_CR_ITSE RTC_CR_ITSE_Msk

#define RTC_CR_COE_Pos 23U
#define RTC_CR_COE_Msk (0x1UL << RTC_CR_COE_Pos)
#define RTC_CR_COE RTC_CR_COE_Msk

#define RTC_CR_OSEL_Pos 21U
#define RTC_CR_OSEL_Msk (0x3UL << RTC_CR_OSEL_Pos)
#define RTC_CR_OSEL RTC_CR_OSEL_Msk

#define RTC_CR_POL_Pos 20U
#define RTC_CR_POL_Msk (0x1UL << RTC_CR_POL_Pos)
#define RTC_CR_POL RTC_CR_POL_Msk

#define RTC_CR_COSEL_Pos 19U
#define RTC_CR_COSEL_Msk (0x1UL << RTC_CR_COSEL_Pos)
#define RTC_CR_COSEL RTC_CR_COSEL_Msk

#define RTC_CR_BKP_Pos 18U
#define RTC_CR_BKP_Msk (0x1UL << RTC_CR_BKP_Pos)
#define RTC_CR_BKP RTC_CR_BKP_Msk

#define RTC_CR_SUB1H_Pos 17U
#define RTC_CR_SUB1H_Msk (0x1UL << RTC_CR_SUB1H_Pos)
#define RTC_CR_SUB1H RTC_CR_SUB1H_Msk

#define RTC_CR_ADD1H_Pos 16U
#define RTC_CR_ADD1H_Msk (0x1UL << RTC_CR_ADD1H_Pos)
#define RTC_CR_ADD1H RTC_CR_ADD1H_Msk

#define RTC_CR_TSIE_Pos 15U
#define RTC_CR_TSIE_Msk (0x1UL << RTC_CR_TSIE_Pos)
#define RTC_CR_TSIE RTC_CR_TSIE_Msk

#define RTC_CR_WUTIE_Pos 14U
#define RTC_CR_WUTIE_Msk (0x1UL << RTC_CR_WUTIE_Pos)
#define RTC_CR_WUTIE RTC_CR_WUTIE_Msk

#define RTC_CR_ALRBIE_Pos 13U
#define RTC_CR_ALRBIE_Msk (0x1UL << RTC_CR_ALRBIE_Pos)
#define RTC_CR_ALRBIE RTC_CR_ALRBIE_Msk

#define RTC_CR_ALRAIE_Pos 12U
#define RTC_CR_ALRAIE_Msk (0x1UL << RTC_CR_ALRAIE_Pos)
#define RTC_CR_ALRAIE RTC_CR_ALRAIE_Msk

#define RTC_CR_TSE_Pos 11U
#define RTC_CR_TSE_Msk (0x1UL << RTC_CR_TSE_Pos)
#define RTC_CR_TSE RTC_CR_TSE_Msk

#define RTC_CR_WUTE_Pos 10U
#define RTC_CR_WUTE_Msk (0x1UL << RTC_CR_WUTE_Pos)
#define RTC_CR_WUTE RTC_CR_WUTE_Msk

#define RTC_CR_ALRBE_Pos 9U
#define RTC_CR_ALRBE_Msk (0x1UL << RTC_CR_ALRBE_Pos)
#define RTC_CR_ALRBE RTC_CR_ALRBE_Msk

#define RTC_CR_ALRAE_Pos 8U
#define RTC_CR_ALRAE_Msk (0x1UL << RTC_CR_ALRAE_Pos)
#define RTC_CR_ALRAE RTC_CR_ALRAE_Msk

#define RTC_CR_FMT_Pos 6U
#define RTC_CR_FMT_Msk (0x1UL << RTC_CR_FMT_Pos)
#define RTC_CR_FMT RTC_CR_FMT_Msk

#define RTC_CR_BYPSHAD_Pos 5U
#define RTC_CR_BYPSHAD_Msk (0x1UL << RTC_CR_BYPSHAD_Pos)
#define RTC_CR_BYPSHAD RTC_CR_BYPSHAD_Msk

#define RTC_CR_REFCKON_Pos 4U
#define RTC_CR_REFCKON_Msk (0x1UL << RTC_CR_REFCKON_Pos)
#define RTC_CR_REFCKON RTC_CR_REFCKON_Msk

#define RTC_CR_TSEDGE_Pos 3U
#define RTC_CR_TSEDGE_Msk (0x1UL << RTC_CR_TSEDGE_Pos)
#define RTC_CR_TSEDGE RTC_CR_TSEDGE_Msk

#define RTC_CR_WUCKSEL_Pos 0U
#define RTC_CR_WUCKSEL_Msk (0x7UL << RTC_CR_WUCKSEL_Pos)
#define RTC_CR_WUCKSEL RTC_CR_WUCKSEL_Msk

/*ISR register.*/
#define RTC_ISR_ITSF_Pos 17U
#define RTC_ISR_ITSF_Msk (0x1UL << RTC_ISR_ITSF_Pos)
#define RTC_ISR_ITSF RTC_ISR_ITSF_Msk

#define RTC_ISR_RECALPF_Pos 16U
#define RTC_ISR_RECALPF_Msk (0x1UL << RTC_ISR_RECALPF_Pos)
#define RTC_ISR_RECALPF RTC_ISR_RECALPF_Msk

#define RTC_ISR_TAMP3F_Pos 15U
#define RTC_ISR_TAMP3F_Msk (0x1UL << RTC_ISR_TAMP3F_Pos)
#define RTC_ISR_TAMP3F RTC_ISR_TAMP3F_Msk

#define RTC_ISR_TAMP2F_Pos 14U
#define RTC_ISR_TAMP2F_Msk (0x1UL << RTC_ISR_TAMP2F_Pos)
#define RTC_ISR_TAMP2F RTC_ISR_TAMP2F_Msk

#define RTC_ISR_TAMP1F_Pos 13U
#define RTC_ISR_TAMP1F_Msk (0x1UL << RTC_ISR_TAMP1F_Pos)
#define RTC_ISR_TAMP1F RTC_ISR_TAMP1F_Msk

#define RTC_ISR_TSOVF_Pos 12U
#define RTC_ISR_TSOVF_Msk (0x1UL << RTC_ISR_TSOVF_Pos)
#define RTC_ISR_TSOVF RTC_ISR_TSOVF_Msk

#define RTC_ISR_TSF_Pos 11U
#define RTC_ISR_TSF_Msk (0x1UL << RTC_ISR_TSF_Pos)
#define RTC_ISR_TSF RTC_ISR_TSF_Msk

#define RTC_ISR_WUTF_Pos 10U
#define RTC_ISR_WUTF_Msk (0x1UL << RTC_ISR_WUTF_Pos)
#define RTC_ISR_WUTF RTC_ISR_WUTF_Msk

#define RTC_ISR_ALRBF_Pos 9U
#define RTC_ISR_ALRBF_Msk (0x1UL << RTC_ISR_ALRBF_Pos)
#define RTC_ISR_ALRBF RTC_ISR_ALRBF_Msk

#define RTC_ISR_ALRAF_Pos 8U
#define RTC_ISR_ALRAF_Msk (0x1UL << RTC_ISR_ALRAF_Pos)
#define RTC_ISR_ALRAF RTC_ISR_ALRAF_Msk

#define RTC_ISR_INIT_Pos 7U
#define RTC_ISR_INIT_Msk (0x1UL << RTC_ISR_INIT_Pos)
#define RTC_ISR_INIT RTC_ISR_INIT_Msk

#define RTC_ISR_INITF_Pos 6U
#define RTC_ISR_INITF_Msk (0x1UL << RTC_ISR_INITF_Pos)
#define RTC_ISR_INITF RTC_ISR_INITF_Msk

#define RTC_ISR_RSF_Pos 5U
#define RTC_ISR_RSF_Msk (0x1UL << RTC_ISR_RSF_Pos)
#define RTC_ISR_RSF RTC_ISR_RSF_Msk

#define RTC_ISR_INITS_Pos 4U
#define RTC_ISR_INITS_Msk (0x1UL << RTC_ISR_INITS_Pos)
#define RTC_ISR_INITS RTC_ISR_INITS_Msk

#define RTC_ISR_SHPF_Pos 3U
#define RTC_ISR_SHPF_Msk (0x1UL << RTC_ISR_SHPF_Pos)
#define RTC_ISR_SHPF RTC_ISR_SHPF_Msk

#define RTC_ISR_WUTWF_Pos 2U
#define RTC_ISR_WUTWF_Msk (0x1UL << RTC_ISR_WUTWF_Pos)
#define RTC_ISR_WUTWF RTC_ISR_WUTWF_Msk

#define RTC_ISR_ALRBWF_Pos 1U
#define RTC_ISR_ALRBWF_Msk (0x1UL << RTC_ISR_ALRBWF_Pos)
#define RTC_ISR_ALRBWF RTC_ISR_ALRBWF_Msk

#define RTC_ISR_ALRAWF_Pos 0U
#define RTC_ISR_ALRAWF_Msk (0x1UL << RTC_ISR_ALRAWF_Pos)
#define RTC_ISR_ALRAWF RTC_ISR_ALRAWF_Msk

/*PRER register.*/
#define RTC_PRER_PREDIV_A_Pos 16U
#define RTC_PRER_PREDIV_A_Msk (0x7FUL << RTC_PRER_PREDIV_A_Pos)
#define RTC_PRER_PREDIV_A RTC_PRER_PREDIV_A_Msk

#define RTC_PRER_PREDIV_S_Pos 0U
#define RTC_PRER_PREDIV_S_Msk (0x7FFFUL << RTC_PRER_PREDIV_S_Pos)
#define RTC_PRER_PREDIV_S RTC_PRER_PREDIV_S_Msk

/*WUTR register.*/
#define RTC_WUTR_WUTOCLR_Pos 16U
#define RTC_WUTR_WUTOCLR_Msk (0xFFFFUL << RTC_WUTR_WUTOCLR_Pos)
#define RTC_WUTR_WUTOCLR RTC_WUTR_WUTOCLR_Msk

#define RTC_WUTR_WUT_Pos 0U
#define RTC_WUTR_WUT_Msk (0xFFFFUL << RTC_WUTR_WUT_Pos)
#define RTC_WUTR_WUT RTC_WUTR_WUT_Msk

/*ALRMAR register.*/
#define RTC_ALRMAR_MSK4_Pos 31U
#define RTC_ALRMAR_MSK4_Msk (0x1UL << RTC_ALRMAR_MSK4_Pos)
#define RTC_ALRMAR_MSK4 RTC_ALRMAR_MSK4_Msk

#define RTC_ALRMAR_WDSEL_Pos 30U
#define RTC_ALRMAR_WDSEL_Msk (0x1UL << RTC_ALRMAR_WDSEL_Pos)
#define RTC_ALRMAR_WDSEL RTC_ALRMAR_WDSEL_Msk

#define RTC_ALRMAR_DT_Pos 28U
#define RTC_ALRMAR_DT_Msk (0x3UL << RTC_ALRMAR_DT_Pos)
#define RTC_ALRMAR_DT RTC_ALRMAR_DT_Msk

#define RTC_ALRMAR_DU_Pos 24U
#define RTC_ALRMAR_DU_Msk (0xFUL << RTC_ALRMAR_DU_Pos)
#define RTC_ALRMAR_DU RTC_ALRMAR_DU_Msk

#define RTC_ALRMAR_MSK3_Pos 23U
#define RTC_ALRMAR_MSK3_Msk (0x1UL << RTC_ALRMAR_MSK3_Pos)
#define RTC_ALRMAR_MSK3 RTC_ALRMAR_MSK3_Msk

#define RTC_ALRMAR_PM_Pos 22U
#define RTC_ALRMAR_PM_Msk (0x1UL << RTC_ALRMAR_PM_Pos)
#define RTC_ALRMAR_PM RTC_ALRMAR_PM_Msk

#define RTC_ALRMAR_HT_Pos 20U
#define RTC_ALRMAR_HT_Msk (0x3UL << RTC_ALRMAR_HT_Pos)
#define RTC_ALRMAR_HT RTC_ALRMAR_HT_Msk

#define RTC_ALRMAR_HU_Pos 16U
#define RTC_ALRMAR_HU_Msk (0xFUL << RTC_ALRMAR_HU_Pos)
#define RTC_ALRMAR_HU RTC_ALRMAR_HU_Msk

#define RTC_ALRMAR_MSK2_Pos 15U
#define RTC_ALRMAR_MSK2_Msk (0x1UL << RTC_ALRMAR_MSK2_Pos)
#define RTC_ALRMAR_MSK2 RTC_ALRMAR_MSK2_Msk

#define RTC_ALRMAR_MNT_Pos 12U
#define RTC_ALRMAR_MNT_Msk (0x7UL << RTC_ALRMAR_MNT_Pos)
#define RTC_ALRMAR_MNT RTC_ALRMAR_MNT_Msk

#define RTC_ALRMAR_MNU_Pos 8U
#define RTC_ALRMAR_MNU_Msk (0xFUL << RTC_ALRMAR_MNU_Pos)
#define RTC_ALRMAR_MNU RTC_ALRMAR_MNU_Msk

#define RTC_ALRMAR_MSK1_Pos 7U
#define RTC_ALRMAR_MSK1_Msk (0x1UL << RTC_ALRMAR_MSK1_Pos)
#define RTC_ALRMAR_MSK1 RTC_ALRMAR_MSK1_Msk

#define RTC_ALRMAR_ST_Pos 4U
#define RTC_ALRMAR_ST_Msk (0x7UL << RTC_ALRMAR_ST_Pos)
#define RTC_ALRMAR_ST RTC_ALRMAR_ST_Msk

#define RTC_ALRMAR_SU_Pos 0U
#define RTC_ALRMAR_SU_Msk (0xFUL << RTC_ALRMAR_SU_Pos)
#define RTC_ALRMAR_SU RTC_ALRMAR_SU_Msk

/*ALRMBR register.*/
#define RTC_ALRMBR_MSK4_Pos 31U
#define RTC_ALRMBR_MSK4_Msk (0x1UL << RTC_ALRMBR_MSK4_Pos)
#define RTC_ALRMBR_MSK4 RTC_ALRMBR_MSK4_Msk

#define RTC_ALRMBR_WDSEL_Pos 30U
#define RTC_ALRMBR_WDSEL_Msk (0x1UL << RTC_ALRMBR_WDSEL_Pos)
#define RTC_ALRMBR_WDSEL RTC_ALRMBR_WDSEL_Msk

#define RTC_ALRMBR_DT_Pos 28U
#define RTC_ALRMBR_DT_Msk (0x3UL << RTC_ALRMBR_DT_Pos)
#define RTC_ALRMBR_DT RTC_ALRMBR_DT_Msk

#define RTC_ALRMBR_DU_Pos 24U
#define RTC_ALRMBR_DU_Msk (0xFUL << RTC_ALRMBR_DU_Pos)
#define RTC_ALRMBR_DU RTC_ALRMBR_DU_Msk

#define RTC_ALRMBR_MSK3_Pos 23U
#define RTC_ALRMBR_MSK3_Msk (0x1UL << RTC_ALRMBR_MSK3_Pos)
#define RTC_ALRMBR_MSK3 RTC_ALRMBR_MSK3_Msk

#define RTC_ALRMBR_PM_Pos 22U
#define RTC_ALRMBR_PM_Msk (0x1UL << RTC_ALRMBR_PM_Pos)
#define RTC_ALRMBR_PM RTC_ALRMBR_PM_Msk

#define RTC_ALRMBR_HT_Pos 20U
#define RTC_ALRMBR_HT_Msk (0x3UL << RTC_ALRMBR_HT_Pos)
#define RTC_ALRMBR_HT RTC_ALRMBR_HT_Msk

#define RTC_ALRMBR_HU_Pos 16U
#define RTC_ALRMBR_HU_Msk (0xFUL << RTC_ALRMBR_HU_Pos)
#define RTC_ALRMBR_HU RTC_ALRMBR_HU_Msk

#define RTC_ALRMBR_MSK2_Pos 15U
#define RTC_ALRMBR_MSK2_Msk (0x1UL << RTC_ALRMBR_MSK2_Pos)
#define RTC_ALRMBR_MSK2 RTC_ALRMBR_MSK2_Msk

#define RTC_ALRMBR_MNT_Pos 12U
#define RTC_ALRMBR_MNT_Msk (0x7UL << RTC_ALRMBR_MNT_Pos)
#define RTC_ALRMBR_MNT RTC_ALRMBR_MNT_Msk

#define RTC_ALRMBR_MNU_Pos 8U
#define RTC_ALRMBR_MNU_Msk (0xFUL << RTC_ALRMBR_MNU_Pos)
#define RTC_ALRMBR_MNU RTC_ALRMBR_MNU_Msk

#define RTC_ALRMBR_MSK1_Pos 7U
#define RTC_ALRMBR_MSK1_Msk (0x1UL << RTC_ALRMBR_MSK1_Pos)
#define RTC_ALRMBR_MSK1 RTC_ALRMBR_MSK1_Msk

#define RTC_ALRMBR_ST_Pos 4U
#define RTC_ALRMBR_ST_Msk (0x7UL << RTC_ALRMBR_ST_Pos)
#define RTC_ALRMBR_ST RTC_ALRMBR_ST_Msk

#define RTC_ALRMBR_SU_Pos 0U
#define RTC_ALRMBR_SU_Msk (0xFUL << RTC_ALRMBR_SU_Pos)
#define RTC_ALRMBR_SU RTC_ALRMBR_SU_Msk

/*WPR register.*/
#define RTC_WPR_KEY_Pos 0U
#define RTC_WPR_KEY_Msk (0xFFUL << RTC_WPR_KEY_Pos)
#define RTC_WPR_KEY RTC_WPR_KEY_Msk

/*SSR register.*/
#define RTC_SSR_SS_Pos 0U
#define RTC_SSR_SS_Msk (0xFFFFUL << RTC_SSR_SS_Pos)
#define RTC_SSR_SS RTC_SSR_SS_Msk

/*SHIFTR register.*/
#define RTC_SHIFTR_ADD1S_Pos 31U
#define RTC_SHIFTR_ADD1S_Msk (0x1UL << RTC_SHIFTR_ADD1S_Pos)
#define RTC_SHIFTR_ADD1S RTC_SHIFTR_ADD1S_Msk

#define RTC_SHIFTR_SUBFS_Pos 0U
#define RTC_SHIFTR_SUBFS_Msk (0x7FFFUL << RTC_SHIFTR_SUBFS_Pos)
#define RTC_SHIFTR_SUBFS RTC_SHIFTR_SUBFS_Msk

/*TSTR register.*/
#define RTC_TSTR_PM_Pos 22U
#define RTC_TSTR_PM_Msk (0x1UL << RTC_TSTR_PM_Pos)
#define RTC_TSTR_PM RTC_TSTR_PM_Msk

#define RTC_TSTR_HT_Pos 20U
#define RTC_TSTR_HT_Msk (0x3UL << RTC_TSTR_HT_Pos)
#define RTC_TSTR_HT RTC_TSTR_HT_Msk

#define RTC_TSTR_HU_Pos 16U
#define RTC_TSTR_HU_Msk (0xFUL << RTC_TSTR_HU_Pos)
#define RTC_TSTR_HU RTC_TSTR_HU_Msk

#define RTC_TSTR_MNT_Pos 12U
#define RTC_TSTR_MNT_Msk (0x7UL << RTC_TSTR_MNT_Pos)
#define RTC_TSTR_MNT RTC_TSTR_MNT_Msk

#define RTC_TSTR_MNU_Pos 8U
#define RTC_TSTR_MNU_Msk (0xFUL << RTC_TSTR_MNU_Pos)
#define RTC_TSTR_MNU RTC_TSTR_MNU_Msk

#define RTC_TSTR_ST_Pos 4U
#define RTC_TSTR_ST_Msk (0x7UL << RTC_TSTR_ST_Pos)
#define RTC_TSTR_ST RTC_TSTR_ST_Msk

#define RTC_TSTR_SU_Pos 0U
#define RTC_TSTR_SU_Msk (0xFUL << RTC_TSTR_SU_Pos)
#define RTC_TSTR_SU RTC_TSTR_SU_Msk

/*TSDR register.*/
#define RTC_TSDR_WDU_Pos 13U
#define RTC_TSDR_WDU_Msk (0x7UL << RTC_TSDR_WDU_Pos)
#define RTC_TSDR_WDU RTC_TSDR_WDU_Msk

#define RTC_TSDR_MT_Pos 12U
#define RTC_TSDR_MT_Msk (0x1UL << RTC_TSDR_MT_Pos)
#define RTC_TSDR_MT RTC_TSDR_MT_Msk

#define RTC_TSDR_MU_Pos 8U
#define RTC_TSDR_MU_Msk (0xFUL << RTC_TSDR_MU_Pos)
#define RTC_TSDR_MU RTC_TSDR_MU_Msk

#define RTC_TSDR_DT_Pos 4U
#define RTC_TSDR_DT_Msk (0x3UL << RTC_TSDR_DT_Pos)
#define RTC_TSDR_DT RTC_TSDR_DT_Msk

#define RTC_TSDR_DU_Pos 0U
#define RTC_TSDR_DU_Msk (0xFUL << RTC_TSDR_DU_Pos)
#define RTC_TSDR_DU RTC_TSDR_DU_Msk

/*TSSSR register.*/
#define RTC_TSSSR_SS_Pos 0U
#define RTC_TSSSR_SS_Msk (0xFFFFUL << RTC_TSSSR_SS_Pos)
#define RTC_TSSSR_SS RTC_TSSSR_SS_Msk

/*CALR register.*/
#define RTC_CALR_CALP_Pos 15U
#define RTC_CALR_CALP_Msk (0x1UL << RTC_CALR_CALP_Pos)
#define RTC_CALR_CALP RTC_CALR_CALP_Msk

#define RTC_CALR_CALW8_Pos 14U
#define RTC_CALR_CALW8_Msk (0x1UL << RTC_CALR_CALW8_Pos)
#define RTC_CALR_CALW8 RTC_CALR_CALW8_Msk

#define RTC_CALR_CALW16_Pos 13U
#define RTC_CALR_CALW16_Msk (0x1UL << RTC_CALR_CALW16_Pos)
#define RTC_CALR_CALW16 RTC_CALR_CALW16_Msk

#define RTC_CALR_CALM_Pos 0U
#define RTC_CALR_CALM_Msk (0x1FFUL << RTC_CALR_CALM_Pos)
#define RTC_CALR_CALM RTC_CALR_CALM_Msk

/*TAMPCR registers.*/
#define RTC_TAMPCR_TAMP3MF_Pos 24U
#define RTC_TAMPCR_TAMP3MF_Msk (0x1UL << RTC_TAMPCR_TAMP3MF_Pos)
#define RTC_TAMPCR_TAMP3MF RTC_TAMPCR_TAMP3MF_Msk

#define RTC_TAMPCR_TAMP3NOERASE_Pos 23U
#define RTC_TAMPCR_TAMP3NOERASE_Msk (0x1UL << RTC_TAMPCR_TAMP3NOERASE_Pos)
#define RTC_TAMPCR_TAMP3NOERASE RTC_TAMPCR_TAMP3NOERASE_Msk

#define RTC_TAMPCR_TAMP3IE_Pos 22U
#define RTC_TAMPCR_TAMP3IE_Msk (0x1UL << RTC_TAMPCR_TAMP3IE_Pos)
#define RTC_TAMPCR_TAMP3IE RTC_TAMPCR_TAMP3IE_Msk

#define RTC_TAMPCR_TAMP2MF_Pos 21U
#define RTC_TAMPCR_TAMP2MF_Msk (0x1UL << RTC_TAMPCR_TAMP2MF_Pos)
#define RTC_TAMPCR_TAMP2MF RTC_TAMPCR_TAMP2MF_Msk

#define RTC_TAMPCR_TAMP2NOERASE_Pos 20U
#define RTC_TAMPCR_TAMP2NOERASE_Msk (0x1UL << RTC_TAMPCR_TAMP2NOERASE_Pos)
#define RTC_TAMPCR_TAMP2NOERASE RTC_TAMPCR_TAMP2NOERASE_Msk

#define RTC_TAMPCR_TAMP2IE_Pos 19U
#define RTC_TAMPCR_TAMP2IE_Msk (0x1UL << RTC_TAMPCR_TAMP2IE_Pos)
#define RTC_TAMPCR_TAMP2IE RTC_TAMPCR_TAMP2IE_Msk

#define RTC_TAMPCR_TAMP1MF_Pos 18U
#define RTC_TAMPCR_TAMP1MF_Msk (0x1UL << RTC_TAMPCR_TAMP1MF_Pos)
#define RTC_TAMPCR_TAMP1MF RTC_TAMPCR_TAMP1MF_Msk

#define RTC_TAMPCR_TAMP1NOERASE_Pos 17U
#define RTC_TAMPCR_TAMP1NOERASE_Msk (0x1UL << RTC_TAMPCR_TAMP1NOERASE_Pos)
#define RTC_TAMPCR_TAMP1NOERASE RTC_TAMPCR_TAMP1NOERASE_Msk

#define RTC_TAMPCR_TAMP1IE_Pos 16U
#define RTC_TAMPCR_TAMP1IE_Msk (0x1UL << RTC_TAMPCR_TAMP1IE_Pos)
#define RTC_TAMPCR_TAMP1IE RTC_TAMPCR_TAMP1IE_Msk

/*ALRMASSR registers.*/
#define RTC_ALRMASSR_MASKSS_Pos 24U
#define RTC_ALRMASSR_MASKSS_Msk (0xFUL << RTC_ALRMASSR_MASKSS_Pos)
#define RTC_ALRMASSR_MASKSS RTC_ALRMASSR_MASKSS_Msk

#define RTC_ALRMASSR_SS_Pos 0U
#define RTC_ALRMASSR_SS_Msk (0x7FFFUL << RTC_ALRMASSR_SS_Pos)
#define RTC_ALRMASSR_SS RTC_ALRMASSR_SS_Msk

/*ALRMBSSR registers.*/
#define RTC_ALRMBSSR_MASKSS_Pos 24U
#define RTC_ALRMBSSR_MASKSS_Msk (0xFUL << RTC_ALRMBSSR_MASKSS_Pos)
#define RTC_ALRMBSSR_MASKSS RTC_ALRMBSSR_MASKSS_Msk

#define RTC_ALRMBSSR_SS_Pos 0U
#define RTC_ALRMBSSR_SS_Msk (0x7FFFUL << RTC_ALRMBSSR_SS_Pos)
#define RTC_ALRMBSSR_SS RTC_ALRMBSSR_SS_Msk

/*OR register.*/
#define RTC_OR_RTC_OUT_RMP_Pos 1U
#define RTC_OR_RTC_OUT_RMP_Msk (0x1UL << RTC_OR_RTC_OUT_RMP_Pos)
#define RTC_OR_RTC_OUT_RMP RTC_OR_RTC_OUT_RMP_Msk

#define RTC_OR_RTC_ALARM_TYPE_Pos 0U
#define RTC_OR_RTC_ALARM_TYPE_Msk (0x1UL << RTC_OR_ALARM_TYPE_Pos)
#define RTC_OR_RTC_ALARM_TYPE RTC_OR_RTC_ALARM_TYPE_Msk
#endif

/* ========================================================================= */
/* ============                       I2C                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR1; /*!< I2C Control Register 1. */
	__IOM uint32_t CR2; /*!< I2C Control Register 2. */
	__IOM uint32_t OAR1; /*!< I2C Own Address 1 Register. */
	__IOM uint32_t OAR2; /*!< I2C Own Address 2 Register. */
	__IOM uint32_t TIMINGR; /*!< I2C Timing Register. */
	__IOM uint32_t TIMEOUTR; /*!< I2C Timeout Register. */
	__IOM uint32_t ISR; /*!< I2C Interrupt and Status Register. */
	__OM uint32_t ICR; /*!< I2C Interrupt Clear Register. */
	__IM uint32_t PECR; /*!< I2C Packet Error Checking Register. */
	__IM uint32_t RXDR; /*!< I2C Receive Data Register. */
	__IOM uint32_t TXDR; /*!< I2C Transmit Data Register. */
}STM32L4xx_I2C_TypeDef;

/*CR1 register.*/
#define I2C_CR1_PECEN_Pos 23U
#define I2C_CR1_PECEN_Msk (0x1UL << I2C_CR1_PECEN_Pos)
#define I2C_CR1_PECEN I2C_CR1_PECEN_Msk

#define I2C_CR1_ALERTEN_Pos 22U
#define I2C_CR1_ALERTEN_Msk (0x1UL << I2C_CR1_ALERTEN_Pos)
#define I2C_CR1_ALERTEN I2C_CR1_ALERTEN_Msk

#define I2C_CR1_SMBDEN_Pos 21U
#define I2C_CR1_SMBDEN_Msk (0x1UL << I2C_CR1_SMBDEN_Pos)
#define I2C_CR1_SMBDEN I2C_CR1_SMBDEN_Msk

#define I2C_CR1_SMBHEN_Pos 20U
#define I2C_CR1_SMBHEN_Msk (0x1UL << I2C_CR1_SMBHEN_Pos)
#define I2C_CR1_SMBHEN I2C_CR1_SMBHEN_Msk

#define I2C_CR1_GCEN_Pos 19U
#define I2C_CR1_GCEN_Msk (0x1UL << I2C_CR1_GCEN_Pos)
#define I2C_CR1_GCEN I2C_CR1_GCEN_Msk

#define I2C_CR1_WUPEN_Pos 18U
#define I2C_CR1_WUPEN_Msk (0x1UL << I2C_CR1_WUPEN_Pos)
#define I2C_CR1_WUPEN I2C_CR1_WUPEN_Msk

#define I2C_CR1_NOSTRETCH_Pos 17U
#define I2C_CR1_NOSTRETCH_Msk (0x1UL << I2C_CR1_NOSTRETCH_Pos)
#define I2C_CR1_NOSTRETCH I2C_CR1_NOSTRETCH_Msk

#define I2C_CR1_SBC_Pos 16U
#define I2C_CR1_SBC_Msk (0x1UL << I2C_CR1_SBC_Pos)
#define I2C_CR1_SBC I2C_CR1_SBC_Msk

#define I2C_CR1_RXDMAEN_Pos 15U
#define I2C_CR1_RXDMAEN_Msk (0x1UL << I2C_CR1_RXDMAEN_Pos)
#define I2C_CR1_RXDMAEN I2C_CR1_RXDMAEN_Msk

#define I2C_CR1_TXDMAEN_Pos 14U
#define I2C_CR1_TXDMAEN_Msk (0x1UL << I2C_CR1_TXDMAEN_Pos)
#define I2C_CR1_TXDMAEN I2C_CR1_TXDMAEN_Msk

#define I2C_CR1_ANFOFF_Pos 12U
#define I2C_CR1_ANFOFF_Msk (0x1UL << I2C_CR1_ANFOFF_Pos)
#define I2C_CR1_ANFOFF I2C_CR1_ANFOFF_Msk

#define I2C_CR1_DNF_Pos 8U
#define I2C_CR1_DNF_Msk (0xFUL << I2C_CR1_DNF_Pos)
#define I2C_CR1_DNF I2C_CR1_DNF_Msk

#define I2C_CR1_ERRIE_Pos 7U
#define I2C_CR1_ERRIE_Msk (0x1UL << I2C_CR1_ERRIE_Pos)
#define I2C_CR1_ERRIE I2C_CR1_ERRIE_Msk

#define I2C_CR1_TCIE_Pos 6U
#define I2C_CR1_TCIE_Msk (0x1UL << I2C_CR1_TCIE_Pos)
#define I2C_CR1_TCIE I2C_CR1_TCIE_Msk

#define I2C_CR1_STOPIE_Pos 5U
#define I2C_CR1_STOPIE_Msk (0x1UL << I2C_CR1_STOPIE_Pos)
#define I2C_CR1_STOPIE I2C_CR1_STOPIE_Msk

#define I2C_CR1_NACKIE_Pos 4U
#define I2C_CR1_NACKIE_Msk (0x1UL << I2C_CR1_NACKIE_Pos)
#define I2C_CR1_NACKIE I2C_CR1_NACKIE_Msk

#define I2C_CR1_ADDRIE_Pos 3U
#define I2C_CR1_ADDRIE_Msk (0x1UL << I2C_CR1_ADDRIE_Pos)
#define I2C_CR1_ADDRIE I2C_CR1_ADDRIE_Msk

#define I2C_CR1_RXIE_Pos 2U
#define I2C_CR1_RXIE_Msk (0x1UL << I2C_CR1_RXIE_Pos)
#define I2C_CR1_RXIE I2C_CR1_RXIE_Msk

#define I2C_CR1_TXIE_Pos 1U
#define I2C_CR1_TXIE_Msk (0x1UL << I2C_CR1_TXIE_Pos)
#define I2C_CR1_TXIE I2C_CR1_TXIE_Msk

#define I2C_CR1_PE_Pos 0U
#define I2C_CR1_PE_Msk (0x1UL << I2C_CR1_PE_Pos)
#define I2C_CR1_PE I2C_CR1_PE_Msk

/*CR2 register.*/
#define I2C_CR2_PECBYTE_Pos 26U
#define I2C_CR2_PECBYTE_Msk (0x1UL << I2C_CR2_PECBYTE_Pos)
#define I2C_CR2_PECBYTE I2C_CR2_PECBYTE_Msk

#define I2C_CR2_AUTOEND_Pos 25U
#define I2C_CR2_AUTOEND_Msk (0x1UL << I2C_CR2_AUTOEND_Pos)
#define I2C_CR2_AUTOEND I2C_CR2_AUTOEND_Msk

#define I2C_CR2_RELOAD_Pos 24U
#define I2C_CR2_RELOAD_Msk (0x1UL << I2C_CR2_RELOAD_Pos)
#define I2C_CR2_RELOAD I2C_CR2_RELOAD_Msk

#define I2C_CR2_NBYTES_Pos 16U
#define I2C_CR2_NBYTES_Msk (0xFFUL << I2C_CR2_NBYTES_Pos)
#define I2C_CR2_NBYTES I2C_CR2_NBYTES_Msk

#define I2C_CR2_NACK_Pos 15U
#define I2C_CR2_NACK_Msk (0x1UL << I2C_CR2_NACK_Pos)
#define I2C_CR2_NACK I2C_CR2_NACK_Msk

#define I2C_CR2_STOP_Pos 14U
#define I2C_CR2_STOP_Msk (0x1UL << I2C_CR2_STOP_Pos)
#define I2C_CR2_STOP I2C_CR2_STOP_Msk

#define I2C_CR2_START_Pos 13U
#define I2C_CR2_START_Msk (0x1UL << I2C_CR2_START_Pos)
#define I2C_CR2_START I2C_CR2_START_Msk

#define I2C_CR2_HEAD10R_Pos 12U
#define I2C_CR2_HEAD10R_Msk (0x1UL << I2C_CR2_HEAD10R_Pos)
#define I2C_CR2_HEAD10R I2C_CR2_HEAD10R_Msk

#define I2C_CR2_ADD10_Pos 11U
#define I2C_CR2_ADD10_Msk (0x1UL << I2C_CR2_ADD10_Pos)
#define I2C_CR2_ADD10 I2C_CR2_ADD10_Msk

#define I2C_CR2_RDWRN_Pos 10U
#define I2C_CR2_RDWRN_Msk (0x1UL << I2C_CR2_RDWRN_Pos)
#define I2C_CR2_RDWRN I2C_CR2_RDWRN_Msk

#define I2C_CR2_SADD_Pos 0U
#define I2C_CR2_SADD_Msk (0x3FFUL << I2C_CR2_SADD_Pos)
#define I2C_CR2_SADD I2C_CR2_SADD_Msk

/*OAR1 register.*/
#define I2C_OAR1_OA1EN_Pos 15U
#define I2C_OAR1_OA1EN_Msk (0x1UL << I2C_OAR1_OA1EN_Pos)
#define I2C_OAR1_OA1EN I2C_OAR1_OA1EN_Msk

#define I2C_OAR1_OA1MODE_Pos 10U
#define I2C_OAR1_OA1MODE_Msk (0x1UL << I2C_OAR1_OA1MODE_Pos)
#define I2C_OAR1_OA1MODE I2C_OAR1_OA1MODE_Msk

#define I2C_OAR1_OA1_Pos 0U
#define I2C_OAR1_OA1_Msk (0x3FFUL << I2C_OAR1_OA1_Pos)
#define I2C_OAR1_OA1 I2C_OAR1_OA1_Msk

/*OAR2 register.*/
#define I2C_OAR2_OA2EN_Pos 15U
#define I2C_OAR2_OA2EN_Msk (0x1UL << I2C_OAR2_OA2EN_Pos)
#define I2C_OAR2_OA2EN I2C_OAR2_OA2EN_Msk

#define I2C_OAR2_OA2MSK_Pos 8U
#define I2C_OAR2_OA2MSK_Msk (0x7UL << I2C_OAR2_OA2MSK_Pos)
#define I2C_OAR2_OA2MSK I2C_OAR2_OA2MSK_Msk

#define I2C_OAR2_OA2_Pos 1U
#define I2C_OAR2_OA2_Msk (0x7FUL << I2C_OAR2_OA2_Pos)
#define I2C_OAR2_OA2 I2C_OAR2_OA2_Msk

/*TIMINGR register.*/
#define I2C_TIMINGR_PRESC_Pos 28U
#define I2C_TIMINGR_PRESC_Msk (0xFUL << I2C_TIMINGR_PRESC_Pos)
#define I2C_TIMINGR_PRESC I2C_TIMINGR_PRESC_Msk

#define I2C_TIMINGR_SCLDEL_Pos 20U
#define I2C_TIMINGR_SCLDEL_Msk (0xFUL << I2C_TIMINGR_SCLDEL_Pos)
#define I2C_TIMINGR_SCLDEL I2C_TIMINGR_SCLDEL_Msk

#define I2C_TIMINGR_SDADEL_Pos 16U
#define I2C_TIMINGR_SDADEL_Msk (0xFUL << I2C_TIMINGR_SDADEL_Pos)
#define I2C_TIMINGR_SDADEL I2C_TIMINGR_SDADEL_Msk

#define I2C_TIMINGR_SCLH_Pos 8U
#define I2C_TIMINGR_SCLH_Msk (0xFFUL << I2C_TIMINGR_SCLH_Pos)
#define I2C_TIMINGR_SCLH I2C_TIMINGR_SCLH_Msk

#define I2C_TIMINGR_SCLL_Pos 0U
#define I2C_TIMINGR_SCLL_Msk (0xFFUL << I2C_TIMINGR_SCLL_Pos)
#define I2C_TIMINGR_SCLL I2C_TIMINGR_SCLL_Msk

/*TIMEOUTR register.*/
#define I2C_TIMEOUTR_TEXTEN_Pos 31U
#define I2C_TIMEOUTR_TEXTEN_Msk (0x1UL << I2C_TIMEOUTR_TEXTEN_Pos)
#define I2C_TIMEOUTR_TEXTEN I2C_TIMEOUTR_TEXTEN_Msk

#define I2C_TIMEOUTR_TIMEOUTB_Pos 16U
#define I2C_TIMEOUTR_TIMEOUTB_Msk (0xFFFUL << I2C_TIMEOUTR_TIMEOUTB_Pos)
#define I2C_TIMEOUTR_TIMEOUTB I2C_TIMEOUTR_TIMEOUTB_Msk

#define I2C_TIMEOUTR_TIMEOUTEN_Pos 16U
#define I2C_TIMEOUTR_TIMEOUTEN_Msk (0x1UL << I2C_TIMEOUTR_TIMEOUTEN_Pos)
#define I2C_TIMEOUTR_TIMEOUTEN I2C_TIMEOUTR_TIMEOUTEN_Msk

#define I2C_TIMEOUTR_TIDLE_Pos 12U
#define I2C_TIMEOUTR_TIDLE_Msk (0x1UL << I2C_TIMEOUTR_TIDLE_Pos)
#define I2C_TIMEOUTR_TIDLE I2C_TIMEOUTR_TIDLE_Msk

#define I2C_TIMEOUTR_TIMEOUTA_Pos 0U
#define I2C_TIMEOUTR_TIMEOUTA_Msk (0xFFFUL << I2C_TIMEOUTR_TIMEOUTA_Pos)
#define I2C_TIMEOUTR_TIMEOUTA I2C_TIMEOUTR_TIMEOUTA_Msk

/*ISR register.*/
#define I2C_ISR_ADDCODE_Pos 17U
#define I2C_ISR_ADDCODE_Msk (0x7FUL << I2C_ISR_ADDCODE_Pos)
#define I2C_ISR_ADDCODE I2C_ISR_ADDCODE_Msk

#define I2C_ISR_DIR_Pos 16U
#define I2C_ISR_DIR_Msk (0x1UL << I2C_ISR_DIR_Pos)
#define I2C_ISR_DIR I2C_ISR_DIR_Msk

#define I2C_ISR_BUSY_Pos 15U
#define I2C_ISR_BUSY_Msk (0x1UL << I2C_ISR_BUSY_Pos)
#define I2C_ISR_BUSY I2C_ISR_BUSY_Msk

#define I2C_ISR_ALERT_Pos 13U
#define I2C_ISR_ALERT_Msk (0x1UL << I2C_ISR_ALERT_Pos)
#define I2C_ISR_ALERT I2C_ISR_ALERT_Msk

#define I2C_ISR_TIMEOUT_Pos 12U
#define I2C_ISR_TIMEOUT_Msk (0x1UL << I2C_ISR_TIMEOUT_Pos)
#define I2C_ISR_TIMEOUT I2C_ISR_TIMEOUT_Msk

#define I2C_ISR_PECERR_Pos 11U
#define I2C_ISR_PECERR_Msk (0x1UL << I2C_ISR_PECERR_Pos)
#define I2C_ISR_PECERR I2C_ISR_PECERR_Msk

#define I2C_ISR_OVR_Pos 10U
#define I2C_ISR_OVR_Msk (0x1UL << I2C_ISR_OVR_Pos)
#define I2C_ISR_OVR I2C_ISR_OVR_Msk

#define I2C_ISR_ARLO_Pos 9U
#define I2C_ISR_ARLO_Msk (0x1UL << I2C_ISR_ARLO_Pos)
#define I2C_ISR_ARLO I2C_ISR_ARLO_Msk

#define I2C_ISR_BERR_Pos 8U
#define I2C_ISR_BERR_Msk (0x1UL << I2C_ISR_BERR_Pos)
#define I2C_ISR_BERR I2C_ISR_BERR_Msk

#define I2C_ISR_TCR_Pos 7U
#define I2C_ISR_TCR_Msk (0x1UL << I2C_ISR_TCR_Pos)
#define I2C_ISR_TCR I2C_ISR_TCR_Msk

#define I2C_ISR_TC_Pos 6U
#define I2C_ISR_TC_Msk (0x1UL << I2C_ISR_TC_Pos)
#define I2C_ISR_TC I2C_ISR_TC_Msk

#define I2C_ISR_STOPF_Pos 5U
#define I2C_ISR_STOPF_Msk (0x1UL << I2C_ISR_STOPF_Pos)
#define I2C_ISR_STOPF I2C_ISR_STOPF_Msk

#define I2C_ISR_NACKF_Pos 4U
#define I2C_ISR_NACKF_Msk (0x1UL << I2C_ISR_NACKF_Pos)
#define I2C_ISR_NACKF I2C_ISR_NACKF_Msk

#define I2C_ISR_ADDR_Pos 3U
#define I2C_ISR_ADDR_Msk (0x1UL << I2C_ISR_ADDR_Pos)
#define I2C_ISR_ADDR I2C_ISR_ADDR_Msk

#define I2C_ISR_RXNE_Pos 2U
#define I2C_ISR_RXNE_Msk (0x1UL << I2C_ISR_RXNE_Pos)
#define I2C_ISR_RXNE I2C_ISR_RXNE_Msk

#define I2C_ISR_TXIS_Pos 1U
#define I2C_ISR_TXIS_Msk (0x1UL << I2C_ISR_TXIS_Pos)
#define I2C_ISR_TXIS I2C_ISR_TXIS_Msk

#define I2C_ISR_TXE_Pos 0U
#define I2C_ISR_TXE_Msk (0x1UL << I2C_ISR_TXE_Pos)
#define I2C_ISR_TXE I2C_ISR_TXE_Msk

/*ICR register.*/
#define I2C_ICR_ALERTCF_Pos 13U
#define I2C_ICR_ALERTCF_Msk (0x1UL << I2C_ICR_ALERTCF_Pos)
#define I2C_ICR_ALERTCF I2C_ICR_ALERTCF_Msk

#define I2C_ICR_TIMOUTCF_Pos 12U
#define I2C_ICR_TIMOUTCF_Msk (0x1UL << I2C_ICR_TIMOUTCF_Pos)
#define I2C_ICR_TIMOUTCF I2C_ICR_TIMOUTCF_Msk

#define I2C_ICR_PECCF_Pos 11U
#define I2C_ICR_PECCF_Msk (0x1UL << I2C_ICR_PECCF_Pos)
#define I2C_ICR_PECCF I2C_ICR_PECCF_Msk

#define I2C_ICR_OVRCF_Pos 10U
#define I2C_ICR_OVRCF_Msk (0x1UL << I2C_ICR_OVRCF_Pos)
#define I2C_ICR_OVRCF I2C_ICR_OVRCF_Msk

#define I2C_ICR_ARLOCF_Pos 9U
#define I2C_ICR_ARLOCF_Msk (0x1UL << I2C_ICR_ARLOCF_Pos)
#define I2C_ICR_ARLOCF I2C_ICR_ARLOCF_Msk

#define I2C_ICR_BERRCF_Pos 8U
#define I2C_ICR_BERRCF_Msk (0x1UL << I2C_ICR_BERRCF_Pos)
#define I2C_ICR_BERRCF I2C_ICR_BERRCF_Msk

#define I2C_ICR_STOPCF_Pos 5U
#define I2C_ICR_STOPCF_Msk (0x1UL << I2C_ICR_STOPCF_Pos)
#define I2C_ICR_STOPCF I2C_ICR_STOPCF_Msk

#define I2C_ICR_NACKCF_Pos 4U
#define I2C_ICR_NACKCF_Msk (0x1UL << I2C_ICR_NACKCF_Pos)
#define I2C_ICR_NACKCF I2C_ICR_NACKCF_Msk

#define I2C_ICR_ADDRCF_Pos 3U
#define I2C_ICR_ADDRCF_Msk (0x1UL << I2C_ICR_ADDRCF_Pos)
#define I2C_ICR_ADDRCF I2C_ICR_ADDRCF_Msk

/*PECR register.*/
#define I2C_PECR_PEC_Pos 0U
#define I2C_PECR_PEC_Msk (0xFFUL << I2C_PECR_PEC_Pos)
#define I2C_PECR_PEC I2C_PECR_PEC_Msk

/*RXDR register.*/
#define I2C_RXDR_RXDATA_Pos 0U
#define I2C_RXDR_RXDATA_Msk (0xFFUL << I2C_RXDR_RXDATA_Pos)
#define I2C_RXDR_RXDATA I2C_RXDR_RXDATA_Msk

/*TXDR register.*/
#define I2C_TXDR_TXDATA_Pos 0U
#define I2C_TXDR_TXDATA_Msk (0xFFUL << I2C_TXDR_TXDATA_Pos)
#define I2C_TXDR_TXDATA I2C_TXDR_TXDATA_Msk

/* ========================================================================= */
/* ============                      USART                      ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR1; /*!< USART Control Register 1. */
	__IOM uint32_t CR2; /*!< USART Control Register 2. */
	__IOM uint32_t CR3; /*!< USART Control Register 3. */
	__IOM uint32_t BRR; /*!< USART Baud Rate Register. */
	__IOM uint32_t GTPR; /*!< USART Guard Time and Prescaler Register. */
	__IOM uint32_t RTOR; /*!< USART Receiver Timeout Register. */
	__OM uint32_t RQR; /*!< USART Request Register. */
	__IM uint32_t ISR; /*!< USART Interrupt and Status Register. */
	__IOM uint32_t ICR; /*!< USART Interrupt flag Clear Register. */
	__IM uint32_t RDR; /*!< USART Receive Data Register. */
	__IOM uint32_t TDR; /*!< USART Transmit Data Register. */
}STM32L4xx_USART_TypeDef;

/*CR1 register.*/
#define USART_CR1_EOBIE_Pos 27U
#define USART_CR1_EOBIE_Msk (0x1UL << USART_CR1_EOBIE_Pos)
#define USART_CR1_EOBIE USART_CR1_EOBIE_Msk

#define USART_CR1_RTOIE_Pos 26U
#define USART_CR1_RTOIE_Msk (0x1UL << USART_CR1_RTOIE_Pos)
#define USART_CR1_RTOIE USART_CR1_RTOIE_Msk

#define USART_CR1_DEAT_Pos 21U
#define USART_CR1_DEAT_Msk (0x1FUL << USART_CR1_DEAT_Pos)
#define USART_CR1_DEAT USART_CR1_DEAT_Msk

#define USART_CR1_DEDT_Pos 16U
#define USART_CR1_DEDT_Msk (0x1FUL << USART_CR1_DEDT_Pos)
#define USART_CR1_DEDT USART_CR1_DEDT_Msk

#define USART_CR1_OVER8_Pos 15U
#define USART_CR1_OVER8_Msk (0x1UL << USART_CR1_OVER8_Pos)
#define USART_CR1_OVER8 USART_CR1_OVER8_Msk

#define USART_CR1_CMIE_Pos 14U
#define USART_CR1_CMIE_Msk (0x1UL << USART_CR1_CMIE_Pos)
#define USART_CR1_CMIE USART_CR1_CMIE_Msk

#define USART_CR1_MME_Pos 13U
#define USART_CR1_MME_Msk (0x1UL << USART_CR1_MME_Pos)
#define USART_CR1_MME USART_CR1_MME_Msk

#define USART_CR1_M1_Pos 28U
#define USART_CR1_M0_Pos 12U
#define USART_CR1_M_Msk ((0x1UL << USART_CR1_M1_Pos) | (0x1UL << USART_CR1_M0_Pos))
#define USART_CR1_M USART_CR1_M_Msk
#define USART_CR1_M_8_DATA_BITS ((0x0UL << USART_CR1_M1_Pos) | (0x0UL << USART_CR1_M0_Pos))
#define USART_CR1_M_9_DATA_BITS ((0x0UL << USART_CR1_M1_Pos) | (0x1UL << USART_CR1_M0_Pos))
#define USART_CR1_M_7_DATA_BITS ((0x1UL << USART_CR1_M1_Pos) | (0x0UL << USART_CR1_M0_Pos))

#define USART_CR1_WAKE_Pos 11U
#define USART_CR1_WAKE_Msk (0x1UL << USART_CR1_WAKE_Pos)
#define USART_CR1_WAKE USART_CR1_WAKE_Msk

#define USART_CR1_PCE_Pos 10U
#define USART_CR1_PCE_Msk (0x1UL << USART_CR1_PCE_Pos)
#define USART_CR1_PCE USART_CR1_PCE_Msk

#define USART_CR1_PS_Pos 9U
#define USART_CR1_PS_Msk (0x1UL << USART_CR1_PS_Pos)
#define USART_CR1_PS USART_CR1_PS_Msk

#define USART_CR1_PEIE_Pos 8U
#define USART_CR1_PEIE_Msk (0x1UL << USART_CR1_PEIE_Pos)
#define USART_CR1_PEIE USART_CR1_PEIE_Msk

#define USART_CR1_TXEIE_Pos 7U
#define USART_CR1_TXEIE_Msk (0x1UL << USART_CR1_TXEIE_Pos)
#define USART_CR1_TXEIE USART_CR1_TXEIE_Msk

#define USART_CR1_TCIE_Pos 6U
#define USART_CR1_TCIE_Msk (0x1UL << USART_CR1_TCIE_Pos)
#define USART_CR1_TCIE USART_CR1_TCIE_Msk

#define USART_CR1_RXNEIE_Pos 5U
#define USART_CR1_RXNEIE_Msk (0x1UL << USART_CR1_RXNEIE_Pos)
#define USART_CR1_RXNEIE USART_CR1_RXNEIE_Msk

#define USART_CR1_IDLEIE_Pos 4U
#define USART_CR1_IDLEIE_Msk (0x1UL << USART_CR1_IDLEIE_Pos)
#define USART_CR1_IDLEIE USART_CR1_IDLEIE_Msk

#define USART_CR1_TE_Pos 3U
#define USART_CR1_TE_Msk (0x1UL << USART_CR1_TE_Pos)
#define USART_CR1_TE USART_CR1_TE_Msk

#define USART_CR1_RE_Pos 2U
#define USART_CR1_RE_Msk (0x1UL << USART_CR1_RE_Pos)
#define USART_CR1_RE USART_CR1_RE_Msk

#define USART_CR1_UESM_Pos 1U
#define USART_CR1_UESM_Msk (0x1UL << USART_CR1_UESM_Pos)
#define USART_CR1_UESM USART_CR1_UESM_Msk

#define USART_CR1_UE_Pos 0U
#define USART_CR1_UE_Msk (0x1UL << USART_CR1_UE_Pos)
#define USART_CR1_UE USART_CR1_UE_Msk

/*CR2 register.*/
#define USART_CR2_ADD_Pos 24U
#define USART_CR2_ADD_Msk (0xFFUL << USART_CR2_ADD_Pos)
#define USART_CR2_ADD USART_CR2_ADD_Msk

#define USART_CR2_RTOEN_Pos 23U
#define USART_CR2_RTOEN_Msk (0x1UL << USART_CR2_RTOEN_Pos)
#define USART_CR2_RTOEN USART_CR2_RTOEN_Msk

#define USART_CR2_ABRMOD_Pos 21U
#define USART_CR2_ABRMOD_Msk (0x3UL << USART_CR2_ABRMOD_Pos)
#define USART_CR2_ABRMOD USART_CR2_ABRMOD_Msk

#define USART_CR2_ABREN_Pos 20U
#define USART_CR2_ABREN_Msk (0x1UL << USART_CR2_ABREN_Pos)
#define USART_CR2_ABREN USART_CR2_ABREN_Msk

#define USART_CR2_MSBFIRST_Pos 19U
#define USART_CR2_MSBFIRST_Msk (0x1UL << USART_CR2_MSBFIRST_Pos)
#define USART_CR2_MSBFIRST USART_CR2_MSBFIRST_Msk

#define USART_CR2_DATAINV_Pos 18U
#define USART_CR2_DATAINV_Msk (0x1UL << USART_CR2_DATAINV_Pos)
#define USART_CR2_DATAINV USART_CR2_DATAINV_Msk

#define USART_CR2_TXINV_Pos 17U
#define USART_CR2_TXINV_Msk (0x1UL << USART_CR2_TXINV_Pos)
#define USART_CR2_TXINV USART_CR2_TXINV_Msk

#define USART_CR2_RXINV_Pos 16U
#define USART_CR2_RXINV_Msk (0x1UL << USART_CR2_RXINV_Pos)
#define USART_CR2_RXINV USART_CR2_RXINV_Msk

#define USART_CR2_SWAP_Pos 15U
#define USART_CR2_SWAP_Msk (0x1UL << USART_CR2_SWAP_Pos)
#define USART_CR2_SWAP USART_CR2_SWAP_Msk

#define USART_CR2_LINEN_Pos 14U
#define USART_CR2_LINEN_Msk (0x1UL << USART_CR2_LINEN_Pos)
#define USART_CR2_LINEN USART_CR2_LINEN_Msk

#define USART_CR2_STOP_Pos 12U
#define USART_CR2_STOP_Msk (0x3UL << USART_CR2_STOP_Pos)
#define USART_CR2_STOP USART_CR2_STOP_Msk
#define USART_CR2_STOP_1_BIT (0x0UL << USART_CR2_STOP_Pos)
#define USART_CR2_STOP_0_5_BIT (0x1UL << USART_CR2_STOP_Pos)
#define USART_CR2_STOP_2_BIT (0x2UL << USART_CR2_STOP_Pos)
#define USART_CR2_STOP_1_5_BIT (0x3UL << USART_CR2_STOP_Pos)

#define USART_CR2_CLKEN_Pos 11U
#define USART_CR2_CLKEN_Msk (0x1UL << USART_CR2_CLKEN_Pos)
#define USART_CR2_CLKEN USART_CR2_CLKEN_Msk

#define USART_CR2_CPOL_Pos 10U
#define USART_CR2_CPOL_Msk (0x1UL << USART_CR2_CPOL_Pos)
#define USART_CR2_CPOL USART_CR2_CPOL_Msk

#define USART_CR2_CPHA_Pos 9U
#define USART_CR2_CPHA_Msk (0x1UL << USART_CR2_CPHA_Pos)
#define USART_CR2_CPHA USART_CR2_CPHA_Msk

#define USART_CR2_LBCL_Pos 8U
#define USART_CR2_LBCL_Msk (0x1UL << USART_CR2_LBCL_Pos)
#define USART_CR2_LBCL USART_CR2_LBCL_Msk

#define USART_CR2_LBDIE_Pos 6U
#define USART_CR2_LBDIE_Msk (0x1UL << USART_CR2_LBDIE_Pos)
#define USART_CR2_LBDIE USART_CR2_LBDIE_Msk

#define USART_CR2_LBDL_Pos 5U
#define USART_CR2_LBDL_Msk (0x1UL << USART_CR2_LBDL_Pos)
#define USART_CR2_LBDL USART_CR2_LBDL_Msk

#define USART_CR2_ADDM7_Pos 4U
#define USART_CR2_ADDM7_Msk (0x1UL << USART_CR2_ADDM7_Pos)
#define USART_CR2_ADDM7 USART_CR2_ADDM7_Msk

/*CR3 register.*/
#define USART_CR3_TCBGTIE_Pos 24U
#define USART_CR3_TCBGTIE_Msk (0x1UL << USART_CR3_TCBGTIE_Pos)
#define USART_CR3_TCBGTIE USART_CR3_TCBGTIE_Msk

#define USART_CR3_UCESM_Pos 23U
#define USART_CR3_UCESM_Msk (0x1UL << USART_CR3_UCESM_Pos)
#define USART_CR3_UCESM USART_CR3_UCESM_Msk

#define USART_CR3_WUFIE_Pos 22U
#define USART_CR3_WUFIE_Msk (0x1UL << USART_CR3_WUFIE_Pos)
#define USART_CR3_WUFIE USART_CR3_WUFIE_Msk

#define USART_CR3_WUS1_Pos 21U
#define USART_CR3_WUS1_Msk (0x1UL << USART_CR3_WUS1_Pos)
#define USART_CR3_WUS1 USART_CR3_WUS1_Msk

#define USART_CR3_WUS0_Pos 20U
#define USART_CR3_WUS0_Msk (0x1UL << USART_CR3_WUS0_Pos)
#define USART_CR3_WUS0 USART_CR3_WUS0_Msk

#define USART_CR3_SCARCNT2_Pos 19U
#define USART_CR3_SCARCNT2_Msk (0x1UL << USART_CR3_SCARCNT2_Pos)
#define USART_CR3_SCARCNT2 USART_CR3_SCARCNT2_Msk

#define USART_CR3_SCARCNT1_Pos 18U
#define USART_CR3_SCARCNT1_Msk (0x1UL << USART_CR3_SCARCNT1_Pos)
#define USART_CR3_SCARCNT1 USART_CR3_SCARCNT1_Msk

#define USART_CR3_SCARCNT0_Pos 17U
#define USART_CR3_SCARCNT0_Msk (0x1UL << USART_CR3_SCARCNT0_Pos)
#define USART_CR3_SCARCNT0 USART_CR3_SCARCNT0_Msk

#define USART_CR3_DEP_Pos 15U
#define USART_CR3_DEP_Msk (0x1UL << USART_CR3_DEP_Pos)
#define USART_CR3_DEP USART_CR3_DEP_Msk

#define USART_CR3_DEM_Pos 14U
#define USART_CR3_DEM_Msk (0x1UL << USART_CR3_DEM_Pos)
#define USART_CR3_DEM USART_CR3_DEM_Msk

#define USART_CR3_DDRE_Pos 13U
#define USART_CR3_DDRE_Msk (0x1UL << USART_CR3_DDRE_Pos)
#define USART_CR3_DDRE USART_CR3_DDRE_Msk

#define USART_CR3_OVRDIS_Pos 12U
#define USART_CR3_OVRDIS_Msk (0x1UL << USART_CR3_OVRDIS_Pos)
#define USART_CR3_OVRDIS USART_CR3_OVRDIS_Msk

#define USART_CR3_ONEBIT_Pos 11U
#define USART_CR3_ONEBIT_Msk (0x1UL << USART_CR3_ONEBIT_Pos)
#define USART_CR3_ONEBIT USART_CR3_ONEBIT_Msk

#define USART_CR3_CTSIE_Pos 10U
#define USART_CR3_CTSIE_Msk (0x1UL << USART_CR3_CTSIE_Pos)
#define USART_CR3_CTSIE USART_CR3_CTSIE_Msk

#define USART_CR3_CTSE_Pos 9U
#define USART_CR3_CTSE_Msk (0x1UL << USART_CR3_CTSE_Pos)
#define USART_CR3_CTSE USART_CR3_CTSE_Msk

#define USART_CR3_RTSE_Pos 8U
#define USART_CR3_RTSE_Msk (0x1UL << USART_CR3_RTSE_Pos)
#define USART_CR3_RTSE USART_CR3_RTSE_Msk

#define USART_CR3_DMAT_Pos 7U
#define USART_CR3_DMAT_Msk (0x1UL << USART_CR3_DMAT_Pos)
#define USART_CR3_DMAT USART_CR3_DMAT_Msk

#define USART_CR3_DMAR_Pos 6U
#define USART_CR3_DMAR_Msk (0x1UL << USART_CR3_DMAR_Pos)
#define USART_CR3_DMAR USART_CR3_DMAR_Msk

#define USART_CR3_SCEN_Pos 5U
#define USART_CR3_SCEN_Msk (0x1UL << USART_CR3_SCEN_Pos)
#define USART_CR3_SCEN USART_CR3_SCEN_Msk

#define USART_CR3_NACK_Pos 4U
#define USART_CR3_NACK_Msk (0x1UL << USART_CR3_NACK_Pos)
#define USART_CR3_NACK USART_CR3_NACK_Msk

#define USART_CR3_HDSEL_Pos 3U
#define USART_CR3_HDSEL_Msk (0x1UL << USART_CR3_HDSEL_Pos)
#define USART_CR3_HDSEL USART_CR3_HDSEL_Msk

#define USART_CR3_IRLP_Pos 2U
#define USART_CR3_IRLP_Msk (0x1UL << USART_CR3_IRLP_Pos)
#define USART_CR3_IRLP USART_CR3_IRLP_Msk

#define USART_CR3_IREN_Pos 1U
#define USART_CR3_IREN_Msk (0x1UL << USART_CR3_IREN_Pos)
#define USART_CR3_IREN USART_CR3_IREN_Msk

#define USART_CR3_EIE_Pos 0U
#define USART_CR3_EIE_Msk (0x1UL << USART_CR3_EIE_Pos)
#define USART_CR3_EIE USART_CR3_EIE_Msk

/*BRR register.*/
#define USART_BRR_Pos 0U
#define USART_BRR_Msk (0xFFFFUL << USART_BRR_Pos)
#define USART_BRR USART_BRR_Msk

/*GTPR register.*/
#define USART_GTPR_GT_Pos 8U
#define USART_GTPR_GT_Msk (0xFFUL << USART_GTPR_GT_Pos)
#define USART_GTPR_GT USART_GTPR_GT_Msk

#define USART_GTPR_PSC_Pos 0U
#define USART_GTPR_PSC_Msk (0xFFUL << USART_GTPR_PSC_Pos)
#define USART_GTPR_PSC USART_GTPR_PSC_Msk

/*RTOR register.*/
#define USART_RTOR_BLEN_Pos 24U
#define USART_RTOR_BLEN_Msk (0xFFUL << USART_RTOR_BLEN_Pos)
#define USART_RTOR_BLEN USART_RTOR_BLEN_Msk

#define USART_RTOR_RTO_Pos 0U
#define USART_RTOR_RTO_Msk (0xFFFFFFUL << USART_RTOR_RTO_Pos)
#define USART_RTOR_RTO USART_RTOR_RTO_Msk

/*RQR register.*/
#define USART_RQR_TXFRQ_Pos 4U
#define USART_RQR_TXFRQ_Msk (0x1UL << USART_RQR_TXFRQ_Pos)
#define USART_RQR_TXFRQ USART_RQR_TXFRQ_Msk

#define USART_RQR_RXFRQ_Pos 3U
#define USART_RQR_RXFRQ_Msk (0x1UL << USART_RQR_RXFRQ_Pos)
#define USART_RQR_RXFRQ USART_RQR_RXFRQ_Msk

#define USART_RQR_MMRQ_Pos 2U
#define USART_RQR_MMRQ_Msk (0x1UL << USART_RQR_MMRQ_Pos)
#define USART_RQR_MMRQ USART_RQR_MMRQ_Msk

#define USART_RQR_SBKRQ_Pos 1U
#define USART_RQR_SBKRQ_Msk (0x1UL << USART_RQR_SBKRQ_Pos)
#define USART_RQR_SBKRQ USART_RQR_SBKRQ_Msk

#define USART_RQR_ABRRQ_Pos 0U
#define USART_RQR_ABRRQ_Msk (0x1UL << USART_RQR_ABRRQ_Pos)
#define USART_RQR_ABRRQ USART_RQR_ABRRQ_Msk

/*ISR register.*/
#define USART_ISR_TCBGT_Pos 25U
#define USART_ISR_TCBGT_Msk (0x1UL << USART_ISR_TCBGT_Pos)
#define USART_ISR_TCBGT USART_ISR_TCBGT_Msk

#define USART_ISR_REACK_Pos 22U
#define USART_ISR_REACK_Msk (0x1UL << USART_ISR_REACK_Pos)
#define USART_ISR_REACK USART_ISR_REACK_Msk

#define USART_ISR_TEACK_Pos 21U
#define USART_ISR_TEACK_Msk (0x1UL << USART_ISR_TEACK_Pos)
#define USART_ISR_TEACK USART_ISR_TEACK_Msk

#define USART_ISR_WUF_Pos 20U
#define USART_ISR_WUF_Msk (0x1UL << USART_ISR_WUF_Pos)
#define USART_ISR_WUF USART_ISR_WUF_Msk

#define USART_ISR_RWU_Pos 19U
#define USART_ISR_RWU_Msk (0x1UL << USART_ISR_RWU_Pos)
#define USART_ISR_RWU USART_ISR_RWU_Msk

#define USART_ISR_SBKF_Pos 18U
#define USART_ISR_SBKF_Msk (0x1UL << USART_ISR_SBKF_Pos)
#define USART_ISR_SBKF USART_ISR_SBKF_Msk

#define USART_ISR_CMF_Pos 17U
#define USART_ISR_CMF_Msk (0x1UL << USART_ISR_CMF_Pos)
#define USART_ISR_CMF USART_ISR_CMF_Msk

#define USART_ISR_BUSY_Pos 16U
#define USART_ISR_BUSY_Msk (0x1UL << USART_ISR_BUSY_Pos)
#define USART_ISR_BUSY USART_ISR_BUSY_Msk

#define USART_ISR_ABRF_Pos 15U
#define USART_ISR_ABRF_Msk (0x1UL << USART_ISR_ABRF_Pos)
#define USART_ISR_ABRF USART_ISR_ABRF_Msk

#define USART_ISR_ABRE_Pos 14U
#define USART_ISR_ABRE_Msk (0x1UL << USART_ISR_ABRE_Pos)
#define USART_ISR_ABRE USART_ISR_ABRE_Msk

#define USART_ISR_EOBF_Pos 12U
#define USART_ISR_EOBF_Msk (0x1UL << USART_ISR_EOBF_Pos)
#define USART_ISR_EOBF USART_ISR_EOBF_Msk

#define USART_ISR_RTOF_Pos 11U
#define USART_ISR_RTOF_Msk (0x1UL << USART_ISR_RTOF_Pos)
#define USART_ISR_RTOF USART_ISR_RTOF_Msk

#define USART_ISR_CTS_Pos 10U
#define USART_ISR_CTS_Msk (0x1UL << USART_ISR_CTS_Pos)
#define USART_ISR_CTS USART_ISR_CTS_Msk

#define USART_ISR_CTSIF_Pos 9U
#define USART_ISR_CTSIF_Msk (0x1UL << USART_ISR_CTSIF_Pos)
#define USART_ISR_CTSIF USART_ISR_CTSIF_Msk

#define USART_ISR_LBDF_Pos 8U
#define USART_ISR_LBDF_Msk (0x1UL << USART_ISR_LBDF_Pos)
#define USART_ISR_LBDF USART_ISR_LBDF_Msk

#define USART_ISR_TXE_Pos 7U
#define USART_ISR_TXE_Msk (0x1UL << USART_ISR_TXE_Pos)
#define USART_ISR_TXE USART_ISR_TXE_Msk

#define USART_ISR_TC_Pos 6U
#define USART_ISR_TC_Msk (0x1UL << USART_ISR_TC_Pos)
#define USART_ISR_TC USART_ISR_TC_Msk

#define USART_ISR_RXNE_Pos 5U
#define USART_ISR_RXNE_Msk (0x1UL << USART_ISR_RXNE_Pos)
#define USART_ISR_RXNE USART_ISR_RXNE_Msk

#define USART_ISR_IDLE_Pos 4U
#define USART_ISR_IDLE_Msk (0x1UL << USART_ISR_IDLE_Pos)
#define USART_ISR_IDLE USART_ISR_IDLE_Msk

#define USART_ISR_ORE_Pos 3U
#define USART_ISR_ORE_Msk (0x1UL << USART_ISR_ORE_Pos)
#define USART_ISR_ORE USART_ISR_ORE_Msk

#define USART_ISR_NF_Pos 2U
#define USART_ISR_NF_Msk (0x1UL << USART_ISR_NF_Pos)
#define USART_ISR_NF USART_ISR_NF_Msk

#define USART_ISR_FE_Pos 1U
#define USART_ISR_FE_Msk (0x1UL << USART_ISR_FE_Pos)
#define USART_ISR_FE USART_ISR_FE_Msk

#define USART_ISR_PE_Pos 0U
#define USART_ISR_PE_Msk (0x1UL << USART_ISR_PE_Pos)
#define USART_ISR_PE USART_ISR_PE_Msk

/*ICR register.*/
#define USART_ICR_WUCF_Pos 20U
#define USART_ICR_WUCF_Msk (0x1UL << USART_ICR_WUCF_Pos)
#define USART_ICR_WUCF USART_ICR_WUCF_Msk

#define USART_ICR_CMCF_Pos 17U
#define USART_ICR_CMCF_Msk (0x1UL << USART_ICR_CMCF_Pos)
#define USART_ICR_CMCF USART_ICR_CMCF_Msk

#define USART_ICR_EOBCF_Pos 12U
#define USART_ICR_EOBCF_Msk (0x1UL << USART_ICR_EOBCF_Pos)
#define USART_ICR_EOBCF USART_ICR_EOBCF_Msk

#define USART_ICR_RTOCF_Pos 11U
#define USART_ICR_RTOCF_Msk (0x1UL << USART_ICR_RTOCF_Pos)
#define USART_ICR_RTOCF USART_ICR_RTOCF_Msk

#define USART_ICR_CTSCF_Pos 9U
#define USART_ICR_CTSCF_Msk (0x1UL << USART_ICR_CTSCF_Pos)
#define USART_ICR_CTSCF USART_ICR_CTSCF_Msk

#define USART_ICR_LBDCF_Pos 8U
#define USART_ICR_LBDCF_Msk (0x1UL << USART_ICR_LBDCF_Pos)
#define USART_ICR_LBDCF USART_ICR_LBDCF_Msk

#define USART_ICR_TXCBGTCF_Pos 7U
#define USART_ICR_TXCBGTCF_Msk (0x1UL << USART_ICR_TXCBGTCF_Pos)
#define USART_ICR_TXCBGTCF USART_ICR_TXCBGTCF_Msk

#define USART_ICR_TCCF_Pos 6U
#define USART_ICR_TCCF_Msk (0x1UL << USART_ICR_TCCF_Pos)
#define USART_ICR_TCCF USART_ICR_TCCF_Msk

#define USART_ICR_IDLECF_Pos 4U
#define USART_ICR_IDLECF_Msk (0x1UL << USART_ICR_IDLECF_Pos)
#define USART_ICR_IDLECF USART_ICR_IDLECF_Msk

#define USART_ICR_ORECF_Pos 3U
#define USART_ICR_ORECF_Msk (0x1UL << USART_ICR_ORECF_Pos)
#define USART_ICR_ORECF USART_ICR_ORECF_Msk

#define USART_ICR_NCF_Pos 2U
#define USART_ICR_NCF_Msk (0x1UL << USART_ICR_NCF_Pos)
#define USART_ICR_NCF USART_ICR_NCF_Msk

#define USART_ICR_FECF_Pos 1U
#define USART_ICR_FECF_Msk (0x1UL << USART_ICR_FECF_Pos)
#define USART_ICR_FECF USART_ICR_FECF_Msk

#define USART_ICR_PECF_Pos 0U
#define USART_ICR_PECF_Msk (0x1UL << USART_ICR_PECF_Pos)
#define USART_ICR_PECF USART_ICR_PECF_Msk

/*RDR register.*/
#define USART_RDR_Pos 0U
#define USART_RDR_Msk (0xFFUL << USART_RDR_Pos)
#define USART_RDR USART_RDR_RDR_Msk

/*TDR register.*/
#define USART_TDR_Pos 0U
#define USART_TDR_Msk (0xFFUL << USART_TDR_Pos)
#define USART_TDR USART_TDR_Msk

/* ========================================================================= */
/* ============                      LPUART                     ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR1; /*!< LPUART Control Register 1. */
	__IOM uint32_t CR2; /*!< LPUART Control Register 2. */
	__IOM uint32_t CR3; /*!< LPUART Control Register 3. */
	__IOM uint32_t BRR; /*!< LPUART Baud Rate Register. */
	uint32_t RESERVED0[2];
	__OM uint32_t RQR; /*!< LPUART Request Register. */
	__IM uint32_t ISR; /*!< LPUART Interrupt and Status Register. */
	__IOM uint32_t ICR; /*!< LPUART Interrupt flag Clear Register. */
	__IM uint32_t RDR; /*!< LPUART Receive Data Register. */
	__IOM uint32_t TDR; /*!< LPUART Transmit Data Register. */
}STM32L4xx_LPUART_TypeDef;

/*CR1 register.*/
#define LPUART_CR1_DEAT_Pos 21U
#define LPUART_CR1_DEAT_Msk (0x1FUL << LPUART_CR1_DEAT_Pos)
#define LPUART_CR1_DEAT LPUART_CR1_DEAT_Msk

#define LPUART_CR1_DEDT_Pos 16U
#define LPUART_CR1_DEDT_Msk (0x1FUL << LPUART_CR1_DEDT_Pos)
#define LPUART_CR1_DEDT LPUART_CR1_DEDT_Msk

#define LPUART_CR1_CMIE_Pos 14U
#define LPUART_CR1_CMIE_Msk (0x1UL << LPUART_CR1_CMIE_Pos)
#define LPUART_CR1_CMIE LPUART_CR1_CMIE_Msk

#define LPUART_CR1_MME_Pos 13U
#define LPUART_CR1_MME_Msk (0x1UL << LPUART_CR1_MME_Pos)
#define LPUART_CR1_MME LPUART_CR1_MME_Msk

#define LPUART_CR1_M1_Pos 28U
#define LPUART_CR1_M0_Pos 12U
#define LPUART_CR1_M_Msk ((0x1UL << LPUART_CR1_M1_Pos) | (0x1UL << LPUART_CR1_M0_Pos))
#define LPUART_CR1_M LPUART_CR1_M_Msk
#define LPUART_CR1_M_8_DATA_BITS ((0x0UL << LPUART_CR1_M1_Pos) | (0x0UL << LPUART_CR1_M0_Pos))
#define LPUART_CR1_M_9_DATA_BITS ((0x0UL << LPUART_CR1_M1_Pos) | (0x1UL << LPUART_CR1_M0_Pos))
#define LPUART_CR1_M_7_DATA_BITS ((0x1UL << LPUART_CR1_M1_Pos) | (0x0UL << LPUART_CR1_M0_Pos))

#define LPUART_CR1_WAKE_Pos 11U
#define LPUART_CR1_WAKE_Msk (0x1UL << LPUART_CR1_WAKE_Pos)
#define LPUART_CR1_WAKE LPUART_CR1_WAKE_Msk

#define LPUART_CR1_PCE_Pos 10U
#define LPUART_CR1_PCE_Msk (0x1UL << LPUART_CR1_PCE_Pos)
#define LPUART_CR1_PCE LPUART_CR1_PCE_Msk

#define LPUART_CR1_PS_Pos 9U
#define LPUART_CR1_PS_Msk (0x1UL << LPUART_CR1_PS_Pos)
#define LPUART_CR1_PS LPUART_CR1_PS_Msk

#define LPUART_CR1_PEIE_Pos 8U
#define LPUART_CR1_PEIE_Msk (0x1UL << LPUART_CR1_PEIE_Pos)
#define LPUART_CR1_PEIE LPUART_CR1_PEIE_Msk

#define LPUART_CR1_TXEIE_Pos 7U
#define LPUART_CR1_TXEIE_Msk (0x1UL << LPUART_CR1_TXEIE_Pos)
#define LPUART_CR1_TXEIE LPUART_CR1_TXEIE_Msk

#define LPUART_CR1_TCIE_Pos 6U
#define LPUART_CR1_TCIE_Msk (0x1UL << LPUART_CR1_TCIE_Pos)
#define LPUART_CR1_TCIE LPUART_CR1_TCIE_Msk

#define LPUART_CR1_RXNEIE_Pos 5U
#define LPUART_CR1_RXNEIE_Msk (0x1UL << LPUART_CR1_RXNEIE_Pos)
#define LPUART_CR1_RXNEIE LPUART_CR1_RXNEIE_Msk

#define LPUART_CR1_IDLEIE_Pos 4U
#define LPUART_CR1_IDLEIE_Msk (0x1UL << LPUART_CR1_IDLEIE_Pos)
#define LPUART_CR1_IDLEIE LPUART_CR1_IDLEIE_Msk

#define LPUART_CR1_TE_Pos 3U
#define LPUART_CR1_TE_Msk (0x1UL << LPUART_CR1_TE_Pos)
#define LPUART_CR1_TE LPUART_CR1_TE_Msk

#define LPUART_CR1_RE_Pos 2U
#define LPUART_CR1_RE_Msk (0x1UL << LPUART_CR1_RE_Pos)
#define LPUART_CR1_RE LPUART_CR1_RE_Msk

#define LPUART_CR1_UESM_Pos 1U
#define LPUART_CR1_UESM_Msk (0x1UL << LPUART_CR1_UESM_Pos)
#define LPUART_CR1_UESM LPUART_CR1_UESM_Msk

#define LPUART_CR1_UE_Pos 0U
#define LPUART_CR1_UE_Msk (0x1UL << LPUART_CR1_UE_Pos)
#define LPUART_CR1_UE LPUART_CR1_UE_Msk

/*CR2 register.*/
#define LPUART_CR2_ADD_Pos 24U
#define LPUART_CR2_ADD_Msk (0xFFUL << LPUART_CR2_ADD_Pos)
#define LPUART_CR2_ADD LPUART_CR2_ADD_Msk

#define LPUART_CR2_MSBFIRST_Pos 19U
#define LPUART_CR2_MSBFIRST_Msk (0x1UL << LPUART_CR2_MSBFIRST_Pos)
#define LPUART_CR2_MSBFIRST LPUART_CR2_MSBFIRST_Msk

#define LPUART_CR2_DATAINV_Pos 18U
#define LPUART_CR2_DATAINV_Msk (0x1UL << LPUART_CR2_DATAINV_Pos)
#define LPUART_CR2_DATAINV LPUART_CR2_DATAINV_Msk

#define LPUART_CR2_TXINV_Pos 17U
#define LPUART_CR2_TXINV_Msk (0x1UL << LPUART_CR2_TXINV_Pos)
#define LPUART_CR2_TXINV LPUART_CR2_TXINV_Msk

#define LPUART_CR2_RXINV_Pos 16U
#define LPUART_CR2_RXINV_Msk (0x1UL << LPUART_CR2_RXINV_Pos)
#define LPUART_CR2_RXINV LPUART_CR2_RXINV_Msk

#define LPUART_CR2_SWAP_Pos 15U
#define LPUART_CR2_SWAP_Msk (0x1UL << LPUART_CR2_SWAP_Pos)
#define LPUART_CR2_SWAP LPUART_CR2_SWAP_Msk

#define LPUART_CR2_STOP_Pos 12U
#define LPUART_CR2_STOP_Msk (0x3UL << LPUART_CR2_STOP_Pos)
#define LPUART_CR2_STOP LPUART_CR2_STOP_Msk
#define LPUART_CR2_STOP_1_BIT (0x0UL << LPUART_CR2_STOP_Pos)
#define LPUART_CR2_STOP_0_5_BIT (0x1UL << LPUART_CR2_STOP_Pos)
#define LPUART_CR2_STOP_2_BIT (0x2UL << LPUART_CR2_STOP_Pos)
#define LPUART_CR2_STOP_1_5_BIT (0x3UL << LPUART_CR2_STOP_Pos)

#define LPUART_CR2_CLKEN_Pos 11U
#define LPUART_CR2_CLKEN_Msk (0x1UL << LPUART_CR2_CLKEN_Pos)
#define LPUART_CR2_CLKEN LPUART_CR2_CLKEN_Msk

#define LPUART_CR2_ADDM7_Pos 4U
#define LPUART_CR2_ADDM7_Msk (0x1UL << LPUART_CR2_ADDM7_Pos)
#define LPUART_CR2_ADDM7 LPUART_CR2_ADDM7_Msk

/*CR3 register.*/
#define LPUART_CR3_UCESM_Pos 23U
#define LPUART_CR3_UCESM_Msk (0x1UL << LPUART_CR3_UCESM_Pos)
#define LPUART_CR3_UCESM LPUART_CR3_UCESM_Msk

#define LPUART_CR3_WUFIE_Pos 22U
#define LPUART_CR3_WUFIE_Msk (0x1UL << LPUART_CR3_WUFIE_Pos)
#define LPUART_CR3_WUFIE LPUART_CR3_WUFIE_Msk

#define LPUART_CR3_WUS_Pos 20U
#define LPUART_CR3_WUS_Msk (0x1UL << LPUART_CR3_WUS_Pos)
#define LPUART_CR3_WUS LPUART_CR3_WUS_Msk

#define LPUART_CR3_DEP_Pos 15U
#define LPUART_CR3_DEP_Msk (0x1UL << LPUART_CR3_DEP_Pos)
#define LPUART_CR3_DEP LPUART_CR3_DEP_Msk

#define LPUART_CR3_DEM_Pos 14U
#define LPUART_CR3_DEM_Msk (0x1UL << LPUART_CR3_DEM_Pos)
#define LPUART_CR3_DEM LPUART_CR3_DEM_Msk

#define LPUART_CR3_DDRE_Pos 13U
#define LPUART_CR3_DDRE_Msk (0x1UL << LPUART_CR3_DDRE_Pos)
#define LPUART_CR3_DDRE LPUART_CR3_DDRE_Msk

#define LPUART_CR3_OVRDIS_Pos 12U
#define LPUART_CR3_OVRDIS_Msk (0x1UL << LPUART_CR3_OVRDIS_Pos)
#define LPUART_CR3_OVRDIS LPUART_CR3_OVRDIS_Msk

#define LPUART_CR3_CTSIE_Pos 10U
#define LPUART_CR3_CTSIE_Msk (0x1UL << LPUART_CR3_CTSIE_Pos)
#define LPUART_CR3_CTSIE LPUART_CR3_CTSIE_Msk

#define LPUART_CR3_CTSE_Pos 9U
#define LPUART_CR3_CTSE_Msk (0x1UL << LPUART_CR3_CTSE_Pos)
#define LPUART_CR3_CTSE LPUART_CR3_CTSE_Msk

#define LPUART_CR3_RTSE_Pos 8U
#define LPUART_CR3_RTSE_Msk (0x1UL << LPUART_CR3_RTSE_Pos)
#define LPUART_CR3_RTSE LPUART_CR3_RTSE_Msk

#define LPUART_CR3_DMAT_Pos 7U
#define LPUART_CR3_DMAT_Msk (0x1UL << LPUART_CR3_DMAT_Pos)
#define LPUART_CR3_DMAT LPUART_CR3_DMAT_Msk

#define LPUART_CR3_DMAR_Pos 6U
#define LPUART_CR3_DMAR_Msk (0x1UL << LPUART_CR3_DMAR_Pos)
#define LPUART_CR3_DMAR LPUART_CR3_DMAR_Msk

#define LPUART_CR3_HDSEL_Pos 3U
#define LPUART_CR3_HDSEL_Msk (0x1UL << LPUART_CR3_HDSEL_Pos)
#define LPUART_CR3_HDSEL LPUART_CR3_HDSEL_Msk

#define LPUART_CR3_EIE_Pos 0U
#define LPUART_CR3_EIE_Msk (0x1UL << LPUART_CR3_EIE_Pos)
#define LPUART_CR3_EIE LPUART_CR3_EIE_Msk

/*BRR register.*/
#define LPUART_BRR_Pos 0U
#define LPUART_BRR_Msk (0xFFFFFUL << LPUART_BRR_Pos)
#define LPUART_BRR LPUART_BRR_Msk

/*GTPR register.*/
#define LPUART_GTPR_GT_Pos 8U
#define LPUART_GTPR_GT_Msk (0xFFUL << LPUART_GTPR_GT_Pos)
#define LPUART_GTPR_GT LPUART_GTPR_GT_Msk

#define LPUART_GTPR_PSC_Pos 0U
#define LPUART_GTPR_PSC_Msk (0xFFUL << LPUART_GTPR_PSC_Pos)
#define LPUART_GTPR_PSC LPUART_GTPR_PSC_Msk

/*RTOR register.*/
#define LPUART_RTOR_BLEN_Pos 24U
#define LPUART_RTOR_BLEN_Msk (0xFFUL << LPUART_RTOR_BLEN_Pos)
#define LPUART_RTOR_BLEN LPUART_RTOR_BLEN_Msk

#define LPUART_RTOR_RTO_Pos 0U
#define LPUART_RTOR_RTO_Msk (0xFFFFFFUL << LPUART_RTOR_RTO_Pos)
#define LPUART_RTOR_RTO LPUART_RTOR_RTO_Msk

/*RQR register.*/
#define LPUART_RQR_RXFRQ_Pos 3U
#define LPUART_RQR_RXFRQ_Msk (0x1UL << LPUART_RQR_RXFRQ_Pos)
#define LPUART_RQR_RXFRQ LPUART_RQR_RXFRQ_Msk

#define LPUART_RQR_MMRQ_Pos 2U
#define LPUART_RQR_MMRQ_Msk (0x1UL << LPUART_RQR_MMRQ_Pos)
#define LPUART_RQR_MMRQ LPUART_RQR_MMRQ_Msk

#define LPUART_RQR_SBKRQ_Pos 1U
#define LPUART_RQR_SBKRQ_Msk (0x1UL << LPUART_RQR_SBKRQ_Pos)
#define LPUART_RQR_SBKRQ LPUART_RQR_SBKRQ_Msk

/*ISR register.*/
#define LPUART_ISR_REACK_Pos 22U
#define LPUART_ISR_REACK_Msk (0x1UL << LPUART_ISR_REACK_Pos)
#define LPUART_ISR_REACK LPUART_ISR_REACK_Msk

#define LPUART_ISR_TEACK_Pos 21U
#define LPUART_ISR_TEACK_Msk (0x1UL << LPUART_ISR_TEACK_Pos)
#define LPUART_ISR_TEACK LPUART_ISR_TEACK_Msk

#define LPUART_ISR_WUF_Pos 20U
#define LPUART_ISR_WUF_Msk (0x1UL << LPUART_ISR_WUF_Pos)
#define LPUART_ISR_WUF LPUART_ISR_WUF_Msk

#define LPUART_ISR_RWU_Pos 19U
#define LPUART_ISR_RWU_Msk (0x1UL << LPUART_ISR_RWU_Pos)
#define LPUART_ISR_RWU LPUART_ISR_RWU_Msk

#define LPUART_ISR_SBKF_Pos 18U
#define LPUART_ISR_SBKF_Msk (0x1UL << LPUART_ISR_SBKF_Pos)
#define LPUART_ISR_SBKF LPUART_ISR_SBKF_Msk

#define LPUART_ISR_CMF_Pos 17U
#define LPUART_ISR_CMF_Msk (0x1UL << LPUART_ISR_CMF_Pos)
#define LPUART_ISR_CMF LPUART_ISR_CMF_Msk

#define LPUART_ISR_BUSY_Pos 16U
#define LPUART_ISR_BUSY_Msk (0x1UL << LPUART_ISR_BUSY_Pos)
#define LPUART_ISR_BUSY LPUART_ISR_BUSY_Msk

#define LPUART_ISR_CTS_Pos 10U
#define LPUART_ISR_CTS_Msk (0x1UL << LPUART_ISR_CTS_Pos)
#define LPUART_ISR_CTS LPUART_ISR_CTS_Msk

#define LPUART_ISR_CTSIF_Pos 9U
#define LPUART_ISR_CTSIF_Msk (0x1UL << LPUART_ISR_CTSIF_Pos)
#define LPUART_ISR_CTSIF LPUART_ISR_CTSIF_Msk

#define LPUART_ISR_TXE_Pos 7U
#define LPUART_ISR_TXE_Msk (0x1UL << LPUART_ISR_TXE_Pos)
#define LPUART_ISR_TXE LPUART_ISR_TXE_Msk

#define LPUART_ISR_TC_Pos 6U
#define LPUART_ISR_TC_Msk (0x1UL << LPUART_ISR_TC_Pos)
#define LPUART_ISR_TC LPUART_ISR_TC_Msk

#define LPUART_ISR_RXNE_Pos 5U
#define LPUART_ISR_RXNE_Msk (0x1UL << LPUART_ISR_RXNE_Pos)
#define LPUART_ISR_RXNE LPUART_ISR_RXNE_Msk

#define LPUART_ISR_IDLE_Pos 4U
#define LPUART_ISR_IDLE_Msk (0x1UL << LPUART_ISR_IDLE_Pos)
#define LPUART_ISR_IDLE LPUART_ISR_IDLE_Msk

#define LPUART_ISR_ORE_Pos 3U
#define LPUART_ISR_ORE_Msk (0x1UL << LPUART_ISR_ORE_Pos)
#define LPUART_ISR_ORE LPUART_ISR_ORE_Msk

#define LPUART_ISR_NF_Pos 2U
#define LPUART_ISR_NF_Msk (0x1UL << LPUART_ISR_NF_Pos)
#define LPUART_ISR_NF LPUART_ISR_NF_Msk

#define LPUART_ISR_FE_Pos 1U
#define LPUART_ISR_FE_Msk (0x1UL << LPUART_ISR_FE_Pos)
#define LPUART_ISR_FE LPUART_ISR_FE_Msk

#define LPUART_ISR_PE_Pos 0U
#define LPUART_ISR_PE_Msk (0x1UL << LPUART_ISR_PE_Pos)
#define LPUART_ISR_PE LPUART_ISR_PE_Msk

/*ICR register.*/
#define LPUART_ICR_WUCF_Pos 20U
#define LPUART_ICR_WUCF_Msk (0x1UL << LPUART_ICR_WUCF_Pos)
#define LPUART_ICR_WUCF LPUART_ICR_WUCF_Msk

#define LPUART_ICR_CMCF_Pos 17U
#define LPUART_ICR_CMCF_Msk (0x1UL << LPUART_ICR_CMCF_Pos)
#define LPUART_ICR_CMCF LPUART_ICR_CMCF_Msk

#define LPUART_ICR_CTSCF_Pos 9U
#define LPUART_ICR_CTSCF_Msk (0x1UL << LPUART_ICR_CTSCF_Pos)
#define LPUART_ICR_CTSCF LPUART_ICR_CTSCF_Msk

#define LPUART_ICR_TCCF_Pos 6U
#define LPUART_ICR_TCCF_Msk (0x1UL << LPUART_ICR_TCCF_Pos)
#define LPUART_ICR_TCCF LPUART_ICR_TCCF_Msk

#define LPUART_ICR_IDLECF_Pos 4U
#define LPUART_ICR_IDLECF_Msk (0x1UL << LPUART_ICR_IDLECF_Pos)
#define LPUART_ICR_IDLECF LPUART_ICR_IDLECF_Msk

#define LPUART_ICR_ORECF_Pos 3U
#define LPUART_ICR_ORECF_Msk (0x1UL << LPUART_ICR_ORECF_Pos)
#define LPUART_ICR_ORECF LPUART_ICR_ORECF_Msk

#define LPUART_ICR_NCF_Pos 2U
#define LPUART_ICR_NCF_Msk (0x1UL << LPUART_ICR_NCF_Pos)
#define LPUART_ICR_NCF LPUART_ICR_NCF_Msk

#define LPUART_ICR_FECF_Pos 1U
#define LPUART_ICR_FECF_Msk (0x1UL << LPUART_ICR_FECF_Pos)
#define LPUART_ICR_FECF LPUART_ICR_FECF_Msk

#define LPUART_ICR_PECF_Pos 0U
#define LPUART_ICR_PECF_Msk (0x1UL << LPUART_ICR_PECF_Pos)
#define LPUART_ICR_PECF LPUART_ICR_PECF_Msk

/*RDR register.*/
#define LPUART_RDR_Pos 0U
#define LPUART_RDR_Msk (0xFFUL << LPUART_RDR_Pos)
#define LPUART_RDR LPUART_RDR_Msk

/*TDR register.*/
#define LPUART_TDR_Pos 0U
#define LPUART_TDR_Msk (0xFFUL << LPUART_TDR_Pos)
#define LPUART_TDR LPUART_TDR_Msk

/* ========================================================================= */
/* ============                       SPI                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR1; /*!< SPI Control Register 1. */
	__IOM uint32_t CR2; /*!< SPI Control Register 2. */
	__IOM uint32_t SR; /*!< SPI Status Register. */
	__IOM uint32_t DR; /*!< SPI Data Register. */
	__IOM uint32_t CRCPR; /*!< SPI CRC Polynomial Register. */
	__IM uint32_t RXCRCR; /*!< SPI Rx CRC Register. */
	__IM uint32_t TXCRCR; /*!< SPI Tx CRC Register. */
}STM32L4xx_SPI_TypeDef;

/*CR1 register.*/
#define SPI_CR1_BIDIMODE_Pos 15U
#define SPI_CR1_BIDIMODE_Msk (0x1UL << SPI_CR1_BIDIMODE_Pos)
#define SPI_CR1_BIDIMODE SPI_CR1_BIDIMODE_Msk

#define SPI_CR1_BIDIOE_Pos 14U
#define SPI_CR1_BIDIOE_Msk (0x1UL << SPI_CR1_BIDIOE_Pos)
#define SPI_CR1_BIDIOE SPI_CR1_BIDIOE_Msk

#define SPI_CR1_CRCEN_Pos 13U
#define SPI_CR1_CRCEN_Msk (0x1UL << SPI_CR1_CRCEN_Pos)
#define SPI_CR1_CRCEN SPI_CR1_CRCEN_Msk

#define SPI_CR1_CRCNEXT_Pos 12U
#define SPI_CR1_CRCNEXT_Msk (0x1UL << SPI_CR1_CRCNEXT_Pos)
#define SPI_CR1_CRCNEXT SPI_CR1_CRCNEXT_Msk

#define SPI_CR1_CRCL_Pos 11U
#define SPI_CR1_CRCL_Msk (0x1UL << SPI_CR1_CRCL_Pos)
#define SPI_CR1_CRCL SPI_CR1_CRCL_Msk

#define SPI_CR1_RXONLY_Pos 10U
#define SPI_CR1_RXONLY_Msk (0x1UL << SPI_CR1_RXONLY_Pos)
#define SPI_CR1_RXONLY SPI_CR1_RXONLY_Msk

#define SPI_CR1_SSM_Pos 9U
#define SPI_CR1_SSM_Msk (0x1UL << SPI_CR1_SSM_Pos)
#define SPI_CR1_SSM SPI_CR1_SSM_Msk

#define SPI_CR1_SSI_Pos 8U
#define SPI_CR1_SSI_Msk (0x1UL << SPI_CR1_SSI_Pos)
#define SPI_CR1_SSI SPI_CR1_SSI_Msk

#define SPI_CR1_LSBFIRST_Pos 7U
#define SPI_CR1_LSBFIRST_Msk (0x1UL << SPI_CR1_LSBFIRST_Pos)
#define SPI_CR1_LSBFIRST SPI_CR1_LSBFIRST_Msk

#define SPI_CR1_SPE_Pos 6U
#define SPI_CR1_SPE_Msk (0x1UL << SPI_CR1_SPE_Pos)
#define SPI_CR1_SPE SPI_CR1_SPE_Msk

#define SPI_CR1_BR_Pos 3U
#define SPI_CR1_BR_Msk (0x7UL << SPI_CR1_BR_Pos)
#define SPI_CR1_BR SPI_CR1_BR_Msk
#define SPI_CR1_BR_DIV_2 (0x0UL << SPI_CR1_BR_Pos)
#define SPI_CR1_BR_DIV_4 (0x1UL << SPI_CR1_BR_Pos)
#define SPI_CR1_BR_DIV_8 (0x2UL << SPI_CR1_BR_Pos)
#define SPI_CR1_BR_DIV_16 (0x3UL << SPI_CR1_BR_Pos)
#define SPI_CR1_BR_DIV_32 (0x4UL << SPI_CR1_BR_Pos)
#define SPI_CR1_BR_DIV_64 (0x5UL << SPI_CR1_BR_Pos)
#define SPI_CR1_BR_DIV_128 (0x6UL << SPI_CR1_BR_Pos)
#define SPI_CR1_BR_DIV_256 (0x7UL << SPI_CR1_BR_Pos)

#define SPI_CR1_MSTR_Pos 2U
#define SPI_CR1_MSTR_Msk (0x1UL << SPI_CR1_MSTR_Pos)
#define SPI_CR1_MSTR SPI_CR1_MSTR_Msk

#define SPI_CR1_CPOL_Pos 1U
#define SPI_CR1_CPOL_Msk (0x1UL << SPI_CR1_CPOL_Pos)
#define SPI_CR1_CPOL SPI_CR1_CPOL_Msk

#define SPI_CR1_CPHA_Pos 0U
#define SPI_CR1_CPHA_Msk (0x1UL << SPI_CR1_CPHA_Pos)
#define SPI_CR1_CPHA SPI_CR1_CPHA_Msk

/*CR2 register.*/
#define SPI_CR2_LDMA_TX_Pos 14U
#define SPI_CR2_LDMA_TX_Msk (0x1UL << SPI_CR2_LDMA_TX_Pos)
#define SPI_CR2_LDMA_TX SPI_CR2_LDMA_TX_Msk

#define SPI_CR2_LDMA_RX_Pos 13U
#define SPI_CR2_LDMA_RX_Msk (0x1UL << SPI_CR2_LDMA_RX_Pos)
#define SPI_CR2_LDMA_RX SPI_CR2_LDMA_RX_Msk

#define SPI_CR2_FRXTH_Pos 12U
#define SPI_CR2_FRXTH_Msk (0x1UL << SPI_CR2_FRXTH_Pos)
#define SPI_CR2_FRXTH SPI_CR2_FRXTH_Msk

#define SPI_CR2_DS_Pos 8U
#define SPI_CR2_DS_Msk (0xFUL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS SPI_CR2_DS_Msk
#define SPI_CR2_DS_4_BIT (0x3UL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS_5_BIT (0x4UL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS_6_BIT (0x5UL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS_7_BIT (0x6UL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS_8_BIT (0x7UL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS_9_BIT (0x8UL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS_10_BIT (0x9UL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS_11_BIT (0xAUL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS_12_BIT (0xBUL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS_13_BIT (0xCUL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS_14_BIT (0xDUL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS_15_BIT (0xEUL << SPI_CR2_DS_Pos)
#define SPI_CR2_DS_16_BIT (0xFUL << SPI_CR2_DS_Pos)

#define SPI_CR2_TXEIE_Pos 7U
#define SPI_CR2_TXEIE_Msk (0x1UL << SPI_CR2_TXEIE_Pos)
#define SPI_CR2_TXEIE SPI_CR2_TXEIE_Msk

#define SPI_CR2_RXEIE_Pos 6U
#define SPI_CR2_RXEIE_Msk (0x1UL << SPI_CR2_RXEIE_Pos)
#define SPI_CR2_RXEIE SPI_CR2_RXEIE_Msk

#define SPI_CR2_ERRIE_Pos 5U
#define SPI_CR2_ERRIE_Msk (0x1UL << SPI_CR2_ERRIE_Pos)
#define SPI_CR2_ERRIE SPI_CR2_ERRIE_Msk

#define SPI_CR2_FRF_Pos 4U
#define SPI_CR2_FRF_Msk (0x1UL << SPI_CR2_FRF_Pos)
#define SPI_CR2_FRF SPI_CR2_FRF_Msk

#define SPI_CR2_NSSP_Pos 3U
#define SPI_CR2_NSSP_Msk (0x1UL << SPI_CR2_NSSP_Pos)
#define SPI_CR2_NSSP SPI_CR2_NSSP_Msk

#define SPI_CR2_SSOE_Pos 2U
#define SPI_CR2_SSOE_Msk (0x1UL << SPI_CR2_SSOE_Pos)
#define SPI_CR2_SSOE SPI_CR2_SSOE_Msk

#define SPI_CR2_TXDMAEN_Pos 1U
#define SPI_CR2_TXDMAEN_Msk (0x1UL << SPI_CR2_TXDMAEN_Pos)
#define SPI_CR2_TXDMAEN SPI_CR2_TXDMAEN_Msk

#define SPI_CR2_RXDMAEN_Pos 0U
#define SPI_CR2_RXDMAEN_Msk (0x1UL << SPI_CR2_RXDMAEN_Pos)
#define SPI_CR2_RXDMAEN SPI_CR2_RXDMAEN_Msk

/*SR register.*/
#define SPI_SR_FTLVL_Pos 11U
#define SPI_SR_FTLVL_Msk (0x3UL << SPI_SR_FTLVL_Pos)
#define SPI_SR_FTLVL SPI_SR_FTLVL_Msk
#define SPI_SR_FTLVL_EMPTY (0x0UL << SPI_SR_FTLVL_Pos)
#define SPI_SR_FTLVL_1_DIV_4 (0x1UL << SPI_SR_FTLVL_Pos)
#define SPI_SR_FTLVL_1_DIV_2 (0x2UL << SPI_SR_FTLVL_Pos)
#define SPI_SR_FTLVL_FULL (0x3UL << SPI_SR_FTLVL_Pos)

#define SPI_SR_FRLVL_Pos 9U
#define SPI_SR_FRLVL_Msk (0x3UL << SPI_SR_FRLVL_Pos)
#define SPI_SR_FRLVL SPI_SR_FRLVL_Msk
#define SPI_SR_FRLVL_EMPTY (0x0UL << SPI_SR_FRLVL_Pos)
#define SPI_SR_FRLVL_1_DIV_4 (0x1UL << SPI_SR_FRLVL_Pos)
#define SPI_SR_FRLVL_1_DIV_2 (0x2UL << SPI_SR_FRLVL_Pos)
#define SPI_SR_FRLVL_FULL (0x3UL << SPI_SR_FRLVL_Pos)

#define SPI_SR_FRE_Pos 8U
#define SPI_SR_FRE_Msk (0x1UL << SPI_SR_FRE_Pos)
#define SPI_SR_FRE SPI_SR_FRE_Msk

#define SPI_SR_BSY_Pos 7U
#define SPI_SR_BSY_Msk (0x1UL << SPI_SR_BSY_Pos)
#define SPI_SR_BSY SPI_SR_BSY_Msk

#define SPI_SR_OVR_Pos 6U
#define SPI_SR_OVR_Msk (0x1UL << SPI_SR_OVR_Pos)
#define SPI_SR_OVR SPI_SR_OVR_Msk

#define SPI_SR_MODF_Pos 5U
#define SPI_SR_MODF_Msk (0x1UL << SPI_SR_MODF_Pos)
#define SPI_SR_MODF SPI_SR_MODF_Msk

#define SPI_SR_CRCERR_Pos 4U
#define SPI_SR_CRCERR_Msk (0x1UL << SPI_SR_CRCERR_Pos)
#define SPI_SR_CRCERR SPI_SR_CRCERR_Msk

#define SPI_SR_TXE_Pos 1U
#define SPI_SR_TXE_Msk (0x1UL << SPI_SR_TXE_Pos)
#define SPI_SR_TXE SPI_SR_TXE_Msk

#define SPI_SR_RXNE_Pos 0U
#define SPI_SR_RXNE_Msk (0x1UL << SPI_SR_RXNE_Pos)
#define SPI_SR_RXNE SPI_SR_RXNE_Msk

/*DR register.*/
#define SPI_DR_Pos 0U
#define SPI_DR_Msk (0xFFFFUL << SPI_DR_Pos)
#define SPI_DR SPI_DR_Msk

/*CRCPR register.*/
#define SPI_CRCPR_CRCPOLY_Pos 0U
#define SPI_CRCPR_CRCPOLY_Msk (0xFFFFUL << SPI_CRCPR_CRCPOLY_Pos)
#define SPI_CRCPR_CRCPOLY SPI_CRCPR_CRCPOLY_Msk

/*RXCRCR register.*/
#define SPI_RXCRCR_RXCRCR_Pos 0U
#define SPI_RXCRCR_RXCRCR_Msk (0xFFFFUL << SPI_RXCRCR_RXCRCR_Pos)
#define SPI_RXCRCR_RXCRCR SPI_RXCRCR_RXCRCR_Msk

/*TXCRCR register.*/
#define SPI_TXCRCR_TXCRCR_Pos 0U
#define SPI_TXCRCR_TXCRCR_Msk (0xFFFFUL << SPI_TXCRCR_RXCRCR_Pos)
#define SPI_TXCRCR_TXCRCR SPI_TXCRCR_TXCRCR_Msk

#if !defined(STM32L412) && !defined(STM32L422)
/* ========================================================================= */
/* ============                       SAI                       ============ */
/* ========================================================================= */
typedef struct
{
	uint32_t RESERVED0;
	__IOM uint32_t ACR1; /*!< SAI Configuration Register 1. */
	__IOM uint32_t ACR2; /*!< SAI Configuration Register 2. */
	__IOM uint32_t AFRCR; /*!< SAI Frame Configuration Register. */
	__IOM uint32_t ASLOTR; /*!< SAI Slot Register. */
	__IOM uint32_t AIM; /*!< SAI Interrupt Mask register. */
	__IM uint32_t ASR; /*!< SAI Status Register. */
	__OM uint32_t ACLRFR; /*!< SAI Clear Flag Register. */
	__IOM uint32_t ADR; /*!< SAI Data Register. */
	__IOM uint32_t BCR1; /*!< SAI Configuration Register 1. */
	__IOM uint32_t BCR2; /*!< SAI Configuration Register 2. */
	__IOM uint32_t BFRCR; /*!< SAI Frame Configuration Register. */
	__IOM uint32_t BSLOTR; /*!< SAI Slot Register. */
	__IOM uint32_t BIM; /*!< SAI Interrupt Mask register. */
	__IM uint32_t BSR; /*!< SAI Status Register. */
	__OM uint32_t BCLRFR; /*!< SAI Clear Flag Register. */
	__IOM uint32_t BDR; /*!< SAI Data Register. */
}STM32L4xx_SAI_TypeDef;

/*ACR1 register.*/
#define SAI_ACR1_MCKDIV_Pos 20U
#define SAI_ACR1_MCKDIV_Msk (0xFUL << SAI_ACR1_MCKDIV_Pos)
#define SAI_ACR1_MCKDIV SAI_ACR1_MCKDIV_Msk

#define SAI_ACR1_NODIV_Pos 19U
#define SAI_ACR1_NODIV_Msk (0x1UL << SAI_ACR1_NODIV_Pos)
#define SAI_ACR1_NODIV SAI_ACR1_NODIV_Msk

#define SAI_ACR1_DMAEN_Pos 17U
#define SAI_ACR1_DMAEN_Msk (0x1UL << SAI_ACR1_DMAEN_Pos)
#define SAI_ACR1_DMAEN SAI_ACR1_DMAEN_Msk

#define SAI_ACR1_SAIEN_Pos 16U
#define SAI_ACR1_SAIEN_Msk (0x1UL << SAI_ACR1_SAIEN_Pos)
#define SAI_ACR1_SAIEN SAI_ACR1_SAIEN_Msk

#define SAI_ACR1_OUTDRIV_Pos 13U
#define SAI_ACR1_OUTDRIV_Msk (0x1UL << SAI_ACR1_OUTDRIV_Pos)
#define SAI_ACR1_OUTDRIV SAI_ACR1_OUTDRIV_Msk

#define SAI_ACR1_MONO_Pos 12U
#define SAI_ACR1_MONO_Msk (0x1UL << SAI_ACR1_MONO_Pos)
#define SAI_ACR1_MONO SAI_ACR1_MONO_Msk

#define SAI_ACR1_SYNCEN_Pos 10U
#define SAI_ACR1_SYNCEN_Msk (0x3UL << SAI_ACR1_SYNCEN_Pos)
#define SAI_ACR1_SYNCEN SAI_ACR1_SYNCEN_Msk
#define SAI_ACR1_SYNCEN_ASYNCH (0x0UL << SAI_ACR1_SYNCEN_Pos)
#define SAI_ACR1_SYNCEN_SYNCH (0x1UL << SAI_ACR1_SYNCEN_Pos)

#define SAI_ACR1_CKSTR_Pos 9U
#define SAI_ACR1_CKSTR_Msk (0x1UL << SAI_ACR1_CKSTR_Pos)
#define SAI_ACR1_CKSTR SAI_ACR1_CKSTR_Msk

#define SAI_ACR1_LSBFIRST_Pos 8U
#define SAI_ACR1_LSBFIRST_Msk (0x1UL << SAI_ACR1_LSBFIRST_Pos)
#define SAI_ACR1_LSBFIRST SAI_ACR1_LSBFIRST_Msk

#define SAI_ACR1_DS_Pos 5U
#define SAI_ACR1_DS_Msk (0x7UL << SAI_ACR1_DS_Pos)
#define SAI_ACR1_DS SAI_ACR1_DS_Msk
#define SAI_ACR1_DS_8_BIT (0x2UL << SAI_ACR1_DS_Pos)
#define SAI_ACR1_DS_10_BIT (0x3UL << SAI_ACR1_DS_Pos)
#define SAI_ACR1_DS_16_BIT (0x4UL << SAI_ACR1_DS_Pos)
#define SAI_ACR1_DS_20_BIT (0x5UL << SAI_ACR1_DS_Pos)
#define SAI_ACR1_DS_24_BIT (0x6UL << SAI_ACR1_DS_Pos)
#define SAI_ACR1_DS_32_BIT (0x7UL << SAI_ACR1_DS_Pos)

#define SAI_ACR1_PRTCFG_Pos 2U
#define SAI_ACR1_PRTCFG_Msk (0x3UL << SAI_ACR1_PRTCFG_Pos)
#define SAI_ACR1_PRTCFG SAI_ACR1_PRTCFG_Msk
#define SAI_ACR1_PRTCFG_FREE (0x0UL << SAI_ACR1_PRTCFG_Pos)
#define SAI_ACR1_PRTCFG_SPDIF (0x1UL << SAI_ACR1_PRTCFG_Pos)
#define SAI_ACR1_PRTCFG_AC97 (0x2UL << SAI_ACR1_PRTCFG_Pos)

#define SAI_ACR1_MODE_Pos 0U
#define SAI_ACR1_MODE_Msk (0x3UL << SAI_ACR1_MODE_Pos)
#define SAI_ACR1_MODE SAI_ACR1_MODE_Msk
#define SAI_ACR1_MODE_MASTER_TRANSMITTER (0x0UL << SAI_ACR1_MODE_Pos)
#define SAI_ACR1_MODE_MASTER_RECEIVER (0x1UL << SAI_ACR1_MODE_Pos)
#define SAI_ACR1_MODE_SLAVE_TRANSMITTER (0x2UL << SAI_ACR1_MODE_Pos)
#define SAI_ACR1_MODE_SLAVE_RECEIVER (0x3UL << SAI_ACR1_MODE_Pos)

/*BCR1 register.*/
#define SAI_BCR1_MCKDIV_Pos 20U
#define SAI_BCR1_MCKDIV_Msk (0xFUL << SAI_BCR1_MCKDIV_Pos)
#define SAI_BCR1_MCKDIV SAI_BCR1_MCKDIV_Msk

#define SAI_BCR1_NODIV_Pos 19U
#define SAI_BCR1_NODIV_Msk (0x1UL << SAI_BCR1_NODIV_Pos)
#define SAI_BCR1_NODIV SAI_BCR1_NODIV_Msk

#define SAI_BCR1_DMAEN_Pos 17U
#define SAI_BCR1_DMAEN_Msk (0x1UL << SAI_BCR1_DMAEN_Pos)
#define SAI_BCR1_DMAEN SAI_BCR1_DMAEN_Msk

#define SAI_BCR1_SAIEN_Pos 16U
#define SAI_BCR1_SAIEN_Msk (0x1UL << SAI_BCR1_SAIEN_Pos)
#define SAI_BCR1_SAIEN SAI_BCR1_SAIEN_Msk

#define SAI_BCR1_OUTDRIV_Pos 13U
#define SAI_BCR1_OUTDRIV_Msk (0x1UL << SAI_BCR1_OUTDRIV_Pos)
#define SAI_BCR1_OUTDRIV SAI_BCR1_OUTDRIV_Msk

#define SAI_BCR1_MONO_Pos 12U
#define SAI_BCR1_MONO_Msk (0x1UL << SAI_BCR1_MONO_Pos)
#define SAI_BCR1_MONO SAI_BCR1_MONO_Msk

#define SAI_BCR1_SYNCEN_Pos 10U
#define SAI_BCR1_SYNCEN_Msk (0x3UL << SAI_BCR1_SYNCEN_Pos)
#define SAI_BCR1_SYNCEN SAI_BCR1_SYNCEN_Msk
#define SAI_BCR1_SYNCEN_ASYNCH (0x0UL << SAI_BCR1_SYNCEN_Pos)
#define SAI_BCR1_SYNCEN_SYNCH (0x1UL << SAI_BCR1_SYNCEN_Pos)

#define SAI_BCR1_CKSTR_Pos 9U
#define SAI_BCR1_CKSTR_Msk (0x1UL << SAI_BCR1_CKSTR_Pos)
#define SAI_BCR1_CKSTR SAI_BCR1_CKSTR_Msk

#define SAI_BCR1_LSBFIRST_Pos 8U
#define SAI_BCR1_LSBFIRST_Msk (0x1UL << SAI_BCR1_LSBFIRST_Pos)
#define SAI_BCR1_LSBFIRST SAI_BCR1_LSBFIRST_Msk

#define SAI_BCR1_DS_Pos 5U
#define SAI_BCR1_DS_Msk (0x7UL << SAI_BCR1_DS_Pos)
#define SAI_BCR1_DS SAI_BCR1_DS_Msk
#define SAI_BCR1_DS_8_BIT (0x2UL << SAI_BCR1_DS_Pos)
#define SAI_BCR1_DS_10_BIT (0x3UL << SAI_BCR1_DS_Pos)
#define SAI_BCR1_DS_16_BIT (0x4UL << SAI_BCR1_DS_Pos)
#define SAI_BCR1_DS_20_BIT (0x5UL << SAI_BCR1_DS_Pos)
#define SAI_BCR1_DS_24_BIT (0x6UL << SAI_BCR1_DS_Pos)
#define SAI_BCR1_DS_32_BIT (0x7UL << SAI_BCR1_DS_Pos)

#define SAI_BCR1_PRTCFG_Pos 2U
#define SAI_BCR1_PRTCFG_Msk (0x3UL << SAI_BCR1_PRTCFG_Pos)
#define SAI_BCR1_PRTCFG SAI_BCR1_PRTCFG_Msk
#define SAI_BCR1_PRTCFG_FREE (0x0UL << SAI_BCR1_PRTCFG_Pos)
#define SAI_BCR1_PRTCFG_SPDIF (0x1UL << SAI_BCR1_PRTCFG_Pos)
#define SAI_BCR1_PRTCFG_AC97 (0x2UL << SAI_BCR1_PRTCFG_Pos)

#define SAI_BCR1_MODE_Pos 0U
#define SAI_BCR1_MODE_Msk (0x3UL << SAI_BCR1_MODE_Pos)
#define SAI_BCR1_MODE SAI_BCR1_MODE_Msk
#define SAI_BCR1_MODE_MASTER_TRANSMITTER (0x0UL << SAI_BCR1_MODE_Pos)
#define SAI_BCR1_MODE_MASTER_RECEIVER (0x1UL << SAI_BCR1_MODE_Pos)
#define SAI_BCR1_MODE_SLAVE_TRANSMITTER (0x2UL << SAI_BCR1_MODE_Pos)
#define SAI_BCR1_MODE_SLAVE_RECEIVER (0x3UL << SAI_BCR1_MODE_Pos)

/*ACR2 register.*/
#define SAI_ACR2_COMP_Pos 14U
#define SAI_ACR2_COMP_Msk (0x3UL << SAI_ACR2_COMP_Pos)
#define SAI_ACR2_COMP SAI_ACR2_COMP_Msk

#define SAI_ACR2_CPL_Pos 13U
#define SAI_ACR2_CPL_Msk (0x1UL << SAI_ACR2_CPL_Pos)
#define SAI_ACR2_CPL SAI_ACR2_CPL_Msk

#define SAI_ACR2_MUTECNT_Pos 7U
#define SAI_ACR2_MUTECNT_Msk (0x3FUL << SAI_ACR2_MUTECNT_Pos)
#define SAI_ACR2_MUTECNT SAI_ACR2_MUTECNT_Msk

#define SAI_ACR2_MUTEVAL_Pos 6U
#define SAI_ACR2_MUTEVAL_Msk (0x1UL << SAI_ACR2_MUTEVAL_Pos)
#define SAI_ACR2_MUTEVAL SAI_ACR2_MUTEVAL_Msk

#define SAI_ACR2_MUTE_Pos 5U
#define SAI_ACR2_MUTE_Msk (0x1UL << SAI_ACR2_MUTE_Pos)
#define SAI_ACR2_MUTE SAI_ACR2_MUTE_Msk

#define SAI_ACR2_TRIS_Pos 4U
#define SAI_ACR2_TRIS_Msk (0x1UL << SAI_ACR2_TRIS_Pos)
#define SAI_ACR2_TRIS SAI_ACR2_TRIS_Msk

#define SAI_ACR2_FFLUSH_Pos 3U
#define SAI_ACR2_FFLUSH_Msk (0x1UL << SAI_ACR2_FFLUSH_Pos)
#define SAI_ACR2_FFLUSH SAI_ACR2_FFLUSH_Msk

#define SAI_ACR2_FTH_Pos 0U
#define SAI_ACR2_FTH_Msk (0x7UL << SAI_ACR2_FTH_Pos)
#define SAI_ACR2_FTH SAI_ACR2_FTH_Msk
#define SAI_ACR2_FTH_EMPTY (0x0UL << SAI_ACR2_FTH_Pos)
#define SAI_ACR2_FTH_1_DIV_4 (0x1UL << SAI_ACR2_FTH_Pos)
#define SAI_ACR2_FTH_1_DIV_2 (0x2UL << SAI_ACR2_FTH_Pos)
#define SAI_ACR2_FTH_3_DIV_4 (0x3UL << SAI_ACR2_FTH_Pos)
#define SAI_ACR2_FTH_FULL (0x4UL << SAI_ACR2_FTH_Pos)

/*BCR2 register.*/
#define SAI_BCR2_COMP_Pos 14U
#define SAI_BCR2_COMP_Msk (0x3UL << SAI_BCR2_COMP_Pos)
#define SAI_BCR2_COMP SAI_BCR2_COMP_Msk

#define SAI_BCR2_CPL_Pos 13U
#define SAI_BCR2_CPL_Msk (0x1UL << SAI_BCR2_CPL_Pos)
#define SAI_BCR2_CPL SAI_BCR2_CPL_Msk

#define SAI_BCR2_MUTECNT_Pos 7U
#define SAI_BCR2_MUTECNT_Msk (0x3FUL << SAI_BCR2_MUTECNT_Pos)
#define SAI_BCR2_MUTECNT SAI_BCR2_MUTECNT_Msk

#define SAI_BCR2_MUTEVAL_Pos 6U
#define SAI_BCR2_MUTEVAL_Msk (0x1UL << SAI_BCR2_MUTEVAL_Pos)
#define SAI_BCR2_MUTEVAL SAI_BCR2_MUTEVAL_Msk

#define SAI_BCR2_MUTE_Pos 5U
#define SAI_BCR2_MUTE_Msk (0x1UL << SAI_BCR2_MUTE_Pos)
#define SAI_BCR2_MUTE SAI_BCR2_MUTE_Msk

#define SAI_BCR2_TRIS_Pos 4U
#define SAI_BCR2_TRIS_Msk (0x1UL << SAI_BCR2_TRIS_Pos)
#define SAI_BCR2_TRIS SAI_BCR2_TRIS_Msk

#define SAI_BCR2_FFLUSH_Pos 3U
#define SAI_BCR2_FFLUSH_Msk (0x1UL << SAI_BCR2_FFLUSH_Pos)
#define SAI_BCR2_FFLUSH SAI_BCR2_FFLUSH_Msk

#define SAI_BCR2_FTH_Pos 0U
#define SAI_BCR2_FTH_Msk (0x7UL << SAI_BCR2_FTH_Pos)
#define SAI_BCR2_FTH SAI_BCR2_FTH_Msk
#define SAI_BCR2_FTH_EMPTY (0x0UL << SAI_BCR2_FTH_Pos)
#define SAI_BCR2_FTH_1_DIV_4 (0x1UL << SAI_BCR2_FTH_Pos)
#define SAI_BCR2_FTH_1_DIV_2 (0x2UL << SAI_BCR2_FTH_Pos)
#define SAI_BCR2_FTH_3_DIV_4 (0x3UL << SAI_BCR2_FTH_Pos)
#define SAI_BCR2_FTH_FULL (0x4UL << SAI_BCR2_FTH_Pos)

/*AFRCR register.*/
#define SAI_AFRCR_FSOFF_Pos 18U
#define SAI_AFRCR_FSOFF_Msk (0x1UL << SAI_AFRCR_FSOFF_Pos)
#define SAI_AFRCR_FSOFF SAI_AFRCR_FSOFF_Msk

#define SAI_AFRCR_FSPOL_Pos 17U
#define SAI_AFRCR_FSPOL_Msk (0x1UL << SAI_AFRCR_FSPOL_Pos)
#define SAI_AFRCR_FSPOL SAI_AFRCR_FSPOL_Msk

#define SAI_AFRCR_FSDEF_Pos 16U
#define SAI_AFRCR_FSDEF_Msk (0x1UL << SAI_AFRCR_FSDEF_Pos)
#define SAI_AFRCR_FSDEF SAI_AFRCR_FSDEF_Msk

#define SAI_AFRCR_FSALL_Pos 8U
#define SAI_AFRCR_FSALL_Msk (0x7FUL << SAI_AFRCR_FSALL_Pos)
#define SAI_AFRCR_FSALL SAI_AFRCR_FSALL_Msk

#define SAI_AFRCR_FRL_Pos 0U
#define SAI_AFRCR_FRL_Msk (0xFFUL << SAI_AFRCR_FRL_Pos)
#define SAI_AFRCR_FRL SAI_AFRCR_FRL_Msk

/*BFRCR register.*/
#define SAI_BFRCR_FSOFF_Pos 18U
#define SAI_BFRCR_FSOFF_Msk (0x1UL << SAI_BFRCR_FSOFF_Pos)
#define SAI_BFRCR_FSOFF SAI_BFRCR_FSOFF_Msk

#define SAI_BFRCR_FSPOL_Pos 17U
#define SAI_BFRCR_FSPOL_Msk (0x1UL << SAI_BFRCR_FSPOL_Pos)
#define SAI_BFRCR_FSPOL SAI_BFRCR_FSPOL_Msk

#define SAI_BFRCR_FSDEF_Pos 16U
#define SAI_BFRCR_FSDEF_Msk (0x1UL << SAI_BFRCR_FSDEF_Pos)
#define SAI_BFRCR_FSDEF SAI_BFRCR_FSDEF_Msk

#define SAI_BFRCR_FSALL_Pos 8U
#define SAI_BFRCR_FSALL_Msk (0x7FUL << SAI_BFRCR_FSALL_Pos)
#define SAI_BFRCR_FSALL SAI_BFRCR_FSALL_Msk

#define SAI_BFRCR_FRL_Pos 0U
#define SAI_BFRCR_FRL_Msk (0xFFUL << SAI_BFRCR_FRL_Pos)
#define SAI_BFRCR_FRL SAI_BFRCR_FRL_Msk

/*ASLOTR register.*/
#define SAI_ASLOTR_SLOTEN_Pos 16U
#define SAI_ASLOTR_SLOTEN_Msk (0xFFFFUL << SAI_ASLOTR_SLOTEN_Pos)
#define SAI_ASLOTR_SLOTEN SAI_ASLOTR_SLOTEN_Msk

#define SAI_ASLOTR_NBSLOT_Pos 8U
#define SAI_ASLOTR_NBSLOT_Msk (0xFUL << SAI_ASLOTR_NBSLOT_Pos)
#define SAI_ASLOTR_NBSLOT SAI_ASLOTR_NBSLOT_Msk

#define SAI_ASLOTR_SLOTSZ_Pos 6U
#define SAI_ASLOTR_SLOTSZ_Msk (0x3UL << SAI_ASLOTR_SLOTSZ_Pos)
#define SAI_ASLOTR_SLOTSZ SAI_ASLOTR_SLOTSZ_Msk
#define SAI_ASLOTR_SLOTSZ_DS (0x0UL << SAI_ASLOTR_SLOTSZ_Pos)
#define SAI_ASLOTR_SLOTSZ_16_BIT (0x1UL << SAI_ASLOTR_SLOTSZ_Pos)
#define SAI_ASLOTR_SLOTSZ_32_BIT (0x2UL << SAI_ASLOTR_SLOTSZ_Pos)

#define SAI_ASLOTR_FBOFF_Pos 0U
#define SAI_ASLOTR_FBOFF_Msk (0x1FUL << SAI_ASLOTR_FBOFF_Pos)
#define SAI_ASLOTR_FBOFF SAI_ASLOTR_FBOFF_Msk

/*BSLOTR register.*/
#define SAI_BSLOTR_SLOTEN_Pos 16U
#define SAI_BSLOTR_SLOTEN_Msk (0xFFFFUL << SAI_BSLOTR_SLOTEN_Pos)
#define SAI_BSLOTR_SLOTEN SAI_BSLOTR_SLOTEN_Msk

#define SAI_BSLOTR_NBSLOT_Pos 8U
#define SAI_BSLOTR_NBSLOT_Msk (0xFUL << SAI_BSLOTR_NBSLOT_Pos)
#define SAI_BSLOTR_NBSLOT SAI_BSLOTR_NBSLOT_Msk

#define SAI_BSLOTR_SLOTSZ_Pos 6U
#define SAI_BSLOTR_SLOTSZ_Msk (0x3UL << SAI_BSLOTR_SLOTSZ_Pos)
#define SAI_BSLOTR_SLOTSZ SAI_BSLOTR_SLOTSZ_Msk
#define SAI_BSLOTR_SLOTSZ_DS (0x0UL << SAI_BSLOTR_SLOTSZ_Pos)
#define SAI_BSLOTR_SLOTSZ_16_BIT (0x1UL << SAI_BSLOTR_SLOTSZ_Pos)
#define SAI_BSLOTR_SLOTSZ_32_BIT (0x2UL << SAI_BSLOTR_SLOTSZ_Pos)

#define SAI_BSLOTR_FBOFF_Pos 0U
#define SAI_BSLOTR_FBOFF_Msk (0x1FUL << SAI_BSLOTR_FBOFF_Pos)
#define SAI_BSLOTR_FBOFF SAI_BSLOTR_FBOFF_Msk

/*AIM register.*/
#define SAI_AIM_LFSDETIE_Pos 6U
#define SAI_AIM_LFSDETIE_Msk (0x1UL << SAI_AIM_LFSDETIE_Pos)
#define SAI_AIM_LFSDETIE SAI_AIM_LFSDETIE_Msk

#define SAI_AIM_AFSDETIE_Pos 5U
#define SAI_AIM_AFSDETIE_Msk (0x1UL << SAI_AIM_AFSDETIE_Pos)
#define SAI_AIM_AFSDETIE SAI_AIM_AFSDETIE_Msk

#define SAI_AIM_CNRDYIE_Pos 4U
#define SAI_AIM_CNRDYIE_Msk (0x1UL << SAI_AIM_CNRDYIE_Pos)
#define SAI_AIM_CNRDYIE SAI_AIM_CNRDYIE_Msk

#define SAI_AIM_FREQIE_Pos 3U
#define SAI_AIM_FREQIE_Msk (0x1UL << SAI_AIM_FREQIE_Pos)
#define SAI_AIM_FREQIE SAI_AIM_FREQIE_Msk

#define SAI_AIM_WCKCFGIE_Pos 2U
#define SAI_AIM_WCKCFGIE_Msk (0x1UL << SAI_AIM_WCKCFGIE_Pos)
#define SAI_AIM_WCKCFGIE SAI_AIM_WCKCFGIE_Msk

#define SAI_AIM_MUTEDETIE_Pos 1U
#define SAI_AIM_MUTEDETIE_Msk (0x1UL << SAI_AIM_MUTEDETIE_Pos)
#define SAI_AIM_MUTEDETIE SAI_AIM_MUTEDETIE_Msk

#define SAI_AIM_OVRUDRIE_Pos 0U
#define SAI_AIM_OVRUDRIE_Msk (0x1UL << SAI_AIM_OVRUDRIE_Pos)
#define SAI_AIM_OVRUDRIE SAI_AIM_OVRUDRIE_Msk

/*BIM register.*/
#define SAI_BIM_LFSDETIE_Pos 6U
#define SAI_BIM_LFSDETIE_Msk (0x1UL << SAI_BIM_LFSDETIE_Pos)
#define SAI_BIM_LFSDETIE SAI_BIM_LFSDETIE_Msk

#define SAI_BIM_AFSDETIE_Pos 5U
#define SAI_BIM_AFSDETIE_Msk (0x1UL << SAI_BIM_AFSDETIE_Pos)
#define SAI_BIM_AFSDETIE SAI_BIM_AFSDETIE_Msk

#define SAI_BIM_CNRDYIE_Pos 4U
#define SAI_BIM_CNRDYIE_Msk (0x1UL << SAI_BIM_CNRDYIE_Pos)
#define SAI_BIM_CNRDYIE SAI_BIM_CNRDYIE_Msk

#define SAI_BIM_FREQIE_Pos 3U
#define SAI_BIM_FREQIE_Msk (0x1UL << SAI_BIM_FREQIE_Pos)
#define SAI_BIM_FREQIE SAI_BIM_FREQIE_Msk

#define SAI_BIM_WCKCFGIE_Pos 2U
#define SAI_BIM_WCKCFGIE_Msk (0x1UL << SAI_BIM_WCKCFGIE_Pos)
#define SAI_BIM_WCKCFGIE SAI_BIM_WCKCFGIE_Msk

#define SAI_BIM_MUTEDETIE_Pos 1U
#define SAI_BIM_MUTEDETIE_Msk (0x1UL << SAI_BIM_MUTEDETIE_Pos)
#define SAI_BIM_MUTEDETIE SAI_BIM_MUTEDETIE_Msk

#define SAI_BIM_OVRUDRIE_Pos 0U
#define SAI_BIM_OVRUDRIE_Msk (0x1UL << SAI_BIM_OVRUDRIE_Pos)
#define SAI_BIM_OVRUDRIE SAI_BIM_OVRUDRIE_Msk

/*ASR register.*/
#define SAI_ASR_FLVL_Pos 16U
#define SAI_ASR_FLVL_Msk (0x7UL << SAI_ASR_FLVL_Pos)
#define SAI_ASR_FLVL SAI_ASR_FLVL_Msk

#define SAI_ASR_LFSDET_Pos 6U
#define SAI_ASR_LFSDET_Msk (0x1UL << SAI_ASR_LFSDET_Pos)
#define SAI_ASR_LFSDET SAI_ASR_LFSDET_Msk

#define SAI_ASR_AFSDET_Pos 5U
#define SAI_ASR_AFSDET_Msk (0x1UL << SAI_ASR_AFSDET_Pos)
#define SAI_ASR_AFSDET SAI_ASR_AFSDET_Msk

#define SAI_ASR_CNRDY_Pos 4U
#define SAI_ASR_CNRDY_Msk (0x1UL << SAI_ASR_CNRDY_Pos)
#define SAI_ASR_CNRDY SAI_ASR_CNRDY_Msk

#define SAI_ASR_FREQ_Pos 3U
#define SAI_ASR_FREQ_Msk (0x1UL << SAI_ASR_FREQ_Pos)
#define SAI_ASR_FREQ SAI_ASR_FREQ_Msk

#define SAI_ASR_WCKCFG_Pos 2U
#define SAI_ASR_WCKCFG_Msk (0x1UL << SAI_ASR_WCKCFG_Pos)
#define SAI_ASR_WCKCFG SAI_ASR_WCKCFG_Msk

#define SAI_ASR_MUTEDET_Pos 1U
#define SAI_ASR_MUTEDET_Msk (0x1UL << SAI_ASR_MUTEDET_Pos)
#define SAI_ASR_MUTEDET SAI_ASR_MUTEDET_Msk

#define SAI_ASR_OVRUDR_Pos 0U
#define SAI_ASR_OVRUDR_Msk (0x1UL << SAI_ASR_OVRUDR_Pos)
#define SAI_ASR_OVRUDR SAI_ASR_OVRUDR_Msk

/*BSR register.*/
#define SAI_BSR_FLVL_Pos 16U
#define SAI_BSR_FLVL_Msk (0x7UL << SAI_BSR_FLVL_Pos)
#define SAI_BSR_FLVL SAI_BSR_FLVL_Msk

#define SAI_BSR_LFSDET_Pos 6U
#define SAI_BSR_LFSDET_Msk (0x1UL << SAI_BSR_LFSDET_Pos)
#define SAI_BSR_LFSDET SAI_BSR_LFSDET_Msk

#define SAI_BSR_AFSDET_Pos 5U
#define SAI_BSR_AFSDET_Msk (0x1UL << SAI_BSR_AFSDET_Pos)
#define SAI_BSR_AFSDET SAI_BSR_AFSDET_Msk

#define SAI_BSR_CNRDY_Pos 4U
#define SAI_BSR_CNRDY_Msk (0x1UL << SAI_BSR_CNRDY_Pos)
#define SAI_BSR_CNRDY SAI_BSR_CNRDY_Msk

#define SAI_BSR_FREQ_Pos 3U
#define SAI_BSR_FREQ_Msk (0x1UL << SAI_BSR_FREQ_Pos)
#define SAI_BSR_FREQ SAI_BSR_FREQ_Msk

#define SAI_BSR_WCKCFG_Pos 2U
#define SAI_BSR_WCKCFG_Msk (0x1UL << SAI_BSR_WCKCFG_Pos)
#define SAI_BSR_WCKCFG SAI_BSR_WCKCFG_Msk

#define SAI_BSR_MUTEDET_Pos 1U
#define SAI_BSR_MUTEDET_Msk (0x1UL << SAI_BSR_MUTEDET_Pos)
#define SAI_BSR_MUTEDET SAI_BSR_MUTEDET_Msk

#define SAI_BSR_OVRUDR_Pos 0U
#define SAI_BSR_OVRUDR_Msk (0x1UL << SAI_BSR_OVRUDR_Pos)
#define SAI_BSR_OVRUDR SAI_BSR_OVRUDR_Msk

/*ACLRFR register.*/
#define SAI_ACLRFR_CLFSDET_Pos 6U
#define SAI_ACLRFR_CLFSDET_Msk (0x1UL << SAI_ACLRFR_CLFSDET_Pos)
#define SAI_ACLRFR_CLFSDET SAI_ACLRFR_CLFSDET_Msk

#define SAI_ACLRFR_CAFSDET_Pos 5U
#define SAI_ACLRFR_CAFSDET_Msk (0x1UL << SAI_ACLRFR_CAFSDET_Pos)
#define SAI_ACLRFR_CAFSDET SAI_ACLRFR_CAFSDET_Msk

#define SAI_ACLRFR_CCNRDY_Pos 4U
#define SAI_ACLRFR_CCNRDY_Msk (0x1UL << SAI_ACLRFR_CCNRDY_Pos)
#define SAI_ACLRFR_CCNRDY SAI_ACLRFR_CCNRDY_Msk

#define SAI_ACLRFR_CWCKCFG_Pos 2U
#define SAI_ACLRFR_CWCKCFG_Msk (0x1UL << SAI_ACLRFR_CWCKCFG_Pos)
#define SAI_ACLRFR_CWCKCFG SAI_ACLRFR_CWCKCFG_Msk

#define SAI_ACLRFR_CMUTEDET_Pos 1U
#define SAI_ACLRFR_CMUTEDET_Msk (0x1UL << SAI_ACLRFR_CMUTEDET_Pos)
#define SAI_ACLRFR_CMUTEDET SAI_ACLRFR_CMUTEDET_Msk

#define SAI_ACLRFR_COVRURD_Pos 0U
#define SAI_ACLRFR_COVRURD_Msk (0x1UL << SAI_ACLRFR_COVRURD_Pos)
#define SAI_ACLRFR_COVRURD SAI_ACLRFR_COVRURD_Msk

/*BCLRFR register.*/
#define SAI_BCLRFR_CLFSDET_Pos 6U
#define SAI_BCLRFR_CLFSDET_Msk (0x1UL << SAI_BCLRFR_CLFSDET_Pos)
#define SAI_BCLRFR_CLFSDET SAI_BCLRFR_CLFSDET_Msk

#define SAI_BCLRFR_CAFSDET_Pos 5U
#define SAI_BCLRFR_CAFSDET_Msk (0x1UL << SAI_BCLRFR_CAFSDET_Pos)
#define SAI_BCLRFR_CAFSDET SAI_BCLRFR_CAFSDET_Msk

#define SAI_BCLRFR_CCNRDY_Pos 4U
#define SAI_BCLRFR_CCNRDY_Msk (0x1UL << SAI_BCLRFR_CCNRDY_Pos)
#define SAI_BCLRFR_CCNRDY SAI_BCLRFR_CCNRDY_Msk

#define SAI_BCLRFR_CWCKCFG_Pos 2U
#define SAI_BCLRFR_CWCKCFG_Msk (0x1UL << SAI_BCLRFR_CWCKCFG_Pos)
#define SAI_BCLRFR_CWCKCFG SAI_BCLRFR_CWCKCFG_Msk

#define SAI_BCLRFR_CMUTEDET_Pos 1U
#define SAI_BCLRFR_CMUTEDET_Msk (0x1UL << SAI_BCLRFR_CMUTEDET_Pos)
#define SAI_BCLRFR_CMUTEDET SAI_BCLRFR_CMUTEDET_Msk

#define SAI_BCLRFR_COVRURD_Pos 0U
#define SAI_BCLRFR_COVRURD_Msk (0x1UL << SAI_BCLRFR_COVRURD_Pos)
#define SAI_BCLRFR_COVRURD SAI_BCLRFR_COVRURD_Msk
#endif

#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
/* ========================================================================= */
/* ============                      SWPMI                      ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CR; /*!< SWPMI Configuration/Control Register. */
	__IOM uint32_t BRR; /*!< SWPMI BitRate Register. */
	uint32_t RESERVED0; 
	__IM uint32_t ISR; /*!< SWPMI Interrupt and Status Register. */
	__IOM uint32_t ICR; /*!< SWPMI Interrupt flag Clear Register. */
	__IOM uint32_t IER; /*!< SWPMI Interrupt Enable Register. */
	__IM uint32_t RFL; /*!< SWPMI Receive Frame Length register. */
	__OM uint32_t TDR; /*!< SWPMI Transmit Data Register. */
	__IM uint32_t RDR; /*!< SWPMI Receive Data Register. */
	__IOM uint32_t OR; /*!< SWPMI Option Register. */
}STM32L4xx_SWPMI_TypeDef;

/*CR register.*/
#define SWPMI_CR_DEACT_Pos 10U
#define SWPMI_CR_DEACT_Msk (0x1UL << SWPMI_CR_DEACT_Pos)
#define SWPMI_CR_DEACT SWPMI_CR_DEACT_Msk

#define SWPMI_CR_SWPACT_Pos 5U
#define SWPMI_CR_SWPACT_Msk (0x1UL << SWPMI_CR_SWPACT_Pos)
#define SWPMI_CR_SWPACT SWPMI_CR_SWPACT_Msk

#define SWPMI_CR_LPBK_Pos 4U
#define SWPMI_CR_LPBK_Msk (0x1UL << SWPMI_CR_LPBK_Pos)
#define SWPMI_CR_LPBK SWPMI_CR_LPBK_Msk

#define SWPMI_CR_TXMODE_Pos 3U
#define SWPMI_CR_TXMODE_Msk (0x1UL << SWPMI_CR_TXMODE_Pos)
#define SWPMI_CR_TXMODE SWPMI_CR_TXMODE_Msk

#define SWPMI_CR_RXMODE_Pos 2U
#define SWPMI_CR_RXMODE_Msk (0x1UL << SWPMI_CR_RXMODE_Pos)
#define SWPMI_CR_RXMODE SWPMI_CR_RXMODE_Msk

#define SWPMI_CR_TXDMA_Pos 1U
#define SWPMI_CR_TXDMA_Msk (0x1UL << SWPMI_CR_TXDMA_Pos)
#define SWPMI_CR_TXDMA SWPMI_CR_TXDMA_Msk

#define SWPMI_CR_RXDMA_Pos 0U
#define SWPMI_CR_RXDMA_Msk (0x1UL << SWPMI_CR_RDMA_Pos)
#define SWPMI_CR_RXDMA SWPMI_CR_RXDMA_Msk

/*BRR register.*/
#define SWPMI_BRR_BR_Pos 0U
#define SWPMI_BRR_BR_Msk (0x3FUL << SWPMI_BBR_BR_Pos)
#define SWPMI_BRR_BR SWPMI_BRR_BR_Msk

/*ISR register.*/
#define SWPMI_ISR_DEACTF_Pos 10U
#define SWPMI_ISR_DEACTF_Msk (0x1UL << SWPMI_ISR_DEACTF_Pos)
#define SWPMI_ISR_DEACTF SWPMI_ISR_DEACTF_Msk

#define SWPMI_ISR_SUSP_Pos 9U
#define SWPMI_ISR_SUSP_Msk (0x1UL << SWPMI_ISR_SUSP_Pos)
#define SWPMI_ISR_SUSP SWPMI_ISR_SUSP_Msk

#define SWPMI_ISR_SRF_Pos 8U
#define SWPMI_ISR_SRF_Msk (0x1UL << SWPMI_ISR_SRF_Pos)
#define SWPMI_ISR_SRF SWPMI_ISR_SRF_Msk

#define SWPMI_ISR_TCF_Pos 7U
#define SWPMI_ISR_TCF_Msk (0x1UL << SWPMI_ISR_TCF_Pos)
#define SWPMI_ISR_TCF SWPMI_ISR_TCF_Msk

#define SWPMI_ISR_TXE_Pos 6U
#define SWPMI_ISR_TXE_Msk (0x1UL << SWPMI_ISR_TXE_Pos)
#define SWPMI_ISR_TXE SWPMI_ISR_TXE_Msk

#define SWPMI_ISR_RXNE_Pos 5U
#define SWPMI_ISR_RXNE_Msk (0x1UL << SWPMI_ISR_RXNE_Pos)
#define SWPMI_ISR_RXNE SWPMI_ISR_RXNE_Msk

#define SWPMI_ISR_TXUNRF_Pos 4U
#define SWPMI_ISR_TXUNRF_Msk (0x1UL << SWPMI_ISR_TXUNRF_Pos)
#define SWPMI_ISR_TXUNRF SWPMI_ISR_TXUNRF_Msk

#define SWPMI_ISR_RXOVRF_Pos 3U
#define SWPMI_ISR_RXOVRF_Msk (0x1UL << SWPMI_ISR_RXOVRF_Pos)
#define SWPMI_ISR_RXOVRF SWPMI_ISR_RXOVRF_Msk

#define SWPMI_ISR_RXBERF_Pos 2U
#define SWPMI_ISR_RXBERF_Msk (0x1UL << SWPMI_ISR_RXBERF_Pos)
#define SWPMI_ISR_RXBERF SWPMI_ISR_RXBERF_Msk

#define SWPMI_ISR_TXBEF_Pos 1U
#define SWPMI_ISR_TXBEF_Msk (0x1UL << SWPMI_ISR_TXBEF_Pos)
#define SWPMI_ISR_TXBEF SWPMI_ISR_TXBEF_Msk

#define SWPMI_ISR_RXBFF_Pos 0U
#define SWPMI_ISR_RXBFF_Msk (0x1UL << SWPMI_ISR_RXBFF_Pos)
#define SWPMI_ISR_RXBFF SWPMI_ISR_RXBFF_Msk

/*ICR register.*/
#define SWPMI_ICR_CSRF_Pos 8U
#define SWPMI_ICR_CSRF_Msk (0x1UL << SWPMI_ICR_CSRF_Pos)
#define SWPMI_ICR_CSRF SWPMI_ICR_CSRF_Msk

#define SWPMI_ICR_CTCF_Pos 7U
#define SWPMI_ICR_CTCF_Msk (0x1UL << SWPMI_ICR_CTCF_Pos)
#define SWPMI_ICR_CTCF SWPMI_ICR_CTCF_Msk

#define SWPMI_ICR_CTXUNRF_Pos 4U
#define SWPMI_ICR_CTXUNRF_Msk (0x1UL << SWPMI_ICR_CTXUNRF_Pos)
#define SWPMI_ICR_CTXUNRF SWPMI_ICR_CTXUNRF_Msk

#define SWPMI_ICR_CRXOVRF_Pos 3U
#define SWPMI_ICR_CRXOVRF_Msk (0x1UL << SWPMI_ICR_CRXOVRF_Pos)
#define SWPMI_ICR_CRXOVRF SWPMI_ICR_CRXOVRF_Msk

#define SWPMI_ICR_CRXBERF_Pos 2U
#define SWPMI_ICR_CRXBERF_Msk (0x1UL << SWPMI_ICR_CRXBERF_Pos)
#define SWPMI_ICR_CRXBERF SWPMI_ICR_CRXBERF_Msk

#define SWPMI_ICR_CTXBEF_Pos 1U
#define SWPMI_ICR_CTXBEF_Msk (0x1UL << SWPMI_ICR_CTXBEF_Pos)
#define SWPMI_ICR_CTXBEF SWPMI_ICR_CTXBEF_Msk

#define SWPMI_ICR_CRXBFF_Pos 0U
#define SWPMI_ICR_CRXBFF_Msk (0x1UL << SWPMI_ICR_CRXBFF_Pos)
#define SWPMI_ICR_CRXBFF SWPMI_ICR_CRXBFF_Msk

/*IER register.*/
#define SWPMI_IER_SRIE_Pos 8U
#define SWPMI_IER_SRIE_Msk (0x1UL << SWPMI_IER_SRIE_Pos)
#define SWPMI_IER_SRIE SWPMI_IER_SRIE_Msk

#define SWPMI_IER_TCIE_Pos 7U
#define SWPMI_IER_TCIE_Msk (0x1UL << SWPMI_IER_TCIE_Pos)
#define SWPMI_IER_TCIE SWPMI_IER_TCIE_Msk

#define SWPMI_IER_TIE_Pos 6U
#define SWPMI_IER_TIE_Msk (0x1UL << SWPMI_IER_TIE_Pos)
#define SWPMI_IER_TIE SWPMI_IER_TIE_Msk

#define SWPMI_IER_RIE_Pos 5U
#define SWPMI_IER_RIE_Msk (0x1UL << SWPMI_IER_RIE_Pos)
#define SWPMI_IER_RIE SWPMI_IER_RIE_Msk

#define SWPMI_IER_TXUNRIE_Pos 4U
#define SWPMI_IER_TXUNRIE_Msk (0x1UL << SWPMI_IER_TXUNRIE_Pos)
#define SWPMI_IER_TXUNRIE SWPMI_IER_TXUNRIE_Msk

#define SWPMI_IER_RXOVRIE_Pos 3U
#define SWPMI_IER_RXOVRIE_Msk (0x1UL << SWPMI_IER_RXOVRIE_Pos)
#define SWPMI_IER_RXOVRIE SWPMI_IER_RXOVRIE_Msk

#define SWPMI_IER_RXBERIE_Pos 2U
#define SWPMI_IER_RXBERIE_Msk (0x1UL << SWPMI_IER_RXBERIE_Pos)
#define SWPMI_IER_RXBERIE SWPMI_IER_RXBERIE_Msk

#define SWPMI_IER_TXBEIE_Pos 1U
#define SWPMI_IER_TXBEIE_Msk (0x1UL << SWPMI_IER_TXBEIE_Pos)
#define SWPMI_IER_TXBEIE SWPMI_IER_TXBEIE_Msk

#define SWPMI_IER_RXBFIE_Pos 0U
#define SWPMI_IER_RXBFIE_Msk (0x1UL << SWPMI_IER_RXBFIE_Pos)
#define SWPMI_IER_RXBFIE SWPMI_IER_RXBFIE_Msk

/*RFL register.*/
#define SWPMI_RFL_RFL_Pos 0U
#define SWPMI_RFL_RFL_Msk (0x1FUL << SWPMI_RFL_RFL_Pos)
#define SWPMI_RFL_RFL SWPMI_RFL_RFL_Msk

/*OR register.*/
#define SWPMI_OR_SWP_CLASS_Pos 1U
#define SWPMI_OR_SWP_CLASS_Msk (0x1UL << SWPMI_OR_SWP_CLASS_Pos)
#define SWPMI_OR_SWP_CLASS SWPMI_OR_SWP_CLASS_Msk

#define SWPMI_OR_SWP_TBYP_Pos 0U
#define SWPMI_OR_SWP_TBYP_Msk (0x1UL << SWPMI_OR_SWP_TBYP_Pos)
#define SWPMI_OR_SWP_TBYP SWPMI_OR_SWP_TBYP_Msk
#endif

#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
/* ========================================================================= */
/* ============                      SDMMC                      ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t POWER; /*!< SDMMC Power control Register. */
	__IOM uint32_t CLKCR; /*!< SDMMC Clock Control Register. */
	__IOM uint32_t ARG; /*!< SDMMC Argument register. */
	__IOM uint32_t CMD; /*!< SDMMC Command register. */
	__IM uint32_t RESPCMD; /*!< SDMMC Command Response register. */
	__IM uint32_t RESP1; /*!< SDMMC Response 1 register. */
	__IM uint32_t RESP2; /*!< SDMMC Response 2 register. */
	__IM uint32_t RESP3; /*!< SDMMC Response 3 register. */
	__IM uint32_t RESP4; /*!< SDMMC Response 4 register. */
	__IOM uint32_t DTIMER; /*!< SDMMC Data Timeout Register. */
	__IOM uint32_t DLEN; /*!< SDMMC Data Length register. */
	__IOM uint32_t DCTRL; /*!< SDMMC Data Control register. */
	__IM uint32_t DCOUNT; /*!< SDMMC Data Count register. */
	__IM uint32_t STA; /*!< SDMMC Status register. */
	__OM uint32_t ICR; /*!< SDMMC Interrupt Clear Register. */
	__IOM uint32_t MASK; /*!< SDMMC interrupt Mask register. */
	uint32_t RESERVED0[2U];
	__IM uint32_t FIFOCNT; /*!< SDMMC FIFO Counter register. */
	uint32_t RESERVED1[14U];
	__IOM uint32_t FIFO; /*!< SDMMC FIFO register. */
}STM32L4xx_SDMMC_TypeDef;

/*POWER register.*/
#define SDMMC_POWER_PWRCTRL_Pos 0U
#define SDMMC_POWER_PWRCTRL_Msk (0x3UL << SDMMC_POWER_PWRCTRL_Pos)
#define SDMMC_POWER_PWRCTRL SDMMC_POWER_PWRCTRL_Msk
#define SDMMC_POWER_PWRCTRL_OFF (0x0UL << SDMMC_POWER_PWRCTRL_Pos)
#define SDMMC_POWER_PWRCTRL_ON (0x3UL << SDMMC_POWER_PWRCTRL_Pos)

/*CLKCR register.*/
#define SDMMC_CLKCR_HWFC_EN_Pos 14U
#define SDMMC_CLKCR_HWFC_EN_Msk (0x1UL << SDMMC_CLKCR_HWFC_EN_Pos)
#define SDMMC_CLKCR_HWFC_EN SDMMC_CLKCR_HWFC_EN_Msk

#define SDMMC_CLKCR_NEGEDGE_Pos 13U
#define SDMMC_CLKCR_NEGEDGE_Msk (0x1UL << SDMMC_CLKCR_NEGEDGE_Pos)
#define SDMMC_CLKCR_NEGEDGE SDMMC_CLKCR_NEGEDGE_Msk

#define SDMMC_CLKCR_WIDBUS_Pos 11U
#define SDMMC_CLKCR_WIDBUS_Msk (0x3UL << SDMMC_CLKCR_WIDBUS_Pos)
#define SDMMC_CLKCR_WIDBUS SDMMC_CLKCR_WIDBUS_Msk
#define SDMMC_CLKCR_WIDBUS_DEFAULT (0x0UL << SDMMC_CLKCR_WIDBUS_Pos)
#define SDMMC_CLKCR_WIDBUS_4_WIDE (0x1UL << SDMMC_CLKCR_WIDBUS_Pos)
#define SDMMC_CLKCR_WIDBUS_8_WIDE (0x2UL << SDMMC_CLKCR_WIDBUS_Pos)

#define SDMMC_CLKCR_BYPASS_Pos 10U
#define SDMMC_CLKCR_BYPASS_Msk (0x1UL << SDMMC_CLKCR_BYPASS_Pos)
#define SDMMC_CLKCR_BYPASS SDMMC_CLKCR_BYPASS_Msk

#define SDMMC_CLKCR_PWRSAV_Pos 9U
#define SDMMC_CLKCR_PWRSAV_Msk (0x1UL << SDMMC_CLKCR_PWRSAV_Pos)
#define SDMMC_CLKCR_PWRSAV SDMMC_CLKCR_PWRSAV_Msk

#define SDMMC_CLKCR_CLKEN_Pos 8U
#define SDMMC_CLKCR_CLKEN_Msk (0x1UL << SDMMC_CLKCR_CLKEN_Pos)
#define SDMMC_CLKCR_CLKEN SDMMC_CLKCR_CLKEN_Msk

#define SDMMC_CLKCR_CLKDIV_Pos 0U
#define SDMMC_CLKCR_CLKDIV_Msk (0xFFUL << SDMMC_CLKCR_CLKDIV_Pos)
#define SDMMC_CLKCR_CLKDIV SDMMC_CLKCR_CLKDIV_Msk

/*CMD register.*/
#define SDMMC_CMD_SDIOSuspend_Pos 11U
#define SDMMC_CMD_SDIOSuspend_Msk (0x1UL << SDMMC_CMD_SDIOSuspend_Pos)
#define SDMMC_CMD_SDIOSuspend SDMMC_CMD_SDIOSuspend_Msk

#define SDMMC_CMD_CPSMEN_Pos 10U
#define SDMMC_CMD_CPSMEN_Msk (0x1UL << SDMMC_CMD_CPSMEN_Pos)
#define SDMMC_CMD_CPSMEN SDMMC_CMD_CPSMEN_Msk

#define SDMMC_CMD_WAITPEND_Pos 9U
#define SDMMC_CMD_WAITPEND_Msk (0x1UL << SDMMC_CMD_WAITPEND_Pos)
#define SDMMC_CMD_WAITPEND SDMMC_CMD_WAITPEND_Msk

#define SDMMC_CMD_WAITINT_Pos 8U
#define SDMMC_CMD_WAITINT_Msk (0x1UL << SDMMC_CMD_WAITINT_Pos)
#define SDMMC_CMD_WAITINT SDMMC_CMD_WAITINT_Msk

#define SDMMC_CMD_WAITRESP_Pos 6U
#define SDMMC_CMD_WAITRESP_Msk (0x3UL << SDMMC_CMD_WAITRESP_Pos)
#define SDMMC_CMD_WAITRESP SDMMC_CMD_WAITRESP_Msk
#define SDMMC_CMD_WAITRESP_NONE_0 (0x0UL << SDMMC_CMD_WAITRESP_Pos)
#define SDMMC_CMD_WAITRESP_SHORT (0x1UL << SDMMC_CMD_WAITRESP_Pos)
#define SDMMC_CMD_WAITRESP_NONE_1 (0x2UL << SDMMC_CMD_WAITRESP_Pos)
#define SDMMC_CMD_WAITRESP_LONG (0x3UL << SDMMC_CMD_WAITRESP_Pos)

#define SDMMC_CMD_CMDINDEX_Pos 0U
#define SDMMC_CMD_CMDINDEX_Msk (0x3FUL << SDMMC_CMD_CMDINDEX_Pos)
#define SDMMC_CMD_CMDINDEX SDMMC_CMD_CMDINDEX_Msk

/*RESPCMD register.*/
#define SDMMC_RESPCMD_RESPCMD_Pos 0U
#define SDMMC_RESPCMD_RESPCMD_Msk (0x3FUL << SDMMC_RESPCMD_RESPCMD_Pos)
#define SDMMC_RESPCMD_RESPCMD SDMMC_RESPCMD_RESPCMD_Msk

/*DLEN register.*/
#define SDMMC_DLEN_DATALENGTH_Pos 0U
#define SDMMC_DLEN_DATALENGTH_Msk (0x1FFFFFFUL << SDMMC_DLEN_DATALENGTH_Pos)
#define SDMMC_DLEN_DATALENGTH SDMMC_DLEN_DATALENGTH_Msk

/*DCTRL register.*/
#define SDMMC_DCTRL_SDIOEN_Pos 11U
#define SDMMC_DCTRL_SDIOEN_Msk (0x1UL << SDMMC_DCTRL_SDIOEN_Pos)
#define SDMMC_DCTRL_SDIOEN SDMMC_DCTRL_SDIOEN_Msk

#define SDMMC_DCTRL_RWMOD_Pos 10U
#define SDMMC_DCTRL_RWMOD_Msk (0x1UL << SDMMC_DCTRL_RWMOD_Pos)
#define SDMMC_DCTRL_RWMOD SDMMC_DCTRL_RWMOD_Msk

#define SDMMC_DCTRL_RWSTOP_Pos 9U
#define SDMMC_DCTRL_RWSTOP_Msk (0x1UL << SDMMC_DCTRL_RWSTOP_Pos)
#define SDMMC_DCTRL_RWSTOP SDMMC_DCTRL_RWSTOP_Msk

#define SDMMC_DCTRL_RWSTART_Pos 8U
#define SDMMC_DCTRL_RWSTART_Msk (0x1UL << SDMMC_DCTRL_RWSTART_Pos)
#define SDMMC_DCTRL_RWSTART SDMMC_DCTRL_RWSTART_Msk

#define SDMMC_DCTRL_DBLOCKSIZE_Pos 4U
#define SDMMC_DCTRL_DBLOCKSIZE_Msk (0xFUL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE SDMMC_DCTRL_DBLOCKSIZE_Msk
#define SDMMC_DCTRL_DBLOCKSIZE_0 (0x0UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_1 (0x1UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_2 (0x2UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_3 (0x3UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_4 (0x4UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_5 (0x5UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_6 (0x6UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_7 (0x7UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_8 (0x8UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_9 (0x9UL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_10 (0xAUL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_11 (0xBUL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_12 (0xCUL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_13 (0xDUL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_14 (0xEUL << SDMMC_DCTRL_DBLOCKSIZE_Pos)
#define SDMMC_DCTRL_DBLOCKSIZE_15 (0xFUL << SDMMC_DCTRL_DBLOCKSIZE_Pos)

#define SDMMC_DCTRL_DMAEN_Pos 3U
#define SDMMC_DCTRL_DMAEN_Msk (0x1UL << SDMMC_DCTRL_DMAEN_Pos)
#define SDMMC_DCTRL_DMAEN SDMMC_DCTRL_DMAEN_Msk

#define SDMMC_DCTRL_DTMODE_Pos 2U
#define SDMMC_DCTRL_DTMODE_Msk (0x1UL << SDMMC_DCTRL_DTMODE_Pos)
#define SDMMC_DCTRL_DTMODE SDMMC_DCTRL_DTMODE_Msk

#define SDMMC_DCTRL_DTDIR_Pos 1U
#define SDMMC_DCTRL_DTDIR_Msk (0x1UL << SDMMC_DCTRL_DTDIR_Pos)
#define SDMMC_DCTRL_DTDIR SDMMC_DCTRL_DTDIR_Msk

#define SDMMC_DCTRL_DTEN_Pos 0U
#define SDMMC_DCTRL_DTEN_Msk (0x1UL << SDMMC_DCTRL_DTEN_Pos)
#define SDMMC_DCTRL_DTEN SDMMC_DCTRL_DTEN_Msk

/*DCOUNT register.*/
#define SDMMC_DCOUNT_DATACOUNT_Pos 0U
#define SDMMC_DCOUNT_DATACOUNT_Msk (0x1FFFFFFUL << SDMMC_DCOUNT_DATACOUNT_Pos)
#define SDMMC_DCOUNT_DATACOUNT SDMMC_DCOUNT_DATACOUNT_Msk

/*STA register.*/
#define SDMMC_STA_SDIOIT_Pos 22U
#define SDMMC_STA_SDIOIT_Msk (0x1UL << SDMMC_STA_SDIOIT_Pos)
#define SDMMC_STA_SDIOIT SDMMC_STA_SDIOIT_Msk

#define SDMMC_STA_RXDAVL_Pos 21U
#define SDMMC_STA_RXDAVL_Msk (0x1UL << SDMMC_STA_RXDAVL_Pos)
#define SDMMC_STA_RXDAVL SDMMC_STA_RXDAVL_Msk

#define SDMMC_STA_TXDAVL_Pos 20U
#define SDMMC_STA_TXDAVL_Msk (0x1UL << SDMMC_STA_TXDAVL_Pos)
#define SDMMC_STA_TXDAVL SDMMC_STA_TXDAVL_Msk

#define SDMMC_STA_RXFIFOE_Pos 19U
#define SDMMC_STA_RXFIFOE_Msk (0x1UL << SDMMC_STA_RXFIFOE_Pos)
#define SDMMC_STA_RXFIFOE SDMMC_STA_RXFIFOE_Msk

#define SDMMC_STA_TXFIFOE_Pos 18U
#define SDMMC_STA_TXFIFOE_Msk (0x1UL << SDMMC_STA_TXFIFOE_Pos)
#define SDMMC_STA_TXFIFOE SDMMC_STA_TXFIFOE_Msk

#define SDMMC_STA_RXFIFOF_Pos 17U
#define SDMMC_STA_RXFIFOF_Msk (0x1UL << SDMMC_STA_RXFIFOF_Pos)
#define SDMMC_STA_RXFIFOF SDMMC_STA_RXFIFOF_Msk

#define SDMMC_STA_TXFIFOF_Pos 16U
#define SDMMC_STA_TXFIFOF_Msk (0x1UL << SDMMC_STA_TXFIFOF_Pos)
#define SDMMC_STA_TXFIFOF SDMMC_STA_TXFIFOF_Msk

#define SDMMC_STA_RXFIFOHF_Pos 15U
#define SDMMC_STA_RXFIFOHF_Msk (0x1UL << SDMMC_STA_RXFIFOHF_Pos)
#define SDMMC_STA_RXFIFOHF SDMMC_STA_RXFIFOHF_Msk

#define SDMMC_STA_TXFIFOHE_Pos 14U
#define SDMMC_STA_TXFIFOHE_Msk (0x1UL << SDMMC_STA_TXFIFOHE_Pos)
#define SDMMC_STA_TXFIFOHE SDMMC_STA_TXFIFOHE_Msk

#define SDMMC_STA_RXACT_Pos 13U
#define SDMMC_STA_RXACT_Msk (0x1UL << SDMMC_STA_RXACT_Pos)
#define SDMMC_STA_RXACT SDMMC_STA_RXACT_Msk

#define SDMMC_STA_TXACT_Pos 12U
#define SDMMC_STA_TXACT_Msk (0x1UL << SDMMC_STA_TXACT_Pos)
#define SDMMC_STA_TXACT SDMMC_STA_TXACT_Msk

#define SDMMC_STA_CMDACT_Pos 11U
#define SDMMC_STA_CMDACT_Msk (0x1UL << SDMMC_STA_CMDACT_Pos)
#define SDMMC_STA_CMDACT SDMMC_STA_CMDACT_Msk

#define SDMMC_STA_DBCKEND_Pos 10U
#define SDMMC_STA_DBCKEND_Msk (0x1UL << SDMMC_STA_DBCKEND_Pos)
#define SDMMC_STA_DBCKEND SDMMC_STA_DBCKEND_Msk

#define SDMMC_STA_DATAEND_Pos 8U
#define SDMMC_STA_DATAEND_Msk (0x1UL << SDMMC_STA_DATAEND_Pos)
#define SDMMC_STA_DATAEND SDMMC_STA_DATAEND_Msk

#define SDMMC_STA_CMDSENT_Pos 7U
#define SDMMC_STA_CMDSENT_Msk (0x1UL << SDMMC_STA_CMDSENT_Pos)
#define SDMMC_STA_CMDSENT SDMMC_STA_CMDSENT_Msk

#define SDMMC_STA_CMDREND_Pos 6U
#define SDMMC_STA_CMDREND_Msk (0x1UL << SDMMC_STA_CMDREND_Pos)
#define SDMMC_STA_CMDREND SDMMC_STA_CMDREND_Msk

#define SDMMC_STA_RXOVERR_Pos 5U
#define SDMMC_STA_RXOVERR_Msk (0x1UL << SDMMC_STA_RXOVERR_Pos)
#define SDMMC_STA_RXOVERR SDMMC_STA_RXOVERR_Msk

#define SDMMC_STA_TXUNDERR_Pos 4U
#define SDMMC_STA_TXUNDERR_Msk (0x1UL << SDMMC_STA_TXUNDERR_Pos)
#define SDMMC_STA_TXUNDERR SDMMC_STA_TXUNDERR_Msk

#define SDMMC_STA_DTIMEOUT_Pos 3U
#define SDMMC_STA_DTIMEOUT_Msk (0x1UL << SDMMC_STA_DTIMEOUT_Pos)
#define SDMMC_STA_DTIMEOUT SDMMC_STA_DTIMEOUT_Msk

#define SDMMC_STA_CTIMEOUT_Pos 2U
#define SDMMC_STA_CTIMEOUT_Msk (0x1UL << SDMMC_STA_CTIMEOUT_Pos)
#define SDMMC_STA_CTIMEOUT SDMMC_STA_CTIMEOUT_Msk

#define SDMMC_STA_DCRCFAIL_Pos 1U
#define SDMMC_STA_DCRCFAIL_Msk (0x1UL << SDMMC_STA_DCRCFAIL_Pos)
#define SDMMC_STA_DCRCFAIL SDMMC_STA_DCRCFAIL_Msk

#define SDMMC_STA_CCRCFAIL_Pos 0U
#define SDMMC_STA_CCRCFAIL_Msk (0x1UL << SDMMC_STA_CCRCFAIL_Pos)
#define SDMMC_STA_CCRCFAIL SDMMC_STA_CCRCFAIL_Msk

/*ICR register.*/
#define SDMMC_ICR_SDIOITC_Pos 22U
#define SDMMC_ICR_SDIOITC_Msk (0x1UL << SDMMC_ICR_SDIOITC_Pos)
#define SDMMC_ICR_SDIOITC SDMMC_ICR_SDIOITC_Msk

#define SDMMC_ICR_DBCKENDC_Pos 10U
#define SDMMC_ICR_DBCKENDC_Msk (0x1UL << SDMMC_ICR_DBCKENDC_Pos)
#define SDMMC_ICR_DBCKENDC SDMMC_ICR_DBCKENDC_Msk

#define SDMMC_ICR_DATAENDC_Pos 8U
#define SDMMC_ICR_DATAENDC_Msk (0x1UL << SDMMC_ICR_DATAENDC_Pos)
#define SDMMC_ICR_DATAENDC SDMMC_ICR_DATAENDC_Msk

#define SDMMC_ICR_CMDSENTC_Pos 7U
#define SDMMC_ICR_CMDSENTC_Msk (0x1UL << SDMMC_ICR_CMDSENTC_Pos)
#define SDMMC_ICR_CMDSENTC SDMMC_ICR_CMDSENTC_Msk

#define SDMMC_ICR_CMDRENDC_Pos 6U
#define SDMMC_ICR_CMDRENDC_Msk (0x1UL << SDMMC_ICR_CMDRENDC_Pos)
#define SDMMC_ICR_CMDRENDC SDMMC_ICR_CMDRENDC_Msk

#define SDMMC_ICR_RXOVERRC_Pos 5U
#define SDMMC_ICR_RXOVERRC_Msk (0x1UL << SDMMC_ICR_RXOVERRC_Pos)
#define SDMMC_ICR_RXOVERRC SDMMC_ICR_RXOVERRC_Msk

#define SDMMC_ICR_TXUNDERRC_Pos 4U
#define SDMMC_ICR_TXUNDERRC_Msk (0x1UL << SDMMC_ICR_TXUNDERRC_Pos)
#define SDMMC_ICR_TXUNDERRC SDMMC_ICR_TXUNDERRC_Msk

#define SDMMC_ICR_DTIMEOUTC_Pos 3U
#define SDMMC_ICR_DTIMEOUTC_Msk (0x1UL << SDMMC_ICR_DTIMEOUTC_Pos)
#define SDMMC_ICR_DTIMEOUTC SDMMC_ICR_DTIMEOUTC_Msk

#define SDMMC_ICR_CTIMEOUTC_Pos 2U
#define SDMMC_ICR_CTIMEOUTC_Msk (0x1UL << SDMMC_ICR_CTIMEOUTC_Pos)
#define SDMMC_ICR_CTIMEOUTC SDMMC_ICR_CTIMEOUTC_Msk

#define SDMMC_ICR_DCRCFAILC_Pos 1U
#define SDMMC_ICR_DCRCFAILC_Msk (0x1UL << SDMMC_ICR_DCRCFAILC_Pos)
#define SDMMC_ICR_DCRCFAILC SDMMC_ICR_DCRCFAILC_Msk

#define SDMMC_ICR_CCRCFAILC_Pos 0U
#define SDMMC_ICR_CCRCFAILC_Msk (0x1UL << SDMMC_ICR_CCRCFAILC_Pos)
#define SDMMC_ICR_CCRCFAILC SDMMC_ICR_CCRCFAILC_Msk

/*MASK register.*/
#define SDMMC_MASK_SDIOTIE_Pos 22U
#define SDMMC_MASK_SDIOTIE_Msk (0x1UL << SDMMC_MASK_SDIOTIE_Pos)
#define SDMMC_MASK_SDIOTIE SDMMC_MASK_SDIOTIE_Msk

#define SDMMC_MASK_RXDAVLIE_Pos 21U
#define SDMMC_MASK_RXDAVLIE_Msk (0x1UL << SDMMC_MASK_RXDAVLIE_Pos)
#define SDMMC_MASK_RXDAVLIE SDMMC_MASK_RXDAVLIE_Msk

#define SDMMC_MASK_TXDAVLIE_Pos 20U
#define SDMMC_MASK_TXDAVLIE_Msk (0x1UL << SDMMC_MASK_TXDAVLIE_Pos)
#define SDMMC_MASK_TXDAVLIE SDMMC_MASK_TXDAVLIE_Msk

#define SDMMC_MASK_RXFIFOEIE_Pos 19U
#define SDMMC_MASK_RXFIFOEIE_Msk (0x1UL << SDMMC_MASK_RXFIFOEIE_Pos)
#define SDMMC_MASK_RXFIFOEIE SDMMC_MASK_RXFIFOEIE_Msk

#define SDMMC_MASK_TXFIFOEIE_Pos 18U
#define SDMMC_MASK_TXFIFOEIE_Msk (0x1UL << SDMMC_MASK_TXFIFOEIE_Pos)
#define SDMMC_MASK_TXFIFOEIE SDMMC_MASK_TXFIFOEIE_Msk

#define SDMMC_MASK_RXFIFOFIE_Pos 17U
#define SDMMC_MASK_RXFIFOFIE_Msk (0x1UL << SDMMC_MASK_RXFIFOFIE_Pos)
#define SDMMC_MASK_RXFIFOFIE SDMMC_MASK_RXFIFOFIE_Msk

#define SDMMC_MASK_TXFIFOFIE_Pos 16U
#define SDMMC_MASK_TXFIFOFIE_Msk (0x1UL << SDMMC_MASK_TXFIFOFIE_Pos)
#define SDMMC_MASK_TXFIFOFIE SDMMC_MASK_TXFIFOFIE_Msk

#define SDMMC_MASK_RXFIFOHFIE_Pos 15U
#define SDMMC_MASK_RXFIFOHFIE_Msk (0x1UL << SDMMC_MASK_RXFIFOHFIE_Pos)
#define SDMMC_MASK_RXFIFOHFIE SDMMC_MASK_RXFIFOHFIE_Msk

#define SDMMC_MASK_TXFIFOHEIE_Pos 14U
#define SDMMC_MASK_TXFIFOHEIE_Msk (0x1UL << SDMMC_MASK_TXFIFOHEIE_Pos)
#define SDMMC_MASK_TXFIFOHEIE SDMMC_MASK_TXFIFOHEIE_Msk

#define SDMMC_MASK_RXACTIE_Pos 13U
#define SDMMC_MASK_RXACTIE_Msk (0x1UL << SDMMC_MASK_RXACTIE_Pos)
#define SDMMC_MASK_RXACTIE SDMMC_MASK_RXACTIE_Msk

#define SDMMC_MASK_TXACTIE_Pos 12U
#define SDMMC_MASK_TXACTIE_Msk (0x1UL << SDMMC_MASK_TXACTIE_Pos)
#define SDMMC_MASK_TXACTIE SDMMC_MASK_TXACTIE_Msk

#define SDMMC_MASK_CMDACTIE_Pos 11U
#define SDMMC_MASK_CMDACTIE_Msk (0x1UL << SDMMC_MASK_CMDACTIE_Pos)
#define SDMMC_MASK_CMDACTIE SDMMC_MASK_CMDACTIE_Msk

#define SDMMC_MASK_DBCKENDIE_Pos 10U
#define SDMMC_MASK_DBCKENDIE_Msk (0x1UL << SDMMC_MASK_DBCKENDIE_Pos)
#define SDMMC_MASK_DBCKENDIE SDMMC_MASK_DBCKENDIE_Msk

#define SDMMC_MASK_DATAENDIE_Pos 8U
#define SDMMC_MASK_DATAENDIE_Msk (0x1UL << SDMMC_MASK_DATAENDIE_Pos)
#define SDMMC_MASK_DATAENDIE SDMMC_MASK_DATAENDIE_Msk

#define SDMMC_MASK_CMDSENTIE_Pos 7U
#define SDMMC_MASK_CMDSENTIE_Msk (0x1UL << SDMMC_MASK_CMDSENTIE_Pos)
#define SDMMC_MASK_CMDSENTIE SDMMC_MASK_CMDSENTIE_Msk

#define SDMMC_MASK_CMDRENDIE_Pos 6U
#define SDMMC_MASK_CMDRENDIE_Msk (0x1UL << SDMMC_MASK_CMDRENDIE_Pos)
#define SDMMC_MASK_CMDRENDIE SDMMC_MASK_CMDRENDIE_Msk

#define SDMMC_MASK_RXOVERRIE_Pos 5U
#define SDMMC_MASK_RXOVERRIE_Msk (0x1UL << SDMMC_MASK_RXOVERRIE_Pos)
#define SDMMC_MASK_RXOVERRIE SDMMC_MASK_RXOVERRIE_Msk

#define SDMMC_MASK_TXUNDERRIE_Pos 4U
#define SDMMC_MASK_TXUNDERRIE_Msk (0x1UL << SDMMC_MASK_TXUNDERRIE_Pos)
#define SDMMC_MASK_TXUNDERRIE SDMMC_MASK_TXUNDERRIE_Msk

#define SDMMC_MASK_DTIMEOUTIE_Pos 3U
#define SDMMC_MASK_DTIMEOUTIE_Msk (0x1UL << SDMMC_MASK_DTIMEOUTIE_Pos)
#define SDMMC_MASK_DTIMEOUTIE SDMMC_MASK_DTIMEOUTIE_Msk

#define SDMMC_MASK_CTIMEOUTIE_Pos 2U
#define SDMMC_MASK_CTIMEOUTIE_Msk (0x1UL << SDMMC_MASK_CTIMEOUTIE_Pos)
#define SDMMC_MASK_CTIMEOUTIE SDMMC_MASK_CTIMEOUTIE_Msk

#define SDMMC_MASK_DCRCFAILIE_Pos 1U
#define SDMMC_MASK_DCRCFAILIE_Msk (0x1UL << SDMMC_MASK_DCRCFAILIE_Pos)
#define SDMMC_MASK_DCRCFAILIE SDMMC_MASK_DCRCFAILIE_Msk

#define SDMMC_MASK_CCRCFAILIE_Pos 0U
#define SDMMC_MASK_CCRCFAILIE_Msk (0x1UL << SDMMC_MASK_CCRCFAILIE_Pos)
#define SDMMC_MASK_CCRCFAILIE SDMMC_MASK_CCRCFAILIE_Msk

/*FIFOCNT register.*/
#define SDMMC_FIFOCNT_FIFOCOUNT_Pos 0U
#define SDMMC_FIFOCNT_FIFOCOUNT_Msk (0xFFFFFFUL << SDMMC_FIFOCNT_FIFOCOUNT_Pos)
#define SDMMC_FIFOCNT_FIFOCOUNT SDMMC_FIFOCNT_FIFOCOUNT_Msk
#endif

#if !defined(STM32L412) && !defined(STM32L422)
/* ========================================================================= */
/* ============                       CAN                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t MCR; /*!< CAN Master Control Register. */
	__IOM uint32_t MSR; /*!< CAN Master Status Register. */
	__IOM uint32_t TSR; /*!< CAN Transmit Status Register. */
	__IOM uint32_t RF0R; /*!< CAN Receive FIFO 0 Register. */
	__IOM uint32_t RF1R; /*!< CAN Receive FIFO 1 Register. */ 
	__IOM uint32_t IER; /*!< CAN Interrupt Enable Register. */
	__IOM uint32_t ESR; /*!< CAN Error Status Register. */
	__IOM uint32_t BTR; /*!< CAN Bit Timing Register. */
}STM32L4xx_CAN_control_and_status_TypeDef;

typedef struct
{
	__IOM uint32_t TI0R; /*!< CAN Tx mailbox Identifier Register 0. */
	__IOM uint32_t TDT0R; /*!< CAN Tx mailbox Data length and Timestamp Register 0. */
	__IOM uint32_t TDL0R; /*!< CAN Tx mailbox Data Low Register 0. */
	__IOM uint32_t TDH0R; /*!< CAN Tx mailbox Data High Register 0. */
	__IOM uint32_t TI1R; /*!< CAN Tx mailbox Identifier Register 1. */
	__IOM uint32_t TDT1R; /*!< CAN Tx mailbox Data length and Timestamp Register 1. */
	__IOM uint32_t TDL1R; /*!< CAN Tx mailbox Data Low Register 1. */
	__IOM uint32_t TDH1R; /*!< CAN Tx mailbox Data High Register 1. */
	__IOM uint32_t TI2R; /*!< CAN Tx mailbox Identifier Register 2. */
	__IOM uint32_t TDT2R; /*!< CAN Tx mailbox Data length and Timestamp Register 2. */
	__IOM uint32_t TDL2R; /*!< CAN Tx mailbox Data Low Register 2. */
	__IOM uint32_t TDH2R; /*!< CAN Tx mailbox Data High Register 2. */
	__IM uint32_t RI0R; /*!< CAN Rx mailbox Identifier Register 0. */
	__IM uint32_t RDT0R; /*!< CAN Rx mailbox Data length and Timestamp Register 0. */
	__IM uint32_t RDL0R; /*!< CAN Rx mailbox Data Low Register 0. */
	__IM uint32_t RDH0R; /*!< CAN Rx mailbox Data High Register 0. */
	__IM uint32_t RI1R; /*!< CAN Rx mailbox Identifier Register 1. */
	__IM uint32_t RDT1R; /*!< CAN Rx mailbox Data length and Timestamp Register 1. */
	__IM uint32_t RDL1R; /*!< CAN Rx mailbox Data Low Register 1. */
	__IM uint32_t RDH1R; /*!< CAN Rx mailbox Data High Register 1. */
}STM32L4xx_CAN_mailbox_TypeDef;

typedef struct
{
	__IOM uint32_t FMR; /*!< CAN Filter Master Register. */
	__IOM uint32_t FM1R; /*!< CAN Filter Mode Register. */
	uint32_t RESERVED0;
	__IOM uint32_t FS1R; /*!< CAN Filter Scale Register. */
	uint32_t RESERVED1;
	__IOM uint32_t FFA1R; /*!< CAN Filter FIFO Assignment Register. */
	uint32_t RESERVED2;
	__IOM uint32_t FA1R; /*!< CAN Filter Activation Register. */
	uint32_t RESERVED3[8];
	__IOM uint32_t F0R1; /*!< CAN Filter Bank 0 Register 1. */
	__IOM uint32_t F0R2; /*!< CAN Filter Bank 0 Register 2. */
	__IOM uint32_t F1R1; /*!< CAN Filter Bank 1 Register 1. */
	__IOM uint32_t F1R2; /*!< CAN Filter Bank 1 Register 2. */
	__IOM uint32_t F2R1; /*!< CAN Filter Bank 2 Register 1. */
	__IOM uint32_t F2R2; /*!< CAN Filter Bank 2 Register 2. */
	__IOM uint32_t F3R1; /*!< CAN Filter Bank 3 Register 1. */
	__IOM uint32_t F3R2; /*!< CAN Filter Bank 3 Register 2. */
	__IOM uint32_t F4R1; /*!< CAN Filter Bank 4 Register 1. */
	__IOM uint32_t F4R2; /*!< CAN Filter Bank 4 Register 2. */
	__IOM uint32_t F5R1; /*!< CAN Filter Bank 5 Register 1. */
	__IOM uint32_t F5R2; /*!< CAN Filter Bank 5 Register 2. */
	__IOM uint32_t F6R1; /*!< CAN Filter Bank 6 Register 1. */
	__IOM uint32_t F6R2; /*!< CAN Filter Bank 6 Register 2. */
	__IOM uint32_t F7R1; /*!< CAN Filter Bank 7 Register 1. */
	__IOM uint32_t F7R2; /*!< CAN Filter Bank 7 Register 2. */
	__IOM uint32_t F8R1; /*!< CAN Filter Bank 8 Register 1. */
	__IOM uint32_t F8R2; /*!< CAN Filter Bank 8 Register 2. */
	__IOM uint32_t F9R1; /*!< CAN Filter Bank 9 Register 1. */
	__IOM uint32_t F9R2; /*!< CAN Filter Bank 9 Register 2. */
	__IOM uint32_t F10R1; /*!< CAN Filter Bank 10 Register 1. */
	__IOM uint32_t F10R2; /*!< CAN Filter Bank 10 Register 2. */
	__IOM uint32_t F11R1; /*!< CAN Filter Bank 11 Register 1. */
	__IOM uint32_t F11R2; /*!< CAN Filter Bank 11 Register 2. */
	__IOM uint32_t F12R1; /*!< CAN Filter Bank 12 Register 1. */
	__IOM uint32_t F12R2; /*!< CAN Filter Bank 12 Register 2. */
	__IOM uint32_t F13R1; /*!< CAN Filter Bank 13 Register 1. */
	__IOM uint32_t F13R2; /*!< CAN Filter Bank 13 Register 2. */
	__IOM uint32_t F14R1; /*!< CAN Filter Bank 14 Register 1. */
	__IOM uint32_t F14R2; /*!< CAN Filter Bank 14 Register 2. */
	__IOM uint32_t F15R1; /*!< CAN Filter Bank 15 Register 1. */
	__IOM uint32_t F15R2; /*!< CAN Filter Bank 15 Register 2. */
	__IOM uint32_t F16R1; /*!< CAN Filter Bank 16 Register 1. */
	__IOM uint32_t F16R2; /*!< CAN Filter Bank 16 Register 2. */
	__IOM uint32_t F17R1; /*!< CAN Filter Bank 17 Register 1. */
	__IOM uint32_t F17R2; /*!< CAN Filter Bank 17 Register 2. */
	__IOM uint32_t F18R1; /*!< CAN Filter Bank 18 Register 1. */
	__IOM uint32_t F18R2; /*!< CAN Filter Bank 18 Register 2. */
	__IOM uint32_t F19R1; /*!< CAN Filter Bank 19 Register 1. */
	__IOM uint32_t F19R2; /*!< CAN Filter Bank 19 Register 2. */
	__IOM uint32_t F20R1; /*!< CAN Filter Bank 20 Register 1. */
	__IOM uint32_t F20R2; /*!< CAN Filter Bank 20 Register 2. */
	__IOM uint32_t F21R1; /*!< CAN Filter Bank 21 Register 1. */
	__IOM uint32_t F21R2; /*!< CAN Filter Bank 21 Register 2. */
	__IOM uint32_t F22R1; /*!< CAN Filter Bank 22 Register 1. */
	__IOM uint32_t F22R2; /*!< CAN Filter Bank 22 Register 2. */
	__IOM uint32_t F23R1; /*!< CAN Filter Bank 23 Register 1. */
	__IOM uint32_t F23R2; /*!< CAN Filter Bank 23 Register 2. */
	__IOM uint32_t F24R1; /*!< CAN Filter Bank 24 Register 1. */
	__IOM uint32_t F24R2; /*!< CAN Filter Bank 24 Register 2. */
	__IOM uint32_t F25R1; /*!< CAN Filter Bank 25 Register 1. */
	__IOM uint32_t F25R2; /*!< CAN Filter Bank 25 Register 2. */
	__IOM uint32_t F26R1; /*!< CAN Filter Bank 26 Register 1. */
	__IOM uint32_t F26R2; /*!< CAN Filter Bank 26 Register 2. */
	__IOM uint32_t F27R1; /*!< CAN Filter Bank 27 Register 1. */
	__IOM uint32_t F27R2; /*!< CAN Filter Bank 27 Register 2. */
}STM32L4xx_CAN_filter_TypeDef;

/*MCR register.*/
#define CAN_MCR_DBF_Pos 16U
#define CAN_MCR_DBF_Msk (0x1UL << CAN_MCR_DBF_Pos)
#define CAN_MCR_DBF CAN_MCR_DBF_Msk

#define CAN_MCR_RESET_Pos 15U
#define CAN_MCR_RESET_Msk (0x1UL << CAN_MCR_RESET_Pos)
#define CAN_MCR_RESET CAN_MCR_RESET_Msk

#define CAN_MCR_TTCM_Pos 7U
#define CAN_MCR_TTCM_Msk (0x1UL << CAN_MCR_TTCM_Pos)
#define CAN_MCR_TTCM CAN_MCR_TTCM_Msk

#define CAN_MCR_ABOM_Pos 6U
#define CAN_MCR_ABOM_Msk (0x1UL << CAN_MCR_ABOM_Pos)
#define CAN_MCR_ABOM CAN_MCR_ABOM_Msk

#define CAN_MCR_AWUM_Pos 5U
#define CAN_MCR_AWUM_Msk (0x1UL << CAN_MCR_AWUM_Pos)
#define CAN_MCR_AWUM CAN_MCR_AWUM_Msk

#define CAN_MCR_NART_Pos 4U
#define CAN_MCR_NART_Msk (0x1UL << CAN_MCR_NART_Pos)
#define CAN_MCR_NART CAN_MCR_NART_Msk

#define CAN_MCR_RFLM_Pos 3U
#define CAN_MCR_RFLM_Msk (0x1UL << CAN_MCR_RFLM_Pos)
#define CAN_MCR_RFLM CAN_MCR_RFLM_Msk

#define CAN_MCR_TXFP_Pos 2U
#define CAN_MCR_TXFP_Msk (0x1UL << CAN_MCR_TXFP_Pos)
#define CAN_MCR_TXFP CAN_MCR_TXFP_Msk

#define CAN_MCR_SLEEP_Pos 1U
#define CAN_MCR_SLEEP_Msk (0x1UL << CAN_MCR_SLEEP_Pos)
#define CAN_MCR_SLEEP CAN_MCR_SLEEP_Msk

#define CAN_MCR_INRQ_Pos 0U
#define CAN_MCR_INRQ_Msk (0x1UL << CAN_MCR_INRQ_Pos)
#define CAN_MCR_INRQ CAN_MCR_INRQ_Msk

/*MSR register.*/
#define CAN_MSR_RX_Pos 11U
#define CAN_MSR_RX_Msk (0x1UL << CAN_MSR_RX_Pos)
#define CAN_MSR_RX CAN_MSR_RX_Msk

#define CAN_MSR_SAMP_Pos 10U
#define CAN_MSR_SAMP_Msk (0x1UL << CAN_MSR_SAMP_Pos)
#define CAN_MSR_SAMP CAN_MSR_SAMP_Msk

#define CAN_MSR_RXM_Pos 9U
#define CAN_MSR_RXM_Msk (0x1UL << CAN_MSR_RXM_Pos)
#define CAN_MSR_RXM CAN_MSR_RXM_Msk

#define CAN_MSR_TXM_Pos 8U
#define CAN_MSR_TXM_Msk (0x1UL << CAN_MSR_TXM_Pos)
#define CAN_MSR_TXM CAN_MSR_TXM_Msk

#define CAN_MSR_SLAKI_Pos 4U
#define CAN_MSR_SLAKI_Msk (0x1UL << CAN_MSR_SLAKI_Pos)
#define CAN_MSR_SLAKI CAN_MSR_SLAKI_Msk

#define CAN_MSR_WKUI_Pos 3U
#define CAN_MSR_WKUI_Msk (0x1UL << CAN_MSR_WKUI_Pos)
#define CAN_MSR_WKUI CAN_MSR_WKUI_Msk

#define CAN_MSR_ERRI_Pos 2U
#define CAN_MSR_ERRI_Msk (0x1UL << CAN_MSR_ERRI_Pos)
#define CAN_MSR_ERRI CAN_MSR_ERRI_Msk

#define CAN_MSR_SLAK_Pos 1U
#define CAN_MSR_SLAK_Msk (0x1UL << CAN_MSR_SLAK_Pos)
#define CAN_MSR_SLAK CAN_MSR_SLAK_Msk

#define CAN_MSR_INAK_Pos 0U
#define CAN_MSR_INAK_Msk (0x1UL << CAN_MSR_INAK_Pos)
#define CAN_MSR_INAK CAN_MSR_INAK_Msk

/*TSR register.*/
#define CAN_TSR_LOW2_Pos 31U
#define CAN_TSR_LOW2_Msk (0x1UL << CAN_TSR_LOW2_Pos)
#define CAN_TSR_LOW2 CAN_TSR_LOW2_Msk

#define CAN_TSR_LOW1_Pos 30U
#define CAN_TSR_LOW1_Msk (0x1UL << CAN_TSR_LOW1_Pos)
#define CAN_TSR_LOW1 CAN_TSR_LOW1_Msk

#define CAN_TSR_LOW0_Pos 29U
#define CAN_TSR_LOW0_Msk (0x1UL << CAN_TSR_LOW0_Pos)
#define CAN_TSR_LOW0 CAN_TSR_LOW0_Msk

#define CAN_TSR_TME2_Pos 28U
#define CAN_TSR_TME2_Msk (0x1UL << CAN_TSR_TME2_Pos)
#define CAN_TSR_TME2 CAN_TSR_TME2_Msk

#define CAN_TSR_TME1_Pos 27U
#define CAN_TSR_TME1_Msk (0x1UL << CAN_TSR_TME1_Pos)
#define CAN_TSR_TME1 CAN_TSR_TME1_Msk

#define CAN_TSR_TME0_Pos 26U
#define CAN_TSR_TME0_Msk (0x1UL << CAN_TSR_TME0_Pos)
#define CAN_TSR_TME0 CAN_TSR_TME0_Msk

#define CAN_TSR_CODE_Pos 24U
#define CAN_TSR_CODE_Msk (0x3UL << CAN_TSR_CODE_Pos)
#define CAN_TSR_CODE CAN_TSR_CODE_Msk

#define CAN_TSR_ABRQ2_Pos 23U
#define CAN_TSR_ABRQ2_Msk (0x1UL << CAN_TSR_ABRQ2_Pos)
#define CAN_TSR_ABRQ2 CAN_TSR_ABRQ2_Msk

#define CAN_TSR_TERR2_Pos 19U
#define CAN_TSR_TERR2_Msk (0x1UL << CAN_TSR_TERR2_Pos)
#define CAN_TSR_TERR2 CAN_TSR_TERR2_Msk

#define CAN_TSR_ALST2_Pos 18U
#define CAN_TSR_ALST2_Msk (0x1UL << CAN_TSR_ALST2_Pos)
#define CAN_TSR_ALST2 CAN_TSR_ALST2_Msk

#define CAN_TSR_TXOK2_Pos 17U
#define CAN_TSR_TXOK2_Msk (0x1UL << CAN_TSR_TXOK2_Pos)
#define CAN_TSR_TXOK2 CAN_TSR_TXOK2_Msk

#define CAN_TSR_RQCP2_Pos 16U
#define CAN_TSR_RQCP2_Msk (0x1UL << CAN_TSR_RQCP2_Pos)
#define CAN_TSR_RQCP2 CAN_TSR_RQCP2_Msk

#define CAN_TSR_ABRQ1_Pos 15U
#define CAN_TSR_ABRQ1_Msk (0x1UL << CAN_TSR_ABRQ1_Pos)
#define CAN_TSR_ABRQ1 CAN_TSR_ABRQ1_Msk

#define CAN_TSR_TERR1_Pos 11U
#define CAN_TSR_TERR1_Msk (0x1UL << CAN_TSR_TERR1_Pos)
#define CAN_TSR_TERR1 CAN_TSR_TERR1_Msk

#define CAN_TSR_ALST1_Pos 10U
#define CAN_TSR_ALST1_Msk (0x1UL << CAN_TSR_ALST1_Pos)
#define CAN_TSR_ALST1 CAN_TSR_ALST1_Msk

#define CAN_TSR_TXOK1_Pos 9U
#define CAN_TSR_TXOK1_Msk (0x1UL << CAN_TSR_TXOK1_Pos)
#define CAN_TSR_TXOK1 CAN_TSR_TXOK1_Msk

#define CAN_TSR_RQCP1_Pos 8U
#define CAN_TSR_RQCP1_Msk (0x1UL << CAN_TSR_RQCP1_Pos)
#define CAN_TSR_RQCP1 CAN_TSR_RQCP1_Msk

#define CAN_TSR_TERR0_Pos 3U
#define CAN_TSR_TERR0_Msk (0x1UL << CAN_TSR_TERR0_Pos)
#define CAN_TSR_TERR0 CAN_TSR_TERR0_Msk

#define CAN_TSR_ALST0_Pos 2U
#define CAN_TSR_ALST0_Msk (0x1UL << CAN_TSR_ALST0_Pos)
#define CAN_TSR_ALST0 CAN_TSR_ALST0_Msk

#define CAN_TSR_TXOK0_Pos 1U
#define CAN_TSR_TXOK0_Msk (0x1UL << CAN_TSR_TXOK0_Pos)
#define CAN_TSR_TXOK0 CAN_TSR_TXOK0_Msk

#define CAN_TSR_RQCP0_Pos 0U
#define CAN_TSR_RQCP0_Msk (0x1UL << CAN_TSR_RQCP0_Pos)
#define CAN_TSR_RQCP0 CAN_TSR_RQCP0_Msk

/*RF0R register.*/
#define CAN_RF0R_RFOM0_Pos 5U
#define CAN_RF0R_RFOM0_Msk (0x1UL << CAN_RF0R_RFOM0_Pos)
#define CAN_RF0R_RFOM0 CAN_RF0R_RFOM0_Msk

#define CAN_RF0R_FOVR0_Pos 4U
#define CAN_RF0R_FOVR0_Msk (0x1UL << CAN_RF0R_FOVR0_Pos)
#define CAN_RF0R_FOVR0 CAN_RF0R_FOVR0_Msk

#define CAN_RF0R_FULL0_Pos 3U
#define CAN_RF0R_FULL0_Msk (0x1UL << CAN_RF0R_FULL0_Pos)
#define CAN_RF0R_FULL0 CAN_RF0R_FULL0_Msk

#define CAN_RF0R_FMP0_Pos 0U
#define CAN_RF0R_FMP0_Msk (0x3UL << CAN_RF0R_FMP0_Pos)
#define CAN_RF0R_FMP0 CAN_RF0R_FMP0_Msk

/*RF1R register.*/
#define CAN_RF1R_RFOM1_Pos 5U
#define CAN_RF1R_RFOM1_Msk (0x1UL << CAN_RF1R_RFOM1_Pos)
#define CAN_RF1R_RFOM1 CAN_RF1R_RFOM1_Msk

#define CAN_RF1R_FOVR1_Pos 4U
#define CAN_RF1R_FOVR1_Msk (0x1UL << CAN_RF1R_FOVR1_Pos)
#define CAN_RF1R_FOVR1 CAN_RF1R_FOVR1_Msk

#define CAN_RF1R_FULL1_Pos 3U
#define CAN_RF1R_FULL1_Msk (0x1UL << CAN_RF1R_FULL1_Pos)
#define CAN_RF1R_FULL1 CAN_RF1R_FULL1_Msk

#define CAN_RF1R_FMP1_Pos 0U
#define CAN_RF1R_FMP1_Msk (0x3UL << CAN_RF1R_FMP1_Pos)
#define CAN_RF1R_FMP1 CAN_RF1R_FMP1_Msk

/*IER register.*/
#define CAN_IER_SLKIE_Pos 17U
#define CAN_IER_SLKIE_Msk (0x1UL << CAN_IER_SLKIE_Pos)
#define CAN_IER_SLKIE CAN_IER_SLKIE_Msk

#define CAN_IER_WKUIE_Pos 16U
#define CAN_IER_WKUIE_Msk (0x1UL << CAN_IER_WKUIE_Pos)
#define CAN_IER_WKUIE CAN_IER_WKUIE_Msk

#define CAN_IER_ERRIE_Pos 15U
#define CAN_IER_ERRIE_Msk (0x1UL << CAN_IER_ERRIE_Pos)
#define CAN_IER_ERRIE CAN_IER_ERRIE_Msk

#define CAN_IER_LECIE_Pos 11U
#define CAN_IER_LECIE_Msk (0x1UL << CAN_IER_LECIE_Pos)
#define CAN_IER_LECIE CAN_IER_LECIE_Msk

#define CAN_IER_BOFIE_Pos 10U
#define CAN_IER_BOFIE_Msk (0x1UL << CAN_IER_BOFIE_Pos)
#define CAN_IER_BOFIE CAN_IER_BOFIE_Msk

#define CAN_IER_EPVIE_Pos 9U
#define CAN_IER_EPVIE_Msk (0x1UL << CAN_IER_EPVIE_Pos)
#define CAN_IER_EPVIE CAN_IER_EPVIE_Msk

#define CAN_IER_EWGIE_Pos 8U
#define CAN_IER_EWGIE_Msk (0x1UL << CAN_IER_EWGIE_Pos)
#define CAN_IER_EWGIE CAN_IER_EWGIE_Msk

#define CAN_IER_FOVIE1_Pos 6U
#define CAN_IER_FOVIE1_Msk (0x1UL << CAN_IER_FOVIE1_Pos)
#define CAN_IER_FOVIE1 CAN_IER_FOVIE1_Msk

#define CAN_IER_FFIE1_Pos 5U
#define CAN_IER_FFIE1_Msk (0x1UL << CAN_IER_FFIE1_Pos)
#define CAN_IER_FFIE1 CAN_IER_FFIE1_Msk

#define CAN_IER_FMPIE1_Pos 4U
#define CAN_IER_FMPIE1_Msk (0x1UL << CAN_IER_FMPIE1_Pos)
#define CAN_IER_FMPIE1 CAN_IER_FMPIE1_Msk

#define CAN_IER_FOVIE0_Pos 3U
#define CAN_IER_FOVIE0_Msk (0x1UL << CAN_IER_FOVIE0_Pos)
#define CAN_IER_FOVIE0 CAN_IER_FOVIE0_Msk

#define CAN_IER_FFIE0_Pos 2U
#define CAN_IER_FFIE0_Msk (0x1UL << CAN_IER_FFIE0_Pos)
#define CAN_IER_FFIE0 CAN_IER_FFIE0_Msk

#define CAN_IER_FMPIE0_Pos 1U
#define CAN_IER_FMPIE0_Msk (0x1UL << CAN_IER_FMPIE0_Pos)
#define CAN_IER_FMPIE0 CAN_IER_FMPIE0_Msk

#define CAN_IER_TMEIE_Pos 0U
#define CAN_IER_TMEIE_Msk (0x1UL << CAN_IER_TMEIE_Pos)
#define CAN_IER_TMEIE CAN_IER_TMEIE_Msk

/*ESR register.*/
#define CAN_ESR_REC_Pos 24U
#define CAN_ESR_REC_Msk (0xFFUL << CAN_ESR_REC_Pos)
#define CAN_ESR_REC CAN_ESR_REC_Msk

#define CAN_ESR_TEC_Pos 16U
#define CAN_ESR_TEC_Msk (0xFFUL << CAN_ESR_TEC_Pos)
#define CAN_ESR_TEC CAN_ESR_TEC_Msk

#define CAN_ESR_LEC_Pos 4U
#define CAN_ESR_LEC_Msk (0x7UL << CAN_ESR_LEC_Pos)
#define CAN_ESR_LEC CAN_ESR_LEC_Msk
#define CAN_ESR_LEC_NO_ERROR (0x0UL << CAN_ESR_LEC_Pos)
#define CAN_ESR_LEC_STUFF_ERROR (0x1UL << CAN_ESR_LEC_Pos)
#define CAN_ESR_LEC_FORM_ERROR (0x2UL << CAN_ESR_LEC_Pos)
#define CAN_ESR_LEC_ACK_ERROR (0x3UL << CAN_ESR_LEC_Pos)
#define CAN_ESR_LEC_BIT_RECESSIVE_ERROR (0x4UL << CAN_ESR_LEC_Pos)
#define CAN_ESR_LEC_BIT_DOMINANT_ERROR (0x5UL << CAN_ESR_LEC_Pos)
#define CAN_ESR_LEC_CRC_ERROR (0x6UL << CAN_ESR_LEC_Pos)
#define CAN_ESR_LEC_SET_BY_SOFTWARE (0x7UL << CAN_ESR_LEC_Pos)

#define CAN_ESR_BOFF_Pos 2U
#define CAN_ESR_BOFF_Msk (0x1UL << CAN_ESR_BOFF_Pos)
#define CAN_ESR_BOFF CAN_ESR_BOFF_Msk

#define CAN_ESR_EPVF_Pos 1U
#define CAN_ESR_EPVF_Msk (0x1UL << CAN_ESR_EPVF_Pos)
#define CAN_ESR_EPVF CAN_ESR_EPVF_Msk

#define CAN_ESR_EWGF_Pos 0U
#define CAN_ESR_EWGF_Msk (0x1UL << CAN_ESR_EWGF_Pos)
#define CAN_ESR_EWGF CAN_ESR_EWGF_Msk

/*BTR register.*/
#define CAN_BTR_SILM_Pos 31U
#define CAN_BTR_SILM_Msk (0x1UL << CAN_BTR_SILM_Pos)
#define CAN_BTR_SILM CAN_BTR_SILM_Msk

#define CAN_BTR_LBKM_Pos 30U
#define CAN_BTR_LBKM_Msk (0x1UL << CAN_BTR_LBKM_Pos)
#define CAN_BTR_LBKM CAN_BTR_LBKM_Msk

#define CAN_BTR_SJW_Pos 24U
#define CAN_BTR_SJW_Msk (0x3UL << CAN_BTR_SJW_Pos)
#define CAN_BTR_SJW CAN_BTR_SJW_Msk

#define CAN_BTR_TS2_Pos 20U
#define CAN_BTR_TS2_Msk (0x7UL << CAN_BTR_TS2_Pos)
#define CAN_BTR_TS2 CAN_BTR_TS2_Msk

#define CAN_BTR_TS1_Pos 16U
#define CAN_BTR_TS1_Msk (0xFUL << CAN_BTR_TS1_Pos)
#define CAN_BTR_TS1 CAN_BTR_TS1_Msk

#define CAN_BTR_BRP_Pos 0U
#define CAN_BTR_BRP_Msk (0x3FFUL << CAN_BTR_BRP_Pos)
#define CAN_BTR_BRP CAN_BTR_BRP_Msk

/*TIR register.*/
#define CAN_TIR_STID_Pos 21U
#define CAN_TIR_STID_Msk (0x7FFUL << CAN_TIR_STID_Pos)
#define CAN_TIR_STID CAN_TIR_STID_Msk

#define CAN_TIR_EXID_MSB_Pos 21U
#define CAN_TIR_EXID_LSB_Pos 3U
#define CAN_TIR_EXID_Msk ((0x7FFUL << CAN_TIR_EXID_MSB_Pos) | (0x3FFFFUL << CAN_TIR_EXID_LSB_Pos))
#define CAN_TIR_EXID CAN_TIR_EXID_Msk

#define CAN_TIR_IDE_Pos 2U
#define CAN_TIR_IDE_Msk (0x1UL << CAN_TIR_IDE_Pos)
#define CAN_TIR_IDE CAN_TIR_IDE_Msk

#define CAN_TIR_RTR_Pos 1U
#define CAN_TIR_RTR_Msk (0x1UL << CAN_TIR_RTR_Pos)
#define CAN_TIR_RTR CAN_TIR_RTR_Msk

#define CAN_TIR_TXRQ_Pos 0U
#define CAN_TIR_TXRQ_Msk (0x1UL << CAN_TIR_TXRQ_Pos)
#define CAN_TIR_TXRQ CAN_TIR_TXRQ_Msk

/*TDTR register.*/
#define CAN_TDTR_TIME_Pos 16U
#define CAN_TDTR_TIME_Msk (0xFFFFUL << CAN_TDTR_TIME_Pos)
#define CAN_TDTR_TIME CAN_TDTR_TIME_Msk

#define CAN_TDTR_DLC_Pos 0U
#define CAN_TDTR_DLC_Msk (0xFUL << CAN_TDTR_DLC_Pos)
#define CAN_TDTR_DLC CAN_TDTR_DLC_Msk

/*TDLR register.*/
#define CAN_TDLR_DATA3_Pos 24U
#define CAN_TDLR_DATA3_Msk (0xFFUL << CAN_TDLR_DATA3_Pos)
#define CAN_TDLR_DATA3 CAN_TDLR_DATA3_Msk

#define CAN_TDLR_DATA2_Pos 16U
#define CAN_TDLR_DATA2_Msk (0xFFUL << CAN_TDLR_DATA2_Pos)
#define CAN_TDLR_DATA2 CAN_TDLR_DATA2_Msk

#define CAN_TDLR_DATA1_Pos 8U
#define CAN_TDLR_DATA1_Msk (0xFFUL << CAN_TDLR_DATA1_Pos)
#define CAN_TDLR_DATA1 CAN_TDLR_DATA1_Msk

#define CAN_TDLR_DATA0_Pos 0U
#define CAN_TDLR_DATA0_Msk (0xFFUL << CAN_TDLR_DATA0_Pos)
#define CAN_TDLR_DATA0 CAN_TDLR_DATA0_Msk

/*TDHR register.*/
#define CAN_TDLR_DATA7_Pos 24U
#define CAN_TDLR_DATA7_Msk (0xFFUL << CAN_TDLR_DATA7_Pos)
#define CAN_TDLR_DATA7 CAN_TDLR_DATA7_Msk

#define CAN_TDLR_DATA6_Pos 16U
#define CAN_TDLR_DATA6_Msk (0xFFUL << CAN_TDLR_DATA6_Pos)
#define CAN_TDLR_DATA6 CAN_TDLR_DATA6_Msk

#define CAN_TDLR_DATA5_Pos 8U
#define CAN_TDLR_DATA5_Msk (0xFFUL << CAN_TDLR_DATA5_Pos)
#define CAN_TDLR_DATA5 CAN_TDLR_DATA5_Msk

#define CAN_TDLR_DATA4_Pos 0U
#define CAN_TDLR_DATA4_Msk (0xFFUL << CAN_TDLR_DATA4_Pos)
#define CAN_TDLR_DATA4 CAN_TDLR_DATA4_Msk

/*RIR register.*/
#define CAN_RIR_STID_Pos 21U
#define CAN_RIR_STID_Msk (0x7FFUL << CAN_RIR_STID_Pos)
#define CAN_RIR_STID CAN_RIR_STID_Msk

#define CAN_RIR_EXID_MSB_Pos 21U
#define CAN_RIR_EXID_LSB_Pos 3U
#define CAN_RIR_EXID_Msk ((0x7FFUL << CAN_RIR_EXID_MSB_Pos) | (0x3FFFFUL << CAN_RIR_EXID_LSB_Pos))
#define CAN_RIR_EXID CAN_RIR_EXID_Msk

#define CAN_RIR_IDE_Pos 2U
#define CAN_RIR_IDE_Msk (0x1UL << CAN_RIR_IDE_Pos)
#define CAN_RIR_IDE CAN_RIR_IDE_Msk

#define CAN_RIR_RTR_Pos 1U
#define CAN_RIR_RTR_Msk (0x1UL << CAN_RIR_RTR_Pos)
#define CAN_RIR_RTR CAN_RIR_RTR_Msk

/*RDTR register.*/
#define CAN_RDTR_TIME_Pos 16U
#define CAN_RDTR_TIME_Msk (0xFFFFUL << CAN_RDTR_TIME_Pos)
#define CAN_RDTR_TIME CAN_RDTR_TIME_Msk

#define CAN_RDTR_FMI_Pos 8U
#define CAN_RDTR_FMI_Msk (0xFFUL << CAN_RDTR_FMI_Pos)
#define CAN_RDTR_FMI CAN_RDTR_FMI_Msk

#define CAN_RDTR_DLC_Pos 0U
#define CAN_RDTR_DLC_Msk (0xFUL << CAN_RDTR_DLC_Pos)
#define CAN_RDTR_DLC CAN_RDTR_DLC_Msk

/*RDLR register.*/
#define CAN_RDLR_DATA3_Pos 24U
#define CAN_RDLR_DATA3_Msk (0xFFUL << CAN_RDLR_DATA3_Pos)
#define CAN_RDLR_DATA3 CAN_RDLR_DATA3_Msk

#define CAN_RDLR_DATA2_Pos 16U
#define CAN_RDLR_DATA2_Msk (0xFFUL << CAN_RDLR_DATA2_Pos)
#define CAN_RDLR_DATA2 CAN_RDLR_DATA2_Msk

#define CAN_RDLR_DATA1_Pos 8U
#define CAN_RDLR_DATA1_Msk (0xFFUL << CAN_RDLR_DATA1_Pos)
#define CAN_RDLR_DATA1 CAN_RDLR_DATA1_Msk

#define CAN_RDLR_DATA0_Pos 0U
#define CAN_RDLR_DATA0_Msk (0xFFUL << CAN_RDLR_DATA0_Pos)
#define CAN_RDLR_DATA0 CAN_RDLR_DATA0_Msk

/*RDHR register.*/
#define CAN_RDHR_DATA7_Pos 24U
#define CAN_RDHR_DATA7_Msk (0xFFUL << CAN_RDHR_DATA7_Pos)
#define CAN_RDHR_DATA7 CAN_RDHR_DATA3_Msk

#define CAN_RDHR_DATA6_Pos 16U
#define CAN_RDHR_DATA6_Msk (0xFFUL << CAN_RDHR_DATA6_Pos)
#define CAN_RDHR_DATA6 CAN_RDHR_DATA6_Msk

#define CAN_RDHR_DATA5_Pos 8U
#define CAN_RDHR_DATA5_Msk (0xFFUL << CAN_RDLR_DATA5_Pos)
#define CAN_RDHR_DATA5 CAN_RDHR_DATA5_Msk

#define CAN_RDHR_DATA4_Pos 0U
#define CAN_RDHR_DATA4_Msk (0xFFUL << CAN_RDHR_DATA4_Pos)
#define CAN_RDHR_DATA4 CAN_RDHR_DATA4_Msk

/*FMR register.*/
#define CAN_FMR_FINIT_Pos 0U
#define CAN_FMR_FINIT_Msk (0x1UL << CAN_FMR_FINIT_Pos)
#define CAN_FMR_FINIT CAN_FMR_FINIT_Msk

/*FM1R register.*/
#define CAN_FM1R_FBM13_Pos 13U
#define CAN_FM1R_FBM13_Msk (0x1U << CAN_FM1R_FBM13_Pos)
#define CAN_FM1R_FBM13 CAN_FM1R_FBM13_Msk

#define CAN_FM1R_FBM12_Pos 12U
#define CAN_FM1R_FBM12_Msk (0x1U << CAN_FM1R_FBM12_Pos)
#define CAN_FM1R_FBM12 CAN_FM1R_FBM12_Msk

#define CAN_FM1R_FBM11_Pos 11U
#define CAN_FM1R_FBM11_Msk (0x1U << CAN_FM1R_FBM11_Pos)
#define CAN_FM1R_FBM11 CAN_FM1R_FBM11_Msk

#define CAN_FM1R_FBM10_Pos 10U
#define CAN_FM1R_FBM10_Msk (0x1U << CAN_FM1R_FBM10_Pos)
#define CAN_FM1R_FBM10 CAN_FM1R_FBM10_Msk

#define CAN_FM1R_FBM9_Pos 9U
#define CAN_FM1R_FBM9_Msk (0x1U << CAN_FM1R_FBM9_Pos)
#define CAN_FM1R_FBM9 CAN_FM1R_FBM9_Msk

#define CAN_FM1R_FBM8_Pos 8U
#define CAN_FM1R_FBM8_Msk (0x1U << CAN_FM1R_FBM8_Pos)
#define CAN_FM1R_FBM8 CAN_FM1R_FBM8_Msk

#define CAN_FM1R_FBM7_Pos 7U
#define CAN_FM1R_FBM7_Msk (0x1U << CAN_FM1R_FBM7_Pos)
#define CAN_FM1R_FBM7 CAN_FM1R_FBM7_Msk

#define CAN_FM1R_FBM6_Pos 6U
#define CAN_FM1R_FBM6_Msk (0x1U << CAN_FM1R_FBM6_Pos)
#define CAN_FM1R_FBM6 CAN_FM1R_FBM6_Msk

#define CAN_FM1R_FBM5_Pos 5U
#define CAN_FM1R_FBM5_Msk (0x1U << CAN_FM1R_FBM5_Pos)
#define CAN_FM1R_FBM5 CAN_FM1R_FBM5_Msk

#define CAN_FM1R_FBM4_Pos 4U
#define CAN_FM1R_FBM4_Msk (0x1U << CAN_FM1R_FBM4_Pos)
#define CAN_FM1R_FBM4 CAN_FM1R_FBM4_Msk

#define CAN_FM1R_FBM3_Pos 3U
#define CAN_FM1R_FBM3_Msk (0x1U << CAN_FM1R_FBM3_Pos)
#define CAN_FM1R_FBM3 CAN_FM1R_FBM3_Msk

#define CAN_FM1R_FBM2_Pos 2U
#define CAN_FM1R_FBM2_Msk (0x1U << CAN_FM1R_FBM2_Pos)
#define CAN_FM1R_FBM2 CAN_FM1R_FBM2_Msk

#define CAN_FM1R_FBM1_Pos 1U
#define CAN_FM1R_FBM1_Msk (0x1U << CAN_FM1R_FBM1_Pos)
#define CAN_FM1R_FBM1 CAN_FM1R_FBM1_Msk

#define CAN_FM1R_FBM0_Pos 0U
#define CAN_FM1R_FBM0_Msk (0x1U << CAN_FM1R_FBM0_Pos)
#define CAN_FM1R_FBM0 CAN_FM1R_FBM0_Msk

/*FS1R register.*/
#define CAN_FS1R_FSC13_Pos 13U
#define CAN_FS1R_FSC13_Msk (0x1U << CAN_FS1R_FSC13_Pos)
#define CAN_FS1R_FSC13 CAN_FS1R_FSC13_Msk

#define CAN_FS1R_FSC12_Pos 12U
#define CAN_FS1R_FSC12_Msk (0x1U << CAN_FS1R_FSC12_Pos)
#define CAN_FS1R_FSC12 CAN_FS1R_FSC12_Msk

#define CAN_FS1R_FSC11_Pos 11U
#define CAN_FS1R_FSC11_Msk (0x1U << CAN_FS1R_FSC11_Pos)
#define CAN_FS1R_FSC11 CAN_FS1R_FSC11_Msk

#define CAN_FS1R_FSC10_Pos 10U
#define CAN_FS1R_FSC10_Msk (0x1U << CAN_FS1R_FSC10_Pos)
#define CAN_FS1R_FSC10 CAN_FS1R_FSC10_Msk

#define CAN_FS1R_FSC9_Pos 9U
#define CAN_FS1R_FSC9_Msk (0x1U << CAN_FS1R_FSC9_Pos)
#define CAN_FS1R_FSC9 CAN_FS1R_FSC9_Msk

#define CAN_FS1R_FSC8_Pos 8U
#define CAN_FS1R_FSC8_Msk (0x1U << CAN_FS1R_FSC8_Pos)
#define CAN_FS1R_FSC8 CAN_FS1R_FSC8_Msk

#define CAN_FS1R_FSC7_Pos 7U
#define CAN_FS1R_FSC7_Msk (0x1U << CAN_FS1R_FSC7_Pos)
#define CAN_FS1R_FSC7 CAN_FS1R_FSC7_Msk

#define CAN_FS1R_FSC6_Pos 6U
#define CAN_FS1R_FSC6_Msk (0x1U << CAN_FS1R_FSC6_Pos)
#define CAN_FS1R_FSC6 CAN_FS1R_FSC6_Msk

#define CAN_FS1R_FSC5_Pos 5U
#define CAN_FS1R_FSC5_Msk (0x1U << CAN_FS1R_FSC5_Pos)
#define CAN_FS1R_FSC5 CAN_FS1R_FSC5_Msk

#define CAN_FS1R_FSC4_Pos 4U
#define CAN_FS1R_FSC4_Msk (0x1U << CAN_FS1R_FSC4_Pos)
#define CAN_FS1R_FSC4 CAN_FS1R_FSC4_Msk

#define CAN_FS1R_FSC3_Pos 3U
#define CAN_FS1R_FSC3_Msk (0x1U << CAN_FS1R_FSC3_Pos)
#define CAN_FS1R_FSC3 CAN_FS1R_FSC3_Msk

#define CAN_FS1R_FSC2_Pos 2U
#define CAN_FS1R_FSC2_Msk (0x1U << CAN_FS1R_FSC2_Pos)
#define CAN_FS1R_FSC2 CAN_FS1R_FSC2_Msk

#define CAN_FS1R_FSC1_Pos 1U
#define CAN_FS1R_FSC1_Msk (0x1U << CAN_FS1R_FSC1_Pos)
#define CAN_FS1R_FSC1 CAN_FS1R_FSC1_Msk

#define CAN_FS1R_FSC0_Pos 0U
#define CAN_FS1R_FSC0_Msk (0x1U << CAN_FS1R_FSC0_Pos)
#define CAN_FS1R_FSC0 CAN_FS1R_FSC0_Msk

/*FFA1R register.*/
#define CAN_FFA1R_FFA13_Pos 13U
#define CAN_FFA1R_FFA13_Msk (0x1U << CAN_FFA1R_FFA13_Pos)
#define CAN_FFA1R_FFA13 CAN_FFA1R_FFA13_Msk

#define CAN_FFA1R_FFA12_Pos 12U
#define CAN_FFA1R_FFA12_Msk (0x1U << CAN_FFA1R_FFA12_Pos)
#define CAN_FFA1R_FFA12 CAN_FFA1R_FFA12_Msk

#define CAN_FFA1R_FFA11_Pos 11U
#define CAN_FFA1R_FFA11_Msk (0x1U << CAN_FFA1R_FFA11_Pos)
#define CAN_FFA1R_FFA11 CAN_FFA1R_FFA11_Msk

#define CAN_FFA1R_FFA10_Pos 10U
#define CAN_FFA1R_FFA10_Msk (0x1U << CAN_FFA1R_FFA10_Pos)
#define CAN_FFA1R_FFA10 CAN_FFA1R_FFA10_Msk

#define CAN_FFA1R_FFA9_Pos 9U
#define CAN_FFA1R_FFA9_Msk (0x1U << CAN_FFA1R_FFA9_Pos)
#define CAN_FFA1R_FFA9 CAN_FFA1R_FFA9_Msk

#define CAN_FFA1R_FFA8_Pos 8U
#define CAN_FFA1R_FFA8_Msk (0x1U << CAN_FFA1R_FFA8_Pos)
#define CAN_FFA1R_FFA8 CAN_FFA1R_FFA8_Msk

#define CAN_FFA1R_FFA7_Pos 7U
#define CAN_FFA1R_FFA7_Msk (0x1U << CAN_FFA1R_FFA7_Pos)
#define CAN_FFA1R_FFA7 CAN_FFA1R_FFA7_Msk

#define CAN_FFA1R_FFA6_Pos 6U
#define CAN_FFA1R_FFA6_Msk (0x1U << CAN_FFA1R_FFA6_Pos)
#define CAN_FFA1R_FFA6 CAN_FFA1R_FFA6_Msk

#define CAN_FFA1R_FFA5_Pos 5U
#define CAN_FFA1R_FFA5_Msk (0x1U << CAN_FFA1R_FFA5_Pos)
#define CAN_FFA1R_FFA5 CAN_FFA1R_FFA5_Msk

#define CAN_FFA1R_FFA4_Pos 4U
#define CAN_FFA1R_FFA4_Msk (0x1U << CAN_FFA1R_FFA4_Pos)
#define CAN_FFA1R_FFA4 CAN_FFA1R_FFA4_Msk

#define CAN_FFA1R_FFA3_Pos 3U
#define CAN_FFA1R_FFA3_Msk (0x1U << CAN_FFA1R_FFA3_Pos)
#define CAN_FFA1R_FFA3 CAN_FFA1R_FFA3_Msk

#define CAN_FFA1R_FFA2_Pos 2U
#define CAN_FFA1R_FFA2_Msk (0x1U << CAN_FFA1R_FFA2_Pos)
#define CAN_FFA1R_FFA2 CAN_FFA1R_FFA2_Msk

#define CAN_FFA1R_FFA1_Pos 1U
#define CAN_FFA1R_FFA1_Msk (0x1U << CAN_FFA1R_FFA1_Pos)
#define CAN_FFA1R_FFA1 CAN_FFA1R_FFA1_Msk

#define CAN_FFA1R_FFA0_Pos 0U
#define CAN_FFA1R_FFA0_Msk (0x1U << CAN_FFA1R_FFA0_Pos)
#define CAN_FFA1R_FFA0 CAN_FFA1R_FFA0_Msk

/*FA1R register.*/
#define CAN_FA1R_FACT13_Pos 13U
#define CAN_FA1R_FACT13_Msk (0x1U << CAN_FA1R_FACT13_Pos)
#define CAN_FA1R_FACT13 CAN_FA1R_FACT13_Msk

#define CAN_FA1R_FACT12_Pos 12U
#define CAN_FA1R_FACT12_Msk (0x1U << CAN_FA1R_FACT12_Pos)
#define CAN_FA1R_FACT12 CAN_FA1R_FACT12_Msk

#define CAN_FA1R_FACT11_Pos 11U
#define CAN_FA1R_FACT11_Msk (0x1U << CAN_FA1R_FACT11_Pos)
#define CAN_FA1R_FACT11 CAN_FA1R_FACT11_Msk

#define CAN_FA1R_FACT10_Pos 10U
#define CAN_FA1R_FACT10_Msk (0x1U << CAN_FA1R_FACT10_Pos)
#define CAN_FA1R_FACT10 CAN_FA1R_FACT10_Msk

#define CAN_FA1R_FACT9_Pos 9U
#define CAN_FA1R_FACT9_Msk (0x1U << CAN_FA1R_FACT9_Pos)
#define CAN_FA1R_FACT9 CAN_FA1R_FACT9_Msk

#define CAN_FA1R_FACT8_Pos 8U
#define CAN_FA1R_FACT8_Msk (0x1U << CAN_FA1R_FACT8_Pos)
#define CAN_FA1R_FACT8 CAN_FA1R_FACT8_Msk

#define CAN_FA1R_FACT7_Pos 7U
#define CAN_FA1R_FACT7_Msk (0x1U << CAN_FA1R_FACT7_Pos)
#define CAN_FA1R_FACT7 CAN_FA1R_FACT7_Msk

#define CAN_FA1R_FACT6_Pos 6U
#define CAN_FA1R_FACT6_Msk (0x1U << CAN_FA1R_FACT6_Pos)
#define CAN_FA1R_FACT6 CAN_FA1R_FACT6_Msk

#define CAN_FA1R_FACT5_Pos 5U
#define CAN_FA1R_FACT5_Msk (0x1U << CAN_FA1R_FACT5_Pos)
#define CAN_FA1R_FACT5 CAN_FA1R_FACT5_Msk

#define CAN_FA1R_FACT4_Pos 4U
#define CAN_FA1R_FACT4_Msk (0x1U << CAN_FA1R_FACT4_Pos)
#define CAN_FA1R_FACT4 CAN_FA1R_FACT4_Msk

#define CAN_FA1R_FACT3_Pos 3U
#define CAN_FA1R_FACT3_Msk (0x1U << CAN_FA1R_FACT3_Pos)
#define CAN_FA1R_FACT3 CAN_FA1R_FACT3_Msk

#define CAN_FA1R_FACT2_Pos 2U
#define CAN_FA1R_FACT2_Msk (0x1U << CAN_FA1R_FACT2_Pos)
#define CAN_FA1R_FACT2 CAN_FA1R_FACT2_Msk

#define CAN_FA1R_FACT1_Pos 1U
#define CAN_FA1R_FACT1_Msk (0x1U << CAN_FA1R_FACT1_Pos)
#define CAN_FA1R_FACT1 CAN_FA1R_FACT1_Msk

#define CAN_FA1R_FACT0_Pos 0U
#define CAN_FA1R_FACT0_Msk (0x1U << CAN_FA1R_FACT0_Pos)
#define CAN_FA1R_FACT0 CAN_FA1R_FACT0_Msk

/*FR register.*/
#define CAN_FR_FB31_Pos 31U
#define CAN_FR_FB31_Msk (0x1U << CAN_FR_FB31_Pos)
#define CAN_FR_FB31 CAN_FR_FB31_Msk

#define CAN_FR_FB30_Pos 30U
#define CAN_FR_FB30_Msk (0x1U << CAN_FR_FB30_Pos)
#define CAN_FR_FB30 CAN_FR_FB30_Msk

#define CAN_FR_FB29_Pos 29U
#define CAN_FR_FB29_Msk (0x1U << CAN_FR_FB29_Pos)
#define CAN_FR_FB29 CAN_FR_FB29_Msk

#define CAN_FR_FB28_Pos 28U
#define CAN_FR_FB28_Msk (0x1U << CAN_FR_FB28_Pos)
#define CAN_FR_FB28 CAN_FR_FB28_Msk

#define CAN_FR_FB27_Pos 27U
#define CAN_FR_FB27_Msk (0x1U << CAN_FR_FB27_Pos)
#define CAN_FR_FB27 CAN_FR_FB27_Msk

#define CAN_FR_FB26_Pos 26U
#define CAN_FR_FB26_Msk (0x1U << CAN_FR_FB26_Pos)
#define CAN_FR_FB26 CAN_FR_FB26_Msk

#define CAN_FR_FB25_Pos 25U
#define CAN_FR_FB25_Msk (0x1U << CAN_FR_FB25_Pos)
#define CAN_FR_FB25 CAN_FR_FB25_Msk

#define CAN_FR_FB24_Pos 24U
#define CAN_FR_FB24_Msk (0x1U << CAN_FR_FB24_Pos)
#define CAN_FR_FB24 CAN_FR_FB24_Msk

#define CAN_FR_FB23_Pos 23U
#define CAN_FR_FB23_Msk (0x1U << CAN_FR_FB23_Pos)
#define CAN_FR_FB23 CAN_FR_FB23_Msk

#define CAN_FR_FB22_Pos 22U
#define CAN_FR_FB22_Msk (0x1U << CAN_FR_FB22_Pos)
#define CAN_FR_FB22 CAN_FR_FB22_Msk

#define CAN_FR_FB21_Pos 21U
#define CAN_FR_FB21_Msk (0x1U << CAN_FR_FB21_Pos)
#define CAN_FR_FB21 CAN_FR_FB21_Msk

#define CAN_FR_FB20_Pos 20U
#define CAN_FR_FB20_Msk (0x1U << CAN_FR_FB20_Pos)
#define CAN_FR_FB20 CAN_FR_FB20_Msk

#define CAN_FR_FB19_Pos 19U
#define CAN_FR_FB19_Msk (0x1U << CAN_FR_FB19_Pos)
#define CAN_FR_FB19 CAN_FR_FB19_Msk

#define CAN_FR_FB18_Pos 18U
#define CAN_FR_FB18_Msk (0x1U << CAN_FR_FB18_Pos)
#define CAN_FR_FB18 CAN_FR_FB18_Msk

#define CAN_FR_FB17_Pos 17U
#define CAN_FR_FB17_Msk (0x1U << CAN_FR_FB17_Pos)
#define CAN_FR_FB17 CAN_FR_FB17_Msk

#define CAN_FR_FB16_Pos 16U
#define CAN_FR_FB16_Msk (0x1U << CAN_FR_FB16_Pos)
#define CAN_FR_FB16 CAN_FR_FB16_Msk

#define CAN_FR_FB15_Pos 15U
#define CAN_FR_FB15_Msk (0x1U << CAN_FR_FB15_Pos)
#define CAN_FR_FB15 CAN_FR_FB15_Msk

#define CAN_FR_FB14_Pos 14U
#define CAN_FR_FB14_Msk (0x1U << CAN_FR_FB14_Pos)
#define CAN_FR_FB14 CAN_FR_FB14_Msk

#define CAN_FR_FB13_Pos 13U
#define CAN_FR_FB13_Msk (0x1U << CAN_FR_FB13_Pos)
#define CAN_FR_FB13 CAN_FR_FB13_Msk

#define CAN_FR_FB12_Pos 12U
#define CAN_FR_FB12_Msk (0x1U << CAN_FR_FB12_Pos)
#define CAN_FR_FB12 CAN_FR_FB12_Msk

#define CAN_FR_FB11_Pos 11U
#define CAN_FR_FB11_Msk (0x1U << CAN_FR_FB11_Pos)
#define CAN_FR_FB11 CAN_FR_FB11_Msk

#define CAN_FR_FB10_Pos 10U
#define CAN_FR_FB10_Msk (0x1U << CAN_FR_FB10_Pos)
#define CAN_FR_FB10 CAN_FR_FB10_Msk

#define CAN_FR_FB9_Pos 9U
#define CAN_FR_FB9_Msk (0x1U << CAN_FR_FB9_Pos)
#define CAN_FR_FB9 CAN_FR_FB9_Msk

#define CAN_FR_FB8_Pos 8U
#define CAN_FR_FB8_Msk (0x1U << CAN_FR_FB8_Pos)
#define CAN_FR_FB8 CAN_FR_FB8_Msk

#define CAN_FR_FB7_Pos 7U
#define CAN_FR_FB7_Msk (0x1U << CAN_FR_FB7_Pos)
#define CAN_FR_FB7 CAN_FR_FB7_Msk

#define CAN_FR_FB6_Pos 6U
#define CAN_FR_FB6_Msk (0x1U << CAN_FR_FB6_Pos)
#define CAN_FR_FB6 CAN_FR_FB6_Msk

#define CAN_FR_FB5_Pos 5U
#define CAN_FR_FB5_Msk (0x1U << CAN_FR_FB5_Pos)
#define CAN_FR_FB5 CAN_FR_FB5_Msk

#define CAN_FR_FB4_Pos 4U
#define CAN_FR_FB4_Msk (0x1U << CAN_FR_FB4_Pos)
#define CAN_FR_FB4 CAN_FR_FB4_Msk

#define CAN_FR_FB3_Pos 3U
#define CAN_FR_FB3_Msk (0x1U << CAN_FR_FB3_Pos)
#define CAN_FR_FB3 CAN_FR_FB3_Msk

#define CAN_FR_FB2_Pos 2U
#define CAN_FR_FB2_Msk (0x1U << CAN_FR_FB2_Pos)
#define CAN_FR_FB2 CAN_FR_FB2_Msk

#define CAN_FR_FB1_Pos 1U
#define CAN_FR_FB1_Msk (0x1U << CAN_FR_FB1_Pos)
#define CAN_FR_FB1 CAN_FR_FB1_Msk

#define CAN_FR_FB0_Pos 0U
#define CAN_FR_FB0_Msk (0x1U << CAN_FR_FB0_Pos)
#define CAN_FR_FB0 CAN_FR_FB0_Msk
#endif

#if !defined(STM32L431) && !defined(STM32L451)
/* ========================================================================= */
/* ============                       USB                       ============ */
/* ========================================================================= */
typedef struct
{
	__IOM uint32_t CNTR; /*!< USB control register */
	__IOM uint32_t ISTR; /*!< USB interrupt status register */
	__IM uint32_t FNR; /*!< USB frame number register */
	__IOM uint32_t DADDR; /*!< USB device address */
	__IOM uint32_t BTABLE; /*!< Buffer table address */
	__IOM uint32_t LPMCSR; /*!< LPM control and status register */
	__IOM uint32_t BCDR; /*!< Battery charging detector */
}STM32L4xx_USB_FS_TypeDef;

/*CNTR register*/
#define USB_CNTR_CTRM_Pos 15U
#define USB_CNTR_CTRM_Msk (0x1UL << USB_CNTR_CTRM_Pos)
#define USB_CNTR_CTRM USB_CNTR_CTRM_Msk

#define USB_CNTR_PMAOVRM_Pos 14U
#define USB_CNTR_PMAOVRM_Msk (0x1UL << USB_CNTR_PMAOVRM_Pos)
#define USB_CNTR_PMAOVRM USB_CNTR_PMAOVRM_Msk

#define USB_CNTR_ERRM_Pos 13U
#define USB_CNTR_ERRM_Msk (0x1UL << USB_CNTR_ERRM_Pos)
#define USB_CNTR_ERRM USB_CNTR_ERRM_Msk

#define USB_CNTR_WAKEUPM_Pos 12U
#define USB_CNTR_WAKEUPM_Msk (0x1UL << USB_CNTR_WAKEUPM_Pos)
#define USB_CNTR_WAKEUPM USB_CNTR_WAKEUPM_Msk

#define USB_CNTR_SUSPM_Pos 11U
#define USB_CNTR_SUSPM_Msk (0x1UL << USB_CNTR_SUSPM_Pos)
#define USB_CNTR_SUSPM USB_CNTR_SUSPM_Msk

#define USB_CNTR_RESETM_Pos 10U
#define USB_CNTR_RESETM_Msk (0x1UL << USB_CNTR_RESETM_Pos)
#define USB_CNTR_RESETM USB_CNTR_RESETM_Msk

#define USB_CNTR_SOFM_Pos 9U
#define USB_CNTR_SOFM_Msk (0x1UL << USB_CNTR_SOFM_Pos)
#define USB_CNTR_SOFM USB_CNTR_SOFM_Msk

#define USB_CNTR_ESOFM_Pos 8U
#define USB_CNTR_ESOFM_Msk (0x1UL << USB_CNTR_ESOFM_Pos)
#define USB_CNTR_ESOFM USB_CNTR_ESOFM_Msk

#define USB_CNTR_L1REQM_Pos 7U
#define USB_CNTR_L1REQM_Msk (0x1UL << USB_CNTR_L1REQM_Pos)
#define USB_CNTR_L1REQM USB_CNTR_L1REQM_Msk

#define USB_CNTR_L1RESUME_Pos 5U
#define USB_CNTR_L1RESUME_Msk (0x1UL << USB_CNTR_L1RESUME_Pos)
#define USB_CNTR_L1RESUME USB_CNTR_L1RESUME_Msk

#define USB_CNTR_RESUME_Pos 4U
#define USB_CNTR_RESUME_Msk (0x1UL << USB_CNTR_RESUME_Pos)
#define USB_CNTR_RESUME USB_CNTR_RESUME_Msk

#define USB_CNTR_FSUSP_Pos 3U
#define USB_CNTR_FSUSP_Msk (0x1UL << USB_CNTR_FSUSP_Pos)
#define USB_CNTR_FSUSP USB_CNTR_FSUSP_Msk

#define USB_CNTR_LPMODE_Pos 2U
#define USB_CNTR_LPMODE_Msk (0x1UL << USB_CNTR_LPMODE_Pos)
#define USB_CNTR_LPMODE USB_CNTR_LPMODE_Msk

#define USB_CNTR_PDWN_Pos 1U
#define USB_CNTR_PDWN_Msk (0x1UL << USB_CNTR_PDWN_Pos)
#define USB_CNTR_PDWN USB_CNTR_PDWN_Msk

#define USB_CNTR_FRES_Pos 0U
#define USB_CNTR_FRES_Msk (0x1UL << USB_CNTR_FRES_Pos)
#define USB_CNTR_FRES USB_CNTR_FRES_Msk

/*ISTR register*/
#define USB_ISTR_CTR_Pos 15U
#define USB_ISTR_CTR_Msk (0x1UL << USB_ISTR_CTR_Pos)
#define USB_ISTR_CTR USB_ISTR_CTR_Msk

#define USB_ISTR_PMAOVR_Pos 14U
#define USB_ISTR_PMAOVR_Msk (0x1UL << USB_ISTR_PMAOVR_Pos)
#define USB_ISTR_PMAOVR USB_ISTR_PMAOVR_Msk

#define USB_ISTR_ERR_Pos 13U
#define USB_ISTR_ERR_Msk (0x1UL << USB_ISTR_ERR_Pos)
#define USB_ISTR_ERR USB_ISTR_ERR_Msk

#define USB_ISTR_WKUP_Pos 12U
#define USB_ISTR_WKUP_Msk (0x1UL << USB_ISTR_WKUP_Pos)
#define USB_ISTR_WKUP USB_ISTR_WKUP_Msk

#define USB_ISTR_SUSP_Pos 11U
#define USB_ISTR_SUSP_Msk (0x1UL << USB_ISTR_SUSP_Pos)
#define USB_ISTR_SUSP USB_ISTR_SUSP_Msk

#define USB_ISTR_RESET_Pos 10U
#define USB_ISTR_RESET_Msk (0x1UL << USB_ISTR_RESET_Pos)
#define USB_ISTR_RESET USB_ISTR_RESET_Msk

#define USB_ISTR_SOF_Pos 9U
#define USB_ISTR_SOF_Msk (0x1UL << USB_ISTR_SOF_Pos)
#define USB_ISTR_SOF USB_ISTR_SOF_Msk

#define USB_ISTR_ESOF_Pos 8U
#define USB_ISTR_ESOF_Msk (0x1U << USB_ISTR_ESOF_Pos)
#define USB_ISTR_ESOF USB_ISTR_ESOF_Msk

#define USB_ISTR_L1REQ_Pos 7U
#define USB_ISTR_L1REQ_Msk (0x1UL << USB_ISTR_L1REQ_Pos)
#define USB_ISTR_L1REQ USB_ISTR_L1REQ_Msk

#define USB_ISTR_DIR_Pos 4U
#define USB_ISTR_DIR_Msk (0x1UL << USB_ISTR_DIR_Pos)
#define USB_ISTR_DIR USB_ISTR_DIR_Msk

#define USB_ISTR_EP_ID_Pos 0U
#define USB_ISTR_EP_ID_Msk (0xFUL << USB_ISTR_EP_ID_Pos)
#define USB_ISTR_EP_ID USB_ISTR_EP_ID_Msk

/*FNR register*/
#define USB_FNR_RXDP_Pos 15U
#define USB_FNR_RXDP_Msk (0x1UL << USB_FNR_RXDP_Pos)
#define USB_FNR_RXDP USB_FNR_RXDP_Msk

#define USB_FNR_RXDM_Pos 14U
#define USB_FNR_RXDM_Msk (0x1UL << USB_FNR_RXDM_Pos)
#define USB_FNR_RXDM USB_FNR_RXDM_Msk

#define USB_FNR_LCK_Pos 13U
#define USB_FNR_LCK_Msk (0x1UL << USB_FNR_LCK_Pos)
#define USB_FNR_LCK USB_FNR_LCK_Msk

#define USB_FNR_LSOF_Pos 12U
#define USB_FNR_LSOF_Msk (0x3UL << USB_FNR_LSOF_Pos)
#define USB_FNR_LSOF USB_FNR_LSOF_Msk

#define USB_FNR_FN_Pos 0U
#define USB_FNR_FN_Msk (0x7FFUL << USB_FNR_FN_Pos)
#define USB_FNR_FN USB_FNR_FN_Msk

/*DADDR register*/
#define USB_DADDR_EF_Pos 7U
#define USB_DADDR_EF_Msk (0x1UL << USB_DADDR_EF_Pos)
#define USB_DADDR_EF USB_DADDR_EF_Msk

#define USB_DADDR_ADD_Pos 0U
#define USB_DADDR_ADD_Msk (0x3FUL << USB_DADDR_ADD_Pos)
#define USB_DADDR_ADD USB_DADDR_ADD_Msk

/*BTABLE register*/
#define USB_BTABLE_Pos 3U
#define USB_BTABLE_Msk (0xFFF8UL << USB_BTABLE_Pos)
#define USB_BTABLE USB_BTABLE_Msk

/*LPMCSR register*/
#define USB_LPMCSR_BESL_Pos 4U
#define USB_LPMCSR_BESL_Msk (0xFUL << USB_LPMCSR_BESL_Pos)
#define USB_LPMCSR_BESL USB_LPMCSR_BESL_Msk

#define USB_LPMCSR_REMWAKE_Pos 3U
#define USB_LPMCSR_REMWAKE_Msk (0x1UL << USB_LPMCSR_REMWAKE_Pos)
#define USB_LPMCSR_REMWAKE USB_LPMCSR_REMWAKE_Msk

#define USB_LPMCSR_LPMACK_Pos 1U
#define USB_LPMCSR_LPMACK_Msk (0x1UL << USB_LPMCSR_LPMACK_Pos)
#define USB_LPMCSR_LPMACK USB_LPMCSR_LPMACK_Msk

#define USB_LPMCSR_LPMEN_Pos 0U
#define USB_LPMCSR_LPMEN_Msk (0x1UL << USB_LPMCSR_LPMEN_Pos)
#define USB_LPMCSR_LPMEN USB_LPMCSR_LPMEN_Msk

/*BCDR register*/
#define USB_BCDR_DPPU_Pos 15U
#define USB_BCDR_DPPU_Msk (0x1UL << USB_BCDR_DPPU_Pos)
#define USB_BCDR_DPPU USB_BCDR_DPPU_Msk

#define USB_BCDR_PS2DET_Pos 7U
#define USB_BCDR_PS2DET_Msk (0x1UL << USB_BCDR_PS2DET_Pos)
#define USB_BCDR_PS2DET USB_BCDR_PS2DET_Msk

#define USB_BCDR_SDET_Pos 6U
#define USB_BCDR_SDET_Msk (0x1UL << USB_BCDR_SDET_Pos)
#define USB_BCDR_SDET USB_BCDR_SDET_Msk

#define USB_BCDR_PDET_Pos 5U
#define USB_BCDR_PDET_Msk (0x1UL << USB_BCDR_PDET_Pos)
#define USB_BCDR_PDET USB_BCDR_PDET_Msk

#define USB_BCDR_DCDET_Pos 4U
#define USB_BCDR_DCDET_Msk (0x1UL << USB_BCDR_DCDET_Pos)
#define USB_BCDR_DCDET USB_BCDR_DCDET_Msk

#define USB_BCDR_CDEN_Pos 3U
#define USB_BCDR_CDEN_Msk (0x1UL << USB_BCDR_CDEN_Pos)
#define USB_BCDR_CDEN USB_BCDR_CDEN_Msk

#define USB_BCDR_PDEN_Pos 2U
#define USB_BCDR_PDEN_Msk (0x1UL << USB_BCDR_PDEN_Pos)
#define USB_BCDR_PDEN USB_BCDR_PDEN_Msk

#define USB_BCDR_DCDEN_Pos 1U
#define USB_BCDR_DCDEN_Msk (0x1UL << USB_BCDR_DCDEN_Pos)
#define USB_BCDR_DCDEN USB_BCDR_DCDEN_Msk

#define USB_BCDR_BCDEN_Pos 0U
#define USB_BCDR_BCDEN_Msk (0x1UL << USB_BCDR_BCDEN_Pos)
#define USB_BCDR_BCDEN USB_BCDR_BCDEN_Msk

/*EPR register*/
#define USB_EPR_CTR_RX_Pos 15U
#define USB_EPR_CTR_RX_Msk (0x1UL << USB_EPR_CTR_RX_Pos)
#define USB_EPR_CTR_RX USB_EPR_CTR_RX_Msk

#define USB_EPR_DTOG_RX_Pos 14U
#define USB_EPR_DTOG_RX_Msk (0x1UL << USB_EPR_DTOG_RX_Pos)
#define USB_EPR_DTOG_RX USB_EPR_DTOG_RX_Msk
#define USBD_EPR_SWBUF_TX_Pos 14U
#define USBD_EPR_SWBUF_TX_Msk (0x1UL << USBD_EPR_SWBUF_TX_Pos)
#define USBD_EPR_SWBUF_TX USBD_EPR_SWBUF_TX_Msk

#define USB_EPR_STAT_RX_Pos 12U
#define USB_EPR_STAT_RX_Msk (0x3UL << USB_EPR_STAT_RX_Pos)
#define USB_EPR_STAT_RX USB_EPR_STAT_RX_Msk
#define USB_EPR_STAT_RX_DISABLED (0x0UL << USB_EPR_STAT_RX_Pos)
#define USB_EPR_STAT_RX_STALL (0x1UL << USB_EPR_STAT_RX_Pos)
#define USB_EPR_STAT_RX_NAK (0x2UL << USB_EPR_STAT_RX_Pos)
#define USB_EPR_STAT_RX_VALID (0x3UL << USB_EPR_STAT_RX_Pos)

#define USB_EPR_SETUP_Pos 11U
#define USB_EPR_SETUP_Msk (0x3UL << USB_EPR_SETUP_Pos)
#define USB_EPR_SETUP USB_EPR_SETUP_Msk

#define USB_EPR_TYPE_Pos 9U
#define USB_EPR_TYPE_Msk (0x3UL << USB_EPR_TYPE_Pos)
#define USB_EPR_TYPE USB_EPR_TYPE_Msk
#define USB_EPR_TYPE_BULK (0x0UL << USB_EPR_TYPE_Pos)
#define USB_EPR_TYPE_CONTROL (0x1UL << USB_EPR_TYPE_Pos)
#define USB_EPR_TYPE_ISOCHRONOUS (0x2UL << USB_EPR_TYPE_Pos)
#define USB_EPR_TYPE_INTERRUPT (0x3UL << USB_EPR_TYPE_Pos)
#define USB_EPR_KIND_Pos 8U
#define USB_EPR_KIND_Msk (0x1UL << USB_EPR_KIND_Pos)
#define USB_EPR_KIND USB_EPR_KIND_Msk

#define USB_EPR_CTR_TX_Pos 7U
#define USB_EPR_CTR_TX_Msk (0x1UL << USB_EPR_CTR_TX_Pos)
#define USB_EPR_CTR_TX USB_EPR_CTR_TX_Msk

#define USB_EPR_DTOG_TX_Pos 6U
#define USB_EPR_DTOG_TX_Msk (0x1UL << USB_EPR_DTOG_TX_Pos)
#define USB_EPR_DTOG_TX USB_EPR_DTOG_TX_Msk
#define USBD_EPR_SWBUF_RX_Pos 6U
#define USBD_EPR_SWBUF_RX_Msk (0x1UL << USBD_EPR_SWBUF_RX_Pos)
#define USBD_EPR_SWBUF_RX USBD_EPR_SWBUF_RX_Msk

#define USB_EPR_STAT_TX_Pos 4U
#define USB_EPR_STAT_TX_Msk (0x3UL << USB_EPR_STAT_TX_Pos)
#define USB_EPR_STAT_TX USB_EPR_STAT_TX_Msk
#define USB_EPR_STAT_TX_DISABLED (0x0UL << USB_EPR_STAT_TX_Pos)
#define USB_EPR_STAT_TX_STALL (0x1UL << USB_EPR_STAT_TX_Pos)
#define USB_EPR_STAT_TX_NAK (0x2UL << USB_EPR_STAT_TX_Pos)
#define USB_EPR_STAT_TX_VALID (0x3UL << USB_EPR_STAT_TX_Pos)

#define USB_EPR_EA_Pos 0U
#define USB_EPR_EA_Msk (0xFUL << USB_EPR_EA_Pos)
#define USB_EPR_EA USB_EPR_EA_Msk

#define USBD_EPR_RW_Msk (USBD_EP_SETUP_Msk | USBD_EP_TYPE_Msk | USBD_EP_KIND_Msk | USBD_EP_ADDRESS_Msk)
#define USBD_EPR_RW USBD_EPR_RW_Msk
#define USBD_EPR_RC_W0_Msk (USBD_EP_CTR_RX_Msk | USBD_EP_CTR_TX_Msk)
#define USBD_EPR_RC_W0 USBD_EPR_RC_W0_Msk
#define USBD_EPR_T_Msk (USBD_EP_DTOG_RX_Msk | USBD_EP_STAT_RX_Msk | USBD_EP_DTOG_TX_Msk | USBD_EP_STAT_TX_Msk)
#define USBD_EPR_T USBD_EPR_T_Msk
#endif

/* ========================================================================= */
/* ============                     DBGMCU                      ============ */
/* ========================================================================= */
typedef struct
{
	__IM uint32_t IDCODE; /*!< Debug MCU device ID Code . */
	__IOM uint32_t CR; /*!< Debug MCU Configuration Register. */
	__IOM uint32_t APB1FZR1; /*!< Debug MCU APB1 Freeze Register 1. */
	__IOM uint32_t APB1FZR2; /*!< Debug MCU APB1 Freeze Register 2. */
	__IOM uint32_t APB2FZR; /*!< Debug MCU APB2 Freeze Register. */
}STM32L4xx_DBGMCU_TypeDef;

/*IDCODE register.*/
#define DBGMCU_IDCODE_REV_ID_Pos 16U
#define DBGMCU_IDCODE_REV_ID_Msk (0xFFFFUL << DBGMCU_IDCODE_REV_ID_Pos)
#define DBGMCU_IDCODE_REV_ID DBGMCU_IDCODE_REV_ID_Msk

#define DBGMCU_IDCODE_DEV_ID_Pos 0U
#define DBGMCU_IDCODE_DEV_ID_Msk (0xFFFUL << DBGMCU_IDCODE_DEV_ID_Pos)
#define DBGMCU_IDCODE_DEV_ID DBGMCU_IDCODE_DEV_ID_Msk

/*CR register.*/
#define DBGMCU_CR_TRACE_MODE_Pos 6U
#define DBGMCU_CR_TRACE_MODE_Msk (0x3UL << DBGMCU_CR_TRACE_MODE_Pos)
#define DBGMCU_CR_TRACE_MODE DBGMCU_CR_TRACE_MODE_Msk

#define DBGMCU_CR_IOEN_Pos 5U
#define DBGMCU_CR_IOEN_Msk (0x1UL << DBGMCU_CR_IOEN_Pos)
#define DBGMCU_CR_IOEN DBGMCU_CR_IOEN_Msk

#define DBGMCU_CR_DBG_STANDBY_Pos 2U
#define DBGMCU_CR_DBG_STANDBY_Msk (0x1UL << DBGMCU_CR_DBG_STANDBY_Pos)
#define DBGMCU_CR_DBG_STANDBY DBGMCU_CR_DBG_STANDBY_Msk

#define DBGMCU_CR_DBG_STOP_Pos 1U
#define DBGMCU_CR_DBG_STOP_Msk (0x1UL << DBGMCU_CR_DBG_STOP_Pos)
#define DBGMCU_CR_DBG_STOP DBGMCU_CR_DBG_STOP_Msk

#define DBGMCU_CR_DBG_SLEEP_Pos 0U
#define DBGMCU_CR_DBG_SLEEP_Msk (0x1UL << DBGMCU_CR_DBG_SLEEP_Pos)
#define DBGMCU_CR_DBG_SLEEP DBGMCU_CR_DBG_SLEEP_Msk

/*APB1FZR1 register.*/
#define DBGMCU_APB1FZR1_DBG_LPTIM1_STOP_Pos 31U
#define DBGMCU_APB1FZR1_DBG_LPTIM1_STOP_Msk (0x1UL << DBGMCU_APB1FZR1_DBG_LPTIM1_STOP_Pos)
#define DBGMCU_APB1FZR1_DBG_LPTIM1_STOP DBGMCU_APB1FZR1_DBG_LPTIM1_STOP_Msk

#define DBGMCU_APB1FZR1_DBG_CAN1_STOP_Pos 25U
#define DBGMCU_APB1FZR1_DBG_CAN1_STOP_Msk (0x1UL << DBGMCU_APB1FZR1_DBG_CAN1_STOP_Pos)
#define DBGMCU_APB1FZR1_DBG_CAN1_STOP DBGMCU_APB1FZR1_DBG_CAN1_STOP_Msk

#define DBGMCU_APB1FZR1_DBG_I2C3_STOP_Pos 23U
#define DBGMCU_APB1FZR1_DBG_I2C3_STOP_Msk (0x1UL << DBGMCU_APB1FZR1_DBG_I2C3_STOP_Pos)
#define DBGMCU_APB1FZR1_DBG_I2C3_STOP DBGMCU_APB1FZR1_DBG_I2C3_STOP_Msk

#define DBGMCU_APB1FZR1_DBG_I2C2_STOP_Pos 22U
#define DBGMCU_APB1FZR1_DBG_I2C2_STOP_Msk (0x1UL << DBGMCU_APB1FZR1_DBG_I2C2_STOP_Pos)
#define DBGMCU_APB1FZR1_DBG_I2C2_STOP DBGMCU_APB1FZR1_DBG_I2C2_STOP_Msk

#define DBGMCU_APB1FZR1_DBG_I2C1_STOP_Pos 21U
#define DBGMCU_APB1FZR1_DBG_I2C1_STOP_Msk (0x1UL << DBGMCU_APB1FZR1_DBG_I2C1_STOP_Pos)
#define DBGMCU_APB1FZR1_DBG_I2C1_STOP DBGMCU_APB1FZR1_DBG_I2C1_STOP_Msk

#define DBGMCU_APB1FZR1_DBG_IWDG_STOP_Pos 12U
#define DBGMCU_APB1FZR1_DBG_IWDG_STOP_Msk (0x1UL << DBGMCU_APB1FZR1_DBG_IWDG_STOP_Pos)
#define DBGMCU_APB1FZR1_DBG_IWDG_STOP DBGMCU_APB1FZR1_DBG_IWDG_STOP_Msk

#define DBGMCU_APB1FZR1_DBG_WWDG_STOP_Pos 11U
#define DBGMCU_APB1FZR1_DBG_WWDG_STOP_Msk (0x1UL << DBGMCU_APB1FZR1_DBG_WWDG_STOP_Pos)
#define DBGMCU_APB1FZR1_DBG_WWDG_STOP DBGMCU_APB1FZR1_DBG_WWDG_STOP_Msk

#define DBGMCU_APB1FZR1_DBG_RTC_STOP_Pos 10U
#define DBGMCU_APB1FZR1_DBG_RTC_STOP_Msk (0x1UL << DBGMCU_APB1FZR1_DBG_RTC_STOP_Pos)
#define DBGMCU_APB1FZR1_DBG_RTC_STOP DBGMCU_APB1FZR1_DBG_RTC_STOP_Msk

#define DBGMCU_APB1FZR1_DBG_TIM7_STOP_Pos 5U
#define DBGMCU_APB1FZR1_DBG_TIM7_STOP_Msk (0x1UL << DBGMCU_APB1FZR1_DBG_TIM7_STOP_Pos)
#define DBGMCU_APB1FZR1_DBG_TIM7_STOP DBGMCU_APB1FZR1_DBG_TIM7_STOP_Msk

#define DBGMCU_APB1FZR1_DBG_TIM6_STOP_Pos 4U
#define DBGMCU_APB1FZR1_DBG_TIM6_STOP_Msk (0x1UL << DBGMCU_APB1FZR1_DBG_TIM6_STOP_Pos)
#define DBGMCU_APB1FZR1_DBG_TIM6_STOP DBGMCU_APB1FZR1_DBG_TIM6_STOP_Msk

#define DBGMCU_APB1FZR1_DBG_TIM2_STOP_Pos 0U
#define DBGMCU_APB1FZR1_DBG_TIM2_STOP_Msk (0x1UL << DBGMCU_APB1FZR1_DBG_TIM2_STOP_Pos)
#define DBGMCU_APB1FZR1_DBG_TIM2_STOP DBGMCU_APB1FZR1_DBG_TIM2_STOP_Msk

/*APB1FZR2 register.*/
#define DBGMCU_APB1FZR2_DBG_LPTIM2_STOP_Pos 5U
#define DBGMCU_APB1FZR2_DBG_LPTIM2_STOP_Msk (0x1UL << DBGMCU_APB1FZR2_DBG_LPTIM2_STOP_Pos)
#define DBGMCU_APB1FZR2_DBG_LPTIM2_STOP DBGMCU_APB1FZR2_DBG_LPTIM2_STOP_Msk

/*APB2FZR register.*/
#define DBGMCU_APB2FZR_DBG_TIM16_STOP_Pos 17U
#define DBGMCU_APB2FZR_DBG_TIM16_STOP_Msk (0x1UL << DBGMCU_APB2FZR_DBG_TIM16_STOP_Pos)
#define DBGMCU_APB2FZR_DBG_TIM16_STOP DBGMCU_APB2FZR_DBG_TIM16_STOP_Msk

#define DBGMCU_APB2FZR_DBG_TIM15_STOP_Pos 16U
#define DBGMCU_APB2FZR_DBG_TIM15_STOP_Msk (0x1UL << DBGMCU_APB2FZR_DBG_TIM15_STOP_Pos)
#define DBGMCU_APB2FZR_DBG_TIM15_STOP DBGMCU_APB2FZR_DBG_TIM15_STOP_Msk

#define DBGMCU_APB2FZR_DBG_TIM1_STOP_Pos 11U
#define DBGMCU_APB2FZR_DBG_TIM1_STOP_Msk (0x1UL << DBGMCU_APB2FZR_DBG_TIM1_STOP_Pos)
#define DBGMCU_APB2FZR_DBG_TIM1_STOP DBGMCU_APB2FZR_DBG_TIM1_STOP_Msk

/* ========================================================================= */
/* ============           Device Electronic Signature           ============ */
/* ========================================================================= */
typedef struct
{
	__IM uint32_t UID[3]; /*!< Unique device ID register (96 bit). */
}STM32L4xx_DES_UID_TypeDef;

typedef struct
{
	__IM uint32_t FLASH_SIZE; /*!< Flash Size data register. */
}STM32L4xx_DES_Flash_Size_Data_TypeDef;

typedef struct
{
	__IM uint32_t PKG; /*!< Package data register. */
}STM32L4xx_DES_Package_Data_TypeDef;

#define DES_FLASH_SIZE_DATA_Pos 0U
#define DES_FLASH_SIZE_DATA_Msk (0xFFFFUL << FLASH_SIZE_DATA_Pos)
#define DES_FLASH_SIZE_DATA DES_FLASH_SIZE_DATA_Msk

#define DES_PKG_DATA_Pos 0U
#define DES_PKG_DATA_Msk (0x1FUL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA DES_PKG_DATA_Msk
#define DES_PKG_DATA_LQFP64 (0x0UL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA_WLCSP64 (0x1UL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA_LQFP100 (0x2UL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA_WLCSP36 (0x5UL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA_UFQFPN32 (0x8UL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA_LQFP32 (0x9UL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA_UFQFPN48 (0xAUL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA_LQFP48 (0xBUL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA_WLCSP49 (0xCUL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA_UFBGA64 (0xDUL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA_UFBGA100 (0xEUL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA_WLCSP36_SMPS (0xFUL << DES_PKG_DATA_Pos)
#define DES_PKG_DATA_LQFP64_SMPS (0x16UL << DES_PKG_DATA_Pos)

/* ================  End of section using anonymous unions  ================ */
#if defined (__GNUC__)
/* anonymous unions are enabled by default */
#else
#warning Not supported compiler type
#endif


/* ========================================================================= */
/* ============     Device Specific Peripheral Address Map      ============ */
/* ========================================================================= */

/* Peripheral and SRAM base address */
#define STM32L4xx_FLASH_BASE       (0x08000000UL)                              /* (FLASH     ) Base Address */
#define STM32L4xx_SRAM_BASE        (0x20000000UL)                              /* (SRAM      ) Base Address */
#define STM32L4xx_PERIPH_BASE      (0x40000000UL)                              /* (Peripheral) Base Address */

/* Peripheral memory map */
#define STM32L4xx_APB1_PERIPHERAL_BASE STM32L4xx_PERIPH_BASE
#define STM32L4xx_APB2_PERIPHERAL_BASE (STM32L4xx_PERIPH_BASE + 0x10000UL)
#define STM32L4xx_AHB1_PERIPHERAL_BASE (STM32L4xx_PERIPH_BASE + 0x20000UL)
#define STM32L4xx_AHB2_PERIPHERAL_BASE (STM32L4xx_PERIPH_BASE + 0x8000000UL)

#define STM32L4xx_RNG_BASE (STM32L4xx_AHB2_PERIPHERAL_BASE + 0x8060800UL)
#if defined(STM32L422) || defined(STM32L442) || defined(STM32L443) || defined(STM32L462)
#define STM32L4xx_AES_BASE (STM32L4xx_AHB2_PERIPHERAL_BASE + 0x8060000UL)
#endif
#define STM32L4xx_ADC_BASE (STM32L4xx_AHB2_PERIPHERAL_BASE + 0x8040000UL)
#define STM32L4xx_GPIOH_BASE (STM32L4xx_AHB2_PERIPHERAL_BASE + 0x1C00UL)
#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
#define STM32L4xx_GPIOE_BASE (STM32L4xx_AHB2_PERIPHERAL_BASE + 0x1000UL)
#endif
#if !defined(STM32L432) && !defined(STM32L442)
#define STM32L4xx_GPIOD_BASE (STM32L4xx_AHB2_PERIPHERAL_BASE + 0xC00UL)
#endif
#define STM32L4xx_GPIOC_BASE (STM32L4xx_AHB2_PERIPHERAL_BASE + 0x800UL)
#define STM32L4xx_GPIOB_BASE (STM32L4xx_AHB2_PERIPHERAL_BASE + 0x400UL)
#define STM32L4xx_GPIOA_BASE STM32L4xx_AHB2_PERIPHERAL_BASE


#define STM32L4xx_TSC_BASE (STM32L4xx_AHB1_PERIPHERAL_BASE + 0x4000UL)
#define STM32L4xx_CRC_BASE (STM32L4xx_AHB1_PERIPHERAL_BASE + 0x3000UL)
#define STM32L4xx_FLASH_PERIPHERAL_BASE (STM32L4xx_AHB1_PERIPHERAL_BASE + 0x2000UL)
#define STM32L4xx_RCC_BASE (STM32L4xx_AHB1_PERIPHERAL_BASE + 0x1000UL)
#define STM32L4xx_DMA2_BASE (STM32L4xx_AHB1_PERIPHERAL_BASE + 0x400UL)
#define STM32L4xx_DMA1_BASE STM32L4xx_AHB1_PERIPHERAL_BASE


#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define STM32L4xx_DFSDM1_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x6000UL)
#endif
#if !defined(STM32L412) && !defined(STM32L422)
#define STM32L4xx_SAI1_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x5400UL)
#endif
#define STM32L4xx_TIM16_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x4400UL)
#define STM32L4xx_TIM15_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x4000UL)
#define STM32L4xx_USART1_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x3800UL)
#define STM32L4xx_SPI1_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x3000UL)
#define STM32L4xx_TIM1_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x2C00UL)
#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
#define STM32L4xx_SDMMC_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x2800UL)
#endif
#define STM32L4xx_FIREWALL_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x1C00UL)
#define STM32L4xx_EXTI_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x400UL)
#define STM32L4xx_COMP_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x200UL)
#if !defined(STM32L412) && !defined(STM32L422)&& !defined(STM32L432) && !defined(STM32L442)
#define STM32L4xx_VREFBUF_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x30UL)
#endif
#define STM32L4xx_SYSCFG_BASE STM32L4xx_APB2_PERIPHERAL_BASE


#define STM32L4xx_LPTIM2_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x9400UL)
#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
#define STM32L4xx_SWPMI1_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x8800UL)
#endif
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define STM32L4xx_I2C4_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x8400UL)
#endif
#define STM32L4xx_LPUART1_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x8000UL)
#define STM32L4xx_LPTIM1_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x7C00UL)
#define STM32L4xx_OPAMP_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x7800UL)
#if !defined(STM32L412) && !defined(STM32L422)
#define STM32L4xx_DAC1_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x7400UL)
#endif
#define STM32L4xx_PWR_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x7000UL)
#if !defined(STM32L431) && !defined(STM32L451)
#define STM32L4xx_USB_FS_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x6800UL)
#define STM32L4xx_USB_SRAM_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x6C00UL)
#endif
#if !defined(STM32L412) && !defined(STM32L422)
#define STM32L4xx_CAN1_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x6400UL)
#endif
#define STM32L4xx_CRS_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x6000UL)
#define STM32L4xx_I2C3_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x5C00UL)
#if !defined(STM32L432) && !defined(STM32L442)
#define STM32L4xx_I2C2_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x5800UL)
#endif
#define STM32L4xx_I2C1_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x5400UL)
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define STM32L4xx_UART4_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x4C00UL)
#endif
#if !defined(STM32L432) && !defined(STM32L442)
#define STM32L4xx_USART3_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x4800UL)
#endif
#define STM32L4xx_USART2_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x4400UL)
#define STM32L4xx_SPI3_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x3C00UL)
#if !defined(STM32L432) && !defined(STM32L442)
#define STM32L4xx_SPI2_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x3800UL)
#endif
#define STM32L4xx_IWDG_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x3000UL)
#define STM32L4xx_WWDG_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x2C00UL)
#define STM32L4xx_RTC_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x2800UL)
#if defined(STM32L433) || defined(STM32L443)
#define STM32L4xx_LCD_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x2400UL)
#endif
#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
#define STM32L4xx_TIM7_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x1400UL)
#endif
#define STM32L4xx_TIM6_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x1000UL)
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define STM32L4xx_TIM3_BASE (STM32L4xx_APB2_PERIPHERAL_BASE + 0x400UL)
#endif
#define STM32L4xx_TIM2_BASE STM32L4xx_APB2_PERIPHERAL_BASE

/*Misc*/
#define STM32L4xx_DES_UNIQUE_DEVICE_ID_BASE (0x1FFF7590UL)
#define STM32L4xx_DES_FLASH_SIZE_DATA_BASE (0x1FFF75E0UL) 
#define STM32L4xx_DES_PACKAGE_DATA_BASE (0x1FFF7500UL)

/* ========================================================================= */
/* ============             Peripheral declaration              ============ */
/* ========================================================================= */
#define RNG ((STM32L4xx_RNG_TypeDef *) STM32L4xx_RNG_BASE)
#if defined(STM32L422) || defined(STM32L442) || defined(STM32L443) || defined(STM32L462)
#define AES ((STM32L4xx_AES_TypeDef *) STM32L4xx_AES_BASE)
#endif
#define ADC_MASTER ((STM32L4xx_ADC_TypeDef *) STM32L4xx_ADC_BASE)
#define ADC_SLAVE ((STM32L4xx_ADC_TypeDef *) STM32L4xx_ADC_BASE + 0x100UL)
#define ADC_COMMON ((STM32L4xx_ADC_CM_TypeDef *) STM32L4xx_ADC_BASE + 0x300UL)
#define GPIOH ((STM32L4xx_GPIO_TypeDef *) STM32L4xx_GPIOH_BASE)
#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
#define GPIOE ((STM32L4xx_GPIO_TypeDef *) STM32L4xx_GPIOE_BASE)
#endif
#if !defined(STM32L432) && !defined(STM32L442)
#define GPIOD ((STM32L4xx_GPIO_TypeDef *) STM32L4xx_GPIOD_BASE)
#endif
#define GPIOC ((STM32L4xx_GPIO_TypeDef *)STM32L4xx_GPIOC_BASE)
#define GPIOB ((STM32L4xx_GPIO_TypeDef*) STM32L4xx_GPIOB_BASE)
#define GPIOA ((STM32L4xx_GPIO_TypeDef*) STM32L4xx_GPIOA_BASE)
#define TSC ((STM32L4xx_TSC_TypeDef*) STM32L4xx_TSC_BASE)
#define CRC ((STM32L4xx_CRC_TypeDef*) STM32L4xx_CRC_BASE)
#define FLASH ((STM32L4xx_FLASH_TypeDef*) STM32L4xx_FLASH_PERIPHERAL_BASE)
#define RCC ((STM32L4xx_RCC_TypeDef *) STM32L4xx_RCC_BASE)
#define DMA2 ((STM32L4xx_DMA_TypeDef *) STM32L4xx_DMA2_BASE) 
#define DMA1 ((STM32L4xx_DMA_TypeDef *) STM32L4xx_DMA1_BASE) 
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define DFSDM_CH0 ((STM32L4xx_DFSDM_CH_TypeDef *) STM32L4xx_DFSDM1_BASE)
#define DFSDM_CH1 ((STM32L4xx_DFSDM_CH_TypeDef *) (STM32L4xx_DFSDM1_BASE + 0x20UL))
#define DFSDM_CH2 ((STM32L4xx_DFSDM_CH_TypeDef *) (STM32L4xx_DFSDM1_BASE + 0x40UL))
#define DFSDM_CH3 ((STM32L4xx_DFSDM_CH_TypeDef *) (STM32L4xx_DFSDM1_BASE + 0x60UL))

#define DFSDM_FLT0 ((STM32L4xx_DFSDM_FLT_TypeDef *) (STM32L4xx_DFSDM1_BASE + 0x100UL))
#define DFSDM_FLT1 ((STM32L4xx_DFSDM_FLT_TypeDef *) (STM32L4xx_DFSDM1_BASE + 0x180UL))
#endif
#if !defined(STM32L412) && !defined(STM32L422)
#define SAI ((STM32L4xx_SAI_TypeDef *) STM32L4xx_SAI1_BASE)
#endif
#define TIM16 ((STM32L4xx_TIM16_TypeDef *) STM32L4xx_TIM16_BASE)
#define TIM15 ((STM32L4xx_TIM15_TypeDef *) STM32L4xx_TIM15_BASE)
#define USART1 ((STM32L4xx_USART_TypeDef *) STM32L4xx_USART1_BASE)
#define SPI1 ((STM32L4xx_SPI_TypeDef *) STM32L4xx_SPI1_BASE)
#define TIM1 ((STM32L4xx_TIM1_TypeDef *) STM32L4xx_TIM1_BASE)
#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
#define SDMMC ((STM32L4xx_SDMMC_TypeDef *) STM32L4xx_SDMMC_BASE)
#endif
#define FIREWALL ((STM32L4xx_FW_TypeDef *) STM32L4xx_FIREWALL_BASE)
#define EXTI ((STM32L4xx_EXTI_TypeDef *) STM32L4xx_EXTI_BASE)
#define COMP1 ((STM32L4xx_COMP_TypeDef *) STM32L4xx_COMP_BASE)
#if !defined(STM32L412) && !defined(STM32L422)
#define COMP2 ((STM32L4xx_COMP_TypeDef *) STM32L4xx_COMP_BASE + 0x4UL)
#endif
#if !defined(STM32L412) && !defined(STM32L422) && !defined(STM32L432) && !defined(STM32L442)
#define VREFBUF ((STM32L4xx_VREFBUF_TypeDef *) STM32L4xx_VREFBUF_BASE)
#endif
#define SYSCFG ((STM32L4xx_SYSCFG_TypeDef *) STM32L4xx_SYSCFG_BASE)
#define LPTIM2 ((STM32L4xx_LPTIM_TypeDef *) STM32L4xx_LPTIM2_BASE)
#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
#define SWPMI ((STM32L4xx_SWPMI_TypeDef *) STM32L4xx_SWPMI1_BASE)
#endif
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define I2C4 ((STM32L4xx_I2C_TypeDef *) STM32L4xx_I2C4_BASE)
#endif
#define LPURART ((STM32L4xx_LPUART_TypeDef *) STM32L4xx_LPUART1_BASE)
#define LPTIM1 ((STM32L4xx_LPTIM_TypeDef *) STM32L4xx_LPTIM1_BASE)
#define OPAMP ((STM32L4xx_OPAMP_TypeDef *) STM32L4xx_OPAMP_BASE)
#if !defined(STM32L412) && !defined(STM32L422)
#define DAC ((STM32L4xx_DAC_TypeDef *) STM32L4xx_DAC1_BASE)
#endif
#define PWR ((STM32L4xx_PWR_TypeDef*) STM32L4xx_PWR_BASE)
#if !defined(STM32L431) && !defined(STM32L451)
#define USB ((STM32L4xx_USB_FS_TypeDef*) STM32L4xx_USB_FS_BASE)
#endif
#if !defined(STM32L412) && !defined(STM32L422)
#define CAN ((STM32L4xx_CAN_control_and_status_TypeDef*) STM32L4xx_CAN1_BASE)
#define CAN_MAILBOX ((STM32L4xx_CAN_mailbox_TypeDef*) (STM32L4xx_CAN1_BASE + 0x180UL))
#define CAN_FILTER ((STM32L4xx_CAN_filter_TypeDef*) (STM32L4xx_CAN1_BASE + 0x200UL))
#endif
#define CRS ((STM32L4xx_CRS_TypeDef*) STM32L4xx_CRS_BASE)
#define I2C3 ((STM32L4xx_I2C_TypeDef*) STM32L4xx_I2C3_BASE)
#if !defined(STM32L432) && !defined(STM32L442)
#define I2C2 ((STM32L4xx_I2C_TypeDef*) STM32L4xx_I2C2_BASE)
#endif
#define I2C1 ((STM32L4xx_I2C_TypeDef*) STM32L4xx_I2C1_BASE)
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define UART4 ((STM32L4xx_USART_TypeDef*) STM32L4xx_UART4_BASE)
#endif
#if !defined(STM32L432) && !defined(STM32L442)
#define USART3 ((STM32L4xx_USART_TypeDef*) STM32L4xx_USART3_BASE)
#endif
#define USART2 ((STM32L4xx_USART_TypeDef*) STM32L4xx_USART2_BASE)
#if !defined(STM32L412) && !defined(STM32L422)
#define SPI3 ((STM32L4xx_SPI_TypeDef*) STM32L4xx_SPI3_BASE)
#endif
#if !defined(STM32L432) && !defined(STM32L442)
#define SPI2 ((STM32L4xx_SPI_TypeDef*) STM32L4xx_SPI2_BASE)
#endif	
#define IWDG ((STM32L4xx_IWDG_TypeDef*) STM32L4xx_IWDG_BASE)
#define WWDG ((STM32L4xx_WWDG_TypeDef*) STM32L4xx_WWDG_BASE)
#define RTC ((STM32L4xx_RTC_TypeDef*) STM32L4xx_RTC_BASE)
#if defined(STM32L433) || defined(STM32L443)
#define LCD ((STM32L4xx_LCD_TypeDef*) STM32L4xx_LCD_BASE)
#endif
#if defined(STM32L431) || defined(STM32L432) || defined(STM32L433) || defined(STM32L442) || defined(STM32L443)
#define TIM7 ((STM32L4xx_TIM6_7_TypeDef *) STM32L4xx_TIM7_BASE)
#endif
#define TIM6 ((STM32L4xx_TIM6_7_TypeDef *) STM32L4xx_TIM6_BASE)
#if defined(STM32L451) || defined(STM32L452) || defined(STM32L462)
#define TIM3 ((STM32L4xx_TIM2_3_TypeDef *) STM32L4xx_TIM3_BASE)
#endif
#define TIM2 ((STM32L4xx_TIM2_3_TypeDef *) STM32L4xx_TIM2_BASE)
#define UID ((STM32L4xx_DES_UID_TypeDef *) STM32L4xx_DES_UNIQUE_DEVICE_ID_BASE)
#define FLASH_SIZE_DATA ((STM32L4xx_DES_Flash_Size_Data_TypeDef *) STM32L4xx_DES_FLASH_SIZE_DATA_BASE)
#define PACKAGE_DATA ((STM32L4xx_DES_Package_Data_TypeDef *) STM32L4xx_DES_PACKAGE_DATA_BASE)


/* ========================================================================= */
/* ============                     MACROS                      ============ */
/* ========================================================================= */

#define SET(reg, flags) ((reg) |= (flags))
#define CLEAR(reg, flags) ((reg) &= ~(flags))
#define GET(reg, mask) ((reg) & (mask))
#define TOGGLE(reg, flags) ((reg) ^= (flags))

#define GET_BIT(reg, bit_mask) (!((reg) & (bit_mask))) ? 0x0U : 0x1U
#define GET_VAL(reg, mask, pos) (((reg) & (mask)) >> (pos))

#define MIN(x, y) ((x) < (y)) ? (x) : (y)
#define MAX(x, y) ((x) > (y)) ? (x) : (y)
#define UNUSED(val) (void)(val)

#ifdef __cplusplus
}
#endif
#endif //endif STM32L4xx_H