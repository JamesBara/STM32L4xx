/**************************************************************************//**
 * @file     system_ARMCM4.c
 * @brief    CMSIS Device System Source File for
 *           ARMCM4 Device
 * @version  V1.0.1
 * @date     15. November 2019
 ******************************************************************************/
/*
 * Copyright (c) 2009-2019 Arm Limited. All rights reserved.
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

#include <stdint.h>
#include "stm32l4xx.h"

 /*----------------------------------------------------------------------------
 Define clocks
 *----------------------------------------------------------------------------*/
#define  XTAL            (8000000UL)      /* External Oscillator frequency */
#define  HSI16           (16000000UL)     /* HSI Frequency */
#define  MSI_DEFAULT    (4000000UL)      /*Default startup frequency*/

static const uint32_t msi_range[] = { 100000UL, 200000UL, 400000UL, 800000UL, 1000000UL, 2000000UL, MSI_DEFAULT, 8000000UL, 16000000UL, 24000000UL, 32000000UL, 48000000UL };

 /*----------------------------------------------------------------------------
 Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
extern const VECTOR_TABLE_Type __VECTOR_TABLE[101];

/*----------------------------------------------------------------------------
System Core Clock Variable
*----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = MSI_DEFAULT;  /* System Core Clock Frequency */


/*----------------------------------------------------------------------------
System Core Clock update function
*----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)
{

	switch (GET(RCC->CFGR, RCC_CFGR_SWS_Msk))
	{
		case RCC_CFGR_SWS_MSI:
			if (GET_BIT(RCC->CR, RCC_CR_MSIRGSEL_Msk))
			{
				/*Get the value of the RCC->CR register for the msi bits. Use it to get the correct frequency from the table.*/
				SystemCoreClock = msi_range[GET_VAL(RCC->CR, RCC_CR_MSIRANGE_Msk, RCC_CR_MSIRANGE_Pos)];
			}
			else
			{
				/*Get the value of the RCC->CSR register for the msi bits. Use it to get the correct frequency from the table.*/
				SystemCoreClock = msi_range[GET_VAL(RCC->CSR, RCC_CSR_MSISRANGE_Msk, RCC_CSR_MSISRANGE_Pos)];
			}
			break;
		case RCC_CFGR_SWS_HSI16:
			SystemCoreClock = HSI16;
			break;
		case RCC_CFGR_SWS_HSE:
			SystemCoreClock = XTAL;
			break;
		case RCC_CFGR_SWS_PLL:
			switch (GET(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_Msk))
			{
			case 0:
				SystemCoreClock = MSI_DEFAULT;
				break;
			case RCC_PLLCFGR_PLLSRC_MSI:
				if (GET_BIT(RCC->CR, RCC_CR_MSIRGSEL_Msk))
				{
					/* Get the value of the RCC->PLLCFGR register for the plln, pplm and pllr bits.
					* Get the value of the RCC->CR register for the msi bits. Use it to get the correct msi frequency from the table.
					* Calculate the frequency using the formula: plln / (pllm + 1) * msi_frequency / ((pllr + 1) * 2)
					*/
					SystemCoreClock = ((GET_VAL(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_Msk, RCC_PLLCFGR_PLLN_Pos)
						/ (GET_VAL(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk, RCC_PLLCFGR_PLLM_Pos) + 1UL))
						* msi_range[GET_VAL(RCC->CR, RCC_CR_MSIRANGE_Msk,RCC_CR_MSIRANGE_Pos)]) 
						/ ((GET_VAL(RCC->PLLCFGR, RCC_PLLCFGR_PLLR_Msk, RCC_PLLCFGR_PLLR_Pos) + 1UL) << 1UL);
				}
				else
				{
					/* Get the value of the RCC->PLLCFGR register for the plln, pplm and pllr bits.
					* Get the value of the RCC->CSR register for the msi bits. Use it to get the correct msi frequency from the table.
					* Calculate the frequency using the formula: plln / (pllm + 1) * msi_frequency / ((pllr + 1) * 2)
					*/
					SystemCoreClock = ((GET_VAL(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_Msk,RCC_PLLCFGR_PLLN_Pos)
						/ (GET_VAL(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk, RCC_PLLCFGR_PLLM_Pos) + 1UL))
						* msi_range[GET_VAL(RCC->CSR, RCC_CSR_MSISRANGE_Msk, RCC_CSR_MSISRANGE_Pos)]) 
						/ ((GET_VAL(RCC->PLLCFGR, RCC_PLLCFGR_PLLR_Msk,RCC_PLLCFGR_PLLR_Pos) + 1UL) << 1UL);
				}
				break;
			case RCC_PLLCFGR_PLLSRC_HSI16:
				/* Get the value of the RCC->PLLCFGR register for the plln, pplm and pllr bits.
				* Calculate the frequency using the formula: plln / (pllm + 1) * hsi16_frequency / ((pllr + 1) * 2)
				*/
				SystemCoreClock = ((GET_VAL(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_Msk, RCC_PLLCFGR_PLLN_Pos)
					/ (GET_VAL(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk, RCC_PLLCFGR_PLLM_Pos) + 1UL))
					* HSI16) / ((GET_VAL(RCC->PLLCFGR, RCC_PLLCFGR_PLLR_Msk, RCC_PLLCFGR_PLLR_Pos) + 1UL) << 1UL);
				break;
			case RCC_PLLCFGR_PLLSRC_HSE:
				/* Get the value of the RCC->PLLCFGR register for the plln, pplm and pllr bits.
				* Calculate the frequency using the formula: plln / (pllm + 1) * hse_frequency / ((pllr + 1) * 2)
				*/
				SystemCoreClock = ((GET_VAL(RCC->PLLCFGR, RCC_PLLCFGR_PLLN_Msk, RCC_PLLCFGR_PLLN_Pos)
					/ (GET_VAL(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_Msk, RCC_PLLCFGR_PLLM_Pos) + 1UL))
					* XTAL) / ((GET_VAL(RCC->PLLCFGR, RCC_PLLCFGR_PLLR_Msk, RCC_PLLCFGR_PLLR_Pos) + 1UL) << 1UL);
				break;
			}
			break;
	}
}

/*----------------------------------------------------------------------------
System initialization function
*----------------------------------------------------------------------------*/
void SystemInit (void)
{

#if defined (__VTOR_PRESENT) && (__VTOR_PRESENT == 1U)
	SCB->VTOR = (uint32_t) &(__VECTOR_TABLE[0]);
#endif

#if defined (__FPU_USED) && (__FPU_USED == 1U)
	SCB->CPACR |= ((3U << 10U*2U) |           /* enable CP10 Full Access */
		(3U << 11U*2U)  );         /* enable CP11 Full Access */
#endif

#ifdef UNALIGNED_SUPPORT_DISABLE
	SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
#endif

	SystemCoreClock = MSI_DEFAULT;
}