#ifndef SPINLOCK_STM32L4XX_H
#define SPINLOCK_STM32L4XX_H

#include <stdint.h>
#include "stm32l4xx.h"


/**
 * @brief Do nothing for a specified amount of time.
 * @param max Amount of time to wait.
 * @return 
 */
__STATIC_FORCEINLINE void __spinlock(uint32_t max)
{
  __ASM volatile (
  ".syntax unified\n"
  "0:\n\t"
    "subs %0,%0,#1\n\t"
    "bne  0b\n"
  : "+l" (max) : : "cc"
  );
}


#endif /*SPINLOCK_STM32L4XX_H*/