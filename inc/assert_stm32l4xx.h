#ifndef ASSERT_STM32L4XX_H
#define ASSERT_STM32L4XX_H

#include "stm32l4xx.h"

/**
 * @brief Used by the ASSERT() macro. A software breakpoint is used in debug builds
 * and a device reset in release builds.
 * 
 * @param file
 * @param line 
 * @param func 
 * @param val 
 */
static inline void __assert_func(const char* file, int line, const char* func, const char* val)
{
    UNUSED(file);
    UNUSED(line);
    UNUSED(func);
    UNUSED(val);
    #ifndef DEBUG
        NVIC_SystemReset();
    #else
        __BKPT(0);
    #endif /*DEBUG*/
}

/************************************************
 * @brief Custom assert Macro
 ***********************************************/
#define ASSERT(param) ((param) ? (void)0 : __assert_func(__FILE__, __LINE__, __func__, #param))


#endif /*ASSERT_STM32L4XX_H*/