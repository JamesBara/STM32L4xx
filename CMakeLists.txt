cmake_minimum_required (VERSION 3.29.0)
project(STM32L4xx)
add_library(STM32L4xx INTERFACE)

enable_language(C)

target_include_directories(STM32L4xx INTERFACE
    CMSIS_6/CMSIS/Core/Include    
    inc
)

target_sources(STM32L4xx INTERFACE
    src/startup_stm32l4xx.c
    src/system_stm32l4xx.c
    src/syscalls.c
)


