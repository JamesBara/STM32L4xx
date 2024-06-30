cmake_minimum_required (VERSION 3.29.0)
project(STM32L4xx)
add_library(STM32L4xx INTERFACE)

enable_language(C)

target_compile_definitions(STM32L4xx INTERFACE
    $<$<CONFIG:Debug>:DEBUG>
    # User definition for microcontroller used. Valid options:
    #STM32L412, STM32L422, STM32L431, STM32L432, STM32L433, STM32L442, STM32L443, STM32L451, STM32L452, STM32L462.
)


target_include_directories(STM32L4xx INTERFACE
    CMSIS_6/CMSIS/Core/Include    
    inc
)

target_sources(STM32L4xx INTERFACE
    src/startup_stm32l4xx.c
    src/system_stm32l4xx.c
    src/syscalls.c
)


