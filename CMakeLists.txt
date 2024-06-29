cmake_minimum_required (VERSION 3.29.0)
project(STM32L4xx)
add_library(STM32L4xx INTERFACE)

# Enable CMake support for C language
enable_language(C)

target_compile_definitions(STM32L4xx INTERFACE
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(STM32L4xx PRIVATE
    CMSIS_6\CMSIS\Core\Include
)

target_include_directories(STM32L4xx INTERFACE
    inc
)

target_sources(STM32L4xx INTERFACE
    src/startup_stm32l4xx.c
    src/system_stm32l4xx.c
    src/syscalls.c
)

