cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME Test01_2026_03_26)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# 确保激活 C 语言与汇编
project(${CMAKE_PROJECT_NAME} C ASM) 
message("Build type: " ${CMAKE_BUILD_TYPE})

add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

# ==========================================
# 🚨 战队核心库配置区 🚨
# ==========================================

# 1. 搜索所有 .c 源码
file(GLOB_RECURSE TEAM_LIB_SOURCES 
    "BSP/*.c" 
    "MODULE/*.c"
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/main.c
    ${TEAM_LIB_SOURCES}
)

# 2. 添加所有头文件路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/DSP/Include   # <== 修复了 arm_math.h 找不到的核心问题
    BSP/CAN
    BSP/USART
    BSP/DWT
    MODULE
    MODULE/algorithm
    MODULE/daemon
    MODULE/motor
    MODULE/motor/DJImotor
    MODULE/relay
    MODULE/remote
)

# 3. 宏定义 (已剔除 __FPU_PRESENT 消除黄字警告)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 4. 链接系统数学库与大疆 DSP 硬件加速库
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m
    ${CMAKE_SOURCE_DIR}/libarm_cortexM4lf_math.a
)