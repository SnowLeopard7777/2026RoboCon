cmake_minimum_required(VERSION 3.22)

project(RC_NewR1_ElevateR2_M3508_Keil C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set(MCU_FLAGS
    -mcpu=cortex-m7
    -mfpu=fpv5-d16
    -mfloat-abi=hard
    -mthumb
)

add_compile_options(
    ${MCU_FLAGS}
    -Wall
    -ffunction-sections
    -fdata-sections
    -fno-common
    -fmessage-length=0
)

add_link_options(
    ${MCU_FLAGS}
    -T${CMAKE_SOURCE_DIR}/STM32H743VITX_FLASH.ld
    -Wl,--gc-sections
    -Wl,-Map=${PROJECT_NAME}.map
    --specs=nano.specs
    --specs=nosys.specs
)

set(FREERTOS_PORT_DIR ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F)

add_executable(${PROJECT_NAME}
    startup_stm32h743xx.s
    Core/Src/main.c
    Core/Src/system_stm32h7xx.c
    Core/Src/stm32h7xx_hal_msp.c
    Core/Src/stm32h7xx_it.c
    Core/Src/stm32h7xx_hal_timebase_tim.c
    Core/Src/fdcan.c
    Core/Src/usart.c
    Core/Src/dma.c
    Core/Src/gpio.c
    Core/Src/freertos.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
    Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    Middlewares/Third_Party/FreeRTOS/Source/list.c
    Middlewares/Third_Party/FreeRTOS/Source/queue.c
    Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    Middlewares/Third_Party/FreeRTOS/Source/timers.c
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    ${FREERTOS_PORT_DIR}/port.c
    Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    BSP/bsp_dwt.c
    BSP/bsp_fdcan.c
    BSP/bsp_usart.c
    MOUDLE/algorithm/controller.c
    MOUDLE/daemon/daemon.c
    MOUDLE/motor/dji_motor.c
    MOUDLE/remote/remote.c
    APP/elevate_control.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    Core/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Include
    Drivers/CMSIS/Device/ST/STM32H7xx/Include
    Drivers/CMSIS/RTOS2/Include
    Middlewares/Third_Party/FreeRTOS/Source/include
    Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    ${FREERTOS_PORT_DIR}
    BSP
    MOUDLE
    MOUDLE/algorithm
    MOUDLE/daemon
    MOUDLE/motor
    MOUDLE/remote
    APP
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32H743xx
)

target_link_libraries(${PROJECT_NAME} m)
