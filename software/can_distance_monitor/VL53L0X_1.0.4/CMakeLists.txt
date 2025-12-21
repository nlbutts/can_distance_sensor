cmake_minimum_required(VERSION 3.22)

# VL53L0X Library
project(vl53l0x_lib C)

# Get STM32 include paths and defines from parent if available
if(NOT DEFINED MX_Include_Dirs)
    # Define manually if not inherited from parent
    set(MX_Include_Dirs
        ${CMAKE_SOURCE_DIR}/Core/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32G0xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32G0xx_HAL_Driver/Inc/Legacy
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32G0xx/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    )
endif()

if(NOT DEFINED MX_Defines_Syms)
    set(MX_Defines_Syms
        CMSIS_device_header=<stm32g0xx.h>
        USE_HAL_DRIVER
        STM32G0B1xx
    )
endif()

# Add the VL53L0X API core source files
add_library(vl53l0x STATIC
    Api/core/src/vl53l0x_api.c
    Api/core/src/vl53l0x_api_calibration.c
    Api/core/src/vl53l0x_api_core.c
    Api/core/src/vl53l0x_api_ranging.c
    Api/core/src/vl53l0x_api_strings.c
    ${CMAKE_SOURCE_DIR}/Core/Src/vl53l0x_stm32_platform.c
)

# Include directories for VL53L0X
target_include_directories(vl53l0x PUBLIC
    Api/core/inc
    Api/platform/inc
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${MX_Include_Dirs}
)

# Apply STM32 compile definitions
target_compile_definitions(vl53l0x PRIVATE
    ${MX_Defines_Syms}
)

# Set C standard
set_target_properties(vl53l0x PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)
