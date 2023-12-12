# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Include build functions from Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)


add_compile_options(-include "${CMAKE_CURRENT_LIST_DIR}/pico-motor-controller-lib/lib/include/includemyPicolib_config.h")

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(PAIND C CXX ASM)

# Creates a pico-sdk subdirectory in our project for the libraries
pico_sdk_init()


# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
    main.c  
    platform.c  
)

add_subdirectory(./pico-motor-controller-lib/lib)

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_adc   
    hardware_dma 
    pico_multicore
    hardware_pio
    pico_motor_controller_lib
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

