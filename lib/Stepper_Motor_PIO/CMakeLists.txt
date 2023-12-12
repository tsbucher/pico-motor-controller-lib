
add_library(pio_stepper INTERFACE)

pico_generate_pio_header(pio_stepper ${CMAKE_CURRENT_LIST_DIR}/stepper.pio)
pico_generate_pio_header(pio_stepper ${CMAKE_CURRENT_LIST_DIR}/pacer.pio)
pico_generate_pio_header(pio_stepper ${CMAKE_CURRENT_LIST_DIR}/counter.pio)

target_include_directories(pio_stepper INTERFACE
  ../include
)

target_link_libraries(
    pio_stepper INTERFACE
    pico_stdlib
    hardware_gpio
    hardware_dma 
    pico_multicore
    hardware_pio
)

target_sources(pio_stepper INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/stepper_PIO.c
  ${CMAKE_CURRENT_LIST_DIR}/stepper_PIO.h
)

