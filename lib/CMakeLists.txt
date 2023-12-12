add_library(pico_motor_controller_lib INTERFACE)

#This line is only used if a stepper motor is controlled with the PIO state machines.
#Otherwise it can be commeted out.
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/Stepper_Motor_PIO)

pico_generate_pio_header(${PROJECT_NAME}  ${CMAKE_CURRENT_LIST_DIR}/PIO_Encoder/quadrature_encoder.pio)

target_include_directories(pico_motor_controller_lib
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(pico_motor_controller_lib
  INTERFACE
  #This line is only used if a stepper motor is controlled with the PIO state machines.
  #Otherwise it can be commeted out.
  pio_stepper
)

target_sources(pico_motor_controller_lib
  INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/my_led.c
  ${CMAKE_CURRENT_LIST_DIR}/include/my_led.h
  ${CMAKE_CURRENT_LIST_DIR}/stepper.c
  ${CMAKE_CURRENT_LIST_DIR}/include/stepper.h
  ${CMAKE_CURRENT_LIST_DIR}/dc_motor.c
  ${CMAKE_CURRENT_LIST_DIR}/include/dc_motor.h
  ${CMAKE_CURRENT_LIST_DIR}/button.c
  ${CMAKE_CURRENT_LIST_DIR}/include/button.h
  ${CMAKE_CURRENT_LIST_DIR}/my_adc.c
  ${CMAKE_CURRENT_LIST_DIR}/include/my_adc.h
  ${CMAKE_CURRENT_LIST_DIR}/include/PID_drive.h
  ${CMAKE_CURRENT_LIST_DIR}/PID_drive.c
  ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.c
  ${CMAKE_CURRENT_LIST_DIR}/include/quadrature_encoder.h
  ${CMAKE_CURRENT_LIST_DIR}/include/includemyPicolib.h
  ${CMAKE_CURRENT_LIST_DIR}/include/myhandletype.h
)
