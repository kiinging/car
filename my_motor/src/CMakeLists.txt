add_executable(my_motor
        main.cpp
        GPIOInputMgr.cpp
        GPIOObserver.cpp     
        MotorMgr.cpp
        )
pico_set_program_name(my_motor "{NAME}")
pico_set_program_version(my_motor "0.1")
        

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(my_motor 
	pico_stdlib 
	hardware_pwm
    hardware_i2c
	)



# enable usb output, disable uart output
pico_enable_stdio_usb(my_motor 0)
pico_enable_stdio_uart(my_motor 1)



# Add the standard include files to the build
target_include_directories(my_motor PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)



pico_add_extra_outputs(my_motor)

