add_executable(${NAME}
        main.cpp
        GPIOInputMgr.cpp
        GPIOObserver.cpp     
        MotorMgr.cpp
        MotorPID.cpp
        )
pico_set_program_name(${NAME} "{NAME}")
pico_set_program_version(${NAME} "0.1")
                
# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	pico_stdlib 
	hardware_pwm
	)



# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 0)
pico_enable_stdio_uart(${NAME} 1)


# Add the standard include files to the build
target_include_directories(${NAME} PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)


# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})