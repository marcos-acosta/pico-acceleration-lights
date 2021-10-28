add_executable(accelerationLights
        accelerationLights.c
        )

# Pull in our (to be renamed) simple get you started dependencies
target_link_libraries(accelerationLights pico_stdlib hardware_i2c)

# enable usb output, disable uart output
pico_enable_stdio_usb(accelerationLights 1)
pico_enable_stdio_uart(accelerationLights 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(accelerationLights)
