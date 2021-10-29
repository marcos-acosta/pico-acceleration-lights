add_executable(mpu6050
        mpu6050.c
        )

# Pull in our (to be renamed) simple get you started dependencies
target_link_libraries(mpu6050 pico_stdlib hardware_i2c)

# enable usb output, disable uart output
pico_enable_stdio_usb(mpu6050 1)
pico_enable_stdio_uart(mpu6050 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(mpu6050)
