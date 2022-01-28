add_executable(accelerationLights)

# enable usb output, disable uart output
pico_enable_stdio_usb(accelerationLights 1)
pico_enable_stdio_uart(accelerationLights 0)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(accelerationLights ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

target_sources(accelerationLights PRIVATE accelerationLights.c)

target_link_libraries(accelerationLights PRIVATE pico_stdlib hardware_pio hardware_i2c hardware_adc)
pico_add_extra_outputs(accelerationLights)