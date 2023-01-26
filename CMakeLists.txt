cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(dvp2spi_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(dvp2spi)
# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(dvp2spi ${CMAKE_CURRENT_LIST_DIR}/dvp2spi.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})
target_sources(dvp2spi PRIVATE dvp2spi.c)
pico_enable_stdio_usb(dvp2spi 0)
pico_enable_stdio_uart(dvp2spi 1)
pico_add_extra_outputs(dvp2spi)
target_link_libraries(dvp2spi PRIVATE pico_stdlib hardware_dma hardware_pio hardware_spi)


add_executable(dvp_capture)
# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(dvp_capture ${CMAKE_CURRENT_LIST_DIR}/dvp2spi.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})
target_sources(dvp_capture PRIVATE dvp_capture.c)
pico_enable_stdio_usb(dvp_capture 0)
pico_enable_stdio_uart(dvp_capture 1)
pico_add_extra_outputs(dvp_capture)
target_link_libraries(dvp_capture PRIVATE pico_stdlib hardware_dma hardware_pio)