cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(dvp2spi_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(dvp_capture)
# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(dvp_capture ${CMAKE_CURRENT_LIST_DIR}/dvp_in.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})
target_sources(dvp_capture PRIVATE dvp_capture.c)
pico_enable_stdio_usb(dvp_capture 0)
pico_enable_stdio_uart(dvp_capture 1)
pico_set_binary_type(dvp_capture default)
pico_add_extra_outputs(dvp_capture)
target_link_libraries(dvp_capture PRIVATE pico_stdlib hardware_dma hardware_pio)

add_executable(dvp2spi)
# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(dvp2spi ${CMAKE_CURRENT_LIST_DIR}/dvp_in.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})
target_sources(dvp2spi PRIVATE dvp2spi.c)
pico_enable_stdio_usb(dvp2spi 0)
pico_enable_stdio_uart(dvp2spi 1)
pico_set_binary_type(dvp2spi default)
pico_add_extra_outputs(dvp2spi)
target_link_libraries(dvp2spi PRIVATE pico_stdlib hardware_dma hardware_pio hardware_spi)

add_executable(dvp_process)
# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(dvp_process ${CMAKE_CURRENT_LIST_DIR}/dvp_in.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})
target_sources(dvp_process PRIVATE dvp_process.c)
pico_enable_stdio_usb(dvp_process 0)
pico_enable_stdio_uart(dvp_process 1)
pico_set_binary_type(dvp_process default)
pico_add_extra_outputs(dvp_process)
target_link_libraries(dvp_process PRIVATE pico_stdlib pico_multicore hardware_dma hardware_pio)

add_executable(dvp2proc2spi)
# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(dvp2proc2spi ${CMAKE_CURRENT_LIST_DIR}/dvp_in.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})
target_sources(dvp2proc2spi PRIVATE dvp2proc2spi.c)
pico_enable_stdio_usb(dvp2proc2spi 0)
pico_enable_stdio_uart(dvp2proc2spi 1)
pico_set_binary_type(dvp2proc2spi default)
pico_add_extra_outputs(dvp2proc2spi)
target_link_libraries(dvp2proc2spi PRIVATE pico_stdlib pico_multicore hardware_dma hardware_pio hardware_spi)
