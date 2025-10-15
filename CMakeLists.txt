cmake_minimum_required(VERSION 3.13)
project(HAL_USBD_CH32 C CXX ASM)

add_library(HAL_USBD_CH32 INTERFACE)

target_sources(HAL_USBD_CH32
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/src/usb_hal.c
)

target_include_directories(HAL_USBD_CH32 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/inc
)