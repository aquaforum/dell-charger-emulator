cmake_minimum_required(VERSION 3.5)

project(dell LANGUAGES C)

add_executable(dell main.c crc8.c crc8.h)

add_definitions("-mmcu=atmega328p")
add_definitions("-DF_CPU=32000000")
add_definitions("-fshort-enums")

target_link_options(dell PUBLIC "-mmcu=atmega328p")

if(CMAKE_BUILD_TYPE MATCHES Release)
   set(CMAKE_C_FLAGS_RELEASE "-Os")
   set(CMAKE_CXX_FLAGS_RELEASE "-Os")
endif(CMAKE_BUILD_TYPE MATCHES Release)

set(AVR_UPLOADTOOL avrdude)
