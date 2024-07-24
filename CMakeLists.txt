cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
set(CMAKE_TOOLCHAIN_FILE cmake/toolchain-arm-none-eabi-gcc.cmake)
project(loracsp VERSION 0.1.0 LANGUAGES CXX C ASM)

include(cmake/stm32wlex.cmake)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/rtos/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

aux_source_directory(src _SOURCES)
aux_source_directory(rtos _SOURCES)

add_executable(loracsp ${_SOURCES})

target_link_libraries(loracsp stm32wlex)
set_property(TARGET loracsp PROPERTY CXX_STANDARD 17)
stm32_add_linker_script(loracsp PRIVATE flash.ld)
stm32_add_flash_targets(loracsp)
