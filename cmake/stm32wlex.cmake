include(FetchContent)
FetchContent_Declare(libopencm3
  GIT_REPOSITORY git@github.com:neovc/stm32wl
  GIT_TAG main
)
FetchContent_MakeAvailable(libopencm3)
FetchContent_GetProperties(libopencm3)

# create a target to build libopencm3 -- only for the target we need
add_custom_target(libopencm3 make TARGETS=stm32/wl WORKING_DIRECTORY ${libopencm3_SOURCE_DIR})

# Create a specific CPU target with the appropriate options etc
add_library(stm32wlex STATIC IMPORTED)
set_property(TARGET stm32wlex PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${libopencm3_SOURCE_DIR}/include)
set_property(TARGET stm32wlex PROPERTY IMPORTED_LOCATION ${libopencm3_SOURCE_DIR}/lib/libopencm3_stm32wl.a)
add_dependencies(stm32wlex libopencm3)
target_link_directories(stm32wlex INTERFACE ${libopencm3_SOURCE_DIR}/lib)

target_compile_definitions(stm32wlex INTERFACE -DSTM32WL)

set(COMPILE_OPTIONS 
  -fno-common
  -DSTM32WL
  -Os
  -mcpu=cortex-m4
  -mthumb
  -fno-exceptions
  -ffunction-sections
  -fdata-sections
  -fsingle-precision-constant
  -fno-strict-aliasing
  -DUSE_CMAKE
  -Wall
)
set(LINK_OPTIONS
  --static
  -nostartfiles
  -Wl,--gc-sections
  -mthumb
  -mcpu=cortex-m4
  --specs=nano.specs
  -Wl,--start-group
  -lc
  -lgcc
  -lnosys
  -Wl,--end-group
  -lm
)
target_compile_options(stm32wlex INTERFACE ${COMPILE_OPTIONS})
target_link_options(stm32wlex INTERFACE ${LINK_OPTIONS})


function(stm32_add_flash_targets TARGET)
    add_custom_target(${TARGET}-stlink-flash
      bash -c "openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg \
                -f /usr/share/openocd/scripts/target/stm32wlx.cfg \
                -c 'reset_config none; program ${TARGET}.elf verify reset exit'"
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      DEPENDS ${TARGET}
      VERBATIM
    )
endfunction()
