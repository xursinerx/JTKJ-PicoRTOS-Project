# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

# Define app name once
set(APP_NAME jtkj_example)

project(${APP_NAME} C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# FREERTOS: include FreeRTOS Kernel libraries
include(FreeRTOS_Kernel_import.cmake)


# Configuration files should go in the include path. Make them available to all libraries at least for FreeRTOS
# Other libraries would use target_include_directories.
# FREERTOS:
# set(FREERTOS_CONFIG_DIR "${CMAKE_SOURCE_DIR}/config")
#include_directories(${FREERTOS_CONFIG_DIR})

# Option 2. Only for FreeRTOS target. 
target_include_directories(FreeRTOS-Kernel INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/config
  )
  
  
  
  # Add librabries. The TKJHAT does not use libraries from FreeRTOS
  add_subdirectory(libs/TKJHAT)
  # Support for usb serial communication
  add_subdirectory(libs/usb-cdc)
  # Add examples
  add_subdirectory(examples/test_cdc)
  add_subdirectory(examples/test_microphone)


  
  # Add executable. Default name is the project name, version 0.1
  add_executable(${APP_NAME}
      src/main.c
      )

  # Links. Add all libraries that application is using. It must at least use the pico_stdlib and the FreeRTOS-Kernel-Heap4
  # FREERTOS: Note, you should pick the FreeRTOS library that suits you best:
  #
  # FreeRTOS-Kernel-Heap1 thru FreeRTOS-Kernel_Heap4
  # or
  # FreeRTOS-Kernel-Static
  target_link_libraries(${APP_NAME}
    pico_stdlib
    FreeRTOS-Kernel-Heap4
    TKJHAT_SDK
    cfg-usbcdc
)


if (PICO_CYW43_SUPPORTED)
    target_link_libraries(${APP_NAME} pico_cyw43_arch_none)
endif()

#Support for printf in usb
pico_enable_stdio_usb(${APP_NAME} 1)
pico_enable_stdio_uart(${APP_NAME} 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(${APP_NAME})

# add url via pico_set_program_url
