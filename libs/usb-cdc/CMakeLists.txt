#Include in target_link_libraries 
add_library(cfg-usbcdc INTERFACE)

target_sources(cfg-usbcdc INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/src/usb_descriptors.c
)
target_include_directories(cfg-usbcdc INTERFACE
${CMAKE_CURRENT_LIST_DIR}/config
)
#target_compile_definitions(cfg-usbcdc INTERFACE
#  TUSB_CONFIG_FILE="\"${CMAKE_CURRENT_LIST_DIR}/config/tusb_config.h\""
#)

target_link_libraries(cfg-usbcdc INTERFACE  
                pico_unique_id 
                tinyusb_board 
                tinyusb_device)
