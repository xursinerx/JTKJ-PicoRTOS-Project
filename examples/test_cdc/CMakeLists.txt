add_executable(test_cdc
  ${CMAKE_CURRENT_LIST_DIR}/src/main.c
)

target_link_libraries(test_cdc PRIVATE
  pico_stdlib
  cfg-usbcdc
)

pico_enable_stdio_usb(test_cdc 1)
pico_enable_stdio_uart(test_cdc 0)

pico_add_extra_outputs(test_cdc)
