add_executable(test_dual_cdc
  ${CMAKE_CURRENT_LIST_DIR}/src/main.c
)

target_link_libraries(test_dual_cdc PRIVATE
  pico_stdlib
#Comment FreeRTOS ones if you are not using main_freertos.c
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
  usb_serial_debug
)

pico_enable_stdio_usb(test_dual_cdc 0)
pico_enable_stdio_uart(test_dual_cdc 0)

pico_add_extra_outputs(test_dual_cdc)
