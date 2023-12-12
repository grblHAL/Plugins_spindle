add_library(spindle INTERFACE)

target_sources(spindle INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/modbus_rtu.c
 ${CMAKE_CURRENT_LIST_DIR}/select.c
 ${CMAKE_CURRENT_LIST_DIR}/onoff.c
 ${CMAKE_CURRENT_LIST_DIR}/pwm_clone.c
 ${CMAKE_CURRENT_LIST_DIR}/stepper.c
 ${CMAKE_CURRENT_LIST_DIR}/vfd/spindle.c
 ${CMAKE_CURRENT_LIST_DIR}/vfd/huanyang.c
 ${CMAKE_CURRENT_LIST_DIR}/vfd/huanyang2.c
 ${CMAKE_CURRENT_LIST_DIR}/vfd/h100.c
 ${CMAKE_CURRENT_LIST_DIR}/vfd/modvfd.c
 ${CMAKE_CURRENT_LIST_DIR}/vfd/gs20.c
 ${CMAKE_CURRENT_LIST_DIR}/vfd/yl620.c
 ${CMAKE_CURRENT_LIST_DIR}/vfd/nowforever.c
)

target_include_directories(spindle INTERFACE ${CMAKE_CURRENT_LIST_DIR})
