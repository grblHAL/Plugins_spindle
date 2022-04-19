add_library(spindle INTERFACE)

target_sources(spindle INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/modbus.c
 ${CMAKE_CURRENT_LIST_DIR}/select.c
 ${CMAKE_CURRENT_LIST_DIR}/huanyang.c
)

target_include_directories(spindle INTERFACE ${CMAKE_CURRENT_LIST_DIR})
