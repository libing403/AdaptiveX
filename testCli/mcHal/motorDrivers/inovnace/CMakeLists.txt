
add_library(motorDriver STATIC motorDriver.c)
target_include_directories(motorDriver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
