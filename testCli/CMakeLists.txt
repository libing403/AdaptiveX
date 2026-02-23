cmake_minimum_required(VERSION 3.22 FATAL_ERROR)
project(testCli )
set(LANGUAGES C Cxx)

#add_subdirectory(mcHal/motorDrivers)
add_executable(testCli "")
target_include_directories(testCli 
PUBLIC
${CMAKE_CURRENT_LIST_DIR}
${CMAKE_CURRENT_LIST_DIR}/../../adxlib/adxlib_windows/flexCore/include
${CMAKE_CURRENT_LIST_DIR}/mcHal/motorDrivers
)
target_sources(testCli
    PRIVATE
    testFile.cpp
    testFun.cpp
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/../../adxlib/adxlib_windows/flexCore/include/cmdNo.h
    ${CMAKE_CURRENT_LIST_DIR}/../../adxlib/adxlib_windows/flexCore/include/flexCore_if.h
)
target_link_directories(testCli
 PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/../../adxlib/adxlib_windows/flexCore/lib
 
  )
target_link_libraries(testCli
  PUBLIC
    winmm
    flexCore
    #motor_hal
)
install(TARGETS testCli
    RUNTIME DESTINATION debug_windows
)

add_executable(logger "logger.cpp")
target_include_directories(testCli 
PUBLIC
${CMAKE_CURRENT_LIST_DIR}
)
install(TARGETS logger
    RUNTIME DESTINATION debug_windows
)