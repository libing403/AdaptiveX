cmake_minimum_required(VERSION 3.5)
project(testsim)
set(LANGUAGES CXX C)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_CXX_STANDARD 17)

#添加依赖库信息
include_directories("D:/Program Files/vcpkg/installed/x64-windows/include")
set(ZeroMQ_DIR "D:/Program Files/vcpkg/installed/x64-windows/share/zeromq")
find_package(ZeroMQ REQUIRED)
if(NOT ZeroMQ_FOUND)
    message(FATAL_ERROR "ZeroMQ was not found.")
endif()
set(cppzmq_DIR "D:/Program Files/vcpkg/installed/x64-windows/share/cppzmq")
find_package(cppzmq REQUIRED)
if(NOT cppzmq_FOUND)
    message(FATAL_ERROR "cppzmq was not found.")
endif()


if(NOT COPPELIASIM_INCLUDE_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(COPPELIASIM_INCLUDE_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/include)
    else()
        message(FATAL_ERROR "Environment variable COPPELIASIM_ROOT_DIR is not set")
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${COPPELIASIM_INCLUDE_DIR}/cmake)

find_package(CoppeliaSim 4.1.0.0 REQUIRED)
# find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(GENERATE OFF CACHE BOOL "Generate wrappers for objects and methods (requires CoppeliaSim to be running).")
set(GENERATE_INCLUDE_OBJECTS "sim,simIK" CACHE STRING "Include only given objects (e.g.: \"sim,simIK\") or leave blank to include all objects in wrapper code generation.")
set(GENERATE_EXCLUDE_OBJECTS "" CACHE STRING "Exclude given objects (e.g.: \"simUI,simIM\") from being generated. Note: option GENERATE_INCLUDE_OBJECTS, if different from \"\", has precedence over this.")
set(GENERATE_EXCLUDE_METHODS "sim.test,sim.auxFunc,sim.getStringSignal,sim.getInt32Signal,sim.getFloatSignal" CACHE STRING "Exclude given methods (e.g.: \"sim.test,simUI.foo\") from being generated.")


add_library(RemoteAPIClient STATIC RemoteAPIClient.cpp)
target_compile_definitions(RemoteAPIClient PUBLIC -DSIM_REMOTEAPICLIENT_OBJECTS)
target_include_directories(RemoteAPIClient PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/jsoncons/include)
target_link_libraries(RemoteAPIClient PUBLIC cppzmq)


add_library(remote_api_c_wrapper "")

target_sources(remote_api_c_wrapper PRIVATE
remote_api_c_wrapper.cpp
)


target_include_directories(remote_api_c_wrapper PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/jsoncons/include)
target_include_directories(remote_api_c_wrapper PUBLIC 
.
${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_link_libraries(remote_api_c_wrapper PRIVATE RemoteAPIClient)

add_executable(example example.c)
target_link_libraries(example PRIVATE remote_api_c_wrapper)


# add_library(motorDriver STATIC "")
# target_sources(motorDriver PRIVATE sim_driver.c)
# target_include_directories(motorDriver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
# target_link_libraries(motorDriver PRIVATE remote_api_c_wrapper)
