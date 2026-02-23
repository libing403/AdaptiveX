
add_subdirectory(coppeliasim)
add_library(motor_hal STATIC "")
set(CMAKE_CXX_STANDARD 20)
target_sources(motor_hal 
PRIVATE
motorDrv.cpp
PUBLIC
${CMAKE_CURRENT_LIST_DIR}/motorDrv.h
${CMAKE_CURRENT_LIST_DIR}/coppeliasim/RemoteAPIClient.h
)
target_include_directories(motor_hal PUBLIC 
${CMAKE_CURRENT_LIST_DIR}/coppeliasim
${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(motor_hal PUBLIC RemoteAPIClient)