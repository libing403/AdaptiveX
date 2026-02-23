
# Add a library with the above sources
add_library(ulog  STATIC ulog.c ulog.h)

target_include_directories(ulog PUBLIC ${CMAKE_CURRENT_LIST_DIR} )
target_link_directories(ulog PUBLIC ${CMAKE_CURRENT_LIST_DIR})
# message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
# install(TARGETS ulog
#     ARCHIVE DESTINATION ulog/
#     LIBRARY DESTINATION ulog/
#     RUNTIME DESTINATION ulog/
# )
# install(FILES ulog.h DESTINATION ulog/)
