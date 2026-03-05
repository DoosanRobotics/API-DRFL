cmake_minimum_required(VERSION 3.16)
project(drfl_windows_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DRFL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
set(DRFL_INCLUDE_DIR "${DRFL_ROOT}/include")
set(DRFL_LIB_DIR "${DRFL_ROOT}/library/Windows/64bits")

add_executable(drfl_windows_example
    main.cpp
    util.hpp
)

target_include_directories(drfl_windows_example PRIVATE
    "${DRFL_INCLUDE_DIR}"
)

# MinGW uses import libraries (.dll.a) for linking.
target_link_libraries(drfl_windows_example PRIVATE
    "${DRFL_LIB_DIR}/DRFLWin64.dll.a"
)

add_custom_command(TARGET drfl_windows_example POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${DRFL_LIB_DIR}/DRFLWin64.dll"
        "${DRFL_LIB_DIR}/PocoFoundation64.dll"
        "${DRFL_LIB_DIR}/PocoNet64.dll"
        $<TARGET_FILE_DIR:drfl_windows_example>
)
