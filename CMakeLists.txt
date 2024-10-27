cmake_minimum_required(VERSION 3.12.2)
project(cpptemplateproject VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(thirdparty/Catch2)
add_subdirectory(thirdparty/fmt)
include_directories(thirdparty/olcPixelGameEngine)

# Main application
add_executable(main
    src/main.cpp # error: Fixme:
)
target_include_directories(main PUBLIC inc)
target_link_libraries(main
    fmt
    X11
    GL
    png
)

# Test application
add_executable(test_main
    test/test_main.cpp
)
target_include_directories(test_main PUBLIC inc)

target_include_directories(test_main PUBLIC ${CATCH_INCLUDE_DIR})
target_link_libraries(test_main
    Catch2::Catch2WithMain
)
