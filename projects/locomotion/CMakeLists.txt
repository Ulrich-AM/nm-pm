cmake_minimum_required(VERSION 3.20)
project(nm_pm_locomotion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

find_package(raylib 6.2 QUIET)
if (NOT raylib_FOUND)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 6.2
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(raylib)
endif()

add_executable(locomotion_phase0
    phases/phase0/main.cpp
)
target_include_directories(locomotion_phase0 PRIVATE phases/phase0)
target_link_libraries(locomotion_phase0 PRIVATE raylib)

add_executable(locomotion_phase0_tests
    phases/phase0/test_phase0.cpp
)
target_include_directories(locomotion_phase0_tests PRIVATE phases/phase0)

if (MSVC)
    target_compile_options(locomotion_phase0 PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase0_tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(locomotion_phase0 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase0_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if (BUILD_TESTING)
    add_test(NAME locomotion_phase0_tests COMMAND locomotion_phase0_tests)
endif()
