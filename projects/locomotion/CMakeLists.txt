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

add_executable(locomotion_phase1
    phases/phase1/main.cpp
)
target_include_directories(locomotion_phase1 PRIVATE phases/phase0 phases/phase1)
target_link_libraries(locomotion_phase1 PRIVATE raylib)

add_executable(locomotion_phase1_tests
    phases/phase1/test_phase1.cpp
)
target_include_directories(locomotion_phase1_tests PRIVATE phases/phase0 phases/phase1)

add_executable(locomotion_phase2
    phases/phase2/main.cpp
)
target_include_directories(locomotion_phase2 PRIVATE phases/phase0 phases/phase1 phases/phase2)
target_link_libraries(locomotion_phase2 PRIVATE raylib)

add_executable(locomotion_phase2_tests
    phases/phase2/test_phase2.cpp
)
target_include_directories(locomotion_phase2_tests PRIVATE phases/phase0 phases/phase1 phases/phase2)

add_executable(locomotion_phase3
    phases/phase3/main.cpp
)
target_include_directories(locomotion_phase3 PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3)
target_link_libraries(locomotion_phase3 PRIVATE raylib)

add_executable(locomotion_phase3_tests
    phases/phase3/test_phase3.cpp
)
target_include_directories(locomotion_phase3_tests PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3)

add_executable(locomotion_phase4
    phases/phase4/main.cpp
)
target_include_directories(locomotion_phase4 PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3 phases/phase4)
target_link_libraries(locomotion_phase4 PRIVATE raylib)

add_executable(locomotion_phase4_tests
    phases/phase4/test_phase4.cpp
)
target_include_directories(locomotion_phase4_tests PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3 phases/phase4)

add_executable(locomotion_phase5
    phases/phase5/main.cpp
)
target_include_directories(locomotion_phase5 PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3 phases/phase4 phases/phase5)
target_link_libraries(locomotion_phase5 PRIVATE raylib)

add_executable(locomotion_phase5_tests
    phases/phase5/test_phase5.cpp
)
target_include_directories(locomotion_phase5_tests PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3 phases/phase4 phases/phase5)

add_executable(locomotion_phase6
    phases/phase6/main.cpp
)
target_include_directories(locomotion_phase6 PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3 phases/phase4 phases/phase5 phases/phase6)
target_link_libraries(locomotion_phase6 PRIVATE raylib)

add_executable(locomotion_phase6_tests
    phases/phase6/test_phase6.cpp
)
target_include_directories(locomotion_phase6_tests PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3 phases/phase4 phases/phase5 phases/phase6)

add_executable(locomotion_phase7
    phases/phase7/main.cpp
)
target_include_directories(locomotion_phase7 PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3 phases/phase4 phases/phase5 phases/phase6 phases/phase7)
target_link_libraries(locomotion_phase7 PRIVATE raylib)

add_executable(locomotion_phase7_tests
    phases/phase7/test_phase7.cpp
)
target_include_directories(locomotion_phase7_tests PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3 phases/phase4 phases/phase5 phases/phase6 phases/phase7)

add_executable(locomotion_phase8
    phases/phase8/main.cpp
)
target_include_directories(locomotion_phase8 PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3 phases/phase4 phases/phase5 phases/phase6 phases/phase7 phases/phase8)
target_link_libraries(locomotion_phase8 PRIVATE raylib)

add_executable(locomotion_phase8_tests
    phases/phase8/test_phase8.cpp
)
target_include_directories(locomotion_phase8_tests PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3 phases/phase4 phases/phase5 phases/phase6 phases/phase7 phases/phase8)

add_executable(locomotion_phase9
    phases/phase9/main.cpp
)
target_include_directories(locomotion_phase9 PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3 phases/phase4 phases/phase5 phases/phase6 phases/phase7 phases/phase8 phases/phase9)
target_link_libraries(locomotion_phase9 PRIVATE raylib)

# Compact build identifier for harness (release tag + short SHA when available).
set(_locomotion_build_tag "dev")
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/RELEASE_REQUEST")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/RELEASE_REQUEST" _locomotion_build_tag)
    string(STRIP "${_locomotion_build_tag}" _locomotion_build_tag)
endif()
set(_locomotion_git_sha "local")
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../.."
    OUTPUT_VARIABLE _locomotion_git_sha
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
target_compile_definitions(locomotion_phase9 PRIVATE
    "LOCOMOTION_BUILD_TAG=\"${_locomotion_build_tag}\""
    "LOCOMOTION_GIT_SHA=\"${_locomotion_git_sha}\""
)

add_executable(locomotion_phase9_tests
    phases/phase9/test_phase9.cpp
)
target_include_directories(locomotion_phase9_tests PRIVATE phases/phase0 phases/phase1 phases/phase2 phases/phase3 phases/phase4 phases/phase5 phases/phase6 phases/phase7 phases/phase8 phases/phase9)

if (MSVC)
    target_compile_options(locomotion_phase0 PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase0_tests PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase1 PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase1_tests PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase2 PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase2_tests PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase3 PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase3_tests PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase4 PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase4_tests PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase5 PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase5_tests PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase6 PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase6_tests PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase7 PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase7_tests PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase8 PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase8_tests PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase9 PRIVATE /W4 /permissive-)
    target_compile_options(locomotion_phase9_tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(locomotion_phase0 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase0_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase1 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase1_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase2 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase2_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase3 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase3_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase4 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase4_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase5 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase5_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase6 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase6_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase7 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase7_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase8 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase8_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase9 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(locomotion_phase9_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if (BUILD_TESTING)
    add_test(NAME locomotion_phase0_tests COMMAND locomotion_phase0_tests)
    add_test(NAME locomotion_phase1_tests COMMAND locomotion_phase1_tests)
    add_test(NAME locomotion_phase2_tests COMMAND locomotion_phase2_tests)
    add_test(NAME locomotion_phase3_tests COMMAND locomotion_phase3_tests)
    add_test(NAME locomotion_phase4_tests COMMAND locomotion_phase4_tests)
    add_test(NAME locomotion_phase5_tests COMMAND locomotion_phase5_tests)
    add_test(NAME locomotion_phase6_tests COMMAND locomotion_phase6_tests)
    add_test(NAME locomotion_phase7_tests COMMAND locomotion_phase7_tests)
    add_test(NAME locomotion_phase8_tests COMMAND locomotion_phase8_tests)
    add_test(NAME locomotion_phase9_tests COMMAND locomotion_phase9_tests)
endif()
