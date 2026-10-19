cmake_minimum_required(VERSION 3.20)
project(view_dependent_sprite_projection LANGUAGES CXX)

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

# Phase 8 animation import bridge. Assimp is kept out of the earlier harnesses
# and is linked only to the body-animation executable.
find_package(assimp 6.0.5 QUIET)
if (NOT assimp_FOUND)
    set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
    set(ASSIMP_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
    set(ASSIMP_INSTALL OFF CACHE BOOL "" FORCE)
    set(ASSIMP_WARNINGS_AS_ERRORS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        assimp
        GIT_REPOSITORY https://github.com/assimp/assimp.git
        GIT_TAG v6.0.5
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(assimp)
endif()

if (TARGET assimp::assimp)
    set(VDSPR_ASSIMP_TARGET assimp::assimp)
else()
    set(VDSPR_ASSIMP_TARGET assimp)
endif()

add_executable(vdspr_phase5
    src/main.cpp
)
target_include_directories(vdspr_phase5 PRIVATE src)
target_link_libraries(vdspr_phase5 PRIVATE raylib)

add_executable(vdspr_phase6
    src/articulation_lab.cpp
)
target_include_directories(vdspr_phase6 PRIVATE src)
target_link_libraries(vdspr_phase6 PRIVATE raylib)

add_executable(vdspr_phase7
    src/chain_lab.cpp
)
target_include_directories(vdspr_phase7 PRIVATE src)
target_link_libraries(vdspr_phase7 PRIVATE raylib)

add_executable(vdspr_phase8
    src/body_lab.cpp
)
target_include_directories(vdspr_phase8 PRIVATE src)
target_link_libraries(vdspr_phase8 PRIVATE raylib ${VDSPR_ASSIMP_TARGET})

if (MSVC)
    target_compile_options(vdspr_phase5 PRIVATE /W4 /permissive-)
    target_compile_options(vdspr_phase6 PRIVATE /W4 /permissive-)
    target_compile_options(vdspr_phase7 PRIVATE /W4 /permissive-)
    target_compile_options(vdspr_phase8 PRIVATE /W4 /permissive-)
else()
    target_compile_options(vdspr_phase5 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(vdspr_phase6 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(vdspr_phase7 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(vdspr_phase8 PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(vdspr_math_tests
    tests/test_math.cpp
)
target_include_directories(vdspr_math_tests PRIVATE src)

include(CTest)
if (BUILD_TESTING)
    add_test(NAME vdspr_math_tests COMMAND vdspr_math_tests)
endif()
