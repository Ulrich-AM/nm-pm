cmake_minimum_required(VERSION 3.20)
project(nm_pm_blood_simulation LANGUAGES CXX)

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

add_executable(blood_phase0
    phases/phase0/main.cpp
)
target_include_directories(blood_phase0 PRIVATE phases/phase0)
target_link_libraries(blood_phase0 PRIVATE raylib)

add_executable(blood_phase0_tests
    phases/phase0/test_phase0.cpp
)
target_include_directories(blood_phase0_tests PRIVATE phases/phase0)

add_executable(blood_phase1
    phases/phase1/main.cpp
)
target_include_directories(blood_phase1 PRIVATE phases/phase0 phases/phase1)
target_link_libraries(blood_phase1 PRIVATE raylib)

add_executable(blood_phase1_tests
    phases/phase1/test_phase1.cpp
)
target_include_directories(blood_phase1_tests PRIVATE phases/phase1)

add_executable(blood_phase2
    phases/phase2/main.cpp
)
target_include_directories(blood_phase2 PRIVATE phases/phase0 phases/phase2)
target_link_libraries(blood_phase2 PRIVATE raylib)

add_executable(blood_phase2_tests
    phases/phase2/test_phase2.cpp
)
target_include_directories(blood_phase2_tests PRIVATE phases/phase0 phases/phase2)

add_executable(blood_phase3
    phases/phase3/main.cpp
)
target_include_directories(blood_phase3 PRIVATE phases/phase0 phases/phase2 phases/phase3)
target_link_libraries(blood_phase3 PRIVATE raylib)

add_executable(blood_phase3_tests
    phases/phase3/test_phase3.cpp
)
target_include_directories(blood_phase3_tests PRIVATE phases/phase0 phases/phase2 phases/phase3)

add_executable(blood_phase4
    phases/phase4/main.cpp
)
target_include_directories(blood_phase4 PRIVATE phases/phase0 phases/phase2 phases/phase3 phases/phase4)
target_link_libraries(blood_phase4 PRIVATE raylib)

add_executable(blood_phase4_tests
    phases/phase4/test_phase4.cpp
)
target_include_directories(blood_phase4_tests PRIVATE phases/phase0 phases/phase2 phases/phase3 phases/phase4)

add_executable(blood_phase5
    phases/phase5/main.cpp
)
target_include_directories(blood_phase5 PRIVATE phases/phase0 phases/phase2 phases/phase3 phases/phase4 phases/phase5)
target_link_libraries(blood_phase5 PRIVATE raylib)

add_executable(blood_phase5_tests
    phases/phase5/test_phase5.cpp
)
target_include_directories(blood_phase5_tests PRIVATE phases/phase0 phases/phase2 phases/phase3 phases/phase4 phases/phase5)

add_executable(blood_phase6
    phases/phase6/main.cpp
)
target_include_directories(blood_phase6 PRIVATE phases/phase0 phases/phase2 phases/phase3 phases/phase4 phases/phase5 phases/phase6)
target_link_libraries(blood_phase6 PRIVATE raylib)

add_executable(blood_phase6_tests
    phases/phase6/test_phase6.cpp
)
target_include_directories(blood_phase6_tests PRIVATE phases/phase0 phases/phase2 phases/phase3 phases/phase4 phases/phase5 phases/phase6)

add_executable(blood_phase7
    phases/phase7/main.cpp
)
target_include_directories(blood_phase7 PRIVATE phases/phase0 phases/phase2 phases/phase3 phases/phase4 phases/phase5 phases/phase6 phases/phase7)
target_link_libraries(blood_phase7 PRIVATE raylib)

add_executable(blood_phase7_tests
    phases/phase7/test_phase7.cpp
)
target_include_directories(blood_phase7_tests PRIVATE phases/phase0 phases/phase2 phases/phase3 phases/phase4 phases/phase5 phases/phase6 phases/phase7)

if (MSVC)
    target_compile_options(blood_phase0 PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase0_tests PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase1 PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase1_tests PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase2 PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase2_tests PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase3 PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase3_tests PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase4 PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase4_tests PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase5 PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase5_tests PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase6 PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase6_tests PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase7 PRIVATE /W4 /permissive-)
    target_compile_options(blood_phase7_tests PRIVATE /W4 /permissive-)
else()
    target_compile_options(blood_phase0 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase0_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase1 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase1_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase2 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase2_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase3 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase3_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase4 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase4_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase5 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase5_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase6 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase6_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase7 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(blood_phase7_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if (BUILD_TESTING)
    add_test(NAME blood_phase0_tests COMMAND blood_phase0_tests)
    add_test(NAME blood_phase1_tests COMMAND blood_phase1_tests)
    add_test(NAME blood_phase2_tests COMMAND blood_phase2_tests)
    add_test(NAME blood_phase3_tests COMMAND blood_phase3_tests)
    add_test(NAME blood_phase4_tests COMMAND blood_phase4_tests)
    add_test(NAME blood_phase5_tests COMMAND blood_phase5_tests)
    add_test(NAME blood_phase6_tests COMMAND blood_phase6_tests)
    add_test(NAME blood_phase7_tests COMMAND blood_phase7_tests)
endif()
