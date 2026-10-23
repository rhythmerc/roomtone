cmake_minimum_required(VERSION 3.16)
project(roomtone VERSION 0.1.0 LANGUAGES CXX)

# Roomtone: room light estimation for mixed reality. The library has no
# dependencies; the replay tool and the tests are on when it's the top-level
# project.
option(ROOMTONE_TOOLS "Build roomtone_replay" ${PROJECT_IS_TOP_LEVEL})
option(ROOMTONE_TESTS "Build the tests" ${PROJECT_IS_TOP_LEVEL})

add_library(roomtone STATIC src/room.cpp src/record.cpp)
add_library(roomtone::roomtone ALIAS roomtone)
target_include_directories(roomtone PUBLIC include)
target_compile_features(roomtone PUBLIC cxx_std_17)
set_target_properties(roomtone PROPERTIES POSITION_INDEPENDENT_CODE ON)
if (MSVC)
  target_compile_options(roomtone PRIVATE /W4)
else ()
  target_compile_options(roomtone PRIVATE -Wall -Wextra)
endif ()

if (ROOMTONE_TOOLS)
  add_executable(roomtone_replay tools/roomtone_replay.cpp)
  target_link_libraries(roomtone_replay PRIVATE roomtone)
endif ()

if (ROOMTONE_TESTS)
  enable_testing()
  add_executable(roomtone_test_room tests/test_room.cpp)
  target_link_libraries(roomtone_test_room PRIVATE roomtone)
  add_test(NAME room COMMAND roomtone_test_room)
endif ()
