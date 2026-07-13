cmake_minimum_required(VERSION 3.16)
project(minecoaster CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG 5.5
)
FetchContent_MakeAvailable(raylib)

add_executable(minecoaster src/main.cpp)
target_link_libraries(minecoaster raylib)

set_target_properties(minecoaster PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

include(CTest)
if(BUILD_TESTING)
  add_test(NAME v1_geometry_issues COMMAND minecoaster --v1issues 64)
  add_test(NAME v1_legacy_audit COMMAND minecoaster --audit 64)
  add_test(NAME v1_element_census COMMAND minecoaster --census 64)
  add_test(NAME v1_launch_pacing COMMAND minecoaster --launchaudit)
  add_test(NAME v1_force_envelope COMMAND minecoaster --forceaudit 64)
  add_test(
    NAME v1_deterministic_export
    COMMAND ${CMAKE_COMMAND}
      -DEXE=$<TARGET_FILE:minecoaster>
      -DOUT_DIR=${CMAKE_CURRENT_BINARY_DIR}
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/V1Determinism.cmake)
endif()
