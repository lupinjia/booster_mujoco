cmake_minimum_required(VERSION 3.16)
project(booster_mujoco)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

# ---------------------------------------------------------------
# locate booster_robotics_sdk (override with -DBOOSTER_SDK_DIR=...)
# ---------------------------------------------------------------
set(BOOSTER_SDK_DIR "" CACHE PATH "Path to the booster_robotics_sdk root")
if(NOT BOOSTER_SDK_DIR)
  get_filename_component(_booster_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
  set(BOOSTER_SDK_DIR "${_booster_root}/booster_robotics_sdk")
endif()
message(STATUS "Using booster_robotics_sdk at ${BOOSTER_SDK_DIR}")

set(BOOSTER_SDK_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
if(NOT BOOSTER_SDK_ARCH)
    execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE BOOSTER_SDK_ARCH
        OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
if(BOOSTER_SDK_ARCH MATCHES "^(AMD64|amd64)$")
    set(BOOSTER_SDK_ARCH x86_64)
endif()

set(BOOSTER_SDK_LIBRARY "${BOOSTER_SDK_DIR}/lib/${BOOSTER_SDK_ARCH}/libbooster_robotics_sdk.a")
if(NOT EXISTS "${BOOSTER_SDK_LIBRARY}")
    message(FATAL_ERROR "Booster Robotics SDK library not found: ${BOOSTER_SDK_LIBRARY}")
endif()
message(STATUS "Using booster SDK library: ${BOOSTER_SDK_LIBRARY}")

find_package(mujoco REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(glfw3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/src
  ${BOOSTER_SDK_DIR}/include
  )

file(GLOB SIM_SRC
    src/joystick/joystick.cc
    src/mujoco/*.cc
)

set(SIM_DEPENDENCIES
    pthread
    ${BOOSTER_SDK_LIBRARY}
    mujoco::mujoco
    glfw
    yaml-cpp
    boost_program_options
    fmt::fmt
    ${CMAKE_DL_LIBS}
    rt
)

add_executable(booster_mujoco
  ${SIM_SRC}
  src/main.cc)
target_link_libraries(booster_mujoco
  ${SIM_DEPENDENCIES}
  ${OpenCV_LIBS})

add_executable(test test/test_booster_sdk.cpp)
target_link_libraries(test
  pthread
  ${BOOSTER_SDK_LIBRARY}
  ${CMAKE_DL_LIBS}
  rt)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
