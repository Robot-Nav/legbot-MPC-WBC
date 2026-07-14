cmake_minimum_required(VERSION 3.5)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)
set(CMAKE_DISABLE_SOURCE_CHANGES  ON)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(SEND_ERROR "In-source builds are not allowed.")
endif ()

set(CMAKE_COLOR_MAKEFILE   ON)
#execute_process(COMMAND ../scripts/make_types.sh)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#set(CMAKE_VERBOSE_MAKEFILE ON)

find_path(EIGEN3_INCLUDE_DIR Eigen/Core
  PATHS /usr/include/eigen3 /usr/local/include/eigen3)
find_path(LCM_INCLUDE_DIR lcm/lcm-cpp.hpp
  PATHS /usr/include /usr/local/include /usr/local/include/lcm)
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  PATHS "$ENV{HOME}/.mujoco/mujoco-3.3.6/include" /usr/local/include /usr/include)
find_library(MUJOCO_LIBRARY mujoco
  PATHS "$ENV{HOME}/.mujoco/mujoco-3.3.6/lib" /usr/local/lib /usr/lib)
set(LCM_TYPES_CPP_DIR ${PROJECT_SOURCE_DIR}/lcm-types/cpp)

if(NOT EIGEN3_INCLUDE_DIR)
  message(FATAL_ERROR "Eigen3 headers were not found. Install libeigen3-dev or set EIGEN3_INCLUDE_DIR.")
endif()

if(NOT LCM_INCLUDE_DIR)
  message(WARNING "LCM headers were not found. Targets using LCM need liblcm-dev or an equivalent LCM install.")
else()
  include_directories(${LCM_INCLUDE_DIR})
  if(EXISTS ${LCM_INCLUDE_DIR}/lcm)
    include_directories(${LCM_INCLUDE_DIR}/lcm)
  endif()
endif()

if(NOT EXISTS ${LCM_TYPES_CPP_DIR})
  message(WARNING "LCM generated C++ headers were not found at ${LCM_TYPES_CPP_DIR}. Run scripts/make_types.sh after installing lcm-gen.")
endif()

if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
  message(STATUS "MuJoCo found: ${MUJOCO_LIBRARY}")
else()
  message(WARNING "MuJoCo C headers/library were not found. The mujoco_legbot_sim target will be disabled.")
endif()

option(MINI_CHEETAH_BUILD "use compiler flags for mini cheetah computer" OFF)
set(BUILD_TYPE_RELEASE TRUE)

option(NO_SIM "Do not build simulator" OFF)

set(CHEETAH_ARCH_FLAGS "-march=native")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(CHEETAH_ARCH_FLAGS "")
  message(STATUS "ARM64 build detected: disabling -march=native for portable OrangePi runtime")
endif()

if(MINI_CHEETAH_BUILD)
  SET (THIS_COM "../" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)
  set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb -Wall \
  -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
  -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
  -Wshadow -Wsign-promo -Werror")
  set(CMAKE_C_FLAGS "-O3  -ggdb -std=gnu99 -I.")
  message("**** Mini-Cheetah build enabled ****")
else(MINI_CHEETAH_BUILD)
  SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_BINARY_DIR}/Configuration.h)

  if(CMAKE_SYSTEM_NAME MATCHES Linux)
    set(CMAKE_CXX_FLAGS "-O3 -no-pie ${CHEETAH_ARCH_FLAGS} -ggdb -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    -Wshadow -Wsign-promo -Werror -Wno-array-bounds")
  elseif(APPLE)
    set(CMAKE_CXX_FLAGS "-O3 -march=native -ggdb -Wall \
    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
    -Wshadow -Wsign-promo")
    include_directories("/usr/local/include/")   # lcm includes
  endif()

  set(CMAKE_C_FLAGS "-O3  -ggdb  ${CHEETAH_ARCH_FLAGS} -std=gnu99 -I.")
  message("**** Mini-Cheetah build disabled ****")
endif(MINI_CHEETAH_BUILD)

set(CMAKE_CXX_STANDARD 17)

# Pinocchio 动力学库（QPWBC 依赖）
find_package(pinocchio REQUIRED)
include_directories(${pinocchio_INCLUDE_DIRS})

#find_package(lcm)

add_subdirectory(robot)
add_subdirectory(third-party)
add_subdirectory(common)

if(NO_SIM)

else(NO_SIM)
  add_subdirectory(sim)
endif()

add_subdirectory(user)
add_subdirectory(rc_test)
if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
  add_subdirectory(mujoco_bridge)
endif()

add_executable(legbot_model_check tools/legbot_model_check.cpp)
target_include_directories(legbot_model_check PRIVATE
  ${PROJECT_SOURCE_DIR}/common/include
  ${LCM_TYPES_CPP_DIR}
  ${CMAKE_BINARY_DIR})
target_link_libraries(legbot_model_check biomimetics)
