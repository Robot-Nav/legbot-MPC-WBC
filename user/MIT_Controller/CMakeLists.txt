include_directories("./")
include_directories("./Controllers")
include_directories("../../robot/include")
include_directories("../../common/include/")
include_directories("../../common/FootstepPlanner")
include_directories("../../third-party/")
include_directories(SYSTEM "../../third-party/qpOASES/include")
include_directories("../../third-party/ParamHandler")

if(EXISTS ${LCM_TYPES_CPP_DIR})
  include_directories(${LCM_TYPES_CPP_DIR})
endif()
if(LCM_INCLUDE_DIR)
  include_directories(${LCM_INCLUDE_DIR})         # lcm includes
endif()
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

file(GLOB_RECURSE sources
"./*.cpp"
"FSM_States/*.cpp" 
"Controllers/BalanceController/*.cpp" 
"Controllers/convexMPC/*.cpp")

add_subdirectory(Controllers/WBC)
add_subdirectory(Controllers/WBC_Ctrl)

add_executable(mit_ctrl ${sources} MIT_Controller.cpp main.cpp)
target_link_libraries(mit_ctrl robot biomimetics)
target_link_libraries(mit_ctrl dynacore_param_handler qpOASES)
target_link_libraries(mit_ctrl osqp)
target_link_libraries(mit_ctrl WBC_Ctrl)

find_package(unitree_sdk2 QUIET
  PATHS /opt/unitree_robotics/lib/cmake/unitree_sdk2)
if(TARGET unitree_sdk2)
  set(legbot_bridge_sources)
  foreach(src ${sources})
    get_filename_component(src_name ${src} NAME)
    if(NOT src_name STREQUAL "main.cpp")
      list(APPEND legbot_bridge_sources ${src})
    endif()
  endforeach()
  function(add_legbot_real_bridge_target target_name bridge_source)
    add_executable(${target_name}
      ${legbot_bridge_sources}
      ${PROJECT_SOURCE_DIR}/${bridge_source})
    target_include_directories(${target_name} PRIVATE
      ${PROJECT_SOURCE_DIR}/user/MIT_Controller
      ${PROJECT_SOURCE_DIR}/robot/include
      ${PROJECT_SOURCE_DIR}/common/include
      ${PROJECT_SOURCE_DIR}/common/FootstepPlanner
      ${PROJECT_SOURCE_DIR}/third-party
      ${PROJECT_SOURCE_DIR}/third-party/ParamHandler
      ${CMAKE_BINARY_DIR})
    target_link_libraries(${target_name} robot biomimetics)
    target_link_libraries(${target_name} dynacore_param_handler qpOASES)
    target_link_libraries(${target_name} osqp)
    target_link_libraries(${target_name} WBC_Ctrl)
    target_link_libraries(${target_name} unitree_sdk2)
  endfunction()

  add_legbot_real_bridge_target(legbot_real_bridge_dryrun
    tools/legbot_real_bridge_dryrun.cpp)
  add_legbot_real_bridge_target(legbot_real_bridge_gait0_minimal
    tools/legbot_real_bridge_gait0_minimal.cpp)
  find_library(FMT_LIBRARY fmt)
  if(FMT_LIBRARY)
    target_link_libraries(legbot_real_bridge_dryrun ${FMT_LIBRARY})
    target_link_libraries(legbot_real_bridge_gait0_minimal ${FMT_LIBRARY})
  endif()
else()
  message(WARNING "unitree_sdk2 was not found; legbot real bridge targets are disabled.")
endif()
