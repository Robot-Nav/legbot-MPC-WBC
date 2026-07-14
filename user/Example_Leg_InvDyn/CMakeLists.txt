include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
if(EXISTS ${LCM_TYPES_CPP_DIR})
  include_directories(${LCM_TYPES_CPP_DIR})
endif()

add_executable(leg_invdyn_ctrl main.cpp Leg_InvDyn_Controller.cpp)
target_link_libraries(leg_invdyn_ctrl robot)
