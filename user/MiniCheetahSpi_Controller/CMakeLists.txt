include_directories(${PROJECT_SOURCE_DIR})
include_directories("../../robot/include")
include_directories("../../common/include/")
if(EXISTS ${LCM_TYPES_CPP_DIR})
  include_directories(${LCM_TYPES_CPP_DIR})
endif()

add_executable(mcspi_ctrl main.cpp MiniCheetahSpi_Controller.cpp)
target_link_libraries(mcspi_ctrl robot biomimetics dynacore_param_handler qpOASES)
