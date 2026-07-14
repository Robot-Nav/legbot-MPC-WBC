# 统一 QP-WBC 库：Pinocchio 动力学 + OSQP 求解
add_library(QPWBC SHARED
  QPWBC.cpp
  PinocchioDynamics.cpp
)
target_link_libraries(QPWBC osqp ${pinocchio_LIBRARIES})
target_include_directories(QPWBC PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
