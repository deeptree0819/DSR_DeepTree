cmake_minimum_required(VERSION 3.8)
project(pick_place_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------- 의존 패키지 ----------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# ---------- 인터페이스 생성 ----------
rosidl_generate_interfaces(${PROJECT_NAME}
  # 토픽 메시지
  "msg/TaskStatus.msg"
  "msg/GripperState.msg"

  # 서비스
  "srv/SetGripper.srv"
  "srv/GetTaskList.srv"

  # 액션
  "action/PickAndPlace.action"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()