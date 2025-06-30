cmake_minimum_required(VERSION 3.0.2)
project(lane_follower)

################################################################################
# 1) 패키지 의존성
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_generation            # 나중에 커스텀 메시지를 추가할 수도 있으니 포함
)

################################################################################
# 2) 메시지/서비스(추가 시) 설정
#    └ 현재는 없지만, msg·srv 파일을 만들면 아래 블록만 주석 해제
################################################################################
# add_message_files(
#   FILES
# )
#
# add_service_files(
#   FILES
# )
#
# generate_messages(
#   DEPENDENCIES std_msgs geometry_msgs
# )

################################################################################
# 3) catkin 패키지화
################################################################################
catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs cv_bridge message_runtime
)

################################################################################
# 4) 설치 대상
################################################################################
## 파이썬 노드
catkin_install_python(
  PROGRAMS
    scripts/lane_detector.py
    scripts/path_planner.py
    scripts/pure_pursuit_ctrl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## launch 파일
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# 5) include 디렉터리 (필요 시)
################################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)
