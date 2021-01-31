set(WORK_SPACE_PATH /home/sineva-sys-master/fusion_ws/src/01-lidar-odometry/src/lidar_localization)
configure_file (
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/global_defination/global_defination.h.in
  ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/global_defination/global_defination.h)
include_directories(${PROJECT_BINARY_DIR}/include)