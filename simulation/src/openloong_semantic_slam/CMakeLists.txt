cmake_minimum_required(VERSION 3.0.2)
project(openloong_semantic_slam)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  nodes/vlp16_to_laserscan.py
  nodes/slam_path_node.py
  nodes/semantic_mapper.py
  nodes/semantic_inference.py
  nodes/semantic_mapper_core.py
  nodes/semantic_mapper_init.py
  nodes/semantic_mapper_pipeline.py
  nodes/semantic_mapper_fusion.py
  nodes/demo_driver.py
  nodes/runtime_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
