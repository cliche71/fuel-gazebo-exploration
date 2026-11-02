cmake_minimum_required(VERSION 3.0.2)
project(fuel_gazebo_official_baseline)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/sensor_bridge.py scripts/command_bridge.py scripts/wait_ready.py
  scripts/make_official_robot.py scripts/check_official.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/command_math.py scripts/sensor_math.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch worlds reference
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md CAMERA.md CHANGES.md LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
