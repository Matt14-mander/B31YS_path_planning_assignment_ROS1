cmake_minimum_required(VERSION 2.8.3)
project(rosbot_navigation)

find_package(catkin REQUIRED)

catkin_package(
)

catkin_install_python(PROGRAMS
  scripts/bfs_planner_grid.py
  scripts/path_follower.py
  scripts/visualizer.py
  scripts/dijkstra_planner_grid.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY maps
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
