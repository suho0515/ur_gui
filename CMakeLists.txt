cmake_minimum_required(VERSION 3.0.2)
project(ur_gui)

find_package(catkin REQUIRED COMPONENTS
    rospy
    )

catkin_package(
    CATKIN_DEPENDS rospy
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

#add_executable(ur_gui_node src/ur_gui_node.py)
#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS src/ur_gui_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
