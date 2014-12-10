cmake_minimum_required(VERSION 2.8.3)
project(thruster_ident_driver)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
controller_interface
controller_manager
hardware_interface
pluginlib
geometry_msgs
control_msgs
#forward_command_controller
control_toolbox
realtime_tools
urdf

)
find_package(Boost REQUIRED COMPONENTS thread system)

include_directories(include ${catkin_INCLUDE_DIRS} SYSTEM ${Boost_INCLUDE_DIR})
# Declare catkin package
catkin_package(
CATKIN_DEPENDS
controller_interface
controller_manager
geometry_msgs
control_msgs
control_toolbox
realtime_tools
urdf
forward_command_controller
pluginlib
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}

src/thruster_ident_driver.cpp

)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(thruster_ident_hw_loop src/thruster_ident_hw_loop.cpp)
target_link_libraries(thruster_ident_hw_loop ${catkin_LIBRARIES}  ${roscpp_LIBRARIES} ${Boost_LIBRARIES} )

# Install
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES 	thruster_ident_driver.xml
				thruster_ident_param.yaml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


