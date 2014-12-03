#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 							BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/src/
rsync -avzh ./include/thruster_ident_driver/*.hpp 	BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/include/thruster_ident_driver/
rsync -avzh CMakeLists.txt 							BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/
rsync -avzh *.xml 									BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/
rsync -avzh ./launch/*.launch	 					BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/launch
rsync -avzh *.yaml		 							BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/
rsync -avzh *.md									BBB_local:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/

echo "All done, Good Success!"