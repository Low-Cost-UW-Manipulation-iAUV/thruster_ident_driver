#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/src/
rsync -avzh ./include/thruster_ident_driver/*.hpp 	BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/include/thruster_ident_driver/
rsync -avzh CMakeLists.txt 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/
rsync -avzh *.xml 								BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/
rsync -avzh ./launch/*.launch	 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/launch
rsync -avzh *.yaml		 						BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/
rsync -avzh *.md								BBB_BRLNET:/home/ubuntu/uwesub_catkin_workspace/src/thruster_ident_driver/

echo "All done, Good Success!"