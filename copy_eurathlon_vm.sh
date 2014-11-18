#!/bin/bash
echo "I'll copy all ADIS_Interface files over to the BBB"

rsync -avzh ./src/*.cpp 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/thruster_ident_driver/src/
rsync -avzh ./include/thruster_ident_driver/*.hpp 	eurathlon_vm:/home/euratlhon/uwesub_msc/src/thruster_ident_driver/include/thruster_ident_driver/
rsync -avzh CMakeLists.txt 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/thruster_ident_driver/
rsync -avzh *.xml 								eurathlon_vm:/home/euratlhon/uwesub_msc/src/thruster_ident_driver/
rsync -avzh *.launch	 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/thruster_ident_driver/
rsync -avzh *.yaml		 						eurathlon_vm:/home/euratlhon/uwesub_msc/src/thruster_ident_driver/
rsync -avzh *.md								eurathlon_vm:/home/euratlhon/uwesub_msc/src/thruster_ident_driver/

echo "All done, Good Success!"