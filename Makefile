.PHONY clean build debug all

clean:
	echo -e "########## CLEANING TURTLEBOT4_WS ##########"
        rm -rf src/build/ src/log/ src/install/
        echo -e "####### CLEANING WORKSPACE COMPLETE ########"
build:
        echo -e "########## BUILDING TURTLEBOT4_WS #########"
        cd $(PWD)/turtlebot4_ws
        colcon build --symlink-install
        cd ..
        echo -e "####### TURTLEBOT4_WS BUILD COMPLETE ######"
debug:
        echo -e "#### BUILDING TURTLEBOT4_WS WITH DEBUG ####"
        cd $(PWD)/turtlebot4_ws
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
        cd ..
        echo -e "#### TURTLEBOT4_WS DEBUG BUILD COMPLETE ####"
all:

