cd ../crazyswarm/ros_ws/src
ln -s ../../../aimotion-crazypack/aimotion_crazypack .
cd ..
catkin_make
cd ../../aimotion-crazypack
git submodule init
git submodule update
