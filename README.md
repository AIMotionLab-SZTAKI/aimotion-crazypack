# aimotion-crazypack
Our code for extending the functionalities of Crazyswarm

## Installation

1. `$ mkdir cfcontrol`
2. Install [Crazyswarm](https://crazyswarm.readthedocs.io/en/latest/installation.html) to cfcontrol
3. Clone `aimotion-crazypack` to cfcontrol
4. Run setup_crazypack.sh

	`cd ../crazyswarm/ros_ws/src
	ln -s ../../../aimotion-crazypack/aimotion_crazypack .
	cd ..
	catkin_make
	cd ../../aimotion-crazypack
	git submodule init
	git submodule update`
