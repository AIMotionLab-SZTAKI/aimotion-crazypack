# aimotion-crazypack
Our code for extending the functionalities of Crazyswarm.

## Installation

1. Open a terminal, type `mkdir cfcontrol`
2. Install [Crazyswarm](https://crazyswarm.readthedocs.io/en/latest/installation.html) to cfcontrol
3. Clone `aimotion-crazypack` to cfcontrol
4. Run `setup_crazypack.sh`
5. Add lines to .bashrc: 
```
source /path/to/crazyswarm/ros_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/path/to/crazyswarm/ros_ws/src/crazyswarm/scripts
```
