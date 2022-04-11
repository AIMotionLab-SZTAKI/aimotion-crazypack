rosrun crazyflie_tools reboot --uri "radio://0/90/2M/E7E7E7E704"
rosrun crazyflie_tools reboot --uri "radio://0/90/2M/E7E7E7E705"
sleep 3
roslaunch aimotion_crazypack hover_swarm.launch 
