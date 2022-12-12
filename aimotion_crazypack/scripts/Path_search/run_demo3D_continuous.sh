rosrun crazyflie_tools reboot --uri "radio://0/80/2M/E7E7E7E703"
rosrun crazyflie_tools reboot --uri "radio://0/80/2M/E7E7E7E705"
rosrun crazyflie_tools reboot --uri "radio://0/90/2M/E7E7E7E706"
rosrun crazyflie_tools reboot --uri "radio://0/90/2M/E7E7E7E710"
sleep 3
roslaunch aimotion_crazypack hover_swarm.launch &
ros_id=$!
sleep 2
gnome-terminal --tab -- /usr/bin/env python3.8 socket_send_pos.py
sleep 1
gnome-terminal --tab -- /usr/bin/env python3.8 measure.py
sleep 1
gnome-terminal --tab -- /usr/bin/env python3.8 planner_runner3D.py
sleep 1
gnome-terminal --tab -- /usr/bin/env python3.8 dijkstra_flight3D_continuous.py --max_drones=4
sleep 1000
kill $ros_id
