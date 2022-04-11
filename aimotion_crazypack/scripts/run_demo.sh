rosrun crazyflie_tools reboot --uri "radio://0/90/2M/E7E7E7E704"
rosrun crazyflie_tools reboot --uri "radio://0/90/2M/E7E7E7E705"
sleep 3
roslaunch aimotion_crazypack hover_swarm.launch &
ros_id=$!
sleep 3
./repeat_dijkstra.sh
sleep 1
kill $ros_id
sleep 3
rosrun crazyflie_tools reboot --uri "radio://0/90/2M/E7E7E7E704"
rosrun crazyflie_tools reboot --uri "radio://0/90/2M/E7E7E7E705"
sleep 3
roslaunch aimotion_crazypack hover_swarm.launch &
ros_id=$!
sleep 3
python3 twodrone_flip.py
sleep 1
kill $ros_id
