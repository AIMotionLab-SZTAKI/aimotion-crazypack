sed -i '2,22d' ../launch/crazyflies.yaml
rosrun crazyflie_tools reboot --uri "radio://0/90/2M/E7E7E7E70A"
sleep 5
roslaunch aimotion_crazypack hover_swarm.launch &
ros_id2=$!
sleep 3
python3 flip.py
sleep 200
