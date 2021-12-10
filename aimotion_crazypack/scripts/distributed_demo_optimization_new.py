"""
Main function

Ways to improve:
    - hyperparam optimization
"""

import time
# from group import Group
# from obstacle import Obstacle
from src import *
import matplotlib.pyplot as plt
import numpy as np

import random
import sys

import os
os.system("mkdir log")
os.system("mkdir urdf")
os.system("mkdir csv")
os.system("mkdir figures")
os.system("mkdir video")
os.system("mkdir yaml")





def run_optimizaiton(corners_list, start_position, goal_position, min_iterations, max_iterations, stage):


    def writing_parameters_to_file(iteration_times):
        """
        Writing parameteres to file
        """
        cwd = os.getcwd()
        log_file = open(cwd + "/log/" + "log.txt", "w")
    
        # Writing time required for iteration into file
        log_file.writelines('\n')
        log_file.writelines('Iteration times: \n')
        for i in range(len(iteration_times)):
            log_file.writelines(str(iteration_times[i]) + '\n')
    
        log_file.writelines('Final time: \n')
        log_file.writelines(str(np.sum(iteration_times)))


    targetHeight = 0.8
    # n_iterations = 50
    obstacles = []
    # start_position=[-1.1, 0.5]
    # goal_position=[1.0, -0.5]
    # start_position=[-0.7, 0.4]
    # goal_position=[1.1, -0.4]
    
    plt.close('all')
    seed = random.randrange(sys.maxsize) % 100
    random.Random(seed)
    print("Seed was:", seed)

    
    # Torus
    if len(corners_list) > 0:
        obstacle = Obstacle(ID = 0, corners = corners_list[0], obstacle_type = "torus")
        obstacles += [obstacle]
        obstacle = Obstacle(ID = 1, corners = corners_list[1], obstacle_type = "torus")
        obstacles += [obstacle]
        obstacles[-2].generate_urdf_torus(obstacles[-1].corners)
    

    try:
        1/0
        # Poles read from optitrack
        import rospy
        from geometry_msgs.msg import PoseStamped
        rospy.init_node("optitrack_vrpn_listener", anonymous = False)
        for i in range(10):
            topic = '/vrpn_client_node/object_' + str(i) + '/pose'
            res = rospy.wait_for_message(topic = topic, topic_type = PoseStamped, timeout = 0.1)
            middle_x = res.x
            middle_y = res.y
            
            size = 0.1
            delta = size/2
            corner1 = [middle_x - delta, middle_y - delta]
            corner2 = [middle_x + delta, middle_y - delta]
            corner3 = [middle_x + delta, middle_y + delta]
            corner4 = [middle_x - delta, middle_y + delta]
            corners_list = [corner1] + [corner2] + [corner3] + [corner4]
            
            obstacle = Obstacle(ID = 2 + i, corners = corners_list[0], obstacle_type = "pole")
            obstacles += [obstacle]
            
            
    except:

        # read from csv
        obstacle_positions, vehicle_positions = read_optitrack()

        # !! Make obst 0 the first 2 obstacles!
        key = "obst0"
        torus_middle = [obstacle_positions[key]['x'], -obstacle_positions[key]['z']]
        radious = 0.80 / 2
        length = 2
        width = 0.02
        # width = 0.3
        targetHeight = obstacle_positions[key]['y'] + 0.85 / 2  # 0.85m is the diameter of the hula hoop

        # First create the bottom gate
        corner1 = (np.array(torus_middle) + np.array([0, -radious - length / 2]) + np.array(
            [-width / 2, -length / 2])).tolist()
        corner2 = (np.array(torus_middle) + np.array([0, -radious - length / 2]) + np.array(
            [width / 2, -length / 2])).tolist()
        corner3 = (np.array(torus_middle) + np.array([0, -radious - length / 2]) + np.array(
            [width / 2, length / 2])).tolist()
        corner4 = (np.array(torus_middle) + np.array([0, -radious - length / 2]) + np.array(
            [-width / 2, length / 2])).tolist()
        corners = [corner1, corner2, corner3, corner4]

        obstacle = Obstacle(ID=0, corners=corners, obstacle_type="torus")
        obstacles += [obstacle]

        # Now the second part, the top gate:
        corner1 = (np.array(torus_middle) + np.array([0, radious + length / 2]) + np.array(
            [-width / 2, -length / 2])).tolist()
        corner2 = (np.array(torus_middle) + np.array([0, radious + length / 2]) + np.array(
            [width / 2, -length / 2])).tolist()
        corner3 = (np.array(torus_middle) + np.array([0, radious + length / 2]) + np.array(
            [width / 2, length / 2])).tolist()
        corner4 = (np.array(torus_middle) + np.array([0, radious + length / 2]) + np.array(
            [-width / 2, length / 2])).tolist()
        corners = [corner1, corner2, corner3, corner4]

        obstacle = Obstacle(ID=1, corners=corners, obstacle_type="torus")
        obstacles += [obstacle]
        obstacles[-2].generate_urdf_torus(obstacles[-1].corners)

        for i, key in enumerate(obstacle_positions):

            # This will be our torus
            if key == "obst0":
                pass
            # Rest of the obstacles :)
            else:
                corners = [obstacle_positions[key]['x'], -obstacle_positions[key]['z']]
                obstacle = Obstacle(ID=len(obstacles), corners=corners,
                                    obstacle_type="pole")  # Randomly placed obstacle
                obstacles += [obstacle]

        """        
        # read from csv
        obstacle_positions, vehicle_positions = read_optitrack()
        for i, key in enumerate(obstacle_positions):

            # This will be our torus
            if key == "obst0":
                torus_middle = [obstacle_positions[key]['x'], -obstacle_positions[key]['z']]
                radious = 0.80/2
                length = 2
                width = 0.02
                targetHeight = obstacle_positions[key]['y'] + 0.85/2 # 0.85m is the diameter of the hula hoop

                # First create the bottom gate
                corner1 = (np.array(torus_middle) + np.array([0, -radious-length/2]) + np.array([-width/2, -length/2])).tolist()
                corner2 = (np.array(torus_middle) + np.array([0, -radious-length/2]) + np.array([width/2, -length/2])).tolist()
                corner3 = (np.array(torus_middle) + np.array([0, -radious-length/2]) + np.array([width/2, length/2])).tolist()
                corner4 = (np.array(torus_middle) + np.array([0, -radious-length/2]) + np.array([-width/2, length/2])).tolist()
                corners = [corner1, corner2, corner3, corner4]

                obstacle = Obstacle(ID = 0, corners = corners, obstacle_type = "torus")
                obstacles += [obstacle]

                # Now the second part, the top gate:
                corner1 = (np.array(torus_middle) + np.array([0, radious+length/2]) + np.array([-width/2, -length/2])).tolist()
                corner2 = (np.array(torus_middle) + np.array([0, radious+length/2]) + np.array([width/2, -length/2])).tolist()
                corner3 = (np.array(torus_middle) + np.array([0, radious+length/2]) + np.array([width/2, length/2])).tolist()
                corner4 = (np.array(torus_middle) + np.array([0, radious+length/2]) + np.array([-width/2, length/2])).tolist()
                corners = [corner1, corner2, corner3, corner4]

                obstacle = Obstacle(ID = 1, corners = corners, obstacle_type = "torus")
                obstacles += [obstacle]
                obstacles[-2].generate_urdf_torus(obstacles[-1].corners)

             # Rest of the obstacles :)        
            else:
                corners = [obstacle_positions[key]['x'], -obstacle_positions[key]['z']]
                obstacle = Obstacle(ID = i+3, corners = corners,obstacle_type="pole") # Randomly placed obstacle
                obstacles += [obstacle]
        """

    # Create group
    group = Group(n_vehicles=3, start_position = start_position, goal_position = goal_position, stage = stage, cwd = os.getcwd())
    group.set_group_position(
        position=group.start_position,targetHeight = targetHeight, position_type='initial')
    group.set_group_position(
        position=group.goal_position, targetHeight = targetHeight, position_type='final')
    group.add_obstacles(obstacles)
    group.organise_neighbours()
    
    
    # if plot_environment == True:
    #     group.plot_setup()
    #     n_iterations = 0
    #     # return 0
    # else:

    print("Prepare STARTED")
    group.prepare()
    print("Prepare DONE")
    t_iter = time.time()
    iteration_times = []
    collision_happened = True
    i = 0
    while collision_happened or i < min_iterations:
        group.solve()
    
        # Time-related things
        iteration_times += [time.time() - t_iter]
        print(str(i) + "th iteration time: " +
              str(time.time() - t_iter) + " seconds")
        t_iter = time.time()
    
        # group.plotter(iternum=i, seed=seed)
        collision_happened = group.check_collision()
        
        i = i + 1
        if i > max_iterations:
            collision_happened = group.check_collision(plot_distances = True)
            # group.plot_vehicle_trajectories_gradient()
            group.save_trajectory_to_csv(t_desired = 4.0, t_hover = 0)
            break
        
        if collision_happened == False and i >= min_iterations:
            collision_happened = group.check_collision(plot_distances = True)
            # group.plot_vehicle_trajectories_gradient()
            group.save_trajectory_to_csv(t_desired = 4.0, t_hover = 0)

        if i == 50:
            for j in range(len(group.vehicles)):
                group.vehicles[j].rho = group.vehicles[j].rho / 2
        if i == 100:
            for j in range(len(group.vehicles)):
                group.vehicles[j].rho = group.vehicles[j].rho / 5
        # if i == 150:
        #     for j in range(len(group.vehicles)):
        #         group.vehicles[j].rho = group.vehicles[j].rho / 2

    group.plot_moovie_frames(iternum=i, seed=seed)
    
    writing_parameters_to_file(iteration_times)
    
    # for vehicle in group.vehicles:
    #     for line in vehicle.maze_printable:
            # print(line)
            
    for i in range(len(group.vehicles)):
        print(group.vehicles[i].solution['f'])
        
    return [vehicle.solution['f'].full().reshape(1, -1).tolist()[0][0] for vehicle in group.vehicles], group
    # return group
        

corners_list = []


group_stages = []
min_iterations = 20 # 20
max_iterations = 150 # 300

    
def run_main(command_list):
    corners_list, start_position, goal_position, min_iterations, max_iterations, stage = command_list
    costs, group = run_optimizaiton(corners_list, start_position, goal_position, min_iterations, max_iterations, stage)

    return 0
    
def stage_csv_parser(num_of_stages, num_of_vehicles = 3):
    import csv
    cwd = os.getcwd()
    
    for vehicle in range(num_of_vehicles):
        mode = 'r'
        append_row = []
        for stage in range(num_of_stages):
            with open(cwd + '/csv/' + 'stage_' + str(stage) + '_vehicle' + str(vehicle) + '.csv', mode = mode) as csvfile:
                reader = csv.reader(csvfile)
                for i, row in enumerate(reader):
                    if i != 0:
                        append_row += [row]
        mode = 'w'
        first_line = ['duration', 'x^0', 'x^1', 'x^2', 'x^3', 'x^4', 'x^5', 'x^6', 'x^7', 'y^0', 'y^1', 'y^2', 'y^3', 'y^4', 'y^5', 'y^6', 'y^7', 'z^0', 'z^1', 'z^2', 'z^3', 'z^4', 'z^5', 'z^6', 'z^7', 'yaw^0', 'yaw^1', 'yaw^2', 'yaw^3', 'yaw^4', 'yaw^5', 'yaw^6', 'yaw^7']
        with open(cwd + '/csv/' + 'vehicle' + str(vehicle) + '.csv', mode = mode) as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(first_line)
            for i, row in enumerate(append_row):
                writer.writerow(row)

stage = 0
command_list = []
x_max = 1.1
x_min = -0.85 # or -0.85 for example

y_min_hula_hoop = -0.05
y_max_hula_hoop = 0.05

y_min_audience = -0.35
y_max_audience = 0.35


# hula to audience
command = [[], [x_max, y_min_hula_hoop], [x_min, y_min_audience], min_iterations, max_iterations, stage]
command_list += [command]

# audience to hula
stage = stage + 1
command = [[], [x_min, y_min_audience], [x_max, y_max_hula_hoop], min_iterations, max_iterations, stage]
command_list += [command]

# hula to audience
stage = stage + 1
command = [[], [x_max, y_max_hula_hoop], [x_min, y_max_audience], min_iterations, max_iterations, stage]
command_list += [command]

# audience to hula
stage = stage + 1
command = [[], [x_min, y_max_audience], [x_max, y_min_hula_hoop], min_iterations, max_iterations, stage]
command_list += [command]

# hula to audience
stage = stage + 1
command = [[], [x_max, y_min_hula_hoop], [x_min, y_max_audience], min_iterations, max_iterations, stage]
command_list += [command]

for command in command_list:
    run_main(command)
stage_csv_parser(stage + 1)

"""
from multiprocessing import Pool
if __name__ == '__main__':
    pool = Pool()
    stage = 0
    command_list = []
    # x_shift = 0.0
    # command = [[], [-1.0 + x_shift, -0.35], [1.05 + x_shift, -0.35], min_iterations, max_iterations, stage]
    # command_list += [command]
    # stage = stage + 1
    # command = [[], [1.15 + x_shift, -0.35], [-1.0 + x_shift, 0.35], min_iterations, max_iterations, stage]
    # command_list += [command]
    # stage = stage + 1
    # command = [[], [-1.0 + x_shift, 0.35], [1.15 + x_shift, 0.35], min_iterations, max_iterations, stage]
    # command_list += [command]
    # stage = stage + 1
    # command = [[], [1.15 + x_shift, 0.35], [-1.0 + x_shift, -0.35], min_iterations, max_iterations, stage]
    # command_list += [command]

    # "Normal direction"
    "The 'other' direction LOL"
    x_max = 1.25
    x_min = -1.05
    command = [[], [x_max, -0.35], [-1.05, -0.25], min_iterations, max_iterations, stage]
    command_list += [command]
    stage = stage + 1
    command = [[], [-1.05, -0.25], [x_max, -0.35], min_iterations, max_iterations, stage]
    command_list += [command]
    stage = stage + 1
    command = [[], [x_max, -0.35], [-1.05, 0.25], min_iterations, max_iterations, stage]
    command_list += [command]
    stage = stage + 1
    command = [[], [-1.05, 0.25], [x_max, -0.35], min_iterations, max_iterations, stage]
    command_list += [command]
    stage = stage + 1
    command = [[], [x_max, -0.35], [-1.05, -0.25], min_iterations, max_iterations, stage]
    command_list += [command]
    # stage = stage + 1
    
    
    pool.map(run_main, command_list)
    stage_csv_parser(stage + 1)
    
    
"""
