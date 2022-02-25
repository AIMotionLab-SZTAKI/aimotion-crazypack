"""Script demonstrating the flip algorithm.

The simulation is run by a `CtrlAviary` environment.
The stabilization control is given by the PID implementation in `DSLPIDControl`.

Example
-------
In a terminal, run as:

    $ python flip.py

Notes
-----


"""
import sys
sys.path.append('../crazyflie-demo-simulation/')
import time
import argparse
import numpy as np
import pybullet as p


from gym_pybullet_drones.envs.BaseAviary import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.VisionAviary import VisionAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.Flip import Flip
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

if __name__ == "__main__":

    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Crazyflie flip script using CtrlAviary or VisionAviary and DSLPIDControl')
    parser.add_argument('--drone',              default="cf2x",     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--num_drones',         default=1,          type=int,           help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--physics',            default="pyb",      type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--vision',             default=False,      type=str2bool,      help='Whether to use VisionAviary (default: False)', metavar='')
    parser.add_argument('--gui',                default=True,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=False,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--plot',               default=True,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--user_debug_gui',     default=False,      type=str2bool,      help='Whether to add debug lines and parameters to the GUI (default: False)', metavar='')
    parser.add_argument('--aggregate',          default=False,      type=str2bool,      help='Whether to aggregate physics steps (default: False)', metavar='')
    parser.add_argument('--obstacles',          default=False,       type=str2bool,      help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=240,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--control_freq_hz',    default=120,         type=int,           help='Control frequency in Hz (default: 48)', metavar='')
    parser.add_argument('--duration_sec',       default=2,          type=int,           help='Duration of the simulation in seconds (default: 5)', metavar='')
    parser.add_argument('--slowmo',             default=True,          type=str2bool,           help='Wether to simulate in slow motion', metavar='')
    ARGS = parser.parse_args()

    #### Initialize the simulation #############################
    over = False
    H = 1
    H_STEP = .05
    W_STEP = .5
    INIT_XYZS = np.array([[(i%2)*W_STEP, np.floor(i/2)*W_STEP, H] for i in range(ARGS.num_drones)])
    AGGR_PHY_STEPS = int(ARGS.simulation_freq_hz/ARGS.control_freq_hz) if ARGS.aggregate else 1

    #### Create the environment with or without video capture ##
    if ARGS.vision:
        env = VisionAviary(drone_model=ARGS.drone,
                           num_drones=ARGS.num_drones,
                           initial_xyzs=INIT_XYZS,
                           physics=ARGS.physics,
                           neighbourhood_radius=10,
                           freq=ARGS.simulation_freq_hz,
                           aggregate_phy_steps=AGGR_PHY_STEPS,
                           gui=ARGS.gui,
                           record=ARGS.record_video,
                           obstacles=ARGS.obstacles
                           )
    else:
        env = CtrlAviary(drone_model=ARGS.drone,
                         num_drones=ARGS.num_drones,
                         initial_xyzs=INIT_XYZS,
                         physics=ARGS.physics,
                         neighbourhood_radius=10,
                         freq=ARGS.simulation_freq_hz,
                         aggregate_phy_steps=AGGR_PHY_STEPS,
                         gui=ARGS.gui,
                         record=ARGS.record_video,
                         obstacles=ARGS.obstacles,
                         user_debug_gui=ARGS.user_debug_gui
                         )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    #### Initialize trajectory ######################
    PERIOD = 2
    NUM_WP = ARGS.control_freq_hz*PERIOD # number of work points
    TARGET_POS = np.zeros((NUM_WP, 3))   # target positions
    for i in range(NUM_WP):
        TARGET_POS[i, :] = INIT_XYZS[0, 0], INIT_XYZS[0, 1], INIT_XYZS[0, 2]
    wp_counters = np.array([int((i*NUM_WP/6)%NUM_WP) for i in range(ARGS.num_drones)])

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=int(ARGS.simulation_freq_hz/AGGR_PHY_STEPS),
                    num_drones=ARGS.num_drones
                    )

    #### Initialize the controllers ############################
    ctrl = [DSLPIDControl(env) for i in range(ARGS.num_drones)]

    if ARGS.drone == DroneModel.CF2P:
        flip = Flip("cf2p")
        sections = [(0.5259, [-42.346, 0, 0], 0.1),
                    (0.37948, [297.8296, 0, 0], 0.23),
                    (0.174888, [0, 0, 0], 0.138),
                    (0.379484, [-297.8296, 0, 0], 0.2),
                    (0.502654, [59.2655, 0, 0], 0.075)]
    elif ARGS.drone == DroneModel.CF2X:
        flip = Flip("cf2x")
        sections = [(0.49, [42.346, 0, 0], 0.12),
                    (0.37948, [-297.8296, 0, 0], 0.28),
                    (0.174888, [0, 0, 0], 0.16),
                    (0.379484, [297.8296, 0, 0], 0.2),
                    (0.502654, [-59.2655, 0, 0], 0.075)]
    else:
        print('Unexpected drone type, quitting')
        sys.exit(100)
    T = flip.get_durations(sections)
    T = T*env.SIM_FREQ + ARGS.simulation_freq_hz/10

    p.resetDebugVisualizerCamera(cameraDistance=1,
                                         cameraYaw=90,
                                         cameraPitch=-30,
                                         cameraTargetPosition=[0, 0, 1],
                                         physicsClientId=env.CLIENT
                                         )

    #### Run the simulation ####################################
    CTRL_EVERY_N_STEPS = int(np.floor(env.SIM_FREQ/ARGS.control_freq_hz))
    action = {str(i): np.array([0, 0, 0, 0]) for i in range(ARGS.num_drones)}
    START = time.time()
    for i in range(0, int(ARGS.duration_sec*env.SIM_FREQ), AGGR_PHY_STEPS):

        #### Step the simulation ###################################
        obs, reward, done, info = env.step(action)

        #### Compute control at the desired frequency ##############
        if i%CTRL_EVERY_N_STEPS == 0:
            #### Compute control for the current way point #############
            for j in range(ARGS.num_drones):
                if i < ARGS.simulation_freq_hz/10:
                    action[str(j)], _, _ = ctrl[j].computeControlFromState(control_timestep=CTRL_EVERY_N_STEPS*env.TIMESTEP,
                                                                           state=obs[str(j)]["state"],
                                                                           target_pos=INIT_XYZS[j, :]  # np.hstack([TARGET_POS[wp_counters[j], 0:3]])
                                                                           )
                elif not over:
                    possibleT = [k for k, x in enumerate(T) if i/x < 1]
                    if len(possibleT) != 0:
                        num_sec = np.min(possibleT)  # decide in which section we are
                        action[str(j)] = flip.compute_control_from_section(sections[num_sec], obs[str(j)]["state"][9:12])
                    else:
                        over = True  # the flipping maneuvre is over
                        print(['Flipping is over at t=', float(i) / env.SIM_FREQ, ', position ',
                               obs[str(j)]["state"][0:3], ', attitude ', p.getEulerFromQuaternion(obs[str(j)]["state"][3:7])])
                else:
                    action[str(j)], _, _ = ctrl[j].computeControlFromState(
                            control_timestep=CTRL_EVERY_N_STEPS * env.TIMESTEP,
                            state=obs[str(j)]["state"],
                            target_pos=INIT_XYZS[j, :],
                            target_rpy=[0, 0, 0]
                            )

            #### Go to the next way point and loop #####################
            for j in range(ARGS.num_drones):
                wp_counters[j] = wp_counters[j] + 1 if wp_counters[j] < (NUM_WP-1) else 0

        #### Log the simulation ####################################
        for j in range(ARGS.num_drones):
            logger.log(drone=j,
                       timestamp=i/env.SIM_FREQ,
                       state=obs[str(j)]["state"],
                       control=np.hstack([TARGET_POS[wp_counters[j], 0:2], H+j*H_STEP, np.zeros(9)])
                       )
        if ARGS.slowmo: time.sleep(0.0005)
        #### Printout ##############################################
        if i%env.SIM_FREQ == 0:
            env.render()
            #### Print matrices with the images captured by each drone #
            if ARGS.vision:
                for j in range(ARGS.num_drones):
                    print(obs[str(j)]["rgb"].shape, np.average(obs[str(j)]["rgb"]),
                          obs[str(j)]["dep"].shape, np.average(obs[str(j)]["dep"]),
                          obs[str(j)]["seg"].shape, np.average(obs[str(j)]["seg"])
                          )
        #### Sync the simulation ###################################
        if ARGS.gui:
            sync(i, START, env.TIMESTEP)

    #### Close the environment #################################
    env.close()

    #### Save the simulation results ###########################
    logger.save()

    #### Plot the simulation results ###########################
    if ARGS.plot:
        logger.plot()


