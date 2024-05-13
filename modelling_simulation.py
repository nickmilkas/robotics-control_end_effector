import pinocchio as pin
import os
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer
import time
import numpy as np

VISUALIZER = MeshcatVisualizer


def load_franka():
    current_path = os.path.abspath('')
    robot_path = os.path.join(current_path, "robot")

    robot = RobotWrapper.BuildFromURDF(os.path.join(robot_path, "franka.urdf"), package_dirs=robot_path)

    model = robot.model
    data = robot.data

    return robot, model, data


def step_world(q_state, u_state, control_torc, dt, model_r: pin.Model, data_r):

    up_limit = model_r.upperPositionLimit
    down_limit = model_r.lowerPositionLimit
    velocity_limit = model_r.velocityLimit

    print(up_limit)
    print(down_limit)
    print(velocity_limit)
    acceleration = pin.aba(model_r, data_r, q_state, u_state, control_torc)
    u_state_new = u_state + acceleration * dt

    delta_q = u_state_new * dt

    q_state_new = pin.integrate(model_r, q_state, delta_q)

    if np.any(u_state_new > velocity_limit) or np.any(q_state_new < down_limit) or np.any(q_state_new > up_limit):
        u_state_new = np.zeros(7)
        q_state_new = u_state
        print("PROBLEM PROBLEM")

    return q_state_new, u_state_new


def make_motion(model_robot, data_robot, q_pos, u_pos, control_torc, dt, number_of_motions):
    q_list = []
    q_pos_values = q_pos
    u_pos_values = u_pos

    for _ in range(number_of_motions):
        q_pos_new, u_pos_new = step_world(q_pos_values, u_pos_values, control_torc, dt, model_robot, data_robot)
        q_list.append(q_pos_new)

        q_pos_values = q_pos_new
        u_pos_values = u_pos_new

    return q_list


def visualize(robot, q_values):
    robot.setVisualizer(VISUALIZER())
    robot.initViewer(open=True, loadModel=robot)
    robot.loadViewerModel("pinocchio")

    for q_pos in q_values:
        print(q_pos)
        robot.display(q_pos)
        time.sleep(0.03)

    return 0
