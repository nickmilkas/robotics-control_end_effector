from modelling_simulation import *
from task_space_controller import *
import numpy as np


def a_part():
    robot_r, model_r, data_r = load_franka("franka.urdf")
    dt = 0.001

    q_init = np.array([1.5, 1.6, 1.5, -2.0, 0.0, 3.7, 0.0])

    u_init = np.zeros(7)

    control_t = np.array([0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0])

    qs = make_motion(model_r, data_r, q_init, u_init, control_t, dt, 300)

    # qs = make_motion_without_gravity(model_r, data_r, q_init, u_init, control_t, dt, 300)
    visualize(robot_r, qs)


def b_part():
    robot_2, model_rob, data_rob = load_franka("franka.urdf")
    dt = 0.001

    q_init = np.array([1.5, 1.6, 1.5, -2.0, 0.0, 3.7, 0.0])

    u_init = np.zeros(7)

    q_goal = np.array([1.5, 1.6, 1.5, -2.0, 0.0, 3.7, 0.0])
    kp = 100
    ki = 0.001
    kd = 0.001

    qs = make_motion_with_controller(model_rob, data_rob, q_goal, q_init, u_init, dt, kp, ki, kd, 300)
    visualize(robot_2, qs)


if __name__ == "__main__":
    b_part()
