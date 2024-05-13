from modelling_simulation import *
import numpy as np


def a_part():
    robot_r, model_r, data_r = load_franka()
    dt = 0.001

    q_init = np.array([0.0, 1.2, 2.0, -1.57, 0.0, 1.57, 0.0])

    u_init = np.zeros(7)

    control_t = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    qs = make_motion(model_r, data_r, q_init, u_init, control_t, dt, 300)

    visualize(robot_r, qs)


if __name__ == "__main__":
    a_part()
