from modelling_simulation import *
from task_space_controller import *
from evaluation import *
import numpy as np


def a_part(dt):
    robot_1, model_1, data_1 = load_franka("franka.urdf")

    q_init = np.array([0, 1.6, 1.5, -2.0, 0.0, 3.7, 0.0])

    u_init = np.zeros(7)

    control_t = np.array([0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0])

    # qs = make_motion(model_1, data_1, q_init, u_init, control_t, dt, 300)

    qs = make_motion_without_gravity(model_1, data_1, q_init, u_init, control_t, dt, 300)
    visualize(robot_1, qs)


def b_part(kp, ki, kd, dt):
    robot_2, model_2, data_2 = load_franka("franka.urdf")

    q_init = np.array([1.5, 1.6, 1.5, -2.0, 0.0, 3.7, 0.0])
    q_goal = np.array([1.8, 1.2, 0.0, -2.0, 0.0, 3.7, 0.0])

    _, jacobian = compute_jacobian_end_effector(model_2, data_2, q_init)
    x_e_table = error_table(model_2, data_2, q_goal, q_init)
    fw_robot = controller_update(x_e_table, kp, ki, kd, dt)
    torques = calculate_tor(model_2, data_2, q_init, x_e_table, kp, ki, kd, dt)

    print("Jacobian is ", jacobian)
    print("================================")
    print("Error is ", x_e_table)
    print("================================")
    print("Fw is ", fw_robot)
    print("================================")
    print("Torque are ", torques)


def c_part(kp, ki, kd, dt):
    u_init = np.zeros(7)
    robot_3, model_3, data_3 = load_franka("franka.urdf")
    q_start, q_finish = evaluation_scenarios(3)

    q_pos = make_motion_with_controller(model_3, data_3, q_finish, q_start, u_init, kp, ki, kd, dt, 1000)

    visualize(robot_3, q_pos)


if __name__ == "__main__":
    dt_v = 0.001

    kp_v = 90
    ki_v = 0
    kd_v = 1
    # a_part(dt)
    # b_part(kp_v, ki_v, kd_v, dt_v)
    c_part(kp_v, ki_v, kd_v, dt_v)
