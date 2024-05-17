import numpy as np
import pinocchio as pin
from modelling_simulation import load_franka, step_world, visualize

# 1. Find errors for orientation and translation
# 2. Create a controller to make motion

previous_error = np.zeros((6, 1))
integral = np.zeros((6, 1))


def for_k_all(model_r, data_r, q_pos):
    pin.forwardKinematics(model_r, data_r, q_pos)
    pin.updateFramePlacements(model_r, data_r)


def compute_jacobian_end_effector(model_r, data_r, q_pos):
    for_k_all(model_r, data_r, q_pos)
    frame_id = model_r.getFrameId("panda_ee")
    t_w = data_r.oMf[frame_id].copy()
    jacob = pin.computeFrameJacobian(model_r, data_r, q_pos, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    return t_w, jacob


def controller_update(x_e, kp, ki, kd, dt):
    global previous_error
    global integral

    x_e = x_e.reshape(-1, 1)

    derivative = (x_e - previous_error) / dt
    integral += x_e * dt

    p = kp * x_e
    i = ki * integral
    c = kd * derivative

    f_w = p + i + c
    previous_error = x_e

    return f_w


def calculate_tor(model, data, q_posit, x_e, kp, ki, kd, dt):
    _, jacobian = compute_jacobian_end_effector(model, data, q_posit)

    fw = controller_update(x_e, kp, ki, kd, dt)
    print(jacobian)
    print(fw)
    return jacobian @ fw


def calculate_orientation_error(model, data, q_desired, q_current):
    t_wd, _ = compute_jacobian_end_effector(model, data, q_desired)
    t_wb, _ = compute_jacobian_end_effector(model, data, q_current)
    r_wd = t_wd.rotation
    r_wb = t_wb.rotation
    error = pin.log3(r_wd @ r_wb.T)

    return error.reshape((3, 1))



def calculate_translation_error(model, data, q_desired, q_current):
    t_wd, _ = compute_jacobian_end_effector(model, data, q_desired)
    t_wb, _ = compute_jacobian_end_effector(model, data, q_current)

    print(t_wb)
    print(t_wb)
    error = t_wd.translation - t_wb.translation
    return error.reshape((3, 1))


def error_table(model, data, q_desired, q_current):
    trans_error = calculate_translation_error(model, data, q_desired, q_current)
    orient_error = calculate_orientation_error(model, data, q_desired, q_current)

    # Concatenate translation and orientation errors into a single 6x1 array
    r_e = np.concatenate((trans_error, orient_error))
    return r_e


def make_motion_with_controller(model, data, q_desired, q_start, u_start, dt, kp, ki, kd, number_of_motions):
    q_list = []
    q_position = q_start
    u_value = u_start

    for _ in range(number_of_motions):
        error = error_table(model, data, q_desired, q_start)
        tor_values = calculate_tor(model, data, q_desired, error, kp, ki, kd, dt)
        q_pos_new, u_pos_new = step_world(q_position, u_value, tor_values, dt, model, data)
        q_list.append(q_pos_new)

        q_position = q_pos_new
        u_value = u_pos_new

    return q_list
