import numpy as np
import pinocchio as pin
from modelling_simulation import step_world

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
    integral += (x_e * dt)

    p = kp * x_e
    i = ki * integral
    c = kd * derivative

    f_w = p + i + c
    previous_error = np.copy(x_e)

    return f_w


def null_space_projection(jacobian):
    identity = np.eye(jacobian.shape[1])
    pseudo_inverse = np.linalg.pinv(jacobian)
    null_space_proj = identity - pseudo_inverse @ jacobian
    return null_space_proj


def compute_task_space_inertia_matrix(model_r, data_r, q_pos, jacobian):
    pin.computeMinverse(model_r, data_r, q_pos)
    mass_matrix_inv = data_r.Minv
    task_inertia_matrix = np.linalg.inv(jacobian @ mass_matrix_inv @ jacobian.T)
    return task_inertia_matrix


def calculate_tor(model, data, q_posit, x_e, kp, ki, kd, dt, reg_task_weight=0.01):
    torque_limit = 200
    _, jacobian = compute_jacobian_end_effector(model, data, q_posit)
    jacobian_t = jacobian.T
    fw = controller_update(x_e, kp, ki, kd, dt)

    null_space_proj = null_space_projection(jacobian)
    secondary_task = -reg_task_weight * q_posit.reshape(-1, 1)
    regularization_torque = null_space_proj @ secondary_task

    final_torque = jacobian_t @ fw + regularization_torque

    final_torque = np.clip(final_torque, -torque_limit, torque_limit)

    return final_torque


def calculate_orientation_error(model, data, q_desired, q_current):
    t_wd, _ = compute_jacobian_end_effector(model, data, q_desired)
    t_wb, _ = compute_jacobian_end_effector(model, data, q_current)
    r_wd = t_wd.rotation
    r_wb = t_wb.rotation

    product = r_wd @ r_wb.T

    orient_error = (pin.log3(product)).reshape(3, 1)

    return orient_error


def calculate_motion_error(model, data, q_desired, q_current):
    t_wd, _ = compute_jacobian_end_effector(model, data, q_desired)
    t_wb, _ = compute_jacobian_end_effector(model, data, q_current)

    tran_error = (t_wd.translation - t_wb.translation).reshape((3, 1))
    return tran_error


def error_table(model, data, q_desired, q_current):
    trans_error = calculate_motion_error(model, data, q_desired, q_current)
    orient_error = calculate_orientation_error(model, data, q_desired, q_current)

    x_e = np.concatenate((trans_error, orient_error))

    return x_e


def make_motion_with_controller(model, data, q_desired, q_start, u_start, kp, ki, kd, dt, number_of_motions):
    error_threshold = 0.3
    q_list = []
    error_list = []
    q_current = q_start
    q_list.append(q_current)
    u_value = u_start
    print("Q start: ", q_start)

    for _ in range(number_of_motions):
        new_error = error_table(model, data, q_desired, q_current)
        error_list.append(new_error)

        if np.linalg.norm(new_error) < error_threshold:
            print("Desired position reached!")
            break

        tor_values = calculate_tor(model, data, q_current, new_error, kp, ki, kd, dt)
        q_pos_new, u_pos_new = step_world(q_current, u_value, tor_values, dt, model, data)
        print("Q new: ", q_pos_new)

        q_list.append(q_pos_new)
        q_current = q_pos_new
        u_value = u_pos_new

    return q_list, error_list
