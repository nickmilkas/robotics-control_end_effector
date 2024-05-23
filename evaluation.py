import numpy as np


def scenario_1():
    q_start1 = np.array([0.1, 1.5, 1.4, -1.8, 0.1, 3.6, 0.1])
    q_target1 = np.array([0.6, -0.645, -0.65, -0.15, -0.31, 0.18, 0.3])
    return q_start1, q_target1


def scenario_2():
    q_start2 = np.array([0.1, 1.5, 1.4, -1.8, 0.1, 3.6, 0.1])
    q_target2 = np.array([0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398])
    return q_start2, q_target2


def scenario_3():
    q_start3 = np.array([0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398])
    q_target3 = np.array([0.0, 0.0, 0.0, -2.356194, 0.0, 1.570796, 0.785398])
    return q_start3, q_target3


def scenario_4():
    q_start3 = np.array([0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398])
    q_target3 = np.array([0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796 + 0.785398, 0.785398 + 0.785398])
    return q_start3, q_target3


def evaluation_scenarios(number_of_scenario):
    list_of_scenarios = [scenario_1(), scenario_2(), scenario_3(), scenario_4()]

    selected_scenario = list_of_scenarios[number_of_scenario]

    return selected_scenario
