import numpy as np


def scenario_1():
    q_start1 = np.array([0.0, 0.0, 0.0, -1.5, 0.0, 2.0, 0.0])
    q_target1 = np.array([1.5, 1.2, 1.8, -0.5, 0.0, 2.5, 0.2])
    return q_start1, q_target1


def scenario_2():
    q_start2 = np.array([-0.5, 0.5, -0.3, -1.5, 0.3, 2.0, -0.2])
    q_target2 = np.array([-2.8973, -1.7628, -2.8973, -0.0698, -2.8973, -3.7525, -2.8973])
    return q_start2, q_target2


def scenario_3():
    q_start3 = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
    q_target3 = np.random.uniform(-2.8973, 2.8973, size=7)
    return q_start3, q_target3


def evaluation_scenarios(number_of_scenario):
    list_of_scenarios = [scenario_1(), scenario_2(), scenario_3()]

    selected_scenario = list_of_scenarios[number_of_scenario]

    return selected_scenario
