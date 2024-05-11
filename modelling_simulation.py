import numpy as np
import pinocchio as pin
import os

from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer

VISUALIZER = MeshcatVisualizer


def load_franka():
    current_path = os.path.abspath('')
    robot_path = os.path.join(current_path, "robot")

    robot = RobotWrapper.BuildFromURDF(os.path.join(robot_path, "franka.urdf"), package_dirs=robot_path)

    model = robot.model
    data = robot.data
    print("Robot is",robot)
    print("Model is",model)
    print("Data are",data)

    return robot, model, data


def step_world():
    robot_f, model_f, data_f, = load_franka()

    robot_f.setVisualizer(VISUALIZER())
    robot_f.initViewer()
    robot_f.loadViewerModel("pinocchio")
    q = pin.randomConfiguration(model_f)
    robot_f.display(q)
    return 0


step_world()
