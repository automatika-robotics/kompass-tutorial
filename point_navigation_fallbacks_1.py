import numpy as np
import os
from kompass.robot import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
    RobotConfig,
)
from kompass.control import ControllersID

# GET ACTIONS
from kompass.actions import Action

from kompass.components import (
    Controller,
    DriveManager,
    Planner,
    PlannerConfig,
    LocalMapper,
)
from kompass.launcher import Launcher
from kompass.ros import Topic


config_file = "params/turtlebot3.toml"

# Setup your robot configuration
my_robot = RobotConfig(
    model_type=RobotType.DIFFERENTIAL_DRIVE,
    geometry_type=RobotGeometry.Type.CYLINDER,
    geometry_params=np.array([0.1, 0.3]),
    ctrl_vx_limits=LinearCtrlLimits(max_vel=0.4, max_acc=1.5, max_decel=2.5),
    ctrl_omega_limits=AngularCtrlLimits(
        max_vel=0.4, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3
    ),
)

config = PlannerConfig(robot=my_robot, loop_rate=1.0)
planner = Planner(component_name="planner", config=config)

controller = Controller(component_name="controller")
driver = DriveManager(component_name="drive_manager")
mapper = LocalMapper(component_name="local_mapper")


# Configure Controller options
controller.algorithm = ControllersID.DWA
controller.direct_sensor = True

# FALLBACKS
# Add on Any fail policy to the DriveManager
driver.on_fail(action=Action(driver.restart))

controller.on_algorithm_fail(
    action=Action(method=controller.set_algorithm, args=(ControllersID.STANLEY,))
)

# Publish Twist or TwistStamped from the DriveManager based on the distribution
if "ROS_DISTRO" in os.environ and (
    os.environ["ROS_DISTRO"] in ["rolling", "jazzy", "kilted"]
):
    cmd_msg_type: str = "TwistStamped"
else:
    cmd_msg_type = "Twist"

driver.outputs(robot_command=Topic(name="/cmd_vel", msg_type=cmd_msg_type))


clicked_point = Topic(name="/clicked_point", msg_type="PointStamped")
planner.inputs(goal_point=clicked_point)

# Setup the launcher
launcher = Launcher(config_file=config_file)

# Add Kompass components
launcher.kompass(
    components=[planner, controller, mapper, driver],
    multiprocessing=True,
)


# Get odom from localizer filtered odom for all components
odom_topic = Topic(name="/odometry/filtered", msg_type="Odometry")
launcher.inputs(location=odom_topic)

# Set the robot config for all components
launcher.robot = my_robot

# Add same fallback behaviour to all components
launcher.on_fail(action_name="restart", max_retries=3)

launcher.bringup()
