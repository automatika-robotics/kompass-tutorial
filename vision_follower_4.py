import numpy as np
from kompass_core.models import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
)

from kompass_interfaces.action import TrackVisionTarget
from kompass import event
from kompass.actions import ComponentActions
from kompass.actions import Action
from kompass.components import DriveManager, Controller, ControllerConfig
from kompass.config import RobotConfig
from kompass.launcher import Launcher

from agents.models import VisionModel
from agents.config import VisionConfig
from agents.ros import Topic
from agents.components import Vision

config_file = "./params/turtlebot3.toml"

# Define topics for the vision follower
# image0 is the input RGB image topic, detections_topic is the output topic for detected objects
image0 = Topic(name="/image_raw", msg_type="Image")
detections_topic = Topic(name="detections", msg_type="Detections")

# Define a Vision component using on-device local classifier
# This component will detect objects in image0 topic and publish the results to the detections topic
detection_config = VisionConfig(threshold=0.5, enable_local_classifier=True, device_local_classifier="cpu")

object_detection = VisionModel(
    name="object_detection",
    checkpoint="rtmdet_tiny_8xb32-300e_coco",
)

vision = Vision(
    inputs=[image0],
    outputs=[detections_topic],
    trigger=image0,
    config=detection_config,
    # model_client=roboml_detection,
    component_name="agents_detection_component",
)

# robot configuration
robot = RobotConfig(
    model_type=RobotType.DIFFERENTIAL_DRIVE,
    geometry_type=RobotGeometry.Type.BOX,
    geometry_params=np.array([0.60, 0.37, 0.4]),
    ctrl_vx_limits=LinearCtrlLimits(max_vel=1.5, max_acc=2.5, max_decel=7.5),
    ctrl_omega_limits=AngularCtrlLimits(
        max_vel=1.5, max_acc=2.5, max_decel=4.0, max_steer=np.pi / 2
    ),
)

# Add a DriveManager, LocalMapper, and Controller components
# DriveManager will handle the robot's movement, LocalMapper will process the LiDAR data,
# and Controller will manage the robot's behavior based on the vision detections
driver = DriveManager(component_name="kompass_drive_manager")


# Controller configuration for the VisionRGBDFollower algorithm
config = ControllerConfig(ctrl_publish_type="Sequence")
controller = Controller(component_name="kompass_controller", config=config)
controller.inputs(
    vision_detections=detections_topic)

controller.algorithm = "VisionRGBFollower"
controller.direct_sensor = True

start_following_topic = Topic(name="start_following", msg_type="Bool")

# Event to trigger the vision following system when the start_following topic is set to True
event_vision_following_cmd = event.OnEqual(
    "start_following",
    start_following_topic,
    True,
    ("data"),
)

#  Define an action to send a goal to the TrackVisionTarget action server
req = TrackVisionTarget.Goal()
req.label = "person"
send_goal_action: Action = ComponentActions.send_action_goal(
    action_name="/kompass_controller/track_vision_target",
    action_type=TrackVisionTarget,
    action_request_msg=req,
)


launcher = Launcher(config_file=config_file)

# Set robot for the whole stack
launcher.robot = robot

events_actions = {
    event_vision_following_cmd: send_goal_action,
}
launcher.add_pkg(
    components=[vision],
    ros_log_level="info",
    package_name="automatika_embodied_agents",
    executable_entry_point="executable",
    events_actions=events_actions,
    multiprocessing=True,
)

launcher.kompass(
    components=[driver, controller],
    activate_all_components_on_start=True,
    multiprocessing=True,
)

# Enable UI with start_following_topic as input and image0 and detections_topic as outputs
launcher.enable_ui(
    inputs=[start_following_topic],
    outputs=[image0, detections_topic],
)

launcher.bringup()
