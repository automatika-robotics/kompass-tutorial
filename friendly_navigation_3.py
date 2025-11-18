import numpy as np
from agents.components import Vision
from agents.models import VisionModel
from agents.config import VisionConfig
from agents.ros import Topic

from kompass.components import DriveManager
from kompass.robot import (
    AngularCtrlLimits,
    LinearCtrlLimits,
    RobotGeometry,
    RobotType,
    RobotConfig,
)
from kompass import event
from kompass.actions import Action
from kompass.launcher import Launcher

config_file = "params/vision.yaml"

# RGB camera input topic is set to the compressed image topic
image0 = Topic(name="/image_raw/compressed", msg_type="CompressedImage")

# Select the output topics: detections (and/or trackings)
detections_topic = Topic(name="detections", msg_type="Detection2D")


# Configure the Vision Component
object_detection = VisionModel(
    name="object_detection",
    checkpoint="rtmdet_tiny_8xb32-300e_coco",
)

# Select the vision component configuration
detection_config = VisionConfig(
    threshold=0.5, enable_local_classifier=True, device_local_classifier="cpu"
)

# Create the vision component
vision = Vision(
    inputs=[image0],
    outputs=[detections_topic],
    trigger=image0,
    config=detection_config,
    model_client=object_detection,
    component_name="detection_component",
)

# Setup your robot configuration
my_robot = RobotConfig(
    model_type=RobotType.DIFFERENTIAL_DRIVE,
    geometry_type=RobotGeometry.Type.CYLINDER,
    geometry_params=np.array([0.1, 0.3]),
    ctrl_vx_limits=LinearCtrlLimits(max_vel=1.0, max_acc=1.5, max_decel=2.5),
    ctrl_omega_limits=AngularCtrlLimits(
        max_vel=1.0, max_acc=2.0, max_decel=2.0, max_steer=np.pi / 3
    ),
)

# Setup the drive manager
driver = DriveManager(component_name="driver")

# Define an event to be triggered when a person is detected for the first time
# This event will trigger when the 'detections_topic' changes its value and contains any of the specified labels for the first time i.e. the event will not keep on getting trigger while a person is continuously detected
person_labels = ["person", "pedestrian", "human"]
event_person_detected_first_time = event.OnChangeContainsAny(
    "person_detected",
    detections_topic,
    person_labels,
    ("labels"),
)


# Define an action to reduce the speed limit of the DriveManager
action_reduce_speed_limit = Action(
    method=driver.set_param,
    kwargs={
        "param_name": "robot.ctrl_vx_limits.max_vel",
        "new_value": 0.3,
        "keep_alive": True,
    },
)


events_actions = {
    event_person_detected_first_time: action_reduce_speed_limit,
}

launcher = Launcher()

# Add the vision component form "automatika_embodied_agents" package
launcher.add_pkg(
    components=[vision],
    package_name="automatika_embodied_agents",
    executable_entry_point="executable",
    multiprocessing=True,
    ros_log_level="Info",
    rclpy_log_level="Error",
)

# Add Kompass components
launcher.kompass(
    components=[driver],
    events_actions=events_actions,
    multiprocessing=True,
    activate_all_components_on_start=True,
)
# Set the robot config for all components
launcher.robot = my_robot
launcher.bringup()
