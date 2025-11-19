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

from kompass.components import (
    Controller,
    DriveManager,
    Planner,
    PlannerConfig,
    LocalMapper,
    MapServer,
    MapServerConfig,
)
from kompass.launcher import Launcher
from kompass.ros import Topic

from ament_index_python.packages import (
    get_package_share_directory,
)


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
controller.direct_sensor = False


# Serving the global map
map_topic = Topic(name="map", msg_type="OccupancyGrid")

kompass_sim_pkg = get_package_share_directory("kompass_sim")
reference_map = os.path.join(kompass_sim_pkg, "maps", "turtlebot3_gazebo_house.yaml")

config = MapServerConfig(
    loop_rate=1.0,
    map_file_path=reference_map,  # Path to a 2D map yaml file or a point cloud file
    # map_file_path="/path_to_pcd_file.pcd",  # Path to a 2D map yaml file or a point cloud file
    # z_ground_limit=0.01,
    # grid_resolution=0.5,
    # pc_publish_row=True,
)
map_server = MapServer(component_name="global_map_server", config=config)
map_server.outputs(map=map_topic)

# Setup the launcher
launcher = Launcher(config_file=config_file)

# Add Kompass components
launcher.kompass(
    components=[planner, controller, mapper, driver],
    multiprocessing=True,
)


# Get odom from localizer filtered odom for all components
# odom_topic = Topic(name="/odometry/filtered", msg_type="Odometry")
# launcher.inputs(location=odom_topic)

# Set the robot config for all components
launcher.robot = my_robot

launcher.bringup()
