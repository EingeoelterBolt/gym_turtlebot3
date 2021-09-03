import gym
import inspect
import numpy as np
import os
import psutil
import rclpy
import shlex
import subprocess
import sys
import time
import yaml
from dictlib import Dict
from gym import spaces
from math import atan2, pi
from pathlib import Path
from scipy import spatial

# Solution B - If the script importing the module is not in a package
# Adapted from: https://gist.github.com/JungeAlexander/6ce0a5213f3af56d7369
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
package_dir = Path(current_dir).resolve().parent.parent.parent
sys.path.insert(0, str(package_dir))

from nodes.services.clients import GazeboResetSimulationServiceClient
from nodes.topics.subscribers import *
from nodes.topics.publishers import *


class Turtlebot3Env(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(self):
        """Initialize environment. Launches needed Subscriber and Publisher nodes and gazebo simulation.
        Also rclpy is initialized here so don't init it in your main code.
        This behavior of initializing rclpy here, could be changed when changing the code for use with multiple environments.
        """
        super(Turtlebot3Env, self).__init__()
        # Launch rclpy
        rclpy.init()
        # Read config file
        self.env_config = self._get_config()
        # Get coordinates of target position
        self.target_position = self.env_config.worlds.turtlebot3_world.target
        # Launch gazebo with turtlebot3
        self._launch_turtlebot_gazebo()
        # Launch odometry, velocity, LaserScan subscriber node
        self.odometry_subscriber_node = OdometrySubscriber()
        self.velocity_subscriber_node = VelocitySubscriber()
        self.laserscan_subscriber_node = LaserscanSubscriber()
        # Launch velocity publisher node
        self.velocity_publisher_node = VelocityPublisher()
        # Define action space and observation space
        self.action_space = self._create_action_space()
        self.observation_space = self._create_observation_space()

    def _update_subscriber_nodes(self) -> None:
        """Used to update subscriber nodes by spinning once."""
        rclpy.spin_once(self.odometry_subscriber_node)
        rclpy.spin_once(self.velocity_subscriber_node)
        rclpy.spin_once(self.laserscan_subscriber_node)

    def _create_observation_space(self) -> spaces.Box:
        """Create observation space.
        For more information about the format and how to interpret it, read https://stackoverflow.com/a/44404347.

        Returns:
            spaces.Box: Observation space format which is needed to pass to the environment.
        """
        # Define variables for the observation space.
        # Helpful link: https://stackoverflow.com/questions/44404281/openai-gym-understanding-action-space-notation-spaces-box
        # Define lower and upper bound for the observations.
        # Lower and upper bound distance to target.
        distance_low = np.NINF
        distance_high = np.Inf
        # Lower and upper bound angle to target
        angle_to_target_min = -1.0
        angle_to_target_max = 1.0
        # Lower and upper bound theta (euler_orientation on the Z-component) of turtlebot3.
        theta_low = -1.0
        theta_high = 1.0
	# Lower and upper bound velocity.         
        velocity_low_x = -1.0
        velocity_high_x = 1.0
        # Lower and upper bound angular velocity.  
        angular_low_z = -1.0
        angular_high_z = 1.0
        # Get number of lasers
        number_of_lasers = self.env_config.worlds.turtlebot3_world.number_of_lasers
        # Create array for lower bound laser values. result -> [0,0,0,0,...,0] of number_of_lasers length (len == number_of_lasers).
        laser_low = np.full((1, number_of_lasers), 0.0)
        # Create array for higher laser values. Values from model.sdf file or though reading /scan subscriber node. result -> [3.5, 3.5, 3.5,...., 3.5] of number_of_lasers length (len == number_of_lasers).
        laser_high = np.full((1, number_of_lasers), 3.5)
        
        
        # Define lower bound for observation space as np.array.
        observation_space_low = np.append(
            laser_low,
            np.array(
                [
                    velocity_low_x,
                    angular_low_z,
                    theta_low,
                    distance_low,
                    angle_to_target_min,
                ]
            ),
        )
        # Define upper bound for observation space as np.array.
        observation_space_high = np.append(
            laser_high,
            np.array(
                [
                    velocity_high_x,
                    angular_high_z,
                    theta_high,
                    distance_high,
                    angle_to_target_max,
                ]
            ),
        )
	
	# Return observation space.
        return spaces.Box(
            low=observation_space_low,
            high=observation_space_high,
            shape=observation_space_low.shape,
            dtype=np.float32,
        )

    def _create_action_space(self) -> spaces.Box:
        """Define and create action space format.
        First value of the action space is for the velocity, the second the angular rotation velocity.
        Value are between [-1, 1] and can be interpret as precentages.
        When using neural networks use tanh-activation-function as output to make sure to be in that range.

        Returns:
            spaces.Box: Action space format which is passed from the environment.
        """
        # Lower bound for action space
        action_space_low = np.array([-1, -1])
        # Higher bound for action space
        action_space_high = np.array([1, 1])

	# Return action space.
        return spaces.Box(
            low=action_space_low,
            high=action_space_high,
            shape=action_space_low.shape,
            dtype=np.float32,
        )

    def step(self, action: np.ndarray, time_per_action=0.1):
        """Do a step with specified time (in seconds) per action. Order of execution: action, new observation, calculate potential reward, check if on target position or hitting wall.

        Args:
            action (np.ndarray): Action to take (velocity and angular velocity) in percentage. Values can be between [-1, 1] e.g [-0.2, 0.4]
            time_per_action (float, optional): Time in seconds for a step/a action. Defaults to 0.1.

        Returns:
            numpy.ndarray, float, bool, string: observation, reward, done, info
        """

        # Specifiy threshhold for hitting wall.
        wall_threshold = 0.18
        # Specifiy threshhold for completing the task successfully.
        success_distance_threshold = 0.4
        # Flag that marks the termination of an episode
        done = False
        # action[0] equals linear_x and action[1] equals angular_z. It can be understood as percentage values between [-1, 1]
        # Use .item() to convert numpy.float32 to python native float. This is needed, because every publisher Expects python float for values to publish.
        linear_x = action[0][0].item()
        angular_z = action[0][1].item()
        # Constraint linear_x to be between [-1,1]
        if linear_x >= 1.0:
            linear_x = 1.0
        if linear_x <= -1.0:
            linear_x = -1.0
        # Constraint angular_z to be between [-1,1]
        if angular_z >= 1.0:
            angular_z = 1.0
        if angular_z <= -1.0:
            angular_z = -1.0
        # Publish to turtlebot3 (cmd_vel node)
        self.velocity_publisher_node.move(
            linear={"x": 0.26 * linear_x, "y": 0.0, "z": 0.0},
            angular={"x": 0.0, "y": 0.0, "z": 1.82 * angular_z},
        )
        # Wait fot specified time period.
        time.sleep(time_per_action)
        # Update Observations.
        self._update_subscriber_nodes()
        # Get observations.
        observation = self._get_observation()
        # Get reward.
        reward = self._get_reward()
        # Calculate distance to target.
        distance_to_target_position = self._get_euclidean_distance()
        # Check if turtlebot3 is in the threshold for successfully finishing the episode
        if distance_to_target_position <= success_distance_threshold:
            done = True
            info = f"Target spot reached in the given range (distance) <={success_distance_threshold}"
            reward = 1000
            # reset enviorment
            self.reset()
            return observation, reward, done, info
        # Check if a laserscan point returns a value below the specified wall_threshold. If yes it means turtlebot3 hits the wall.
        # From the observation the values which are not laserscan values have to be removed first before evaluation e.g. observation[:-5] for removing the last 5 values from the array.
        if (observation[:-5] <= wall_threshold / 3.5).any():
            done = True
            # Give high negative reward because of hitting the wall.
            reward = -150
            info = "Turtlebot3 hit the wall!"
            # reset environment
            self.reset()
            return observation, reward, done, info
        info = "Step successfull. Next step can be done"
        return observation, reward, done, info

    def reset(self):
        """Reset gazebo simulation, by bringing all models in the simulation to the original position.

        Returns:
            numpy.ndarray: Returns observation after reseting the simulation.
        """
        # Set velocity and angular values to zero
        self.velocity_publisher_node.move(
            linear={"x": 0.0, "y": 0.0, "z": 0.0},
            angular={"x": 0.0, "y": 0.0, "z": 0.0},
        )
        time.sleep(1)
        # Reset environment
        reset = GazeboResetSimulationServiceClient()
        reset.send_request()
        time.sleep(1.5)
        # Update Observations.
        self._update_subscriber_nodes()
        # Get newest observation after reset.
        observation = self._get_observation()
        return observation

    def _get_euclidean_distance(self) -> float:
        """Get euclidean distance between turtlebot3 and target.

        Returns:
            float: Distance between turtlebot3 and target.
        """
        distance = spatial.distance.euclidean(
            # Current Turtlebot3 position.
            [
                self.odometry_subscriber_node.position.x,
                self.odometry_subscriber_node.position.y,
            ],
            # Target position.
            [
                self.target_position.x,
                self.target_position.y,
            ],
        )
        return distance

    def _get_observation(self):
        """Get observation of turtlebot3.

        Returns:
            np.array: Observation which the turtlebot3 makes.
        """
        # Get velocity.
        velocity_x = self.velocity_subscriber_node.linear.x
        # Get angular velocity.
        angular_z = self.velocity_subscriber_node.angular.z
        # Get angle to target.
        angle_to_target = self._get_angle_to_target()
        # Get distance to target.
        distance_to_target = self._get_euclidean_distance()
        # Get laserscan.
        laserscan = self.laserscan_subscriber_node.ranges
        # Get theta (direction on which turtlebot3 shows).
        theta = self.odometry_subscriber_node.euler_orientation.z
        # Create numpy array with all observations and normalise them.
        observation = np.append(
            laserscan / 3.5,
            np.array(
                [
                    velocity_x / 0.26,
                    angular_z / 1.82,
                    theta / pi,
                    distance_to_target,
                    angle_to_target,
                ]
            ),
        )
        return observation

    def _get_angle_to_target(self):
        """Get angle to target
  
        Returns:
            float: Angle to target in range (-pi, pi)
        """
        # Calculate delta from the target position and turtlebot3 position
        delta_x = self.target_position.x - self.odometry_subscriber_node.position.x
        delta_y = self.target_position.y - self.odometry_subscriber_node.position.y
        # Calculate angle to target
        angle_to_target = atan2(delta_y, delta_x)
        return angle_to_target

    def _get_reward(self):
        """Returns the reward when calling the step()-method.

        Returns:
            float: Reward for taking the action on the step()-method.
        """
        reward = 0.0
        # Get euclidean distance to target position
        distance_to_target = -self._get_euclidean_distance()
        # Get angle to target position
        angle_to_target = abs(self._get_angle_to_target())
        # Create reward (negative)
        reward = distance_to_target - abs(angle_to_target)
        return reward

    def render(self, mode="human"):
        # TODO: Change code for use with gzserver only (without GUI) and gzclient (with GUI, like for now).
        pass

    def close(self):
        # TODO: Change behaivor when changing code for multiple environments.
        """Kills all running gazebo process and shutdowns rclpy."""
        # Kill all gazebo instances if running (gzserver and gzclient)
        self._kill_process("gzclient")
        self._kill_process("gzserver")
        # Shutdown rclpy
        rclpy.shutdown()

    def _get_config(self) -> Dict:
        """Read configuration file.

        Returns:
            Dict: Configuration as Dict.
        """
        current_dir = os.path.dirname(
            os.path.abspath(inspect.getfile(inspect.currentframe()))
        )
        config_file_path = Path(current_dir).resolve().parent.parent.parent
        config_file_path = os.path.join(config_file_path, "config.yaml")
        with open(config_file_path, "r") as f:
            config = yaml.safe_load(f)
        # Convert python dict to dictlib dict (allows to access values with dot notation eg. config.model)
        config = Dict(config)
        return config

    def _launch_turtlebot_gazebo(self, world_name="turtlebot3_world.launch.py"):
        """Launch specified gazebo world with GUI (gzclient and gzserver).

        Args:
            world_name (str, optional): Name of the world. Defaults to "turtlebot3_world.launch.py".
        """
        package = "turtlebot3_gazebo"
        # world_name = "turtlebot3_house.launch.py"
        launch_gazebo_command = f"ros2 launch {package} {world_name}"
        # Load config
        config = self._get_config()
        print(f"config: {config}")
        # Set turtlebot3 model
        os.environ["TURTLEBOT3_MODEL"] = config["model"]
        # Kill all gazebo instances if running (gzserver and gzclient)
        self._kill_process("gzclient")
        self._kill_process("gzserver")
        # Launch turtlebot3 together with gazebo environment adapted
        # from: https://answers.ros.org/question/42849/how-to-launch-a-launch-file-from-python-code/?answer=286903#post-id-286903
        # and https://fabianlee.org/2019/09/15/python-getting-live-output-from-subprocess-using-poll/
        # TODO: Better implementation of when to continue the script, right now it could be, that the condition is to fast and the gazebo simulation is not running.
        #       When using jupyter notebooks this should not be a problem, because the user can wait until gazebo is launched.
        process = subprocess.Popen(
            shlex.split(launch_gazebo_command), shell=False, stdout=subprocess.PIPE
        )
        while True:
            output = process.stdout.readline()
            if process.poll() is not None:
                break
            if "Going to publish joint [wheel_right_joint]" in output.strip().decode(
                "utf-8"
            ):
                break

    def _kill_process(self, process_name):
        """Kill all running processes by name

        Args:
            process_name (str): process name e.g. gzserver or firefox
        """
        for proc in psutil.process_iter():
            # check whether the process name matches
            if proc.name() == process_name:
                proc.kill()
