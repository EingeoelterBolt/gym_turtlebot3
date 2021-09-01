import numpy as np
from dictlib import Dict
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from numpy import inf
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from transforms3d.euler import quat2euler

# see: https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
# https://hackmd.io/@1IzBzEXXRsmj6-nLXZ9opw/r1zrNKBWU/%2F%401IzBzEXXRsmj6-nLXZ9opw%2FBkaxoWRiI
# https://answers.ros.org/question/360676/qos-python-code-for-create_subscriber/
# https://github.com/ros2/rclpy/blob/master/rclpy/test/test_qos.py


from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)


class OdometrySubscriber(Node):
    """OdometrySubscriber for reading and saving odometry data.
    Initialize first after initializing rclpy.int() in your main file, else you will get an Error.
    Also before calling the position, quaternion_orientation or euler_orientation run atleast once rclpy [rclpy.spin_once(Node)] to receive data, else a ValueError will occur.

    Args:
        Node (rclpy.node): Inherit from rclpy.node class. To use it just call OdometrySubscriber() in your main file.

    Raises:
        ValueError: Raises when position, quaternion_orientation and/or euler_orientation is empty. Spin atleast once with rclpy e.g. rclpy.spin_once(Node)

    Returns:
        Node: Receives and saves data when spinning with rclpy e.g. rclpy.spin_once(Node). Variables that can be used: position, quaternion_orientation and euler_orientation
    """

    def __init__(self, node_name="Custom_Odometry_Subscriber"):
        """
        Args:
            node_name (str, optional): Name of of Node which will shown on rqt_graph. Defaults to "Odometry_Subscriber".
        """
        super().__init__(node_name)
        self.odometry_subscriber_node = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self.__callback,
            qos_profile=QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.__position = None
        self.__quaternion_orientation = None
        self.__euler_orientation = None

    def __callback(self, msg) -> Dict:
        """Callback function which will be call on every rclpy spinning. It's used to save the paramters position and quanternion_orientation to the class.

        Args:
            msg (nav_msgs.msg): Odometry msg data from the callback call from create_subscription method.
        """
        self.__position = Dict(
            {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
            }
        )
        self.__quaternion_orientation = Dict(
            {
                "x": msg.pose.pose.orientation.x,
                "y": msg.pose.pose.orientation.y,
                "z": msg.pose.pose.orientation.z,
                "w": msg.pose.pose.orientation.w,
            }
        )

    @property  # Getter euler_orientation
    def euler_orientation(self) -> Dict:
        """Dict: Converts quaternion orientation to euler orientation. rclpy has to spin atleast once"""
        if (
            self.__quaternion_orientation is None
            or type(self.__quaternion_orientation) is not Dict
        ):
            raise ValueError(
                f"""quaternion_orientation == {self.__quaternion_orientation}!\n
                Subscriber does not got any information about quaternion_orientation.\n
                To calculate the odometry euler coordinations quaternion_orientation is needed.\n
                Make sure to run a publisher publishing to topic: '/odom', before access euler_orientation"""
            )
        else:
            # Get quaternion
            quaternion = self.__quaternion_orientation
            # Convert to euler
            euler = quat2euler(
                [quaternion["w"], quaternion["x"], quaternion["y"], quaternion["z"]]
            )
            # Convert tuple to dict
            euler = Dict({"x": euler[0], "y": euler[1], "z": euler[2]})
            return euler

    @property  # Getter quaternion_orientation
    def quaternion_orientation(self) -> Dict:
        """Dict: Get quaternion_orientation after rclpy spins atleast once"""
        if (
            self.__quaternion_orientation is None
            or type(self.__quaternion_orientation) is not Dict
        ):
            raise ValueError(
                f"""quaternion_orientation == {self.__quaternion_orientation}!\n
                Subscriber does not got any information about quaternion_orientation.\n
                Make sure to run a publisher publishing to topic: '/odom', before access quaternion_orientation"""
            )
        else:
            return self.__quaternion_orientation

    @property  # Getter position
    def position(self) -> Dict:
        """Dict: Get current position. rclpy has to spin atleast once."""
        if self.__position is None or type(self.__position) is not Dict:
            raise ValueError(
                f"""position == {self.__position}!\n
                Subscriber does not got any information about position.\n
                Make sure to run a publisher publishing to topic: '/odom', before access position"""
            )
        else:
            return self.__position


class LaserscanSubscriber(Node):
    """Laserscan subscriber node for reading and saving laserscan data.
    Initialize first after initializing rclpy.int() in your main file, else you will get an Error.

    Args:
        Node (rclpy.node): Inherit from rclpy.node class. To use it just call LaserscanSubscriber() in your main file.
    """

    def __init__(self, node_name="Custom_LaserScan_Subscriber"):
        """
        Args:
            node_name (str, optional): Name of of Node which will shown on rqt_graph. Defaults to "LaserScan_Subscriber".
        """
        super().__init__(node_name)
        self.odometry_subscriber_node = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.__callback,
            qos_profile=qos_profile_sensor_data,  # qos_profile_sensor_data allows packet loss
        )
        self.__angle_min = None
        self.__angle_max = None
        self.__angle_increment = None
        self.__range_min = None
        self.__range_max = None
        self.__ranges = None

    def __callback(self, msg):
        """Callback function which will be call on every rclpy spinning. It's used to save the laserscan data to the class.
           Laserscan data which are reading Inf-values are changed to range_max value. The Default is 3.50.

        Args:
            msg (sensor_msgs.msg): Laser msg data from the callback call from create_subscription method.
        """
        self.__angle_min = msg.angle_min
        self.__angle_max = msg.angle_max
        self.__angle_increment = msg.angle_increment
        self.__range_min = msg.range_min
        self.__range_max = msg.range_max
        # Get Laser data and replace all 'inf' values with range_max
        measurements = np.array(msg.ranges)
        measurements[measurements == inf] = msg.range_max
        self.__ranges = measurements

    @property  # Getter angle_min
    def angle_min(self) -> float:
        return self.__angle_min

    @property  # Getter angle_max
    def angle_max(self) -> float:
        return self.__angle_max

    @property  # Getter angle_increment
    def angle_increment(self) -> float:
        return self.__angle_increment

    @property  # Getter range_min
    def range_min(self) -> float:
        """float: Get minimum range value. rclpy has to spin atleast once."""
        return self.__range_min

    @property  # Getter range_max
    def range_max(self) -> float:
        """float: Get maximum range value. rclpy has to spin atleast once."""
        return self.__range_max

    @property  # Getter ranges
    def ranges(self) -> np.ndarray:
        """np.ndarray: Get all measured values from the laserscan. When e.g. 360 Laserscans used it returns 360 values as np.ndarray. rclpy has to spin atleast once."""
        return self.__ranges


class VelocitySubscriber(Node):
    """Velocity subscriber node for reading and saving velocity data.
    Initialize first after initializing rclpy.int() in your main file, else you will get an Error.

    Args:
        Node (rclpy.node): Inherit from rclpy.node class. To use it just call VelocitySubscriber() in your main file and call rclpy.spin_one(node).
    """

    def __init__(self, node_name="Custom_Velocity_Subscriber"):
        """
        Args:
            node_name (str, optional): Name of of Node which will shown on rqt_graph. Defaults to "Velocity_Subscriber".
        """
        super().__init__(node_name)
        self.odometry_subscriber_node = self.create_subscription(
            msg_type=Twist,
            topic="/cmd_vel",
            callback=self.__callback,
            qos_profile=QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.__linear = None
        self.__angular = None

    def __callback(self, msg) -> Dict:
        """Callback function which will be call on every rclpy spinning. It's used to save the paramters linear velocity and angular velocity to the class.

        Args:
            msg (geometry_msgs.msg): Velocity msg data from the callback call from create_subscription method.
        """
        self.__linear = Dict(
            {
                "x": msg.linear.x,
                "y": msg.linear.y,
                "z": msg.linear.z,
            }
        )
        self.__angular = Dict(
            {
                "x": msg.angular.x,
                "y": msg.angular.y,
                "z": msg.angular.z,
            }
        )

    @property  # Getter linear
    def linear(self) -> Dict:
        """Dict: Get current linear velocity. rclpy has to spin atleast once."""
        return self.__linear

    @property  # Getter angular
    def angular(self) -> Dict:
        """Dict: Get current angular velocity. rclpy has to spin atleast once."""
        return self.__angular
