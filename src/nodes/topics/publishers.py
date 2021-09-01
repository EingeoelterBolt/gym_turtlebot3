import numpy as np
from geometry_msgs.msg import Twist
from rclpy.node import Node


class VelocityPublisher(Node):
    def __init__(self, node_name="Custom_Velocity_Publisher"):
        """
        Args:
            node_name (str, optional): Name of of Node which will shown on rqt_graph. Defaults to "Velocity_Publisher".
        """
        super().__init__(node_name)
        self.velocity_publisher_node = self.create_publisher(Twist, "/cmd_vel", 10)

    def move(self, linear: np.float, angular: np.float) -> None:
        """Publish velocity to /cmd_vel node. Both linear and angular have to be a Dict in the structure: {'x': 0.0, 'y': 0.0, 'z': 0.0}. The Dict-values have to be float numbers.

        Args:
            linear (np.float): linear velocity to publish. Has to be structure in form: {'x': 0.0, 'y': 0.0, 'z': 0.0} and the values have to be float numbers.
            angular (np.float): angular velocity to publish. Has to be structure in form: {'x': 0.0, 'y': 0.0, 'z': 0.0} and the values have to be float numbers.
        """
        vel_msg = Twist()
        vel_msg.linear.x = linear["x"]
        vel_msg.linear.y = linear["y"]
        vel_msg.linear.z = linear["z"]
        vel_msg.angular.x = angular["x"]
        vel_msg.angular.y = angular["y"]
        vel_msg.angular.z = angular["z"]
        # Publish message
        self.velocity_publisher_node.publish(vel_msg)
