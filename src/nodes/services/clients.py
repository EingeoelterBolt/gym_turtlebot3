from rclpy.node import Node
from std_srvs.srv import Empty


class GazeboResetSimulationServiceClient(Node):
    def __init__(self, node_name="GazeboResetSimulationServiceClient"):
        """Reset Gazebo simulation.

        Args:
            node_name (str, optional): Name of of Node which will shown on rqt_graph. Defaults to "GazeboResetSimulationServiceClient".
        """
        super().__init__(node_name)
        self.client = self.create_client(Empty, "/reset_simulation")
        while not self.client.wait_for_service(timeout_sec=None):
            self.get_logger().info(
                f"service: {node_name} not available, waiting again..."
            )
        self.request = Empty.Request()

    def send_request(self):
        """Send reset request"""
        self.future = self.client.call_async(self.request)


class GazeboPausePhysicsServiceClient(Node):
    def __init__(self, node_name="GazeboPausePhysicsServiceClient"):
        """Pause Gazebo simulation.

        Args:
            node_name (str, optional): Name of of Node which will shown on rqt_graph. Defaults to "GazeboPausePhysicsServiceClient".
        """
        super().__init__(node_name)
        self.client = self.create_client(Empty, "/pause_physics")
        while not self.client.wait_for_service(timeout_sec=None):
            self.get_logger().info(
                f"service: {node_name} not available, waiting again..."
            )
        self.request = Empty.Request()

    def send_request(self):
        """Send pause request"""
        self.future = self.client.call_async(self.request)


class GazeboUnpausePhysicsServiceClient(Node):
    def __init__(self, node_name="GazeboUnpausePhysicsServiceClient"):
        """Unpause Gazebo simulation.

        Args:
            node_name (str, optional): Name of of Node which will shown on rqt_graph. Defaults to "GazeboUnpausePhysicsServiceClient".
        """
        super().__init__(node_name)
        self.client = self.create_client(Empty, "/unpause_physics")
        while not self.client.wait_for_service(timeout_sec=None):
            self.get_logger().info(
                f"service: {node_name} not available, waiting again..."
            )
        self.request = Empty.Request()

    def send_request(self):
        """Send unpause request"""
        self.future = self.client.call_async(self.request)
