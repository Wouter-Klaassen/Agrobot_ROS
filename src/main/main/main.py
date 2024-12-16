import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator')
        self.subscription = self.create_subscription(String, 'cmd', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, 'cmd', 10)
        self.subscription = self.create_subscription(String, 'EmergencyStop', self.listener_callback, 10)
        self.subscription = self.create_subscription(String, 'GoToPos', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, 'GoToPos', 10)
        self.subscription = self.create_subscription(String, 'HarvestCrops', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, 'HarvestCrops', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"received: {msg.data}")

class MoveNode(Node):
    def __init__(self):
        super().__init__('Move')
        self.subscription = self.create_subscription(String, 'GoToPos', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, 'GoToPos', 10)
        self.publisher = self.create_publisher(String, 'DriveVal', 10)
        self.subscription = self.create_subscription(String, 'CurrentPos', self.listener_callback, 10)


    def listener_callback(self, msg):
        self.get_logger().info(f"received: {msg.data}")

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('EmergencyStop')
        self.publisher = self.create_publisher(String, 'EmergencyStop', 10)

class UWBNode(Node):
    def __init__(self):
        super().__init__('UWB')
        self.publisher = self.create_publisher(String, 'CurrentPos', 10)

class HarvestNode(Node):
    def __init__(self):
        super().__init__('Harvest')
        self.publisher = self.create_publisher(String, 'HarvestCrops', 10)
        self.subscription = self.create_subscription(String, 'HarvestCrops', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"received: {msg.data}")

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    executor.add_node(CoordinatorNode())
    executor.add_node(MoveNode())
    executor.add_node(UWBNode())
    executor.add_node(EmergencyStopNode())
    executor.add_node(HarvestNode())
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()