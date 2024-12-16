import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk

class GUINode(Node):
    def __init__(self):
        super().__init__('GUI')
        self.subscription = self.create_subscription(String, 'cmd', self.listener_callback, 10)
        self.string_publisher = self.create_publisher(String, 'cmd', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"received: {msg.data}")

    def publish_string(self, value: str):
        msg = String()
        msg.data = value
        self.string_publisher.publish(msg)
        self.get_logger().info(f"Published string: {msg.data}")

class GUI:
    def __init__(self, root, ros_publisher):
        self.root = root
        self.ros_publisher = ros_publisher

        self.button_up = tk.Button(root, text="↑", command=self.send_up)
        self.button_up.pack(pady=10)
        self.button_up.place(x=50, y=10)

        self.button_down = tk.Button(root, text="↓", command=self.send_down)
        self.button_down.pack(pady=10)
        self.button_down.place(x=50, y=50)

        self.button_left = tk.Button(root, text="←", command=self.send_left)
        self.button_left.pack(pady=10)
        self.button_left.place(x=10, y=30)

        self.button_right = tk.Button(root, text="→", command=self.send_right)
        self.button_right.pack(pady=10)
        self.button_right.place(x=90, y=30)

    def send_up(self):
        self.ros_publisher.publish_string("up")

    def send_down(self):
        self.ros_publisher.publish_string("down")

    def send_left(self):
        self.ros_publisher.publish_string("left")

    def send_right(self):
        self.ros_publisher.publish_string("right")


def main():
    rclpy.init()
    ros_publisher = GUINode()
    root = tk.Tk()
    root.title("Agrobot GUI")

    gui = GUI(root, ros_publisher)
    root.mainloop()

    rclpy.shutdown()

if __name__ == "__main__":
    main()