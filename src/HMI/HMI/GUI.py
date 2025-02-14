from tkinter.scrolledtext import ScrolledText
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import tkinter as tk
from geometry_msgs.msg import Point32

class GUINode(Node):
    def __init__(self):
        super().__init__('GUI')
        self.subscription = self.create_subscription(String, 'cmd', self.listener_callback, 10)
        self.string_publisher = self.create_publisher(String, 'cmd', 10)
        self.point32_publisher = self.create_publisher(Point32, 'xyz', 10)
        self.bool_publisher = self.create_publisher(Bool, 'gripper',10)

    def listener_callback(self, msg):
        # Verkrijg de naam van het topic
        topic_name = self.subscription.topic_name
        self.get_logger().info(f"Received: {msg.data} from topic: {topic_name}")

    def publish_string(self, value: str):
        msg = String()
        msg.data = value
        self.string_publisher.publish(msg)
        topic_name = self.string_publisher.topic_name
        self.get_logger().info(f"Published string: {msg.data} in {topic_name}")

    def publish_point32(self, x: float, y: float, z: float):
      # Maak een Point32 bericht met de opgegeven x waarde en stel y en z in op 0
        msg = Point32()
        msg.x = x
        msg.y = y
        msg.z = z
        # Publiceer het bericht naar het topic
        self.point32_publisher.publish(msg)

        topic_name = self.point32_publisher.topic_name
        self.get_logger().info(f"Published: x={msg.x}, y={msg.y}, z={msg.z} in {topic_name}")

class GUI:
    def __init__(self, root, ros_publisher):
        self.root = root
        self.ros_publisher = ros_publisher

        #mainframe
        self.root_frame = tk.Frame(root, width=500, height=400, bg="grey")
        self.root_frame.pack(pady=20, padx=20)
        self.root_frame.place(x=5, y=5)

        #autonoom frame
        self.autonoom_frame = tk.Frame(self.root_frame, width=140, height=90, bg="lightgrey")
        self.autonoom_frame.grid(row=0, column=0,padx=10,pady=10)

        #gantry frame
        self.gantry_frame = tk.Frame(self.root_frame, width=140, height=90, bg="lightgrey")
        self.gantry_frame.grid(row=0, column=1,padx=10,pady=10)

        #rupsband frame
        self.rups_frame = tk.Frame(self.root_frame, width=140, height=90, bg="lightgrey")
        self.rups_frame.grid(row=0, column=3, padx=10, pady=10)

        #autonoom knoppen
        self.button_start = tk.Button(self.autonoom_frame, text="Start",bg="green", command=self.send_start)
        self.button_start.grid(row=0, column=0, padx=10, pady=10)

        self.button_goto_start = tk.Button(self.autonoom_frame, text="Go to Start", bg="orange", command=self.send_goto_start)
        self.button_goto_start.grid(row=1, column=0, padx=10, pady=10)

        self.button_stop = tk.Button(self.autonoom_frame, text="Stop", bg="red", command=self.send_stop)
        self.button_stop.grid(row=2, column=0, padx=10, pady=10)

        #gantry input
        self.xlabel = tk.Label(self.gantry_frame,text="x:")
        self.xlabel.grid(row=0, column=0, padx=10, pady=10)
        self.entryx = tk.Entry(self.gantry_frame, width=5)
        self.entryx.grid(row=0, column=1, padx=10, pady=10)

        self.ylabel = tk.Label(self.gantry_frame,text="y:")
        self.ylabel.grid(row=1, column=0, padx=10, pady=10)
        self.entryy = tk.Entry(self.gantry_frame, width=5)
        self.entryy.grid(row=1, column=1, padx=10, pady=10)

        self.zlabel = tk.Label(self.gantry_frame,text="z:")
        self.zlabel.grid(row=2, column=0, padx=10, pady=10)
        self.entryz = tk.Entry(self.gantry_frame, width=5)
        self.entryz.grid(row=2, column=1, padx=10, pady=10)

        self.send_button = tk.Button(self.gantry_frame, text="Send", command=self.display_message)
        self.send_button.grid(row=3, column=1, padx=10, pady=10)

        self.initiate_button = tk.Button(self.gantry_frame, text="Gripper dicht", command=self.send_initiate)
        self.initiate_button.grid(row=3, column=0, padx=10, pady=10)

        self.gripper_open = tk.Button(self.gantry_frame, text="Gripper open", command=self.send_initiate)
        self.gripper_open.grid(row=4, column=0, padx=10, pady=10)

        self.gripper_close = tk.Button(self.gantry_frame, text="initiate", command=self.send_initiate)
        self.gripper_close.grid(row=4, column=1, padx=10, pady=10)

        #rupsband knoppen
        self.button_up = tk.Button(self.rups_frame, text="↑", command=self.send_up)
        self.button_up.grid(row=0, column=1, padx=10, pady=10)

        self.button_down = tk.Button(self.rups_frame, text="↓", command=self.send_down)
        self.button_down.grid(row=2, column=1, padx=10, pady=10)

        self.button_left = tk.Button(self.rups_frame, text="←", command=self.send_left)
        self.button_left.grid(row=1, column=0, padx=10, pady=10)

        self.button_right = tk.Button(self.rups_frame, text="→", command=self.send_right)
        self.button_right.grid(row=1, column=2, padx=10, pady=10)

        self.button_stop = tk.Button(self.rups_frame, text="■", command=self.send_stop)
        self.button_stop.grid(row=1, column=1, padx=10, pady=10)

        #GUI terminal
        self.text_window = ScrolledText(self.root_frame, wrap=tk.WORD, height=15,width=30)
        self.text_window.grid(row=0, column=4, padx=10, pady=10)

    def send_up(self):
        self.ros_publisher.publish_string("up")

    def send_initiate(self):
        self.ros_publisher.publish_string("initiate")

    def send_down(self):
        self.ros_publisher.publish_string("down")

    def send_left(self):
        self.ros_publisher.publish_string("left")

    def send_right(self):
        self.ros_publisher.publish_string("right")

    def send_start(self):
        self.ros_publisher.publish_string("start")

    def send_goto_start(self):
        self.ros_publisher.publish_string("go to start")

    def send_stop(self):
        self.ros_publisher.publish_string("stop")

    def display_message(self):
        try:
            x = float(self.entryx.get())
            y = float(self.entryy.get())
            z = float(self.entryz.get())

        # Maak een Point32 bericht
            point = Point32()
            point.x = x
            point.y = y
            point.z = z

            self.ros_publisher.publish_point32(point.x,point.y,point.z)
            self.text_window.delete(1.0, tk.END)
            self.text_window.insert(tk.END, f"# Published: x={x}, y={y}, z={z}")
        except ValueError:
            # Als de gebruiker iets invoert wat geen geldig getal is
           self.text_window.delete(1.0, tk.END)
           self.text_window.insert(tk.END, "# Error: Please enter valid numeric values for x, y, and z.")

def main():
    rclpy.init()
    ros_publisher = GUINode()
    root = tk.Tk()
    root.title("Agrobot GUI")
    root.geometry("800x300")
    GUI(root, ros_publisher)
    root.mainloop()

    rclpy.shutdown()

if __name__ == "__main__":
    main()