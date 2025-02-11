import sys
import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from navigation.dwm1001_apiCommands import DWM1001_API_COMMANDS

class DWM1001Localizer(Node):
    def __init__(self):
        super().__init__('dwm1001_localizer')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('tag_name', 'default_tag')
        self.declare_parameter('use_network', False)
        self.declare_parameter('network', "default")
        self.declare_parameter('verbose', True)

        # Get parameters
        self.dwm_port = self.get_parameter('port').get_parameter_value().string_value
        self.tag_name = self.get_parameter('tag_name').get_parameter_value().string_value
        self.use_network = self.get_parameter('use_network').get_parameter_value().bool_value
        self.network = self.get_parameter('network').get_parameter_value().string_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        # Define target positions
        self.target_positions = {
            "P1": (2.5, 1.25), "P2": (2.5, 5.75), "P3": (1.0, 5.75), "P4": (1.0, 1.25),
            "B11": (2.5, 2.5), "B12": (2.5, 3.5), "B13": (2.5, 4.5),
            "B21": (1.0, 2.5), "B22": (1.0, 3.5), "B23": (1.0, 4.5)
        }
        self.position_tolerance = 0.1  # Allow small deviation

        # Create dictionary to store publishers
        self.topics = {}

        # Serial port settings
        try:
            self.serialPortDWM1001 = serial.Serial(
                port=self.dwm_port,
                baudrate=115200,
                parity=serial.PARITY_ODD,
                stopbits=serial.STOPBITS_TWO,
                bytesize=serial.SEVENBITS
            )
            if self.serialPortDWM1001.isOpen():
                self.get_logger().info(f"Port opened: {self.serialPortDWM1001.name}")
                self.initializeDWM1001API()
            else:
                self.get_logger().error(f"Cannot open port: {self.dwm_port}")
                raise RuntimeError("Failed to open serial port")
        except Exception as e:
            self.get_logger().error(f"Serial port setup failed: {e}")
            sys.exit(1)

        # Set a timer to repeatedly call the main loop
        self.timer = self.create_timer(1.0, self.main)

    def initializeDWM1001API(self):
        """Initialize DWM1001 API by sending reset and ENTER commands"""
        try:
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            time.sleep(0.5)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            time.sleep(0.5)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            time.sleep(0.5)
            self.get_logger().info("DWM1001 API initialized")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize DWM1001 API: {e}")

    def main(self):
        """Main function to read and process data from the DWM1001 module"""
        if not self.serialPortDWM1001.isOpen():
            self.get_logger().error("Serial port is not open.")
            return

        try:
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.get_logger().info("Reading DWM1001 coordinates")

            # Read data from the serial port
            serialReadLine = str(self.serialPortDWM1001.read_until())
            self.get_logger().debug(f"Raw serial data: {serialReadLine}")
            self.publishTagPositions(serialReadLine)
        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")

    def publishTagPositions(self, serialData):
        """Publish tag and anchor positions based on serial data"""
        try:
            self.get_logger().debug(f"Raw serial data: {serialData}")
            
            arrayData = [x.strip() for x in serialData.strip().split(',')]
            self.get_logger().debug(f"Parsed serial data: {arrayData}")

            # Publish tag position
            if len(arrayData) >= 5 and "POS" in arrayData[-5]:
                try:
                    tag_x = float(arrayData[-4])
                    tag_y = float(arrayData[-3])
                    tag_z = float(arrayData[-2])

                    if self.tag_name not in self.topics:
                        self.topics[self.tag_name] = self.create_publisher(
                            PoseStamped,
                            f'/dwm1001/tag/{self.tag_name}/position',
                            10
                        )

                    p = PoseStamped()
                    p.header.stamp = self.get_clock().now().to_msg()
                    p.pose.position.x = tag_x
                    p.pose.position.y = tag_y
                    p.pose.position.z = tag_z
                    p.pose.orientation.w = 1.0
                    self.topics[self.tag_name].publish(p)
                    self.get_logger().info(f"Tag {self.tag_name}: x: {tag_x}, y: {tag_y}, z: {tag_z}")

                    # Check if the tag is near any target position
                    for key, (target_x, target_y) in self.target_positions.items():
                        if abs(tag_x - target_x) < self.position_tolerance and \
                           abs(tag_y - target_y) < self.position_tolerance:
                            self.get_logger().info(f"You've passed {key}!")
                            break
                except Exception as e:
                    self.get_logger().error(f"Error publishing tag position: {e}")
            else:
                self.get_logger().warning("No valid POS data found in serial data")

        except Exception as e:
            self.get_logger().error(f"Error processing serial data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DWM1001Localizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        if node.serialPortDWM1001.isOpen():
            node.serialPortDWM1001.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
