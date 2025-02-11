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

            if len(arrayData) > 0 and "DIST" in arrayData[0]:
                number_of_anchors = int((len(arrayData) - 7) / 6)
                self.get_logger().info(f"Number of anchors detected: {number_of_anchors}")

                for i in range(number_of_anchors):
                    try:
                        # Parse anchor information
                        node_id = arrayData[2 + 6 * i]
                        x = float(arrayData[4 + 6 * i])
                        y = float(arrayData[5 + 6 * i])
                        z = float(arrayData[6 + 6 * i])
                        dist = float(arrayData[7 + 6 * i])

                        # Create publisher if not already present
                        if node_id not in self.topics:
                            self.topics[node_id] = self.create_publisher(
                                PoseStamped,
                                f'/dwm1001/{self.network}/anchor/{node_id}/position',
                                10
                            )
                            self.topics[f"{node_id}_dist"] = self.create_publisher(
                                Float64,
                                f'/dwm1001/{self.network}/tag/{self.tag_name}/to/anchor/{node_id}/distance',
                                10
                            )

                        # Publish anchor position
                        p = PoseStamped()
                        p.header.stamp = self.get_clock().now().to_msg()
                        p.pose.position.x = x
                        p.pose.position.y = y
                        p.pose.position.z = z
                        p.pose.orientation.w = 1.0
                        self.topics[node_id].publish(p)
                        self.get_logger().info(f"Anchor {node_id}: x: {x}, y: {y}, z: {z}")

                        # Publish anchor distance
                        self.topics[f"{node_id}_dist"].publish(Float64(data=dist))
                    except Exception as e:
                        self.get_logger().error(f"Error processing anchor {i}: {e}")

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
