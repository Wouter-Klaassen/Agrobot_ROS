import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np

class CropDetectionNode(Node):
    def _init_(self):
        super()._init_('crop_detection_node')

        # Publisher for detections
        self.detection_publisher = self.create_publisher(String, '/crop_detections', 10)

        # Initialize RoboflowOak
        self.rf = RoboflowOak(model="cropsdetection", confidence=0.05, overlap=0.5,
                              version="1", api_key="h8kAUqPQkblRbC08ds9u", rgb=True,
                              depth=True, device=None, blocking=True)

        # Timer for periodic processing
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz
        self.get_logger().info('CropDetectionNode has started.')

    def process_frame(self):
        t0 = time.time()

        # Perform model inference
        result, frame, raw_frame, depth = self.rf.detect()
        predictions = result["predictions"]

        # Prepare detection data
        detection_data = []
        for p in predictions:
            bbox_center_x = p.x
            bbox_center_y = p.y
            confidence = p.confidence
            detected_class = p.class_name

            detection_data.append(f"Detected {detected_class} with confidence {confidence:.2f} at ({bbox_center_x}, {bbox_center_y})")

        # Publish detections
        if detection_data:
            detection_message = String()
            detection_message.data = "\n".join(detection_data)
            self.detection_publisher.publish(detection_message)

        # Log detections
        for detection in detection_data:
            self.get_logger().info(detection)

        # Benchmarking FPS
        t = time.time() - t0
        self.get_logger().info(f"FPS: {1/t:.2f}")

        # Show video feed with detections
        cv2.imshow("frame", frame)

        # Show depth map (if depth sensor is available)
        if depth is not None:
            max_depth = np.amax(depth)
            cv2.imshow("depth", depth / max_depth)

        # Close on 'q'
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            self.get_logger().info('Shutting down node.')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CropDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if _name_ == '_main_':
    main()
