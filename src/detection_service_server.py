import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from cropdetections.srv import Detection  # Verander naar je eigen package en service naam
from roboflowoak import RoboflowOak
import time
import numpy as np
import cv2

class DetectionServiceServer(Node):
    def __init__(self):
        super().__init__('detection_service_server')
        self.srv = self.create_service(Detection, 'get_detection', self.get_detection_callback)
        
        # Initialiseer RoboflowOak
        self.rf = RoboflowOak(
            model="cropsdetection", 
            confidence=0.05, 
            overlap=0.5,
            version="1", 
            api_key="h8kAUqPQkblRbC08ds9u", 
            rgb=True,
            depth=True, 
            device=None, 
            blocking=True
        )

    def get_detection_callback(self, request, response):
        result, frame, raw_frame, depth = self.rf.detect()
        predictions = result["predictions"]
        
        if predictions:
            p = predictions[0]  # Neem de eerste voorspelling
            response.detected_class = p.class_name
            response.bbox_center_x = p.x
            response.bbox_center_y = p.y
        else:
            response.detected_class = "No detection"
            response.bbox_center_x = 0.0
            response.bbox_center_y = 0.0
        
        self.get_logger().info(f"Detection response: {response.detected_class} at ({response.bbox_center_x}, {response.bbox_center_y})")
        return response


def main(args=None):
    rclpy.init(args=args)
    detection_service = DetectionServiceServer()
    rclpy.spin(detection_service)
    detection_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

