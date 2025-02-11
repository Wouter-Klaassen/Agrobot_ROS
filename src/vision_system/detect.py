from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np
import rclpy
from rclpy.node import Node
from cropdetections.srv import Detection  


if __name__ == '__main__':
    # Instantieer een object (rf) met de RoboflowOak module
    rf = RoboflowOak(model="cropsdetection", confidence=0.05, overlap=0.5,
                      version="1", api_key="h8kAUqPQkblRbC08ds9u", rgb=True,
                      depth=True, device=None, blocking=True)

    while True:
        t0 = time.time()
        # Voer model-inferentie uit
        result, frame, raw_frame, depth = rf.detect()
        predictions = result["predictions"]

        # Print objectdetecties
        for p in predictions:
            bbox_center_x = p.x  
            bbox_center_y = p.y  
            confidence = p.confidence  
            detected_class = p.class_name  # Gebruik .class_name i.p.v. .class  
            
            print(f"Detected {detected_class} with confidence {confidence:.2f} at ({bbox_center_x}, {bbox_center_y})")

        # Benchmarking: meet FPS
        t = time.time()-t0
        print("FPS ", 1/t)

        # Toon dieptekaart (comment uit als je geen dieptesensor hebt)
        max_depth = np.amax(depth)
        cv2.imshow("depth", depth/max_depth)

        # Toon videofeed met detecties
        cv2.imshow("frame", frame)

        # Sluit met 'q'
        if cv2.waitKey(1) == ord('q'):
            break
