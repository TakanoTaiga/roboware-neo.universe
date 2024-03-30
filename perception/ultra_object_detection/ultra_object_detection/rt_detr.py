from typing import List, Dict

from ultralytics import RTDETR
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as rosimg
import cv_bridge

from bboxes_ex_msgs.msg import BoundingBox
from bboxes_ex_msgs.msg import BoundingBoxes

class rt_detr_node(Node):
    def __init__(self):
        super().__init__('rt_detr_node')
        self.model = RTDETR('rtdetr-l.pt')
        
        self.subscription = self.create_subscription(
            rosimg,
            '/camera/color/image_raw',
            self.listener_callback,0)
        
        self.publisher_result = self.create_publisher(BoundingBoxes, 'object_detect/result', 0)
        self.publisher_debug_image = self.create_publisher(rosimg, 'object_detect/debug/image', 0)

        self.subscription
        self.bridge = cv_bridge.CvBridge()
        
    def parse_boxes(self, results: Results) -> BoundingBoxes:
        bbox_array_msg = BoundingBoxes()

        for box_data in results.boxes:
            msg = BoundingBox()

            # get boxes values
            box = box_data.xywh[0]
            xmin = max(0, min(int(box[0] - box[2] / 2), 65535))
            ymin = max(0, min(int(box[1] - box[3] / 2), 65535))
            xmax = max(0, min(int(box[0] + box[2] / 2), 65535))
            ymax = max(0, min(int(box[1] + box[3] / 2), 65535))
            
            msg.xmin = xmin
            msg.ymin = ymin
            msg.xmax = xmax
            msg.ymax = ymax

            msg.class_id_int = int(box_data.cls)
            msg.class_id = self.model.names[int(box_data.cls)]
            msg.probability = float(box_data.conf)

            bbox_array_msg.bounding_boxes.append(msg)

        return bbox_array_msg


    def listener_callback(self, msg):
        results = self.model.predict(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'),conf=0.7,half=True)
        results: Results = results[0].cpu()
        bbox_msg = self.parse_boxes(results)
        bbox_msg.image_header = msg.header
        self.publisher_result.publish(bbox_msg)
        
        # for debug
        annotated_frame = results[0].plot()
        imgMsg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
        self.publisher_debug_image.publish(imgMsg)

def main(args=None):
    print('start ultra_obejct_detection package rt_detr node')
    rclpy.init(args=args)
    minimal_subscriber = rt_detr_node()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()