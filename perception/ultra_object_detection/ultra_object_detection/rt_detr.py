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

from foxglove_msgs.msg import ImageAnnotations, PointsAnnotation, TextAnnotation
from foxglove_msgs.msg import Point2 , Color as foxgloveColor

class rt_detr_node(Node):
    def __init__(self):
        super().__init__('rt_detr_node')
        self.model = RTDETR('rtdetr-l.pt')
        
        self.subscription = self.create_subscription(
            rosimg,
            'input/raw_image',
            self.listener_callback,0)
        
        self.publisher_result = self.create_publisher(BoundingBoxes, 'output/result', 0)
        self.publisher_debug_image = self.create_publisher(rosimg, 'debug/image', 0)
        self.publisher_image_markers = self.create_publisher(ImageAnnotations, 'debug/image_markers', 0)

        self.subscription
        self.bridge = cv_bridge.CvBridge()
        
        self.get_logger().info('start ultra_obejct_detection package rt_detr node')
                
    def publish_image_markers(self, bbox_msg: BoundingBoxes, header):
        markers_msg = ImageAnnotations()
        for i, box in enumerate(bbox_msg.bounding_boxes):
            marker = PointsAnnotation()
            marker.timestamp = bbox_msg.header.stamp
            marker.type = 2
            marker.points = [
                Point2(x=float(box.xmin), y=float(box.ymin)),  # 左上
                Point2(x=float(box.xmax), y=float(box.ymin)),  # 右上
                Point2(x=float(box.xmax), y=float(box.ymax)),  # 右下
                Point2(x=float(box.xmin), y=float(box.ymax)),  # 左下
                Point2(x=float(box.xmin), y=float(box.ymin))   # 最初の点に戻る
            ]
            marker.thickness = 2.0
            marker.outline_color = foxgloveColor(r=0.905, g=0.298, b=0.235, a=1.0)
            marker.fill_color = foxgloveColor(r=0.0, g=0.0, b=0.0, a=0.0)
            
            markers_msg.points.append(marker)
            
            label = TextAnnotation()
            label.timestamp = bbox_msg.header.stamp
            label.position = Point2(x=float(box.xmin), y=float(box.ymin))
            label.text = box.class_id
            label.font_size = 30.0
            label.text_color = foxgloveColor(r=1.0, g=1.0, b=1.0, a=1.0)
            label.background_color = foxgloveColor(r=0.905, g=0.298, b=0.235, a=1.0)
            markers_msg.texts.append(label)

        self.publisher_image_markers.publish(markers_msg)
        
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
        results = self.model.predict(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), conf=0.7, half=True)
        if len(results) > 0 and results[0].boxes:
            results: Results = results[0].cpu()
            bbox_msg = self.parse_boxes(results)
            bbox_msg.image_header = msg.header
            self.publisher_result.publish(bbox_msg)
            
            # for debug
            annotated_frame = results.plot()
            imgMsg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.publisher_debug_image.publish(imgMsg)
            
            self.publish_image_markers(bbox_msg, msg.header)
        else:
            self.publisher_result.publish(BoundingBoxes())
            self.publisher_debug_image.publish(msg)
            self.publisher_image_markers.publish(ImageAnnotations())
            


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = rt_detr_node()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()