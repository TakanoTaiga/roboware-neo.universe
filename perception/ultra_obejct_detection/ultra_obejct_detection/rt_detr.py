import cv2
from ultralytics import RTDETR

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as rosimg
import cv_bridge

class rt_detr_node(Node):
    def __init__(self):
        super().__init__('rt_detr_node')
        self.model = RTDETR('rtdetr-l.pt')
        
        self.subscription = self.create_subscription(
            rosimg,
            '/camera/color/image_raw',
            self.listener_callback,0)
        self.publisher = self.create_publisher(rosimg, 'image_caption', 0)
        self.subscription
        self.bridge = cv_bridge.CvBridge()

    def listener_callback(self, msg):
        results = self.model.predict(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'),conf=0.7,half=True)
        annotated_frame = results[0].plot()
        imgMsg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
        self.publisher.publish(imgMsg)
        # cv2.imshow("YOLOv8 Inference", annotated_frame)
        # cv2.waitKey(1) 


def main(args=None):
    print('start ultra_obejct_detection package rt_detr node')
    rclpy.init(args=args)
    minimal_subscriber = rt_detr_node()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()