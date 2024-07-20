import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_ = self.create_publisher(String, 'output/data', 10)
        self.serial_port = serial.Serial('/dev/tty.usbmodem101', 230400, timeout=0.01)
        self.timer = self.create_timer(0.01, self.read_serial_data)

    def read_serial_data(self):
        latest_data = None
        while self.serial_port.in_waiting > 0:
            latest_data = self.serial_port.readline().decode('utf-8').strip()

        if latest_data:
            msg = String()
            msg.data = latest_data
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()