import rclpy
from rclpy.node import Node
import socket
from rw_common_msgs.msg import UDPPacketBinary

class UdpReceive(Node):
    def __init__(self):
        super().__init__('udp_receive_node')

        self.declare_parameter('port', 64203)

        self.port = self.get_parameter('port').get_parameter_value().integer_value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.setblocking(False)

        self.bin_publisher = self.create_publisher(UDPPacketBinary, 'output/udp_packet_binary', 10)
        timer_period = 0.005
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            data, (address, port) = self.sock.recvfrom(1024)
            uint8_msg = UDPPacketBinary()
            uint8_msg.data = list(data)
            uint8_msg.address = address
            uint8_msg.port = port
            self.bin_publisher.publish(uint8_msg)
        except BlockingIOError:
            pass

def main():
    rclpy.init()
    node = UdpReceive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
        
if __name__ == "__main__":
    main()
