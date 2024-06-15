import rclpy
from rclpy.node import Node
import socket
from rw_common_msgs.msg import UDPPacketBinary

class UdpSend(Node):
    def __init__(self):
        super().__init__('udp_send_node')
        self.sub_udp_bin =self.create_subscription(
            UDPPacketBinary, '/input/udp_packet_binary', self.sub_callback, 10
        )
    
    def sub_callback(self, msg):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
            destination_address = (msg.address, msg.port)
            udp_socket.sendto(msg.data, destination_address)
        
def main():
    rclpy.init()
    node = UdpSend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
        
if __name__ == "__main__":
    main()
