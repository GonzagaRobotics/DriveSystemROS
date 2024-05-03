import rclpy
from rclpy.node import Node, Subscription, Publisher
from rcs_interfaces.msg import Killswitch
from drive_system_interfaces.msg import Input, SystemStatus, WheelStatus
import serial

# Number of bytes we send to the serial port
SERIAL_TX_LEN = 12
# Number of bytes we read from the serial port
SERIAL_RX_LEN = 100


def main(args=None):
    rclpy.init(args=args)
    node = Node("drive_system_ros")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
