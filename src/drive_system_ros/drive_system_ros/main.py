import rclpy
from rclpy.node import Node, Subscription, Publisher, Timer
from rcs_interfaces.msg import Killswitch
from drive_system_interfaces.msg import Input, SystemStatus, WheelStatus
import serial

# How often we send data to the microcontroller
SERIAL_SEND_FREQ = 10
# Number of bytes we send to the serial port
SERIAL_TX_LEN = 12
# Number of bytes we read from the serial port
SERIAL_RX_LEN = 100


class DriveSystemROS(Node):
    serial_port: serial.Serial
    killswitch_sub: Subscription
    input_sub: Subscription
    status_pub: Publisher
    send_timer: Timer

    need_send_serial: bool = False
    last_enable: bool = True
    last_input: Input = Input()

    need_send_ros: bool = False
    last_status: SystemStatus = SystemStatus()

    def __init__(self):
        super().__init__("drive_system_ros")

        serial_port_param = self.declare_parameter(
            "serial_port",
            descriptor="Serial port the microcontroller is connected to"
        )

        if not serial_port_param.value:
            self.get_logger().error("Serial port parameter not set")
            rclpy.shutdown()
            return

        self.serial_port = serial.Serial(serial_port_param.value, timeout=0)
        self.send_data()

        self.killswitch_sub = self.create_subscription(
            Killswitch,
            "/killswitch",
            self.killswitch_callback,
            10
        )

        self.input_sub = self.create_subscription(
            Input,
            "/drive_system/input",
            self.input_callback,
            10
        )

        self.status_pub = self.create_publisher(
            SystemStatus,
            "/drive_system/status",
            10
        )

        self.send_timer = self.create_timer(
            1 / SERIAL_SEND_FREQ,
            self.send_timer_callback
        )

    def send_data(self):
        bytes_sending = bytearray(SERIAL_TX_LEN)

        bytes_sending[0:2] = b"EN"
        bytes_sending[3] = 1 if self.last_enable else 0

        bytes_sending[4:5] = b"FB"
        forwards_backwards = int(self.last_input.forwards_backwards * 32767)
        bytes_sending[6] = (forwards_backwards >> 8) & 0xFF
        bytes_sending[7] = forwards_backwards & 0xFF

        bytes_sending[8:9] = b"LR"
        left_right = int(self.last_input.left_right * 32767)
        bytes_sending[10] = (left_right >> 8) & 0xFF
        bytes_sending[11] = left_right & 0xFF

        self.serial_port.write(bytes_sending)

    def killswitch_callback(self, msg: Killswitch):
        self.last_enable = msg.enable
        self.need_send_serial = True

    def input_callback(self, msg: Input):
        self.last_input = msg
        self.need_send_serial = True

    def send_timer_callback(self):
        if self.need_send_serial:
            self.send_data()
            self.need_send_serial = False

        if self.need_send_ros:
            self.status_pub.publish(self.last_status)
            self.need_send_ros = False

    def check_for_data(self):
        bytes_received = self.serial_port.read(SERIAL_RX_LEN)

        if len(bytes_received) != SERIAL_RX_LEN:
            self.get_logger().warn("Received %d bytes, expected %d",
                                   len(bytes_received), SERIAL_RX_LEN)
            return

        status_msg = SystemStatus()

        status_msg.enabled = bytes_received[3] == 1

        for i in range(6):
            status_msg.wheels[i] = self.decode_wheel_status(bytes_received, i)

        self.last_status = status_msg
        self.need_send_ros = True

    def decode_wheel_status(self, bytes_received: bytes, index: int) -> WheelStatus:
        wheel_status = WheelStatus()

        start = 4 + index * 16

        wheel_status.desired_power = int.from_bytes(
            bytes_received[start + 2:start + 4],
            byteorder="little",
            signed=True
        )

        wheel_status.actual_power = int.from_bytes(
            bytes_received[start + 6:start + 8],
            byteorder="little",
            signed=True
        )

        wheel_status.current_rpm = int.from_bytes(
            bytes_received[start + 10:start + 12],
            byteorder="little",
            signed=True
        )

        wheel_status.control_active = bytes_received[start + 15] == 1

        return wheel_status

    def __del__(self):
        # Quickly send a message to the microcontroller to disable the motors
        self.last_enable = False
        self.send_data()

        super().__del__()


def main(args=None):
    rclpy.init(args=args)
    node = DriveSystemROS()

    try:
        while rclpy.ok():
            node.check_for_data()
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
