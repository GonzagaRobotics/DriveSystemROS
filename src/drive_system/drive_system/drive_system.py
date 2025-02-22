import rclpy
import serial
from rclpy.node import Node, Subscription
from geometry_msgs.msg import Vector3


class DriveSystem(Node):
    drive_sub: Subscription
    ser: serial.Serial

    def __init__(self):
        super().__init__('drive_system')

        self.ser = self.find_serial()

        self.drive_sub = self.create_subscription(
            Vector3,
            '/drive_system/drive',
            self.drive_callback,
            10
        )

    def __del__(self):
        if self.ser.is_open:
            self.ser.close()

    def find_serial(self) -> serial.Serial:
        # TODO: We are likely going to have multiple potential
        # serial devices active, so we will also need some mechanism to find
        # the correct one.
        # TODO: we'll also have to check for permission, as /dev/ttyUSB0 defaults to root
        #   and we need to run `sudo chmod 666 /dev/ttyUSB0` when the esp32 is plugged in again
        return serial.Serial('/dev/ttyUSB0')

    def drive_callback(self, msg):
        # Extract FB and LR
        FB = msg.x
        LR = msg.y

        # Convert from doubles [-1, 1] to integers [0, 200]
        FB = round(FB * 100 + 100)
        LR = round(LR * 100 + 100)

        self.get_logger().info(f"Received command - FB: {FB} LR: {LR}")

        # TODO: Actually test this

        # Format the command and send it
        command = f"FB:{FB} LR:{LR} EN:1\n"

        try:
            self.ser.write(command.encode())
        except Exception as exc:
            self.get_logger().error(f"Failed to send command: {exc}")

        # TODO: Eventually we will want a response from the microcontroller


def main(args=None):
    rclpy.init(args=args)

    node = DriveSystem()

    node.get_logger().info(
        f"Drive System ready. Using serial port: {node.ser.name}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
