import rclpy
import serial
from rclpy.node import Node
from example_interfaces.msg import Float32MultiArray

class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller_node')

        # Set up the serial connection
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust port as needed

        # Create a subscriber for the speed topic
        self.subscription = self.create_subscription(
            Float32MultiArray,  # Expecting an array of two floats [FB, LR]
            'speed_topic',      # Topic name
            self.speed_callback,
            10                  # Queue size
        )

    def speed_callback(self, msg):
        # Extract the FB and LR values from the message
        FB, LR = msg.data
        self.get_logger().info(f"Received speeds - FB: {FB} LR: {LR}")
        signal = f"FB:{FB} LR:{LR}\n"  # Format the signal
        self.ser.write(signal.encode())  # Send the signal as bytes
        self.get_logger().info(f"Sent to serial: {signal.strip()}")

        # Listen for response from the microcontroller
        response = self.ser.readline().decode().strip()  # Read and decode response
        if response:
            self.get_logger().info(f"Received from microcontroller: {response}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        # Close the serial connection on exit
        if node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
