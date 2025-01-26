import rclpy
import serial
from rclpy.node import Node
from example_interfaces.msg import Float32MultiArray

class SpeedControllerNode(Node):
    def __init__(self):
        super().__init__('speed_controller_node')

        self.serial_port = self.find_serial_port()
        if not self.serial_port:
            self.get_logger().error("No serial ports detected")
            rclpy.shutdown()
            return

            #serial connection
            self.ser = serial.Serial(self.serial_port, 9600, timeout=1)
            self.get_logger().info(f"Connected to serial port: {self.serial_port}")

        self.subscription = self.create_subscription(
            Float32MultiArray,  # Expecting an array of two floats [FB, LR]
            'speed_topic',    
            self.speed_callback,
            10              
        )

    def speed_callback(self, msg):
        # Extract FB and LR 
        FB, LR = msg.data
        self.get_logger().info(f"Received speeds - FB: {FB} LR: {LR}")
        signal = f"FB:{FB} LR:{LR} EN:1\n"  # Format the signal
        self.ser.write(signal.encode())  # Send the signal as bytes
        self.get_logger().info(f"Sent to serial: {signal.strip()}")

        # response from the microcontroller
        response = self.ser.readline().decode().strip()
        if response:
            self.get_logger().info(f"Received from microcontroller: {response}")
        return
    
def main(args=None):
    rclpy.init(args=args)
    node = SpeedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        if node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
