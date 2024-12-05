#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist  # For velocity commands
from std_msgs.msg import Bool        # For enabling/disabling traction control
import serial
import time

class RoverMotorControl:
    def __init__(self, port="/dev/ttyUSB0", baudrate=9600):
        # Initialize ROS node
        rospy.init_node('rover_motor_control', anonymous=True)
        
        # Serial communication setup
        self.serial_connection = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Give some time for the serial connection to stabilize

        # ROS subscribers
        rospy.Subscriber("/cmd_vel", Twist, self.command_callback)  # For motor speed commands
        rospy.Subscriber("/traction_control", Bool, self.traction_control_callback)  # Enable/disable traction control

        # Internal variables
        self.speed_fb = 0  # Forward/backward speed
        self.speed_lr = 0  # Left/right speed
        self.traction_enabled = 0

    def command_callback(self, msg):
        """Callback for velocity commands."""
        self.speed_fb = int(msg.linear.x * 100)  # Scale from m/s to -100 to 100
        self.speed_lr = int(msg.angular.z * 100)  # Scale from rad/s to -100 to 100
        self.send_motor_command()

    def traction_control_callback(self, msg):
        """Callback for traction control enable/disable."""
        self.traction_enabled = 1 if msg.data else 0
        self.send_motor_command()

    def send_motor_command(self):
        """Send motor command to the Arduino."""
        command = f"FB:{self.speed_fb} LR:{self.speed_lr} EN:{self.traction_enabled}\n"
        rospy.loginfo(f"Sending command: {command.strip()}")
        self.serial_connection.write(command.encode())

    def read_feedback(self):
        """Read feedback from the Arduino."""
        if self.serial_connection.in_waiting > 0:
            feedback = self.serial_connection.readline().decode('utf-8').strip()
            if feedback:
                rospy.loginfo(f"Arduino feedback: {feedback}")

    def run(self):
        """Main loop."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.read_feedback()
            rate.sleep()

if __name__ == "__main__":
    try:
        # Replace '/dev/ttyUSB0' with the appropriate port for your Arduino
        rover = RoverMotorControl(port="/dev/ttyUSB0", baudrate=9600)
        rover.run()
    except rospy.ROSInterruptException:
        pass
