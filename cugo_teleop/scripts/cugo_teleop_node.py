import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import serial

MAX_SPEED = 0.83 # 3km/h
WHEEL_BASE = 0.28 

class CugoTeleop(Node):
  def __init__(self):
    super().__init__("cugo_teleop_node")

    self.ser = serial.Serial('/dev/ttyACM1', 115200)

    self.twist_subscriber = self.create_subscription(Twist, "cmd_vel", self.callback, 10)

  def callback(self, msg):
    wheel_vel = [0.0, 0.0]
    target_vel = [0, 0]

    wheel_vel[0] = msg.linear.x + msg.angular.z#msg.linear.x + (msg.angular.z * (WHEEL_BASE / 2.0))
    wheel_vel[1] = msg.linear.x - msg.angular.z#msg.linear.x - (msg.angular.z * (WHEEL_BASE / 2.0))

    target_vel[0] = int(wheel_vel[0] / MAX_SPEED * 100)
    target_vel[1] = int(wheel_vel[1] / MAX_SPEED * 100)

    target_vel[0] = min(max(-20, target_vel[0]), 20)
    target_vel[1] = min(max(-20, target_vel[1]), 20)

    target_str = '{},{}\n'.format(int(target_vel[0]), int(target_vel[1]))

    self.ser.write(target_str.encode())

def main(args=None):
   rclpy.init(args=args)
   node = CugoTeleop()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
    main()
