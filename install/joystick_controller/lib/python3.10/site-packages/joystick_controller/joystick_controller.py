import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Joystick Controller Node Started')

        # Kecepatan gerakan robot
        self.linear_speed = 1.0  # Kecepatan linear (m/s)
        self.angular_speed = 1.0  # Kecepatan angular (rad/s)

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Membuat objek Twist untuk mengirimkan perintah gerakan
        self.twist = Twist()

    def joy_callback(self, msg):
        # Input joystick (msg.axes dan msg.buttons) berisi data input joystick
        # axes: 0-> X axis, 1-> Y axis, 2-> Z axis, dst.
        # buttons: 0 -> A, 1 -> B, dst.

        # Set linear.x berdasarkan tombol joystick (misalnya, tombol untuk maju/mundur)
        self.twist.linear.x = self.linear_speed * msg.axes[1]  # Sumbu Y untuk maju/mundur (analogi W/S)

        # Set angular.z berdasarkan tombol joystick (misalnya, tombol untuk berputar kiri/kanan)
        self.twist.angular.z = self.angular_speed * msg.axes[0]  # Sumbu X untuk belok kiri/kanan (analogi A/D)

        # Terbitkan perintah gerakan ke topik /cmd_vel
        self.publisher_.publish(self.twist)
        self.get_logger().info(f'Publishing: linear.x={self.twist.linear.x}, angular.z={self.twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
