from pynput import keyboard
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopPublisher(Node):

    def __init__(self):
        super().__init__('teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self._setup_keyboard_listener()

        # initial speeds
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # speed adjustments
        self.linear_increment = 0.05
        self.angular_increment = 0.1

    def _setup_keyboard_listener(self):
        self.listener = keyboard.Listener(
            on_press=self._on_keyboard_press,
            on_release=self._on_keyboard_release)
        self.listener.start()

    def _on_keyboard_press(self, key):
        if key == keyboard.KeyCode.from_char('w'):
            self.linear_speed += self.linear_increment  # move forward
        elif key == keyboard.KeyCode.from_char('s'):
            self.linear_speed -= self.linear_increment  # move backward
        elif key == keyboard.KeyCode.from_char('a'):
            self.angular_speed += self.angular_increment  # turn left
        elif key == keyboard.KeyCode.from_char('d'):
            self.angular_speed -= self.angular_increment  # turn right
        else:
            return

        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = self.angular_speed
        self.publisher_.publish(self.twist)

    def _on_keyboard_release(self, key):
        if key in [keyboard.KeyCode.from_char(ch) for ch in 'wasd']:
            self.linear_speed = 0.0
            self.angular_speed = 0.0
            self.twist.linear.x = self.linear_speed
            self.twist.angular.z = self.angular_speed
            self.publisher_.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)

    teleop_publisher = TeleopPublisher()

    rclpy.spin(teleop_publisher)

    teleop_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

