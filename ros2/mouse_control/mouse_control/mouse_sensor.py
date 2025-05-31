import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from pynput import mouse

class MouseControl(Node):
    def __init__(self):
        super().__init__('mouse_control')
        self.publisher_ = self.create_publisher(Point, 'mouse_coordinates', 10)
        self.listener = mouse.Listener(on_move=self.on_move)
        self.listener.start()

    def on_move(self, x, y):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    mouse_control = MouseControl()
    rclpy.spin(mouse_control)
    mouse_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
