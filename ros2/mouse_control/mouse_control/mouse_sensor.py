import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from pynput import mouse

class MouseSensor(Node):
    def __init__(self):
        super().__init__('MouseSensor')
        self._publisher_mouse_moved = self.create_publisher(Point, 'mouse_moved', 10)
        self._listener = mouse.Listener(on_move=self.on_move)
        self._listener.start()

    def on_move(self, x, y):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        self._publisher_mouse_moved.publish(msg)
        self.get_logger().debug(f'Publishing: x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    mouse_sensor = MouseSensor()
    rclpy.spin(mouse_sensor)
    mouse_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
