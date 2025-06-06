import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from pynput import mouse
import time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange, SetParametersResult

class MouseSensor(Node):
    def __init__(self):
        super().__init__('MouseSensor')

        interval_description = ParameterDescriptor(
            name= "minimum_time_interval_ms",
            type= ParameterType.PARAMETER_INTEGER,
            description= "Minimum interval between topics in milliseconds",
            integer_range= [IntegerRange(from_value= 0, to_value= 10000)]
        )

        roi_description = ParameterDescriptor(
            name= "region_of_interest",
            type= ParameterType.PARAMETER_INTEGER_ARRAY,
            description= "Region of interest as [x0, y0, x1, y1] coordinates",
            integer_range= [IntegerRange(from_value= 0, to_value= 2000)]
        )

        self.declare_parameter("minimum_time_interval_ms", 10000, interval_description)
        self.declare_parameter("region_of_interest", [0, 0, 0, 0], roi_description)

        self.interval = self.get_parameter("minimum_time_interval_ms").value/1000
        self.roi = self.get_parameter("region_of_interest").value

        self.add_on_set_parameters_callback(self.parameters_callback)
        self.update_time_interval()

        self._publisher_mouse_moved = self.create_publisher(Point, 'mouse_moved', 10)
        self._listener = mouse.Listener(on_move=self.on_move)
        self._listener.start()

        self.__last_call = 0

    def parameters_callback(self, params):
        result = SetParametersResult(successful=True)
        for p in params:
            if p.name == "minimum_time_interval_ms":
                self.update_time_interval(p.value)
            elif p.name == "region_of_interest":
                self.update_roi(p.value)

        return result

    def on_move(self, x, y):
        if not (self.roi[0]<=x<=self.roi[2] and self.roi[1]<=y<=self.roi[3]):
            return

        tm = time.time()
        if abs(tm - self.__last_call) < self.interval:
            return

        self.__last_call = tm

        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        self._publisher_mouse_moved.publish(msg)
        self.get_logger().debug(f'Publishing: x={msg.x}, y={msg.y}')

    def update_time_interval(self, value=None):
        if value is None:
            value = self.get_parameter('minimum_time_interval_ms').value
        self.interval = value / 1000
        self.get_logger().info(f"Listen to mouse move with interval={int(self.interval*1000)}ms")

    def update_roi(self, value):
        if len(value)==4:
            self.roi = value
            self.get_logger().info(f"ROI update {self.roi}")

def main(args=None):
    rclpy.init(args=args)
    mouse_sensor = MouseSensor()
    rclpy.spin(mouse_sensor)
    mouse_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
