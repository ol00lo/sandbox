import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from pynput import mouse
import time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange, SetParametersResult

class MouseSensor(Node):
    def __init__(self):
        super().__init__('MouseSensor')
        self.MAX_WINDOW_SIZE = 100000
        interval_description = ParameterDescriptor(
            name= "minimum_time_interval_ms",
            type= ParameterType.PARAMETER_INTEGER,
            description= "Minimum interval between topics in milliseconds",
            integer_range= [IntegerRange(from_value= 0, to_value= 10000)]
        )

        roi_description = ParameterDescriptor(
            name= "region_of_interest",
            type= ParameterType.PARAMETER_INTEGER_ARRAY,
            description= "Region of interest as [x0, y0, x1, y1] screen pixel coordinates",
            integer_range= [IntegerRange(from_value=0, to_value= self.MAX_WINDOW_SIZE)]
        )

        self.declare_parameter("minimum_time_interval_ms",10, descriptor=interval_description)
        self.declare_parameter("region_of_interest",[0, 0, self.MAX_WINDOW_SIZE, self.MAX_WINDOW_SIZE], descriptor=roi_description)

        self.add_on_set_parameters_callback(self.parameters_callback)
        self.update_time_interval()
        self.update_roi()

        self._publisher_mouse_moved = self.create_publisher(Point, 'mouse_moved', 10)
        self._listener = mouse.Listener(on_move=self.on_move)
        self._listener.start()

        self.__last_call = 0

    def parameters_callback(self, params):
        result = SetParametersResult(successful=True)
        try:
            for p in params:
                if p.name == "minimum_time_interval_ms":
                    self.update_time_interval(p.value)
                elif p.name == "region_of_interest":
                    self.update_roi(p.value)
        except Exception as e:
            result.successful = False
            result.reason = f"Parameter update failed: {str(e)}"
            self.get_logger().error(f"Parameter error: {e}")
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
        msg.z = float(tm)
        

        self._publisher_mouse_moved.publish(msg)
        self.get_logger().debug(f'Publishing: x={msg.x}, y={msg.y}, current time = {msg.z}')

    def update_time_interval(self, value=None):
        if value is None:
            value = self.get_parameter('minimum_time_interval_ms').value
        self.interval = value / 1000
        self.get_logger().info(f"Listen to mouse move with interval={int(self.interval*1000)}ms")

    def update_roi(self, value=None):
        if value is None:
            value = self.get_parameter_or(
                "region_of_interest",
                rclpy.parameter.Parameter("region_of_interest", rclpy.Parameter.Type.INTEGER_ARRAY, [])
                ).value

        if not value:
            value = [0, 0, self.MAX_WINDOW_SIZE, self.MAX_WINDOW_SIZE]

        if len(value) != 4:
            raise ValueError("ROI must contain exactly 4 values [x0,y0,x1,y1]")
        if value[0] >= value[2] or value[1] >= value[3]:
            raise ValueError(f"Invalid coordinates: x0({value[0]}) >= x1({value[2]}) or y0({value[1]}) >= y1({value[3]})")

        self.roi = value
        self.get_logger().info(f"ROI updated to {self.roi}")


def main(args=None):
    rclpy.init(args=args)
    mouse_sensor = MouseSensor()
    rclpy.spin(mouse_sensor)
    mouse_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
