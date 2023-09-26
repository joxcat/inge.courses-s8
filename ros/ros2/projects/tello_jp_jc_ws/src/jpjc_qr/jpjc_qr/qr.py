import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Empty
from geometry_msgs.msg import Twist, Vector3

class QrService(Node):
    def __init__(self):
        super().__init__('qr_service')
        self.get_logger().info("QR Service started")
        self.enabled = False
        self.current_scenar = ""

        self.control = self.create_publisher(
            Twist,
            '/control',
            10,
        )
        self.land = self.create_publisher(
            Empty,
            '/land',
            10,
        )
        
        self.barcode = self.create_subscription(
            String,
            '/barcode',
            self.on_barcode,
            10,
        )
        self.toggle_qr = self.create_subscription(
            Bool,
            '/toggle_qr',
            self.on_toggle_qr,
            10,
        )

    def on_toggle_qr(self, msg):
        self.enabled = msg.data

        if msg.data:
            self.get_logger().info("Enabled QR Service")
        else:
            self.get_logger().info("Disabled QR Service")

    def on_barcode(self, msg):
        if not self.enabled:
            self.get_logger().info("Barcode is disabled")
            return

        if msg.data == "start":
            self.traveling_scenar()
        elif msg.data == "finish" and self.current_scenar == "traveling":
            self.get_logger().info('End traveling')
            self.stop_drone()
        elif msg.data == "stop" and self.current_scenar != "stopping":
            self.current_scenar = "stopping"
            self.get_logger().info('SHUTDOWN')
            self.stop_drone()
            self.land.publish(Empty())
        elif msg.data == "drop_area" and self.current_scenar != "dropping":
            self.current_scenar = "dropping"
            self.get_logger().info('Landing')
            self.land.publish(Empty())
        elif msg.data == "blue_block":
            self.get_logger().info('Unimplemented blue_block')
            self
        elif msg.data == "red_block":
            self.get_logger().info('Unimplemented red_block')
            self

    def traveling_scenar(self):
        if not self.enabled:
            self.get_logger().info("Barcode is disabled")
            return
        
        if self.current_scenar == "traveling":
            return
        else:
            self.current_scenar = "traveling"

        self.get_logger().info('Traveling')

        angular = Vector3()
        angular.x = 0.0
        angular.y = 0.0
        angular.z = 0.0

        linear = Vector3()
        linear.x = 10.0
        linear.y = 0.0
        linear.z = 0.0

        msg = Twist()
        msg.angular = angular
        msg.linear = linear
 
        self.control.publish(msg)

    def stop_drone(self):
        self.current_scenar = ""

        angular = Vector3()
        angular.x = 0.0
        angular.y = 0.0
        angular.z = 0.0

        linear = Vector3()
        linear.x = 0.0
        linear.y = 0.0
        linear.z = 0.0

        msg = Twist()
        msg.angular = angular
        msg.linear = linear
 
        self.control.publish(msg)

def main():
    rclpy.init()
    qr_service = QrService()
    rclpy.spin(qr_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()