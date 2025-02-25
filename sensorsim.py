import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        self.timer = self.create_timer(1.0, self.publish_sensor_data)

    def publish_sensor_data(self):
        # Format: gate_seen,rotation,gate_centered,flare_in_way,too_close_to_side_wall,yaw
        gate_seen = random.choice([True, False])
        rotation = random.uniform(-20, 20) if gate_seen else 0.0
        gate_centered = random.choice([True, False]) if gate_seen else False
        flare_in_way = gate_centered and random.choice([True, False])
        too_close = random.choice([True, False])
        yaw = random.uniform(-180, 180)
        
        msg = String()
        msg.data = f"{gate_seen},{rotation:.1f},{gate_centered},{flare_in_way},{too_close},{yaw:.1f}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published sensor data: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
