import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):

        super().__init__("listener_node")
        self.subscription = self.create_subscription(
            String, 
            "hello_topic",
            self.listener_log,
            1
        )
    
    def listener_log(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

def main():
    rclpy.init()
    node = ListenerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()