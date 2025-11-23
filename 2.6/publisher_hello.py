import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):

        super().__init__("publisher_node")
        self.publisher = self.create_publisher(String, 'hello_topic', 1)
        self.timer = self.create_timer(1.0, self.publish_hello)
    
    def publish_hello(self):
        msg = String()
        msg.data = "Hello world!"
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = PublisherNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()