import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5 #sec
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'message # %d' %self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    try: 
        rclpy.init(args=args)
        publisher = MyPublisher()
        rclpy.spin(publisher)

    except KeyboardInterrupt:
        print(' ... exit node')
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()