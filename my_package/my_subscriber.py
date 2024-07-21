import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('received: "%s"' % msg.data)


def main(args=None):
    try:
        rclpy.init(args=args)
        sub = MySubscriber()
        rclpy.spin(sub)

    except KeyboardInterrupt:
        print('... closing node ...')
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()