import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        timer_period = 0.5 #sec
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        print('Saludos desde ROS2!')


def main(args=None):
    try:
        rclpy.init(args=args)
        my_node = MyNode()
        rclpy.spin(my_node)
        
    except KeyboardInterrupt:
        print(' ... exit node')
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()