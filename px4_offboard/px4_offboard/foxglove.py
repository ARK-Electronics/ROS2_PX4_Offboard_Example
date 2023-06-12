import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import VehicleStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class MySubscriber(Node):

    def __init__(self):

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        super().__init__('my_subscriber')
        self.gps_subscription = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_gps_position',
            self.listener_callback,
            qos_profile)
        self.gps_subscription  # prevent unused variable warning

        # super().__init__('my_status')
        # self.subscription = self.create_subscription(
        #     VehicleStatus,
        #     '/fmu/out/vehicle_status',
        #     self.status_callback,
        #     qos_profile)
        # self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.lat)

    
    

    # def status_callback(self, msg):
    #     self.get_logger().info('I heard: \n')


def main(args=None):
    rclpy.init(args=args)

    my_subscriber = MySubscriber()

    rclpy.spin(my_subscriber)

    my_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()