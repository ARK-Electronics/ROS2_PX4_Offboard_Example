import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import SensorGps
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class MySubscriber(Node):

    def __init__(self):

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        super().__init__('minimal_publisher')
        
        self.gps_subscription = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.Sensor_GPS_callback,
            qos_profile)
        
        self.publisher_nav_sat_fix = self.create_publisher(NavSatFix, '/nav_sat_fix', qos_profile)

    def Sensor_GPS_callback(self, msg):
        self.get_logger().info('Sensor GPS: "%s"\n\n' % msg)
        nav_sat_fix_msg = NavSatFix()

        # nav_sat_fix_msg.header._stamp = msg.timestamp
        nav_sat_fix_msg.header.stamp.sec = int(msg.timestamp / 1e6)
        nav_sat_fix_msg.header.stamp.nanosec = int((msg.timestamp % 1e6) * 1e3)
        nav_sat_fix_msg.header._frame_id = 'PX4_Frame_ID'

        nav_sat_fix_msg.latitude = float(msg.lat)
        nav_sat_fix_msg.longitude = float(msg.lon)
        nav_sat_fix_msg.altitude = float(msg.alt)

        # ChatGPT's solution to covariance
        nav_sat_fix_msg.position_covariance[0] = float(msg.eph**2)  # Variance in East direction
        nav_sat_fix_msg.position_covariance[4] = float(msg.eph**2)  # Variance in North direction
        nav_sat_fix_msg.position_covariance[8] = float(msg.epv**2)  # Variance in Up direction

        # DroneCAN Fix 2 solution
        # nav_sat_fix_msg.position_covariance[0] = float(msg.eph)  # Variance in East direction
        # nav_sat_fix_msg.position_covariance[1] = float(msg.eph)  # Variance in North direction
        # nav_sat_fix_msg.position_covariance[2] = float(msg.epv)  # Variance in Up direction

        # nav_sat_fix_msg.position_covariance[3] = float(msg.s_variance_m_s)  # Variance in East direction
        # nav_sat_fix_msg.position_covariance[4] = float(msg.s_variance_m_s)  # Variance in North direction
        # nav_sat_fix_msg.position_covariance[5] = float(msg.s_variance_m_s)  # Variance in Up direction

        # Set covariance type
        nav_sat_fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        #publish message
        self.publisher_nav_sat_fix.publish(nav_sat_fix_msg)
        


def main(args=None):
    rclpy.init(args=args)

    my_subscriber = MySubscriber()

    rclpy.spin(my_subscriber)

    my_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()