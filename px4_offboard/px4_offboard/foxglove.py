import rclpy
from rclpy.node import Node

from px4_msgs.msg import SensorGps
from px4_msgs.msg import VehicleAcceleration
from px4_msgs.msg import VehicleAngularVelocity

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Accel

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# information for the mentioned topics can be found at
#   https://github.com/ros2/common_interfaces/tree/humble
#   https://github.com/PX4/PX4-Autopilot/tree/main/msg

class MySubscriber(Node):

    def __init__(self):

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        super().__init__('minimal_publisher')
        
        ################################################################
        # vehicle_gps_position(SensorGps.msg) -> NavSatFix.msg
        self.gps_subscription = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.Sensor_GPS_callback,
            qos_profile)
        
        self.publisher_nav_sat_fix = self.create_publisher(NavSatFix, '/nav_sat_fix', qos_profile)
        ################################################################

        ################################################################
        # VehicleAcceleration.msg -> Accel.msg
        self.acceleration_subscription = self.create_subscription(
            VehicleAcceleration,
            '/fmu/out/vehicle_acceleration',
            self.Vehicle_Acceleration_callback,
            qos_profile)
        
        Vehicle_Acceleration_X = 0.0
        Vehicle_Acceleration_Y = 0.0
        Vehicle_Acceleration_Z = 0.0
        
        self.angular_velocity_subscription = self.create_subscription(
            VehicleAngularVelocity,
            '/fmu/out/vehicle_angular_velocity',
            self.Vehicle_Angular_Velocity_callback,
            qos_profile)
    

        self.publisher_accel = self.create_publisher(Accel, '/accel', qos_profile)


        ################################################################

    def Sensor_GPS_callback(self, msg):
        # self.get_logger().info('Sensor GPS: "%s"\n\n' % msg)
        nav_sat_fix_msg = NavSatFix()

        # nav_sat_fix_msg.header._stamp = msg.timestamp
        nav_sat_fix_msg.header.stamp.sec = int(msg.timestamp / 1e6)
        nav_sat_fix_msg.header.stamp.nanosec = int((msg.timestamp % 1e6) * 1e3)
        nav_sat_fix_msg.header._frame_id = 'PX4_Frame_ID'

        nav_sat_fix_msg.latitude = float(msg.lat / 1e7)
        nav_sat_fix_msg.longitude = float(msg.lon / 1e7)
        nav_sat_fix_msg.altitude = float(msg.alt / 1e4)

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
        
    def Vehicle_Acceleration_callback(self, msg):
        # self.get_logger().info('Vehicle Acceleration: "%s"\n\n' % msg)
        self.Vehicle_Acceleration_X = float(msg.xyz[0])
        self.Vehicle_Acceleration_Y = float(msg.xyz[1])
        self.Vehicle_Acceleration_Z = float(msg.xyz[2])

    def Vehicle_Angular_Velocity_callback(self, msg):
        # self.get_logger().info('Vehicle Angular Velocity: "%s"\n\n' % msg)
        accel_msg = Accel()

        # NED -> ENU
        accel_msg.linear.x = self.Vehicle_Acceleration_Y
        accel_msg.linear.y = self.Vehicle_Acceleration_X
        accel_msg.linear.z = -self.Vehicle_Acceleration_Z

        accel_msg.angular.x = float(msg.xyz[1])
        accel_msg.angular.y = float(msg.xyz[0])
        accel_msg.angular.z = -float(msg.xyz[2])

        #publish message
        self.publisher_accel.publish(accel_msg)

def main(args=None):
    rclpy.init(args=args)

    my_subscriber = MySubscriber()

    rclpy.spin(my_subscriber)

    my_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()