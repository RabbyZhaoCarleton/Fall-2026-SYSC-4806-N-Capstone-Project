import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math

class MecanumBridgeNode(Node):
    def __init__(self):
        super().__init__('mecanum_bridge')

        # Serial port configuration (Check with `ls /dev/ttyUSB*` or `/dev/ttyACM*`)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)

        # Robot geometry & encoder specs (Must match ESP32 & physical hardware)
        self.wheel_radius = 0.040  # meters
        self.lx = 0.075              # meters
        self.ly = 0.10              # meters
        self.ticks_per_rev = 1320.0 # Encoder ticks per full wheel revolution

        # Pose state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # ROS 2 Subscribers, Publishers, and TF Broadcaster
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Read serial data at 50Hz
        self.timer = self.create_timer(0.02, self.read_serial_callback)
        self.get_logger().info('Mecanum Serial Bridge started.')

    def cmd_vel_callback(self, msg: Twist):
        # Send Twist command to ESP32
        cmd_str = f"<{msg.linear.x:.3f},{msg.linear.y:.3f},{msg.angular.z:.3f}>\n"
        self.ser.write(cmd_str.encode('utf-8'))

    def read_serial_callback(self):
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('E,'):
                parts = line.split(',')
                if len(parts) == 5:
                    try:
                        ticks = [int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4])]
                        self.publish_odometry(ticks)
                    except ValueError:
                        pass

    def publish_odometry(self, ticks):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = current_time

        # Convert delta ticks to wheel angular velocities (rad/s)
        rad_per_tick = (2.0 * math.pi) / self.ticks_per_rev
        w_fl = (ticks[0] * rad_per_tick) / dt
        w_fr = (ticks[1] * rad_per_tick) / dt
        w_rl = (ticks[2] * rad_per_tick) / dt
        w_rr = (ticks[3] * rad_per_tick) / dt

        # Mecanum Forward Kinematics
        geom = self.lx + self.ly
        vx = (self.wheel_radius / 4.0) * (w_fl + w_fr + w_rl + w_rr)
        vy = (self.wheel_radius / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
        wz = (self.wheel_radius / (4.0 * geom)) * (-w_fl + w_fr - w_rl + w_rr)

        # Integrate pose in world frame
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = wz * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Quaternion from yaw
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        # 1. Broadcast TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # 2. Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
