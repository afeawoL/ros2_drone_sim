import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import numpy as np
from math import sin, cos, pi
import time

class DronePhysics(Node):
    def __init__(self):
        super().__init__('drone_physics')
        
        # Physical parameters
        self.mass = 1.0  # kg
        self.gravity = 9.81  # m/s^2
        self.max_thrust = 20.0  # N
        self.drag_coeff = 0.1
        self.angular_drag_coeff = 0.2
        
        # State variables
        self.position = np.zeros(3)  # x, y, z
        self.velocity = np.zeros(3)  # vx, vy, vz
        self.acceleration = np.zeros(3)  # ax, ay, az
        self.orientation = np.zeros(3)  # roll, pitch, yaw
        self.angular_velocity = np.zeros(3)  # wx, wy, wz
        
        # PID Controllers
        self.pos_pid = {
            'P': 2.0,
            'I': 0.1,
            'D': 1.0,
            'error_sum': np.zeros(3),
            'last_error': np.zeros(3)
        }
        self.att_pid = {
            'P': 3.0,
            'I': 0.2,
            'D': 1.5,
            'error_sum': np.zeros(3),
            'last_error': np.zeros(3)
        }
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'drone/odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'drone/imu', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for physics update (100 Hz)
        self.update_timer = self.create_timer(0.01, self.update_physics)
        
        # Time tracking
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        # Convert twist message to force and torque targets
        self.target_velocity = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        self.target_angular_velocity = np.array([msg.angular.x, msg.angular.y, msg.angular.z])

    def update_physics(self):
        # Time step calculation
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Apply PID control for stability
        force = self.compute_control_force(dt)
        torque = self.compute_control_torque(dt)
        
        # Update linear motion
        gravity_force = np.array([0, 0, -self.mass * self.gravity])
        drag_force = -self.drag_coeff * self.velocity * np.abs(self.velocity)
        net_force = force + gravity_force + drag_force
        
        self.acceleration = net_force / self.mass
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt
        
        # Update angular motion
        angular_drag = -self.angular_drag_coeff * self.angular_velocity
        net_torque = torque + angular_drag
        
        angular_acceleration = net_torque  # Assuming unit inertia tensor for simplicity
        self.angular_velocity += angular_acceleration * dt
        self.orientation += self.angular_velocity * dt
        
        # Normalize orientation angles
        self.orientation = np.mod(self.orientation + pi, 2 * pi) - pi
        
        # Publish state
        self.publish_transform()
        self.publish_odometry()
        self.publish_imu()

    def compute_control_force(self, dt):
        error = np.zeros(3) - self.position  # Hover at origin
        self.pos_pid['error_sum'] += error * dt
        derivative = (error - self.pos_pid['last_error']) / dt
        self.pos_pid['last_error'] = error
        
        force = (self.pos_pid['P'] * error +
                self.pos_pid['I'] * self.pos_pid['error_sum'] +
                self.pos_pid['D'] * derivative)
        
        # Limit maximum force
        force_magnitude = np.linalg.norm(force)
        if force_magnitude > self.max_thrust:
            force = force * self.max_thrust / force_magnitude
        
        return force

    def compute_control_torque(self, dt):
        error = np.zeros(3) - self.orientation  # Target level orientation
        self.att_pid['error_sum'] += error * dt
        derivative = (error - self.att_pid['last_error']) / dt
        self.att_pid['last_error'] = error
        
        torque = (self.att_pid['P'] * error +
                self.att_pid['I'] * self.att_pid['error_sum'] +
                self.att_pid['D'] * derivative)
        
        return torque

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = float(self.position[0])
        t.transform.translation.y = float(self.position[1])
        t.transform.translation.z = float(self.position[2])
        
        # Simplified quaternion from Euler angles
        cy = cos(self.orientation[2] * 0.5)
        sy = sin(self.orientation[2] * 0.5)
        cp = cos(self.orientation[1] * 0.5)
        sp = sin(self.orientation[1] * 0.5)
        cr = cos(self.orientation[0] * 0.5)
        sr = sin(self.orientation[0] * 0.5)
        
        t.transform.rotation.w = cy * cp * cr + sy * sp * sr
        t.transform.rotation.x = cy * cp * sr - sy * sp * cr
        t.transform.rotation.y = sy * cp * sr + cy * sp * cr
        t.transform.rotation.z = sy * cp * cr - cy * sp * sr
        
        self.tf_broadcaster.sendTransform(t)

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = float(self.position[0])
        odom.pose.pose.position.y = float(self.position[1])
        odom.pose.pose.position.z = float(self.position[2])
        
        cy = cos(self.orientation[2] * 0.5)
        sy = sin(self.orientation[2] * 0.5)
        cp = cos(self.orientation[1] * 0.5)
        sp = sin(self.orientation[1] * 0.5)
        cr = cos(self.orientation[0] * 0.5)
        sr = sin(self.orientation[0] * 0.5)
        
        odom.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr
        odom.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr
        odom.pose.pose.orientation.y = sy * cp * sr + cy * sp * cr
        odom.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr
        
        odom.twist.twist.linear.x = float(self.velocity[0])
        odom.twist.twist.linear.y = float(self.velocity[1])
        odom.twist.twist.linear.z = float(self.velocity[2])
        odom.twist.twist.angular.x = float(self.angular_velocity[0])
        odom.twist.twist.angular.y = float(self.angular_velocity[1])
        odom.twist.twist.angular.z = float(self.angular_velocity[2])
        
        self.odom_pub.publish(odom)

    def publish_imu(self):
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        
        # Orientation (same as in transform)
        cy = cos(self.orientation[2] * 0.5)
        sy = sin(self.orientation[2] * 0.5)
        cp = cos(self.orientation[1] * 0.5)
        sp = sin(self.orientation[1] * 0.5)
        cr = cos(self.orientation[0] * 0.5)
        sr = sin(self.orientation[0] * 0.5)
        
        imu.orientation.w = cy * cp * cr + sy * sp * sr
        imu.orientation.x = cy * cp * sr - sy * sp * cr
        imu.orientation.y = sy * cp * sr + cy * sp * cr
        imu.orientation.z = sy * cp * cr - cy * sp * sr
        
        imu.angular_velocity.x = float(self.angular_velocity[0])
        imu.angular_velocity.y = float(self.angular_velocity[1])
        imu.angular_velocity.z = float(self.angular_velocity[2])
        
        imu.linear_acceleration.x = float(self.acceleration[0])
        imu.linear_acceleration.y = float(self.acceleration[1])
        imu.linear_acceleration.z = float(self.acceleration[2] - self.gravity)
        
        self.imu_pub.publish(imu)

def main(args=None):
    rclpy.init(args=args)
    node = DronePhysics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

