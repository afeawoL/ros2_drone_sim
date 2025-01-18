#!/usr/bin/env python3
from enum import Enum, auto
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import asyncio
import sys
import termios
import tty
from threading import Thread

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Define drone states
        class DroneState(Enum):
            IDLE = auto()
            TAKEOFF = auto()
            HOVER = auto()
            LANDING = auto()
            EMERGENCY = auto()
            
        # Constants and parameters
        self.declare_parameter('max_velocity', 2.0,
                            ParameterDescriptor(description='Maximum velocity in m/s'))
        self.declare_parameter('max_altitude', 3.0,
                            ParameterDescriptor(description='Maximum altitude in meters'))
        self.declare_parameter('velocity_step', 0.1,
                            ParameterDescriptor(description='Velocity increment per keystroke'))
        self.declare_parameter('takeoff_velocity', 0.5,
                            ParameterDescriptor(description='Takeoff velocity in m/s'))
        self.declare_parameter('landing_velocity', 0.3,
                            ParameterDescriptor(description='Landing velocity in m/s'))
        
        # Get parameters
        self.MAX_VELOCITY = self.get_parameter('max_velocity').value
        self.MAX_ALTITUDE = self.get_parameter('max_altitude').value
        self.VELOCITY_STEP = self.get_parameter('velocity_step').value
        self.TAKEOFF_VELOCITY = self.get_parameter('takeoff_velocity').value
        self.LANDING_VELOCITY = self.get_parameter('landing_velocity').value
        
        # Control parameters
        self.velocity_ramp_rate = 0.1  # Rate of velocity change
        self.min_altitude = 0.1  # Minimum altitude for takeoff
        self.hover_altitude = 1.0  # Default hover altitude
        self.voltage_threshold = 10.5  # Low battery threshold
        
        # State variables
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.target_velocity = [0.0, 0.0, 0.0]
        self.current_state = DroneState.IDLE
        self.battery_voltage = 12.0
        self.last_command_time = self.get_clock().now()
        
        # Safety flags
        self.emergency_stop = False
        self.low_battery = False
        self.timeout_active = False
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.state_pub = self.create_publisher(
            String,
            'drone_state',
            10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
                
                # Initialize messages
                self.cmd = Twist()
                
                # Set up timers
                self.create_timer(0.1, self.control_loop)  # 10Hz control loop
                self.create_timer(1.0, self.safety_check)  # 1Hz safety check
                
                self.get_logger().info('Drone Controller Node initialized in IDLE state')
                
        
    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        
    def control_loop(self):
        """Main control loop for velocity ramping and state management."""
        if self.emergency_stop or self.current_state == DroneState.EMERGENCY:
            self.cmd = Twist()
            self.publish_cmd()
            return
        
        # Update velocities with ramping
        for i in range(3):
            if abs(self.current_velocity[i] - self.target_velocity[i]) > self.velocity_ramp_rate:
                if self.current_velocity[i] < self.target_velocity[i]:
                    self.current_velocity[i] += self.velocity_ramp_rate
                else:
                    self.current_velocity[i] -= self.velocity_ramp_rate
            else:
                self.current_velocity[i] = self.target_velocity[i]
        
        # Update command message
        self.cmd.linear.x = self.current_velocity[0]
        self.cmd.linear.y = self.current_velocity[1]
        self.cmd.linear.z = self.current_velocity[2]
        
        # State-specific behavior
        if self.current_state == DroneState.TAKEOFF:
            if self.current_position[2] >= self.hover_altitude:
                self.transition_to_state(DroneState.HOVER)
        elif self.current_state == DroneState.LANDING:
            if self.current_position[2] <= self.min_altitude:
                self.transition_to_state(DroneState.IDLE)
                
        self.publish_cmd()
        self.publish_state()
    
    def safety_check(self):
        """Periodic safety checks."""
        # Check for command timeout
        if (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9 > 5.0:
            self.timeout_active = True
            self.get_logger().warn('Command timeout - no recent commands')
        
        # Check battery voltage
        if self.battery_voltage < self.voltage_threshold and not self.low_battery:
            self.low_battery = True
            self.get_logger().warn('Low battery detected')
        
        # Trigger emergency if needed
        if self.timeout_active or self.low_battery:
            self.emergency_stop_cmd()
    
    def takeoff(self):
        """Command the drone to takeoff."""
        if self.current_state == DroneState.IDLE and not self.emergency_stop:
            self.transition_to_state(DroneState.TAKEOFF)
            self.target_velocity[2] = self.TAKEOFF_VELOCITY
            self.get_logger().info('Taking off...')
            
    def land(self):
        """Command the drone to land."""
        if self.current_state in [DroneState.HOVER, DroneState.TAKEOFF]:
            self.transition_to_state(DroneState.LANDING)
            self.target_velocity[2] = -self.LANDING_VELOCITY
            self.get_logger().info('Landing...')
            
    def emergency_stop_cmd(self):
        """Trigger emergency stop."""
        self.emergency_stop = True
        self.transition_to_state(DroneState.EMERGENCY)
        self.target_velocity = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.cmd = Twist()
        self.publish_cmd()
        self.get_logger().warn('EMERGENCY STOP ACTIVATED')
        
    def move(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Move the drone with safety limits."""
        if self.emergency_stop or self.current_state not in [DroneState.HOVER, DroneState.TAKEOFF]:
            return
        
        # Update target velocities with safety limits
        self.target_velocity[0] = max(min(x, self.MAX_VELOCITY), -self.MAX_VELOCITY)
        self.target_velocity[1] = max(min(y, self.MAX_VELOCITY), -self.MAX_VELOCITY)
        self.target_velocity[2] = max(min(z, self.MAX_VELOCITY), -self.MAX_VELOCITY)
        
        # Altitude limits
        if self.current_position[2] >= self.MAX_ALTITUDE and self.target_velocity[2] > 0:
            self.target_velocity[2] = 0.0
        elif self.current_position[2] <= self.min_altitude and self.target_velocity[2] < 0:
            self.target_velocity[2] = 0.0
        
        self.cmd.angular.z = max(min(yaw, self.MAX_VELOCITY), -self.MAX_VELOCITY)
        self.last_command_time = self.get_clock().now()
        self.timeout_active = False
        
    def transition_to_state(self, new_state):
        """Handle state transitions."""
        self.current_state = new_state
        self.get_logger().info(f'Transitioning to state: {new_state.name}')
        
    def publish_cmd(self):
        """Publish command message."""
        self.cmd_vel_pub.publish(self.cmd)
        
    def publish_state(self):
        """Publish current state."""
        msg = String()
        msg.data = self.current_state.name
        self.state_pub.publish(msg)
        
    def process_key(self, key):
        """Process keyboard input."""
        if key == 'q':
            self.emergency_stop_cmd()
            return False
        elif self.emergency_stop:
            return True
            
        if key == 't':
            self.takeoff()
        elif key == 'l':
            self.land()
        elif key == 'w':
            self.move(x=self.VELOCITY_STEP)
        elif key == 's':
            self.move(x=-self.VELOCITY_STEP)
        elif key == 'a':
            self.move(y=self.VELOCITY_STEP)
        elif key == 'd':
            self.move(y=-self.VELOCITY_STEP)
        elif key == 'i':
            self.move(z=self.VELOCITY_STEP)
        elif key == 'k':
            self.move(z=-self.VELOCITY_STEP)
        elif key == 'j':
            self.move(yaw=self.VELOCITY_STEP)
        elif key == 'l':
            self.move(yaw=-self.VELOCITY_STEP)
        return True

async def get_key():
    """Async function to get keyboard input."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

async def key_loop(node):
    """Async loop for keyboard control."""
    while True:
        key = await get_key()
        if not node.process_key(key):
            break
        await asyncio.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    
    # Set up async keyboard handling
    loop = asyncio.get_event_loop()
    try:
        # Run ROS2 spin in a separate thread
        spin_thread = Thread(target=rclpy.spin, args=(node,))
        spin_thread.start()
        
        # Run keyboard control loop
        loop.run_until_complete(key_loop(node))
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()

