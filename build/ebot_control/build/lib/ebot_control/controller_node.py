#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import serial
import math
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # Parameters
        self.declare_parameter('wheel_base', 0.25)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = self.get_clock().now()
        
        # Motor control
        self.target_left_vel = 0.0
        self.target_right_vel = 0.0
        self.current_left_vel = 0.0
        self.current_right_vel = 0.0
        
        # Joint states
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_time = self.get_clock().now()
        
        # Setup serial connection
        try:
            self.ser = serial.Serial(
                self.get_parameter('serial_port').value,
                self.get_parameter('baud_rate').value,
                timeout=1
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            raise
        
        # Subscribers and publishers
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for continuous updates
        self.update_timer = self.create_timer(0.02, self.update_motors_and_joints)
        
        self.get_logger().info('Motor driver node started.')

    def cmd_vel_callback(self, msg):
        # Calculate target wheel velocities
        v = msg.linear.x
        omega = msg.angular.z
        L = self.wheel_base

        self.target_left_vel = v - (L / 2.0) * omega
        self.target_right_vel = v + (L / 2.0) * omega

    def update_motors_and_joints(self):
        # Smooth velocity changes
        acceleration = 0.5  # m/s²
        dt = 0.02  # timer period
        
        # Gradually approach target velocity
        self.current_left_vel = self.apply_acceleration(
            self.current_left_vel, self.target_left_vel, acceleration, dt)
        self.current_right_vel = self.apply_acceleration(
            self.current_right_vel, self.target_right_vel, acceleration, dt)
        
        # Send motor commands
        scale = 100
        left_pwm = int(max(min(self.current_left_vel * scale, 100), -100))
        right_pwm = int(max(min(self.current_right_vel * scale, 100), -100))
        
        cmd = f"{left_pwm},{right_pwm}\n"
        try:
            self.ser.write(cmd.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")
        
        # Update joint states
        current_time = self.get_clock().now()
        dt_joint = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Convert linear velocities to angular (rad/s)
        left_angular_vel = self.current_left_vel / self.wheel_radius
        right_angular_vel = self.current_right_vel / self.wheel_radius
        
        # Update positions
        self.left_wheel_pos += left_angular_vel * dt_joint
        self.right_wheel_pos += right_angular_vel * dt_joint
        
        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = [left_angular_vel, right_angular_vel]
        self.joint_state_pub.publish(joint_state)
        
        # Calculate and publish odometry
        self.publish_odometry(current_time, left_angular_vel, right_angular_vel)

    def apply_acceleration(self, current, target, acceleration, dt):
        if current < target:
            return min(current + acceleration * dt, target)
        else:
            return max(current - acceleration * dt, target)

    def publish_odometry(self, current_time, left_vel, right_vel):
        # Calculate linear and angular velocity
        linear_vel = (right_vel + left_vel) * self.wheel_radius / 2.0
        angular_vel = (right_vel - left_vel) * self.wheel_radius / self.wheel_base
        
        # Calculate time since last update
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time
        
        # Update position
        self.theta += angular_vel * dt
        self.x += linear_vel * math.cos(self.theta) * dt
        self.y += linear_vel * math.sin(self.theta) * dt
        
        # Create and publish TransformStamped
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # Create and publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        self.odom_pub.publish(odom)

    def destroy_node(self):
        # Stop motors before shutting down
        try:
            self.ser.write("0,0\n".encode())
            self.ser.close()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()