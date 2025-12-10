#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        
        # Configure QoS profile for PX4 compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', 
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)

        # Variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.landing_initiated = False
        self.mission_complete = False

        # Timer for publishing setpoints
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        # Check if landed
        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            self.get_logger().info('Vehicle is landing...')

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Switching to offboard mode')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Landing command sent')

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=-5.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0  # North
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        
    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        if self.mission_complete:
            return
            
        self.publish_offboard_control_mode()

        # Engage offboard mode and arm
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        # Flight path
        if self.offboard_setpoint_counter < 11:
            # Pre-flight setpoints
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0)
        elif self.offboard_setpoint_counter < 150:
            # Takeoff and hold
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0)
            if self.offboard_setpoint_counter == 149:
                self.get_logger().info('Holding position at 5m')
        elif self.offboard_setpoint_counter < 250:
            # Move to point 1
            self.publish_trajectory_setpoint(5.0, 0.0, -5.0)
            if self.offboard_setpoint_counter == 249:
                self.get_logger().info('Reached waypoint 1')
        elif self.offboard_setpoint_counter < 350:
            # Move to point 2
            self.publish_trajectory_setpoint(5.0, 5.0, -5.0)
            if self.offboard_setpoint_counter == 349:
                self.get_logger().info('Reached waypoint 2')
        elif self.offboard_setpoint_counter < 450:
            # Move to point 3
            self.publish_trajectory_setpoint(0.0, 5.0, -5.0)
            if self.offboard_setpoint_counter == 449:
                self.get_logger().info('Reached waypoint 3')
        elif self.offboard_setpoint_counter < 550:
            # Return home
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0)
            if self.offboard_setpoint_counter == 549:
                self.get_logger().info('Returned to home position')
        elif self.offboard_setpoint_counter < 650:
            # Descend slowly to 1m
            self.publish_trajectory_setpoint(0.0, 0.0, -1.0)
            if self.offboard_setpoint_counter == 649:
                self.get_logger().info('Descended to 1m')
        elif self.offboard_setpoint_counter == 650:
            # Land command
            if not self.landing_initiated:
                self.land()
                self.landing_initiated = True
                self.get_logger().info('Initiating landing sequence')
        elif self.offboard_setpoint_counter > 750:
            # After landing, disarm
            if not self.mission_complete:
                self.disarm()
                self.mission_complete = True
                self.get_logger().info('Mission complete - vehicle disarmed')
                self.get_logger().info('Shutting down...')
                exit(0)

        if self.offboard_setpoint_counter < 800:
            self.offboard_setpoint_counter += 1


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()