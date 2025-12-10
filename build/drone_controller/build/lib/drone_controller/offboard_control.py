import rclpy
from rclpy.node import Node
from rclpy.qos import QosProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        #Configure Qos profile for PX4 compatibility
        qos_profile = QosProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        #Variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        #Time for publishing setpoints
        self.timer = self.create_timer(0,1, self.timer_callback)

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
    
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm commant sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Switching to offboard mode')

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
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)
    
    def timer_callback(self):
        self.publish_offboard_control_mode()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0)
        elif self.offboard_setpoint_counter < 100:
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0)
        elif self.offboard_setpoint_counter < 200:
            self.publish_trajectory_setpoint(5.0, 0.0, -5.0)
        elif self.offboard_setpoint_counter < 300:
            self.publish_trajectory_setpoint(5.0, 5.0, -5.0)
        elif self.offboard_setpoint_counter < 400:
            self.publish_trajectory_setpoint(0.0, 5.0, -5.0)
        elif self.offboard_setpoint_counter < 500:
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0)
        elif self.offboard_setpoint_counter == 501:
            self.disarm()
            exit(0)

        if self.offboard_setpoint_counter < 501:
            self.offboard_setpoint_counter += 1

    def main(args=None):
        rclpy.init(args=args)
        offboard_control = OffboardControl()
        rclpy.spin(offboard_control)
        offboard_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()