#!/usr/bin/env python3
"""
Drone Commander - ROS2 Node für Drohnensteuerung via PX4
Sendet VehicleCommand Messages an PX4 via uORB/DDS Bridge
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# PX4 Messages
try:
    from px4_msgs.msg import VehicleCommand
    from px4_msgs.msg import VehicleStatus
    from px4_msgs.msg import OffboardControlMode
    from px4_msgs.msg import TrajectorySetpoint
    from px4_msgs.msg import VehicleOdometry
except ImportError:
    print("ERROR: px4_msgs not found. Install with: sudo apt install ros-jazzy-px4-msgs")
    print("       or build from: https://github.com/PX4/px4_msgs")
    import sys
    sys.exit(1)


class DroneCommander(Node):
    """ROS2 Node für Drohnensteuerung"""

    # MAVLink Commands (PX4 Standard)
    CMD_NAV_TAKEOFF = 22
    CMD_NAV_LAND = 21
    CMD_DO_SET_MODE = 176
    CMD_COMPONENT_ARM_DISARM = 400

    # Flight Modes
    MODE_MANUAL = 1
    MODE_ALTCTL = 2
    MODE_POSCTL = 3
    MODE_OFFBOARD = 4
    MODE_STABILIZED = 5

    def __init__(self):
        super().__init__('drone_commander')

        # QoS Profile für PX4 (Best Effort, wie PX4 es erwartet)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )

        # State
        self.vehicle_status = None
        self.current_position = [0.0, 0.0, 0.0]
        self.armed = False
        self.flight_mode = None

        self.get_logger().info('Drone Commander Node initialized')
        self.get_logger().info('Waiting for PX4 connection...')

    def vehicle_status_callback(self, msg):
        """Status Updates von PX4"""
        self.vehicle_status = msg
        self.armed = (msg.arming_state == 2)  # ARMING_STATE_ARMED
        self.flight_mode = msg.nav_state

    def odometry_callback(self, msg):
        """Position Updates"""
        self.current_position = [msg.position[0], msg.position[1], msg.position[2]]

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0,
                            param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        """Generic Vehicle Command senden"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.vehicle_command_pub.publish(msg)
        self.get_logger().info(f'Sent command: {command} with params [{param1}, {param2}, {param3}]')

    def arm(self):
        """Drohne armen (Motoren aktivieren)"""
        self.get_logger().info('Arming drone...')
        self.send_vehicle_command(
            self.CMD_COMPONENT_ARM_DISARM,
            param1=1.0  # 1 = ARM
        )

    def disarm(self):
        """Drohne disarmen"""
        self.get_logger().info('Disarming drone...')
        self.send_vehicle_command(
            self.CMD_COMPONENT_ARM_DISARM,
            param1=0.0  # 0 = DISARM
        )

    def takeoff(self, altitude=5.0):
        """Takeoff zu bestimmter Höhe"""
        self.get_logger().info(f'Taking off to {altitude}m...')

        # First arm
        self.arm()
        rclpy.spin_once(self, timeout_sec=0.5)

        # Then takeoff
        self.send_vehicle_command(
            self.CMD_NAV_TAKEOFF,
            param1=0.0,  # pitch
            param2=0.0,  # empty
            param3=0.0,  # empty
            param4=0.0,  # yaw
            param5=0.0,  # latitude (0 = current position)
            param6=0.0,  # longitude
            param7=altitude
        )

    def land(self):
        """Landen"""
        self.get_logger().info('Landing...')
        self.send_vehicle_command(
            self.CMD_NAV_LAND,
            param1=0.0,  # abort altitude (0 = no abort)
            param2=0.0,  # precision land mode
            param3=0.0,  # empty
            param4=0.0,  # yaw
            param5=0.0,  # latitude
            param6=0.0,  # longitude
            param7=0.0   # altitude
        )

    def set_offboard_mode(self):
        """Offboard Mode aktivieren (für präzise Positionskontrolle)"""
        self.get_logger().info('Setting OFFBOARD mode...')
        self.send_vehicle_command(
            self.CMD_DO_SET_MODE,
            param1=1.0,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=6.0   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        )

    def goto_position(self, x, y, z, yaw=0.0):
        """Fliege zu Position (NED frame: North, East, Down)"""
        self.get_logger().info(f'Going to position: [{x}, {y}, {z}]')

        # Offboard Control Mode setzen
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(offboard_msg)

        # Trajectory Setpoint senden
        traj_msg = TrajectorySetpoint()
        traj_msg.position = [x, y, z]
        traj_msg.yaw = yaw
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(traj_msg)

    def get_status(self):
        """Aktuellen Status zurückgeben"""
        return {
            'armed': self.armed,
            'flight_mode': self.flight_mode,
            'position': self.current_position
        }


def main(args=None):
    rclpy.init(args=args)
    drone = DroneCommander()

    try:
        rclpy.spin(drone)
    except KeyboardInterrupt:
        drone.get_logger().info('Shutting down...')
    finally:
        drone.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
