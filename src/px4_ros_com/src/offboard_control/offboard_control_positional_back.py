#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleAttitude, VehicleLandDetected
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool

import numpy as np
from math import pi

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        qos_profile_vel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.velocity_publisher = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)
        self.offboard_velocity_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.offboard_velocity_callback,
            qos_profile_vel)
        self.offboard_velocity_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel_key',
            self.offboard_velocity_key_callback,
            qos_profile_vel)
        self.arm_subscriber = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            qos_profile)
        self.velocity_mode_subscriber = self.create_subscription(
            Bool,
            '/velocity_mode',
            self.velocity_mode_callback,
            qos_profile)
        self.offboard_mode_subscriber = self.create_subscription(
            Bool,
            '/offboard_mode',
            self.state_offboard,
            qos_profile)
        
        self.attitude_subscriber = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)
        
        self.landed_subcriber = self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self.land_message_callback,
            qos_profile)
        

        # Initialize variables
        self.initialize_parameters()

        self.velocity = Vector3()
        self.yaw = 0.0  #yaw value we send as command
        self.trueYaw = 0.0  #current yaw value of drone

        self.velocity_mode = True  # True for velocity control, False for position control
        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def velocity_mode_callback(self, msg):
        self.velocity_mode = msg.data
        self.get_logger().info(f"Velocity Mode: {self.velocity_mode}")

    def arm_message_callback(self, msg):
        if (msg.data == False):
            if (self.landed == True):
                self.get_logger().info("Disarming")
                self.disarm()
                self.current_state = "IDLE"
                self.arm_message = msg.data
            else:
                self.get_logger().info("Cannot disarm, vehicle is not landed")

        else:
            self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")
    
    def initialize_parameters(self):
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -1.0
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.flightCheck = False
        self.failsafe = False
        self.offboard_mode = False
        self.arm_message = False
        self.landed = True
        
        self.current_state = "IDLE"
        self.last_state = self.current_state
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def land_message_callback(self, msg):
        self.landed = msg.landed
        if self.landed == True and self.current_state == "OFFBOARD":
            self.get_logger().info("Vehicle Landed")
            self.initialize_parameters()  # Reset parameters when landed

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""    
        
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            if msg.arming_state == False:
                self.current_state = "IDLE"

            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")
         
        if (msg.nav_state != 14):
            self.offboard_mode = False
               
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass
        
        self.vehicle_status = msg
    
    #receives Twist commands from Teleop and converts NED -> FLU
    def offboard_velocity_key_callback(self, msg):
        if not self.velocity_mode:
            return
        #implements NED -> FLU Transformation
        # X (FLU) is -Y (NED)
        self.velocity.x = -msg.linear.y
        # Y (FLU) is X (NED)
        self.velocity.y = msg.linear.x
        # Z (FLU) is -Z (NED)
        self.velocity.z = -msg.linear.z
        # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
        self.yaw = msg.angular.z
        
    #receives Twist commands from Nav2 and converts NED -> FLU
    def offboard_velocity_callback(self, msg):
        if self.velocity_mode:
             return
        #implements NED -> FLU Transformation
        # X (FLU) is -Y (NED)
        self.velocity.x = -msg.linear.y
        # Y (FLU) is X (NED)
        self.velocity.y = msg.linear.x
        # Z (FLU) is -Z (NED)
        self.velocity.z = -msg.linear.z
        # A conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.trueYaw)
        self.yaw = msg.angular.z
    

    #receives current trajectory values from drone and grabs the yaw value of the orientation
    def attitude_callback(self, msg):
        orientation_q = msg.q

        #trueYaw is the drones current yaw value
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))


    def state_offboard(self):
        if self.offboard_mode:
            return
        self.offboard_setpoint_counter = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.offboard_mode = True
        self.get_logger().info("Switching to offboard mode")
        
    # CONTINOUSLY SENDING ARM COMMANDS, TO-DO FIX MAYBE
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        # self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')
        
    # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=-self.takeoff_height) # param7 is altitude in meters
        self.get_logger().info("Takeoff command send")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self, velocity_mode):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = not velocity_mode
        msg.velocity = velocity_mode
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_velocity_setpoint(self):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        

        # Compute velocity in the world frame
        cos_yaw = np.cos(self.trueYaw)
        sin_yaw = np.sin(self.trueYaw)
        velocity_world_x = float(self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
        velocity_world_y = float(self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)
        
        #self.get_logger().info(f"Publishing setpoints {[velocity_world_x, velocity_world_y, self.velocity.z]}")

        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [velocity_world_x, velocity_world_y, self.velocity.z]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]

        msg.yaw = float('nan')
        msg.yawspeed = self.yaw

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing setpoints {[x, y, z]}")


    def publish_position_setpoint(self):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        
        speed = 0.5
        
        x = self.vehicle_local_position.x
        y = self.vehicle_local_position.y
        z = self.vehicle_local_position.z # or, self.takeoff_height
        cos_yaw = np.cos(self.trueYaw)
        sin_yaw = np.sin(self.trueYaw)
        delta_x = float(self.velocity.x * cos_yaw - self.velocity.y * sin_yaw) * speed
        delta_y = float(self.velocity.x * sin_yaw + self.velocity.y * cos_yaw) * speed
        
        msg.position = [x+delta_y, y+delta_x, z]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        ## TO-DO
        msg.yaw = float('nan')  # pi/2 = (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
        #self.get_logger().info(f"delta (x,y): {[delta_x, delta_y]}")
        self.trajectory_setpoint_publisher.publish(msg)
        

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal(self.velocity_mode)

        match self.current_state:
            case "IDLE":
                if(self.flightCheck and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.offboard_setpoint_counter > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")
                self.arm() #send arm command
                self.take_off() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                
                if (self.offboard_mode == False):
                    self.state_offboard()

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False


        if(self.offboard_mode == True):
            if self.velocity_mode == True:
                self.publish_velocity_setpoint()
            else:
                self.publish_position_setpoint()
         
        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)
            
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

