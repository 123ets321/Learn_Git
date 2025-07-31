#! /usr/bin/env python3
"""
Developed by : AI Team (Skytex)
Version : 1.0.0
Date : 4th Jul 2025 
Description : This script handles lock on target (track vbtg) and dive (vbtg and coordinated) logic.
              In this attitude heartbeat is published in order to send attitude setpoints to the autopilot

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleLocalPositionSetpoint,\
                         VehicleStatus, VehicleGlobalPosition, VehicleOdometry, FailsafeFlags, SensorAccel, VehicleAttitude, VehicleAttitudeSetpoint, AirspeedValidated, HoverThrustEstimate
import time
import numpy as np
from math import nan, isnan, atan2, pi, sqrt, asin
import utm
from std_msgs.msg import String, Float64
import json
from vtol_stack.helper import UavHelper




class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # load ghost config parameters
        self.load_ghost_params()
        self.helper = UavHelper()
        
        self.recovery_waypoints = []
        self.current_recovery_index = 0

        self.done = False
        self.w = 0.0
        self.takeoff_ctr = 0

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
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
        
        self.vehicle_local_position_setpoint_publisher = self.create_publisher(
            VehicleLocalPositionSetpoint, '/fmu/in/vehicle_local_position_setpoint', qos_profile)
        
        self.vehicle_attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        

        self.dive_res_publisher = self.create_publisher(String,
                                                        '/dive_res',
                                                        10)

        self.strike_radius_publisher = self.create_publisher(Float64,
                                                             '/strike_radius',
                                                             10)

        #Create subscribers
        self.vehicle_hover_thrust_subscriber = self.create_subscription(HoverThrustEstimate, 
                                                                        '/fmu/out/hover_thrust_estimate', 
                                                                        self.vehicle_hover_callback, 
                                                                        qos_profile)

        self.vehicle_airspeed_subscriber = self.create_subscription(
            AirspeedValidated, '/fmu/out/airspeed_validated', self.vehicle_airspeed_callback, qos_profile)
        
        self.vehicle_accel_subscriber = self.create_subscription(
            SensorAccel, '/fmu/out/sensor_accel', self.vehicle_accel_callback, qos_profile)
        
        self.vehicle_att_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_att_callback, qos_profile)
        
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)
        
        self.vehicle_odo_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odo_callback, qos_profile)
        
        self.failsafe_flags = self.create_subscription(
            FailsafeFlags, '/fmu/out/failsafe_flags', self.vehicle_failsafe_flag_callback, qos_profile)
        
        self.command_subscriber = self.create_subscription(String,
                                                          "command_publisher",
                                                          self.commands_cb,
                                                          10)
        
        self.target_geolocation_subscriber = self.create_subscription(String,
                                                                      "/target_geolocation",
                                                                      self.target_geolocation_cb,
                                                                      10)

        self.mission_data_subscriber = self.create_subscription(String,
                                                                "/mission_data",
                                                                self.mission_data_cb,
                                                                10)

        # Initialize variables
        self.vehicle_airspeed = AirspeedValidated()
        self.vehicle_att = VehicleAttitude()
        self.vehicle_accel = SensorAccel()
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_local_position_setpoint = VehicleLocalPositionSetpoint()
        self.vehicle_status = VehicleStatus()
        self.vehicle_global_position = VehicleGlobalPosition()
        self.dive_res = String()
        self.strike_radius_to_gcs = Float64()
        self.vehicle_odo = VehicleOdometry()
        self.vehicle_failsafe_flag = FailsafeFlags()
        self.vehicle_hover_thrust = HoverThrustEstimate()


    
        # Create a timer to publish control commands
        self.timer = self.create_timer(self.dive_timer_hz, self.timer_callback)
        # self.guidance = self.create_timer(0.0002, self.guidance_timer)
        

    ############## load params ##############

    def load_ghost_params(self):
        with open("./src/vtol_stack/config_files/ghost_config.json", 'r') as file:
            ghost_params = json.load(file)
        file.close()

        self.dive_timer_hz = ghost_params["dive_code_timer"]
        self.reocver_point_sent = False
        # home params
        home_params = ghost_params["dive_home_params"]
        self.home_alt = home_params["home_alt"]
        self.home_flag = home_params["home_flag"]
        self.home_lat = home_params["home_lat"]
        self.home_lon = home_params["home_lon"]
        self.home_lat_ghost = home_params["home_lat_ghost"]
        self.home_lon_ghost = home_params["home_lon_ghost"]


        vehicle_flags = ghost_params["vehicle_flags"]
        self.takeoff_height = vehicle_flags["takeoff_height"]
        self.air_speed = vehicle_flags["airspeed"]
        self.desired_speed = self.air_speed

        self.id = ghost_params["id"]

        self.threshold = ghost_params["threshold"]

        self.notification = ghost_params["notification"]

        self.available_modes = {int(k):v for k, v in ghost_params["available_modes"].items()}


        # dive and nav related params
        nav_and_dive = ghost_params["nav_dive_params"]
        self.strike_angle = np.deg2rad(nav_and_dive["strike_angle"])
        self.min_dive_start_alt = nav_and_dive["min_dive_start_alt"]
        self.max_bank_angle = np.deg2rad(nav_and_dive["max_bank_angle"])
        self.base_thrust = nav_and_dive["base_thrust"]
        self.max_pitch_angle = nav_and_dive["max_pitch_angle"]
        self.kp = nav_and_dive["kp"]
        self.ki = nav_and_dive["ki"]
        self.sampling_time = nav_and_dive["sampling_time"]
        self.integral_error = nav_and_dive["integral_error"]
        self.cancel_alt = nav_and_dive["cancel_alt"]
        self.rmin = nav_and_dive["rmin"]
        self.vehicle_ini_xl = nav_and_dive["vehicle_ini_xl"]
        self.vehicle_ini_yl = nav_and_dive["vehicle_ini_yl"]
        self.vehicle_ini_zl = nav_and_dive["vehicle_ini_zl"]

        # dive flags
        dive_flags = ghost_params["dive_flags"]
        self.command = dive_flags["command"]
        self.command_data = dive_flags["command_data"]
        self.dive = dive_flags["dive"]
        self.target_lat = dive_flags["target_lat"]
        self.target_lon = dive_flags["target_lon"]
        self.location = dive_flags["location"]
        self.lock = dive_flags["lock"]
        self.is_dive_started = dive_flags["is_dive_started"]
        self.dist_midpoint = dive_flags["dist_midpoint"]
        self.climbed = dive_flags["climbed"]
        self.offboard_flag = dive_flags["offboard_flag"]
        self.cancel_flag = dive_flags["cancel_flag"]


    ############## ros2 callbacks ##############
    
    def vehicle_hover_callback(self, vehicle_hover_thrust):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_hover_thrust = vehicle_hover_thrust



    def target_geolocation_cb(self,msg):
        self.target_geolocation = msg
        target_location = json.loads(self.target_geolocation.data)
        # if target_location["lock"]:
        self.lock = target_location["lock"]
        self.command = target_location["type"]
        # print("lock value received from lock script : ", self.lock)
        if self.lock and self.cancel_flag:
            print("resetted cancel flag")
            self.cancel_flag = 0
        self.location = True
        self.offboard_flag = False
        self.target_lat = target_location["lat_lon_alt"][0]
        self.target_lon = target_location["lat_lon_alt"][1]
        
        self.target_position_x = target_location["lat_lon_alt"][0]
        self.target_position_y = target_location["lat_lon_alt"][1]
        # print("received target geolocation : ", self.target_geolocation)

    def commands_cb(self, msg):
        command_data = json.loads(msg.data)
        self.command = command_data["command"]
        self.command_data = command_data

        # vision based dive
        if self.command == "dive":
            self.dive = True
            self.offboard_flag = False
            self.cancel_flag = 0
  

        # coordinated dive
        elif self.command == "strike_lat_long":
            self.dive = True
            self.offboard_flag = False
            self.cancel_flag = 0
            coords = self.command_data["coordinate"]
            self.target_position_x = coords[1]
            self.target_position_y = coords[0]
        else:
            self.dive = False

        if self.dive:
            wadeoff_threshold_res = json.dumps({"wadeoff_thresh":None,
                                                "dive_status":None,
                                                "wadeoff_status":None,
                                                "received":True})
            self.dive_res.data = wadeoff_threshold_res
            self.dive_res_publisher.publish(self.dive_res)
            
        print("received a command : ", command_data, " dive value : ", self.dive)

        
    
    def vehicle_airspeed_callback(self, vehicle_air):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_airspeed = vehicle_air   


    def vehicle_att_callback(self, vehicle_att_sig):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_att = vehicle_att_sig      
        
    def vehicle_accel_callback(self, vehicle_accel_sig):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_accel = vehicle_accel_sig   

    def vehicle_odo_callback(self, vehicle_odo):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_odo = vehicle_odo

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_global_position_callback(self, vehicle_global_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_global_position = vehicle_global_position
        #print("Home position updated to vehicle's current position:", self.vehicle_global_position.lat, self.vehicle_global_position.lon)
        # if self.vehicle_global_position.lat and self.vehicle_global_position.lon:
        #     self.home_lat = self.vehicle_global_position.lat
        #     self.home_lon = self.vehicle_global_position.lon


    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status


    def vehicle_failsafe_flag_callback(self, vehicle_failsafe_flag):
        self.vehicle_failsafe_flag = vehicle_failsafe_flag
    

    ############## custom functionalities ##############

    def publish_attitude_setpoint(self, roll,pitch, yaw):
        """Publish attitude setpoints using roll, pitch, yaw, and thrust."""
        actual_airspeed = self.vehicle_airspeed.true_airspeed_m_s
        air_speed_error = self.air_speed - actual_airspeed
    
        self.integral_error += air_speed_error*self.sampling_time
        self.integral_error = np.clip(self.integral_error,-0.05,0.05)

        thrustt = self.base_thrust + self.kp*air_speed_error + self.ki *self.integral_error
        pitch = np.clip(pitch, -np.radians(self.max_pitch_angle), np.radians(self.max_pitch_angle))
        thrustt = np.clip(thrustt, 0,1)
        msg = VehicleAttitudeSetpoint()
        msg.roll_body = roll
        msg.pitch_body = -pitch
        msg.yaw_body = yaw
        thrust_x = thrustt*np.cos(-pitch)
        thrust_y = 0.0
        thrust_z = thrustt*np.sin(-pitch)
        msg.thrust_body = [thrust_x, thrust_y, thrust_z]  # Thrust in body frame
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_attitude_setpoint_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position =  [float(x), float(y), float(z)]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    
    def publish_offboard_control_heartbeat_signal(self,position=False, attitude=False):
        # """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = attitude
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)




    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")


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



    def vtol_transition(self, mode):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION,
            param1=mode
        )
        print(f'transition command sent')



    def change_mode(self, mode, submode, name):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=mode, param3=submode)
        self.get_logger().info(f"Switching to {name} mode")
    
    def waypoints(self,curr_lat, curr_lon, curr_alt):
        
        target_lat = curr_lat
        target_lon = curr_lon
        target_lon_m, target_lat_m, _, _ = utm.from_latlon(target_lat, target_lon)
        home_lon_m, home_lat_m, _, _ = utm.from_latlon(self.home_lat_ghost, self.home_lon_ghost)  
        waypoint = np.array([target_lat_m, target_lon_m, -curr_alt]) - np.array([home_lat_m, home_lon_m, 0.0])
        return  float(waypoint[0]), float(waypoint[1]), float(curr_alt)
    
    def update_waypoint(self, id, mission):
        # print("failed now : ", id)
        wp_id = mission[id]["id"]
        self.current_waypoint = wp_id
        self.current_wp_id = wp_id
        self.curr_recovery_lat = mission[id]["coordinate"][1]
        self.curr_recovery_lon = mission[id]["coordinate"][0]
        self.curr_recovery_alt = mission[id]["altitude"]
    
    def get_current_heading(self):
        heading = self.helper.euler_from_quaternion(self.vehicle_att.q[1],self.vehicle_att.q[2],self.vehicle_att.q[3],self.vehicle_att.q[0])
        return heading
    
    
    def change_loiter_altitude(self, alt):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_CHANGE_ALTITUDE, param1=self.home_alt+float(abs(alt)), param2=float(0)
            )
        
    def cancel_straight_following(self,s0,position1, point2, point_in_inertial):
        

        rmin = 2*s0**2/(9.81*np.tan(self.max_bank_angle)) 

        x_l1 = position1[0]
        y_l1 = position1[1]
        z_l1 = position1[2]
        
        recover_point = np.array([point2[0], point2[1], 0.0]) - np.array([point_in_inertial[0], point_in_inertial[1], 0.0]) # changed height sign
        unit_recover_point = recover_point/np.linalg.norm(recover_point)
        angle_unit_recover_point = np.arctan2(unit_recover_point[1], unit_recover_point[0])
        angle_recover = np.radians(3.0)
        pitch_up_direction = np.array([np.cos(angle_recover),0.0, np.sin(angle_recover)])
        rot_matrix_recover = np.array([[np.cos(angle_unit_recover_point), -np.sin(angle_unit_recover_point), 0.0],
                                        [np.sin(angle_unit_recover_point), np.cos(angle_unit_recover_point), 0.0],
                                        [0.0, 0.0, 1.0]])
        pitch_up_vector= np.matmul(rot_matrix_recover, pitch_up_direction.T)
        recover_alt = 300.0
        scale_facrtor_recover = recover_alt/pitch_up_vector[2]
        pitch_up_vector = scale_facrtor_recover*pitch_up_vector

        pitch_up_point = np.array([pitch_up_vector[0], pitch_up_vector[1], -pitch_up_vector[2]]) + point2
        

        point2 = point2 + np.array([0.0, 0.0, -150.0])
        vect1 =  pitch_up_point - point2

        vect2 = np.array([x_l1, y_l1, z_l1]) - point2

        unit_vect2 = vect2/np.linalg.norm(vect2)
        unit_vect1 = vect1/np.linalg.norm(vect1)

        vect3 = np.cross(unit_vect2,unit_vect1)

        y_unit_vector = vect1/np.linalg.norm(vect1)

        z_unit_vector = vect3/np.linalg.norm(vect3) 

        x_unit_vector  = np.cross(y_unit_vector,z_unit_vector)

        x_unit_vector  = x_unit_vector/np.linalg.norm(x_unit_vector) 

        rotation_matrix = np.column_stack((x_unit_vector, y_unit_vector, z_unit_vector))

        inv_rotation_matrix = rotation_matrix.T

        x_e_matrix = np.dot(inv_rotation_matrix,vect2)

        x_e = x_e_matrix[0]
            
        vectt1 =  point_in_inertial - point2

        vect_x = np.dot(inv_rotation_matrix,vectt1)

        theta_i = np.arctan2(-vect_x[1], -vect_x[0])

        v_r = s0*np.sin(theta_i)

        c_cal = (2*s0)/((s0-v_r)*rmin)
        
        f_2 = (s0-v_r) * (self.helper.sech(c_cal*x_e)) + v_r

        f_1 = - np.sign(x_e)*np.sqrt((s0**2)-(f_2**2))

        vect3D = np.array([[f_1],[f_2],[0.0]])

        vector_field_3d = np.matmul(rotation_matrix,vect3D)

        neg_gradient_ang = np.arctan2(vector_field_3d[1], vector_field_3d[0])

        mag_neg1 =  np.sqrt(vector_field_3d[0]**2 + vector_field_3d[1]**2)

        flight_path_angle = np.arctan2(vector_field_3d[2], mag_neg1)

        w_max = np.abs(s0/(rmin)) 

        current_yaw_angle = self.get_current_heading()
        chi_dot = w_max*(2/np.pi)*np.arctan(np.sinh(2.5*(self.helper.check_angle(neg_gradient_ang-current_yaw_angle)))) 
        roll_angle = np.arctan(chi_dot*s0/9.81)
       
        pitch_angle  = flight_path_angle
        yaw_angle = self.helper.check_angle(neg_gradient_ang + w_max*(2/np.pi)*np.arctan(np.sinh(2.5*(self.helper.check_angle(neg_gradient_ang-current_yaw_angle))))) 


        return  float(roll_angle), float(pitch_angle), float(yaw_angle ) 
#------------------------------------------------------------------------------------------------------ Claude
    def mission_data_cb(self, msg):
        """Callback for mission data topic"""
        try:
            mission_data = json.loads(msg.data)
            
            if mission_data.get("command") == "mission_upload":
                self.extract_recovery_waypoints(mission_data)
                self.get_logger().info(f"Mission data received with {len(self.recovery_waypoints)} recovery points")
                
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse mission data JSON")
        except Exception as e:
            self.get_logger().error(f"Mission data callback error: {str(e)}")

    def extract_recovery_waypoints(self, mission_data):
        """Extract recovery waypoints from mission data"""
        self.recovery_waypoints = []
        
        for waypoint in mission_data.get("waypoint", []):
            if waypoint.get("routine") == "recovery":  # Check for recovery routine
                self.recovery_waypoints.append(waypoint)
        
        # Sort by ID to ensure correct sequence
        self.recovery_waypoints.sort(key=lambda x: x.get("id", 0))
        self.current_recovery_index = 0
        
        self.get_logger().info(f"Extracted {len(self.recovery_waypoints)} recovery waypoints")
        
        # Log recovery waypoints for debugging
        for i, wp in enumerate(self.recovery_waypoints):
            self.get_logger().info(f"Recovery WP {i}: ID={wp['id']}, "
                                  f"Coord={wp['coordinate']}, Alt={wp['altitude']}")

    def execute_home_recovery(self):
        """Fallback home recovery logic"""
        home_lon_m, home_lat_m, _, _ = utm.from_latlon(self.home_lat_ghost, self.home_lon_ghost)
        curr_lon_m, curr_lat_m, _, _ = utm.from_latlon(self.vehicle_global_position.lat, self.vehicle_global_position.lon)
        
        x_home = home_lat_m - curr_lat_m
        y_home = home_lon_m - curr_lon_m
        z_home = -200.0
        
        print(f'Home: lat={self.home_lat_ghost}, lon={self.home_lon_ghost}')
        print(f'Current: lat={self.vehicle_global_position.lat}, lon={self.vehicle_global_position.lon}')
        print(f'Target offsets: x_home={x_home}, y_home={y_home}, z_home={z_home}')
        
        self.publish_trajectory_setpoint(x_home, y_home, z_home)
        
        if (self.cancel_flag
            and abs(self.vehicle_odo.position[2]) >= 200.0
            and self.helper.calculate_reached_threshold(
                self.vehicle_global_position.lat,
                self.vehicle_global_position.lon,
                self.home_lat_ghost,
                self.home_lon_ghost,
                threshold=10.0)):
            
            print("Reached home position - Going to HOLD")
            self.complete_recovery_sequence()

    def complete_recovery_sequence(self):
        """Complete recovery and reset all flags"""
        self.change_mode(4.0, 3.0, 'hold')
        self.dive = False
        self.location = False
        self.climbed = False
        self.cancel_flag = 0
        self.current_recovery_index = 0
        self.command = None
        
        wadeoff_threshold_res = json.dumps({
            "wadeoff_thresh": 150,
            "dive_status": False,
            "wadeoff_status": True,
            "recovery_completed": True,
            "received": None
        })
        self.dive_res.data = wadeoff_threshold_res
        self.dive_res_publisher.publish(self.dive_res)
        
    def handle_recovery_mode(self):
        
        self.publish_offboard_control_heartbeat_signal(attitude=False, position=True)
        self.get_logger().info("Sending Position Offboard Heartbeat - Recovery Mode")

        if len(self.recovery_waypoints) > 0:
            current_recovery = self.recovery_waypoints[self.current_recovery_index]
            if current_recovery:
                self.update_waypoint(self.current_recovery_index, self.recovery_waypoints)
                x_target, y_target, z_target = self.waypoints(self.curr_recovery_lat, 
                                                              self.curr_recovery_lon, 
                                                              self.curr_recovery_alt)
                
                print(f'WP_Recovery {self.current_recovery_index}: lat={self.curr_recovery_lat}, lon={self.curr_recovery_lon}, alt={self.curr_recovery_alt}')
                print(f'Current_loc: lat={self.vehicle_global_position.lat}, lon={self.vehicle_global_position.lon}')
                print(f'Target offsets_for_WP: x={x_target}, y={y_target}, z={z_target}')
                
                # if (abs(self.vehicle_odo.position[2]) >= abs(z_target)
                #     and self.helper.calculate_reached_threshold(
                #         self.vehicle_global_position.lat,self.vehicle_global_position.lon,
                #         recovery_lat,recovery_lon,threshold = 10.0)):
                
                if self.helper.calculate_reached_threshold(self.vehicle_global_position.lat,
                                                           self.vehicle_global_position.lon,
                                                           self.curr_recovery_lat,self.curr_recovery_lon,
                                                           threshold = 10.0):
                    
                    print(f"Reached Recovery WP {self.current_recovery_index}")
                    self.current_recovery_index += 1
                    if self.current_recovery_index < len(self.recovery_waypoints):
                        self.get_logger().info("Updating Recovery WP")
                        self.update_waypoint(self.current_recovery_index, self.recovery_waypoints)
                    else:
                        print("All recovery waypoints completed - Going to HOLD")
                        self.complete_recovery_sequence()
                        
                self.publish_trajectory_setpoint(x_target, y_target, -z_target)
            else:
                self.complete_recovery_sequence()
        else:
            self.execute_home_recovery()

#------------------------------------------------------------------------------------------------------ Claude


    def dive_single_vehicle(self, speed0, position1, dive_lat, dive_lon):
        x_l1 = position1[0]
        y_l1 = position1[1]
        z_l1 = position1[2]
    
        rmin = speed0**2/(9.81*np.tan(self.max_bank_angle)) 
       
        target_lon_m, target_lat_m, _, _ = utm.from_latlon(dive_lat, dive_lon)

        if self.is_dive_started is False:
            self.vehicle_ini_xl = x_l1
            self.vehicle_ini_yl = y_l1
            self.vehicle_ini_zl = z_l1
            self.is_dive_started = True

        home_lon_m, home_lat_m, _, _ = utm.from_latlon(self.home_lat_ghost, self.home_lon_ghost) 
        target_pos_x = target_lat_m - home_lat_m  
        target_pos_y = target_lon_m - home_lon_m
        target_pos_z = 0.0  
        
        point1 = np.array([self.vehicle_ini_xl, self.vehicle_ini_yl, 0.0])
        point2 = np.array([target_pos_x,target_pos_y,target_pos_z]) 
        point3 = point1 - point2
        point3 = point3/np.linalg.norm(point3)
        point4 = np.array([0.0,0.0, self.takeoff_height])
        point4 = point4/np.linalg.norm(point4)
        point5 = np.cross(point4,point3) 
        

        matrix1 = np.column_stack((point3, point5, point4))
        point6 = np.array([np.cos(self.strike_angle),0, np.sin(self.strike_angle)]).T
        point7 = np.matmul(matrix1,point6)
        speed_and_t = np.abs(self.vehicle_ini_zl/point7[2])
        point_from_target = (speed_and_t)*point7
        strike_radius = np.sqrt(point_from_target[0]**2 + point_from_target[1]**2)
        point_in_inertial = point_from_target + point2 
        point_in_inertial = np.array([point_in_inertial[0], point_in_inertial[1], -point_in_inertial[2]]) 


        dist_vector = point_in_inertial - np.array([x_l1, y_l1, z_l1])
        distance = np.linalg.norm(dist_vector)
        self.strike_radius_to_gcs.data = strike_radius + 2*rmin
        self.strike_radius_publisher.publish(self.strike_radius_to_gcs)

        if distance < 2*rmin:
            self.dist_midpoint = True
        
        waypoint0 = np.array([self.vehicle_ini_xl, self.vehicle_ini_yl,self.vehicle_ini_zl]) + np.array([1.0, 1.0,0.0])
        waypoint1 = point_in_inertial
       
        c_cal = 1.5/rmin

        
        if self.dist_midpoint is True:
            waypoint0 = point_in_inertial
            waypoint1 = np.array([target_pos_x, target_pos_y, target_pos_z])
            c_cal = 0.46/rmin

        vect1 =  waypoint1 - waypoint0
        unit_vect1 = vect1/np.linalg.norm(vect1)
        vect2 = np.array([x_l1, y_l1, z_l1]) - waypoint0
        unit_vect2 = vect2/np.linalg.norm(vect2)
        a = vect1[1]*unit_vect2[0] - vect1[0]*unit_vect2[1]
        
        if a >= 0:
            vect3 = np.cross(unit_vect1,unit_vect2)
        else:
            vect3 = np.cross(unit_vect2,unit_vect1)
    

        y_unit_vector = unit_vect1
        z_unit_vector = vect3/np.linalg.norm(vect3) 
        x_unit_vector  = np.cross(y_unit_vector,z_unit_vector)
        x_unit_vector  = x_unit_vector/np.linalg.norm(x_unit_vector) 

        rotation_matrix = np.column_stack((x_unit_vector, y_unit_vector, z_unit_vector))

        inv_rotation_matrix = rotation_matrix.T

        x_e_matrix = np.dot(inv_rotation_matrix,vect2)

        x_e = x_e_matrix[0]

        f_2 = speed0 * self.helper.sech(c_cal*x_e)
        f_1 = - speed0 * np.tanh(c_cal*x_e)
   

        vect3D = np.array([[f_1],[f_2],[0.0]])
        vector_field_3d = np.matmul(rotation_matrix,vect3D)
        neg_gradient_ang = np.arctan2(vector_field_3d[1], vector_field_3d[0])
        mag_xy = np.sqrt(vector_field_3d[0]**2 + vector_field_3d[1]**2)
        flight_path_angle = np.arctan2(vector_field_3d[2], mag_xy)

        pitch_angle = flight_path_angle
        current_yaw_angle = self.get_current_heading()

        w_max = np.abs(speed0/(rmin)) 
        
        chi_dot = w_max*(2/np.pi)*np.arctan(np.sinh(2.5*(self.helper.check_angle(neg_gradient_ang-current_yaw_angle)))) 
        roll_angle = np.arctan(chi_dot*speed0/9.81)

        yaw_angle = self.helper.check_angle(current_yaw_angle + w_max*(2/np.pi)*np.arctan(np.sinh(2.5*(self.helper.check_angle(neg_gradient_ang-current_yaw_angle))))) 

        # if self.cancel_flag:
        #     waypoint = np.array([target_pos_x, target_pos_y, target_pos_z])
        #     #roll_angle, pitch_angle, yaw_angle = self.cancel_straight_following(speed0, position1, waypoint, point_in_inertial)
        #     #self.publish_offboard_control_heartbeat_signal(position=True, attitude=False)
        #     #self.publish_trajectory_setpoint(0.0, 0.0, -200)
        #     self.get_logger().info("Returning Home and Loitering")
        
        return  float(roll_angle), float(pitch_angle), float(yaw_angle)


    ############## timer callback ##############

    def timer_callback(self) -> None:
        # print(self.command)
        if self.cancel_flag:
            self.handle_recovery_mode()
            return  # Skip all other logic during recovery

        if self.command == "strike_lat_long" or self.command == "dive":
            self.publish_offboard_control_heartbeat_signal(attitude=True, position=False)
            self.get_logger().info("Sending Attitude Offboard Heartbeat")
                
        if self.home_alt == 0.0 and self.vehicle_global_position.lat > 0.0:
            self.home_alt = self.vehicle_global_position.alt
            self.home_lat_ghost = self.vehicle_global_position.lat
            self.home_lon_ghost = self.vehicle_global_position.lon
            lat = f"Saved home lat : {self.home_lat}"
            lon = f"Saved home lon : {self.home_lon}"
            alt = f"Saved home alt : {self.home_alt}"
            self.get_logger().info(lat)
            self.get_logger().info(lon)
            self.get_logger().info(alt)
            self.home_flag = 1

        if self.command == "dive" or self.command == "target_geolocation" or self.command == "strike_lat_long":
            # climb a required minimum dive altitude
            if self.dive and abs(self.vehicle_odo.position[-1]) < (self.min_dive_start_alt - 5.0) and not self.climbed:
                if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.change_mode(4.0, 3.0, 'hold')
                elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.change_loiter_altitude(alt=self.min_dive_start_alt)

            # send a dive starting response to the user
            if self.dive and abs(self.vehicle_odo.position[-1]) > (self.min_dive_start_alt - 5.0):
                wadeoff_threshold_ht = 150
                wadeoff_threshold_res = json.dumps({"wadeoff_thresh":wadeoff_threshold_ht,
                                                    "dive_status":True,
                                                    "wadeoff_status":False,
                                                    "received":None})
                self.dive_res.data = wadeoff_threshold_res
                self.dive_res_publisher.publish(self.dive_res)
                # print("I climbed")
                self.climbed = True

            # Dive into the target
            if self.dive and self.climbed:
                if self.vehicle_status.nav_state !=VehicleStatus.NAVIGATION_STATE_OFFBOARD and not self.offboard_flag:
                    self.change_mode(6.0, 0.0, "offboard")
                    print("changing mode to offboard")

                elif self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    # print("inside algo")

                    if not self.offboard_flag:
                        self.offboard_flag = True

                    if self.command == "dive" and not self.cancel_flag:
                        if int(self.command_data["value"]) == 0:
                            self.cancel_flag = 1

                    roll_s, pitch_s, yaw_s = self.dive_single_vehicle(self.air_speed,
                                                                    self.vehicle_odo.position,
                                                                    self.target_position_x,
                                                                    self.target_position_y,
                                                                    )
                    
                    self.publish_attitude_setpoint(roll_s, pitch_s, yaw_s)


                    # if (self.cancel_flag 
                    #     and np.abs(self.vehicle_odo.position[2]) >= 200.0 
                    #     and self.helper.calculate_reached_threshold(self.vehicle_global_position.lat,
                    #                                     self.vehicle_global_position.lon,
                    #                                     self.home_lat,
                    #                                     self.home_lon,
                    #                                     self.threshold)):
                    #     print("Reached Recovery Wp....Going to HOLD")
                    #     self.change_mode(4.0, 3.0, 'hold')
                    #     self.dive = False
                    #     self.location = False
                    #     self.climbed = False
                    #     self.cancel_flag = False
                    #     # self.is_dive_started = False
                    #     self.command = None
                    #     wadeoff_threshold_res = json.dumps({"wadeoff_thresh":150,
                    #                                         "dive_status":False,
                    #                                         "wadeoff_status":True,
                    #                                         "received":None})
                    #     self.dive_res.data = wadeoff_threshold_res
                    #     self.dive_res_publisher.publish(self.dive_res)


            # Track the target
            if self.location == True and self.dive == False and not self.cancel_flag:
                if self.vehicle_status.nav_state !=VehicleStatus.NAVIGATION_STATE_OFFBOARD and not self.offboard_flag:
                    self.change_mode(6.0, 0.0, "offboard")
                    print("changing mode to offboard")

                elif self.vehicle_status.nav_state  == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

                    if not self.offboard_flag:
                        self.offboard_flag = True


                    if self.lock == True:
                        roll_s, pitch_s, yaw_s = self.dive_single_vehicle(self.air_speed,
                                                                    self.vehicle_odo.position,
                                                                    self.target_position_x,
                                                                    self.target_position_y,
                                                                    )
                    
                        self.publish_attitude_setpoint(roll_s, pitch_s,yaw_s)
                    
                    if self.lock == False:
                        print("tracking cancelled....loitering")
                        self.change_mode(4.0, 3.0, 'hold')
                        self.location = False
                        self.command = None



def main(args=None) -> None:
    print('Starting dive control node...')
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


