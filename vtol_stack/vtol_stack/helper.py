#!/usr/bin/env python3
import numpy as np
import math
import utm
from netifaces import interfaces, ifaddresses, AF_INET
from math import nan, isnan, atan2, pi, sqrt, asin



# uav helper functions 
class UavHelper:
    def __init__(self):
        pass

    # def get_host_ip(self):
    #     """Retrieve the host IP address."""
    #     try:
    #         # Create a dummy socket to determine the primary IP
    #         s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #         s.connect(("8.8.8.8", 80))  # Use a public DNS server to determine the interface
    #         ip_address = s.getsockname()[0]
    #         s.close()
    #         return ip_address
    #     except Exception as e:
    #         print(f"Error determining host IP: {e}")
    #         return "127.0.0.1"  # Fallback to localhost

    def get_host_ip(self):
        # addrs = [ifaddresses("enp4s0")[AF_INET]] #, ifaddresses("eth1")[AF_INET]
        # jetson_ip = ""
        # for i in range(len(addrs)):
        #     add = addrs[i][0]
        #     ip = add["addr"]
        #     splitted_ip = ip.split(".")
        #     first_two = ".".join(splitted_ip[:2])
        #     last_digit = str(splitted_ip[-1])
        #     if first_two == "10.223" or first_two == "10.10" or last_digit == "46":
        #         print("\n\n\nfound ip : ", ip)
        #         jetson_ip = ip
        #         break
        #print(doodle_ip)
        jetson_ip = "10.10.10.153"
        return jetson_ip
    
    
    def check_angle(self, x):
        if x > np.pi:
            return x - 2 * np.pi
        elif x < -np.pi:
            return x + 2 * np.pi
        else:
            return x
        

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return yaw_z # in radians
        
    
    

    def latlon_to_m(self, lat, lon):
        y, x, _, _ = utm.from_latlon(lat, lon)
        # latx = 6371000 * lat * (np.pi/180)
        # lony = 6371000 * lon * (np.pi/180)
        return x, y
    

    def m_to_latlon(self, x, y):
        lat, lon = utm.to_latlon(y, x, 43, 'Q')
        # lat = x / (6371000 * (np.pi/180))
        # lon = y / (6371000 * (np.pi/180))
        return lat, lon
    

    def get_eucledian_distance(self, x1,y1,x2,y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)
    

    def get_resultant(self, x, y):
        return np.sqrt(x**2 + y**2)
    

    def get_battery_percentage(self, battery_voltage, max_voltage):
        return int((battery_voltage/max_voltage) * 100)
    

    def calculate_reached_threshold(self, uav_lat, uav_lon, wp_lat, wp_lon, threshold):
        uav_x, uav_y = self.latlon_to_m(uav_lat, uav_lon)
        wp_x, wp_y = self.latlon_to_m(wp_lat, wp_lon)
        distance = self.get_eucledian_distance(uav_x, uav_y, wp_x, wp_y)
        if distance < threshold:
            return True
        # else:
        #     print(distance)
        return False
    

    def sech(self, x):
        return 2/((np.exp(x))+(np.exp(-x)))
    

    def home_distance(self, vehicle_lat, vehicle_lon, home_lat, home_lon):
        vehicle_gx, vehicle_gy = self.latlon_to_m(vehicle_lat, vehicle_lon)
        home_gx, home_gy = self.latlon_to_m(home_lat, home_lon)
        home_distance = self.get_eucledian_distance(vehicle_gx, vehicle_gy, home_gx, home_gy)
        return home_distance
    


