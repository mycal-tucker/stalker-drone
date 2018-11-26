#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 14 10:45:38 2018

@author: Rosemary
"""

import time
from networking.wifiConnection import WifiConnection
try:
    from networking.bleConnection import BLEConnection
    BLEAvailable = True
except:
    BLEAvailable = False
from utils.colorPrint import color_print
from commandsandsensors.DroneCommandParser import DroneCommandParser
from commandsandsensors.DroneSensorParser import DroneSensorParser
import math


# set this to true if you want to fly for the demo
testFlying = True

class DroneState:    
    # get the initial state information
        print("sleeping")
        mambo.smart_sleep(1)
        mambo.ask_for_state_update()
        mambo.smart_sleep(1)
        
    """
    Store the mambo's last known sensor values
    """

    def __init__(self):

        # default to full battery
        self.battery = 100

        # drone on the ground
        self.flying_state = "landed"

        # dictionary for extra sensors
        self.sensors_dict = dict()

        self.gun_id = 0
        self.gun_state = None

        self.claw_id = 0
        self.claw_state = None

        # new SDK sends speed, altitude, and quaternions
        self.speed_x = 0
        self.speed_y = 0
        self.speed_z = 0
        self.speed_ts = 0

        # these are only available on wifi
        self.altitude = -1
        self.altitude_ts = 0

        self.quaternion_w = 0
        self.quaternion_x = 0
        self.quaternion_y = 0
        self.quaternion_z = 0
        self.quaternion_ts = -1

        # this is optionally set elsewhere
        self.user_callback_function = None

        #These are being added to the drone state
        self.pos_x=0
        self.pos_y=0

    def set_user_callback_function(self, function, args):
        """
        Sets the user callback function (called everytime the sensors are updated)

        :param function: name of the user callback function
        :param args: arguments (tuple) to the function
        :return:
        """
        self.user_callback_function = function
        self.user_callback_function_args = args

    def update(self, name, value, sensor_enum):
        """
        Update the sensor

        :param name: name of the sensor to update
        :param value: new value for the sensor
        :param sensor_enum: enum list for the sensors that use enums so that we can translate from numbers to strings
        :return:
        """
        #print("updating sensor %s" % name)
        #print(value)
        if (name is None):
            print("Error empty sensor")
            return


        if (name, "enum") in sensor_enum:
            # grab the string value
            if (value > len(sensor_enum[(name, "enum")])):
                value = "UNKNOWN_ENUM_VALUE"
            else:
                enum_value = sensor_enum[(name, "enum")][value]
                value = enum_value


        # add it to the sensors
        if (name == "BatteryStateChanged_battery_percent"):
            self.battery = value
        elif (name == "FlyingStateChanged_state"):
            self.flying_state = value
        elif (name == "ClawState_id"):
            self.claw_id = value
        elif (name == "ClawState_state"):
            self.claw_state = value
        elif (name == "GunState_id"):
            self.gun_id = value
        elif (name == "GunState_state"):
            self.gun_state = value
        elif (name == "DroneSpeed_speed_x"):
            self.speed_x = value
        elif (name == "DroneSpeed_speed_y"):
            self.speed_y = value
        elif (name == "DroneSpeed_speed_z"):
            self.speed_z = value
        elif (name == "DroneSpeed_ts"):
            self.speed_ts = value
            self.pos_x = self.speed_x * self.speed_ts
            self.pos_y = self.speed_y * self.speed_ts
        elif (name == "DroneAltitude_altitude"):
            self.altitude = value
        elif (name == "DroneAltitude_ts"):
            self.altitude_ts = value
        elif (name == "DroneQuaternion_q_w"):
            self.quaternion_w = value
        elif (name == "DroneQuaternion_q_x"):
            self.quaternion_x = value
        elif (name == "DroneQuaternion_q_y"):
            self.quaternion_y = value
        elif (name == "DroneQuaternion_q_z"):
            self.quaternion_z = value
        elif (name == "DroneQuaternion_ts"):
            self.quaternion_ts = value
        else:
            #print "new sensor - add me to the struct but saving in the dict for now"
            self.sensors_dict[name] = value

        # call the user callback if it isn't None
        if (self.user_callback_function is not None):
            self.user_callback_function(self.user_callback_function_args)

    def __str__(self):
        """
        Printed struct for debugging

        :return: string for print calls
        """
        my_str = "mambo state: battery %d, " % self.battery
        my_str += "flying state is %s, " % self.flying_state
        my_str += "speed (x, y, z) and ts is (%f, %f, %f) at %f " % (self.speed_x, self.speed_y, self.speed_z, self.speed_ts)
        if (self.altitude_ts > -1):
            my_str += "altitude (m) %f and ts is %f " % (self.altitude, self.altitude_ts)

        if (self.quaternion_ts > -1):
            my_str += "quaternion (w, x, y, z) and ts is (%f, %f, %f, %f) at %f " % (
                self.quaternion_w, self.quaternion_x, self.quaternion_y, self.quaternion_z, self.quaternion_ts)
        my_str += "gun id: %d, state %s, " % (self.gun_id, self.gun_state)
        my_str += "claw id: %d, state %s, " % (self.claw_id, self.claw_state)
        my_str += "extra sensors: %s," % self.sensors_dict
        return my_str

class Mambo:
    def __init__(self, address, use_wifi=False):
        """
        Initialize with its BLE address - if you don't know the address, call findMambo
        and that will discover it for you.

        You can also connect to the wifi on the FPV camera.  Do not use this if the camera is not connected.  Also,
        ensure you have connected your machine to the wifi on the camera before attempting this or it will not work.

        :param address: unique address for this mambo
        :param use_wifi: set to True to connect with wifi as well as the BLE
        """
        self.address = address
        self.use_wifi = use_wifi
        if (use_wifi):
            self.drone_connection = WifiConnection(self, drone_type="Mambo")
        else:
            if (BLEAvailable):
                self.drone_connection = BLEConnection(address, self)
            else:
                self.drone_connection = None
                color_print(
                    "ERROR: you are trying to use a BLE connection on a system that doesn't have BLE installed.",
                    "ERROR")
                return

        # intialize the command parser
        self.command_parser = DroneCommandParser()

        # initialize the sensors and the parser
        self.sensors = DroneState()
        self.sensor_parser = DroneSensorParser(drone_type="Mambo")

    def set_user_sensor_callback(self, function, args):
        """
        Set the (optional) user callback function for sensors.  Every time a sensor
        is updated, it calls this function.

        :param function: name of the function
        :param args: tuple of arguments to the function
        :return: nothing
        """
        self.sensors.set_user_callback_function(function, args)

    def update_sensors(self, data_type, buffer_id, sequence_number, raw_data, ack):
        """
        Update the sensors (called via the wifi or ble connection)

        :param data: raw data packet that needs to be parsed
        :param ack: True if this packet needs to be ack'd and False otherwise
        """

        sensor_list = self.sensor_parser.extract_sensor_values(raw_data)
        if (sensor_list is not None):
            for sensor in sensor_list:
                (sensor_name, sensor_value, sensor_enum, header_tuple) = sensor
                if (sensor_name is not None):
                    self.sensors.update(sensor_name, sensor_value, sensor_enum)
                    #return x,y,z, roll,pith, yaw, x_speed, y_speed, z_speed, roll_speed, pitch_speed, yaw_speed
                    (roll,pitch,yaw)=self.sensors.quaternion_to_euler_angle(self.quaternion_w, self.quaternion_x,
                                                   self.quaternion_y, self.quaternion_z)
                    #not sure how to calculate roll_speed, pitch_speed, and yaw_speed, so setting equal to 0
                    return [self.pos_x,self.pos_y,self.altitude,roll,pitch,yaw,self.speed_x,self.speed_y,self.speed_z.self,0,0,0]
                    # print(self.sensors)
                else:
                    color_print(
                        "data type %d buffer id %d sequence number %d" % (data_type, buffer_id, sequence_number),
                        "WARN")
                    color_print("This sensor is missing (likely because we don't need it)", "WARN")

        
 ## end of code from online       
    
    def set_user_callback_function(self, user_callback_function=None, user_callback_args=None):   

        if (mambo.sensors.flying_state != "emergency"):
            print("flying state is %s" % mambo.sensors.flying_state)
    
        else:
            print("Sleeeping for 15 seconds")
            mambo.smart_sleep(15)



  
    
