#!/usr/bin/env python
#
# NEPI Dual-Use License
# Project: nepi_sample_auto_scripts
#
# This license applies to any user of NEPI Engine software
#
# Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
# see https://github.com/numurus-nepi/nepi_edge_sdk_base
#
# This software is dual-licensed under the terms of either a NEPI software developer license
# or a NEPI software commercial license.
#
# The terms of both the NEPI software developer and commercial licenses
# can be found at: www.numurus.com/licensing-nepi-engine
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - https://www.numurus.com/licensing-nepi-engine
# - mailto:nepi@numurus.com
#
#

# NEPI LSX Driver Script for SeaLite LED Lights


### Set the namespace before importing rospy
import os
os.environ["ROS_NAMESPACE"] = "/nepi/s2x"
import rospy
import serial
import serial.tools.list_ports
import time
import re
import sys

from std_msgs.msg import UInt8, Empty, String, Bool, Float32
from nepi_ros_interfaces.msg import LSXStatus
from nepi_ros_interfaces.srv import LSXCapabilitiesQuery, LSXCapabilitiesQueryResponse

#########################################
# Sealite LSX Driver Node Class
#########################################

class Sealite_Node(object):
  #######################
  DEFAULT_NODE_NAME='lsx_sealite'
  ### LXS Driver Settings
  # Set driver capability parameters
  HAS_STANDBY_MODE = True
  HAS_INTENSITY_CONTROL = True
  HAS_HW_STROBE_CONTROL = True
  REPORTS_TEMP = True
  # Set Initial LSX Parameters
  STANDBY = False
  INTENSITY = 0.0
  STROBE_ENABLE = False
  
  #######################
  ### LXS Driver NODE Initialization
  def __init__(self,port_str,buad_int,addr_str):
    self.connected = False  
    self.ser_port_str = port_str
    self.ser_buad_int = buad_int
    self.dev_addr_str = addr_str
    port_str = port.split("tty")[1]
    self.node_name = rospy.get_name().split('/')[-1]
    
    self.standby = False
    self.intensity = 0.0
    self.strobe_enable = False
    self.temp_c = 0
    self.check_count = 0 # track sequencial message failures
    rospy.loginfo(self.node_name + ": Starting Initialization")
    # Create LSX Namespaces
    port_str = self.ser_port_str.split("tty")[1]
    NEPI_LSX_BASENAME = rospy.get_namespace() + self.node_name  + "/lsx/"
    NEPI_LSX_CAPABILITY_REPORT_SERVICE = NEPI_LSX_BASENAME + "capabilities_query"
    NEPI_LSX_STATUS_TOPIC = NEPI_LSX_BASENAME + "status"
    NEPI_LSX_ACTIVE_TOPIC = NEPI_LSX_BASENAME + "active" # Use to check for node shutdown
    NEPI_LSX_SET_STANDBY_TOPIC = NEPI_LSX_BASENAME + "set_standby"
    NEPI_LSX_SET_INTENSITY_TOPIC = NEPI_LSX_BASENAME + "set_intensity"
    NEPI_LSX_SET_STROBE_ENABLE_TOPIC = NEPI_LSX_BASENAME + "set_strobe_enable"
    ### LXS Global Variables
    self.serial_port = None
    self.serial_busy = False
    self.lxs_capabilities_report = LSXCapabilitiesQueryResponse()
    self.lxs_status_pub = rospy.Publisher(NEPI_LSX_STATUS_TOPIC, LSXStatus, queue_size=1, latch=True)
    self.lxs_active_pub = rospy.Publisher(NEPI_LSX_ACTIVE_TOPIC, Bool, queue_size=1, latch=False)     
    # Initialize some parameters
    self.serial_port = None
    self.serial_num = ""
    self.hw_version = ""
    self.sw_version = ""
    self.standby = self.STANDBY
    self.intensity = self.INTENSITY
    self.strobe_enable = self.STROBE_ENABLE
    temp_c = 0.0
    ### Try and connect to device
    self.connected = self.lsx_connect()
    if self.connected:
      # Initialize Device settings
      self.set_standby(self.STANDBY)
      self.set_intensity(self.INTENSITY)
      self.set_strobe_enable(self.STROBE_ENABLE)
      
      # Create LSX ROS node
      rospy.loginfo(self.node_name + ': Connected')
      
      # Publish status message
      self.lsx_status_callback()
      # Populate and advertise LSX node capabilities report
      self.lsx_capabilities_report = LSXCapabilitiesQueryResponse()
      self.lsx_capabilities_report.has_standby_mode = self.HAS_STANDBY_MODE
      self.lsx_capabilities_report.has_intensity_control = self.HAS_INTENSITY_CONTROL
      self.lsx_capabilities_report.has_hw_strobe = self.HAS_HW_STROBE_CONTROL
      self.lsx_capabilities_report.reports_temperature = self.REPORTS_TEMP
      rospy.Service(NEPI_LSX_CAPABILITY_REPORT_SERVICE, LSXCapabilitiesQuery, self.lsx_capabilities_query_callback)
      # Start LSX node control subscribers
      rospy.loginfo(self.node_name + ": Starting LSX control subscribers")
      rospy.Subscriber(NEPI_LSX_SET_STANDBY_TOPIC, Bool, self.lsx_set_standby_callback, queue_size = 1)
      rospy.Subscriber(NEPI_LSX_SET_INTENSITY_TOPIC, Float32, self.lsx_set_intensity_callback, queue_size = 1)
      rospy.Subscriber(NEPI_LSX_SET_STROBE_ENABLE_TOPIC, Bool, self.lsx_set_strobe_enable_callback, queue_size = 1)
      # Start an LSX node activity check process that kills node after some number of failed comms attempts
      rospy.loginfo("Starting an activity check process")
      rospy.Timer(rospy.Duration(1), self.lsx_status_timer_callback)
      rospy.Timer(rospy.Duration(.1), self.lsx_check_timer_callback)
      # Initialization Complete
      rospy.loginfo(self.node_name + ": Initialization Complete")
    else:
      rospy.loginfo(self.node_name + ": Shutting down node")
      rospy.loginfo(self.node_name + ": Specified serial port not available")
      rospy.signal_shutdown("Serial port not available")   

  #######################
  ### Class Functions and Callbacks


  ## Callback to regulary check device comms, track failures, and kill unresponsive device connections
  def lsx_status_timer_callback(self,timer):
    #Update the status message
    self.lsx_status_callback()
    
  def lsx_check_timer_callback(self,timer):
    success = False
    port_check = self.check_port(self.ser_port_str)
    if port_check is True:
      if self.serial_port is not None and not rospy.is_shutdown():
        ser_msg= ('!' + self.dev_addr_str + ':INFO?')
        ser_str = (ser_msg + '\r\n')
        b=bytearray()
        b.extend(map(ord, ser_str))
        try:
          while self.serial_busy == True and not rospy.is_shutdown():
            time.sleep(0.01) # Wait for serial port to be available
          self.serial_busy = True
          self.serial_port.write(b)
          time.sleep(.01)
          bs = self.serial_port.readline()
          self.serial_busy = False
          response = bs.decode()
          # Check for valid response 
          if response != None and response != "?" and response[3] == ",":
            success = True
        except Exception as e:
          rospy.logwarn(self.node_name + ": Failed to send or recieve message")
      else:
        rospy.logwarn(self.node_name + ": serial port not defined or ROS shutdown")
    else:
      rospy.logwarn(self.node_name + ": Shutting down device: " +  self.dev_addr_str + " on port " + self.ser_port_str)
      #rospy.loginfo(self.node_name + ": serial port not found")
      rospy.signal_shutdown("Too many comm failures")   
    # Update results and take actions
    if success:
      self.serial_busy = False # Clear the busy indicator
      self.check_count = 0 # reset comms failure count
      self.lxs_active_pub.publish(data=True)
    else:
      self.serial_busy = True # Lock port until valid response
      self.check_count = self.check_count + 1 # increment counter
      self.lxs_active_pub.publish(data=False)
    #print("Current failed comms count: " + str(self.check_count))
    if self.check_count > 0:  # Crashes node if set above 0??
      rospy.logwarn(self.node_name + ": Shutting down device: " +  self.dev_addr_str + " on port " + self.ser_port_str)
      rospy.logwarn(self.node_name + ": Too many comm failures")
      rospy.signal_shutdown("To many comm failures")   
   
  ### Function to try and connect to device at given port and buadrate
  def lsx_connect(self):
    success = False
    port_check = self.check_port(self.ser_port_str)
    if port_check is True:
      try:
        # Try and open serial port
        rospy.loginfo(self.node_name + ": Opening serial port " + self.ser_port_str + " with buadrate: " + str(self.ser_buad_int))
        self.serial_port = serial.Serial(self.ser_port_str,self.ser_buad_int,timeout = 0.1)
        rospy.loginfo(self.node_name + ": Serial port opened")
        # Send Message
        rospy.loginfo(self.node_name + ": Requesting info for device: " + self.dev_addr_str)
        ser_msg = ('!' + self.dev_addr_str + ':INFO?')
        rospy.loginfo(self.node_name + ": Sending serial string: " + ser_msg)
        response = self.send_msg(ser_msg)
        if response != None and response != "?" and response[3] == ",":
          if len(response) > 2:
            ret_addr = response[0:3]
            rospy.loginfo(self.node_name + ": Returned address value: " + ret_addr)
            if ret_addr == self.dev_addr_str:
              rospy.loginfo(self.node_name + ": Connected to device at address: " +  self.dev_addr_str)
              res_split = response.split(',')
              if len(res_split) > 4:
              # Update serial, hardware, and software status values
                self.serial_num = res_split[2]
                self.hw_version = res_split[3]
                self.sw_version = res_split[4]
              success = True
              self.lxs_active_pub.publish(data=True)
            else:
              rospy.logwarn(self.node_name + ": Device returned address: " + ret_addr + " does not match: " +  self.dev_addr_str)
          else:
            rospy.logwarn(self.node_name + ": Device returned invalid response")
        else:
          rospy.logwarn(self.node_name + ": Device returned invalid response")
      except Exception as e:
        rospy.logwarn(self.node_name + ": Something went wrong with connect function at serial port at: " + self.ser_port_str + "(" + str(e) + ")" )
    else:
      rospy.logwarn(self.node_name + ": serial port not active")
    return success

  ### callback to provide capabilities report ###
  def lsx_capabilities_query_callback(self, _):
    return self.lsx_capabilities_report

  ### Status callback
  def lsx_status_callback(self):
    # update status values from device
    success=self.update_status_values()
    # Create LSX status message
    status_msg=LSXStatus()
    status_msg.serial_num = self.serial_num
    status_msg.hw_version = self.hw_version
    status_msg.sw_version = self.sw_version
    status_msg.standby = self.standby
    status_msg.intensity = self.intensity
    status_msg.strobe_enable = self.strobe_enable
    status_msg.temp_c = self.temp_c
    if not rospy.is_shutdown():
      self.lxs_status_pub.publish(status_msg)

  ### Function to upadate status data
  def update_status_values(self):
    success = True
    # Update standby status
    rospy.loginfo(self.node_name + ": Updating standby status")
    ser_msg= ('!' + self.dev_addr_str + ':STBY?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      if response == "0":
        self.standby = False
      elif response == "1":
        self.standby = True
      else:
        success = False
      rospy.loginfo(self.node_name + ": Standby: " + str(self.standby))
    else:
      success = False
    # Update intensity status
    rospy.loginfo(self.node_name + ": Updating intensity status")
    ser_msg= ('!' + self.dev_addr_str + ':LOUT?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      try:
        self.intensity = float(response)/100
        rospy.loginfo(self.node_name + ": Intensity Ratio: " + str(self.intensity))
      except Exception as i:
        self.intensity = -999
        rospy.logwarn(self.node_name + ": Level response was not valid number")
        success = False
      rospy.loginfo(self.node_name + ": Intensity: " + str(self.intensity))
    else:
      self.intensity = -999
      success = False
    # Update strobe enable status
    rospy.loginfo(self.node_name + ": Updating strobe enable status")
    ser_msg= ('!' + self.dev_addr_str + ':PMOD?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      if response == "0":
        self.strobe_enable = False
      elif response == "1" or response == "2":
       self.strobe_enable = True
      else:
        success = False
      rospy.loginfo(self.node_name + ": Strobe Enable: " + str(self.strobe_enable))
    else:
      success = False
    # Update temp status
    rospy.loginfo(self.node_name + ": Updating temp status")
    ser_msg= ('!' + self.dev_addr_str + ':TEMP?')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      try:
        self.temp_c = int(float(response))
        rospy.loginfo(self.node_name + ": Temp Deg C: " + str(self.temp_c))
      except Exception as t:
        self.temp_c = 255
        rospy.logwarn(self.node_name + ": Temp response was not valid number")
        success = False
      rospy.loginfo(self.node_name + ": Temp C: " + str(self.temp_c))
    else:
      self.temp_c = 255
      success = False
    return success

  ### Set standby callback
  def lsx_set_standby_callback(self, standby_msg):
    rospy.loginfo(self.node_name + ": Recieved standby message: (" + str(standby_msg) + ")")
    standby=standby_msg.data
    self.set_standby(standby)
    self.lsx_status_callback()

  ### Function for setting standby mode
  def set_standby(self,standby_val):
    success = False
    if standby_val == True:
      ser_msg= ('!' + self.dev_addr_str + ':STBY=1')
    else:
      ser_msg= ('!' + self.dev_addr_str + ':STBY=0')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      success = True
    return success 

  ### Set intensity callback
  def lsx_set_intensity_callback(self, intensity_msg):
    rospy.loginfo(self.node_name + ": Recieved intensity message (" + str(intensity_msg) + ")")
    intensity=intensity_msg.data
    self.set_intensity(intensity)
    self.lsx_status_callback()

  ### Function for setting standby mode
  def set_intensity(self,intensity_ratio):
    success = False
    if intensity_ratio < 0:
      intensity_ratio = 0
    elif intensity_ratio > 1:
      intensity_ratio = 1
    level_val = int(100*intensity_ratio)
    level_str = str(level_val)
    zero_prefix_len = 3-len(level_str)
    for z in range(zero_prefix_len):
      level_str = ('0' + level_str)
    ser_msg= ('!' + self.dev_addr_str + ':LOUT=' +  level_str)
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      success = True
    return success 

  ### Set strobe enable callback
  def lsx_set_strobe_enable_callback(self, strobe_enable_msg):
    rospy.loginfo(self.node_name + ": Recieved strobe enable message (" + str(strobe_enable_msg) + ")")
    strobe_enable=strobe_enable_msg.data
    self.set_strobe_enable(strobe_enable)
    self.lsx_status_callback()

  ### Function for setting strobe enable mode
  def set_strobe_enable(self,strobe_enable_val):
    success = False
    if strobe_enable_val == True:
      ser_msg= ('!' + self.dev_addr_str + ':PMOD=1')
    else:
      ser_msg= ('!' + self.dev_addr_str + ':PMOD=0')
    response = self.send_msg(ser_msg)
    if response != None and response != "?":
      success = True
    return success  

  def send_msg(self,ser_msg):
    response = None
    if self.serial_port is not None and not rospy.is_shutdown():
      ser_str = (ser_msg + '\r\n')
      b=bytearray()
      b.extend(map(ord, ser_str))
      try:
        while self.serial_busy == True and not rospy.is_shutdown():
          time.sleep(0.01) # Wait for serial port to be available
        self.serial_busy = True
        #print("Sending " + ser_msg + " message")
        self.serial_port.write(b)
        time.sleep(.01)
        bs = self.serial_port.readline()
        self.serial_busy = False
        response = bs.decode()
        #print("Send response received: " + response[0:-2])
      except Exception as e:
        print("Failed to send or recieve message")
    else:
      print("serial port not defined, returning empty string")
    return response

  ### Function for checking if port is available
  def check_port(self,port_str):
    success = False
    ports = serial.tools.list_ports.comports()
    for loc, desc, hwid in sorted(ports):
      if loc == port_str:
        success = True
    return success

  #######################
  ### Cleanup processes on node shutdown
  def cleanup_actions(self):
    rospy.loginfo(self.node_name + ": Shutting down: Executing script cleanup actions")
    self.lxs_status_pub.unregister()
    if self.serial_port is not None:
      self.serial_port.close()
      
if __name__ == '__main__':
  
  rospy.init_node(Sealite_Node.DEFAULT_NODE_NAME)
  name = rospy.get_name()
  rospy.loginfo("Starting: " + name)
  
  # Get the parameters from the param server
  port = rospy.get_param('~port_str') # Crash if none provided
  baud = rospy.get_param('~baud_int') # Crash if none provided
  addr = rospy.get_param('~addr_str') # Crash if none provided
  node = Sealite_Node(port,baud,addr)
  rospy.spin() # Won't return until node is killed
  
  node.cleanup_actions()

