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

# NEPI LSX Auto Discovery Script for SeaLite LED Lights



import os
os.environ["ROS_NAMESPACE"] = "/nepi/s2x"
import rospy
import serial
import serial.tools.list_ports
import time
import re
import subprocess

#Define Discovery Search Parameters
BAUDRATE_LIST = [9600,19200,57600] # Three supported baud rates
ADDRESS_LIST = list(range(1,255))  # Total range 1-255

#########################################
# Sealite Discover Method
#########################################


### Function to try and connect to device
def sealite_discover(baud_list,addr_list,active_port_list):
  dev_ports=[]
  dev_bauds=[]
  dev_addrs=[]
  dev_count = 0
  node_name = rospy.get_name().split('/')[-1]
  # Find serial ports
  rospy.logdebug(node_name + ": Looking for serial ports on device")
  port_list = []
  ports = serial.tools.list_ports.comports()
  for loc, desc, hwid in sorted(ports):
    rospy.logdebug(node_name + ": Found serial_port at: " + loc)
    port_list.append(loc)
  # Checking for devices on available serial ports
  if len(port_list) > 0:
    for port_str in port_list:
      if port_str not in active_port_list:
        for baud_int in baud_list:
          rospy.loginfo(node_name + ": Connecting to serial port " + port_str + " with baudrate: " + str(baud_int))
          try:
            # Try and open serial port
            rospy.loginfo(node_name + ": Opening serial port " + port_str + " with baudrate: " + str(baud_int))
            serial_port = serial.Serial(port_str,baud_int,timeout = 0.005)
            for addr in addr_list:
              addr_str = str(addr)
              zero_prefix_len = 3-len(addr_str)
              for z in range(zero_prefix_len):
                addr_str = ('0' + addr_str)
              # Create message string
              ser_msg= ('!' + addr_str + ':INFO?')
              ser_str = (ser_msg + '\r\n')
              # Send Serial String
              #print("")
              #print("Sending serial message: " + ser_msg)
              b=bytearray()
              b.extend(map(ord, ser_str))
              serial_port.write(b)
              #print("Waiting for response")
              time.sleep(.005)
              bs = serial_port.readline()
              response = bs.decode()
              if len(response) > 2:
                rospy.loginfo(node_name + ": Got response: " + response)
                if response[3] == ',':
                  addr_str = response[0:3]
                  try:
                    addr_int = int(addr)
                    rospy.loginfo(node_name + ": Found device at address: " + addr_str)
                    dev_ports.append(port_str)
                    dev_bauds.append(baud_int)
                    dev_addrs.append(addr_str)
                    dev_count = dev_count + 1
                  except Exception as a:
                    rospy.logwarnd(node_name + ": Returned device message not valid (" + str(a) + ")")
                    
            # Close the port afterwards
            rospy.loginfo(node_name + ": Closing serial port " + port_str)
            serial_port.close()
          except Exception as e:
            rospy.logwarn(node_name + ": Unable to open serial port " + port_str + " with baudrate: " + str(baud_int) + "(" + str(e) + ")")
      else:
        rospy.logwarn(node_name + ": Serial port allready active")
  else:
    rospy.logdebug(node_name + ": No serial ports found")
  rospy.logdebug(node_name + ": Found " + str(dev_count) + " new devices")
  for i in range(dev_count):
    rospy.loginfo(node_name + ": dev_ports[i] " + "  " + str(dev_bauds[i]) + " " + dev_addrs[i])
  return dev_ports,dev_bauds,dev_addrs,port_list


#########################################
# Main
#########################################

if __name__ == '__main__':
  active_port_list = []
  active_node_list = []
  active_subproc_list = []
    
  rospy.init_node('sealite_lsx_detector')
  node_name = rospy.get_name().split('/')[-1]
  rospy.loginfo(node_name + ": Starting")
  while not rospy.is_shutdown():
    # Run Discovery Process at Start
    dev_ports,dev_bauds,dev_addrs,port_list = sealite_discover(BAUDRATE_LIST,ADDRESS_LIST,active_port_list)
    rospy.loginfo_throttle(30, node_name + ": Current Port List = " + str(port_list))
    # Remove nodes from active list if no longer found
    updated_apl = []
    updated_anl = []
    for i in range(len(active_node_list)):
      ap = active_port_list[i]
      an = active_node_list[i]
##      print("Checking active port " + ap + " is still a valid serial port")
##      if ap not in port_list:
##        print("Port " + ap + " no longer available")
##        print("Killing node " + active_node_list[i] )
##        kill_proc = active_subproc_list[i]
##        kill_proc.terminate()
##      else:
      check_topic_name = (an + "/lsx/active")
      rospy.loginfo(node_name + ": Checking for topic name: " + check_topic_name)
      rospy.loginfo(node_name + ": Checking active node " + an + " is still a valid ros node")
      node_exists = False
      topic_list=rospy.get_published_topics(namespace='/')
      for topic_entry in topic_list:
        if topic_entry[0].find(check_topic_name) != -1:
          updated_apl.append(ap)
          updated_anl.append(an)
          node_exists = True
          rospy.loginfo(node_name + ": Found topic name")
          break
      if node_exists is False:
        rospy.loginfo(node_name + ": Node " + an + " no longer active")
      else:
        rospy.loginfo(node_name + ": Node " + an + " still active")
    active_port_list = updated_apl
    active_node_list = updated_anl
    time.sleep(1)
    # Try and create a LSX Node for each found device
    if len(port_list) > 0: 
      for i, port in enumerate(dev_ports):
        if port not in active_port_list:
          port_str = port.split("tty")[1]
          lsx_node_name = "sealite_" + port_str + "_" + dev_addrs[i]
          
          # First, load some params on param server for the node we are about to launch
          namespace = rospy.get_namespace()
          rospy.loginfo(node_name + ": Discover name: " + namespace)
          rospy.set_param(namespace + lsx_node_name + '/port_str', dev_ports[i])
          rospy.set_param(namespace + lsx_node_name + '/baud_int', dev_bauds[i])
          rospy.set_param(namespace + lsx_node_name + '/addr_str', dev_addrs[i])
          time.sleep(2)
          # Pass the name as a regular cmd-line arg since we can't rosrun this new node as it is not currently installed in ROS path
          node_run_cmd = ['rosrun', 'nepi_edge_sdk_sealite', 'sealite_lsx_driver_script.py', '__name:=' + lsx_node_name] 
          p = subprocess.Popen(node_run_cmd)
          active_port_list.append(dev_ports[i])
          active_node_list.append(lsx_node_name)
          active_subproc_list.append(p)
    else:
      rospy.logdebug(node_name + ": No devices found to connect to")  
    
    rospy.loginfo_throttle(30, node_name + ": Current active port list = " + str(active_port_list))
    rospy.loginfo_throttle(30, node_name + ": Current active node list = " + str(active_node_list))
    
    time.sleep(3)
