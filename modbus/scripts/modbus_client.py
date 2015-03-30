#!/usr/bin/env python
########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the Simplified BSD License on
# github: git@www.humarobotics.com:baxter_tasker
# HumaRobotics is a trademark of Generation Robots.
# www.humarobotics.com 

# Copyright (c) 2013, Generation Robots.
# All rights reserved.
# www.generationrobots.com
#   
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation 
#  and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
# THE POSSIBILITY OF SUCH DAMAGE. 
# 
# The views and conclusions contained in the software and documentation are 
# those of the authors and should not be interpreted as representing official 
# policies, either expressed or implied, of the FreeBSD Project.

import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient 
from std_msgs.msg import Int32MultiArray as HoldingRegister

NUM_REGISTERS = 20
ADDRESS_READ_START = 40000
ADDRESS_WRITE_START = 40020

if __name__=="__main__":
    rospy.init_node("modbus_client")
    rospy.loginfo("""
    This file shows the usage of the Modbus Wrapper Client.
    To see the read registers of the modbus server use: rostopic echo /modbus_wrapper/input
    To see sent something to the modbus use a publisher on the topic /modbus_wrapper/output
    This file contains a sample publisher.
    """)
    host = "192.168.0.123"
    port = 502
    if rospy.has_param("~ip"):
        host =  rospy.get_param("~ip")
    else:
        rospy.loginfo("For not using the default IP %s, add an arg e.g.: '_ip:=\"192.168.0.199\"'",host)
    if rospy.has_param("~port"):
        port =  rospy.get_param("~port")
    else:
        rospy.loginfo("For not using the default port %d, add an arg e.g.: '_port:=1234'",port)
    # setup modbus client    
    modclient = ModbusWrapperClient(host,port=port,rate=50,reset_registers=False,sub_topic="modbus_wrapper/output",pub_topic="modbus_wrapper/input")
    modclient.setReadingRegisters(ADDRESS_READ_START,NUM_REGISTERS)
    modclient.setWritingRegisters(ADDRESS_WRITE_START,NUM_REGISTERS)
    rospy.loginfo("Setup complete")
    
    # start listening to modbus and publish changes to the rostopic
    modclient.startListening()
    rospy.loginfo("Listener started")
    
    #################
    # Example 1
    # Sets an individual register using the python interface, which can automatically be reset, if a timeout is given.
    register = 40020
    value = 1
    timeout = 0.5
    modclient.setOutput(register,value,timeout)
    rospy.loginfo("Set and individual output")
    #################
    
    
    
    #################
    # Example 2
    # Create a listener that show us a message if anything on the readable modbus registers change
    rospy.loginfo("All done. Listening to inputs... Terminate by Ctrl+c")
    def showUpdatedRegisters(msg):
        rospy.loginfo("Modbus server registers have been updated: %s",str(msg.data))
    sub = rospy.Subscriber("modbus_wrapper/input",HoldingRegister,showUpdatedRegisters,queue_size=500)
    #################
    
    #################
    # Example 3
    # writing to modbus registers using a standard ros publisher
    pub = rospy.Publisher("modbus_wrapper/output",HoldingRegister,queue_size=500)
    output = HoldingRegister()
    output.data = xrange(20,40)
    output2 = HoldingRegister()
    output2.data = xrange(40,20,-1)
    
    rospy.loginfo("Sending arrays to the modbus server")
    while not rospy.is_shutdown():
        rospy.sleep(1)
        pub.publish(output)
        rospy.sleep(1)
        pub.publish(output2)
    #################
    
    # Stops the listener on the modbus
    modclient.stopListening()
    
    
   
    
    