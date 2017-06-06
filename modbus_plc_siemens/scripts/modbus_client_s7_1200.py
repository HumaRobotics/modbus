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
from modbus_plc_siemens.s7_modbus_client import S7ModbusClient 
from std_msgs.msg import Int32MultiArray as HoldingRegister

if __name__=="__main__":
    rospy.init_node("modbus_client_s7_1200")
    rospy.loginfo("""
    This file shows the usage of the Modbus Wrapper Client.
    To see the read registers of the modbus server use: rostopic echo /modbus_wrapper/input
    To see sent something to the modbus use a publisher on the topic /modbus_wrapper/output
    This file contains a sample publisher.
    """)
    host = "192.168.0.198"
    port = 502
    if rospy.has_param("~ip"):
        host =  rospy.get_param("~ip")
    else:
        rospy.loginfo("For not using the default IP %s, add an arg e.g.: '_ip:=\"192.168.0.198\"'",host)
    if rospy.has_param("~port"):
        port =  rospy.get_param("~port")
    else:
        rospy.loginfo("For not using the default port %d, add an arg e.g.: '_port:=1234'",port)
    # setup modbus client for siemens s7 PLC  
    modclient = S7ModbusClient(host,port)
    rospy.loginfo("Setup complete")
        
    #################        
    # Example 1 
    # Set single registers using the python interface - yeah knight rider is coming!
    outputs = xrange(8,14)
    outputs2 = range(13,7,-1)
    #inputs = range(1,9) 
    
    # for i in xrange(5):
    #     for output in outputs:    
    #         modclient.setOutput(output,1,0.3)
    #         rospy.sleep(0.1)
    #     for output in outputs2:            
    #         modclient.setOutput(output,1,0.3)
    #         rospy.sleep(0.1)
    # rospy.sleep(0.3)
    #################
      
    #################
    # Example 2
    # Create a listener that show us a message if anything on the readable modbus registers change
    rospy.loginfo("All done. Listening to inputs... Terminate by Ctrl+c")
    def showUpdatedRegisters(msg):
        rospy.loginfo("Modbus server registers have been updated: %s",str(msg.data))
    sub = rospy.Subscriber("modbus_wrapper/input",HoldingRegister,showUpdatedRegisters,queue_size=500)
    rospy.spin()
    #################
    
     #################
    # Example 3
    # writing to modbus registers using a standard ros publisher
    
    pub = rospy.Publisher("modbus_wrapper/output",HoldingRegister,queue_size=500)
    output = HoldingRegister()
    output.data = [1 for x in xrange(0,6)]
    output2 = HoldingRegister()
    output2.data = [0 for x in xrange(0,6)]
     
    rospy.loginfo("Sending arrays to the modbus server")
    for i in xrange(5):
        rospy.sleep(1)
        pub.publish(output)
        rospy.sleep(1)
        pub.publish(output2)
    #################
    
    rospy.loginfo("Outputs tests all done, just listening to inputs. Stop listening by pressing Ctrl+c")
    # Stops the listener on the modbus
    modclient.stopListening()
    