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
from modbus.modbus_wrapper_server import ModbusWrapperServer
from std_msgs.msg import Int32MultiArray as HoldingRegister
    
if __name__=="__main__":
    rospy.init_node("modbus_server")
    port = 1234 # custom modbus port without requirement of sudo rights
#     port = 502 # default modbus port
    if rospy.has_param("~port"):
        port =  rospy.get_param("~port")
    else:
        rospy.loginfo("For not using the default port %d, add an arg e.g.: '_port:=1234'",port)
    # Init modbus server with specific port
    mws = ModbusWrapperServer(port)
    # Stop the server if ros is shutdown. This should show that the server is stoppable 
    rospy.on_shutdown(mws.stopServer)
    # Starts the server in a non blocking call
    mws.startServer()
    print "Server started"
    
    ###############
    # Example 1
    # write to the Discrete Input
    mws.setDigitalInput(0,1) # args: address , value. sets address to value
    # Example 2
    # read from clients coil output
    print "waiting for line 0 to be set to True"
    result = mws.waitForCoilOutput(0,5) # args: address,timeout in sec. timeout of 0 is infinite. waits until address is true
    if result:
        print "got line 0 is True from baxter"
    else:
        print "timeout waiting for signal on line 0"
    
    
    ###############
    # Example 3
    # Listen for the writeable modbus registers in any node
    def callback(msg):
        rospy.loginfo("Modbus register have been written: %s",str(msg.data))
        rospy.sleep(2)
    sub = rospy.Subscriber("modbus_server/read_from_registers",HoldingRegister,callback,queue_size=500) 
    ###############
    
    
    ###############
    # Example 4
    # Publisher to write first 20 modbus registers from any node
    pub = rospy.Publisher("modbus_server/write_to_registers",HoldingRegister,queue_size=500)
    rospy.sleep(1)
    msg = HoldingRegister()
    msg.data = range(20)
    msg2 = HoldingRegister()
    msg2.data = range(20,0,-1)
 
    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.sleep(1)
        pub.publish(msg2)
        rospy.sleep(1)
    ################
    rospy.spin()
        
    mws.stopServer()
