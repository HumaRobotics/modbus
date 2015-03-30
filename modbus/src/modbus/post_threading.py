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
#
#############################################################################

"""  
    Provides classes for quick threading of object methods
    
    In your class, just add this line to the __init__:
        self.post=Post(self)
        
    You can now call any object methods with object.post.method(args)
    The Thread object is returned.
    
"""


from threading import Thread

class PostThread(Thread):
    """ Object that manages the threaded execution of a given function """
    
    def __init__(self,func):
        """ Creates the thread object with the method to be called """
        Thread.__init__(self,)
        self.func=func
        self.isRunning=True
        self.result=None
        self.daemon=True
    
    def execute(self,*args,**kwargs):
        """ Store the method call arguments and start the thread, returns the thread object to the caller """
        self.args=args
        self.kwargs=kwargs
        self.start()
        return self

    def run(self):
        """ Thread execution, call the function with saved arguments, saves result and change flag at the end """
        self.result=self.func(*self.args,**self.kwargs)
        self.isRunning=False

    def kill(self):
        if self.isAlive():
            self._Thread__stop()
            
class Post:
    """ Object that provides threaded calls to its parent object methods """
    def __init__(self,parent):
        self.parent=parent
    
    def __getattr__(self,attr):
        """ Find the method asked for in parent object, encapsulate in a PostThread object and send back pointer to execution function"""
        try:
            func=getattr(self.parent,attr)
            post_thread=PostThread(func)
            return post_thread.execute 
        except:
            raise Exception("ERROR: Post call on %s: method %s not found"%(str(self.parent),attr))


if __name__=="__main__":
    class Dummy:
        def __init__(self):
            self.post=Post(self)
        
        def do(self,param):
            import time
            print "Doing... param="+str(param)
            time.sleep(2)
            print "Done"
            return param
            

    dummy=Dummy()
    dummy.do("direct1")
    dummy.post.do("post1")
    dummy.post.do("post2")
    t3=dummy.post.do("post3")
    t3.join()
    print t3.result
    print "Finished"
    