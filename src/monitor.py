#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Python node to monitor Jetson usage"""

import rospy

from std_msgs.msg import Float32

# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

# Import custom message data and dynamic reconfigure variables.
from jetson_botdoc.cfg import docConfig

class JetsonDoctor(object):
    """Node example class."""

    def __init__(self):
        """Read in parameters."""
        """Read in parameters."""
        rate = rospy.get_param('~rate', 1.0)
        self.enable = True
        self.server = DynamicReconfigureServer(docConfig, self.reconfigure_cb)
        self.pub = rospy.Publisher('gpu_usage', Float32, queue_size=10)

        self.enable = rospy.get_param('~enable', True)
        self.message = rospy.get_param('~message', 'hello')

        if self.enable:
            self.start()
        else:
            self.stop()

        rospy.Timer(rospy.Duration(1.0 / rate), self.timed_cb)

    def start(self):
        """Turn on publisher."""
        self.pub = rospy.Publisher('gpu_usage', Float32, queue_size=10)

    def stop(self):
        """Turn off publisher."""
        self.pub.unregister()

    def timed_cb(self, _event):
        """Call at a specified interval to publish message."""
        if not self.enable:
            return

        msg = Float32()
        msg.data = 32.00
        self.pub.publish(msg)

    def reconfigure_cb(self, config, dummy):
        """Create a callback function for the dynamic reconfigure server."""
        if self.enable != config["enable"]:
            if config["enable"]:
                self.start()
            else:
                self.stop()
        self.enable = config["enable"]
        return config


# Main function.
if __name__ == '__main__':
    rospy.init_node('jetson_botdoc')
    try:
        JetsonDoctor()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

