#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Python node to monitor Jetson usage"""

import rospy
from std_msgs.msg import Float32
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from jetson_botdoc.cfg import docConfig
from jetson_botdoc.msg import healthReportData

class JetsonDoctor(object):
    """Main monitor class."""

    def __init__(self):
        rate = rospy.get_param('~rate', 1.0)
        self.enable = True
        self.server = DynamicReconfigureServer(docConfig, self.reconfigure_cb)
        self.pub = rospy.Publisher('gpu_usage', healthReportData, queue_size=10)

        self.enable = rospy.get_param('~enable', True)

        if self.enable:
            self.start()
        else:
            self.stop()

        rospy.Timer(rospy.Duration(1.0 / rate), self.timed_cb)

    def start(self):
        """Turn on publishers"""
        self.pub = rospy.Publisher('gpu_usage', healthReportData, queue_size=10)

    def stop(self):
        """Turn off publishers"""
        self.pub.unregister()

    def timed_cb(self, _event):
        """Call at a specified interval"""
        if not self.enable:
            return

        msg = healthReportData()
        msg.gpu_usage.data = self.get_gpu_usage()
        msg.usbfs_memory_mb.data = self.get_usb_memory()
        msg.data_header.stamp = rospy.get_rostime()
        self.pub.publish(msg)

    def reconfigure_cb(self, config, dummy):
        """Dynamic reconfigure server."""
        if self.enable != config["enable"]:
            if config["enable"]:
                self.start()
            else:
                self.stop()
        self.enable = config["enable"]
        return config

    def get_gpu_usage(self):
        """Fetch the percentage of GPU being used"""
        """-- source: https://github.com/jetsonhacks/gpuGraphTX"""

        gpuLoadFile="/sys/devices/gpu.0/load"
        with open(gpuLoadFile, 'r') as gpuFile:
            fileData = gpuFile.read()
        return float(fileData)/10.0

    def get_usb_memory(self):
        """Fetch the memory available to USB buffer"""
        usbMemLoadFile="/sys/module/usbcore/parameters/usbfs_memory_mb"
        with open(usbMemLoadFile, 'r') as usbMemFile:
            fileData = usbMemFile.read()
        return float(fileData)

if __name__ == '__main__':
    rospy.init_node('jetson_botdoc')
    try:
        JetsonDoctor()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

