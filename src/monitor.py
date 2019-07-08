#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Python node to monitor Jetson usage"""

import rospy

from dynamic_reconfigure.server import (
    Server as DynamicReconfigureServer,
)

from jetson_botdoc.cfg import docConfig
from jetson_botdoc.msg import healthReportData

import pdb
import os


class JetsonDoctor(object):
    """Main monitor class."""

    def __init__(self):
        rate = rospy.get_param("~rate", 1.0)
        self.enable = True
        self.server = DynamicReconfigureServer(
            docConfig, self.reconfigure_cb
        )
        self.pub = rospy.Publisher(
            "xavier_health", healthReportData, queue_size=10
        )

        self.enable = rospy.get_param("~enable", True)

        if self.enable:
            self.start()
        else:
            self.stop()

        rospy.Timer(
            rospy.Duration(1.0 / rate), self.timed_cb
        )

    def start(self):
        """Turn on publishers"""
        self.pub = rospy.Publisher(
            "xavier_health", healthReportData, queue_size=10
        )

    def stop(self):
        """Turn off publishers"""
        self.pub.unregister()

    def timed_cb(self, _event):
        """Call at a specified interval"""
        if not self.enable:
            return

        msg = healthReportData()

        # How much of the GPU's compute is being used
        msg.gpu_usage.data = self.get_gpu_usage()
        if msg.gpu_usage.data > 80.0:
            rospy.logwarn(
                "[GPU] More than 80% load on GPU.."
            )

        # USB3.0 memory bus size (static)
        msg.usbfs_memory_mb.data = self.get_usb_memory()
        if msg.usbfs_memory_mb.data < 1024:
            rospy.logwarn(
                "[USB] usbfs_memory_mb (= %.2f) is lower than recommended",
                msg.usbfs_memory_mb.data,
            )
            rospy.logwarn(
                "[USB] Check github.com/ShreyasSkandanS/nvidia_xavier_utils"
            )

        # RAM stats (in GB)
        total_memory, used_memory, free_memory = (
            self.get_memory_status()
        )
        msg.used_memory.data = used_memory
        msg.total_memory.data = total_memory
        msg.free_memory.data = free_memory
        if msg.free_memory.data < 2.0:
            rospy.logwarn(
                "[RAM] Less than 2GB of RAM remaining!"
            )

        # Internal temperatures (in Celsius)
        cpuT, gpuT, tegraT = self.get_internal_temp()
        msg.cpu_temp.data = cpuT
        msg.gpu_temp.data = gpuT
        msg.tboard_temp.data = tegraT
        if msg.cpu_temp.data > 80:
            rospy.logwarn(
                "[THERMALS] CPU running close to max temp (= %.2f)",
                msg.cpu_temp.data,
            )
        if msg.gpu_temp.data > 80:
            rospy.logwarn(
                "[THERMALS] GPU running close to max temp (= %.2f)",
                msg.gpu_temp.data,
            )

        # Which CPUs are currently active
        cpu_status_list = self.get_cpu_status()
        msg.cpu_0.data = bool(int(cpu_status_list[0]))
        msg.cpu_1.data = bool(int(cpu_status_list[1]))
        msg.cpu_2.data = bool(int(cpu_status_list[2]))
        msg.cpu_3.data = bool(int(cpu_status_list[3]))
        msg.cpu_4.data = bool(int(cpu_status_list[4]))
        msg.cpu_5.data = bool(int(cpu_status_list[5]))

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
        """Fetch the percentage of GPU being used
           ----
           Reference: https://github.com/jetsonhacks/gpuGraphTX"""

        gpuLoadFile = "/sys/devices/gpu.0/load"
        with open(gpuLoadFile, "r") as gpuFile:
            fileData = gpuFile.read()
        return float(fileData) / 10.0

    def get_usb_memory(self):
        """Fetch the memory available to USB buffer"""
        usbMemLoadFile = (
            "/sys/module/usbcore/parameters/usbfs_memory_mb"
        )
        with open(usbMemLoadFile, "r") as usbMemFile:
            fileData = usbMemFile.read()
        return float(fileData)

    def get_cpu_status(self):
        """Fetch status flags for each of the 6 cpus"""
        CPU_COUNT = 6
        status_list = []
        for cpu_id in range(0, CPU_COUNT):
            cpuStatusFile = (
                "/sys/devices/system/cpu/cpu"
                + str(cpu_id)
                + "/online"
            )
            with open(cpuStatusFile, "r") as cpuStatus:
                fileData = cpuStatus.read()
                status_list.append(fileData[0])
        return status_list

    def get_memory_status(self):
        """Fetch basic RAM usage stats"""
        total_m, used_m, free_m = map(
            int,
            os.popen("free -t -m")
            .readlines()[-1]
            .split()[1:],
        )
        return (
            total_m / 1000.0,
            used_m / 1000.0,
            free_m / 1000.0,
        )

    def get_internal_temp(self):
        """Get internal component temperatures
           to find out which thermal zone corresponds to what internal component,
           you can check this at /sys/devices/virtual/thermal/thermal_zoneX/type
           ----
           Reference: jetson_agx_xavier_thermal_design_guide_v1.0.pdf"""
        tempFileBase = "/sys/devices/virtual/thermal/"
        cpuTempFile = tempFileBase + "thermal_zone0/temp"
        gpuTempFile = tempFileBase + "thermal_zone1/temp"
        boardTempFile = tempFileBase + "thermal_zone6/temp"
        with open(cpuTempFile, "r") as cpuTemp:
            cpuTempData = int(cpuTemp.read())
        with open(gpuTempFile, "r") as gpuTemp:
            gpuTempData = int(gpuTemp.read())
        with open(boardTempFile, "r") as boardTemp:
            boardTempData = int(boardTemp.read())
        return (
            cpuTempData / 1000.0,
            gpuTempData / 1000.0,
            boardTempData / 1000.0,
        )


if __name__ == "__main__":
    rospy.init_node("jetson_botdoc")
    try:
        JetsonDoctor()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
