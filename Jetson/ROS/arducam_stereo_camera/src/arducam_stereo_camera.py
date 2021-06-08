#!/usr/bin/env python

import cv2
import numpy as np
from datetime import datetime
import array
import fcntl
import os
import math
import argparse
from utils import ArducamUtils
import arducam_isp

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager

class HelpClass(object):
    def __init__(self, arducam_utils):
        self.arducam_utils = arducam_utils
        self.line_length = 1456
        self.pixel_clock = 96000000.0
        self.time_per_line = self.line_length / self.pixel_clock *1e9
        self.reconfigure()


    def reconfigure(self):
        self.line_length = self.arducam_utils.read_sensor(
            0x380C) << 8 | self.arducam_utils.read_sensor(0x380D)
        self.time_per_line = self.line_length / self.pixel_clock *1e9
        self.setFramerate(40)
        

    def setFramerate(self, val):
        val = max(val, 2)
        val = min(val, 120)
        self.vts = int(self.pixel_clock / (self.line_length * val))
        print("vts: {}".format(self.vts))
        ret = self.arducam_utils.write_sensor(0x380E, (self.vts & 0xFF00) >> 8)
        ret = self.arducam_utils.write_sensor(0x380F, (self.vts & 0x00FF) >> 0)

    def setCtrl(self, name, val):
        # print("name: {}, val:{}".format(name, val))
        if name == "setExposureTime":
            coarse_time = int(math.floor(val*1000 / self.time_per_line))
            coarse_time = min(coarse_time, self.vts - 12)
            ret = self.arducam_utils.write_sensor(0x3500, (coarse_time & 0xF000) >> 12)
            ret = self.arducam_utils.write_sensor(0x3501, (coarse_time & 0x0FF0) >> 4)
            ret = self.arducam_utils.write_sensor(0x3502, (coarse_time & 0x000f) << 4)
        elif name == "setAnalogueGain":
            gain = int(math.floor(val / 100))
            ret = self.arducam_utils.write_sensor(0x3509, (gain & 0x0F) << 4 | int(math.floor((val / 100.0) % 1 * 16)))
        return 0


def resize(frame, dst_width):
    width = frame.shape[1]
    height = frame.shape[0]
    scale = dst_width * 1.0 / width
    return cv2.resize(frame, (int(scale * width), int(scale * height)))

def run(cap, arducam_utils):
    path = os.path.dirname(os.path.abspath(__file__))
    info = arducam_isp.CameraInfo()
    isp = arducam_isp.ISP(path + "/ov9281.json", info)
    cap.grab()
    helper = HelpClass(arducam_utils)
    isp.registerCallback(helper.setCtrl)

    left_info_mgr = CameraInfoManager(cname='left_camera', namespace='left')
    right_info_mgr = CameraInfoManager(cname='right_camera', namespace='right')
    
    left_info_mgr.setURL(left_info_url)
    right_info_mgr.setURL(right_info_url)
    
    left_info_mgr.loadCameraInfo()
    right_info_mgr.loadCameraInfo()

    left_pub = rospy.Publisher('left/image_raw', Image, queue_size=10)
    right_pub = rospy.Publisher('right/image_raw', Image, queue_size=10)

    left_info_pub = rospy.Publisher('left/camera_info', CameraInfo, queue_size=10)
    right_info_pub = rospy.Publisher('right/camera_info', CameraInfo, queue_size=10)

    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if arducam_utils.convert2rgb == 0:
            w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            frame = frame.reshape(int(h), int(w))

        left = frame[:, 0:int(w//2)]
        process_frame = left.astype(np.uint16)
        process_frame <<= 8
        isp.queueBuffer(process_frame)

        frame = arducam_utils.convert(frame)

        encoding = "bgr8" if len(frame.shape) == 3 and frame.shape[2] >= 3 else "mono8"

        width = frame.shape[1]
        height = frame.shape[0]

        left_img = frame[:, :width//2]
        right_img = frame[:, width//2:]

        left_img_msg = bridge.cv2_to_imgmsg(left_img, encoding)
        left_img_msg.header.frame_id = frame_id

        right_img_msg = bridge.cv2_to_imgmsg(right_img, encoding)
        right_img_msg.header.frame_id = frame_id

        capture_time = rospy.Time.now()

        left_img_msg.header.stamp = capture_time
        right_img_msg.header.stamp = capture_time

        left_pub.publish(left_img_msg)
        right_pub.publish(right_img_msg)

        info_left = left_info_mgr.getCameraInfo()
        info_right = right_info_mgr.getCameraInfo()

        info_left.header.stamp = capture_time
        info_right.header.stamp = capture_time

        info_left.header.frame_id = frame_id
        info_right.header.frame_id = frame_id
        
        left_info_pub.publish(info_left)
        right_info_pub.publish(info_right)
        
    pass


def fourcc(a, b, c, d):
    return ord(a) | (ord(b) << 8) | (ord(c) << 16) | (ord(d) << 24)

def pixelformat(string):
    if len(string) != 3 and len(string) != 4:
        msg = "{} is not a pixel format".format(string)
        raise argparse.ArgumentTypeError(msg)
    if len(string) == 3:
        return fourcc(string[0], string[1], string[2], ' ')
    else:
        return fourcc(string[0], string[1], string[2], string[3])

def show_info(arducam_utils):
    _, firmware_version = arducam_utils.read_dev(ArducamUtils.FIRMWARE_VERSION_REG)
    _, sensor_id = arducam_utils.read_dev(ArducamUtils.FIRMWARE_SENSOR_ID_REG)
    _, serial_number = arducam_utils.read_dev(ArducamUtils.SERIAL_NUMBER_REG)
    print("Firmware Version: {}".format(firmware_version))
    print("Sensor ID: 0x{:04X}".format(sensor_id))
    print("Serial Number: 0x{:08X}".format(serial_number))

if __name__ == "__main__":
    rospy.init_node("arducam_stereo_camera")

    try:
        device = rospy.get_param("~device")
    except:
        device = 0

    # open camera
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

    try:
        # set pixel format
        pixfmt = rospy.get_param("~pixelformat")
        if not cap.set(cv2.CAP_PROP_FOURCC, pixelformat(pixfmt)):
            print("Failed to set pixel format.")
    except:
        pass

    arducam_utils = ArducamUtils(device)

    show_info(arducam_utils)
    # turn off RGB conversion
    if arducam_utils.convert2rgb == 0:
        cap.set(cv2.CAP_PROP_CONVERT_RGB, arducam_utils.convert2rgb)
    
    try:
        width = rospy.get_param("~width")
        # set width
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)

        height = rospy.get_param("~height")
        # set height
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    except:
        pass

    try:
        frame_id = rospy.get_param("~frame_id")
    except:
        frame_id = "cam0"

    try:
        left_info_url = rospy.get_param("~left/camera_info_url")
        right_info_url = rospy.get_param("~right/camera_info_url")
    except:
        left_info_url = None
        right_info_url = None

    run(cap, arducam_utils)

    # release camera
    cap.release()
