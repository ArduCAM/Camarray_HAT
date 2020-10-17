#!/usr/bin/env python

import cv2
import numpy as np
from datetime import datetime
import array
import fcntl
import os
import argparse
from utils import ArducamUtils

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager


def resize(frame, dst_width):
    width = frame.shape[1]
    height = frame.shape[0]
    scale = dst_width * 1.0 / width
    return cv2.resize(frame, (int(scale * width), int(scale * height)))

def run(cap, arducam_utils):

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
