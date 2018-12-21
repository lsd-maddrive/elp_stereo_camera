#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo

import os.path
import cv2
from cv_bridge import CvBridge, CvBridgeError

import time

IMAGE_MESSAGE_TOPIC = 'image_color'
CAMERA_MESSAGE_TOPIC = 'cam_info'

rospy.init_node('video_2_camera_stream')

device_path = rospy.get_param('~dev', '/dev/video0')
left_topic_name = rospy.get_param('~left_topic', 'left')
right_topic_name = rospy.get_param('~right_topic', 'right')

queue_size = 10

left_img_pub = rospy.Publisher(left_topic_name + '/' + IMAGE_MESSAGE_TOPIC, Image, queue_size=queue_size)
left_cam_pub = rospy.Publisher(left_topic_name + '/' + CAMERA_MESSAGE_TOPIC, CameraInfo, queue_size=queue_size)
right_img_pub = rospy.Publisher(right_topic_name + '/' + IMAGE_MESSAGE_TOPIC, Image, queue_size=queue_size)
right_cam_pub = rospy.Publisher(right_topic_name + '/' + CAMERA_MESSAGE_TOPIC, CameraInfo, queue_size=queue_size)

rate = rospy.Rate(30)

capture = cv2.VideoCapture(device_path)

if not capture.isOpened():
    rospy.logerr('Failed to open device')
    exit(1)

req_width = 320
req_height = 240

capture.set(cv2.CAP_PROP_FRAME_WIDTH, req_width * 2);
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, req_height);

fr_width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
fr_height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
fr_width_2 = int(fr_width/2)

print(fr_width, fr_height)

bridge = CvBridge()

cam_info        = CameraInfo()
cam_info.height = fr_height
cam_info.width  = fr_width_2

def main():
    while not rospy.is_shutdown():
        meta, frame = capture.read()

        frame_left = frame[:, :fr_width_2, :]
        frame_right = frame[:, fr_width_2:]

        # frame_gaus = cv2.GaussianBlur(frame, (3, 3), 0)
        # frame_gray = cv2.cvtColor(frame_gaus, cv2.COLOR_BGR2GRAY)

        # I want to publish the Canny Edge Image and the original Image

        cam_info.header.stamp = rospy.Time.from_sec(time.time())

        left_cam_pub.publish(cam_info)
        right_cam_pub.publish(cam_info)
        left_img_pub.publish(bridge.cv2_to_imgmsg(frame_left, "bgr8"))
        right_img_pub.publish(bridge.cv2_to_imgmsg(frame_right, "bgr8"))

        # cv2.imshow('l', frame_left)
        # cv2.imshow('r', frame_right)
        # cv2.waitKey(1)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('Exception caught')