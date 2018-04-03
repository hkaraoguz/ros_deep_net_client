#!/usr/bin/env python

# Read a video from url and call deep_object_detection service
import cv2
import numpy as np
import rospy
import sys
from sensor_msgs.msg import Image
from deep_object_detection.srv import *
import os
import copy
import json
import urllib
import argparse
from rospy_message_converter import json_message_converter


def parse_arguments():
    '''
    Function for parsing positional arguments
    '''

    parser = argparse.ArgumentParser()

    parser.add_argument("source_url", help="source url for video feed")

    parser.add_argument("confidence_threshold",help="the threshold for object detection results (0-1.0 range)", nargs='?', type=float, default=0.8)

    return parser.parse_args()


def process_image(image, confidence_threshold):
    '''
    Function that calls the object detection service and outputs the result
    '''

    small_image= cv2.resize(image,(0,0),fx=0.5,fy=0.5, interpolation=cv2.INTER_LINEAR)

    detect_objects = rospy.ServiceProxy('/deep_object_detection/detect_objects', DetectObjects)

    images = []
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.encoding = 'bgr8'
    msg.height = small_image.shape[0]
    msg.width = small_image.shape[1]
    msg.step = small_image.shape[1] * 3
    msg.data = small_image.tostring()
    images.append(msg)
    resp = detect_objects(images,"",confidence_threshold)

    for obj in resp.objects:
        json_str = json_message_converter.convert_ros_message_to_json(obj)
        print json_str
        cv2.rectangle(small_image,(obj.x,obj.y),(obj.x+obj.width, obj.y+obj.height),color=(255,255,0),thickness=2)
        text = obj.label + " " +"%.2f"%obj.confidence
        cv2.putText(small_image,text,(obj.x+obj.width/4,obj.y+obj.height/2),cv2.FONT_HERSHEY_SIMPLEX,0.7,color=(255,0,255),thickness=2)

    return small_image

if __name__ == '__main__':

    rospy.init_node('publisher')
    try:
        rospy.wait_for_service('/deep_object_detection/detect_objects', 5)
    except rospy.ROSException:
        rospy.logwarn("Service wait timeout")
        rospy.signal_shutdown("Service wait timeout")
        sys.exit(-2)

    args = parse_arguments()
    #sourpath = sys.argv[1]
    #print sourpath
    #cap = cv2.VideoCapture(str(sourpath))
    #cap.set(3,320)
    #cap.set(4,240)
    stream=urllib.urlopen(args.source_url)
    bytes=''
    cv2.namedWindow('video')
    while not rospy.is_shutdown():
        bytes+=stream.read(1024)
        a = bytes.find('\xff\xd8')
        b = bytes.find('\xff\xd9')
        if a!=-1 and b!=-1:
            jpg = bytes[a:b+2]
            bytes= bytes[b+2:]
            frame = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
            image = process_image(frame,args.confidence_threshold)

            cv2.imshow('video', image)

            # convert it to byte
            res = cv2.waitKey(2)&0xff
            #if res == "q" quit
            if res == 0x71:
                print "User requests shutdown"
                rospy.signal_shutdown("User requests shutdown")
                cv2.destroyAllWindows()
                sys.exit(0)
