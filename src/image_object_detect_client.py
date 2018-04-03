#!/usr/bin/env python

# Load an image from disk and call deep_object_detection service

import cv2
import numpy as np
import rospy
import sys
from sensor_msgs.msg import Image
from deep_object_detection.srv import *
from ros_file_crawler.srv import *
import os
import copy
import json
from os.path import expanduser
import argparse

def parse_arguments():

    parser = argparse.ArgumentParser()

    home = expanduser("~")

    default_dest_path= os.path.join(home,"image_object_detect_results")

    parser.add_argument("source_path", help="source path for images")

    parser.add_argument("confidence_threshold",help="confidence threshold for object detection (0-1.0 range)", nargs='?', type=float, default=0.8)

    parser.add_argument("destination_path", help="destination path for results", nargs='?', default=default_dest_path)

    parser.add_argument("file_extensions", help="file extensions to be read", nargs='?', default="jpg,png")

    parser.add_argument("excluded_words",help="words that need to be excluded in filenames", nargs='?', default="")



    return parser.parse_args()



if __name__ == '__main__':


    rospy.init_node('image_object_detect_client')

    ''' Get the arguments. The first arguments are
    <source_path> <destination_path> <file_extensions> <excluded_words>.
    Excluded words are the labels that should not be included
    in the filename such as depth, label, etc.

    if len(sys.argv) < 5:
        print "Insufficent number of arguments. Quitting..."
        rospy.signal_shutdown(" Destination Path is not provided")
        sys.exit(-1)
    '''
    '''******* Check the file_crawl and detect_object services *******
    '''
    try:
        rospy.wait_for_service('/ros_file_crawler/crawl_for_files', 5)
    except:
        rospy.logwarn("ros_file_crawler service wait timeout")
        rospy.signal_shutdown("Service wait timeout")
        sys.exit(-2)

    try:
        rospy.wait_for_service('/deep_object_detection/detect_objects', 5)
    except rospy.ROSException:
        rospy.logwarn("deep_object_detection service wait timeout")
        rospy.signal_shutdown("Service wait timeout")
        sys.exit(-2)

    '''*******************************************************************
    '''

    '''**********  Parse arguments ********************
    '''

    args = parse_arguments()

    source_path = args.source_path #sys.argv[1]

    dest_path = args.destination_path #sys.argv[2]

    confidence_threshold = args.confidence_threshold

    if os.path.exists(source_path) == False:
        rospy.logwarn("source path {} does not exist".format(source_path))
        rospy.signal_shutdown("source path does not exist")
        sys.exit(-3)


    file_extension_string = args.file_extensions#sys.argv[3]
    file_extensions = file_extension_string.split(',')


    if os.path.exists(dest_path) == False:
        os.makedirs(dest_path)
    else:
        files = [f for f in os.listdir(dest_path)]
        for f in files:
            if f.lower().endswith(tuple(file_extensions)):
                os.remove(os.path.join(dest_path,f))

    excluded_words_string = args.excluded_words#sys.argv[4]
    excluded_words = excluded_words_string.split(',')
    '''*************************************************
    '''


    try:

        file_crawler = rospy.ServiceProxy('/ros_file_crawler/crawl_for_files',FileList)
        resp_file = file_crawler(source_path,file_extensions,excluded_words)

        images = []
        cv_images = []

        ''' The paths and filenames are returned as json encoded string
        '''
        paths = json.loads(resp_file.paths)
        names = json.loads(resp_file.filenames)

        print names

        for file_extension in file_extensions:

            if file_extension in names.keys():

                paths_for_extension = paths[file_extension]
                filenames_for_extension = names[file_extension]

                for afile in paths_for_extension:

                    image = cv2.imread(afile)
                    cv_images.append(image)
                    #cv2.imshow("im", image)
                    #cv2.waitKey(5)
                    #pub = rospy.Publisher('image', Image, queue_size=1)
                    if image.shape[0] > 64 and image.shape[1] > 64:
                    	msg = Image()
                    	msg.header.stamp = rospy.Time.now()
                    	msg.encoding = 'bgr8'
                    	msg.height = image.shape[0]
                    	msg.width = image.shape[1]
                    	msg.step = image.shape[1] * 3
                    	msg.data = image.tostring()
                    	images.append(msg)

                detect_objects = rospy.ServiceProxy('/deep_object_detection/detect_objects', DetectObjects)
                resp = detect_objects(images," ",confidence_threshold)
                #print resp.objects
                for i in range(len(images)):
                    image = cv_images[i]
                    for obj in resp.objects:
                        if int(obj.imageID) == i:
                            cv2.rectangle(image,(obj.x,obj.y),(obj.x+obj.width, obj.y+obj.height),color=(255,255,0),thickness=2)
                            text = obj.label + "_" +"%.2f"%obj.confidence
                            cv2.putText(image,text,(obj.x+obj.width/4,obj.y+obj.height/2),cv2.FONT_HERSHEY_SIMPLEX,0.5,color=(255,0,0),thickness=1)
                    filename = "processed_image_"+str(i)+".jpg"
                    fullpath = os.path.join(dest_path,filename)
                    cv2.imwrite(fullpath,image)

    except Exception as e:
        rospy.logwarn("Service call failed: {}".format(e))
