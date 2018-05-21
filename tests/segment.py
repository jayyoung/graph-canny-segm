import roslib
import rospy
from sensor_msgs.msg import *
from canny_seg_wrapper.srv import *
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import random


if __name__ == '__main__':
    rospy.init_node('canny_seg_wrapper_server_test', anonymous = False)

    rospy.loginfo("waiting for service")
    rospy.wait_for_service("/canny_seg_wrapper/segment")

    rospy.loginfo("setting up proxy")
    srv = rospy.ServiceProxy("/canny_seg_wrapper/segment",CannySegWrapper)

    rospy.loginfo("loading test images")
    rgb = cv2.imread('./rgb.png')
    depth = cv2.imread('./depth.png', cv2.IMREAD_UNCHANGED) # this is super important

    bridge = CvBridge()

    rgb_msg = bridge.cv2_to_imgmsg(rgb)

    depth_msg = bridge.cv2_to_imgmsg(depth)

    rospy.loginfo("segmenting scene")

    response = srv(rgb_msg,depth_msg)

    rospy.loginfo("displaying results")

    rospy.loginfo("segments: " + str(len(response.output)))

    bridge = CvBridge()
    ims = []
    rgims = []
    for image in response.output[1:]:
        col = (random.randint(0,255),random.randint(0,255),random.randint(0,255))
        im = bridge.imgmsg_to_cv2(image,desired_encoding="bgr8")
        rgims.append(im.copy())

        h = im.shape[0]
        w = im.shape[1]
        for y in range(0, h):
            for x in range(0, w):
                if np.any(im[y, x] != 0):
                    im[y,x] = col

        ims.append(im)

    full_scene = sum(ims)
    rgb_scene = sum(rgims)
    cv2.imwrite("segments.png",full_scene)
    cv2.imwrite("segments_rgb.png",rgb_scene)
    #rospy.sleep(15)
