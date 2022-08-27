#!/usr/bin/env python3

import rospy
import cv2
import numpy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback_point_cloud(msg):
    None
    print("Point cloud received: " + str(msg.width) + "x"+ str(msg.height))
    
    #print(type(msg.data))
    arr = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    rgb = arr['rgb'].copy().view(numpy.uint32)
    r,g,b = ((rgb >> 16) & 255), ((rgb >> 8) & 255), (rgb & 255)
    img = cv2.merge((numpy.asarray(b, dtype='uint8'), numpy.asarray(g, dtype='uint8'), numpy.asarray(r, dtype='uint8')))
    print(img[240,320])
    print(arr[240,320])
    cv2.imshow("Image",img)
    cv2.waitKey(10)
    
    #r = numpy.right_shift(rgb,16)
    #r,g,b = ((numpy.right_shift(numpy.uint64(rgb)) >> 16) & 255), ((rgb >> 8) & 255), (rgb & 255) 
    #r=(numpy.right_shift(numpy.uint64(rgb)) >> 16) & 255
    #print(type(img[240,320]))
    
def callback_image(msg):
    print("Image recieved")
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
    cv2.imshow("Image",cv_image)
    cv2.waitKey(10)
    
def main():
    print("INITIALIZING POINT CLOUD VIEWER")
    rospy.init_node('point_cloud_viewer')
    rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points',PointCloud2,callback_point_cloud)
    #rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_rect_color',Image,callback_image)
    rospy.spin()
    
if __name__ == '__main__':
   main()
    
