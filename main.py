#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import tf
import math
from tf import transformations
import numpy as np
from scipy import stats

from cv_bridge import CvBridge, CvBridgeError



class TakePhoto:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False
        # Connect image topic
        img_topic = "/camera/depth/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second to connection
        rospy.sleep(1)

    def callback(self,msg_depth): # TODO still too noisy!
        try:
            print("Received an image!")

          # The depth image is a single-channel float32 image
          # the values is the distance in mm in z axis
            cv_image = self.bridge.imgmsg_to_cv2(msg_depth, "32FC1")
          # Convert the depth image to a Numpy array since most cv2 functions
          # require Numpy arrays.
            cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
            cv_image_array_int = cv_image_array.astype(int)
            modeOfInts = stats.mode(cv_image_array_int, axis=None)
            modeOfInts = float(modeOfInts.mode[0])


            # print(np.shape(cv_image_array))
            # print(np.shape(elementsWithinThreshold))

            #elementsWithinThreshold = elementsWithinThreshold[elementsWithinThreshold<modeOfInts+threshold]
            #print(elementsWithinThreshold)
          # Normalize the depth image to fall between 0 (black) and 1 (white)
          # http://docs.ros.org/electric/api/rosbag_video/html/bag__to__video_8cpp_source.html lines 95-125
            cv_image = cv2.imread('stop.jpg',0)
            cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))




            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 255, cv2.NORM_MINMAX)
            sorted = np.sort(cv_image_norm,axis = None)
            print(sorted)
            medianOfValues = np.median(sorted, axis = None)
            cvImageSubtracted = cv_image_array - medianOfValues
            cvImageSubtractedAbs = np.absolute(cvImageSubtracted)
            threshold = medianOfValues/10
            print(medianOfValues)
            elementsWithinThreshold = np.nonzero(cvImageSubtractedAbs<threshold)
            xOfObjectPixels = []
            yOfObjectPixels = []
            for i in range(len(elementsWithinThreshold[0])):
                firstIndice = elementsWithinThreshold[0][i]
                secondIndice = elementsWithinThreshold[1][i]
                cv_image_norm[firstIndice][secondIndice] = 255
                xOfObjectPixels.append(firstIndice)
                yOfObjectPixels.append(secondIndice)

            meanOfX = int(np.amin(xOfObjectPixels))
            meanOfY = int(np.amin(yOfObjectPixels))
            dist = cv_image_array[meanOfX,meanOfY]
            print(cv_image_array[meanOfX,meanOfY])  #distance
            objectX = frameLeftTop.x + robot_x + dist *math.cos(robot_theta)
            objectY = frameLeftTop.y + robot_y + dist *math.sin(robot_theta)
            #point = cv_image_norm[100, 100]
            #print(point)
          # Resize to the desired size
            #cv_image_resized = cv2.resize(cv_image_norm, (100,50), interpolation = cv2.INTER_CUBIC)
            #self.depthimg = cv_image_resized
            cv2.imshow("Image from my node", cv_image_norm)
            #print(cv_image_norm)
            try:
                cv2.imwrite("./img11.jpg", cv_image_norm)
            except:
                print("error")
            print("line before write")
            cv2.waitKey(1)
            self.image_received = True
        except CvBridgeError as e:
            print(e)

    def take_picture(self, img_title):
        if self.image_received:
            # Save an image
            print("im working")
            #cv2.imshow('ImageWindow', img_title)
            #cv2.imwrite(img_title, self.image)
            #cv2.waitKey()
            return True
        else:
            return False



if __name__ == '__main__':

    # Initialize
    rospy.init_node('take_photo', anonymous=False)
    camera = TakePhoto()

    # Take a photo

    # Use '_image_title' parameter from command line
    # Default value is 'photo.jpg'
    img_title = "~/catkin_ws/img01.jpg"#rospy.get_param('~image_title', 'photo16U.jpg')

    if camera.take_picture(img_title):
        rospy.loginfo("Saved image " + img_title)
    else:
        rospy.loginfo("No images received")

    print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")
