#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import message_filters
import cv2
import tf
import math
from tf import transformations
import numpy as np
from scipy import stats
from std_msgs.msg import Int32, Float32
from cv_bridge import CvBridge, CvBridgeError
from imageClassifier import ImageClassifier
from sensor_msgs.msg import LaserScan, CameraInfo
from nav_msgs.msg import OccupancyGrid
import image_geometry
import tf2_ros


class TakePhoto:
    def __init__(self):
        self.imageClassifier = ImageClassifier("./yolov3.weights",
        "./yolov3.cfg",
        "./yolov3.txt",0.00392)

        self.bridge = CvBridge()
        rgbSub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
        depthSub = message_filters.Subscriber("/camera/depth/image_raw", Image)
        # To synchronize rgb and depth images
        self.listener = tf.TransformListener()
        ts = message_filters.ApproximateTimeSynchronizer([rgbSub, depthSub], 50, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)
        #constants
        self.cx = 319.5
        self.cy = 239.5
        self.fy = 320.5
        self.fx = 320.5

        #To save images and print
        self.debugMode = True
        #To follow num of images
        self.imageCounter = 0

        self.objectCoordinates = {}

    def getRealWorldCoordinates(self, point, depthImageArray, robotX, robotY, robotTheta) :

        forwardX = depthImageArray[point]
        print("depth value is ", forwardX)
        forwardY = -1 * (forwardX * (point[0] - self.cy) / self.fy)
        print('point 1', point[1])
        print('point 0', point[0])
        print('robotX', robotX)
        forwardZ = forwardX * (point[1] - self.cx) / self.fx
        #xOfObject = robotX + depth * math.cos(robotTheta) + xRelativeToRobot * math.sin(robotTheta)
        #zOfObject = -1 * (robotY + depth * math.sin(robotTheta) + xRelativeToRobot * math.cos(robotTheta))
        xCoord = robotX + forwardX*math.cos(robotTheta) + forwardZ * math.sin(robotTheta)
        yCoord = robotY + forwardX*math.sin(robotTheta) - forwardZ * math.cos(robotTheta)
        #return [forwardX, forwardY, forwardZ]
        #xCoord = robotX + forwardX*math.cos(robotTheta)
        return [xCoord, yCoord, forwardY]

    def processRgb(self, msgRgb, rawImage):
        try:
            #print("Received an image!")
            imageArray = np.array(rawImage, dtype = np.dtype('uint8'))

            labelArray, rectangleInfoArray = self.imageClassifier.classify(imageArray)
            pointListArray = []
            for i in range(len(labelArray)):
                frameLeftTopX = rectangleInfoArray[i][0]
                frameLeftTopY = rectangleInfoArray[i][1]
                frameWidth = rectangleInfoArray[i][2]
                frameHeight = rectangleInfoArray[i][3]

                label = labelArray[i]

                croppedImage = np.copy(imageArray[frameLeftTopY:frameLeftTopY + frameHeight, frameLeftTopX : frameLeftTopX + frameWidth])
                oldpointList = self.contour(croppedImage)
                pointList = [[], []]
                for i in range(len(oldpointList[0])):
                    pointList[0].append(oldpointList[0][i])
                    pointList[1].append(oldpointList[1][i])
                pointListArray.append(pointList)

                if self.debugMode:
                    try:
                        self.imageCounter += 1
                        cv2.imwrite(str(self.imageCounter) + "rgbCroppedImage_" + str(i) + ".jpg", croppedImage)
                    except:
                        print("rgb cropped image error")


            return labelArray, rectangleInfoArray, pointListArray
        except CvBridgeError as e:
            print(e)
    def contour(self, image):
        imgGrayscale = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(imgGrayscale,100,200)
        im2, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE,  cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            cv2.drawContours(image, contour, -1, (0,0,0), -1)
        pointList = np.where(image==(0,0,0))
        pointArray = []
        for i in range(len(pointList[0])):
            pointArray.append((pointList[1][i], pointList[0][i]))
        #cv2.fillPoly(image, pointArray, (0,0,0))
        if self.debugMode:
            cv2.imwrite("./circled.jpg",image)
        return pointList

    def processDepth(self, msgDepth,rects, robotX, robotY, robotTheta, pointList, depthImage):
        try:
            frameLeftTopX = rects[0]
            frameLeftTopY = rects[1]
            frameHeight = rects[3]
            frameWidth = rects[2]
            depthImageArray = np.array(depthImage, dtype = np.dtype('f8'))
            depthImageNormalized = np.empty_like(depthImageArray)
            depthImageNormalized = cv2.normalize(depthImageArray, depthImageNormalized, 0, 255, cv2.NORM_MINMAX)
            depthImageCropped = np.copy(depthImageArray[frameLeftTopY:frameLeftTopY + frameHeight, frameLeftTopX : frameLeftTopX + frameWidth])

            depthImageCroppedNormalized = np.empty_like(depthImageCropped)
            depthImageCroppedNormalized = cv2.normalize(depthImageCropped, depthImageCroppedNormalized, 0, 255, cv2.NORM_MINMAX)
            intensityArray = []
            for i in range(len(pointList[0])):
                intensity = depthImageCroppedNormalized[pointList[0][i], pointList[1][i]]
                if intensity<256 and intensity>0:
                    intensityArray.append(intensity)

            medianOfValues = np.median(intensityArray)
            cv2.imwrite('rectimg.jpg', depthImageCroppedNormalized)

            cvImageSubtracted = depthImageCroppedNormalized - medianOfValues
            cvImageSubtractedAbs = np.absolute(cvImageSubtracted)
            threshold = medianOfValues + 20
            elementsWithinThreshold = np.nonzero(cvImageSubtractedAbs<threshold)
            xOfObjectPixels = []
            yOfObjectPixels = []
            # for i in range(len(elementsWithinThreshold[0])):
            #     firstIndice = elementsWithinThreshold[0][i]
            #     secondIndice = elementsWithinThreshold[1][i]
            #     #depthImageNormalized[firstIndice + frameLeftTopY][secondIndice + frameLeftTopX] = 255
            #     yOfObjectPixels.append(firstIndice)
            #     xOfObjectPixels.append(secondIndice)

            if len(elementsWithinThreshold[0]) <= 0:
                return [-1, -1, -1]
            meanOfX = int(np.mean(elementsWithinThreshold[1]))
            meanOfY = int(np.mean(elementsWithinThreshold[0]))
            indices = [i for i, x in enumerate(elementsWithinThreshold[1]) if x == meanOfX]
            closest = 9999
            closestYIndex = -1
            for i in indices:
                yDifference = abs(meanOfY - elementsWithinThreshold[0][i])
                if(yDifference < closest):
                    closest = yDifference
                    closestYIndex = elementsWithinThreshold[0][i]
            #print(midPoint)
            midPoint = (closestYIndex + frameLeftTopY, meanOfX + frameLeftTopX)

            objectCoordinates = self.getRealWorldCoordinates(midPoint, depthImageArray, robotX, robotY, robotTheta)
            if self.debugMode:
                imgName = "depthimg"
                try:
                    cv2.imwrite( imgName + ".jpg", depthImageNormalized)
                except:
                    print("depth error")
            #print("line before write")

            return objectCoordinates
        except CvBridgeError as e:
            print(e)

    def callback(self, rgb_msg, depth_msg):
        (translation,orientation) = self.listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
        robot_x = translation[0]

        r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
        robot_theta = r_zorient  # only need the z axis
        robot_y = translation[1]
        print("My position in start of callback is: ", robot_x, robot_y, robot_theta)

  # The callback processing the pairs of numbers that arrived at approximately the same time
        rawImage = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depthImage = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

        labelArray, rectangleInfoArray, pointListArray = self.processRgb(rgb_msg, rawImage)
        for i in range(len(rectangleInfoArray)):
            object_coords = self.processDepth(depth_msg,rectangleInfoArray[i], robot_x, robot_y, robot_theta, pointListArray[i], depthImage)
            if object_coords[0] == -1:
                return
            else:
                #print("My position is: ", robot_x, robot_y, robot_theta)
                print("I detected a ", labelArray[i], " and its position is ", object_coords, ".")
            self.saveCoordinates(labelArray[i], object_coords)
            #print(self.objectCoordinates)

        return
    def saveCoordinates(self, label, coords):
        #find how many instances of that object exists in dict
        counter = 0
        while(True):
            counter += 1
            key_label = label + str(counter)
            if key_label in self.objectCoordinates.keys():

                if(self.isWithinThreshold(self.objectCoordinates[key_label][0], coords)):
                    self.objectCoordinates[key_label].append(coords)
                    return
                else:
                    continue
            else:   ##first time this object added
                self.objectCoordinates[key_label] = [coords]
                return

    def isWithinThreshold(self, point1, point2):
        distance = math.sqrt((point1[0]-point2[0]) * (point1[0]-point2[0]) + (point1[1]-point2[1]) * (point1[1]-point2[1]))
        if distance < 3:
            return True
        else:
            return False

        ###gmsl340809
