#!/usr/bin/env python3

#entire frame considering as a rectangle and centre line and centre point estimation

import sys
print(sys.version)


import imutils
import cv2
import numpy as np
import roslib
import sys
import rospy
from std_msgs.msg import Float64, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math

rospy.init_node('image_listener')

pub = rospy.Publisher('lateral_error', Float32MultiArray, queue_size=1)

# DIM = (640, 480)
# K = np.array(
#     [[401.85180238132534, 0.0, 315.0881937529724], [0.0, 534.1449296513911, 267.5899112187299], [0.0, 0.0, 1.0]])
# D = np.array([[-0.044447931423351454], [0.09001009612247628], [-0.017793512771069935], [-0.25484017856839847]])
def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    # channel_count = img.shape[2]
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)

    # mask = np.ones((480, 640))  # (height, width)
    # myROI = [(0,480), (320, 240), (325, 240), (640,480)]  # (x, y)
    # cv2.fillPoly(mask, [np.array(myROI)], 0)
    masked_image = cv2.bitwise_and(img, mask)

    return masked_image
    
def callback(msg):

    #bridge = CvBridge()

    try:
        # Convert your ROS Image message to OpenCV2
        #cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    except CvBridgeError as e:
        print(e)
    else:
        frame = cv_image
        # ret, frame = cam.read()
        height = frame.shape[0]
        width = frame.shape[1]
        #print(height,width)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = brightness_contrast(frame, 0)
        # map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        # undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        pts1 = np.float32([[230, 151], [730, 151], [0, 616], [960, 616]])
        warped_img = warp_image(frame, pts1, 480, 616)
        #print dimensions of warped image frame
        #print(warped_img.shape[0], warped_img.shape[1])
        gauss_img = cv2.GaussianBlur(warped_img, (5, 5), 0)
        hsv_img = cv2.cvtColor(gauss_img, cv2.COLOR_BGR2HSV)
        region_of_interest_vertices = [
            (0, 360),
            (0, 0),
            (480, 0),
            (480, 360)]
        # mask = cv2.inRange(hsv_img, lower_color, upper_color)
        # cv2.imshow('A',mask)
        gray_image = cv2.cvtColor(warped_img, cv2.COLOR_RGB2GRAY)
        # gray_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        # kernel = np.ones((15, 15), np.uint8)
        # thresh = cv2.erode(mask, kernel, iterations=2)
        # thresh = cv2.dilate(thresh, kernel, iterations=2)
        # canny_image = cv2.Canny(thresh, 100, 200)
        #
        cropped_image = region_of_interest(gray_image,
                                              np.array([region_of_interest_vertices], np.int32), )
        #cv2.imshow('TEST', cropped_image)
        #cv2.imshow('TEST', cropped_image)
        contours, hierarchy= cv2.findContours(cropped_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #print((contours))
        # areas = [cv2.contourArea(c) for c in contours]
        # max_index = np.argmax(areas)
        # cnt = contours[max_index]
        if len(contours) > 0:
        #if 0:
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)

            # cv2.rectangle(warped_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.drawContours(warped_img, contours, -1, (0, 255, 0), thickness=1)

            M = cv2.moments(c)
            if M["m00"] == 0:
                return
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            cv2.circle(warped_img, (cX, cY), 5, (36, 255, 12), -1)

	    
            rows, cols = gray_image.shape[:2]
            [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)

            lefty = int((-x * vy / (vx)) + y)
            righty = int(((cols - x) * vy / (vx)) + y)
            # print("line ", (cols - 1, righty), (0, lefty))
            line = np.array([0, lefty, cols - 1, righty])
            heading = self.get_heading_error(line.astype(np.float32))

            # To draw line you can use cv2.line or numpy slicing
            cv2.line(warped_img, (x + int(w / 2), y), (x + int(w / 2), y + h), (0, 0, 255), 3)
            # image[int(cY - h/2):int(cY+h/2), cX] = (36, 255, 12)
            cv2.line(warped_img, (240, cY), (cX, cY), (255, 0, 0), thickness=1)
            width1=200
            # [vx, vy, x, y] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
            # lefty = int((-x * vy / vx) + y)  # n valuue in y = vy/vx*x + n y(x=0)
            # righty = int(((width1 - x) * vy / vx) + y)
            # img = cv2.line(warped_img, (width1, righty), (0, lefty), (255, 0, 0), thickness=1)

            pub.publish(cX-240)

        else:
            rospy.loginfo("No lines")
            pub.publish(0)

        cv2.imshow('Image', warped_img)

        # cv2.imshow('Crop Image', cropped_image)
        # cv2.imshow('Gray Image', gray_image)

        cv2.waitKey(1)

def warp_image(img, pts1, width=640, height=480):
    pts2 = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgOutput = cv2.warpPerspective(img, matrix, (width, height))
    return imgOutput

def get_heading_error(line):

        # bottom point
        x1 = line[0]
        y1 = line[1]
        # Top point (centre)
        x2 = line[2]
        y2 = line[3]

        num = y2 - y1
        if num > 0:
            den = x2 - x1
        else:
            num = y1 - y2
            den = x1 - x2
        if den is not 0:
            slope = num / den
            heading = math.degrees(math.atan(slope))
            # if abs(heading)>90:
            if heading == 0:
                return 0
            sign = -heading / abs(heading)
            heading = abs(90 - (abs(heading))) * sign
        else:
            heading = 0
        return heading

        # slope = (y2 - y1) / (x2 - x1)

def brightness_contrast(img, brightness=100):

    # getTrackbarPos returns the current
    # position of the specified trackbar.
    brightness = cv2.getTrackbarPos('Brightness', 'Image')
    #brightness = 200
    contrast = cv2.getTrackbarPos('Contrast', 'Image')
    #contrast =250
    effect = controller(img, brightness, contrast)

    # The function imshow displays an image
    # in the specified window
    # cv2.imshow('Effect', effect)
    return effect


def controller(img, brightness=255, contrast=127):

    brightness = int((brightness - 0) * (255 - (-255)) / (510 - 0) + (-255))

    contrast = int((contrast - 0) * (127 - (-127)) / (254 - 0) + (-127))

    if brightness != 0:

        if brightness > 0:

            shadow = brightness

            max = 255

        else:

            shadow = 0
            max = 255 + brightness

        al_pha = (max - shadow) / 255
        ga_mma = shadow

        # The function addWeighted calculates
        # the weighted sum of two arrays
        cal = cv2.addWeighted(img, al_pha,
                              img, 0, ga_mma)

    else:
        cal = img

    if contrast != 0:
        Alpha = float(131 * (contrast + 127)) / (127 * (131 - contrast))
        Gamma = 127 * (1 - Alpha)

        # The function addWeighted calculates
        # the weighted sum of two arrays
        cal = cv2.addWeighted(cal, Alpha,
                              cal, 0, Gamma)

        # putText renders the specified text string in the image.
    cv2.putText(cal, 'B:{},C:{}'.format(brightness,
                                        contrast), (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return cal





def main():

    cv2.namedWindow('Image')
    #createTrackbar(trackbarName,
    #windowName, value, count, onChange)
    #Brightness range -255 to 255
    cv2.createTrackbar('Brightness',
                       'Image', 255, 2 * 255,
                       brightness_contrast)

    # Contrast range -127 to 127
    cv2.createTrackbar('Contrast', 'Image',
                       240, 2 * 127,
                       brightness_contrast)


    # Define your image topic
    image_topic = "rover/camera1/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, callback)
    # Spin until ctrl + c
    rospy.spin()
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
