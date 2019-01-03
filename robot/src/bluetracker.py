#!/usr/bin/env python
import cv2
import numpy as np
import math
import rospy
from std_msgs.msg import String
import sys


def tracker():
    rospy.init_node('tracker', anonymous=True)
    pub = rospy.Publisher('detector', String, queue_size=10)
    while not rospy.is_shutdown():
        cap = cv2.VideoCapture(0)
        # dictionary of all contours
        contours = {}
        # array of edges of polygon
        approx = []
        # scale of the text
        scale = 2
        # camera
        tellers = [0, 0, 0]

        print("press q to exit")

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

        def clearteller(nottoclear):
            if(nottoclear == 0):
                tellers[0] += 1
                tellers[1] = 0
                tellers[2] = 0
            elif nottoclear == 1:
                tellers[0] = 0
                tellers[1] += 1
                tellers[2] = 0
            elif nottoclear == 2:
                tellers[0] = 0
                tellers[1] = 0
                tellers[2] += 1

        def angle(pt1, pt2, pt0):
            dx1 = pt1[0][0] - pt0[0][0]
            dy1 = pt1[0][1] - pt0[0][1]
            dx2 = pt2[0][0] - pt0[0][0]
            dy2 = pt2[0][1] - pt0[0][1]
            return float((dx1*dx2 + dy1*dy2))/math.sqrt(float((dx1*dx1 + dy1*dy1))*(dx2*dx2 + dy2*dy2) + 1e-10)

        def checkShape(res, kleur):
            if ret == True:
                # grayscale
                gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
                # Canny
                canny = cv2.Canny(res, 80, 240, 3)
                # contours
                canny2, contours, hierarchy = cv2.findContours(
                    canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for i in range(0, len(contours)):
                    # approximate the contour with accuracy proportional to
                    # the contour perimeter
                    approx = cv2.approxPolyDP(
                        contours[i], cv2.arcLength(contours[i], True)*0.02, True)

                    # Skip small or non-convex objects
                    if(abs(cv2.contourArea(contours[i])) < 100 or not(cv2.isContourConvex(approx))):
                        continue

                    # triangle
                    if(len(approx) == 3):
                        x, y, w, h = cv2.boundingRect(contours[i])
                        clearteller(0)
                        if(tellers[0] == 10):
                            pub.publish("driehoek "+kleur)

                    elif(len(approx) >= 4 and len(approx) <= 6):
                        # nb vertices of a polygonal curve
                        vtc = len(approx)
                        # get cos of all corners
                        cos = []
                        for j in range(2, vtc+1):
                            cos.append(
                                angle(approx[j % vtc], approx[j-2], approx[j-1]))
                        # sort ascending cos
                        cos.sort()
                        # get lowest and highest
                        mincos = cos[0]
                        maxcos = cos[-1]

                        # Use the degrees obtained above and the number of vertices
                        # to determine the shape of the contour
                        x, y, w, h = cv2.boundingRect(contours[i])
                        if(vtc == 4):
                            clearteller(1)
                            if(tellers[1] == 10):

                                pub.publish("vierkant "+kleur)

                    else:
                        # detect and label circle
                        area = cv2.contourArea(contours[i])
                        x, y, w, h = cv2.boundingRect(contours[i])
                        radius = w/2
                        if(abs(1 - (float(w)/h)) <= 2 and abs(1-(area/(math.pi*radius*radius))) <= 0.2):
                            clearteller(2)
                            if(tellers[2] == 10):
                                pub.publish("cirkel "+kleur)

                # end shape detection
                # Display the resulting frame

        while(1):

            # Take each frame
            ret, frame = cap.read()

            # Convert BGR to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # define range of blue color in HSV
            lower_blue = np.array([110, 50, 50])
            upper_blue = np.array([130, 255, 255])
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([20, 255, 255])
            lower_green = np.array([65, 60, 60])
            upper_green = np.array([80, 255, 255])
            lower_yellow = np.array([25, 50, 50])
            upper_yellow = np.array([32, 255, 255])

            # Threshold the HSV image to get only blue colors
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
            red_mask = cv2.inRange(hsv, lower_red, upper_red)
            green_mask = cv2.inRange(hsv, lower_green, upper_green)
            yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            mask = red_mask + blue_mask + green_mask + yellow_mask

            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(frame, frame, mask=mask)
            res_blue = cv2.bitwise_and(frame, frame, mask=blue_mask)
            res_red = cv2.bitwise_and(frame, frame, mask=red_mask)
            res_green = cv2.bitwise_and(frame, frame, mask=green_mask)
            res_yellow = cv2.bitwise_and(frame, frame, mask=yellow_mask)

            checkShape(res_blue, "blauw")
            checkShape(res_red, "rood")
            checkShape(res_green, "groen")
            checkShape(res_yellow, "geel")

            cv2.imshow('frame', frame)
            cv2.imshow('mask', mask)
            cv2.imshow('res', res)

            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break

        cv2.destroyAllWindows()
    rospy.spin()

if __name__ == '__main__':
    try:
        tracker()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        raise SystemExit()
    
