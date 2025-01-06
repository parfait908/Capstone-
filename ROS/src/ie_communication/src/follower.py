#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from qreader import QReader
#from ie_communication.srv import robotGear, robotGearResponse
class Follower:
    def __init__(self):
        self.qreader = QReader()
        self.rate = rospy.Rate(10)
        self.msg = Twist()
        self.bridge = CvBridge()
        self.param = {"KP": 0.0046, "SP": 0.05}

        self.decisionMade = False
        self.hasDetectedQrRecently = False
        self._lastQrCode = None

        self._subcamqr = rospy.Subscriber("/camera_qr_code_feed",Image, self._camqrProcess)
        self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.pub = rospy.Publisher("/error", Float32, queue_size=1)
        self.pub2 = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.img_size = None
        self._black_pixels = 492250
        self._threshold = 0.006
        self._middle_width = 580
        self._autonomous = False
        # rospy.wait_for_service('change_gear')
        # robot_gear = rospy.ServiceProxy('change_gear', robotGear)
        # robot_gear.wait_for_service(10)
        # try:
        #     response = robot_gear(0)
        #     if response.message:
        #         print("Gear changed to autonomous")
        #     self._autonomous =  response.message
        # except rospy.ServiceException as exc:
        #     print("Service did not process request: " + str(exc))

    def _camqrProcess(self, data):
        try: 
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
            decoded_text = self.qreader.detect_and_decode(image=cv_image)
            if decoded_text is not None and len(decoded_text) != 0:
                if decoded_text[0] != self._lastQrCode:
                    self._lastQrCode = decoded_text[0]
                    self.hasDetectedQrRecently = True
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

    def _move_forward(self):
        """Move forward at a constant speed."""
        self.msg.linear.x = self.param["SP"]
        self.msg.angular.z = 0.0
        self.pub2.publish(self.msg)


    def _adjust_orientation(self, error, pixels, mask , w_min, h_min):
        """Adjust the robot's orientation based on the error."""
        # if not self._autonomous:
        #     return None
        

        if (w_min < 100 and h_min < 100) :
            #self._move_forward()
            return None
        
        
        if (pixels <= self._black_pixels + (self._black_pixels * self._threshold)) and  (pixels >= self._black_pixels - (self._black_pixels * self._threshold)):
            self.msg.linear.x = self.param["SP"]
            self.msg.angular.z = -self.param["KP"] * error
            self.pub2.publish(self.msg)
            return None
        
        if pixels > (self._black_pixels + (self._black_pixels * self._threshold)):
            
            left_pixels, right_pixels, onLeft, onRight = self.check_side_pixels(mask)

            if onLeft or onRight:
                top_pixels_remaining, top_pixels_side, bottom_pixels_side, onTop = self.check_straight_pixels(mask)
                return [left_pixels, right_pixels,top_pixels_remaining, top_pixels_side, bottom_pixels_side]
             
            #self._move_forward()
            return [left_pixels, right_pixels,0,0,0]


        self.msg.linear.x = self.param["SP"]
        self.msg.angular.z = -self.param["KP"] * error
        self.pub2.publish(self.msg)
        return None 

    def check_side_pixels(self, mask):
        onLeft = False
        onRight = False

        height, width = mask.shape
        middle_start = (width // 2) - (self._middle_width // 2)
        middle_end = (width // 2) + (self._middle_width // 2)

        middle_mask = np.zeros_like(mask)
        middle_mask[:, middle_start:middle_end] = 255 

        masked_binary = cv2.bitwise_and(mask, cv2.bitwise_not(middle_mask))
        left_half = masked_binary[:, :width // 2]
        right_half = masked_binary[:, width // 2:]
        left_pixels = np.sum(left_half == 255)
        right_pixels = np.sum(right_half == 255)
        print(f"Left pixels: {left_pixels}, Right pixels: {right_pixels}")

        threshold_lr = (self._black_pixels // 3) - 20
        if left_pixels > threshold_lr :
            onLeft = True
        if right_pixels > threshold_lr:
            onRight = True
        
        return left_pixels, right_pixels, onLeft, onRight

    def check_straight_pixels(self, mask):
        onTop = False

        height, width = mask.shape
        middle_start = (width // 2) - (self._middle_width // 2)
        middle_end = (width // 2) + (self._middle_width // 2)

        middle_mask = np.zeros_like(mask)
        middle_mask[:, middle_start:middle_end] = 255 

        masked_binary = cv2.bitwise_and(mask, cv2.bitwise_not(middle_mask))
        top_half = masked_binary[:height // 2, :]
        bottom_half = masked_binary[height // 2:, :]

        top_pixels = np.sum(top_half == 255)
        bottom_pixels = np.sum(bottom_half == 255)
        print(f"Top pixels: {top_pixels}, Bottom pixels: {bottom_pixels}")

        if bottom_pixels >= top_pixels:
            middle_line = mask[:, middle_start:middle_end]
            height, _ = middle_line.shape
            top_half = middle_line[:height // 2, :]
            bottom_half = middle_line[height // 2:, :]

            bottom_half[bottom_half == 255] = 0
            top_pixels_remaining = np.sum(top_half == 255)
            continuity_threshold = 0.5 * np.sum(middle_line == 255)

            top_mask = np.zeros_like(mask)
            top_mask[height // 2:, :] = 255
            masked_binary2 = cv2.bitwise_and(mask, cv2.bitwise_not(top_mask))
            cv2.imshow("masked",masked_binary2)
            if top_pixels_remaining >= continuity_threshold:
                onTop = True
                return top_pixels_remaining, top_pixels, bottom_pixels, onTop
            else:
                return top_pixels_remaining, top_pixels, bottom_pixels, onTop
        else:
            return 0, top_pixels, bottom_pixels, onTop


    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
            black_pixels = np.sum(mask == 255)

            x_last = image.shape[1] / 2
            y_last = image.shape[0] / 2
            self.img_size = image.shape
            Blackline = cv2.inRange(image, (0,0,0), (60,60,60))	
            kernel = np.ones((3,3), np.uint8)
            Blackline = cv2.erode(Blackline, kernel, iterations=5)
            Blackline = cv2.dilate(Blackline, kernel, iterations=9)	
            contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

            contours_blk_len = len(contours_blk)
            if contours_blk_len > 0 :
                if contours_blk_len == 1 :
                    blackbox = cv2.minAreaRect(contours_blk[0])
                else:
                    canditates=[]
                    off_bottom = 0	   
                    for con_num in range(contours_blk_len):		
                        blackbox = cv2.minAreaRect(contours_blk[con_num])
                        (x_min, y_min), (w_min, h_min), ang = blackbox		
                        box = cv2.boxPoints(blackbox)
                        (x_box,y_box) = box[0]
                        if y_box > 358 :		 
                            off_bottom += 1
                        canditates.append((y_box,con_num,x_min,y_min))		
                    canditates = sorted(canditates)
                    if off_bottom > 1:	    
                        canditates_off_bottom=[]
                        for con_num in range ((contours_blk_len - off_bottom), contours_blk_len):
                            (y_highest,con_highest,x_min, y_min) = canditates[con_num]		
                            total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
                            canditates_off_bottom.append((total_distance,con_highest))
                            canditates_off_bottom = sorted(canditates_off_bottom)         
                            (total_distance,con_highest) = canditates_off_bottom[0]         
                            blackbox = cv2.minAreaRect(contours_blk[con_highest])	   
                    else:		
                        (y_highest,con_highest,x_min, y_min) = canditates[contours_blk_len-1]		
                        blackbox = cv2.minAreaRect(contours_blk[con_highest])	 
                (x_min, y_min), (w_min, h_min), ang = blackbox
                x_last = x_min
                y_last = y_min
                if ang < -45 :
                    ang = 90 + ang
                if w_min < h_min and ang > 0:	  
                    ang = (90-ang)*-1
                if w_min > h_min and ang < 0:
                    ang = 90 + ang	  
                setpoint = image.shape[1] / 2
                error = int(x_min - setpoint) 
                ang = int(ang)	 
                box = cv2.boxPoints(blackbox)
                box = np.intp(box)
                cv2.drawContours(image,[box],0,(0,0,255),3)	 
                cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
                cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(image,str(error),(10, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.putText(image,str(f"UCL:{self._black_pixels + (self._black_pixels * self._threshold)}"),(10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.putText(image,str(f"CL:{self._black_pixels}"),(10, 160), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(image,str(f"LCL:{self._black_pixels - (self._black_pixels * self._threshold)}"),(10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                cv2.putText(image,str(f"BP:{black_pixels}"),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                
                if black_pixels > (self._black_pixels + (self._black_pixels * self._threshold)):
                    cv2.line(image, (image.shape[1] // 2, 0), (image.shape[1] // 2, image.shape[0]), (0, 0, 255), 2)
                    cv2.line(image, (0, image.shape[0] // 2), (image.shape[1], image.shape[0] // 2), (0, 0, 255), 2)

                

                px = self._adjust_orientation(error, black_pixels, mask, w_min, h_min)
                if px is not None:
                    cv2.putText(image,str(f"Left px:{px[0]}"),(10, 480), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    cv2.putText(image,str(f"Right px:{px[1]}"),(10, 560), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    cv2.putText(image,str(f"Top px:{px[2]}"),(10, 640), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    cv2.putText(image,str(f"Top px side:{px[3]}"),(10, 720), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    cv2.putText(image,str(f"Bot px side:{px[4]}"),(10, 800), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            else:
                self._move_forward()

           
            
            cv2.imshow('Main', image)
            
        except CvBridgeError as e:
            print(e)
        if cv2.waitKey(1) == 27:
            rospy.signal_shutdown("shutdown")
            cv2.destroyAllWindows()




if __name__ == '__main__':

    rospy.init_node('Follower')
    th = Follower()
    rospy.spin()