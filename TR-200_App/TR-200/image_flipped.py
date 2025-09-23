#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageFlipper:
    def __init__(self):
        # CvBridge를 사용해 ROS 이미지 메시지를 OpenCV 이미지로 변환
        self.bridge = CvBridge()

        # 카메라 토픽에서 이미지 메시지를 구독
        self.image_sub = rospy.Subscriber("/camera_1/rgb/image_rect_color", Image, self.callback)

        # 뒤집힌 이미지를 퍼블리시할 새로운 토픽
        self.image_pub = rospy.Publisher("/camera/image_flipped", Image, queue_size=10)

    def callback(self, data):
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # 이미지를 위아래로 뒤집기 (0은 위아래로 뒤집는 옵션)
            flipped_image = cv2.flip(cv_image, 0)

            # OpenCV 이미지를 다시 ROS Image 메시지로 변환
            flipped_msg = self.bridge.cv2_to_imgmsg(flipped_image, "bgr8")

            # 변환된 이미지를 새로운 토픽에 퍼블리시
            self.image_pub.publish(flipped_msg)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

def main():
    rospy.init_node('image_flipper', anonymous=True)
    ImageFlipper()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down image flipper node.")

if __name__ == '__main__':
    main()
