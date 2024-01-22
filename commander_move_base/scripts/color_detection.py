import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image




class CameraStreamViewer:
       
    def __init__(self):

        rospy.init_node('camera_subscriber', anonymous=True)

        self.image_subscriber = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        # cv2.namedWindow("Camera Stream", cv2.WINDOW_NORMAL)

        self.cv_img = None

        self.bridge = CvBridge()

        self.rate = rospy.rate(15)


    def image_callback(self, msg):
        try:
            # bridge = CvBridge()
            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")

            self.cv_img = cv_image

            # cv2.imshow("Camera Feed" , cv_image)
            # cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def run(self):

        while rospy is open:

            if self.cv_img is not None:

                frame = self.cv_img
                frame = cv2.resize(frame, None, fx=0.3, fy=0.3)
                # iphone filters()
                # Filtering for the green color
                # 1 lower_green = np.array([0, 170, 70], dtype="uint8")
                # 1 upper_green = np.array([50, 255, 220], dtype="uint8")# -100 shadows,88 brightness, 98 stauration, 83 vibrancy
                lower_green = np.array([40, 60, 60], dtype="uint8")
                upper_green = np.array([70, 255, 140], dtype="uint8")
                mask = cv2.inRange(frame, lower_green, upper_green)
                detected_output = cv2.bitwise_and(frame, frame, mask=mask)
                # Find contours in the mask
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                good_contours = []
                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    if w*h > 35:
                        good_contours.append(contour)
                if len(good_contours)>1:
                    big_contour = np.concatenate(good_contours)
                    x, y, w, h = cv2.boundingRect(big_contour)
                    rect = cv2.minAreaRect(big_contour)
                    box = cv2.boxPoints(rect)
                    translated_box = box - np.mean(box, axis=0)
                    scaled_box = translated_box * 20 # 2 is scale factor
                    retranslated_box = scaled_box + np.mean(box, axis=0)
                    cv2.rectangle(frame, (x, y), (x + w - 1, y + h -1), 255, 2)
                # Display the result
                cv2.imshow("Green Color Detection with Bounding Boxes", frame)
                mask = cv2.inRange(frame, lower_green, upper_green)
                detected_output = cv2.bitwise_and(frame, frame, mask = mask)
                cv2.imshow("green color detection", detected_output)
                self.rate.sleep

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break


def main():
    viewer = CameraStreamViewer()
    try:
        viewer.run()
    except KeyboardInterrupt:
        print("shutting down")

if __name__ == '__main__':
    main()

# def image_callback(self, msg):
#     try:
#         # bridge = CvBridge()
#         cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")

#         cv2.imshow("Camera Feed" , cv_image)
#         cv2.waitKey(1)
#     except CvBridgeError as e:
#         print(e)

# def run(self):
#     rospy.spin()

#     cv2.destroyAllWindows()



# Open a video file
# video_path = '//home/jasper/Videos/IMG_2489.mov'  # Replace with the path to your video file
# rospy.init_node('camera_subscriber', anonymous=True)
# test = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
# cv2.namedWindow("Camera Stream", cv2.WINDOW_NORMAL)
# test_bridge = CvBridge()

# cap = cv2.VideoCapture()
# while cap.isOpened():
#     # Read a frame from the video
#     ret, frame = cap.read()
#     if not ret:
#         break
#     # Resize the frame to half its size
#     frame = cv2.resize(frame, None, fx=0.3, fy=0.3)
#     # iphone filters()
#     # Filtering for the green color
#     # 1 lower_green = np.array([0, 170, 70], dtype="uint8")
#     # 1 upper_green = np.array([50, 255, 220], dtype="uint8")# -100 shadows,88 brightness, 98 stauration, 83 vibrancy
#     lower_green = np.array([40, 60, 60], dtype="uint8")
#     upper_green = np.array([70, 255, 140], dtype="uint8")
#     mask = cv2.inRange(frame, lower_green, upper_green)
#     detected_output = cv2.bitwise_and(frame, frame, mask=mask)
#     # Find contours in the mask
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     good_contours = []
#     for contour in contours:
#         x, y, w, h = cv2.boundingRect(contour)
#         if w*h > 35:
#             good_contours.append(contour)
#     if len(good_contours)>1:
#         big_contour = np.concatenate(good_contours)
#         x, y, w, h = cv2.boundingRect(big_contour)
#         rect = cv2.minAreaRect(big_contour)
#         box = cv2.boxPoints(rect)
#         translated_box = box - np.mean(box, axis=0)
#         scaled_box = translated_box * 20 # 2 is scale factor
#         retranslated_box = scaled_box + np.mean(box, axis=0)
#         cv2.rectangle(frame, (x, y), (x + w - 1, y + h -1), 255, 2)
#     # Display the result
#     cv2.imshow("Green Color Detection with Bounding Boxes", frame)
#     mask = cv2.inRange(frame, lower_green, upper_green)
#     detected_output = cv2.bitwise_and(frame, frame, mask = mask)
#     cv2.imshow("green color detection", detected_output)
#     # Break the loop if 'q' key is pressed
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# # Release the video capture object and close all windows
# cap.release()
# cv2.destroyAllWindows()