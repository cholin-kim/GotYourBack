import sys
sys.path.append("/home/surglab/GotYourBack/Basler")
import threading, time
# from Basler.Basler import Basler
from Basler import Basler
from ImgUtils import ImgUtils
import cv2
import numpy as np
from pynput import keyboard

import rospy
from sensor_msgs.msg import Image


class Streaming:
    def __init__(self):
        # Variables
        self.h, self.w = 1200, 1920
        # self.h , self.w = 840, 1344
        self.u, self.v = self.w // 2, self.h // 2
        self.zoom_scale = 1
        self.step = 100
        self.zoom_step = 0.1

        if not rospy.get_node_uri():
            rospy.init_node('Streaming')

        self.cam_L = Basler(serial_number="40262045")
        self.cam_L.start()
        # cam_R = Basler(serial_number="40268300")
        # cam_R.start()
        time.sleep(0.1)

        self.img_pub = rospy.Publisher('img_publisher', Image, queue_size=1)

        # self.listener = keyboard.Listener(on_press=self.on_press)
        # self.listener.start()
        # self.input =

        rate = rospy.Rate(500)

        while not rospy.is_shutdown():
            # stacked = ImgUtils.stack_stereo_img(cam_L.image, cam_R.image, scale=0.5)
            img_resized = ImgUtils.resize(self.cam_L.image, scale=0.5)
            # print(img_resized.shape)
            # quit()

            # print("zoom scale:", self.zoom_scale)
            # print("center pixel:", (u, v))

            # focused_img = cv2.circle(img_resized, (self.u, self.v), radius=1, color=(0, 0, 255), thickness=-1)
            # zoomed_img = self.zoom(img=focused_img)
            # zoomed_img = cv2.putText(zoomed_img, f"scale:{round(self.zoom_scale, 3)}", (100, 100), fontScale=3, fontFace = cv2.FONT_HERSHEY_SIMPLEX, color=(0, 255, 0), thickness=2)
            zoomed_img = img_resized


            # cv2.imshow('', zoomed_img)
            # cv2.waitKey(0)
            msg = Image()
            msg.header.stamp = rospy.Time.now()
            msg.height = zoomed_img.shape[0]
            msg.width = zoomed_img.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.data = np.array(zoomed_img).tobytes()
            self.img_pub.publish(msg)
            rate.sleep()


    def zoom(self, img):
        l, r = int(self.u - self.w / (2 * self.zoom_scale)), int(self.u + self.w / (2 * self.zoom_scale))
        s, g = int(self.v - self.h / (2 * self.zoom_scale)), int(self.v + self.h / (2 * self.zoom_scale))

        img_base = np.zeros_like(img)
        local_center_x, local_center_y = img_base.shape[1] // 2, img_base.shape[0] // 2
        if l < 0: l = 0
        if r > self.w: r = self.w
        if s < 0: s = 0
        if g > self.h: g = self.h

        # crop img
        img_cropped = img[s:g, l:r]
        new_h, new_w = img_cropped.shape[0], img_cropped.shape[1]
        # resize img & pad empty pixels
        resize_ratio = np.min([self.h / new_h, self.w / new_w])
        resized_img = cv2.resize(img_cropped, (int(new_w * resize_ratio), int(new_h * resize_ratio)))
        new_h2, new_w2 = resized_img.shape[0], resized_img.shape[1]
        pt1 = local_center_y - new_h2 // 2 - new_h2 % 2
        pt2 = local_center_x - new_w2 // 2 - new_w2 % 2
        img_base[pt1: local_center_y + new_h2 // 2, pt2: local_center_x + new_w2 // 2] = resized_img
        return img_base

    def on_press(self,key):
        try:
            if key.char == 'w':
                self.v -= self.step
            elif key.char == 'a':
                self.u -= self.step
            elif key.char == 's':
                self.v += self.step
            elif key.char == 'd':
                self.u += self.step
            elif key.char == 'e':
                self.zoom_scale += self.zoom_step
            elif key.char == 'r':
                self.zoom_scale -= self.zoom_step
            elif key.char == 'q':
                self.cam_L.stop()
                quit()
        except AttributeError:
            pass
        # threshold
        if self.v <= 0: self.v = self.step
        if self.v >= self.h: self.v = self.h - self.step
        if self.u <= 0: self.u = self.step
        if self.u >= self.w: self.u = self.w - self.step
        if self.zoom_scale >= 5: self.zoom_scale = 5
        if self.zoom_scale <= 0: self.zoom_scale = self.zoom_step

# global u, v
# global w, h
# global zoom_scale
#
#
# # h, w = 1200, 1920
# h, w = 840, 1344
# u, v = w // 2, h // 2
# zoom_scale = 1
# step = 100
# zoom_step = 0.1
#
#
# def zoom(img, center, zoom_scale):
#     (u, v) = center
#     l, r = int(u - w / (2 * zoom_scale)), int(u + w / (2 * zoom_scale))
#     s, g = int(v - h / (2 * zoom_scale)), int(v + h / (2* zoom_scale))
#
#     img_base = np.zeros_like(img)
#     local_center_x, local_center_y = img_base.shape[1] // 2, img_base.shape[0] // 2
#     if l < 0: l = 0
#     if r > w: r = w
#     if s < 0: s = 0
#     if g > h: g = h
#
#     # crop img
#     img_cropped = img[s:g, l:r]
#     new_h, new_w = img_cropped.shape[0], img_cropped.shape[1]
#     # resize img & pad empty pixels
#     resize_ratio = np.min([h / new_h, w / new_w])
#     resized_img = cv2.resize(img_cropped, (int(new_w * resize_ratio), int(new_h * resize_ratio)))
#     new_h2, new_w2 = resized_img.shape[0], resized_img.shape[1]
#     pt1 = local_center_y - new_h2 // 2 - new_h2 % 2
#     pt2 = local_center_x - new_w2 // 2 - new_w2 % 2
#     img_base[pt1 : local_center_y + new_h2//2, pt2 : local_center_x + new_w2//2] = resized_img
#     return img_base
#
# def on_press(key):
#     global u, v, zoom_scale
#
#     try:
#         if key.char == 'w':
#             v -= step
#         elif key.char == 'a':
#             u -= step
#         elif key.char == 's':
#             v += step
#         elif key.char == 'd':
#             u += step
#         elif key.char == 'e':
#             zoom_scale += zoom_step
#         elif key.char == 'r':
#             zoom_scale -= zoom_step
#         elif key.char == 'q':
#             cam_L.stop()
#             quit()
#     except AttributeError:
#         pass
#     # threshold
#     if v <= 0: v = step
#     if v >= h: v = h - step
#     if u <= 0: u = step
#     if u >= w: u = w - step
#     if zoom_scale >= 5: zoom_scale = 5
#     if zoom_scale <= 0: zoom_scale = zoom_step


if __name__ == "__main__":
    stream = Streaming()
    # import rospy
    # from sensor_msgs.msg import Image
    #
    # if not rospy.get_node_uri():
    #     rospy.init_node('img_publisher')
    #
    # cam_L = Basler(serial_number="40262045")
    # cam_L.start()
    # # cam_R = Basler(serial_number="40268300")
    # # cam_R.start()
    # time.sleep(0.1)
    #
    #
    # img_pub = rospy.Publisher('img_publisher', Image, queue_size=1)
    #
    #
    # listener = keyboard.Listener(on_press=on_press)
    # listener.start()
    #
    # while not rospy.is_shutdown():
    #     # stacked = ImgUtils.stack_stereo_img(cam_L.image, cam_R.image, scale=0.5)
    #     img_resized = ImgUtils.resize(cam_L.image, scale=0.7)
    #     # print(img_resized.shape)
    #     # quit()
    #
    #     print("zoom scale:", zoom_scale)
    #     # print("center pixel:", (u, v))
    #
    #     focused_img = cv2.circle(img_resized, (u, v), radius=1, color=(0, 0, 255), thickness=-1)
    #     zoomed_img = zoom(img=focused_img, center=(u, v), zoom_scale=zoom_scale)
    #
    #
    #     cv2.imshow('', zoomed_img)
    #     cv2.waitKey()
    #     msg = Image()
    #     msg.header.stamp = rospy.Time.now()
    #     msg.height = zoomed_img.shape[0]
    #     msg.width = zoomed_img.shape[1]
    #     msg.encoding = "bgr8"
    #     msg.is_bigendian = False
    #     # msg.step = 3 * zoomed_img.width
    #     msg.data = np.array(zoomed_img).tobytes()
    #     img_pub.publish(msg)






        # key = cv2.waitKey(1) & 0xFF

        # move focus
        # if key == ord('w'):
        #     v -= step
        # elif key == ord('a'):
        #     u -= step
        # elif key == ord('s'):
        #     v += step
        # elif key == ord('d'):
        #     u += step
        #
        # # zoom in or out
        # elif key == ord('i'):
        #     zoom_scale += zoom_step
        # elif key == ord('o'):
        #     zoom_scale -= zoom_step
        #
        # elif key == ord('q'):
        #     break



        # if key == ord('q'):
        #     cam_L.stop()
        #     # cam_R.stop()
        #     break






