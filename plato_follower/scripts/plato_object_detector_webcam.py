#!/usr/bin/env python
from __future__ import print_function
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import imutils
import cv2
import rospy
import darknet as dn
from plato_api.constants import path_yolo, plato_ip
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


rospy.init_node('image_detected_objects_publisher', anonymous=True)
image_pub = rospy.Publisher("img_detected_objects", Image)
bridge = CvBridge()

net = dn.load_net("{}/cfg/yolov3.cfg".format(path_yolo),
                  "{}/weights/yolov3.weights".format(path_yolo), 0)
meta = dn.load_meta("{}/cfg/coco.data".format(path_yolo))

font = cv2.FONT_HERSHEY_SIMPLEX
cv2.namedWindow('Image', flags=cv2.WINDOW_NORMAL)
labels_list = ['person']
chart_colors = [(204, 102, 51), (18, 557, 220), (0, 153, 255),
                (24, 150, 16), (175, 175, 246), (172, 62, 59), (198, 153, 0)]

vs = WebcamVideoStream(
    src='http://{}:8080/stream?topic=/usb_cam/image_raw'.format(plato_ip)).start()
fps = FPS().start()

while True:
    img = vs.read()
    detected_objects = dn.detect(net, meta, img)
    cnt = 0
    if detected_objects != []:
        while cnt < len(detected_objects):
            name = detected_objects[cnt][0]
            if name in labels_list:
                i = labels_list.index(name)
                predict = detected_objects[cnt][1]
                print(name + ":" + str(predict))
                x = detected_objects[cnt][2][0]
                y = detected_objects[cnt][2][1]
                w = detected_objects[cnt][2][2]
                z = detected_objects[cnt][2][3]
                #print (x, y, w, z)

                x_max = int(round((2 * x + w) / 2))
                x_min = int(round((2 * x - w) / 2))
                y_min = int(round((2 * y - z) / 2))
                y_max = int(round((2 * y + z) / 2))
                print(x_min, y_min, x_max, y_max)
                pixel_list = [x_min, y_min, x_max, y_max]
                neg_index = [pixel_list.index(val)
                             for val in pixel_list if val < 0]
                cv2.rectangle(img, (x_min, y_min),
                              (x_max, y_max), (chart_colors[i]), 2)
                if neg_index == []:
                    cv2.rectangle(img, (x_min, y_min - 24),
                                  (x_min + 10 * len(name), y_min), chart_colors[i], -1)
                    cv2.putText(img, name, (x_min, y_min - 12),
                                font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                else:
                    if (y_min < 0 and x_min > 0):
                        cv2.rectangle(
                            img, (x_min, 0), (x_min + 10 * len(name), 24), chart_colors[i], -1)
                        cv2.putText(img, name, (x_min, 12),
                                    font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                    elif (x_min < 0 and y_min > 0):
                        cv2.rectangle(img, (0, y_min - 24),
                                      (10 * len(name), y_min), chart_colors[i], -1)
                        cv2.putText(img, name, (0, y_min - 12),
                                    font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
                    elif (x_min < 0 and y_min < 0):
                        cv2.rectangle(img, (0, 0),
                                      (10 * len(name), 24), chart_colors[i], -1)
                        cv2.putText(img, name, (0, 12), font,
                                    0.5, (0, 0, 0), 1, cv2.LINE_AA)
                # cv2.imshow('image',img)
                #cropped = image.crop((x_min, y_min+20, x_max, y_max))
                cnt += 1
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError as e:
        print(e)
    # cv2.imshow('Image', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()
vs.stop()
