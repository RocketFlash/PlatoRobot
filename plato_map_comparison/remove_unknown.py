#!/usr/bin/env python
import numpy as np
import cv2
import ICP


def start():
    img = cv2.imread('/home/rauf/Desktop/MAPS/test_gmapping.pgm',-1)
    img[img == 205] = 255
    kernel = np.ones((7,7), np.uint8)
    erosion = cv2.erode(img, kernel, iterations = 1)
    blur = cv2.GaussianBlur(erosion,(5,5),0)
    canny = cv2.Canny(blur, 50, 120)
    small_img = cv2.resize(blur, (0,0), fx=0.2, fy=0.2)
    cv2.imshow('image',blur)
    cv2.imshow('small_image',small_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    start()
