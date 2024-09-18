#! /usr/lib/env python3

import cv2

def ordered_contour(image, threshold):

    ret,thresh = cv2.threshold(image,threshold,255,0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    areas = []
    for i in contours:
        areas.append(cv2.contourArea(i))
    
    return contours, areas

## [1]"Find Center of a Blob (Centroid) Using OpenCV (C /Python) | LearnOpenCV", Learnopencv, Available: https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/. [Accessed 17 September. 2024].