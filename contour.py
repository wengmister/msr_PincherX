#! /usr/lib/env python3

import cv2

def ordered_contour(image, threshold):

    blurred_image = cv2.GaussianBlur(image, (5, 5), 0)

    # Apply threshold to get binary image
    ret, thresh = cv2.threshold(blurred_image, threshold, 255, 0)
    
    # Find contours
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None, None, None  # No contours found

    max_area = 0
    largest_com = None
    
    for c in contours:
        # Calculate area of contour
        area = cv2.contourArea(c)
        
        # Ignore very small contours to avoid errors in moment calculation
        if area > 50:

            # Smooth contours
            epsilon = 0.05 * cv2.arcLength(c, True)  # Tweak epsilon for more or less smoothness
            approx = cv2.approxPolyDP(c, epsilon, True)

            # Calculate moments for each contour
            M = cv2.moments(approx)

            # Ensure m00 is not zero to avoid division by zero
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # If this contour is the largest, store its COM
                if area > max_area:
                    max_area = area
                    largest_com = (cX, cY)
    
    return contours, largest_com, max_area

## [1]"Find Center of a Blob (Centroid) Using OpenCV (C /Python) | LearnOpenCV", Learnopencv, Available: https://learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/. [Accessed 17 September. 2024].