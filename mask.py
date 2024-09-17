import numpy as np
import cv2
import matplotlib.pyplot as plt
# import rs_viewer

color_dict_HSV = {'black': [[180, 255, 30], [0, 0, 0]],
              'white': [[180, 18, 255], [0, 0, 231]],
              'red1': [[180, 255, 255], [159, 50, 70]],
              'red2': [[9, 255, 255], [0, 50, 70]],
              'green': [[89, 255, 255], [36, 50, 70]],
              'blue': [[128, 255, 255], [90, 50, 70]],
              'yellow': [[35, 255, 255], [25, 50, 70]],
              'purple': [[158, 255, 255], [100, 30, 30]],
              'orange': [[24, 255, 255], [10, 50, 70]],
              'gray': [[180, 18, 230], [0, 0, 40]]}



def mask_hsv(image, lower_HSV, upper_HSV):

    upper = np.array(upper_HSV)
    lower = np.array(lower_HSV)
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask_hsv = cv2.inRange(image_hsv, lower, upper)

    # cv2.imshow("test", mask_hsv)
    result = cv2.bitwise_and(image, image, mask= mask_hsv)
    return mask_hsv, result


if __name__=="__main__":
    pass
    # test_viewer = rs_viewer.Viewer()
    # color = "purple"

    # print(f"the upper bound for {color} hsv is {color_dict_HSV[color][0]}")
    # print(f"the lower bound for {color} hsv is {color_dict_HSV[color][1]}")

    # _ , color_img = test_viewer.get_frames()

    # mask, masked = mask_hsv(color_img, color_dict_HSV[color][1], color_dict_HSV[color][0])
    # while True:
    #     cv2.imshow("image", color_img)
    #     cv2.imshow("masked", masked)
    #     key = cv2.waitKey(1)

    #     if key & 0xFF == ord('q') or key == 27:
    #         cv2.destroyAllWindows()
    #         break





## [1]"Purple HSV Color Model- flatuicolorpicker : Best Flat Colors UI Design", Flatuicolorpicker, Available: https://flatuicolorpicker.com/purple-hsv-color-model/. [Accessed 17 September. 2024].
## [2]"python - Identifying the range of a color in HSV using OpenCV - Stack Overflow", Stackoverflow, Available: https://stackoverflow.com/questions/36817133/identifying-the-range-of-a-color-in-hsv-using-opencv. [Accessed 17 September. 2024].