import cv2
import numpy as np
# import matplotlib.pyplot as plt
from std_msgs.msg import Float32
import rospy


def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    # print(image.shape)
    y1 = image.shape[0]
    y2 = int(y1 * (3 / 5))
    # print(y1, y2)
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    # print(x1, y1, x2, y2)
    return np.array([x1, y1, x2, y2])


def average_slope_intercept(image, lines):
    left_far_fit = []
    right_far_fit = []
    left_near_fit = []
    right_near_fit = []
    for line in lines:
        # print(line)
        x1, y1, x2, y2 = line.reshape(4)
        # print(x1,y1,x2,y2)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]

        if slope < 0:
            left_near_fit.append((slope, intercept))
        else:
            right_near_fit.append((slope, intercept))
    

    if left_near_fit:
        left_near_fit_average = np.average(left_near_fit, axis=0)
        # print(left_near_fit_average, "left_near")
        left_near_line = make_coordinates(image, left_near_fit_average)
    else:
        left_near_line = np.array([0, 0, 0, 0])
    if right_near_fit:
        right_near_fit_average = np.average(right_near_fit, axis=0)
        # print(right_near_fit_average, "right_near")
        right_near_line = make_coordinates(image, right_near_fit_average)
    else:
        right_near_line = np.array([0, 0, 0, 0])
    if left_near_fit and right_near_fit:
        average_slope = (right_near_fit_average[0] + left_near_fit_average[0])/2
    else:
        average_slope = 0
    # if left_far_fit:
    #     left_far_fit_average = np.average(left_far_fit, axis=0)
    #     print(left_far_fit_average, "left_far")
    #     left_far_line = make_coordinates(image, left_far_fit_average)
    # else:
    #     left_far_line = np.array([0, 0, 0, 0])
    # if right_far_fit:
    #     right_far_fit_average = np.average(right_far_fit, axis=0)
    #     print(right_far_fit_average, "right_far")
    #     right_far_line = make_coordinates(image, right_far_fit_average)
    # else:
    #     right_far_line = np.array([0, 0, 0, 0])
        # print(right_line)
    # left_fit_average = np.average(left_fit, axis=0)
    # right_fit_average = np.average(right_fit, axis=0)
    # print(left_fit_average, "left")
    # print(right_fit_average, "right")
    # left_line = make_coordinates(image, left_fit_average)
    # right_line = make_coordinates(image, right_fit_average)
    # print(left_line)
    # print(right_line)
    return np.array([left_near_line, right_near_line]), average_slope


def canny(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)

    return canny


def display_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            # print(line)
            x1, y1, x2, y2 = line.reshape(4)
            if abs(x1) < image.shape[1] and abs(x2) < image.shape[1]:
                cv2.line(
                    line_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 10
                )
    return line_image


def region_of_interest(image):
    height = image.shape[0]
    width = image.shape[1]
    # polygons = np.array([[(0, height), (width, height), (width, 250), (0, 250)]])
    polygons = np.array(
        [[(0, height), (width, height), (width,150), (0,150)]]
    )

    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


# image = cv2.imread("test_image.jpg")
# lane_image = np.copy(image)
# canny_image = canny(lane_image)
# cropped_image = region_of_interest(canny_image)
# lines = cv2.HoughLinesP(
#     cropped_image, 0.75, np.pi / 180, 150, np.array([]), minLineLength=40, maxLineGap=5
# )
# averaged_lines = average_slope_intercept(lane_image, lines)
# line_image = display_lines(lane_image, averaged_lines)
# combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)
# cv2.imshow("result", combo_image)
# cv2.waitKey(0)
# # plt.imshow(image)
# # plt.show()

pub = rospy.Publisher('slope', Float32, queue_size=10)
rospy.init_node('camera', anonymous=True)
cap = cv2.VideoCapture(0)
cap.set(3,640)
while cap.isOpened():
    _, frame = cap.read()
    canny_image = canny(frame)
    cropped_image = region_of_interest(canny_image)
    lines = cv2.HoughLinesP(
        cropped_image,
        1,
        np.pi / 180,
        50,
        np.array([]),
        minLineLength=40,
        maxLineGap=300,
    )
    averaged_lines, slope = average_slope_intercept(frame, lines)
    print("average slope", slope)
    slope_msg = Float32()
    slope_msg.data = slope
    pub.publish(slope_msg)
    line_image = display_lines(frame, averaged_lines)
    combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    # plt.imshow(frame)
    # plt.show()
    # cv2.imshow("result", combo_image)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
