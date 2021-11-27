# Import socket module
import socket
import cv2
import numpy as np
from PIL import Image
from flask import Flask
from io import BytesIO
import time
import base64
import eventlet
import math
import matplotlib as plt
global sendBack_angle, sendBack_Speed, current_speed, current_angle
sendBack_angle = 0
sendBack_Speed = 0
current_speed = 0
current_angle = 0

error_arr = np.zeros(5)
dt = time.time()
# Create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Define the port on which you want to connect
PORT = 54321
# connect to the server on local computer
s.connect(('127.0.0.1', PORT))


def Control(angle, speed):
    global sendBack_angle, sendBack_Speed
    sendBack_angle = angle
    sendBack_Speed = speed


def Can(image):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blur = cv2.medianBlur(gray, 7)
    #blur = cv2.GaussianBlur(gray, (7, 7), 0)
    canny = cv2.Canny(blur, 25, 255)
    return canny

def bird_view(image):
    w , h = image.shape[1],image.shape[0]
    tl = [0, h//2]
    tr = [w-1, h//2]
    bl = [0, h-1]
    br = [w-1, h-1]

    corner_points_array = np.float32([tl,tr,br,bl])

    # original image dimensions
    width = w-1
    height = h-1

    # Create an array with the parameters (the dimensions) required to build the matrix
    imgTl = [0,0]
    imgTr = [width,0]
    imgBr = [width,height]
    imgBl = [0,height]
    img_params = np.float32([imgTl,imgTr,imgBr,imgBl])

    # Compute and return the transformation matrix
    matrix = cv2.getPerspectiveTransform(corner_points_array,img_params)
    img_transformed = cv2.warpPerspective(image,matrix,(w,h))
    return img_transformed

def ROI(image):
    height = image.shape[0]
    shape = np.array([[10, height], [450, height], [450, 100], [50, 100]])
    mask = np.zeros_like(image)
    if len(image.shape) > 2:
        channel_count = image.shape[1]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    cv2.fillPoly(mask, np.int32([shape]), ignore_mask_color)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

def PID(error, p=0.45, i=0.05, d=0.02):
    global dt
    global error_arr
    error_arr[1:] = error_arr[0:-1]
    print("error_arr[0:-1] is : ",error_arr[0:-1])
    error_arr[0] = error
    P = error * p
    delta_t = time.time() - dt
    dt = time.time()
    D = (error - error_arr[1]) / delta_t * d
    I = np.sum(error_arr) * delta_t * i
    angle = P + I + D
    if abs(angle) > 45:
        angle = np.sign(angle) * 60
    return int(angle)


def display_lines(image, lines):
    line_image = np.zeros_like(image)
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10, 8)
        xa = int((x1 + x2) / 2 - 130)  # red 110 160
        ya = int((y1 + y2) / 2)
        cv2.circle(line_image, (xa - 20, ya), 5, (0, 255, 0), -1)
        x_d = xa - 150  # green 150
        cv2.circle(line_image, (x_d, ya), 5, (0, 0, 255), -1)
        angle_PID = PID(x_d)
        if angle_PID > 20:
            angle_PID = 15
        elif angle_PID < -20:
            angle_PID = -15
    return line_image, angle_PID


def average_slope_intercept(image, lines):
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]  # he so a
        intercept = parameters[1]  # he so b
        if slope > 0:
            right_fit.append((slope, intercept))
    # sap xep right_fit theo chieu tang dan cua intercept
    leng = len(right_fit)
    right_fit = np.array(sorted(right_fit, key=lambda a_entry: a_entry[0]))
    right_fit = right_fit[::-1]
    right_fit_focus = right_fit
    if leng > 2:
        right_fit_focus = right_fit[:1]
    right_fit_average = np.average(right_fit_focus, axis=0)
    right_line = make_coordinates(image, right_fit_average)
    return np.array([right_line])



def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1 * (3/5))
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])


if __name__ == "__main__":
    try:
        while True:

            """
            - Chương trình đưa cho bạn 1 giá trị đầu vào:
                * image: hình ảnh trả về từ xe
                * current_speed: vận tốc hiện tại của xe
                * current_angle: góc bẻ lái hiện tại của xe
            - Bạn phải dựa vào giá trị đầu vào này để tính toán và
            gán lại góc lái và tốc độ xe vào 2 biến:
                * Biến điều khiển: sendBack_angle, sendBack_Speed
                Trong đó:
                    + sendBack_angle (góc điều khiển): [-25, 25]
                        NOTE: ( âm là góc trái, dương là góc phải)
                    + sendBack_Speed (tốc độ điều khiển): [-150, 150]
                        NOTE: (âm là lùi, dương là tiến)
            """

            message_getState = bytes("0", "utf-8")
            s.sendall(message_getState)
            state_date = s.recv(100)

            try:
                current_speed, current_angle = state_date.decode(
                    "utf-8"
                    ).split(' ')
            except Exception as er:
                print(er)
                pass

            message = bytes(f"1 {sendBack_angle} {sendBack_Speed}", "utf-8")
            s.sendall(message)
            data = s.recv(100000)

            try:
                image = cv2.imdecode(
                    np.frombuffer(
                        data,
                        np.uint8
                        ), -1
                    )

                print(current_speed, current_angle)
                print(image.shape)
                sendBack_angle = 0
                sendBack_Speed = 10
                # your process here
                lane_image = np.copy(image)
                
                bird =bird_view(lane_image)
                cv2.imshow("bird_view", bird)
                canny_image = Can(bird)
                cropping_image = ROI(canny_image)
                cv2.imshow("CANNY", canny_image) # vung ROI da duoc xu ly
                cv2.imshow("ROI", cropping_image)
                cv2.waitKey(1)
                try:
                    lines = cv2.HoughLinesP(canny_image, 3, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=20)
                    print("lines : ",lines)
                    averaged_lines = average_slope_intercept(bird, lines)
                    line_image, sendBack_angle = display_lines(bird, averaged_lines)
                    sendBack_angle = - sendBack_angle
                    combo_image = cv2.addWeighted(bird, 0.8, line_image, 1, 1)
                    cv2.imshow("image_line", combo_image)
                    cv2.waitKey(1)

                # Control(angle, speed)
                except Exception as er:
                    print(er)
                if float(current_speed.replace(',','.')) < 16:
                    sendBack_Speed = 140
                elif float(current_speed.replace(',','.')) > 23:
                    sendBack_Speed = -6
                    print('toc do nap len {} : {}'.format(sendBack_angle, sendBack_Speed))
                    print('van toc tra ve {} : {}'.format(current_angle, current_speed))
                    Control(sendBack_angle, sendBack_Speed)
            except Exception as er:
                print(er)
                pass

    finally:
        print('closing socket')
        s.close()
