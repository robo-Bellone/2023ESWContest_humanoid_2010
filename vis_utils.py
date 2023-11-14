import time

import numpy as np
import cv2
from config import X_size, Y_size


def set_camera(cam_name, ratio, reso):
    if cam_name == 'rapa':
        camera = cv2.VideoCapture(0)
    elif cam_name in ['default', 'labtop', '']:
        camera = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    width = int(reso * ratio / 10) * 10
    height = int(reso / 10) * 10
    camera.set(3, width)
    camera.set(4, height)
    camera.set(5, 60)  
    time.sleep(0.5)
    print(f'{cam_name} camera is setted to {width}px x {height}px reso')

    return camera, (width, height)


def grab_frame(cap):
    ret,frame = cap.read()
    #cv2.imshow('tlqkf', frame)
    #print(frame)
    return frame

'''
filter_hsv - Filtering the frame with settled index data.

    input
        frame: frame that you want to filter the specific hsv threshold
        lower: list with 3 elements which you want to cut low
        upper: list with 3 elements which you want to cut high
    
    output
        type: frame which is filtered
'''
def filter_hsv(frame, lower, upper):
    lower = np.array(lower, dtype=np.uint8)
    upper = np.array(upper, dtype=np.uint8)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    return cv2.bitwise_and(frame, frame, mask=mask), mask



'''
    returns contours of masked frame
'''
def get_contours(masked_frame):
    Mask = cv2.GaussianBlur(masked_frame, (9, 9), 0)
    edges = cv2.Canny(Mask, 75, 150)
    contours, _ = cv2.findContours(Mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return contours

'''
    returns weighted summed cntr_x and cntr_y and the clustered numbers
'''
def weighted_sum_moment(contours):
    cX = []
    cY = []
    weight_area = []
    sum_weight = 0

    for contour in contours:
        area = cv2.contourArea(contour)         
        if area > 10:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                # calculate x,y coordinate of center
                cX.append( int(M["m10"] / M["m00"]) )
                cY.append( int(M["m01"] / M["m00"]) )
                weight_area.append(int(area*area))
    cX = np.array(cX).astype(int)
    cY = np.array(cY).astype(int)
    sum_weight = sum(weight_area)
    weight_area = np.array(weight_area).astype(int)

    weight_area = np.divide(weight_area, sum_weight).astype(float)

    cntr_x = np.dot(cX, weight_area).astype(int)
    cntr_y = np.dot(cY, weight_area).astype(int)

    return cntr_x, cntr_y, len(weight_area)
    
    
def get_largest_moment_contour(contours, min_moment_area=10):
    areas = np.array([cv2.contourArea(c) for c in contours])
    valid_areas = areas > min_moment_area
    if np.any(valid_areas):
        largest_contour_index = np.argmax(areas[valid_areas])
        return True, contours[np.where(valid_areas)[0][largest_contour_index]]
    else:
        return False, None
        
def get_lowest_point(contour):
    if contour is not None and len(contour) > 0:
        x, y, w, h = cv2.boundingRect(contour)
        maxY = y + h - 1  # contour의 가장 낮은 y 좌표

        # contour 배열의 형태 확인 및 처리
        if contour.ndim == 3 and contour.shape[1] == 1:  # (n, 1, 2) 형태
            points_with_maxY = contour[contour[:,0,1] == maxY][:,0,:]  # 해당 y 좌표를 가진 모든 포인트
        elif contour.ndim == 2 and contour.shape[1] == 2:  # (n, 2) 형태
            points_with_maxY = contour[contour[:,1] == maxY]  # 해당 y 좌표를 가진 모든 포인트
        else:
            return None

        averageX = np.mean(points_with_maxY[:,0]) if len(points_with_maxY) > 0 else None
        return int(averageX), maxY
    return None

def masked_channel_perc(mask):
    pixels = cv2.countNonZero(mask)
    image_area = X_size * Y_size
    return (pixels / image_area) * 100

def clustering_objects(contours):
    cX = np.array([0])
    cY = np.array([0])
    weight = np.array([0])
    sum = 0

'''
    return frame that dotted where the user described
'''
def draw_dot(frame, cX, cY, color_code = (0,255,0)):
    print(f'{cX},{cY}')
    frame = cv2.circle(frame, (int(cX), int(cY)), radius=10, color = color_code, thickness=-1)
    return frame

'''
    return frame that dotted where the user described
'''
def draw_arrow(frame, sXY, fXY, color_code = [0,255,0], thickness = 5):
    frame = cv2.arrowedLine(frame, (sXY[0], sXY[1]), (fXY[0], fXY[1]), color_code, thickness)
    return frame

def draw_contours(frame, contours):
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 10:
            frame = cv2.drawContours(frame, contour, -1, [255,0,0],5)
    return frame
