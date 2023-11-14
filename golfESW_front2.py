import time
import cv2
import numpy as np

from Drive_motors import *
from config import *
from vis_utils import *
from utils import *


golf_lower, golf_upper, flag_lower, flag_upper = get_color_from_dat((1,2))
i = 0
j = 0
print(get_color_from_dat((1,2)))
print(f'{golf_lower}, {golf_upper}, {flag_lower}, {flag_upper}')

def golfESW():
    global i
    global j
    
    camera, shape = set_camera(cam_name, cam_ratio, cam_reso)
    flag = 0
    tag = 0
    tol = 0
    
    target_angle = 90
    move_motor(init_ALL) # 목 관절 초기화
    
    #cv2.imshow('ball', ball_channel)
    #cv2.imshow('flag', flag_channel)
    
    time.sleep(1)
    tmp_time = time.time()
    
    while True:
        neck_RL_angle, neck_UD_angle = move_motor(-99)
        if(RX_angle(28)): neck_UD_angle = RX_angle(28)
        time.sleep(0.01)
        if(RX_angle(29)): neck_UD_angle = RX_angle(29)
        print(f'{flag} is me, \n{neck_RL_angle} is my neck_rl, {neck_UD_angle} is my neck pitch,\n left turned {tol} times')
        
        frame = grab_frame(camera)
        
        ball_channel, ball_mask = filter_hsv(frame, golf_lower, golf_upper)
        flag_channel, flag_mask = filter_hsv(frame, flag_lower, flag_upper)
        
        
        if flag ==0: # 목 각도 조정
            if masked_channel_perc(ball_mask) * 30 > 5.0:
                cX, _, _ = weighted_sum_moment(get_contours(ball_mask))
                print((X_size/2) - cX)
                if (X_size/2) - cX < -80: #hype
                    move_motor(neck_R_1) # 목 관절 오른쪽
                    tag = 0
                    

                elif (X_size/2) - cX > 80: #hype
                    move_motor(neck_L_1) # 목 관절 왼
                    #목이 움직일 수 없는 각도라면 몸통 조정
                    if neck_RL_angle <= 0:
                        flag = 1
                        tol = tol + 1
                    tag = 0
                else:
                    tag = tag + 1
                    if tag > 5: #hype
                        flag = 1
                        fig = False
            elif masked_channel_perc(ball_mask) * 10 < 5.0 or fig:
                print(f'{i}, {j}')
                if i == 0:
                    i = i + 1
                    move_motor(LR_0)

                elif i == 40: #hype
                    i = 0
                    j = j + 1
                    move_motor(neck_D)
                    move_motor(neck_D)
                    move_motor(neck_D)
                elif j == 10: #hype
                    i = 0
                    j = 0
                    move_motor(init_UD)

                else:
                    i = i + 1
                    move_motor(neck_R)
        
        elif flag == 1: # 몸통 각도 조정
            neck_angle_error = target_angle - neck_RL_angle
            if neck_angle_error < -10 : 
                if time.time() - tmp_time > 1:
                    move_motor(turn_cw)
                    tmp_time = time.time()
                flag = 0
            elif neck_angle_error > 10:
                if time.time() - tmp_time > 1:
                    move_motor(turn_ccw)
                    tmp_time = time.time()
                flag = 0
            else:
                flag = 2    
                fig = True

        elif flag == 2 and fig: # 골프공이 충분히 가까워질 때 까지 접근
            cX, cY, _ = weighted_sum_moment(get_contours(ball_mask))
            move_motor(walk_fw)
            if cY > int(cam_reso*(2/3)): #hype
                move_motor(neck_D)
            elif abs((X_size / 2) - cX) > 100: #hype
                flag = 0
                tag = 0
                tol = 0
                #move_motor(init_UD)
                j = 0
            
            if neck_UD_angle < 5: #hype
                if time.time() - tmp_time > 1:
                    move_motor(walk_bw)
                    tmp_time = time.time()
                
            elif neck_UD_angle > 10:
                flag = 3
                step_bw = 0
                step_left = 0
                tmp_time = time.time()
            
            
        elif flag == 3: # 공과 거리 및 방향 조정
            ret, min_contour = get_largest_moment_contour(get_contours(ball_mask))
            if ret == False:
                flag = 0
                fig = True
            else:
                ball_min_x, ball_min_y = get_lowest_point(min_contour)
                if ((X_size / 2) - ball_min_x) < -10:
                    move_motor(neck_R_1)
                elif ((X_size / 2) - ball_min_x) > 10:
                    move_motor(neck_L_1)
                
                if ((Y_size / 2) - ball_min_y) < -10:
                    move_motor(neck_D_1)
                elif ((Y_size / 2) - ball_min_x) > 10:
                    move_motor(neck_U_1)
                    
                flag = 1
                fig = True
            
            

        elif flag == 4: #깃발과 정렬
            flagX, _, _ = weighted_sum_moment(get_contours(flag_mask))
            if (X_size/2) - flagX < -50: #hype
                if time.time() - tmp_time > 1:
                    move_motor(turn_cw)
                    tmp_time = time.time()
                    flag = 3
            elif (X_size/2) - flagX > 50: #hype
                if time.time() - tmp_time > 1:
                    move_motor(turn_ccw)
                    tmp_time = time.time()
                    flag = 3
            else:
                tag = tag + 1
                if tag > 5:
                    move_motor(init_LR)
                    time.sleep(1)
                    move_motor(init_90)
                    time.sleep(1)
                    flag = 5
        elif flag == 5:
            cX, _, _ = weighted_sum_moment(get_contours(ball_mask))
            if (X_size/2) - cX < -30: #hype
                move_motor(walk_right)
                tag = 0
            elif (X_size/2) - cX > 30: #hype
                move_motor(walk_left)
                tag = 0
            else:
                tag = tag + 1
                if tag > 5:
                    flag = 6
        elif flag == 6:
            time.sleep(3)
            move_motor(swing)
            flag = 7
        
        elif flag == 7:
            print("finished!")
            
        cv2.imshow('tlqkf', frame)
        cv2.imshow('ball', ball_mask)
        cv2.imshow('flag', flag_mask)
        
        if cv2.waitKey(1) != -1:
            break
