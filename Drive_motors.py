import time
import numpy as np
import serial
import platform
import cv2
import sys
from config import state_comm
from con import *
from utils import Utilization


print ("-------------------------------------")
print ("---- (2020-1-20)  MINIROBOT Corp. ---")
print ("-------------------------------------")
   
os_version = platform.platform()
print (" ---> OS " + os_version)
python_version = ".".join(map(str, sys.version_info[:3]))
print (" ---> Python " + python_version)
opencv_version = cv2.__version__
print (" ---> OpenCV  " + opencv_version)
print ("-------------------------------------")
 
BPS =  4800  # 4800,9600,14400, 19200,28800, 57600, 115200
spin_time = 0.001


#---------local Serial Port : ttyS0 --------
#---------USB Serial Port : ttyAMA0 --------
    

neck_UD_angle = 90
neck_RL_angle = 90
Temp_count = 0

con_spinner = Utilization('TX_con', 0.05)
RX_spinner = Utilization('RX_con', 0.002)

class nothing:
    def write(str, str2):
        pass

if state_comm == 0:
    s = serial.Serial('/dev/ttyS0', BPS, timeout=0.01)
    s.flush() # serial cls

elif state_comm == 1:
    s = serial.Serial('/dev/ttyAMA0', BPS, timeout=0.05)
    s.flush()
else:
    s = nothing()


def move_motor(a , s = s, con = con_spinner):
    global neck_RL_angle, neck_UD_angle
    if not con.is_ready():
        if a == -99:
            return neck_RL_angle, neck_UD_angle 
        else:
            return False
    if a == -99:
        return neck_RL_angle, neck_UD_angle 

    con.call()
    print(f'send command : {a}')
    a = serial.to_bytes([a])
    s.write(a)
    return neck_RL_angle, neck_UD_angle 
    '''print(f'R : {s.readline()}')
    time.sleep(0.0001)'''
    
    
def RX_angle(a, s = s):
    tmp = a
    print(f'send command : {a}')
    a = serial.to_bytes([a])
    while s.inWaiting() <= 0:
        s.flush()
        s.write(a)
        time.sleep(0.005)
    #try:
    if s.inWaiting() > 0:
        result = s.readline()
        RX = list(result)
        bool_list = [x == 0 for x in RX]
        result = [d for (d, remove) in zip(RX, bool_list) if not remove]
        if result == []:
            print('tlqkf')
            return RX_angle(tmp)
        print(f'recieving {int(np.mean(result)) - 10}')
        s.flush()
        return int(np.mean(result)) - 10
    else:
        s.flush()
        print(f'recieving {0}')
        return 0
    #except:
        


def move_to_point(arr, s = s, t = 10):
    for idx, ang in arr:
        idx = format(idx, '02')
        ang = (ang * 55) + 20000
        ud_data = f'SP00039,000{idx},{ang};'
        s.write(ud_data.encode())
        time.sleep(0.01)
    t = format(t,'05')
    ud_data = f'PP00039,{t}'
    print(f'S : {ud_data}')
    time.sleep(0.01)
    '''print(f'R : {s.readline()}')
    time.sleep(0.0001)'''

def move_motors(arr, s = s ):
    for idx,ang in arr:
        idx = format(idx, '02')
        ud_data = f'PS000{idx},{ang}'
        s.write(ud_data.encode)
        time.sleep(0.01)

def send_txt(msg_c, s = s):
    s.write(msg_c.encode())
    print(f'S: {msg_c}')



def check_bank_angle(msg):
    global neck_RL_angle
    global neck_UD_angle
    if msg == neck_L:
        neck_RL_angle = neck_RL_angle - 6
    if msg == neck_R:
        neck_RL_angle = neck_RL_angle + 6
        
    if msg == neck_L_1:
        neck_RL_angle = neck_RL_angle - 1
    if msg == neck_R_1:
        neck_RL_angle = neck_RL_angle + 1
        
    if msg == neck_U:
        neck_UD_angle = neck_UD_angle + 6
    if msg == neck_D:
        neck_UD_angle = neck_UD_angle - 6
        
    if msg == neck_U_1:
        neck_UD_angle = neck_UD_angle + 1
    if msg == neck_D_1:
        neck_UD_angle = neck_UD_angle - 1

    if msg == LR_0:
        neck_RL_angle = 0

    if msg == init_LR:
        neck_RL_angle = 90

    if msg == init_UD:
        neck_UD_angle = 90

    if msg == init_ALL:
        neck_RL_angle = 90
        neck_UD_angle = 90
    
    if neck_RL_angle > 180 or neck_RL_angle < 0:
        print(f"neck_RL_angle overflowed to {neck_RL_angle}")
        neck_RL_angle = 180*int(neck_RL_angle > 180)*int(not(neck_RL_angle<0))
        return False
    if neck_UD_angle > 180 or neck_UD_angle < 0:
        print(f"neck_UD_angle overflowed to {neck_UD_angle}")
        neck_UD_angle = 180*int(neck_UD_angle > 180)*int(not(neck_UD_angle<0))
        return False
    
    return True
