import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, TextBox
from Drive_motors import *
#from motion import *
import cv2 
from config import action_opt
from vis_utils import *

plt.ion()

txt = ''

fig = plt.figure()

# HSV 슬라이더 추가
def create_slider(name, ax_position, vmin, vmax, default_val):
    ax = plt.axes(ax_position)
    return Slider(ax, name, vmin, vmax, default_val)

h_lower_slider = create_slider('Hue Lower', [0.25, 0.055, 0.65, 0.03], 0, 179, 0)
s_lower_slider = create_slider('Saturation Lower', [0.25, 0.03, 0.65, 0.03], 0, 255, 0)
v_lower_slider = create_slider('Value Lower', [0.25, 0.005, 0.65, 0.03], 0, 255, 0)

h_upper_slider = create_slider('Hue Upper', [0.25, 0.155, 0.65, 0.03], 0, 179, 179)
s_upper_slider = create_slider('Saturation Upper', [0.25, 0.13, 0.65, 0.03], 0, 255, 255)
v_upper_slider = create_slider('Value Upper', [0.25, 0.105, 0.65, 0.03], 0, 255, 255)

axud = plt.axes([0.25, 0.18, 0.65, 0.03])
UD = Slider(axud, 'UD', 0.0, 360.0, 180.0)
#id1
t_box = plt.axes([0.05, 0.95, 0.8, 0.04])
text_box = TextBox(t_box, 'ID', initial='24')
#id2
t_box2 = plt.axes([0.05, 0.85, 0.8, 0.04])
text_box2 = TextBox(t_box2, 'ID2', initial='24')
#id3
t_box3 = plt.axes([0.05, 0.75, 0.8, 0.04])
text_box3 = TextBox(t_box3, 'ID3', initial='24')
#id4
t_box4 = plt.axes([0.05, 0.65, 0.8, 0.04])
text_box4 = TextBox(t_box4, 'ID4', initial='24')

t_box5 = plt.axes([0.05, 0.55, 0.8, 0.04])
text_box5 = TextBox(t_box5, 'console', initial=';')

t_box6 = plt.axes([0.05, 0.45, 0.8, 0.04])
text_box6 = TextBox(t_box6, 'hsv_color', initial='r')

def save_to_file(_):
    global txt
    file_name = f"config_{txt}.txt"
    print(f"saving {file_name}...")
    with open(file_name, "w") as f:
        sliders = [h_lower_slider, s_lower_slider, v_lower_slider, h_upper_slider, s_upper_slider, v_upper_slider]
        for slider in sliders:
            f.write(f"{slider.val}\n")

def load_from_file(_):
    global txt
    file_name = f"config_{txt}.txt"
    print(f'loading {file_name}...')
    with open(file_name, "r") as f:
        sliders = [h_lower_slider, s_lower_slider, v_lower_slider, h_upper_slider, s_upper_slider, v_upper_slider]
        for slider in sliders:
            slider.set_val(float(f.readline()))

save_ax = plt.axes([0.05, 0.35, 0.1, 0.04])
save_button = Button(save_ax, 'Save')
save_button.on_clicked(save_to_file)

load_ax = plt.axes([0.17, 0.35, 0.1, 0.04])
load_button = Button(load_ax, 'Load')
load_button.on_clicked(load_from_file)

fig.show()

if action_opt == 'cam_on':
    camera = cv2.VideoCapture(1, cv2.CAP_DSHOW)


fig2 = plt.figure(2)
#create subplot
ax1 = plt.subplot(2,2,1)
ax2 = plt.subplot(2,2,2)

ax3 = plt.subplot(2,2,3)
ax4 = plt.subplot(2,2,4)

#create image plot
im1 = ax1.imshow(grab_frame(camera))
im2 = ax2.imshow(grab_frame(camera))
im3 = ax3.imshow(grab_frame(camera))
im4 = ax4.imshow(grab_frame(camera))

def debugging_process():
    #Debug

    # Call update function when slider value is changed
    UD.on_changed(update)
    text_box.on_submit(update)
    text_box2.on_submit(update)
    text_box3.on_submit(update)
    text_box4.on_submit(update)
    text_box5.on_submit(update)
    text_box6.on_submit(update)
    fig.canvas.draw_idle()
    while True:
        
        lower = [h_lower_slider.val, s_lower_slider.val, v_lower_slider.val]
        upper = [h_upper_slider.val, s_upper_slider.val, v_upper_slider.val]

        frame = grab_frame(camera)
        im1.set_data(frame)
        
        filtered = (filter_hsv(frame, lower, upper))[1]
        im2.set_data(cv2.GaussianBlur((filter_hsv(frame, lower, upper))[0], (9, 9), 0))

        contours = get_contours(filtered)
        contoured = draw_contours(filtered, contours)
        im3.set_data(contoured)

        cX, cY, _ = weighted_sum_moment(contours)
        dotted = draw_dot(contoured, cX, cY)
        im4.set_data(dotted)

        fig.canvas.draw_idle()
        plt.pause(0.001)
    
    # display graph
    plt.show()
 

def update(val):
    global txt

    id = '24'
    f = UD.val
    id = int(text_box.text)
    id2 = int(text_box2.text)
    id3 = int(text_box3.text)
    id4 = int(text_box4.text)
    con = text_box5.text
    txt = text_box6.text

    move_motor(int(id), f)
    move_motor(int(id2), f)
    move_motor(int(id3), f)
    move_motor(int(id4), f)
    send_txt(con);



    

 
