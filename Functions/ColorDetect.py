#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/TurboPi/')
import cv2
import time
import math
import signal
import Camera
import threading
import numpy as np
import yaml_handle

# 颜色识别(color recognition)
board = None
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

servo1 = 1500
servo2 = 1500
target_color = ('red', 'green', 'blue')

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

# 初始位置(initial position)
def initMove():
    board.pwm_servo_set_position(1, [[1, servo1], [2, servo2]])

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

_stop = False
color_list = []
size = (640, 480)
__isRunning = False
detect_color = 'None'
start_pick_up = False
draw_color = range_rgb["black"]

# 变量重置(variables reset)
def reset(): 
    global _stop
    global color_list
    global detect_color
    global start_pick_up
    global servo1, servo2
    
    _stop = False
    color_list = []
    detect_color = 'None'
    start_pick_up = False
    servo1 = servo_data['servo1']
    servo2 = servo_data['servo2']

# app初始化调用(app initialization call)
def init():
    print("ColorDetect Init")
    load_config()
    reset()
    initMove()

# app开始玩法调用(app start program call)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorDetect Start")

# app停止玩法调用(app stop program call)
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorDetect Stop")

# app退出玩法调用(app exit program call)
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    set_rgb('None')
    print("ColorDetect Exit")

def setTargetColor(color):
    global target_color

    target_color = color
    return (True, ())


#设置扩展板的RGB灯颜色使其跟要追踪的颜色一致(set the RGB lights on the expansion board to match the color to be tracked)
def set_rgb(color):
    if color == "red":
        board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])
    elif color == "green":
        board.set_rgb([[1, 0, 255, 0], [2, 0, 255, 0]])
    elif color == "blue":
        board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])
    else:
        board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])

# 找出面积最大的轮廓(find the contour with the largest area)
# 参数为要比较的轮廓的列表(the parameter is a list of contours to compare)
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓(iterate through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only the maximal contour with an area greater than 300 is considered valid to filter out interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the maximal contour)

# 机器人移动逻辑处理(robot movement logic processing)
def move():
    global _stop
    global __isRunning
    global detect_color
    global start_pick_up
    

    while True:
        if __isRunning:
            if detect_color != 'None' and start_pick_up:  # 检测到色块(detected color block)
                board.set_buzzer(1900, 0.1, 0.9, 1)# 设置蜂鸣器响0.1秒(set the buzzer to emit for 0.1 second)
                set_rgb(detect_color) # 设置扩展板上的彩灯与检测到的颜色一样(set the colored light on the expansion board to match the detected color)
                
                if detect_color == 'red' :  # 检测到红色,点头(detecting red color, nodding)
                    for i in range(0,3):
                        board.pwm_servo_set_position(0.3, [[1, servo1-100]])
                        time.sleep(0.3)
                        board.pwm_servo_set_position(0.3, [[1, servo1+100]])
                        time.sleep(0.3)
                    board.pwm_servo_set_position(0.5, [[1, servo1]])  # 回到初始位置(return to the initial position)
                    time.sleep(0.5)       
                    
                else:                      # 检测到绿色或者蓝色，则摇头(detecting green or blue color, shaking head)
                    for i in range(0,3):
                        board.pwm_servo_set_position(0.35, [[2, servo2-150]])
                        time.sleep(0.35)
                        board.pwm_servo_set_position(0.35, [[2, servo2+150]])
                        time.sleep(0.35)
                    board.pwm_servo_set_position(0.5, [[2, servo2]])  # 回到初始位置(return to the initial position)
                    time.sleep(0.5)
                    
                _stop = True
                detect_color = 'None'
                start_pick_up = False
                set_rgb(detect_color)
                
            else:
                time.sleep(0.01)
        else:
            if _stop:
                initMove()  # 回到初始位置(return to the initial position)
                _stop = False
                time.sleep(1.5)               
            time.sleep(0.01)

# 运行子线程(run a sub-thread)
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# 机器人图像处理(robot images processing)
def run(img):
    global __isRunning
    global start_pick_up
    global detect_color, draw_color, color_list
    
    if not __isRunning:  # 检测是否开启玩法，没有开启则返回原图像(check if the program is enabled, return the original image if not enabled)
        return img
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to the LAB space)

    color_area_max = None
    max_area = 0
    areaMaxContour_max = 0
    if not start_pick_up:
        for i in target_color:
            if i in lab_data:
                frame_mask = cv2.inRange(frame_lab,
                                             (lab_data[i]['min'][0],
                                              lab_data[i]['min'][1],
                                              lab_data[i]['min'][2]),
                                             (lab_data[i]['max'][0],
                                              lab_data[i]['max'][1],
                                              lab_data[i]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation on the original image and mask)
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算(opening operation)
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算(closing operation)
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find contours)
                areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓(find the maximal contour)
                if areaMaxContour is not None:
                    if area_max > max_area:  # 找最大面积(find the maximal area)
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 2500:  # 有找到最大面积(the maximal area is found)
            rect = cv2.minAreaRect(areaMaxContour_max)
            box = np.intp(cv2.boxPoints(rect))
            
            cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)
            if not start_pick_up:
                if color_area_max == 'red':  # 红色最大(maximum red)
                    color = 1
                elif color_area_max == 'green':  # 绿色最大(maximum green)
                    color = 2
                elif color_area_max == 'blue':  # 蓝色最大(maximum blue)
                    color = 3
                else:
                    color = 0
                color_list.append(color)
                if len(color_list) == 3:  # 多次判断(multiple detection)
                    # 取平均值(get average value)
                    color = np.mean(np.array(color_list))
                    color_list = []
                    start_pick_up = True
                    if color == 1:
                        detect_color = 'red'
                        draw_color = range_rgb["red"]
                    elif color == 2:
                        detect_color = 'green'
                        draw_color = range_rgb["green"]
                    elif color == 3:
                        detect_color = 'blue'
                        draw_color = range_rgb["blue"]
                    else:
                        start_pick_up = False
                        detect_color = 'None'
                        draw_color = range_rgb["black"]
        else:
            if not start_pick_up:
                detect_color = 'None'
                draw_color = range_rgb["black"]
        
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2) # 把检测到的颜色打印在画面上(print the ultrasonic distance measurement on the screen)
    
    return img


#关闭前处理(process program before closing)
def manual_stop(signum, frame):
    global __isRunning
    
    print('关闭中...')
    __isRunning = False
    initMove()  # 舵机回到初始位置(servo returns to the initial position)


if __name__ == '__main__':
    import HiwonderSDK.ros_robot_controller_sdk as rrc
    board = rrc.Board()
    init()
    start()
    camera = Camera.Camera()
    camera.camera_open(correction=True) # 开启畸变矫正,默认不开启(enable distortion correction, disabled by default)
    signal.signal(signal.SIGINT, manual_stop)
    while __isRunning:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240)) # 画面缩放到320*240(resize the image to 320*240)
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()

