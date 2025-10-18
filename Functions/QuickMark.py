#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/TurboPi/')
import cv2
import time
import signal
import Camera
import threading
import numpy as np
import yaml_handle
from  pyzbar.pyzbar import  decode
import HiwonderSDK.mecanum as mecanum

# 二维码识别(QR code recognition)
board = None
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

car = mecanum.MecanumChassis()

servo1 = 1500
servo2 = 1500

car_stop = False
size = (640, 480)
results_num = None
__isRunning = False
results_lock = False

servo_data = None
def load_config():
    global servo_data
    
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

# 初始位置(initial position)
def initMove():
    board.pwm_servo_set_position(1, [[1, servo1], [2, servo2]])

# 变量重置(variable reset)
def reset(): 
    global car_stop
    global servo1, servo2
    global results_num, results_lock
    
    car_stop = False
    results_num = None
    results_lock = False
    servo1 = servo_data['servo1'] - 350
    servo2 = servo_data['servo2']

# app初始化调用(app initialization call)
def init():
    print("QuickMark Init")
    load_config()
    reset()
    initMove()

# app开始玩法调用(app start program call)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("QuickMark Start")

# app停止玩法调用(app stop program call)
def stop():
    global car_stop
    global __isRunning
    car_stop = True
    __isRunning = False
    print("QuickMark Stop")

# app退出玩法调用(app exit program call)
def exit():
    global car_stop
    global __isRunning
    car_stop = True
    __isRunning = False
    print("QuickMark Exit")

# 机器人移动逻辑处理(robot movement logic processing)
def move():
    global __isRunning, car_stop
    global results_num, results_lock

    while True:
        if __isRunning:
            if results_lock and results_num:
                if results_num == 1: # 结果为1，机器人向前移动一段距离(result is 1, the robot moves forward a certain distance)
                    car.set_velocity(45,90,0) # 控制机器人移动函数,线速度45(0~100)，方向角90(0~360)，偏航角速度0(-2~2)(robot motion control function, linear velocity 45(0~100), orientation angle 90(0~360), yaw rate 0(-2~2))
                    time.sleep(1)
                    car.set_velocity(0,90,0) 
                elif results_num == 2: # 结果为2，机器人向后移动一段距离(result is 2, the robot moves backward a certain distance)
                    car.set_velocity(45,270,0)
                    time.sleep(1)
                    car.set_velocity(0,90,0)
                elif results_num == 3: # 结果为3，机器人向右移动一段距离(result is 3, the robot moves to the right a certain distance)
                    car.set_velocity(45,0,0)
                    time.sleep(1)
                    car.set_velocity(0,90,0)
                elif results_num == 4: # 结果为4，机器人向左移动一段距离(result is 4, the robot moves to the left a certain distance)
                    car.set_velocity(45,180,0)
                    time.sleep(1)
                    car.set_velocity(0,90,0)
                
                results_lock = False
                
            else:
                if car_stop:
                    car.set_velocity(0,90,0)
                    car_stop = False
                    time.sleep(0.5)               
                time.sleep(0.01)
        else:
            if car_stop:
                car.set_velocity(0,90,0)
                car_stop = False
                time.sleep(0.5)               
            time.sleep(0.01)

# 运行子线程(run a sub-threat)
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# 机器人图像处理(robot images processing)
results_list = []
def run(img):
    global __isRunning
    global results_num
    global results_lock
    global results_list
    
    if not __isRunning:  # 检测是否开启玩法，没有开启则返回原图像(check if the program is enabled, return the original image if not enabled)
        return img
    
    if results_lock: # 上一个动作还在执行,返回原图像(the previous action is still executing, return to the original image)
        return img
    
    i = 0
    results_num = 0
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    for barcode in decode(img_copy):
        i += 1
        data = barcode.data.decode('utf-8')
        pts = np.array([barcode.polygon],np.int32)
        cv2.polylines(img, [pts], True, (0,255,0), 3)
        cv2.putText(img, 'Data:'+ data, (20,450), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,255,255), 3)
        if i == 1:
            results_list.append(int(data))
            if len(results_list) == 5:
                results_num = np.mean(np.array(results_list))
                results_lock = True
                results_list = []

    return img


#关闭前处理(process program before closing)
def manualcar_stop(signum, frame):
    global __isRunning
    
    print('关闭中...')
    __isRunning = False
    car.set_velocity(0,90,0)


if __name__ == '__main__':
    import HiwonderSDK.ros_robot_controller_sdk as rrc
    board = rrc.Board()
    init()
    start()
    camera = Camera.Camera()
    camera.camera_open(correction=True) # 开启畸变矫正,默认不开启(enable distortion correction, disabled by default)
    signal.signal(signal.SIGINT, manualcar_stop)
    while __isRunning:
        img = camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)  
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    camera.camera_close()
    cv2.destroyAllWindows()

