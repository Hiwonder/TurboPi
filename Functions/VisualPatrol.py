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
import HiwonderSDK.PID as PID
import HiwonderSDK.Misc as Misc
import HiwonderSDK.mecanum as mecanum

# 视觉巡线(visual patrol)
board = None
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


servo1 = 1500
servo2 = 1500
img_centerx = 320
line_centerx = -1
size = (640, 480)
target_color = ()
__isRunning = False
swerve_pid = PID.PID(P=0.001, I=0.00001, D=0.000001)

car = mecanum.MecanumChassis()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}


lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

# 初始位置(initial position)
def initMove():
    car_stop()
    board.pwm_servo_set_position(1, [[1, servo1], [2, servo2]])

# 变量重置(variable reset)
def reset():
    global line_centerx
    global target_color
    global servo1, servo2
    
    line_centerx = -1
    target_color = ()
    servo1 = servo_data['servo1']+350
    servo2 = servo_data['servo2']
    
# app初始化调用(app initialization call)
def init():
    print("VisualPatrol Init")
    load_config()
    reset()
    initMove()

# app开始玩法调用(app start program call)
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("VisualPatrol Start")

# app停止玩法调用(app stop program call)
def stop():
    global __isRunning
    __isRunning = False
    car_stop()
    print("VisualPatrol Stop")

# app退出玩法调用(app exit program call)
def exit():
    global __isRunning
    __isRunning = False
    car_stop()
    print("VisualPatrol Exit")
    
# 设置检测颜色(set detected color)
def setTargetColor(color):
    global target_color

    print("COLOR", color)
    target_color = color
    return (True, ())

# 关闭电机(close motor)
def car_stop():
    car.set_velocity(0,90,0)  # 关闭所有电机(close all motors)
    
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
            if contour_area_temp >= 5:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only the maximal contour with an area greater than 300 is considered valid to filter out interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the maximal contour)

# 机器人移动逻辑处理(robot movement logic processing)
car_en = False
def move():
    global line_centerx, car_en

    while True:
        if __isRunning:
            if line_centerx > 0:
                # 偏差比较小，不进行处理(the deviation is small, no processing needed)
                if abs(line_centerx-img_centerx) < 10:
                    line_centerx = img_centerx
                swerve_pid.SetPoint = img_centerx
                swerve_pid.update(line_centerx) 
                angle = -swerve_pid.output    # 获取PID输出值(get PID output value)
                
                car.set_velocity(50, 90, angle)
                car_en = True              
        else:
            if car_en:
                car_en = False
                car_stop()
            time.sleep(0.01)
 
# 运行子线程(run a sub-thread)
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

roi = [ # [ROI, weight]
        (240, 280,  0, 640, 0.1), 
        (340, 380,  0, 640, 0.3), 
        (430, 460,  0, 640, 0.6)
       ]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]

roi_h_list = [roi_h1, roi_h2, roi_h3]

# 机器人图像处理(robot images processing)
def run(img):
    global line_centerx
    global target_color
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning or target_color == ():
        return img
     
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)         
    centroid_x_sum = 0
    weight_sum = 0
    center_ = []
    n = 0

    #将图像分割成上中下三个部分，这样处理速度会更快，更精确(segmenting the image into three parts (top, middle, and bottom) for faster and more accurate processing)
    for r in roi:
        roi_h = roi_h_list[n]
        n += 1       
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to the LAB space)
        area_max = 0
        areaMaxContour = 0
        for i in target_color:
            if i in lab_data:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation on the original image and mask)
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(corrosion)
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilation)

        cnts = cv2.findContours(dilated , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]#找出所有轮廓(find all contours)
        cnt_large, area = getAreaMaxContour(cnts)#找到最大面积的轮廓(find the contour with the largest area)
        if cnt_large is not None:#如果轮廓不为空(if contour is not none)
            rect = cv2.minAreaRect(cnt_large)#最小外接矩形(minimum bounding rectangle)
            box = np.intp(cv2.boxPoints(rect))#最小外接矩形的四个顶点(the four vertices of the minimum bounding rectangle)
            for i in range(4):
                box[i, 1] = box[i, 1] + (n - 1)*roi_h + roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
            for i in range(4):                
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))

            cv2.drawContours(img, [box], -1, (0,0,255,255), 2)#画出四个点组成的矩形(draw a rectangle formed by four points)
        
            #获取矩形的对角点(get the diagonal points of the rectangle)
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]            
            center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2#中心点(central point)
            cv2.circle(img, (int(center_x), int(center_y)), 5, (0,0,255), -1)#画出中心点(draw central point)
            center_.append([center_x, center_y])                        
            #按权重不同对上中下三个中心点进行求和(sum the upper, middle, and lower center points based on different weights)
            centroid_x_sum += center_x * r[4]
            weight_sum += r[4]
    if weight_sum != 0:
        #求最终得到的中心点(calculate the final center point)
        cv2.circle(img, (line_centerx, int(center_y)), 10, (0,255,255), -1)#画出中心点(draw central point)
        line_centerx = int(centroid_x_sum / weight_sum)  
    else:
        line_centerx = -1
    return img


#关闭前处理(process program before closing)
def manual_stop(signum, frame):
    global __isRunning
    
    print('关闭中...')
    __isRunning = False
    car_stop()  # 关闭所有电机(close all motors)

if __name__ == '__main__':
    import HiwonderSDK.ros_robot_controller_sdk as rrc
    board = rrc.Board()
    init()
    start()
    __isRunning = True
    target_color = ('black',)
    camera = Camera.Camera()
    camera.camera_open(correction=True) # 开启畸变矫正,默认不开启(enable distortion correction, disabled by default)
    signal.signal(signal.SIGINT, manual_stop)
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
