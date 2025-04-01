#import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
import time
import math
import sys, select, termios, tty
import pigpio
import subprocess  
import os  



open_io="sudo pigpiod"
 
close_io="sudo killall pigpiod"
os.system(close_io)
os.system(open_io)
os.system("sudo systemctl stop network-rc.service")
time.sleep(2)
#######################################调整参数############################


kp=0.14
kd=0.8
speed=3305    #本次电机的占空比范围为正转：2350-2450 快-慢  中间占空比为3000 反转：3350-3470 慢-快 频率为200

start_flag=0
stop_flag=1

vis_center=192
steer_center=91  #105 75  左到右   91为中线值

length=400
width=300
end_time=0
start_time=0
yellow_flag=0

old=192
############################################################################
dx_flag=0
bias=0
diff=0
last_diff=0
line=0
lie=0
start_list=0
find_left=0
find_right=0
white=0
black=255
flag0=0
flag1=0
areab=0
servo_pin = 12    # 舵机信号线接树莓派12 GPIO
moto_pin = 13     # 电调信号线接树莓派13 GPIO
mid_sum=0


def car_run (pwm):#电机控制函数
    pi.set_PWM_dutycycle(moto_pin,pwm)#本次电机的占空比范围为正转：2350-2450 快-慢  中间占空比为3000 反转：3350-3470 慢-快 频率为200


def smotor (angle):#舵机控制
    pi.set_PWM_dutycycle(servo_pin,angle)#110--90--70  左--右  800 


def erode_dilate_image(image):#膨胀处理#腐蚀处理
    kernel = cv.getStructuringElement(cv.MORPH_RECT,(3,3))
    image = cv.erode(image,kernel)
    kernel = cv.getStructuringElement(cv.MORPH_RECT,(4,4))
    image = cv.dilate(image,kernel)
    return image



def GaussianBlur(image):        # 高斯模糊
    image = cv.GaussianBlur(image, (5, 5), 0)  
    return image


# 图像输入
cap = cv.VideoCapture(0) 
cap.set(cv.CAP_PROP_FRAME_WIDTH,length)
cap.set(cv.CAP_PROP_FRAME_HEIGHT,width)


#time.sleep(10)

pi = pigpio.pi() # 创建实例初始化


pi.set_mode(servo_pin, pigpio.OUTPUT)#舵机初始化
pi.set_PWM_frequency(servo_pin,50)  # 修改17端口的PWM频率
pi.set_PWM_range(servo_pin,1000)      # 设置PWM占空比的切分
pi.set_PWM_dutycycle(servo_pin,steer_center)#110---70  左--右  800
print("舵机")


pi.set_mode(moto_pin, pigpio.OUTPUT)#电机初始化
pi.set_PWM_frequency(moto_pin,200)  # 修改17端口的PWM频率
pi.set_PWM_range(moto_pin,10000)      # 设置PWM占空比的切分
pi.set_PWM_dutycycle(moto_pin,3000)
print("电机")
#time.sleep(7)


road_width=np.zeros(500)
centerline=np.zeros(500)
num_list=np.zeros((500,500))
rightline=np.zeros(500)
leftline=np.zeros(500)

while True:
    ret, frame = cap.read()  # 获取ret布尔值和frame画面帧
    kknd=frame.copy()
    gs_frame = cv.GaussianBlur(frame, (5, 5), 0)  # 高斯模糊
    hsv = cv.cvtColor(gs_frame, cv.COLOR_BGR2HSV)  # 转化成HSV图像,便于二值化处理和扫描
    erode_hsv = cv.erode(hsv, None, iterations=3)  # 腐蚀 粗的变细
    gy_frame = cv.cvtColor(gs_frame,cv.COLOR_BGR2GRAY)#将捕获的一帧图像灰度化处理
    rs_frame=cv.resize(gy_frame,(length,width)) #改为600*600大小的

    ret,img1=cv.threshold(rs_frame,160,255,cv.THRESH_BINARY)  #二值化    145
    kernel = cv.getStructuringElement(cv.MORPH_RECT,(2,2))       #腐蚀处理
    img1 = cv.erode(img1,kernel)
    kernel = cv.getStructuringElement(cv.MORPH_RECT,(5,5))       #膨胀处理
    img1 = cv.dilate(img1,kernel)
    mask=np.zeros_like(img1)   #变换为numpy格式的图片
    mask=cv.fillPoly(mask,np.array([[[50,120],[0,210],[400,210],[350,120]]]),color=255)   #对感兴趣区域制作掩膜
    masked_edge_img=cv.bitwise_and(img1,mask)   #与运算
    img2=masked_edge_img
    
    mid_sum=0
    for line in range (200,150,-1):#PS:默认顺序为img[行,列]扫线即可和plt逻辑相反不用考虑   
        r_lost=0
        l_lost=0
        for lie in range(int(old),0,-1):
            if((img2[line,lie-1]==white)and(img2[line,lie]==white)and(img2[line,lie+1]!=white)and(img2[line,lie+2]!=white)):
                leftline[line]=lie
                #print(leftline[line])
                l_lost=1
                break
        if(l_lost==0):      #左边丢线处理
            leftline[line]=0
        for lie in range (int(old),384,2):
            if((img2[line,lie-2]!=white)and(img2[line,lie-1]!=white)and(img2[line,lie]==white)and(img2[line,lie+1]==white)):
                rightline[line]=lie
                #print(rightline[line])
                r_lost=1
                break
        if(r_lost==0):      #右边丢线处理
            rightline[line]=384
        # road_width[line]=abs(leftline[line],rightline[line])
        centerline[line]=(rightline[line]+leftline[line])/2
        mid_sum+=centerline[line]
        old = centerline[line]-1
        print(centerline[line])
    old = centerline[195];    #//初次扫线完毕，将old重新赋值 
    mid_final=mid_sum/50
    #PID控制器
    diff=mid_final-vis_center
    bias=kp*diff+kd*(diff-last_diff)
    last_diff = diff
    if bias>=5:
        bias=5
    elif bias<-5:
        bias =-5
    last_bias=bias

    #舵机偏转
    pi.set_PWM_dutycycle(servo_pin,steer_center-bias)#110---70  左--右  800       舵机控制\
    car_run(speed)

    #调试显示图像
    #cv.line(frame, (179,0), (179,216), (0, 255, 255), 5,4)
   # cv.imshow('yuantu',frame)
    #cv.imshow('kankan',kknd)
    #cv.imshow('resize',rs_frame)
    #cv.imshow('erzhihu',img1)
    cv.imshow('erzhihua',img2)
    #cv.waitKey(1)
    #print(stop_flag)
    #print(start_flag) 
    #print(areab)
    
