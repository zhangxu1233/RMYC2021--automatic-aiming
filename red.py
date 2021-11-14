import cv2
import math
import numpy as np
import serial as ser
from loneliness import *
I=1
cap=cv2.VideoCapture(0)

cv2.namedWindow('left_frame',0)
cv2.namedWindow('right_frame',0)
cv2.namedWindow('l_frame',0)
cv2.namedWindow('r_frame',0)

def dispose(image_dispose): 
    image_dispose=cv2.flip(image_dispose,-1)
    R_min = 250
    G_min = 150
    B_min = 150
    R_max = 255
    G_max = 255
    B_max = 255
    lower_rgb = np.array([B_min, G_min, R_min])
    upper_rgb = np.array([B_max, G_max, R_max])
    image_dispose = cv2.inRange(image_dispose,lower_rgb,upper_rgb)
    
    kernel_dilation = np.ones((5,5),np.uint8)  
    image_dispose = cv2.dilate(image_dispose,kernel_dilation,iterations = 1)

    kernel_erode=np.ones((2,2),np.uint8)
    image_dispose=cv2.erode(image_dispose,kernel_erode,iterations=1)
    
    left_image=image_dispose[0:int(480*I),0:int(640*I)]
    right_image=image_dispose[0:int(480*I),int(640*I):int(1280*I)]
    return left_image,right_image

def dep(x1,x2,y1):
    f = (4.65*2.7*640*I)/(24.5*4/5)
    b = 60
    va_z = f * b / (x1-x2)
    va_x = ((x1-320*I) * va_z / f) - b / 2
    va_y = (240*I-y1) * va_z / f
    va_y=va_y+60
    va_z=va_z+60
    dis=math.sqrt(math.pow(va_x,2)+math.pow(va_y,2)+math.pow(va_z,2))
    gg=9.8*(math.pow(dis-150,2))/(math.pow(26,2))/2000
    yaw=math.atan(va_x/math.sqrt(math.pow(va_y,2)+math.pow(va_z,2)))/math.pi*180
    pitch=math.atan((va_y+gg)/math.sqrt(math.pow(va_x,2)+math.pow(va_z,2)))/math.pi*180
    return yaw,pitch,dis

def uart_send(shoot_yaw,shoot_pitch,shoot_dis):
    shoot_yaw=round(shoot_yaw,1)
    shoot_pitch=round(shoot_pitch,1)
    shoot_dis=round(shoot_dis,1)
    shoot_yaw=str(shoot_yaw)
    shoot_pitch=str(shoot_pitch)
    shoot_dis=str(shoot_dis)
    pos=shoot_yaw+" "+shoot_pitch+" "+shoot_dis+" "+""
    uart.write(str(pos).encode('utf-8'))


if __name__ == '__main__':
    uart = ser.Serial('/dev/ttyTHS1',9600,timeout=1)
    cap.set(3, int(1280*I)) 
    cap.set(4, int(480*I))
    ret, frame = cap.read()
    while True:
        new=[]
        for i in range(6):
            ret, frame = cap.read()
            if ret:
                left_frame,right_frame=dispose(frame)
                x1,y1,l_frame=find_armor(left_frame)
                x2,y2,r_frame=find_armor(right_frame)   
                
                try:
                    cv2.resizeWindow('left_frame',320,240)
                    cv2.resizeWindow('right_frame',320,240)
                    cv2.resizeWindow('l_frame',320,240)
                    cv2.resizeWindow('r_frame',320,240)
                    cv2.imshow('left_frame',left_frame)
                    cv2.imshow('right_frame',right_frame)
                    cv2.imshow('l_frame',l_frame)
                    cv2.imshow('r_frame',r_frame)
                except:
                    pass
                    
                new.append([x1,x2,y1,y2])
                if cv2.waitKey(1)==ord('q'):
                    break
            if i==5:
                final=analyze_samples(new)
                yyes,x1_average,x2_average,y1_average,y2_average=judge(final)
                if yyes==1:
                    yaw,pitch,dis=dep(x1_average,x2_average,y1_average)
                    yaw=int(float(yaw))
                    pitch=int(float(pitch))
                    dis=int(float(dis))
                    uart_send(yaw,pitch,dis)
                    print('yaw={}\t\tpitch={}\t\tdis={}'.format(yaw,pitch,dis))
                    print('x1={}\t\tx2={}\t\ty1={}\t\ty2={}'.format(x1_average,x2_average,y1_average,y2_average))
                else:
                    print("judge false")
                    uart_send(0,0,0)
                print('------------------------------------------------------')                 
cap.release()
cv2.destroyAllWindows()