
import cv2
import numpy as np

def analyze_samples(six_samples:list):
    temp_samples=[]
    max_value=-10000
    max_list=[]
    min_value=10000
    min_list=[]
    for i in range(len(six_samples)):
        current=six_samples[i]
        x1,x2,y1,y2=current[0],current[1],current[2],current[3]
        if x1>x2 and abs(y1-y2)<=10:
            if not 0 in current:
                temp_samples.append(current)
                if x1-x2>max_value:
                    if max_list!=current:
                        max_value=x1-x2
                        max_list=current
                if x1-x2<min_value:
                    if min_list!=current:
                        min_value=x1-x2
                        min_list=current
    if len(temp_samples)<=2:
        pass
    else:
        if max_list in temp_samples:
            temp_samples.remove(max_list)
        if min_list in temp_samples:
            temp_samples.remove(min_list)
    return temp_samples


def find_armor(thresh_img):
    try:
        contours,hierarchy=cv2.findContours(thresh_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        max_area=0
        max_arr=np.array([])
        for i in hierarchy:
            for j in i:
                index=j[2]
                if index!=-1:
                    new_contour=contours[index]
                    area=cv2.contourArea(new_contour)
                    if area>max_area:
                        max_area=area
                        max_arr=new_contour
        x_sum=0
        y_sum=0
        x_average=-1
        y_average=-1
        samples=0
        for i in max_arr:
            for j in i:
                x_sum+=j[0]
                y_sum+=j[1]
                samples+=1
        if samples!=0:
            x_average=x_sum/samples
            y_average=y_sum/samples
        new_image=np.zeros((thresh_img.shape[0],thresh_img.shape[1]))
        new_image=cv2.drawContours(new_image,max_arr,-1,(0,0,255),1)
        new_image=cv2.fillConvexPoly(new_image,max_arr,(255,0,0))
        return x_average,y_average,new_image
    except:
        return 0,0,0


def judge(final):
    if len(final)!=0:
        x1_average=0
        x2_average=0
        y1_average=0
        y2_average=0
        for i in final:
            x1_average+=i[0]
            x2_average+=i[1]
            y1_average+=i[2]
            y2_average+=i[3]
        x1_average=int(x1_average/len(final))
        x2_average=int(x2_average/len(final))
        y1_average=int(y1_average/len(final))
        y2_average=int(y2_average/len(final))
        if x1_average!=x2_average:
            return 1,x1_average,x2_average,y1_average,y2_average
        else:
            return 0,0,0,0,0
    else:
        return 0,0,0,0,0