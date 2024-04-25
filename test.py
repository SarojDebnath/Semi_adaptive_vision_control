from simple_pid import PID
import clr
import time
import cv2
import torch
import numpy as np
#CREATE THE ROBOT OBJECT
clr.AddReference("IRA_UR_SocketCtrl_Prog")
import IRA_UR_SocketCtrl_Prog
robot=IRA_UR_SocketCtrl_Prog.SocketCtrl('192.168.1.111',30002,30020,100,1000)
print(robot.Start())
time.sleep(1)
#LOAD THE MODEL
model=torch.hub.load('C:/Users/sarojd.ADVA_MUNICH/Eva/vis_package/adaptive/', 'custom', path='C:/Users/sarojd.ADVA_MUNICH/Eva/vis_package/adaptive/adaptive.pt', source='local')
#Indexing function
def get_desired_order(points, threshold=10):
    sorted_points = sorted(points, key=lambda p: p[0], reverse=True)
    desired_order = []
    for current_point in sorted_points:
        close_neighbor = False
        for existing_point in desired_order:
          if abs(current_point[0] - existing_point[0]) <= threshold:
            close_neighbor = True
            if current_point[1]<existing_point[1]:
                index = desired_order.index(existing_point) 
                desired_order.insert(index, current_point)
                break
            else:
                index = desired_order.index(existing_point) 
                desired_order.insert(index+1, current_point)
                break 
    
        if not close_neighbor:
          desired_order.append(current_point)
        
    return desired_order

#PID SETTING

pidz=PID(0.2, 0.0, 0.0001, setpoint=0.5096)
pidx = PID(0.0002, 0.0, 0.0001, setpoint=-38)
pidy=PID(-0.0002, 0.0, 0.0001, setpoint=-5)
pidx.sample_time = 0.01
pidy.sample_time = 0.01


cap=cv2.VideoCapture(1)
cap.set(3,1280)
cap.set(4,720)

while True:
    ret,frame=cap.read()
    if ret==False:
        raise('No Camrera')
    results = model(frame)
    labels = results.names
    bboxes = results.xyxy[0].numpy()
    store_points_nut=[]
    store_points_bolt=[]
    #Get corrct order for nut and bolt
    for k, bbox in enumerate(bboxes):
        x1, y1, x2, y2 = bbox[:4].astype(int)
        if labels[int(bbox[5])]=='nut':
            store_points_nut.append((x1,y1))
        elif labels[int(bbox[5])]=='bolt':
            store_points_bolt.append((x1,y1))
    desired_order_nut = get_desired_order(store_points_nut)
    desired_order_bolt = get_desired_order(store_points_bolt)
    # Get the bounding box coordinates and labels of detected objects
    nutindex=1
    while True:
        try:
            current_point_nut=desired_order_nut[nutindex]
            break
        except IndexError:
            if len(desired_order_nut)>1:
                nutindex-=1
            else:
                nutindex=0
    boltindex=0
    while True:
        try:
            current_point_bolt=desired_order_bolt[boltindex]
            break
        except IndexError:
            if len(desired_order_bolt)>1:
                boltindex-=1
            else:
                boltindex=0
    if len(bboxes)>0:
        nutxy=0
        boltxy=0
        for i, bbox in enumerate(bboxes):
            x1, y1, x2, y2 = bbox[:4].astype(int)
            confidence = round(float(bbox[4]), 2)
            if confidence>=0.3 and labels[int(bbox[5])]=='nut' and (x1,y1)==(current_point_nut[0],current_point_nut[1]):
                label = f"{labels[int(bbox[5])]}: {confidence}:[{x1/2+x2/2},{y1/2+y2/2}]"
                nutxy=(x1,y1)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            elif confidence>=0.3 and labels[int(bbox[5])]=='bolt' and (x1,y1)==(current_point_bolt[0],current_point_bolt[1]):
                label = f"{labels[int(bbox[5])]}: {confidence}:[{x1/2+x2/2},{y1/2+y2/2}]"
                boltxy=(x1,y1)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    if isinstance(nutxy, tuple) and isinstance(boltxy, tuple):
        print('Difference between x1: ',nutxy[0]-boltxy[0])
        print('Difference between y1: ',nutxy[1]-boltxy[1])
        print('Robot X position: ',list(robot.ActualPoseCartesianRad)[0])
        vel_x=pidz(list(robot.ActualPoseCartesianRad)[0])
        vel_y=pidy(nutxy[1]-boltxy[1])
        vel_z=pidx(nutxy[0]-boltxy[0])
        robot.SpeedL([vel_x,vel_y,vel_z,0,0,0],False,True,0.3,0.5,0.0)
        if (-5.5<=(nutxy[1]-boltxy[1])<=-4.5) and (-38.5<=(nutxy[0]-boltxy[0])<=-37.5) and (0.5094<=list(robot.ActualPoseCartesianRad)[0]<=0.5098):
            #vel_x=pidz(list(robot.ActualPoseCartesianRad)[0])
            #vel_y=pidy(nutxy[1]-boltxy[1])
            #vel_z=pidx(nutxy[0]-boltxy[0])
            #robot.SpeedL([vel_x,vel_y,vel_z,0,0,0],False,True,0.3,0.5,0.0)
            robot.StopL(20.0)
            camera_pose=list(robot.ActualPoseCartesianRad)
            print(camera_pose)
            break
    cv2.imshow('objects',frame)
    if cv2.waitKey(2) & 0xff==27:
        break
cv2.destroyAllWindows()
cap.release()
#time.sleep(5)
camera_pose[0]+=0.1
robot.MoveL(camera_pose,True,True,True,0.3,0.5,0.0)
camera_pose[0]-=0.2
robot.MoveL(camera_pose,True,True,True,0.3,0.5,0.0)
robot.Stop()