import json
import ast
from Visual_Sensing import pid_marvin as pid
import cv2
import time
import numpy as np
from vision_v02 import camera as cam,classifier as c,label as l
import clr
import ctypes
import subprocess
from IRA_vision import objectdetection as od
from System.Collections.Generic import List

clr.AddReference("Winding/IRA_StepperCtrl_Prog")
clr.AddReference("Operation/IRA_UR_SocketCtrl_Prog")
import IRA_UR_SocketCtrl_Prog
import IRA_StepperCtrl_Prog
from IRA_StepperCtrl_Prog import PicStepper
file_path='C:/Users/AdminTE/Marvin/Deploy/Operation/data.json'

def read_data():
    try:
        with open(file_path, 'r') as file:
            config_data = json.load(file)
        return config_data
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Error reading config file: {e}")
        return {}

def write_data(config_data):
    with open(file_path, 'w') as file:
        json.dump(config_data, file, indent=4)

class operation:
    def __init__(self,ip,connect):
        if connect=='connect':
            self.ip=ip
            self.robot=IRA_UR_SocketCtrl_Prog.SocketCtrl(self.ip,30002,30020,100,1000)
            print(self.robot.Start())
            time.sleep(1)
            robotaddress=id(self.robot)
            self.address=f'{robotaddress}'
            print(self.address)
            
        else:
            ip=int(ip)
            self.robot=ctypes.cast(ip, ctypes.py_object).value
            self.address=ip
        self.detect=pid.detection(self.address)
        self.controller = PicStepper ("192.168.1.100",22,"pi","raspberry" ,"pi@raspberrypi:")
        self.controller.Connect()
        self.cmdon = List[str]()
        self.cmdoff = List[str]()
        self.cmdon.Add('def x():\n  set_tool_digital_out(0, True)\n  set_tool_digital_out(1, False)\n end\n')
        self.cmdoff.Add('def y():\n  set_tool_digital_out(0, False)\n  set_tool_digital_out(1, True)\n end\n')
    def robotaddress(self):
        return self.address
    def movej(self,pos=[],rel=False,a=0.3,v=0.5):
        commandpop = 'echo y | plink root@192.168.1.251 -pw easybot "{ echo \\"unlock protective stop\\"; echo \\"quit\\"; } | nc 127.0.0.1 29999"'
        emergency=False
        target=[]
        initjoint=list(self.robot.ActualJointAnglesDeg)
        if rel==True:
            currentpos=list(self.robot.ActualJointAnglesDeg)
            for i in range(6):
                target.append(currentpos[i]+pos[i])
        else:
            target=pos
        self.robot.MoveJ(target,True,False,False,a,v,0.0)
        emergency=self.robot.IsEmergencyStopped or self.robot.IsProtectiveStopped
        if emergency==True:
            self.robot.Stop()
            time.sleep(5)
            process = subprocess.Popen(commandpop, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            time.sleep(1)
            stdout, stderr = process.communicate()
            print(stdout.decode())
            raise Exception('Robot_stopped')
        else:
            return 'done'
    def movel(self,pos=[],rel=False,a=0.3,v=0.5):
        commandpop = 'echo y | plink root@192.168.1.251 -pw easybot "{ echo \\"unlock protective stop\\"; echo \\"quit\\"; } | nc 127.0.0.1 29999"'
        emergency=False
        target=[]
        initjoint=list(self.robot.ActualJointAnglesDeg)
        if rel==True:
            currentpos=list(self.robot.ActualPoseCartesianRad)
            for i in range(6):
                target.append(currentpos[i]+pos[i])
        else:
            target=pos
        self.robot.MoveL(target,True,True,True,a,v,0.0)
        emergency=self.robot.IsEmergencyStopped or self.robot.IsProtectiveStopped
        if emergency==True:
            time.sleep(5)
            process = subprocess.Popen(commandpop, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            time.sleep(1)
            stdout, stderr = process.communicate()
            print(stdout.decode())
            angle=list(self.robot.ActualJointAnglesDeg)
            if angle[5]>=358 or angle[5]<=-358:
                
                angle[5]=0
                self.movej(angle,False,0.3,0.5)
                self.movel(target,False,a,v)
            else:
                self.robot.Stop()
                raise Exception('Robot_stopped')
        else:
            return 'done'
    def disconnectrobot(self):
        self.robot.Stop()
    
    def gripper(self,prompt):
        if prompt=='ON':
            self.robot.AdvancedModeEnable()
            print('Opening')
            self.robot.SendCommandInt(self.cmdon)
            time.sleep(0.5)
            self.robot.AdvancedModeExit()
        elif prompt=='OFF':
            self.robot.AdvancedModeEnable()
            print('Closing Gripper')
            self.robot.SendCommandInt(self.cmdoff)
            time.sleep(0.5)
            self.robot.AdvancedModeExit()
        elif prompt=='STATUS':
            if list(self.robot.ToolOutputState)[0]==True and list(self.robot.ToolOutputState)[1]==False:
                print('Open')
                value='OPENED'
            elif list(self.robot.ToolOutputState)[1]==True and list(self.robot.ToolOutputState)[0]==False:
                print('Close')
                value='CLOSED'
            else: 
                print('Normal')
                value='Normal'
            return value
        
    def invertrobot(self):
        box2=[0.1872569792118477, -0.5658526586016778, 0.24245908591377802, 1.1513610150746634, 1.2502067660550873, -1.2081104427094431]
        self.movel(box2,False,0.3,0.8)
        reverseposedeg=[95.32421692477314, -85.13785215767618, 151.57938660932385, -67.82233308803785, 97.72133328569166, 42.70467861561586]
        self.robot.MoveJ(reverseposedeg,True,False,False,0.2,0.9,0.0)
    def normalrobot(self):
        reverseposerad=[0.18695108358069282, -0.5613902695374441, 0.15448163189625777, 1.1771719683783126, -1.18657826224989, 1.1599749970035191]
        self.movel(reverseposerad,False,0.3,0.5)
        box2=[89.95628929379008, -55.649411147342924, 153.1389783440262, -277.95780742364207, 272.7402179367411, 42.8559775510154]
        self.robot.MoveJ(box2,True,False,False,0.2,0.9,0.0)
    def readlabel(self,box):
        data=read_data()
        labelpose=data[f'camerapose{box}']
        labelpose = ast.literal_eval(labelpose)
        labelpose[0]-=0.12
        labelpose[1]-=0.025
        labelpose[2]-=0.02
        self.movel(labelpose,False,0.2,0.8)
        while True:
            cam.image(0,[1280,120],'rgb',1)
            if box<=1:
                image=cv2.imread('captured.jpg')
                image=cv2.rotate(image, cv2.ROTATE_180)
                cv2.imwrite('captured.jpg',image)
            labelinfo=l.readlabel('captured.jpg',['40','LBADVA','10'])
            try:
                if len(labelinfo[1][0][1])>=17:
                    break
            except IndexError:
                break
                pass
        #text=f'{labelinfo[0][0][1]} and {labelinfo[1][0][1]}'
        black_image=np.zeros((960,1280,3),dtype=np.uint8)
        try:
            cv2.putText(black_image,'Serial Numbers:',(30,120),cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),5)
            cv2.putText(black_image,labelinfo[0][0][1],(30,320),cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),5)
            cv2.putText(black_image,labelinfo[1][0][1],(30,520),cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),5)
            cv2.putText(black_image,labelinfo[2][0][1],(30,720),cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),5)
            labelinfodata=[labelinfo[0][0][1],labelinfo[1][0][1]]
        except IndexError:
            cv2.putText(black_image,'111',(30,220),cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),5)
            labelinfodata=['111','LBADVA']
        cv2.imshow('label',black_image)
        cv2.waitKey(500)
        cv2.destroyAllWindows()
        if len(labelinfodata[1])>17:
            labelinfodata[1]=labelinfodata[1][:17]
        if 'i' or 'I' in labelinfodata[1]:
            d=labelinfodata[1].replace('i','1')
            d=labelinfodata[1].replace('I','1')
            labelinfodata=[labelinfodata[0],d]
        data['labelinfo']=labelinfodata
        write_data(data)
        return labelinfodata
    def detectandread(self,startpose,weights,boxindex,z=True):
        self.detect.xyz(startpose,weights,boxindex,z=True)
        info=self.readlabel(startpose)
        return info
        
    def readtrolley (self):
        #designed for one tower
        data=read_data()
        trolleypose=data['trolleypose']
        self.movel(trolleypose,False,0.3,0.5)
        cam.image(0,[640,480],'rgb',1)
        trolleyinfo=l.readlabel('captured.jpg',['00'])
        #text=f'{labelinfo[0][0][1]} and {labelinfo[1][0][1]}'
        black_image=np.zeros((960,1280,3),dtype=np.uint8)
        try:
            cv2.putText(black_image,trolleyinfo[0][0][1],(30,320),cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),5)
        except IndexError:
            cv2.putText(black_image,'Nothing Detected',(30,220),cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),5)
        cv2.imshow('trolley',black_image)
        cv2.waitKey(3000)
        cv2.destroyAllWindows()
        return trolleyinfo[0][0][1]
    def openbox(self,startpose,boxindex):
        boxindex=1-boxindex
        data=read_data()
        c_pose=[]
        directory=data['directory']
        camera_pose=data[f'camerapose{startpose}']
        camera_pose = ast.literal_eval(camera_pose)
        print('This is camera pose:',camera_pose)
        tooloffset=data['tooloffset']
        print('This is tooloffset:',tooloffset)
        for i in range(6):
            c_pose.append(camera_pose[i]+tooloffset[i])
        print('C_opening Pose according to robot base:\n',c_pose)
        status=self.gripper('STATUS')
        if status=='OPENED':
            self.tooldocker(1)#1 == grab
        if startpose<=1 and list(self.robot.ActualJointAnglesDeg)[3]<-90.0:
            self.invertrobot()
        self.movel(c_pose,False,0.3,0.7)
        c_pose_jointDeg=list(self.robot.ActualJointAnglesDeg)
        if startpose<=1:
            rot=list(self.robot.ActualJointAnglesDeg)
            if rot[5]<180:
                rot[5]+=180
            else:
                rot[5]-=180
            self.robot.MoveJ(rot,True,False,False,0.3,0.5,0.0)
            self.movel([-0.055727497375393976,0,-0.08442447998649394,0,0,0],True,0.5,0.7)
            c_pose=list(self.robot.ActualPoseCartesianRad)
        time.sleep(1)
        self.movel([0,-0.05,0,0,0,0],True,0.2,0.5)
        self.robot.MoveUntilContact([0,-0.05,0,0,0,0],[0,-1,0,0,0,0],True,True, 0.15,3)
        boxtouch=list(self.robot.ActualPoseCartesianRad)
        data['boxtouch']=f'{boxtouch}'
        self.close(startpose)
        #self.movel(c_pose,False,0.2,0.3)
        #Check the opening of the box
        while True:
            if startpose<=1:
                self.robot.MoveJ(c_pose_jointDeg,True,False,False,0.2,0.3,0.0)
            self.movel(camera_pose,False,0.2,0.3)
            self.movel([-0.01,0,0.02,0,0,0],True,0.2,0.3)
            snap=list(self.robot.ActualPoseCartesianRad)
            cam.image(0,[320,120],'rgb',1)
            time.sleep(1)
            self.movel([0,0.02,0,0,0,0],True,0.2,0.3)
            if startpose<=1:
                image=cv2.imread('captured.jpg')
                image=np.rot90(image,2)
                cv2.imwrite(image,'captured.jpg')
            m=c.classify('captured.jpg',f'{directory}open_partiallyopen_close.h5')
            status=cv2.imread('captured.jpg')
            if m==0:
                cv2.putText(status,'Closed',(30,120),cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),5)
                cv2.imshow('Box',status)
                cv2.waitKey(2000)
                cv2.destroyAllWindows()
                self.movel(camera_pose,False,0.2,0.3)
                camera_pose=self.detect.xy(startpose,'copening',1-boxindex,True)####Here XYZ can alo be called with some difficulities
                c_pose=[]
                for i in range(6):
                    c_pose.append(camera_pose[i]+tooloffset[i])
                self.movel(c_pose,False,0.5,0.5)
                
                time.sleep(1)
                self.robot.MoveUntilContact([0,-0.025,0,0,0,0],[0,-1,0,0,0,0],True,True, 0.15,3)
                boxtouch=list(self.robot.ActualPoseCartesianRad)
                self.close(startpose)
                self.movel(c_pose,False,0.2,0.3)
                
            elif m==1:
                cv2.putText(status,'Open',(30,120),cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),5)
                cv2.imshow('Box',status)
                cv2.waitKey(2000)
                cv2.destroyAllWindows()
                break
            else:
                cv2.putText(status,'Partially Open',(30,120),cv2.FONT_HERSHEY_SIMPLEX,3,(0,255,0),5)
                cv2.imshow('Box',status)
                cv2.waitKey(2000)
                cv2.destroyAllWindows()
                while True:
                    localboxtouch=boxtouch
                    self.partially_open(localboxtouch)
                    self.movel(c_pose,False,0.2,0.3)
                    self.movel(camera_pose,False,0.2,0.3)
                    
                    ###Here it can be corrected after testing. It's written on 20.11.2023
                    break 
                break
        data[f'c_pose{startpose}']=f'{c_pose}'
        write_data(data)
        return c_pose
    def insert(self,level):
        data=read_data()
        
        camera_ethernet=data[f'camerapose{level}']
        camera_ethernet = ast.literal_eval(camera_ethernet)
        plugoffset=data['plugoffset']
        
        self.movel(camera_ethernet,False,0.3,0.5)
        
        q=list(self.robot.ActualJointAnglesRad)
        if level<=1:
            q[5]-=1.5707963267948966
        else:
            q[5]+=1.6262505054473877
        self.robot.MoveJ(q,True,False,True,0.3,0.7,0.0)
        insertion=list(self.robot.ActualPoseCartesianRad)
        print('After Rotation: ',insertion)
        #Add offset
        ethernet_insertion_pose=[]
        print('After adding offset: ')
        for i in range(6):
            ethernet_insertion_pose.append(insertion[i]+plugoffset[i])
        if level<=1:
            ethernet_insertion_pose[0]-=0.06371721817734635
            ethernet_insertion_pose[2]-=0.0788381257402728
            print('Ethernet insertion Pose: ',ethernet_insertion_pose)
        self.plugdocker(1,level)
        ethernet_insertion_pose[1]-=0.06
        rotate=ethernet_insertion_pose.copy()
        rotate[0]-=0.05
        print('Ethernet insertion Pose after copying: ',ethernet_insertion_pose)
        
        self.movel(rotate,False,0.105,0.105)
        q2=list(self.robot.ActualJointAnglesDeg)
        if level<=1:
            q2[4]-=10
        else:
            q2[4]+=10
        self.robot.MoveJ(q2,True,False,False,0.3,0.5,0.0)
        
        self.movel(ethernet_insertion_pose,False,0.105,0.105)
        time.sleep(1)
        self.movel([0.05,0,0,0,0,0],True,0.105,0.105)
        unwinding_offset=list(self.robot.ActualPoseCartesianRad)
        
        self.movel(ethernet_insertion_pose,False,0.3,0.5)
        #rotate by degrees
        q1=list(self.robot.ActualJointAnglesDeg)
        q1[5]-=2
        if level<=1:
            q1[5]+=4
        self.robot.MoveJ(q1,True,False,False,0.3,0.5,0.0)
        print('Starts insertion')
        self.robot.SetTcpOffset([0,0,0.238,0,0,0.7854],True)
        time.sleep(1.5)
        
        self.movel([0,0,0,-0.0020931577417542258,-0.062017182530304076,-0.06287342249711914],True,0.3,0.5)
        self.movel([-0.01,-0.03,0,0,0,0],True,0.105,0.105)
        self.movel([0.01,0,0,0,0,0],True,0.105,0.105)
        self.movel([0,-0.03,0,0,0,0],True,0.105,0.105)
        insertion_offset=list(self.robot.ActualPoseCartesianRad)
        
        self.robot.MoveUntilContact([0,-0.05,0,0,0,0],[0,-0.200,0,0,0,0],True,True, 0.15,3)
        
        inity=list(self.robot.ActualPoseCartesianRad)[1]
        init_force=list(self.robot.ActualTCPForce)
        F=data[f'force{level}']
        print('Target force is :',F)
        self.movel([0,-0.015,0,0,0,0],True,0.1,0.3)
        T=3.5#in MoveL23,27
        if level<=1:
            T=6
        t1=time.time()
        while True:
            t2=time.time()
            if t2-t1>T:
                F=F-20
                print(f'timeout {T} s')
            force=list(self.robot.ActualTCPForce)
            net_force=init_force[1]-force[1]
            print(net_force)
            if abs(net_force)>F :
                #self.robot.SpeedL([0,0,0,0,0,0],True,True,0.3,0.5,0.0)
                self.gripper('ON')
                print('Insertion Distance travelled:',abs(list(self.robot.ActualPoseCartesianRad)[1]-inity))
                self.robot.SetTcpOffset([0,0,0.180,0,0,0.7854],True)
                time.sleep(1.5)
                pluginserted=list(self.robot.ActualPoseCartesianRad)
                break
            else:
                
                self.movel([0,-0.001,0,0,0,0],True,0.1,0.5)
                #self.robot.SpeedL([0,-0.005,0,0,0,0],False,True,0.3,0.5,0.0)
                #p=list(self.robot.ActualPoseCartesianRad)
        #time.sleep(2)
        self.movel(ethernet_insertion_pose,False,0.03,0.05)
        
        #self.movel([-0.02,0,0,0,0,0],True,0.3,0.5)
        #retakeplugapproach=list(self.robot.ActualPoseCartesianRad)
        
        print('Here goes Programming')
        data[f'pluginserted{level}']=f'{pluginserted}'
        data[f'ethernet_insertion_pose{level}']=f'{ethernet_insertion_pose}'
        write_data(data)
        chan=8
        mot=1
        adress=data[f'picadress{level}']
        print('Controller is connected: ',self.controller.Connected)
        print('Winding')
        self.controller.I2CMotOnOff(chan,adress,True)
        self.controller.I2CMotWrite(chan,adress,mot,1)
        time.sleep(0.007)
        self.controller.I2CMotWrite(chan,adress,0,1)
        self.controller.I2CMotOnOff(chan,adress,False)
        
    def pulloutplug(self,box):
        if list(self.robot.ActualJointAnglesDeg)[3]>-90.0 and box>1:
            self.normalrobot()
        data=read_data()
        pluginserted=data[f'pluginserted{box}']
        if pluginserted=='noethernet':
            return 'noplug'
        else:
            pluginserted = ast.literal_eval(pluginserted)
            ethernet_insertion_pose=data[f'ethernet_insertion_pose{box}']
            ethernet_insertion_pose = ast.literal_eval(ethernet_insertion_pose)
            self.movel([0,0.04,0,0,0,0],True,0.105,0.105)
            offsetpose=ethernet_insertion_pose.copy()
            offsetpose[0]-=0.02
            if box<=1 and list(self.robot.ActualJointAnglesDeg)[3]<-90.0:
                self.invertrobot()
            
            self.movel(offsetpose,False,0.3,0.5)
            #detect the plug
            plugpx=od.click_and_localize(index=0,size=[640,480],rot=0,mode='rgb',T=1,directory='C:/Users/AdminTE/Marvin/Deploy/Operation/',object='plug',name='plug.jpg',modelpath='C:/Users/AdminTE/Marvin/Deploy/Visual_Sensing/plug.pt')
            #calculate the offset with the shift
            print(plugpx)
            if plugpx[-1]==0:
                print('No plugs are detected and hence considering default values')
                px=437.0
                py=404.5
            elif plugpx[-1]==2:
                px1=plugpx[0][0][1]
                py1=plugpx[0][0][2]
                px2=plugpx[0][1][1]
                py2=plugpx[0][1][2]
                if box % 2 == 0:
                    #consider the lower# Consider the one with greater X
                    if px1>=px2:
                        px=px1
                        py=py1
                    else:
                        px=px2
                        py=py2
                else:
                    #Odd box, consider the opposite of previous case
                    if px1<=px2:
                        px=px1
                        py=py1
                    else:
                        px=px2
                        py=py2
            else:
                px=plugpx[0][0][1]
                py=plugpx[0][0][2]
            #dfx=437 ,dfy= 404.5
            #457.5, 403.5
            pxoffset,pyoffset=px-437.0,py-404.5
            print('OFFSET',pxoffset,pyoffset)
            
            xmm=pyoffset*(0.005/15)-0.004
            zmm=0
            print('Offset',xmm)
            pluginserted[2]-=0.0035+zmm
            pluginserted[1]+=0.013
            pluginserted[0]+=xmm
            if box<=1:
                pluginserted[0]-=0.0015
            self.movel(pluginserted,False,0.1,0.1)
            self.gripper('OFF')
            time.sleep(1)
            self.movel([0,0.09,0,0,0,0],True,0.2,0.7)
            #self.movel(ethernet_insertion_pose,False,0.3,0.7)
            self.movel([0.04,0,0,0,0,0],True,0.2,0.7)
            return 'done'
            
    def closebox(self,box):
        data=read_data()
        shift=data[f'c_pose{box}']
        shift = ast.literal_eval(shift)
        camera_ethernet=data[f'camerapose{box}']
        camera_ethernet = ast.literal_eval(camera_ethernet)
        self.movel(camera_ethernet,False,0.7,0.7)
        shift[0]+=0.11
        self.tooldocker(1)
        if box<=1 and list(self.robot.ActualJointAnglesDeg)[3]<-90.0:
            self.invertrobot()
        self.movel(shift,False,0.2,0.3)
        self.robot.MoveUntilContact([0,-0.035,0,0,0,0],[0,-0.200,0,0,0,0],True,True, 0.15,3)
        self.movel([-0.09,0,0,0,0,0],True,0.2,0.3)
        self.movel([0,-0.004,0,0,0,0],True,0.2,0.3)
        self.movel([0,0.05,0,0,0,0],True,0.2,0.3)
        
    def plugdocker(self,state,level):
        data=read_data()
        plugpredocking=data[f'plugpredocking{level}']
        plugdocking=data[f'plugdocking{level}']
        if state==1:
            self.movel(plugpredocking,False,0.5,0.7)
            #Gripper open
            self.gripper('ON')
            print('before docking')
            self.movel(plugdocking,False,0.3,0.2)
            
            print('after_docking')
            self.gripper('OFF')
            self.movel(plugpredocking,False,0.3,0.4)
        if state==0:
            chan=8
            mot=1
            adress=data[f'picadress{level}']
            
            print('Controller is connected: ',self.controller.Connected)
            print('Winding')
            self.controller.I2CMotOnOff(chan,adress,True)
            self.movel([-0.05,0,0,0,0,0],True,0.2,0.3)
            shift=list(self.robot.ActualPoseCartesianRad)
            
            self.controller.I2CMotWrite(chan,adress,mot,1)
            self.movel(plugpredocking,False,0.16,0.096)
            
            self.controller.I2CMotWrite(chan,adress,0,1)
            self.controller.I2CMotOnOff(chan,adress,False)
            time.sleep(0.5)
            #time.sleep(time_winding)
            print('before docking')
            self.movel(plugdocking,False,0.3,0.2)
            
            print('after_docking')
            
            #Gripper open
            self.gripper('ON')
            self.movel(plugpredocking,False,0.3,0.2)
            
    def tooldocker(self,state):
        invert=False
        if list(self.robot.ActualJointAnglesDeg)[3]>-90.0:
            invert=True
        data=read_data()
        docking=data['docking']
        predocking=data['predocking']
        predockingjoint=data['predockingjoint']
        #state 1 is for grabbing the tool
        if state==1:
            if invert==True:
                self.normalrobot()
            self.movel([0,0.04,0,0,0,0],True,0.2,0.3)
            self.movej(predockingjoint,False,0.4,1)
            #self.movel(predocking,False,0.2,0.5)
            #Gripper open
            self.gripper('ON')
            print('before docking')
            self.movel(docking,False,0.3,0.1)
            print('after_docking')
            self.gripper('OFF')
            self.movel(predocking,False,0.3,0.2)
            self.movel([0,0.05,0,0,0,0],True,0.2,0.3)
            
        if state==0:
            if invert==True:
                self.normalrobot()
            self.movel([0,0.03,0,0,0,0],True,0.2,0.3)
            #self.movej(predockingjoint,False,0.2,0.6)
            self.movel(predocking,False,0.2,0.5)
            print('before docking')
            self.movel(docking,False,0.3,0.2)
            print('after_docking')
            #Gripper open
            self.gripper('ON')
            self.movel(predocking,False,0.3,0.2)
    def close(self,box):
        init_pose=list(self.robot.ActualPoseCartesianRad)
        self.movel([0,0.002,0,0,0,0],True,0.3,0.1)
        #SET TCP
        self.robot.SetTcpOffset([0,-0.02259,0.270,0,0,0.7854],True)
        time.sleep(1)
        #ROTATE
        self.movel([0,0,0,-0.474371835439489,0.21906483144551903,0.46981748247766675],True,0.03,0.5)
    
        #DIAGONAL
        self.movel([0.012,0,0,0,0,0],True,0.3,0.7)
        self.movel([-0.01732,-0.01,0,0,0,0],True,0.3,0.7)
        self.movel([0,-0.005,0,0,0,0],True,0.3,0.7)
        #ROTATE BACK
        self.movel([0,0,0,0.474371835439489,-0.21906483144551903,-0.46981748247766675],True,0.3,0.7)
        
        #MoveuntilContact
        self.movel([-0.015,0,0,0,0,0],True,0.3,0.1)
        
        self.robot.MoveUntilContact([-0.01,0,0,0,0,0],[-0.01,0,0,0,0,0],True,True, 0.15,3)
        self.robot.MoveUntilContact([0,0.01,0,0,0,0],[0,0.01,0,0,0,0],True,True, 0.15,3)
        
    
        #1cm move
        self.movel([0,0.015,0,0,0,0],True,0.3,0.1)
        self.movel([0.03,0.035,0,0,0,0],True,0.3,0.1)
        self.movel([0.03,0,0,0,0,0],True,0.3,0.1)

        self.robot.SetTcpOffset([0,0,0.180,0,0,0.7854],True)
        time.sleep(2)
        #pull back
        self.movel([0,0.05,0,0,0,0],True,0.3,0.5)
        #initPOSE
        init_pose[1]+=0.05
        init_pose[0]-=0.065
        self.movel(init_pose,False,0.3,0.5)
        
        #MOVE FORWARD
        #self.movel([0,-0.15,0,0,0,0],True,0.3,0.5)
        #MOVE TO TOUCH
        self.robot.MoveUntilContact([0,-0.1,0,0,0,0],[0,-1,0,0,0,0],True,True, 0.15,3)
        self.movel([0,0.002,0,0,0,0],True,0.3,0.5)
        self.robot.MoveUntilContact([0.1,0,0,0,0,0],[0.02,0,0,0,0,0],True,True, 0.15,3)
        #rotate
        rotate=list(self.robot.ActualJointAnglesDeg)
        if box<=1:
            self.movej([0,0,0,0,-20,0],True,0.3,0.5)
        else:
            self.movej([0,0,0,0,20,0],True,0.3,0.5)
        
        self.movel([0,-0.03,0,0,0,0],True,0.3,0.5)
        self.movel(init_pose,False,0.3,0.5)
    def partially_open(self,boxtouch):
        boxtouch[1]+=0.01
        self.movel(boxtouch,False,0.2,0.3)
        self.movel([0.1,0,0,0,0,0],True,0.2,0.3)
        self.movel([0,-0.005,0,0,0,0],True,0.2,0.3)
        self.movel([0,0.15,0,0,0,0],True,0.2,0.3)
        
    def testswitch(self,prompt,box):
        #Grab positon of the switch from data according to the box number
        data=read_data()
        switchPos=data[f'switchright{box}'] #Extreme right pos
        print('Switch Pos: ',switchPos)
        if prompt=='PASSED':#PASSED
            switchPos[1]+=0.175
            switchPos[0]+=0.009
            self.movel(switchPos,False,0.2,0.3)
            self.movel([0,-0.08,0,0,0,0],True,0.2,0.3)
            self.robot.MoveUntilContact([0,-0.05,0,0,0,0],[0,-0.02,0,0,0,0],True,True, 0.15,3)
            #Operation
            self.movel([0.015,0,0,0,0,0],True,0.2,0.3)
            #Travel in Y
            self.movel([0,0.08,0,0,0,0],True,0.2,0.3)
            
        elif prompt=='FAILED':#FAIL
            switchPos[0]+=0.044
            switchPos[1]+=0.18
            self.movel(switchPos,False,0.2,0.3)
            self.movel([0,-0.08,0,0,0,0],True,0.2,0.3)
            self.robot.MoveUntilContact([0,-0.05,0,0,0,0],[0,-0.5,0,0,0,0],True,True, 0.15,3)
            self.movel([0,0.005,0,0,0,0],True,0.2,0.3)
            #Operation
            self.movel([-0.017,0,0,0,0,0],True,0.2,0.3)
            #Travel in Y
            self.movel([0,0.08,0,0,0,0],True,0.2,0.3)
        else:
            print(f'Prompt given is {prompt}')
        if box<=1 and list(self.robot.ActualJointAnglesDeg)[5]>=250:
            self.movej([0,0,0,0,0,-360],rel=True,a=0.3,v=0.5)
        return 'done'
    def disconnectwinding(self):
        self.controller.Disconnect()
        status=f"Winding Assembly Connection: {self.controller.Connected}"
        print(status)
        return status