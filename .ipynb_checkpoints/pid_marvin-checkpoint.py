from IRA_vision import objectdetection as od,servo_image as servo
import cv2
import time
import pyrealsense2 as rs
import numpy as np
from statistics import mode
import math
import json
import ctypes

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
        
class detection:
    def __init__(self,robotid):
        robotid=int(robotid)
        self.robot=ctypes.cast(robotid, ctypes.py_object).value
        data=read_data()
        directory=data['directory']
        self.copeningweight=data['copening']
        self.ethernetweight=data['ethernet']
        
    def invertrobot(self):
        box2=[0.1872569792118477, -0.5658526586016778, 0.24245908591377802, 1.1513610150746634, 1.2502067660550873, -1.2081104427094431]
        self.robot.MoveL(box2,True,True,True,0.5,0.5,0.0)
        reverseposedeg=[95.32421692477314, -85.13785215767618, 151.57938660932385, -67.82233308803785, 97.72133328569166, 42.70467861561586]
        self.robot.MoveJ(reverseposedeg,True,False,False,0.5,1,0.0)
    def normalrobot(self):
        reverseposerad=[0.18695108358069282, -0.5613902695374441, 0.15448163189625777, 1.1771719683783126, -1.18657826224989, 1.1599749970035191]
        self.robot.MoveL(reverseposerad,True,True,True,0.3,0.3,0.0)
        box2=[89.95628929379008, -55.649411147342924, 153.1389783440262, -277.95780742364207, 272.7402179367411, 42.8559775510154]
        self.robot.MoveJ(box2,True,False,False,0.5,1,0.0)
        
    def detect(self,level,weights):
        reverse=False
        if (level<=1 and weights=='ethernet') or (level==0 and weights=='copening'):
            reverse=True
        y=240
        object=0
        data=read_data()
        if weights=='ethernet':
            pose=data[f'camerapose{level}']
            pose = ast.literal_eval(pose)
        else:
            pose=data[f'snaplevel{level}']
        print('Taking snap from: ',pose)
        #if poseZ is below a certain point then do some extra movements(These will be for socket0 and socket1)
        if list(self.robot.ActualJointAnglesDeg)[3]<-90.0 and reverse==True:
            self.invertrobot()
        if list(self.robot.ActualJointAnglesDeg)[3]>-90.0 and reverse==False:
            self.normalrobot()
        self.robot.MoveL(pose,True,True,True,0.5,0.5,0.0)
        time.sleep(2)
        
        if weights=='ethernet':
            SIZE=[640,480]
        else:
            SIZE=[960,720]
        #write object detection code here and update position in a loop

        p=od.click_and_localize(index=0,size=SIZE,rot=0,mode='rgb',T=1,directory='C:/Users/AdminTE/Marvin/Deploy_new/Visual_Sensing',object=weights,name='captured1.jpg',modelpath=f'C:/Users/AdminTE/Marvin/Deploy_new/Visual_Sensing/{weights}.pt')
        print(p)
        object=p[-1]
        if object !=0:
            y=p[-2][-1]
        if object==1 and y<=175:
            object=[object,'wrongbox']
        else:
            object=[object,'correctbox']
        if object==0 and weights=='ethernet':
            data[f'pluginserted{level}']='noethernet'
            write_data(data)
        print('Number of Object detected:',object)
        return object
    def xy(self,box,weights,boxindex,z=True):
        print('box index: ',boxindex)
        boxindex=1-boxindex
        print('box index: ',boxindex)
        position=[0,0]
        distanceZ=0
        data=read_data()
        startpose=data[f'box{box}']#this could be a string level3box1
        #A string 'copening','ethernet'
        if weights=='ethernet':
            print('it considered the ethernet')
            startpose=data[f'camerapose{box}']
            startpose = ast.literal_eval(startpose)
            print(startpose)
        if box<=1 and list(self.robot.ActualJointAnglesDeg)[3]<-90.0:
            self.invertrobot()
        self.robot.MoveL(startpose,True,True,True,0.5,0.5,0.0)
        if weights=='ethernet':
            model1=self.modelethernet
        else:
            model1=self.modelcopening
        
        #RUN IRA
        print('Camera_pose from pid:',camera_pose)
        data[f'camerapose{box}']=f'{camera_pose}'
        write_data(data)
        return camera_pose
    def xyz(self,boxvariable,weights,boxindex,z=True):
        boxindex=1-boxindex
        #print(object)
        position=[0,0]
        distanceZ=0
        data=read_data()
        startpose=data[f'box{boxvariable}']#this could be a string level3box1
        print('StartPose:',startpose)
        if weights=='ethernet':
            print('it considered the ethernet')
            startpose=data[f'camerapose{boxvariable}']
            startpose = ast.literal_eval(startpose)
        if boxvariable<=1 and list(self.robot.ActualJointAnglesDeg)[3]<-90.0:
            self.invertrobot()
        self.robot.MoveL(startpose,True,True,True,0.7,0.7,0.0)
        print('weights:',weights)
        if weights=='copening':
            model=self.modelcopening
        else:
            model=self.modelethernet
        #IRA
        print('Camera_pose from pid rounded:',camera_pose)
        data[f'camerapose{boxvariable}']=f'{camera_pose}'
        write_data(data)
        return camera_pose
        #THE BELOW FUNCTION IS JUST FOR MAKING THE PROCESS RELIABLE
'''
    def depth(self,size,location,winsize):
        distanceZ=0
        pipeline = rs.pipeline()
        # Create a config and configure the pipeline to stream
        config = rs.config()
        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                print("There is a depth camera with color sensor")
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)
        config.enable_stream(rs.stream.depth, size[0], size[1], rs.format.z16, 30)
        config.enable_stream(rs.stream.color, size[0], size[1], rs.format.bgr8, 30)
        profile = pipeline.start(config)
    
    
        # Setup the 'High Accuracy'-mode
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        clipping_distance_in_meters = 1
        clipping_distance = clipping_distance_in_meters / depth_scale
        preset_range = depth_sensor.get_option_range(rs.option.visual_preset)
        for i in range(int(preset_range.max)):
            visulpreset = depth_sensor.get_option_value_description(rs.option.visual_preset,i)
            print('%02d: %s'%(i,visulpreset))
            if visulpreset == "High Accuracy":
                depth_sensor.set_option(rs.option.visual_preset, i)
        # enable higher laser-power for better detection
        depth_sensor.set_option(rs.option.laser_power, 180)
        # lower the depth unit for better accuracy and shorter distance covered
        depth_sensor.set_option(rs.option.depth_units, 0.0005)
        align_to = rs.stream.color
        align = rs.align(align_to)
        point_cloud = rs.pointcloud()
        # Skip first frames for auto-exposure to adjust
        for x in range(5):
            pipeline.wait_for_frames()
        t1=time.time()
        while True:
            # Stores next frameset
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            depth_data = np.asanyarray(depth_frame.get_data())
            if color_frame:
                aligned_frames = align.process(frames)
                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
                points=point_cloud.calculate(aligned_depth_frame)
                verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, size[0], 3)
                color_frame = aligned_frames.get_color_frame()
                aligned_depth_data = np.asanyarray(aligned_depth_frame.get_data())
                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue
        
                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                centerx=location[0]
                centery=location[1]
                distance=[]
                distanceZ=0.0
                cv2.rectangle(color_image, (centerx-winsize[0], centery-winsize[1]), (centerx+winsize[0], centery+winsize[1]), (255, 0, 0), 1)
                for k in range(centery-winsize[1],centery+winsize[1]):
                    for l in range(centerx-winsize[0],centerx+winsize[0]):
                        depth=verts[k,l][2]
                        if depth!=0:
                            distance.append(round(depth,3))
                if distance!=[]:
                    distanceZ=mode(distance)
            cv2.namedWindow('depth_cut', cv2.WINDOW_NORMAL)
            cv2.imshow('depth_cut', color_image)
            if cv2.waitKey(1) & 0xFF==27:
                break
            elif time.time()-t1>2:
                break
        pipeline.stop()
        cv2.destroyAllWindows()
        print(distanceZ)
        return distanceZ
'''