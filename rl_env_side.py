import numpy as np
import gym
from util import normalize_01 as normalize
from util import normalize_01, objects_limits, gripper_orn_to_world, gripper_limits, discretize, next_path
from real_robot import RealRobot
import pyrealsense2 as rs
import numpy as np
import json
import math
import matplotlib.pyplot as plt
import serial
import time
import sys
import NetFT
import os
ACTION_SIZE = 3  # z pitch
VECTOR_SIZE = 7


class Sim(RealRobot):

    def __init__(self):

        super().__init__()
        self.d30 = False
        self.d60 = False
        self.use_ft = True
        self.flex = False
        self.paperbox = False
        self.hardcode_z = 0.27  #0.0045 for training book 0.1mm perpage #0.02 for thick coated book(static) 0.2mm per-page #0.00 for plastic sheets 1mm per-page
                                  ## d30 tests #printing book 0.02
        self.rest_ft = []
        self.succ_ft = []
        self.fail_ft = []
        self.rewards_record = []
        COM_PORT = '/dev/ttyACM0'
        BAUD_RATES = 9600
        self.ser = serial.Serial(COM_PORT, BAUD_RATES)

        self.ati_gamma_ip = '192.168.1.1'
        self.ft_sensor = NetFT.Sensor(self.ati_gamma_ip)

        # rl relative
        self.steps = 0
        self.episode = 0

        # camera
        self.depth_image = None

        self.observation_space = gym.spaces.Box(
            low=-1, high=1, shape=(VECTOR_SIZE,), dtype=np.float32)
        self.action_space = self.get_action_space()

        # go camera home
        self.go_c_home()

        # camera
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        print("[INFO] start streaming...")
        profile = self.pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: ", depth_scale)

        intr = (
            profile.get_stream(rs.stream.depth)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        print("width is: ", intr.width)
        print("height is: ", intr.height)
        print("ppx is: ", intr.ppx)
        print("ppy is: ", intr.ppy)
        print("fx is: ", intr.fx)
        print("fy is: ", intr.fy)
        HFOV = math.degrees(2 * math.atan(intr.width / (intr.fx + intr.fy)))
        print("HFOV is", HFOV)
        VFOV = math.degrees(2 * math.atan(intr.height / (intr.fx + intr.fy)))
        print("VFOV is", VFOV)
        self.point_cloud = rs.pointcloud()

    def set_gripper_pressure(self):
        self.ser.write(b'50\n')

    def pre_pressure(self):
        self.ser.write(b'20\n')

    def control_finger(self, if_open=True):

        # TYPE: 0 to open, 1 to flex-flip, 2 to close, other num below 255 to test

        if if_open == True:
            self.ser.write(b'0\n')
            time.sleep(0.5)
            # self.ser.close()

        else:
            self.ser.write(b'1\n')
            time.sleep(1.5)
            self.ser.write(b'2\n')
            time.sleep(0.5)
            # self.ser.close()

        # while self.ser.in_waiting:
        #     mcu_feedback = self.ser.readline().decode()
        #     print('Feedback：', mcu_feedback)

    def control_motor(self,if_open=True):

        if if_open == True:
            self.ser.write(b'4\n')
            time.sleep(0.5)
            # self.ser.close()

        else:
            self.ser.write(b'3\n')
            time.sleep(0.5)  

    def get_fz(self):
        
        ft_result = self.ft_sensor.getMeasurement()
        fz = min(max(ft_result[2]/10000, -1000),1000)
        return fz
    
    def get_ft(self):
        
        ft_result = self.ft_sensor.getMeasurement()
        return ft_result   
    def get_verts(self):

        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        decimation = rs.decimation_filter(2)
        depth_frame = decimation.process(depth_frame)
        points = self.point_cloud.calculate(depth_frame)
        verts = (
            np.asanyarray(points.get_vertices()).view(
                np.float32).reshape(240, 320, 3)
        )  # xyz
        return verts

    def stop_streaming(self):
        self.pipeline.stop()

    def get_action_space(self):
        return gym.spaces.Box(low=-1, high=1, shape=(ACTION_SIZE,), dtype=np.float32)

    def render_camera(self):
        verts = self.get_verts()
        # np.save('verts.npy',verts)
        # self.stop_streaming()

        cz = verts[:, :, 2]
        cz = cz[200-10:200+10, 175-10:175+10]
        self.z_from_camera = np.mean(cz)
        self.depth_image = cz

    def get_observation(self, z, ft):
        
        current_z = normalize(z, [0.2, 1])
        
        fx = round(normalize(ft[0],[-1000, 1000]), 1)
        fy = round(normalize(ft[1],[-1000, 1000]), 1)
        fz = round(normalize(ft[2],[-1000, 1000]), 3)
        
        fa = round(normalize(ft[3],[-1000, 1000]), 1)
        fb = round(normalize(ft[4],[-1000, 1000]), 1)
        fc = round(normalize(ft[5],[-1000, 1000]), 1)
        
        if self.use_ft is False:
            fx = 0
            fy = 0
            fz = 0
            fa = 0
            fb = 0
            fc = 0
        
        gripper_pose = [current_z, fx,fy,fz,fa,fb,fc]

        return np.array(gripper_pose, dtype=np.float32)

    def go_home(self):
        self.go_c_home()
        
    def reset(self):
        if self.d30 is True:
            return self.reset30()
        elif self.d60 is True:
            return self.reset60()
        else:
            return self.reset0()

    def reset60(self):

        self.control_motor(if_open=False)

        self.steps = 0
        self.episode += 1
        self.control_finger(if_open=True)
        # time.sleep(0.5)
        self.go_c_home_60()

        self.render_camera()

        self.tollernce = 0.02
        self.set_tcp(0, 0, 0.14, 0, 0, math.radians(45))
        self.set_tcp(0, 0, 0.14, 0, 0, math.radians(45))
        
        if (0.407-self.z_from_camera) + self.tollernce > 0.3:
            
            print(self.depth_image)
            print(self.z_from_camera)
            input('something wrong need check')
######################################################################3        ########################
        save_path = "./bin/sac_con/"
        os.makedirs(save_path, mode=0o777, exist_ok=True)
        file_path = next_path(save_path + "ft-%s.txt")
        
        if self.hardcode == False:
            down_z = self.z_from_camera -0.14 + self.tollernce
            self.move_tool(0, 0, down_z,acc=0.5,vel=0.5)
        elif self.hardcode == True:
            down_z = self.hardcode_z
            self.move_tool(0, 0, down_z,acc=0.5,vel=0.5)               
                  
        self.set_gripper_pressure()
        
        record_ft = []
        for i in range(200):
            time.sleep(2/200)
            record_ft.append(self.ft_sensor.getMeasurement())
            
        ft_result = self.ft_sensor.getMeasurement()
        np.savetxt(file_path, record_ft)
##########################################################3
        print('ft:', ft_result)
        fx = min(max(ft_result[0]/10000, -1000),1000)
        fy = min(max(ft_result[1]/10000, -1000),1000)
        fz = min(max(ft_result[2]/10000, -1000),1000)
        
        fa = min(max(ft_result[3]/10000, -1000),1000)
        fb = min(max(ft_result[4]/10000, -1000),1000)
        fc = min(max(ft_result[5]/10000, -1000),1000)        
        print('rest_ft:', ft_result, 'normalize:', [fx,fy,fz,fa,fb,fc])

        return self.get_observation(self.z_from_camera, [fx,fy,fz,fa,fb,fc])    

    def reset30(self):

        self.control_motor(if_open=False)

        self.steps = 0
        self.episode += 1
        self.control_finger(if_open=True)
        # time.sleep(0.5)
        self.go_c_home_30()

        self.render_camera()

        self.tollernce = 0.02
        self.set_tcp(0, 0, 0.14, 0, 0, math.radians(45))
        self.set_tcp(0, 0, 0.14, 0, 0, math.radians(45))
        
        if (0.407-self.z_from_camera) + self.tollernce > 0.3:
            
            print(self.depth_image)
            print(self.z_from_camera)
            input('something wrong need check')
######################################################################3        ########################
        save_path = "./bin/sac_con/"
        os.makedirs(save_path, mode=0o777, exist_ok=True)
        file_path = next_path(save_path + "ft-%s.txt")
        
        if self.hardcode == False:
            down_z = self.z_from_camera -0.14 + self.tollernce
            self.move_tool(0, 0, down_z,acc=0.5,vel=0.5)
        elif self.hardcode == True:
            down_z = self.hardcode_z
            self.move_tool(0, 0, down_z,acc=0.5,vel=0.5)
            # down_z = self.hardcode_z
            # self.move_with_ori(
            #         [0, 0, down_z],acc=0.5,vel=0.1)          
                
                  
        self.set_gripper_pressure()
        
        record_ft = []
        for i in range(200):
            time.sleep(2/200)
            record_ft.append(self.ft_sensor.getMeasurement())
            
        ft_result = self.ft_sensor.getMeasurement()
        np.savetxt(file_path, record_ft)
##########################################################3
        print('ft:', ft_result)
        fx = min(max(ft_result[0]/10000, -1000),1000)
        fy = min(max(ft_result[1]/10000, -1000),1000)
        fz = min(max(ft_result[2]/10000, -1000),1000)
        
        fa = min(max(ft_result[3]/10000, -1000),1000)
        fb = min(max(ft_result[4]/10000, -1000),1000)
        fc = min(max(ft_result[5]/10000, -1000),1000)        
        print('rest_ft:', ft_result, 'normalize:', [fx,fy,fz,fa,fb,fc])

        return self.get_observation(self.z_from_camera, [fx,fy,fz,fa,fb,fc])    
        
    def reset0(self):

        self.control_motor(if_open=False)

        self.steps = 0
        self.episode += 1
        self.control_finger(if_open=True)
        # time.sleep(0.5)
        self.go_c_home()

        self.render_camera()

        self.tollernce = 0.029
        self.set_tcp(0, 0, 0.14, 0, 0, math.radians(45))
        self.set_tcp(0, 0, 0.14, 0, 0, math.radians(45))
        
        if (0.407-self.z_from_camera) + self.tollernce > 0.3:
            
            print(self.depth_image)
            print(self.z_from_camera)
            input('something wrong need check')
######################################################################3        ########################
        save_path = "./bin/sac_con/"
        os.makedirs(save_path, mode=0o777, exist_ok=True)
        file_path = next_path(save_path + "ft-%s.txt")
        
        if self.hardcode == False:
            self.move_with_ori(
                    [0, 0, (0.407-self.z_from_camera) + self.tollernce],acc=0.5,vel=0.5)
        elif self.hardcode == True:
            self.move_with_ori(
                    [0, 0, self.hardcode_z+ self.tollernce],acc=0.5,vel=0.1)          
                
                  
        self.set_gripper_pressure()
        
        record_ft = []
        for i in range(200):
            time.sleep(2/200)
            record_ft.append(self.ft_sensor.getMeasurement())
            
        ft_result = self.ft_sensor.getMeasurement()
        np.savetxt(file_path, record_ft)
##########################################################3
        print('ft:', ft_result)
        fx = min(max(ft_result[0]/10000, -1000),1000)
        fy = min(max(ft_result[1]/10000, -1000),1000)
        fz = min(max(ft_result[2]/10000, -1000),1000)
        
        fa = min(max(ft_result[3]/10000, -1000),1000)
        fb = min(max(ft_result[4]/10000, -1000),1000)
        fc = min(max(ft_result[5]/10000, -1000),1000)        
        print('rest_ft:', ft_result, 'normalize:', [fx,fy,fz,fa,fb,fc])

        return self.get_observation(self.z_from_camera, [fx,fy,fz,fa,fb,fc])
    
    def step(self, action):

        done = False
        self.steps += 1
        reward = 0
        observation = None

        rela_z = np.round(discretize(
            action[0]*0.003, [-0.003, -0.002, -0.001, 0, 0.001, 0.002, 0.003]), decimals=4)
        # pitch = np.round(discretize(action[1], [-1,1]), decimals=4)

        if action[1] >= 0.5:
            pitch = 0
        elif 0 <= action[1] < 0.5:
            pitch = -1
        elif -0.5 <= action[1] < 0:
            pitch = -2
        elif -1 <= action[1] < -0.5:
            pitch = -3
        else:
            print(action[1])

        rela_x = np.round(discretize(action[2]*0.003, [-0.003, -0.002, -0.001, 0, 0.001, 0.002, 0.003]), decimals=4)
        
        rela_y = 0
        yaw = 0
        roll = 0
        
        if self.hardcode is True:
            if self.d30 is True:
                rela_z = 0.0001
                real_x = 0
                pitch = -3
                self.hardcode_z = self.hardcode_z + 0.0001
            elif self.d60 is True:
                rela_z = 0.0001
                real_x = 0
                pitch = -3
                self.hardcode_z = self.hardcode_z + 0.0001
            else:
                rela_z = -0.0001
                real_x = 0
                pitch = -3
                self.hardcode_z = self.hardcode_z - 0.0001


        self.set_tcp(0, 0, 0.14, 0, 0, math.radians(45))
        self.set_gripper_ori(math.radians(
            roll), math.radians(pitch), math.radians(yaw))

        self.move_tool(rela_x, rela_y, rela_z)
        self.control_finger(if_open=False)
        time.sleep(1)
        print('rela_z:', rela_z, 'pitch:', pitch, 'rela_x:', rela_x)
        # print('reward:',reward)

        if self.paperbox is True:
            self.move_tool(0, 0, -0.045)
            self.set_gripper_ori(math.radians(0), math.radians(0), math.radians(-55), rot_acc=0.5, rot_vel=0.25)
            self.move_tool(0.275, 0, -0.045)
            self.move_tool(0, 0, -0.0325)
            self.control_finger(if_open=True)
        else:
            self.move_tool(0, 0, -0.0325)
            self.set_gripper_ori(math.radians(0), math.radians(0), math.radians(-55), rot_acc=0.5, rot_vel=0.25)
            self.move_tool(0.275, 0, 0)
            self.move_tool(0, 0, -0.0325)
            self.control_finger(if_open=True)
        
        reward += self.get_reward()
        done = True
        self.rewards_record.append(reward)
        np.savetxt('rewards_record.txt', self.rewards_record)
        return observation, reward, done, {}

    def get_reward(self):

        return reward

    def get_depth_image(self):
        return np.squeeze(self.depth_image), self.z_from_camera
