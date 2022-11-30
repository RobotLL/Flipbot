import time
import time
import socket
import urx
from real_gripper import Robotiq_Two_Finger_Gripper
import pybullet as p
import gym
import pyrealsense2 as rs
import numpy as np
# from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
import math
import os
import math3d as m3d
import serial
import time
import sys
import NetFT
from util import gripper_orn_to_world

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
# crad_00_center
X_OFFSET = -0.530
Y_OFFSET = 0.59 - 0.005
Z_OFFSET = 0.45

Camera_x_offset = 0
Camera_y_offset = 0
Camera_z_offset = 0


class RealRobot(gym.Env):
    def __init__(self):
        super().__init__()
        # Real hardware
        self.robot = urx.Robot("192.168.1.10", use_rt=True)
        # self.robotiq_gripper = Robotiq_Two_Finger_Gripper(self.robot)
        # self.robotiq_gripper.gripper_action_full(210, 1, 1)

        COM_PORT = '/dev/ttyACM0'
        BAUD_RATES = 9600
        self.ser = serial.Serial(COM_PORT, BAUD_RATES)
        self.ati_gamma_ip = '192.168.1.1'
        self.ft_sensor = NetFT.Sensor(self.ati_gamma_ip)

    def close_gripper(self, length=210):
        self.robotiq_gripper.gripper_action_full(length, 100, 100)

    def open_gripper(self, length=175):
        self.robotiq_gripper.gripper_action_full(
            length, 100, 100
        )  # 2.5cm gripper open angle

    def go_r_home(self):
        self.set_tcp(0, 0, 0, 0, 0, 0)
        self.move_world(X_OFFSET, Y_OFFSET, Z_OFFSET, math.pi, 0, 0, 0.1, 0.05)

    def go_c_home(self):
        self.set_tcp(0, 0, 0, 0, 0, 0)
        self.move_world(
            X_OFFSET - Camera_x_offset,
            Y_OFFSET - Camera_y_offset,
            Z_OFFSET,
            math.pi,
            0,
            0,
            1,
            1,
        )

    def go_c_home_30_test(self):
        self.set_tcp(0, 0, 0, 0, 0, 0)
        self.move_world(
            X_OFFSET - Camera_x_offset,
            Y_OFFSET - Camera_y_offset,
            Z_OFFSET,
            math.pi,
            0,
            0,
            0.5,
            0.5,
        )
        self.set_tcp(0, 0, 0.14, 0, 0, math.radians(0))
        self.set_gripper_ori(math.radians(
            0), math.radians(-30), 0, rot_acc=0.5, rot_vel=0.5)
        self.move_tool(rela_x=-0.2, rela_y=0, rela_z=0.3)
        self.move_tool(rela_x=0, rela_y=0, rela_z=-0.28)

    def go_c_home_30(self):
        self.set_tcp(0, 0, 0, 0, 0, 0)
        self.move_joint([-213.43, -106.16, -91.27, -46.81, 105.40, -299.72])

    def go_c_home_60_test(self):
        self.set_tcp(0, 0, 0, 0, 0, 0)
        self.move_world(
            X_OFFSET - Camera_x_offset,
            Y_OFFSET - Camera_y_offset,
            Z_OFFSET,
            math.pi,
            0,
            0,
            0.5,
            0.5,
        )
        self.set_tcp(0, 0, 0.14, 0, 0, math.radians(0))
        self.set_gripper_ori(math.radians(
            0), math.radians(-60), 0, rot_acc=0.5, rot_vel=0.5)
        self.move_tool(rela_x=-0.03, rela_y=-0.03, rela_z=0.26)
        self.move_tool(rela_x=0.05, rela_y=0, rela_z=0)
        self.move_tool(rela_x=0, rela_y=0, rela_z=-0.272)

    def go_c_home_60(self):
        self.set_tcp(0, 0, 0, 0, 0, 0)
        # normal book
        self.move_joint([-228.31, -104.40, -113.27, -3.44, 129.84, -299.58])
        # coted book
        # self.move_joint([-229.91, -106.14, -116.39, 0.49, 131.07, -300.99])

    def move_joint(self, home_position):
        Hong_joint0 = math.radians(home_position[0])
        Hong_joint1 = math.radians(home_position[1])
        Hong_joint2 = math.radians(home_position[2])
        Hong_joint3 = math.radians(home_position[3])
        Hong_joint4 = math.radians(home_position[4])
        Hong_joint5 = math.radians(home_position[5])
        self.robot.movej(
            (
                Hong_joint0,
                Hong_joint1,
                Hong_joint2,
                Hong_joint3,
                Hong_joint4,
                Hong_joint5,
            ),
            0.5,
            0.5,
        )

    def set_tcp(self, x, y, z, rx, ry, rz):
        self.robot.set_tcp((x, y, z, rx, ry, rz))
        time.sleep(1)

    def get_tool_position(self):
        tool_pose = self.robot.getl()
        x = tool_pose[0] - X_OFFSET
        y = tool_pose[1] - Y_OFFSET
        z = tool_pose[2]
        return (x, y, z)

    def cal_furure(self, targetxyz):
        tool_pose = self.robot.getl()
        x = tool_pose[0] - X_OFFSET
        y = tool_pose[1] - Y_OFFSET
        z = tool_pose[2]
        rel_x = targetxyz[0] - x
        rel_y = targetxyz[1] - y
        rel_z = targetxyz[2] - z
        return rel_x, rel_y, rel_z

    def move_with_ori(self, targetxyz, acc=0.2, vel=0.2, wait=True, relative=True):
        tool_pose = self.robot.getl()
        x = tool_pose[0] - X_OFFSET
        y = tool_pose[1] - Y_OFFSET
        z = tool_pose[2]
        rela_x = targetxyz[0] - x
        rela_y = targetxyz[1] - y
        rela_z = targetxyz[2] - z
        self.robot.movel((rela_x, rela_y, rela_z, 0, 0, 0),
                         acc, vel, wait, relative)

    def move_tool(self, rela_x=0, rela_y=0, rela_z=0, acc=0.1, vel=0.1):

        self.robot.translate_tool((rela_x, rela_y, rela_z), acc, vel)

    def move_world(
        self, wx, wy, wz, rx, ry, rz, acc=0.2, vel=0.2, wait=True, relative=True
    ):
        self.robot.movel((wx, wy, wz, rx, ry, rz), acc, vel)

    def set_gripper_ori(self, roll, pitch, yaw, rot_acc=0.01, rot_vel=0.01):

        move = m3d.Transform((0, 0, 0, 0, 0, -yaw))
        self.robot.add_pose_tool(
            move, acc=rot_acc, vel=rot_vel, wait=True, command="movel", threshold=None
        )
        move = m3d.Transform((0, 0, 0, roll, 0, 0))
        self.robot.add_pose_tool(
            move, acc=rot_acc, vel=rot_vel, wait=True, command="movel", threshold=None
        )
        move = m3d.Transform((0, 0, 0, 0, pitch, 0))
        self.robot.add_pose_tool(
            move, acc=rot_acc, vel=rot_vel, wait=True, command="movel", threshold=None
        )

    def resetFT300Sensor(self, tcp_host_ip="192.168.1.102"):
        HOST = tcp_host_ip
        PORT = 63351
        self.serialFT300Sensor = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM)
        self.serialFT300Sensor.connect((HOST, PORT))

    def getFT300SensorData(self):
        while True:
            data = (
                str(self.serialFT300Sensor.recv(1024), "utf-8")
                .replace("(", "")
                .replace(")", "")
                .split(",")
            )
            try:
                data = [float(x) for x in data]
                if len(data) == 6:
                    break
            except:
                pass
        return data

    def get_rob(self):
        pass
        return self.robot

    def get_verts(self, name):
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
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        points = self.point_cloud.calculate(depth_frame)
        verts = (
            np.asanyarray(points.get_vertices()).view(
                np.float32).reshape(480, 640, 3)
        )  # xyz
        np.save(name, verts[:, :, 2])
        self.pipeline.stop()
        return verts

    def set_gripper_pressure(self):
        self.ser.write(b'50\n')

    def pre_pressure(self):
        self.ser.write(b'10\n')

    def control_finger(self, if_open=True):
        if if_open == True:
            self.ser.write(b'0\n')
            time.sleep(0.5)
        else:
            self.ser.write(b'1\n')
            time.sleep(1.5)
            self.ser.write(b'2\n')
            time.sleep(0.5)


# %% fig1
if __name__ == "__main__":
    rob = RealRobot()
    rob.set_tcp(0, 0, 0, 0, 0, 0)
    rob.move_world(X_OFFSET, Y_OFFSET, Z_OFFSET, math.pi, 0, 0, 0.2, 0.2)
    rob.set_gripper_ori(0, 0, math.radians(45), rot_acc=0.5, rot_vel=0.5)
    rob.set_tcp(0, 0, 0.2, 0, 0, 0)
    rob.move_with_ori([0.2, -0.1, 0.2])
    rob.get_verts('fg1_depth.npy')
    rob.go_c_home()
    rob.set_tcp(0, 0, 0.18, 0, 0, 0)
    rob.move_with_ori([0.01, -0.01, 0])
    rob.pre_pressure()
    rob.control_finger(True)
    
#%% fig2
# if __name__ == "__main__":
#     rob = RealRobot()
#     rob.set_tcp(0, 0, 0, 0, 0, 0)
#     rob.move_world(X_OFFSET, Y_OFFSET, Z_OFFSET, math.pi, 0, 0, 0.2, 0.2)
#     rob.set_gripper_ori(0, 0, math.radians(45), rot_acc=0.5, rot_vel=0.5)
#     rob.set_tcp(0, 0, 0.2, 0, 0, 0)
#     rob.move_with_ori([0.2, -0.1, 0.2])
#     rob.get_verts('fg2_depth.npy')
#     rob.go_c_home()
#     rob.set_tcp(0, 0, 0.18, 0, 0, 0)
#     rob.move_with_ori([0, 0, -0.015])
#     rob.pre_pressure()
#     #%%
#     rob.control_finger(True)

# %%
# if __name__ == "__main__":
#     rob = RealRobot()
#     rob.set_tcp(0, 0, 0, 0, 0, 0)
#     rob.move_world(X_OFFSET, Y_OFFSET, Z_OFFSET, math.pi, 0, 0, 0.2, 0.2)
#     rob.set_gripper_ori(0, 0, math.radians(45), rot_acc=0.5, rot_vel=0.5)
#     rob.set_tcp(0, 0, 0.2, 0, 0, 0)
#     rob.move_with_ori([0.2, -0.1, 0.2])
#     rob.get_verts('fg1_depth.npy')
#     # %%
#     rob.go_c_home()
#     # %%
#     # rob.pre_pressure()
#     rob.set_gripper_pressure()
#     rob.control_finger(True)
    # %%
    # get_image
    # # %%
    # rob.set_tcp(0, 0, 0.14, 0, 0, math.radians(45))
    # # rob.set_tcp(0, 0, 0.255, 0, 0, 0)  # 8cm open #0.27
    # # %%
    # rob.set_gripper_ori(0, math.radians(15), 0)
    # #%%
    # rob.set_gripper_ori(0, math.radians(-15), 0)
    # # rob.set_gripper_ori(math.radians(15), 0, 0)
    # # %%
    # rob.move_with_ori([0, 0, 0])
    # %%
