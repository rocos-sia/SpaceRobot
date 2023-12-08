
from coppeliasim_zmqremoteapi_client import *
import PyKDL
import math
# import time
# import threading
# import keyboard

# 创建DH参数数组
forward_dh = [
    (0, 0.3645, 0, math.pi/2),
    (math.pi/2, 0.6115, 0, -math.pi/2),
    (-0, 0, 4.44, 0),
    (-0, 0, 4.44, 0),
    (math.pi/2, 0.6445, 0, math.pi/2),
    (-math.pi/2, 0.6115, 0, -math.pi/2),
    (0, 0.3645, 0, 0)
]

inverse_dh = [
    (0, 0.3645, 0, math.pi/2),
    (math.pi/2, 0.6115, 0, math.pi/2),
    (math.pi/2, 0, 4.44, 0),
    (-0, 0, 4.44, 0),
    (0, 0.6445, 0, -math.pi/2),
    (math.pi/2, 0.6115, 0, math.pi/2),
    (0, 0.3645, 0, 0)
]


#根据DH参数计算正逆运动学
def kinematics_solver(dh_params):
    # 创建机器人对象并添加连杆
    robot = PyKDL.Chain()
    for params in dh_params:
        theta, d, a, alpha = params
        link = PyKDL.Segment()
        joint = PyKDL.Joint(PyKDL.Joint.RotZ)
        frame = PyKDL.Frame()
        frame.p = PyKDL.Vector(a, 0, d)  # 设置平移部分
        frame.M = PyKDL.Rotation.RotZ(theta)
        frame.M =  frame.M * PyKDL.Rotation.RotX(alpha)  # 设置旋转部分
        link = PyKDL.Segment(joint, frame)
        robot.addSegment(link)


    # 创建正运动学求解器
    fk_solver = PyKDL.ChainFkSolverPos_recursive(robot)
    # 逆运动学根据PyKDL.Frame()求解关节位置
    ik_solver = PyKDL.ChainIkSolverPos_LMA(robot)
    return fk_solver,ik_solver
    
# 机器人正逆运动学求解器
dh_params = inverse_dh
tb_fk_solver,tb_ik_solver=kinematics_solver(dh_params)
dh_params = forward_dh
bt_fk_solver,bt_ik_solver=kinematics_solver(dh_params)  
#-------------------------------------------------------------


#仿真模块
client = RemoteAPIClient()
sim = client.require('sim')

#获取机械臂的句柄
#获取base_to_tip的句柄
base_to_tip = sim.getObject("/base_to_tip")
b2t_base_link = sim.getObject("/b2t_base_link")
b2t_joint1 = sim.getObject("/b2t_joint1")
b2t_joint2 = sim.getObject("/b2t_joint2")
b2t_joint3 = sim.getObject("/b2t_joint3")
b2t_joint4 = sim.getObject("/b2t_joint4")
b2t_joint5 = sim.getObject("/b2t_joint5")
b2t_joint6 = sim.getObject("/b2t_joint6")
b2t_joint7 = sim.getObject("/b2t_joint7")
#获取tip_to_base的句柄
tip_to_base = sim.getObject("/tip_to_base")
t2b_base_link = sim.getObject("/t2b_base_link")
t2b_joint1 = sim.getObject("/t2b_joint1")
t2b_joint2 = sim.getObject("/t2b_joint2")
t2b_joint3 = sim.getObject("/t2b_joint3")
t2b_joint4 = sim.getObject("/t2b_joint4")
t2b_joint5 = sim.getObject("/t2b_joint5")
t2b_joint6 = sim.getObject("/t2b_joint6")
t2b_joint7 = sim.getObject("/t2b_joint7")

# sim.setJointPosition(b2t_joint4, 0)

sim.setStepping(True)
sim.startSimulation()

#将tip_to_base机械臂隐藏
#显示模型
sim.setModelProperty(base_to_tip, sim.modelproperty_not_collidable)
sim.step()
#隐藏模型
sim.setModelProperty(tip_to_base, sim.modelproperty_not_visible)
sim.step()


# #改变b2t_joint3的初始位置
# sim.setJointPosition(b2t_joint3,  0)


#将base_to_tip移动对接位置处
num_step = 1000
jnt3step = (-math.pi/4)/num_step
jnt4step = (math.pi/2)/num_step
jnt5step = (-math.pi/4)/num_step
for i in range(num_step):
    sim.setJointTargetPosition(b2t_joint3,  -math.pi/2 - jnt3step*i)
    sim.setJointTargetPosition(b2t_joint4,  jnt4step*i)
    sim.setJointTargetPosition(b2t_joint5,  jnt5step*i)
    sim.step()

#获取base_to_tip末端在世界坐标系下的位姿----需要改进
pose = sim.getObjectPose(b2t_joint7, -1)


#将tip_to_base的基座位姿设置成base_to_tip末端处的位姿
sim.setObjectPosition(t2b_base_link, -1, pose)  # 设置平移部分
# sim.setObjectQuaternion(t2b_base_link, -1, pose[3:6])  # 设置旋转部分
sim.step()

#设置tip_to_base的关节角度，使得tip_to_base的模型与base_to_tip的模型重合
sim.setJointPosition(t2b_joint3,  sim.getJointPosition(b2t_joint3))
sim.setJointPosition(t2b_joint4,  sim.getJointPosition(b2t_joint4))
sim.setJointPosition(t2b_joint5,  sim.getJointPosition(b2t_joint5))

#显示tip_to_base机械臂并隐藏base_to_tip机械臂
sim.setModelProperty(tip_to_base, sim.modelproperty_not_collidable)
#隐藏模型
sim.setModelProperty(base_to_tip, sim.modelproperty_not_visible)

#获取tip_to_base的关节角，将其设置为角度
t2b_joint3_current = sim.getJointPosition(t2b_joint3)
# print(t2b_joint3_current)
t2b_joint4_current = sim.getJointPosition(t2b_joint4)
# print(t2b_joint4_current)
t2b_joint5_current = sim.getJointPosition(t2b_joint5)
# print(t2b_joint5_current)
jnt3step = (-t2b_joint3_current -t2b_joint3_current)/num_step
jnt4step = (-t2b_joint4_current -t2b_joint4_current)/num_step
jnt5step = (-t2b_joint5_current -t2b_joint5_current)/num_step
for i in range(num_step):
    sim.setJointTargetPosition(t2b_joint3,  t2b_joint3_current - jnt3step*i)
    sim.setJointTargetPosition(t2b_joint4,  t2b_joint4_current + jnt4step*i)
    sim.step()

#添加循环让模型维持在该位置
for i in range(1000):
    sim.step()

sim.stopSimulation()
sim.step()
#隐藏模型tip_to_base模型
sim.setModelProperty(tip_to_base, sim.modelproperty_not_visible)
#显示base_to_tip模型
sim.setModelProperty(base_to_tip, sim.modelproperty_not_collidable)
sim.step()

