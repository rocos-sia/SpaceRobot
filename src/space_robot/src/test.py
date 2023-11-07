######测试版#######
from coppeliasim_zmqremoteapi_client import *
import PyKDL
import math
import time
import threading
import keyboard

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

    # # 创建关节角度列表
    # joint_positions = PyKDL.JntArray(len(dh_params))
    # for i in range(len(dh_params)):
    #     joint_positions[i] = 0.0  # 用实际的关节角度替代0.0

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

# sim.setJointPosition(b2t_joint4, math.pi/2)

sim.setStepping(True)
sim.startSimulation()

# #将tip_to_base机械臂隐藏
# #显示模型
# sim.setModelProperty(base_to_tip, sim.modelproperty_not_collidable)
# sim.step()
# #隐藏模型
# sim.setModelProperty(tip_to_base, sim.modelproperty_not_visible)
# sim.step()


#改变b2t_joint3的初始位置
# sim.setJointPosition(b2t_joint3,  -math.pi/2)
#开始仿真

#将base_to_tip移动对接位置处
num_step = 1000
jnt3step = (-math.pi/4)/num_step
jnt4step = (math.pi/2)/num_step
jnt5step = (-math.pi/4)/num_step
# for i in range(num_step):
#     sim.setJointTargetPosition(b2t_joint3,  -math.pi/2 - jnt3step*i)
#     sim.setJointTargetPosition(b2t_joint4,  jnt4step*i)
#     sim.setJointTargetPosition(b2t_joint5,  jnt5step*i)
#     sim.step()

#获取base_to_tip末端在世界坐标系下的位姿----需要改进
bt_pose = sim.getObjectPose(b2t_joint7, -1)
# print(bt_pose)
tb_pose = sim.getObjectPose(t2b_base_link, -1)
# print(tb_pose)
for i in range(3):
    tb_pose[i] = -bt_pose[i]
sim.setObjectPose(t2b_base_link, -1, tb_pose)
# sim.setModelProperty(tip_to_base, sim.modelproperty_not_collidable)
sim.step()

for i in range(500):
    sim.step()
# pose[3] = -0.5008366852707948
# pose[4] = -0.4991110637361582
# pose[5] = 0.4991110637361582

#将tip_to_base的基座位姿设置成base_to_tip末端处的位姿
# sim.setObjectPosition(t2b_base_link, -1, pose)  # 设置平移部分
# sim.setObjectQuaternion(t2b_base_link, -1, pose[3:6])  # 设置旋转部分
sim.step()

#设置tip_to_base的关节角度，使得tip_to_base的模型与base_to_tip的模型重合
sim.setJointPosition(t2b_joint3,  sim.getJointPosition(b2t_joint3))
sim.setJointPosition(t2b_joint4,  sim.getJointPosition(b2t_joint4))
sim.setJointPosition(t2b_joint5,  sim.getJointPosition(b2t_joint5))

# #显示tip_to_base机械臂并隐藏base_to_tip机械臂
# sim.setModelProperty(tip_to_base, sim.modelproperty_not_collidable)
# #隐藏模型
# sim.setModelProperty(base_to_tip, sim.modelproperty_not_visible)

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
# for i in range(num_step):
#     sim.setJointTargetPosition(t2b_joint3,  t2b_joint3_current - jnt3step*i)
#     sim.setJointTargetPosition(t2b_joint4,  t2b_joint4_current + jnt4step*i)
#     # sim.setJointTargetPosition(t2b_joint5,  - jnt5step*i)
#     sim.step()

# t2b_joint3_current = sim.getJointPosition(t2b_joint3)
# t2b_joint4_current = sim.getJointPosition(t2b_joint4)
# sim.setJointPosition(t2b_joint3,t2b_joint3_current)
# sim.setJointPosition(t2b_joint4,t2b_joint4_current)

for i in range(50):
    sim.step()

# #隐藏模型tip_to_base模型
# sim.setModelProperty(tip_to_base, sim.modelproperty_not_visible)
# #显示base_to_tip模型
# sim.setModelProperty(base_to_tip, sim.modelproperty_not_collidable)
# sim.step()








# # print(frame)
# #进行逆运动学求解
# joint_positions = PyKDL.JntArray(len(forward_dh))
# ret = tb_ik_solver.CartToJnt(PyKDL.JntArray(len(forward_dh)), frame, joint_positions)
# # print(ret)
# # print(joint_positions)



#隐藏模型
# sim.setModelProperty(base_link, sim.modelproperty_not_visible)
#显示模型
# sim.setModelProperty(base_link, sim.modelproperty_not_collidable)



# base_link = sim.getObject("/base_link")
# print(base_link)
# sim.setObjectIntParameter()

# joint1 = sim.getObject("joint1")


# sim.setObjectPosition(base_link, -1, [0, 0, 0.3])




# joint5 = sim.getObjectHandle("joint5")
# joint6 = sim.getObjectHandle("joint6")
# sim.setStepping(True)
# simBase=sim.getObject('redundantRobot')
# # sim.setJointPosition(joint4, 0)
# sim.startSimulation()



# # 将机械臂停留在结束位置
# position3 = sim.getJointPosition(joint3)
# position4 = sim.getJointPosition(joint4)
# position5 = sim.getJointPosition(joint5)
# print(position3)
# print(position4)
# print(position5)
# # sim.step()
# sim.setObjectInt32Param(base_link, sim.objintpar#回归仿真前初始状态
# #隐藏模型tip_to_base模型
# sim.setModelProperty(tip_to_base, sim.modelproperty_not_visible)
# #显示base_to_tip模型
# sim.setModelProperty(base_to_tip, sim.modelproperty_not_collidable)
# sim.step()

# print(sim.getObjectInt32Param(base_link, sim.objintparam_visibility_layer))
# sim.step()
sim.stopSimulation()

# #回归仿真前初始状态
# #隐藏模型tip_to_base模型
# sim.setModelProperty(tip_to_base, sim.modelproperty_not_visible)
# #显示base_to_tip模型
# sim.setModelProperty(base_to_tip, sim.modelproperty_not_collidable)
# sim.step()



############################################################################################################
##存在问题：1.仿真运行结束后机械臂无法固定在原来的位置，会返回初始状态。在开始仿真到结束仿真过程中，通过setJointPosition()函数设置关节角度，仿真结束后会返回初始状态.在sim.stopSimulation()结束后设置关节角度，仍然出现相同状况。
##        2.设置姿态时，通过setObjectQuaternion()函数设置姿态，需要再分析。
##        3.