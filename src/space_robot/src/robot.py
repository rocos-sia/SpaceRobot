from coppeliasim_zmqremoteapi_client import *
import PyKDL
import math
from kinematics import Kinematics
import numpy as np
import csv
class Robot:
    def __init__(self):
        #实例化Kinematics类
        self.my_kinematics = Kinematics()
        #机械臂状态
        self.b2t_status = 0 ##状态标志位应当从外部读取,当前采用的是读取模型是否隐藏来确定。
        self.t2b_status = 0
        #初始化仿真模块
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        #获取句柄
        #获取base_to_tip的句柄
        self.base_to_tip = self.sim.getObject("/base_to_tip")
        self.b2t_base_link = self.sim.getObject("/b2t_base_link")
        self.b2t_joint1 = self.sim.getObject("/b2t_joint1")
        self.b2t_joint2 = self.sim.getObject("/b2t_joint2")
        self.b2t_joint3 = self.sim.getObject("/b2t_joint3")
        self.b2t_joint4 = self.sim.getObject("/b2t_joint4")
        self.b2t_joint5 = self.sim.getObject("/b2t_joint5")
        self.b2t_joint6 = self.sim.getObject("/b2t_joint6")
        self.b2t_joint7 = self.sim.getObject("/b2t_joint7")
        #获取tip_to_base的句柄
        self.tip_to_base = self.sim.getObject("/tip_to_base")
        self.t2b_base_link = self.sim.getObject("/t2b_base_link")
        self.t2b_joint1 = self.sim.getObject("/t2b_joint1")
        self.t2b_joint2 = self.sim.getObject("/t2b_joint2")
        self.t2b_joint3 = self.sim.getObject("/t2b_joint3")
        self.t2b_joint4 = self.sim.getObject("/t2b_joint4")
        self.t2b_joint5 = self.sim.getObject("/t2b_joint5")
        self.t2b_joint6 = self.sim.getObject("/t2b_joint6")
        self.t2b_joint7 = self.sim.getObject("/t2b_joint7")

        # self.sim.setJointPosition(getattr(self, f"b2t_joint{3}"),0)
        # self.sim.setJointPosition(getattr(self, f"b2t_joint{4}"),0)  

        #启动仿真
        self.sim.setStepping(True)
        self.sim.startSimulation()
   
    #检查机械臂哪端为基座
    def check_robot_status(self):
        b2t_model_property = self.sim.getModelProperty(self.base_to_tip)
        t2b_model_property = self.sim.getModelProperty(self.tip_to_base)
        visible_flag = self.sim.modelproperty_not_visible
        if b2t_model_property & visible_flag != 0:  #模型型不可见
            #模型base_to_tip不可见
            self.b2t_status = 0     #模型可见标志位
            self.t2b_status = 1
            self.sim.setModelProperty(self.base_to_tip, self.sim.modelproperty_not_visible)     #设置base_to_tip不可见
            self.sim.setModelProperty(self.tip_to_base, self.sim.modelproperty_not_collidable)  #设置tip_to_base可见
        else:
            self.b2t_status = 1     #模型可见标志位
            self.t2b_status = 0
            self.sim.setModelProperty(self.base_to_tip, self.sim.modelproperty_not_collidable)     #设置base_to_tip可见
            self.sim.setModelProperty(self.tip_to_base, self.sim.modelproperty_not_visible)        #设置tip_to_base不可见

    #获取当前关节的关节角度值
    def getJointPosition(self,i):
        if self.b2t_status == 1:
            return self.sim.getJointPosition(getattr(self, f"b2t_joint{i}"))
        else:
            return self.sim.getJointPosition(getattr(self, f"t2b_joint{i}"))


    #延时函数   
    def delay(self,t):
        start_time = self.sim.getSystemTime()
        end_time = start_time
        # print(start_time)
        while end_time - start_time < t:
            end_time = self.sim.getSystemTime()
            self.sim.step()     #不能不加，否则会与coppeliasim断开连接
        # print(end_time)

    #moveJ实现
    def moveJ(self,q_target,max_vel,max_acc):

        # self.check_robot_status()   #检查当前是哪个模型可见

        q_current =[0.0]* self.my_kinematics.jointNum
        for i in range(self.my_kinematics.jointNum):
            if self.b2t_status ==1 :
                q_current[i]= self.sim.getJointPosition(getattr(self, f"b2t_joint{i+1}"))
            else:
                q_current[i]= self.sim.getJointPosition(getattr(self, f"t2b_joint{i+1}"))
        #求最长轨迹所处的关节
        max_diff = float('-inf')  # 初始化为负无穷大
        # max_diff_index = None
        for i, (val1, val2) in enumerate(zip(q_target, q_current)):
            diff = abs(val1 - val2)
            if diff > max_diff:
                max_diff = diff
                # max_diff_index = i

###############进行[0-1]之间的T形轨迹规划###############
        t0,p0,p1,v0,v1 = 0 ,0 ,1 ,0 ,0
        vmax = max_vel / max_diff
        aamax = max_acc / max_diff
        admax = -max_acc / max_diff
        #确定[0-1]之间各阶段的时间
        h = p1 - p0
        vf = math.sqrt((2.0 * aamax * admax * h - aamax * v1**2 + admax * v0**2) / (admax - aamax))
        if vf < vmax:
            vv = vf
        else:
            vv = vmax
        # 计算加速阶段的时间和位移
        Ta = (vv - v0) / aamax
        La = v0 * Ta + (1.0/2.0) * aamax * Ta**2
        # 计算匀速阶段的时间和位移
        Tv = (h - (vv**2 - v0**2) / (2.0 * aamax) - (v1**2 - vv**2) / (2.0 * admax)) / vv
        Lv = vv * Tv
        # 计算减速阶段的时间和位移
        Td = (v1 - vv) / admax
        Ld = vv * Td + (1.0/2.0) * admax * Td**2
        t_total = Ta + Tv + Td

        time_count = 0
        dt = self.sim.getSimulationTimeStep() #coppeliasim软件步长
        # print("dt = ",dt)
        pos = [0.0]* self.my_kinematics.jointNum


# ###################记录关节数据################################
#         csv_file_path = 'joint_positions.csv'

#         with open(csv_file_path, 'w', newline='') as csvfile:
#             csv_writer = csv.writer(csvfile)
#             header = [f'Joint {i+1}' for i in range(self.my_kinematics.jointNum)]
#             csv_writer.writerow(header)
# ##############################################################
        # count = 0
        while time_count < t_total:
            t = time_count - t0
            if t >= 0 and t < Ta:
                p = p0 + v0 * t + (1.0/2.0) * aamax * t**2
                # pd = v0 + aamax * t
                # pdd = aamax
            elif t >= Ta and t < Ta + Tv:
                p = p0 + La + vv * (t - Ta)
                # pd = vv
                # pdd = 0
            elif t >= Ta + Tv and t <= Ta + Tv + Td:
                p = p0 + La + Lv + vv * (t - Ta - Tv) + (1.0/2.0) * admax * (t - Ta - Tv)**2
                # pd = vv + admax * (t - Ta - Tv)
                # pdd = admax
            for i in range(self.my_kinematics.jointNum):
                pos[i] = q_current[i] + p * (q_target[i] - q_current[i])
                if self.b2t_status ==1 :
                    self.sim.setJointTargetPosition(getattr(self, f"b2t_joint{i+1}"),pos[i])
                else:
                    self.sim.setJointTargetPosition(getattr(self, f"t2b_joint{i+1}"),pos[i])

            
# #####################将关节位置记录到csv文件#######################
#             with open(csv_file_path, 'a', newline='') as csvfile:
#                 csv_writer = csv.writer(csvfile)
#                 csv_writer.writerow(pos)
# ###############################################################
            self.sim.step()
            self.delay(dt)
            time_count += dt
            # count +=1
        # print(t_total)
        # print(count)

    #交换基座
    def switch(self):
        if self.b2t_status == 1 :   #当前base_to_tip显示要转换成tip_to_base
            for i in range(self.my_kinematics.jointNum):#将tip_to_base的形状设置成和base_to_tip一致
                self.sim.setJointPosition(getattr(self, f"t2b_joint{i+1}"),self.getJointPosition(self.my_kinematics.jointNum -i))
            #设置base_to_tip的末端和tip_to_base的基座重合
            pose = self.sim.getObjectPose(self.b2t_joint7, -1)
            # original_quaternion = pose[3:]
            # new_quaternion = self.my_kinematics.rotate_quaternion(original_quaternion,'x',np.pi)
            # pose[3:] = new_quaternion
            self.sim.setObjectPosition(self.t2b_base_link,-1,pose)
            #隐藏与显示
            self.sim.setModelProperty(self.base_to_tip, self.sim.modelproperty_not_visible)     #设置base_to_tip不可见
            self.sim.step()
            self.sim.setModelProperty(self.tip_to_base, self.sim.modelproperty_not_collidable)        #设置tip_to_base可见
            self.b2t_status = 0
            self.t2b_status = 1
            self.sim.step()



        else:   ##当前tip_to_base显示,要转换成base_to_tip显示
            for i in range(self.my_kinematics.jointNum):#将base_to_tip的形状设置成和tip_to_base一致
                self.sim.setJointPosition(getattr(self, f"b2t_joint{i+1}"),self.getJointPosition(self.my_kinematics.jointNum -i))
            
            #设置tip_to_base的末端和的base_to_tip基座重合
            pose = self.sim.getObjectPose(self.t2b_joint7, -1)
            # original_quaternion = pose[3:]
            # new_quaternion = self.my_kinematics.rotate_quaternion(original_quaternion,'x',np.pi)
            # pose[3:] = new_quaternion
            self.sim.setObjectPosition(self.b2t_base_link,-1,pose)
            #隐藏与显示
            self.sim.setModelProperty(self.base_to_tip, self.sim.modelproperty_not_collidable)     #设置base_to_tip可见
            self.sim.step()
            self.sim.setModelProperty(self.tip_to_base, self.sim.modelproperty_not_visible)  #设置tip_to_base可见
            self.b2t_status = 1
            self.t2b_status = 0
            self.sim.step()
    
    #将模型回归初始状态
    def model_to_initial(self):
        #隐藏模型tip_to_base模型
        self.sim.setModelProperty(self.tip_to_base, self.sim.modelproperty_not_visible)
        self.sim.setModelProperty(self.base_to_tip, self.sim.modelproperty_not_collidable)
        self.sim.step()
        self.sim.stopSimulation()


