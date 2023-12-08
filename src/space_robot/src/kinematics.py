import PyKDL
import math
import numpy as np
from scipy.spatial.transform import Rotation
class Kinematics:
    def __init__(self):
        #关节数目
        self.jointNum = 7
        #DH参数
        self.__forward_dh = [
                               (0, 0.3645, 0, math.pi/2),
                               (math.pi/2, 0.6115, 0, -math.pi/2),
                               (-0, 0, 4.44, 0),
                               (-0, 0, 4.44, 0),
                               (math.pi/2, 0.6445, 0, math.pi/2),
                               (-math.pi/2, 0.6115, 0, -math.pi/2),
                               (0, 0.3645, 0, 0)
                           ]
        self.__inverse_dh = [
                                (0, 0.3645, 0, math.pi/2),
                                (math.pi/2, 0.6115, 0, math.pi/2),
                                (math.pi/2, 0, 4.44, 0),
                                (-0, 0, 4.44, 0),
                                (0, 0.6445, 0, -math.pi/2),
                                (math.pi/2, 0.6115, 0, math.pi/2),
                                (0, 0.3645, 0, 0)
                           ]
        #创建运动学求解器
        #base_to_top运动学求解器
        self.bt_robot = PyKDL.Chain()
        for params in self.__forward_dh:
            theta, d, a, alpha = params
            link = PyKDL.Segment()
            joint = PyKDL.Joint(PyKDL.Joint.RotZ)
            frame = PyKDL.Frame()
            frame.p = PyKDL.Vector(a, 0, d)  # 设置平移部分
            frame.M = PyKDL.Rotation.RotZ(theta)
            frame.M =  frame.M * PyKDL.Rotation.RotX(alpha)  # 设置旋转部分
            link = PyKDL.Segment(joint, frame)
            self.bt_robot.addSegment(link)
        self.bt_fk_solver = PyKDL.ChainFkSolverPos_recursive(self.bt_robot)
        self.bt_ik_solver = PyKDL.ChainIkSolverPos_LMA(self.bt_robot)
        
        #top_to_base运动学求解器
        self.tb_robot = PyKDL.Chain()
        for params in self.__inverse_dh:
            theta, d, a, alpha = params
            link = PyKDL.Segment()
            joint = PyKDL.Joint(PyKDL.Joint.RotZ)
            frame = PyKDL.Frame()
            frame.p = PyKDL.Vector(a, 0, d)  # 设置平移部分
            frame.M = PyKDL.Rotation.RotZ(theta)
            frame.M =  frame.M * PyKDL.Rotation.RotX(alpha)  # 设置旋转部分
            link = PyKDL.Segment(joint, frame)
            self.tb_robot.addSegment(link)
        self.tb_fk_solver = PyKDL.ChainFkSolverPos_recursive(self.bt_robot)
        self.tb_ik_solver = PyKDL.ChainIkSolverPos_LMA(self.bt_robot)
    
    def deg2rad(self,degrees):
        return degrees * np.pi / 180.0
    
    def rad2deg(self,rad):
        return rad * 180 / np.pi



    def rotate_quaternion(self,original_quaternion, axis, angle):
        """
        绕指定轴旋转四元素

        Parameters:
        - original_quaternion (np.ndarray): 原始四元素
        - axis (str): 旋转轴，可选值为 'x', 'y', 'z'
        - angle (float): 旋转角度（弧度）

        Returns:
        - np.ndarray: 旋转后的四元素
        """
        # 将字符串轴映射到旋转向量
        axis_mapping = {'x': [1, 0, 0], 'y': [0, 1, 0], 'z': [0, 0, 1]}
        rotation_vector = np.array(axis_mapping[axis]) * angle

        # 使用旋转向量创建旋转矩阵
        rotation_matrix = Rotation.from_rotvec(rotation_vector).as_matrix()

        # 将当前四元素转换为旋转矩阵
        rotation_matrix_original = Rotation.from_quat(original_quaternion).as_matrix()

        # 计算新的旋转矩阵
        rotation_matrix_new = rotation_matrix @ rotation_matrix_original

        # 将新的旋转矩阵转换为四元素
        new_quaternion = Rotation.from_matrix(rotation_matrix_new).as_quat()

        return new_quaternion
   




