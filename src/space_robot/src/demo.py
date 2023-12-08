
from robot import Robot
import math

my_robot = Robot()
my_robot.check_robot_status()

max_vel,max_acc = 1, 1
q_tagret1 =[0,0,math.pi/4,math.pi/2,-math.pi/4,0,0]
q_tagret2 =[0,0,-3*math.pi/4,-math.pi/2,-math.pi/4,0,0]
q_tagret3 =[0,0,-math.pi/4,-math.pi/2,-3*math.pi/4,0,0]
q_tagret4 =[0,0,-math.pi/4,math.pi/2,math.pi/4,0,0]



my_robot.moveJ(q_tagret1,max_vel,max_acc)
my_robot.delay(1)

my_robot.switch()
my_robot.moveJ(q_tagret2,max_vel,max_acc)
my_robot.delay(1)


my_robot.switch()
my_robot.moveJ(q_tagret1,max_vel,max_acc)
my_robot.moveJ(q_tagret3,max_vel,max_acc)
my_robot.delay(1)

my_robot.switch()
my_robot.moveJ(q_tagret4,max_vel,max_acc)




my_robot.model_to_initial()