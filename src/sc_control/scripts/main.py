#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np
import array as arr
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import ros
import time
import PID


# l1 = 10 
# l2 = 10 
offthigh = -math.pi/2
offmidleg = -math.pi/4
offcalf = -1 ##Direction factor

ns = '/sc/joint' 
numofleg = 6
numofjointL = 3
# define 6leg with 3 Joint Publisher
# jlpub = np.empty((numofjointL,numofleg), dtype = Float64)
jlpub = np.empty((numofleg,numofjointL),dtype = Float64)
for i in range(numofleg):
    for j in range(numofjointL):
        jlpub[i][j] = rospy.Publisher(ns + str(i+1)+'_'+str(j+1)+'position_controller/command', Float64, queue_size = 1)
numofhand = 2
numofJointH = 3
## define Left & Right Hand Joint Publisher
jLHpub = np.empty(numofJointH, dtype = Float64) #define Left Hand Publisher
jRHpub = np.empty(numofJointH, dtype = Float64) #define Right Hand Publisher
for i in range (numofJointH):
    jLHpub[i] = rospy.Publisher(ns + 'L' + str(i+1) + '_position_controller/command',Float64, queue_size = 1)
    jRHpub[i] = rospy.Publisher(ns + 'R' + str(i+1) + '_position_controller/command',Float64, queue_size= 1)
numofJointT = 5
jTpub = np.empty(numofJointT, dtype = Float64)
for i in range (numofJointT):
    jTpub[i] = rospy.Publisher(ns + 'T' + str (i+1) + '_position_controller/command',Float64, queue_size= 1)
# ## define hand & tail Gripper 
jLGpub = np.empty(2, dtype = Float64)
jRGpub = np.empty(2, dtype = Float64)
jTGpub = np.empty(2, dtype = Float64)
for i in range (2):
    jLGpub[i] = rospy.Publisher(ns + 'L3_' + str(i+1) + 'position_controller/command',Float64, queue_size= 1)
    jRGpub[i] = rospy.Publisher(ns + 'R3_' + str(i+1) + 'position_controller/command',Float64, queue_size= 1)
    jTGpub[i] = rospy.Publisher(ns + 'T5_' + str(i+1) + 'position_controller/command',Float64, queue_size= 1)

def initposition():
    for i in range(numofleg):
        jlpub[i][2].publish(0*math.pi/180) #Calf
        jlpub[i][1].publish(offmidleg*math.pi/180) #Mid leg
        jlpub[i][0].publish(0*math.pi/180) #Thighs
        

# def starup():
#     for i in range (numofleg):
def test():
    for i in range(0,180):
        jlpub[0][2].publish((-i)*math.pi/180) #Calf
        jlpub[0][1].publish((-45+i)*math.pi/180) #Mid leg
        jlpub[0][0].publish((-90+i)*math.pi/180) #Thighs
        rospy.sleep(0.1)


def movingup_down():
        theta = 45
        while (theta > 0):
            theta -= 1 
            for i in range(numofleg):
                jlpub[i][1].publish(theta*math.pi/180) #Mid leg
                # jlpub[i][2].publish(0*math.pi/180) #Calf
            rospy.sleep(0.1)
            print(theta)
        while(theta < -90):
            theta += 1 
            for i in range(numofleg):
                jlpub[i][1].publish(theta*math.pi/180) #Mid leg
                # jlpub[i][2].publish(0*math.pi/180) #Calf
            rospy.sleep(0.1)
            print(theta)

def IKIN(x,y,z):
    # q3 = math.acos((pow(x,2)+pow(y,2)+pow(z,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2))
    # q2 = math.atan(z*(l2*math.cos(q3)+l1)-math.sqrt(pow(x,2)+pow(y,2)*(l2*math.sin(q3))))
    # q1 = x/y
    # x = l1*math.sin(q1)*math.cos(q2)+l2*math.cos(q1)*math.cos(q2)*math.cos(q3) - l2*math.sin(q1)*math.sin(q2)*math.sin(q3)
    # y = l1*math.cos(q1)*math.cos(q2)+l2*math.cos(q1)*math.cos(q2)*math.cos(q3) -l2*math.cos(q1)*math.sin(q2)*math.sin(q3)
    # z = l1*math.sin(q2)+l2*math.sin(q2)*cos(q3)+l2*math.sin(q2)*math.sin(q3)
    l1 = 0.0145
    l2 = 0.070 
    l3 = 0.113
    q1 = math.atan2(y,x)
    niama = (math.cos(q1)*x+math.sin(q1)*y-l1)
    q3 = math.asin((pow(niama,2)+pow(z,2)-pow(l3,2)-pow(l2,2))/(2*l2*l3))-math.pi/2
    niasing = (l3*math.cos(q3)+l2)
    gama = math.atan2(l3*math.sin(q3),(l3*math.cos(q3)+l2))
    q2 = math.asin(z/math.sqrt(pow(niasing,2)+pow((l3*math.sin(q3)),2)))-gama
    # q = [offthigh+q1,offmidleg+q2,offcalf*q3]
    q = [q1,q2,q3]
    qx = [q1*180/math.pi,q2*180/math.pi,q3*180/math.pi]
    print(qx)
    return q

def IKIN_control():
    q  = {}
    q = IKIN(0.1612,0.1128,-0.004)
    jlpub[3][0].publish(q[0])
    jlpub[3][1].publish(q[1])
    jlpub[3][2].publish(q[2])
    print(q)
    rospy.sleep(3)

### May be ? 
# def CheckFKin():
#     q1,q2,q3 =  {},{},{}
#     x,y,z = {},{},{}
#     l1, l2, l3 = 0.0145,0.07,0.113
#     s1 = math.sin(q1) 
#     s2 = math.sin(q2)
#     s3 = math.sin(q3)
#     c1 = math.cos(q1)
#     c2 = math.cos(q2)
#     c3 = math.cos(q3)
#     c23 = math.cos(q2+q3)
#     s23 = math.sin(q2+q3)
#     T03 = np.array([[c1*c23, -c1*s23,   s1,   l1*c1+l2*c1*c2+l3*c1*c23],
#                    [s1*c23, -s1*s23,  -c1,    l1*s1+l2*s1*c2+l3*s1*c23],
#                    [s23,       c23 ,    0,    l2*s2+l3*s23           ],
#                    [0  ,        0  ,    0,          1                ]])
#     for q1 in np.arange(-math.pi/4,math.pi/4,10):
#         for q2 in np.arange(-math.pi/4,3*math.pi/4,10):
#             for q3 in np.arange(0,math.pi):
#                 Object = eval(T03*np.array[[0],[0],[0],[1]])
#                 i = i+1
#                 x[i] = Object(1)
#                 y[i] = Object(1)
#                 z[]


def PID_Regulation(P,I,D,L,theta): ##theta from Inverse Kinematic (IKIN function)
    pid = PID.PID(P,I,D)
    pid.SetPoint = 0.0 
    pid.setSampleTime(0.01)
    END = L ## L  = length time ??
    feedback = 0 
    feedback_list = []
    time_list = []
    setpoint_list = []
    for i in range(1,END):
        pid.update(feedback)
        output = pid.output
        if pid.SetPoint > 0:
            feedback += (output-(1/i))
            if i > 9:
                pid.SetPoint = 1
            time.sleep(0.02)
            feedback_list.append(feedback)
            setpoint_list.append(pid.SetPoint)
            time_list.append(i)
            time_sm = np.array(time_list)
            time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)
    return theta


# def walk_forward():
#     # for i in range(numofleg):
#     #     jlpub[i][2].publish(-45*math.pi/180) ##Calf
#     #     jlpub[i][1].publish(20*math.pi/180) ##Mid Leg
#     # jTpub[1].publish(-45*math.pi/180)
#     # jTpub[2].publish(-45*math.pi/180)
#     x = 0 
#     while (x<10):
#         for i in range(numofjointL):
#             # jlpub[2*i+1][0].publish(offthigh+45*math.pi/180)  #Femur leg 2,4,6 motion 
#             jlpub[2*i+1][0].publish(-15*math.pi/180)  #Femur leg 2,4,6 motion 
#             jlpub[2*i+1][1].publish(10*math.pi/180) #Mid leg 2,4,5motion
#             jlpub[2*i+1][2].publish(-20*math.pi/180) #Calf 2,4,6 motion

#             # jlpub[2*i][0].publish(offthigh+135*math.pi/180) #Femur leg 1,3,5 motion
#             jlpub[2*i][0].publish(15*math.pi/180) #Femur leg 1,3,5 motion
#             jlpub[2*i][1].publish(-20*math.pi/180) #Mid leg 1,3,5 Motion
#             jlpub[2*i][2].publish(-20*math.pi/180) #Calf 1,3,5 motion

#         rospy.sleep(2)
#         for i in range(numofjointL):
#             jlpub[2*i+1][0].publish(15*math.pi/180)  #Femur leg 2,4,6 motion 
#             jlpub[2*i+1][1].publish(-20*math.pi/180) #Mid leg 2,4,5motion
#             jlpub[2*i+1][2].publish(-20*math.pi/180) #Calf 2,4,6 motion
            
#             jlpub[2*i][0].publish(-15*math.pi/180) #Femur leg 1,3,5 motion
#             jlpub[2*i][1].publish(10*math.pi/180) #Mid leg 1,3,5 Motion
#             jlpub[2*i][2].publish(-20*math.pi/180) #Calf 1,3,5 motion
#         rospy.sleep(2)
#         print(x)
#         x += 1

def walk_forward():
    x = 0 
    while (x<10):
        for i in range(numofjointL):
            # jlpub[2*i+1][0].publish(-15*math.pi/180)  #Femur leg 2,4,6 motion 
            jlpub[1][0].publish(15*math.pi/180)
            jlpub[3][0].publish(-15*math.pi/180)
            jlpub[5][0].publish(-15*math.pi/180)
            jlpub[2*i+1][1].publish(20*math.pi/180) #Mid leg 2,4,5motion
            jlpub[2*i+1][2].publish(-20*math.pi/180) #Calf 2,4,6 motion

            # jlpub[2*i][0].publish(offthigh+135*math.pi/180) #Femur leg 1,3,5 motion
            jlpub[0][0].publish(-15*math.pi/180)
            jlpub[2][0].publish(-15*math.pi/180)
            jlpub[4][0].publish(15*math.pi/180)
            jlpub[2*i][1].publish(10*math.pi/180) #Mid leg 1,3,5 Motion
            jlpub[2*i][2].publish(-20*math.pi/180) #Calf 1,3,5 motion

        rospy.sleep(2)
        for i in range(numofjointL):
            # jlpub[2*i+1][0].publish(-15*math.pi/180)  #Femur leg 2,4,6 motion 
            jlpub[1][0].publish(-15*math.pi/180)
            jlpub[3][0].publish(15*math.pi/180)
            jlpub[5][0].publish(15*math.pi/180)
            jlpub[2*i+1][1].publish(10*math.pi/180) #Mid leg 2,4,5motion
            jlpub[2*i+1][2].publish(-20*math.pi/180) #Calf 2,4,6 motion

            # jlpub[2*i][0].publish(offthigh+135*math.pi/180) #Femur leg 1,3,5 motion
            jlpub[0][0].publish(15*math.pi/180)
            jlpub[2][0].publish(15*math.pi/180)
            jlpub[4][0].publish(-15*math.pi/180)
            jlpub[2*i][1].publish(20*math.pi/180) #Mid leg 1,3,5 Motion
            jlpub[2*i][2].publish(-20*math.pi/180) #Calf 1,3,5 motion
        rospy.sleep(2)
        print(x)
        x += 1

def walk_forward_cubicSpline():
    timestep = 0.001
    tf = 1
    ## Mid leg from 10 to -20 and from -20 to 10 
    ## 
    x = 0
    while (x<10):
        for i in range(numofjointL):
            # jlpub[2*i+1][0].publish(-15*math.pi/180)  #Femur leg 2,4,6 motion 
            for t in np.arange(0,tf,timestep):
                theta15 = cubic(-15,15,2,t)*math.pi/180
                theta_15 = cubic(15,-15,2,t)*math.pi/180
                theta20 = cubic(10,20,2,t)*math.pi/180
                theta10 = cubic(20,10,2,t)*math.pi/180
                ##Calf 
                jlpub[i][2].publish(-20*math.pi/180)
                ##Thigh
                jlpub[0][0].publish(theta_15)
                jlpub[1][0].publish(theta15)
                jlpub[2][0].publish(theta_15) 
                jlpub[3][0].publish(theta_15)
                jlpub[4][0].publish(theta15)
                jlpub[5][0].publish(theta_15)
                ##Mid Leg
                jlpub[2*i+1][1].publish(theta20)
                jlpub[2*i][1].publish(theta10)
                # rospy.sleep(timestep)
        for i in range(numofjointL):
            for t in np.arange(0,tf,timestep):
                theta15 = cubic(-15,15,2,t)*math.pi/180
                theta_15 = cubic(15,-15,2,t)*math.pi/180
                theta20 = cubic(10,20,2,t)*math.pi/180
                theta10 = cubic(20,10,2,t)*math.pi/180
                ##Calf 
                jlpub[i][2].publish(-20*math.pi/180)
                ##Thigh
                jlpub[0][0].publish(theta15)
                jlpub[1][0].publish(theta_15)
                jlpub[2][0].publish(theta15) 
                jlpub[3][0].publish(theta15)
                jlpub[4][0].publish(theta_15)
                jlpub[5][0].publish(theta15)
                ##Mid Leg
                jlpub[2*i+1][1].publish(theta10)
                jlpub[2*i][1].publish(theta20)
                # rospy.sleep(timestep)
        print(x)
        x += 1

            
def cubic(thetai, thetaf, tf, t):
    a0 = thetai
    a1 = 0
    a2 = 3/(pow(tf,2))*(thetaf - thetai)
    a3 = -2/(pow(tf,3))*(thetaf - thetai)
    theta0 = a0 + a1*(t) +a2*(pow(t,2)) +a3*(pow(t,3))
    return theta0


if __name__ == '__main__':
    rospy.init_node('main', anonymous=True)
    rospy.sleep(1)
    rospy.loginfo("Scorpion Robot Start")
    rospy.loginfo("ehehehehehhehehehhehehehehehehehehehe")
    try:
        initposition()
        rospy.sleep(5)
        # walk_forward()
        # walk_forward_cubicSpline()
        # rospy.sleep(5)
        # movingup_down()
        # rospy.sleep(5)
        # test()
        # rospy.loginfo("Ikin test")
        # IKIN_control()
        # rospy.sleep(5)
    except rospy.ROSInterruptException:
        pass

