# from std_msgs.msg import Float64
# import numpy as np

# ns = '/sc/joint'
# numofhand = 2
# numofJointH = 3
# ns = '/sc/joint'
# ## define Left & Right Hand Joint Publisher
# jLHpub = np.empty(numofJointH, dtype = Float64) #define Left Hand Publisher
# jRHpub = np.empty(numofJointH, dtype = Float64) #define Right Hand Publisher
# for i in range (numofJointH):
#     jLHpub[i] = (ns + 'L' + str(i+1) + '_position_controller/command')
#     jRHpub[i] = (ns + 'R' + str(i+1) + '_position_controller/command')
#     print(jLHpub[i], jRHpub[i], i)

# numofJointT = 5
# jTpub = np.empty(numofJointT, dtype = Float64)
# for i in range (numofJointT):
#     jTpub[i] = (ns + 'T' + str (i+1) + '_position_controller/command')
#     print(jTpub[i], i)

# ## define hand & tail Gripper 
# jLGpub = np.empty(2, dtype = Float64)
# jRGpub = np.empty(2, dtype = Float64)
# jTGpub = np.empty(2, dtype = Float64)
# for i in range (2):
#     jLGpub[i] = (ns + 'L3_' + str(i+1) + 'position_controller/command')
#     jRGpub[i] = (ns + 'R3_' + str(i+1) + 'position_controller/command')
#     jTGpub[i] = (ns + 'T5_' + str(i+1) + 'position_controller/command')
#     print(jLGpub[i], jRGpub[i], jTGpub[i], i)

# def test():
#     q = 0
#     while(q < 90):
#         q += 1
#         jlpub[0][2].publish(q*math.pi/180)
#         jlpub[1][2].publish(q*math.pi/180)
#         print(q)
#         rospy.sleep(0.5)
        
#     while(q>0):
#         q -= 1
#         jlpub[0][2].publish(q*math.pi/180)
#         jlpub[1][2].publish(q*math.pi/180)
#         print(q)
#         rospy.sleep(0.5)

# def testleg():
#     q = 0
#     while(q < 45):
#         q += 1
#         jlpub[0][2].publish(q*math.pi/180)
#         jlpub[1][2].publish(q*math.pi/180)
#         print(q)
#         rospy.sleep(0.1)
#     while(q>0):
#         q -= 1
#         jlpub[0][2].publish(q*math.pi/180)
#         jlpub[1][2].publish(q*math.pi/180)
#         print(q)
#         rospy.sleep(0.1)

# def testhand():
#     q = 0
#     while(q < 45):
#         q += 1
#         jRHpub[0].publish(q*math.pi/180)
#         jRHpub[1].publish(q*math.pi/180)
#         jRHpub[2].publish(q*math.pi/180)
#         jLHpub[0].publish(q*math.pi/180)
#         jLHpub[1].publish(q*math.pi/180)
#         jLHpub[2].publish(q*math.pi/180)
#         print(q)
#         rospy.sleep(0.1)
        
#     while(q > 0):
#         q -= 1
#         jRHpub[0].publish(q*math.pi/180)
#         jRHpub[1].publish(q*math.pi/180)
#         jRHpub[2].publish(q*math.pi/180)
#         jLHpub[0].publish(q*math.pi/180)
#         jLHpub[1].publish(q*math.pi/180)
#         jLHpub[2].publish(q*math.pi/180)
#         print(q)
#         rospy.sleep(0.1)

# def testtail():
#     q = 0
#     while(q < 45):
#         q += 1
#         jTpub[0].publish(q*math.pi/180)
#         jTpub[1].publish(q*math.pi/180)
#         jTpub[2].publish(q*math.pi/180)
#         jTpub[3].publish(q*math.pi/180)
#         jTpub[4].publish(q*math.pi/180)
#         print(q)
#         rospy.sleep(0.1)
#     while(q > 0):
#         q -= 1
#         jTpub[0].publish(q*math.pi/180)
#         jTpub[1].publish(q*math.pi/180)
#         jTpub[2].publish(q*math.pi/180)
#         jTpub[3].publish(q*math.pi/180)
#         jTpub[4].publish(q*math.pi/180)
#         print(q)
#         rospy.sleep(0.1)

# def testGripper():
#     q = 0
#     while(q < 45):
#         q += 1
#         jLGpub[0].publish(q*math.pi/180)
#         jRGpub[0].publish(q*math.pi/180)
#         jTGpub[0].publish(q*math.pi/180)
#         print(q)
#         rospy.sleep(0.1)
#     while(q > 0):
#         q -= 1
#         jLGpub[0].publish(q*math.pi/180)
#         jRGpub[0].publish(q*math.pi/180)
#         jTGpub[0].publish(q*math.pi/180)
#         print(q)
#         rospy.sleep(0.1)



# rospy.loginfo("test leg Start")
#         testleg()
#         rospy.loginfo("test leg Finish")
#         rospy.sleep(10)
#         rospy.loginfo("test hand Start")
#         testhand()
#         rospy.loginfo("test hand Finish")
#         rospy.sleep(10)
#         rospy.loginfo("test tail Start")
#         testtail()
#         rospy.loginfo("test tail Finish")
#         rospy.sleep(10)
#         rospy.loginfo("test Gripper Start")
#         testGripper()
#         rospy.loginfo("test Gripper Finish")
#         rospy.sleep(10)

# def PID(self,P,I,D,sample,feedback): #feedback about IMU sensor roll pitch yaw ??? acceleration ??? 
#     self.Kp = P
#     self.Ki = I
#     self.Kd = D
#     self.current_time = time.time()
#     def clear(self):
#         self.PTerm = 0.0
#         self.ITerm = 0.0 
#         self.DTerm = 0.0
#         self.last_error = 0.0
#         self.int_error = 0.0

#     u = {}
#     e = {}
#     Ts = 0
#     for i in range(sample):
#         u[i] = self.Kp*e[i]+self.Ki*Ts*e[i]+self.Kd/Ts*(e[i]-e[i-1])
import PID
import time
import matplotlib.pyplot as plt
import numpy as np
#from scipy.interpolate import spline
from scipy.interpolate import BSpline, make_interp_spline #  Switched to BSpline

# def test_pid(P,I,D,L):
#     """Self-test PID class
#     .. note::
#         ...
#         for i in range(1, END):
#             pid.update(feedback)
#             output = pid.output
#             if pid.SetPoint > 0:
#                 feedback += (output - (1/i))
#             if i>9:
#                 pid.SetPoint = 1
#             time.sleep(0.02)
#         ---
#     """
#     pid = PID.PID(P, I, D)

#     pid.SetPoint=0.0
#     pid.setSampleTime(0.01)

#     END = L
#     feedback = 0

#     feedback_list = []
#     time_list = []
#     setpoint_list = []

#     for i in range(1, END):
#         pid.update(feedback)
#         output = pid.output
#         if pid.SetPoint > 0:
#             feedback += (output - (1/i))
#         if i>9:
#             pid.SetPoint = 1
#         time.sleep(0.02)

#         feedback_list.append(feedback)
#         setpoint_list.append(pid.SetPoint)
#         time_list.append(i)

#     time_sm = np.array(time_list)
#     time_smooth = np.linspace(time_sm.min(), time_sm.max(), 300)

#     # feedback_smooth = spline(time_list, feedback_list, time_smooth)
#     # Using make_interp_spline to create BSpline
#     helper_x3 = make_interp_spline(time_list, feedback_list)
#     feedback_smooth = helper_x3(time_smooth)

#     plt.plot(time_smooth, feedback_smooth)
#     plt.plot(time_list, setpoint_list)
#     plt.xlim((0, L))
#     plt.ylim((min(feedback_list)-0.5, max(feedback_list)+0.5))
#     plt.xlabel('time (s)')
#     plt.ylabel('PID (PV)')
#     plt.title('TEST PID')

#     plt.ylim((1-0.5, 1+0.5))

#     plt.grid(True)
#     plt.show()

# if __name__ == "__main__":
#     test_pid(1.2, 1, 0.001, L=50)
# #    test_pid(0.8, L=50)

# import math
# tf  = 1
# timestep = 0.001
# def cubic(thetai, thetaf, tf, t):
#     a0 = thetai
#     a1 = 0
#     a2 = 3/(pow(tf,2))*(thetaf - thetai)
#     a3 = -2/(pow(tf,3))*(thetaf - thetai)
#     theta0 = a0 + a1*(t) +a2*(pow(t,2)) +a3*(pow(t,3))
#     return theta0

# for t in np.arange(0,tf,timestep):
#     theta15 = cubic(-15,15,1,t)#*math.pi/180
#     theta_15 = cubic(15,-15,1,t)#*math.pi/180
#     theta20 = cubic(10,20,1,t)#*math.pi/180
#     theta10 = cubic(20,10,1,t)#*math.pi/180
#     print(theta15,theta_15,theta20,theta10)
#     print(t)

# import math

# def IKIN(x,y,z):
#     # q3 = math.acos((pow(x,2)+pow(y,2)+pow(z,2)-pow(l1,2)-pow(l2,2))/(2*l1*l2))
#     # q2 = math.atan(z*(l2*math.cos(q3)+l1)-math.sqrt(pow(x,2)+pow(y,2)*(l2*math.sin(q3))))
#     # q1 = x/y
#     l1 = 0.0145
#     l2 = 0.070
#     l3 = 0.113
#     q1 = math.atan2(y,x)
#     niama = (math.cos(q1)*x+math.sin(q1)*y-l1)
#     q3 = math.asin((pow(niama,2)+pow(z,2)-pow(l3,2)-pow(l2,2))/(2*l2*l3))-math.pi/2
#     niasing = (l3*math.cos(q3)+l2)
#     gama = math.atan2(l3*math.sin(q3),(l3*math.cos(q3)+l2))
#     q2 = math.asin(z/math.sqrt(pow(niasing,2)+pow((l3*math.sin(q3)),2)))-gama
#     q = [-90+q1*180/math.pi,-45+q2*180/math.pi,-q3*180/math.pi]
#     q1 = [q1*180/math.pi,-q2*180/math.pi,q3*180/math.pi]
#     print(q)
#     print(q1)
#     return q

# IKIN(0.05,0.01,0.05)
# import math 
# import numpy as np

# q1,q2,q3 =  12,15,13
# l1, l2, l3 = 0.0145,0.07,0.113
# s1 = math.sin(q1) 
# s2 = math.sin(q2)
# s3 = math.sin(q3)
# c1 = math.cos(q1)
# c2 = math.cos(q2)
# c3 = math.cos(q3)
# c23 = math.cos(q2+q3)
# s23 = math.sin(q2+q3)
# T03 = np.array([[c1*c23, -c1*s23,   s1,   l1*c1+l2*c1*c2+l3*c1*c23],
#                    [s1*c23, -s1*s23,  -c1,    l1*s1+l2*s1*c2+l3*s1*c23],
#                    [s23,       c23 ,    0,    l2*s2+l3*s23           ],
#                    [0  ,        0  ,    0,          1                ]])
# print(T03)
