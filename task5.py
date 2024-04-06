import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm
from timeit import default_timer as timer 

class Robot():
    #defining the ETS tree of the robot    
    def __init__(self) -> None:
        E1 = rtb.ET.tz(0.333)
        E2 = rtb.ET.Rz()
        E3 = rtb.ET.Ry()
        E4 = rtb.ET.tz(0.316)
        E5 = rtb.ET.Rz()
        E6 = rtb.ET.tx(0.0825)
        E7 = rtb.ET.Ry(flip=True)
        E8 = rtb.ET.tx(-0.0825)
        E9 = rtb.ET.tz(0.384)
        E10 = rtb.ET.Rz()
        E11 = rtb.ET.Ry(flip=True)
        E12 = rtb.ET.tx(0.088)
        E13 = rtb.ET.Rx(np.pi)
        E14 = rtb.ET.tz(0.107)
        E15 = rtb.ET.Rz()
        self.panda=E1*E2*E3*E4*E5*E6*E7*E8*E9*E10*E11*E12*E13*E14*E15

#importing the specifics of the robot
robot=Robot().panda

#setting paramters for IK solvers
ilimit=1000
slimit=2000
tolerance=1e-8


print('Printing the forward kinematics matrix')

arr=list()

for i in range(7):
    print('Enter the value of joint angle q',i,' in degrees',sep='')
    arr.append(float(input()))

joint_arr=np.array(arr) #converting it into a numpy array
joint_arr=joint_arr*np.pi/180 #converting to radians

print('The joint angles are-')
print(joint_arr)

#finding the Jacobian matrix for a given joint angle configuration
start=timer()
geometric=robot.jacob0(joint_arr)
end=timer()

print('Geometric Jacobian computed in world frame')
print('Time taken:',round(end-start,6)*1000,'ms')
print(np.round(geometric,3))

start=timer()
analytical=robot.jacob0_analytical(joint_arr)
end=timer()

print('Analytical Jacobian computed in world frame')
print('Time taken:',round(end-start,6)*1000,'ms')
print(np.round(analytical,3))

#we will now try to compute the required joint velocities for a given end-effector velocity 

vel=list()

vx=float(input('Enter translational velocity in x direction '))
wx=float(input('Enter angular velocity along x axis '))

vy=float(input('Enter translational velocity in y direction '))
wy=float(input('Enter angular velocity along y axis '))

vz=float(input('Enter translational velocity in z direction '))
wz=float(input('Enter angular velocity along z axis '))

end_vel=np.array([vx,vy,vz,wx,wy,wz])

try:
    joint_vel=np.dot(np.linalg.pinv(geometric),end_vel) #calculating the inverse Jacobian
    print('The required joint velocities are:')
    print(np.round(joint_vel,6))
except:
    print('Invalid joint state configuration')


