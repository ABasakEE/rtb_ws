import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
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
    pass


def forward(joint_arr,robot)->sm.SE3:
    fk=np.eye(4)
    for et in robot:
        if et.isjoint:
            # This ET is a variable joint
            # Use the q array to specify the joint angle for the variable ET
            fk = fk @ et.A(joint_arr[et.jindex])
        else:
            # This ET is static
            fk = fk @ et.A()
    return sm.SE3(fk)

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


#converting to an SE3 object
print('\nThe forward kinematics matrix is:')
print(forward(joint_arr,robot))

print ('Exploring inverse kinematics capabilities')
print('End effector position with respect to world frame')
x=float(input('Enter x coordinate '))
y=float(input('Enter y coordinate '))
z=float(input('Enter z coordinate '))

print('\nEnter the desired rotation in ZYX Euler angle format ')
Rz=float(input('Enter z rotation in degrees '))
Ry=float(input('Enter y rotation in degrees '))
Rx=float(input('Enter x rotation in degrees '))

#converting to radians
Rz,Ry,Rx=Rz*np.pi/180,Ry*np.pi/180,Rx*np.pi/180

Tep=sm.SE3.Tx(x)*sm.SE3.Ty(y)*sm.SE3.Tz(z)*sm.SE3.Rz(Rz)*sm.SE3.Ry(Ry)*sm.SE3.Rx(Rx)
print('Target matrix:')
print(sm.SE3(Tep))

print('Using fast Levenberg-Marquadt method')
start=timer()
sol=robot.ik_LM(Tep,ilimit=ilimit,slimit=slimit,tol=tolerance)
end=timer()
print('Elapsed time:',round((end-start)*1000,3),'ms')

if (sol[1]): #checks if solution exists
    print('Solution found')
    joint_arr=np.array(sol[0])
    joint_arr_deg=joint_arr*180/np.pi
    print('Joint angles in degrees:')
    print(joint_arr_deg)
    print('Forward kinematics matrix:')
    print(robot.fkine(joint_arr))
else:
    print('No solution found')


print('Using fast Gauss-Newton method')
start=timer()
sol=robot.ik_GN(Tep,ilimit=ilimit,slimit=slimit,tol=tolerance)
end=timer()
print('Elapsed time:',round((end-start)*1000,3),'ms')

if (sol[1]): #checks if solution exists
    print('Solution found')
    joint_arr=np.array(sol[0])
    joint_arr_deg=joint_arr*180/np.pi
    print('Joint angles in degrees:')
    print(joint_arr_deg)
    print('Forward kinematics matrix:')
    print(robot.fkine(joint_arr))
else:
    print('No solution found')

print('Using fast Newton-Raphson method')
start=timer()
sol=robot.ik_NR(Tep,ilimit=ilimit,slimit=slimit,tol=tolerance)
end=timer()
print('Elapsed time:',round((end-start)*1000,3),'ms')

if (sol[1]): #checks if solution exists
    print('Solution found')
    joint_arr=np.array(sol[0])
    joint_arr_deg=joint_arr*180/np.pi
    print('Joint angles in degrees:')
    print(joint_arr_deg)
    print('Forward kinematics matrix:')
    print(robot.fkine(joint_arr))
else:
    print('No solution found')
    