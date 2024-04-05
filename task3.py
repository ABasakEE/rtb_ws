import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
from timeit import default_timer as timer

#importing the specifics of the robot
robot=rtb.models.Panda()

print('Printing the forward kinematics matrix')
print('Robot starts at its ready pose')

#setting initial joint coordinates to ready pose
robot.q=robot.qr

fk=np.eye(4) #assigning a 4x4 matrix to store the forward kinematics
fk=robot.fkine(robot.q)

arr=list()

for i in range(7):
    print('Enter the value of joint angle q',i,' in degrees',sep='')
    arr.append(float(input()))

joint_arr=np.array(arr,'\n') #converting it into a numpy array
joint_arr=joint_arr*np.pi/180 #converting to radians

print('The joint angles are-')
print(joint_arr)
#setting initial joint coordinates to ready pose
robot.q=robot.qr

fk=np.eye(4) #assigning a 4x4 matrix to store the forward kinematics
fk=robot.fkine(robot.q)


for et in robot:
    if et.isjoint:
        # This ET is a variable joint
        # Use the q array to specify the joint angle for the variable ET
        fk = fk @ et.A(joint_arr[et.jindex])
    else:
        # This ET is static
        fk = fk @ et.A()

#converting to an SE3 object
print('\nThe forward kinematics matrix is:')
print(sm.SE3(fk))

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
print(sm.SE3(Tep))

print('Using fast Levenberg-Marquadt method')
start=timer()
sol=robot.ik_LM(Tep)
end=timer()
print('Elapsed time:',round((end-start)*1000,3),'ms')

if (sol[1]): #checks if solution exists
    print('Solution found')
    joint_arr=np.array(sol[0])
    joint_arr=joint_arr*180/np.pi
    print('Joint angles in degrees:')
    print(joint_arr)
else:
    print('No solution found')


print('Using fast Gauss-Newton method')
start=timer()
sol=robot.ik_GN(Tep)
end=timer()
print('Elapsed time:',round((end-start)*1000,3),'ms')

if (sol[1]): #checks if solution exists
    print('Solution found')
    joint_arr=np.array(sol[0])
    joint_arr=joint_arr*180/np.pi
    print('Joint angles in degrees:')
    print(joint_arr)
else:
    print('No solution found')

print('Using fast Newton-Raphson method')
start=timer()
sol=robot.ik_NR(Tep)
end=timer()
print('Elapsed time:',round((end-start)*1000,3),'ms')

if (sol[1]): #checks if solution exists
    print('Solution found')
    joint_arr=np.array(sol[0])
    joint_arr=joint_arr*180/np.pi
    print('Joint angles in degrees:')
    print(joint_arr)
else:
    print('No solution found')

