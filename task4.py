import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np

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
    
#here we will explore different algorithms for generating trajectory paths
robot=Robot().panda

#setting paramters for IK solvers
ilimit=100
slimit=200
tolerance=1e-7
steps=200

#first we will take the input and output poses 
print('Initial end effector position with respect to world frame')
x=float(input('Enter x coordinate '))
y=float(input('Enter y coordinate '))
z=float(input('Enter z coordinate '))

print('\nEnter the desired rotation in ZYX Euler angle format ')
Rz=float(input('Enter z rotation in degrees '))
Ry=float(input('Enter y rotation in degrees '))
Rx=float(input('Enter x rotation in degrees '))

#converting to radians
Rz,Ry,Rx=Rz*np.pi/180,Ry*np.pi/180,Rx*np.pi/180

Te0=sm.SE3.Tx(x)*sm.SE3.Ty(y)*sm.SE3.Tz(z)*sm.SE3.Rz(Rz)*sm.SE3.Ry(Ry)*sm.SE3.Rx(Rx)
print('Target matrix:')
print(sm.SE3(Te0))

sol1,sol2=None,None

#we will use the IK_LM method here
sol1=robot.ik_LM(Tep=Te0,ilimit=ilimit,slimit=slimit,tol=tolerance)
flag1,flag2=True,True

if (not sol1[1]):
    flag1=False
    print('Invalid joint state configuration')

if (flag1):
    #taking the output pose 
    print('Final end effector position with respect to world frame')
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
    
    sol2=robot.ik_LM(Tep=Tep,ilimit=ilimit,slimit=slimit,tol=tolerance)
    if (not sol2):
        print('Invalid joint state configuration')
        flag=False

if (flag1 and flag2):
    print('Using a quintic polynomial to generate the trajectory')
    q0=sol1[0]
    q1=sol2[0]
    
    #generating the path
    path_q=rtb.jtraj(q0,q1,steps)
    
    robot.plot(path_q.q,name='Quintic',limits=[-0.5,0.8,-0.5,0.8,0,1.0],backend='pyplot',dt=0.02,vellipse=True,jointaxes=True,eeframe=True,shadow=True)
    
    
    print('Using a trapezoidal path plan')
    path_t=rtb.ctraj(Te0,Tep,steps)
    robot.plot(path_q.q,name='Quintic',limits=[-0.5,0.8,-0.5,0.8,0,1.0],backend='pyplot',dt=0.02,vellipse=True,jointaxes=True,eeframe=True,shadow=True)
    
    

    
    





