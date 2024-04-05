import roboticstoolbox as rtb

#importing and printing the specifics of the robot
panda=rtb.models.Panda()
print(panda) 

#computing the FK for an already defined qr
Te = panda.fkine(panda.qr)  # forward kinematics
print(Te)

#animating the movement of robot from one pose to another
qt = rtb.jtraj(panda.qr, panda.qz, 100)
panda.plot(qt.q,limits=[-0.5,0.8,-0.5,0.8,0,1.0],backend='pyplot',dt=0.02,vellipse=True,jointaxes=True,eeframe=True,shadow=True)
