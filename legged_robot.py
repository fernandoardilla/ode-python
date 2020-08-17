#!/usr/bin/python
# -*- coding: utf-8 -*-



import ctypes

import drawstuffpy as ds
import numpy as np
import odepy
import random
import math
import array as arr

class Env(object):
    def __init__(self, world, space):
        q = odepy.dQuaternion()
        odepy.dQFromAxisAndAngle(q, 0, 0, 1, 0)


class Robot(object):

    def __init__(self, world, space):
        
        e45 = 0.707106781

        Body = [0.08, 0.05, 1.0]
        self.Coxa   = 0.03
        self.Femur  = 0.10
        self.Tibia  = 0.15
        link1=[ 0.015,  self.Coxa  , 0.2]
        link2=[ 0.010,  self.Femur , 0.2]
        link3=[ 0.005,  self.Tibia , 0.2]
        Link0 = e45 * Body[0]
        pz=0.25
        posBaseLeg=[[ Link0,-Link0,pz],
                    [-Link0,-Link0,pz],
                    [-Link0, Link0,pz],
                    [ Link0, Link0,pz]]
        
        # ジオメトリの作成
        self.__Body   = AddCylinder(world, space, r=Body[0] , l=Body[1] , m=Body[2] , px=0.0, py=0.0, pz=pz)
        self.__link01 = AddCapsule (world, space, r=link1[0], l=link1[1], m=link1[2], px=posBaseLeg[0][0], py=posBaseLeg[0][1]                              , pz=posBaseLeg[0][2])
        self.__link02 = AddCapsule (world, space, r=link2[0], l=link2[1], m=link2[2], px=posBaseLeg[0][0], py=posBaseLeg[0][1]-(link1[1]/2+link2[1]/2)      , pz=posBaseLeg[0][2])
        self.__link03 = AddCapsule (world, space, r=link3[0], l=link3[1], m=link3[2], px=posBaseLeg[0][0], py=posBaseLeg[0][1]-(link1[1]/2+link2[1]+link3[1]/2), pz=posBaseLeg[0][2])
        
        self.__joint01 = self.AddJoint(world, odepy.dGeomGetBody(self.__Body)  , odepy.dGeomGetBody(self.__link01), [posBaseLeg[0][0],posBaseLeg[0][1]                      ,posBaseLeg[0][2]], [ 0, 0,-1])
        self.__joint02 = self.AddJoint(world, odepy.dGeomGetBody(self.__link01), odepy.dGeomGetBody(self.__link02), [posBaseLeg[0][0],posBaseLeg[0][1]-link1[1]/2           ,posBaseLeg[0][2]], [ 1, 0, 0])
        self.__joint03 = self.AddJoint(world, odepy.dGeomGetBody(self.__link02), odepy.dGeomGetBody(self.__link03), [posBaseLeg[0][0],posBaseLeg[0][1]-(link1[1]/2+link2[1]),posBaseLeg[0][2]], [ 1, 0, 0])

        self.__link11 = AddCapsule (world, space, r=link1[0], l=link1[1], m=link1[2], px=posBaseLeg[1][0], py=posBaseLeg[1][1]                       , pz=posBaseLeg[1][2])
        self.__link12 = AddCapsule (world, space, r=link2[0], l=link2[1], m=link2[2], px=posBaseLeg[1][0], py=posBaseLeg[1][1]-(link1[1]/2+link2[1]/2)      , pz=posBaseLeg[1][2])
        self.__link13 = AddCapsule (world, space, r=link3[0], l=link3[1], m=link3[2], px=posBaseLeg[1][0], py=posBaseLeg[1][1]-(link1[1]/2+link2[1]+link3[1]/2), pz=posBaseLeg[1][2])
        
        self.__joint11 = self.AddJoint(world, odepy.dGeomGetBody(self.__Body)  , odepy.dGeomGetBody(self.__link11), [posBaseLeg[1][0],posBaseLeg[1][1]                      ,posBaseLeg[1][2]], [ 0, 0,-1])
        self.__joint12 = self.AddJoint(world, odepy.dGeomGetBody(self.__link11), odepy.dGeomGetBody(self.__link12), [posBaseLeg[1][0],posBaseLeg[1][1]-link1[1]/2           ,posBaseLeg[1][2]], [ 1, 0, 0])
        self.__joint13 = self.AddJoint(world, odepy.dGeomGetBody(self.__link12), odepy.dGeomGetBody(self.__link13), [posBaseLeg[1][0],posBaseLeg[1][1]-(link1[1]/2+link2[1]),posBaseLeg[1][2]], [ 1, 0, 0])

        self.__link21 = AddCapsule (world, space, r=link1[0], l=link1[1], m=link1[2], px=posBaseLeg[2][0], py=posBaseLeg[2][1]                       , pz=posBaseLeg[2][2])
        self.__link22 = AddCapsule (world, space, r=link2[0], l=link2[1], m=link2[2], px=posBaseLeg[2][0], py=posBaseLeg[2][1]+(link1[1]/2+link2[1]/2)      , pz=posBaseLeg[2][2])
        self.__link23 = AddCapsule (world, space, r=link3[0], l=link3[1], m=link3[2], px=posBaseLeg[2][0], py=posBaseLeg[2][1]+(link1[1]/2+link2[1]+link3[1]/2), pz=posBaseLeg[2][2])
        
        self.__joint21 = self.AddJoint(world, odepy.dGeomGetBody(self.__Body)  , odepy.dGeomGetBody(self.__link21), [posBaseLeg[2][0],posBaseLeg[2][1]                      ,posBaseLeg[2][2]], [ 0, 0, 1])
        self.__joint22 = self.AddJoint(world, odepy.dGeomGetBody(self.__link22), odepy.dGeomGetBody(self.__link21), [posBaseLeg[2][0],posBaseLeg[2][1]+link1[1]/2           ,posBaseLeg[2][2]], [ 1, 0, 0])
        self.__joint23 = self.AddJoint(world, odepy.dGeomGetBody(self.__link23), odepy.dGeomGetBody(self.__link22), [posBaseLeg[2][0],posBaseLeg[2][1]+(link1[1]/2+link2[1]),posBaseLeg[2][2]], [ 1, 0, 0])

        self.__link31 = AddCapsule (world, space, r=link1[0], l=link1[1], m=link1[2], px=posBaseLeg[3][0], py=posBaseLeg[3][1]                       , pz=posBaseLeg[3][2])
        self.__link32 = AddCapsule (world, space, r=link2[0], l=link2[1], m=link2[2], px=posBaseLeg[3][0], py=posBaseLeg[3][1]+(link1[1]/2+link2[1]/2)      , pz=posBaseLeg[3][2])
        self.__link33 = AddCapsule (world, space, r=link3[0], l=link3[1], m=link3[2], px=posBaseLeg[3][0], py=posBaseLeg[3][1]+(link1[1]/2+link2[1]+link3[1]/2), pz=posBaseLeg[3][2])
        
        self.__joint31 = self.AddJoint(world, odepy.dGeomGetBody(self.__Body)  , odepy.dGeomGetBody(self.__link31), [posBaseLeg[3][0],posBaseLeg[3][1]                      ,posBaseLeg[3][2]], [ 0, 0, 1])
        self.__joint32 = self.AddJoint(world, odepy.dGeomGetBody(self.__link32), odepy.dGeomGetBody(self.__link31), [posBaseLeg[3][0],posBaseLeg[3][1]+link1[1]/2           ,posBaseLeg[3][2]], [ 1, 0, 0])
        self.__joint33 = self.AddJoint(world, odepy.dGeomGetBody(self.__link33), odepy.dGeomGetBody(self.__link32), [posBaseLeg[3][0],posBaseLeg[3][1]+(link1[1]/2+link2[1]),posBaseLeg[3][2]], [ 1, 0, 0])
 
        self.__jointAngle = [   np.pi / 4.0, np.pi / 5.0, -np.pi / 2.0, 
                               -np.pi / 4.0, np.pi / 5.0, -np.pi / 2.0, 
                               -np.pi / 4.0, np.pi / 5.0, -np.pi / 2.0, 
                                np.pi / 4.0, np.pi / 5.0, -np.pi / 2.0 ]
        # self.__jointAngle = [    120.0*np.pi/180, 45.0*np.pi/180, -135.0*np.pi/180,              
        #                         -45.0*np.pi/180, 45.0*np.pi/180, -135.0*np.pi/180,
        #                         -45.0*np.pi/180, 45.0*np.pi/180, -135.0*np.pi/180,
        #                          90.0*np.pi/180, 45.0*np.pi/180, -135.0*np.pi/180 ]  
        """ self.__jointAngle = [ 0.0, 0.0, 0.0,              
                                0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0 ]  """
        self.__eJointAngle = [ 0.0, 0.0, 0.0,              
                                0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0 ]                     #          +X
        self.init_POS_EoF =[[ 0.1,-0.1,-0.1],                       #           |
                            [-0.1,-0.1,-0.1],                       #           |
                            [-0.1, 0.1,-0.1],                       #           |
                            [ 0.1, 0.1,-0.1]]                       # +Y--------|------- -Y
        self.POS_EoF =     [[ 0.1,-0.1,-0.1],                       #           |
                            [-0.1,-0.1,-0.1],                       #           |
                            [-0.1, 0.1,-0.1],                       #           |
                            [ 0.1, 0.1,-0.1]]                       #          -X

    def GetJointValues(self):
        return self.__jointValues

    def SetJointValues(self, jointValues):
        self.__jointValues = jointValues

    def GetGeoms(self):
        return  self.__Body, self.__link01, self.__link02, self.__link03, \
                             self.__link11, self.__link12, self.__link13, \
                             self.__link21, self.__link22, self.__link23, \
                             self.__link31, self.__link32, self.__link33

    def AddJoint(self, world, body1, body2, pos, axis):
        joint = odepy.dJointCreateHinge(world, 0)
        odepy.dJointAttach(joint, body1, body2)
        odepy.dJointSetHingeAnchor(joint, *pos)
        odepy.dJointSetHingeAxis(joint, *axis)
        
        return joint

    def drawRobot(self):
        # Drawing process
        for geom in self.GetGeoms():
            ds.dsSetColorAlpha(0.7, 0.7, 1.0, 1.0)
            r = odepy.dReal()
            l = odepy.dReal()
            if odepy.dGeomGetClass(geom) == odepy.dCylinderClass:
                odepy.dGeomCylinderGetParams(geom, ctypes.byref(r), ctypes.byref(l))
                ds.dsDrawCylinderD(odepy.dGeomGetPosition(geom), odepy.dGeomGetRotation(geom), l.value, r.value)
            if odepy.dGeomGetClass(geom) == odepy.dCapsuleClass:
                if geom==self.__link01:
                    ds.dsSetColorAlpha(0.7, 1.0, 1.0, 1.0)
                if geom==self.__link11:
                    ds.dsSetColorAlpha(0.7, 0.0, 0.0, 1.0)
                odepy.dGeomCapsuleGetParams(geom, ctypes.byref(r), ctypes.byref(l))
                ds.dsDrawCapsuleD(odepy.dGeomGetPosition(geom), odepy.dGeomGetRotation(geom), l.value, r.value)

    def inverseKinematicLeg(self, id, x, y, z):
        angles=[]
        if (id == 0 or id == 1):
            y = -y 
        L1 = self.Coxa/2
        L2 = self.Femur
        L3 = self.Tibia

        angles.append(np.arctan2(x,y))
        R = np.sqrt( y*y + x*x)
        active_length = R - L1
        SWE = np.sqrt( z*z + active_length * active_length)
        b = np.arctan2(z, active_length)                                    
        angles.append(np.arccos((L2 * L2 + SWE * SWE - L3 * L3) / (2 * L2 * SWE)) + b)
        angles.append(np.arccos((L2 * L2 + L3 * L3 - SWE * SWE) / (2 * L2 * L3)) - np.pi)
    
        return angles
            
    def Control(self, tDelta):
        x=self.inverseKinematicLeg(0,x=self.POS_EoF[0][0],y=self.POS_EoF[0][1],z=self.POS_EoF[0][2])
        self.__jointAngle[0],self.__jointAngle[1],self.__jointAngle[2]=x[0],x[1],x[2]
        x=self.inverseKinematicLeg(1,x=self.POS_EoF[1][0],y=self.POS_EoF[1][1],z=self.POS_EoF[1][2])
        self.__jointAngle[3],self.__jointAngle[4],self.__jointAngle[5]=x[0],x[1],x[2]
        x=self.inverseKinematicLeg(2,x=self.POS_EoF[2][0],y=self.POS_EoF[2][1],z=self.POS_EoF[2][2])
        self.__jointAngle[6],self.__jointAngle[7],self.__jointAngle[8]=x[0],x[1],x[2]
        x=self.inverseKinematicLeg(3,x=self.POS_EoF[3][0],y=self.POS_EoF[3][1],z=self.POS_EoF[3][2])
        self.__jointAngle[9],self.__jointAngle[10],self.__jointAngle[11]=x[0],x[1],x[2]
            
        print(self.__jointAngle[0]*180/np.pi,self.__jointAngle[3]*180/np.pi,self.__jointAngle[6]*180/np.pi,self.__jointAngle[9]*180/np.pi)
        print('{}\t{}\t{}\t{}'.format(self.__jointAngle[0],self.__jointAngle[3], self.__jointAngle[6],self.__jointAngle[9]))
        #a=-20 ~ 110     -45~45  0 ~ -135
        kp, kd = 10.0, 0.9
        fMax = 100.0
        for i, joint in enumerate([self.__joint01, self.__joint02, self.__joint03,
                                   self.__joint11, self.__joint12, self.__joint13,
                                   self.__joint21, self.__joint22, self.__joint23,
                                   self.__joint31, self.__joint32, self.__joint33]):
            jointAngle = odepy.dJointGetHingeAngle(joint)  # pi と -pi を越える際に不連続です。
            eJointAngle = self.__jointAngle[i] - jointAngle
            # jointValue が不連続であることに対応して以下の処理を行います。
            while eJointAngle > np.pi:
                eJointAngle -= 2.0 * np.pi
            while eJointAngle < -np.pi:
                eJointAngle += 2.0 * np.pi
            u = kp * eJointAngle + kd * (eJointAngle - self.__eJointAngle[i]) / tDelta
            odepy.dJointSetHingeParam(joint, odepy.dParamVel, u)
            odepy.dJointSetHingeParam(joint, odepy.dParamFMax, fMax)
            self.__eJointAngle[i] = eJointAngle

    def getEoFRobot(self, POS_EoF):
        
        return


def AddCapsule(world, space, r, l, m, px, py, pz):
    geom = odepy.dCreateCapsule(space, r, l)
    body = odepy.dBodyCreate(world)
    mass = odepy.dMass()
    direction = 3  # z 軸方向
    odepy.dMassSetZero(ctypes.byref(mass))
    odepy.dMassSetCapsuleTotal(ctypes.byref(mass), m, direction, r, l)
    odepy.dGeomSetBody(geom, body)
    odepy.dBodySetMass(body, ctypes.byref(mass))
    odepy.dBodySetPosition(body, px, py, pz)
    q = odepy.dQuaternion()
    odepy.dQFromAxisAndAngle(q, 1, 0, 0, np.pi / 2.0)
    odepy.dBodySetQuaternion(body, q)
    return geom

def AddCylinder(world, space, r, l, m, px, py, pz):
    geom = odepy.dCreateCylinder(space, r, l)
    body = odepy.dBodyCreate(world)
    mass = odepy.dMass()
    direction=3
    odepy.dMassSetZero(ctypes.byref(mass))
    odepy.dMassSetCylinderTotal(ctypes.byref(mass), m, direction, r, l)
    odepy.dGeomSetBody(geom, body)
    odepy.dBodySetMass(body, ctypes.byref(mass))
    odepy.dBodySetPosition(body, px, py, pz)
    q = odepy.dQuaternion()
    odepy.dQFromAxisAndAngle(q, 0, 0, 1, 0.0)
    odepy.dBodySetQuaternion(body, q)
    return geom

def AddBox( world, space, lx, ly, lz, m, px, py, pz):
    geom = odepy.dCreateBox(space, lx, ly, lz)
    body = odepy.dBodyCreate(world)
    mass = odepy.dMass()
    odepy.dMassSetZero(ctypes.byref(mass))
    odepy.dMassSetBoxTotal(ctypes.byref(mass), m, lx, ly, lz)
    odepy.dGeomSetBody(geom, body)
    odepy.dBodySetMass(body, ctypes.byref(mass))
    odepy.dBodySetPosition(body, px, py, pz)
    q = odepy.dQuaternion()
    odepy.dQFromAxisAndAngle(q, 0, 0, 1, np.pi / 2.0)
    odepy.dBodySetQuaternion(body, q)
    return geom

def Main():
    world, space, ground, contactgroup = CreateWorld()

    robot = Robot(world, space)         

    def __StepCallback(pause):          #simloop in C++
        # Advance time
        tDelta = 0.01
        robot.Control(tDelta)
        odepy.dSpaceCollide(space, 0, odepy.dNearCallback(__NearCallback))
       
        odepy.dWorldStep(world, tDelta)
        odepy.dJointGroupEmpty(contactgroup)
        
        robot.drawRobot()

    def __Command(cmd):
        if chr(cmd) == 'x':
            motor.SetAlpha(motor.GetAlpha() + 1)

    def __NearCallback(data, o1, o2):
            o1IsGround = ctypes.addressof(ground.contents) == ctypes.addressof(o1.contents)
            o2IsGround = ctypes.addressof(ground.contents) == ctypes.addressof(o2.contents)
            if not (o1IsGround or o2IsGround):
                return
            N = 10
            contacts = (odepy.dContact * N)()
            n = odepy.dCollide(o1, o2, N, ctypes.byref(contacts[0].geom), ctypes.sizeof(odepy.dContact))
            for i in range(n):
                contact = contacts[i]
                contact.surface.mu = float('inf')
                contact.surface.mode = odepy.dContactApprox1 | odepy.dContactSlip1 | odepy.dContactSlip2 | odepy.dContactSoftERP | odepy.dContactSoftCFM 
                contact.surface.slip1=0.0001
                contact.surface.slip2=0.0001
                contact.surface.soft_erp = 0.2
                contact.surface.soft_cfm = 1e-4
                c = odepy.dJointCreateContact(world, contactgroup, ctypes.byref(contact))
                odepy.dJointAttach(c, odepy.dGeomGetBody(contact.geom.g1), odepy.dGeomGetBody(contact.geom.g2))

    RunDrawStuff(__StepCallback, __Command)
    DestroyWorld(world, space)

class Motor(object):

    def __init__(self, world, body1, body2, alpha=0.0, beta=0.0, gamma=0.0):
        self.__alpha = alpha
        self.__beta = beta
        self.__gamma = gamma
        self.__motor = odepy.dJointCreateAMotor(world, 0)

        odepy.dJointAttach(self.__motor, body1, body2)
        odepy.dJointSetAMotorMode(self.__motor, odepy.dAMotorEuler)
        odepy.dJointSetAMotorNumAxes(self.__motor, 3)

        # Sets the Euler angle axis.

        # The first axis (0) is fixed to body1。
        odepy.dJointSetAMotorAxis(self.__motor, 0, 1, 1, 0, 0)

        # The second axis is calculated to be orthogonal to the other two axes.

        # The third axis (2) is fixed to body2。
        odepy.dJointSetAMotorAxis(self.__motor, 2, 2, 0, 0, 1)

    def GetAlpha(self):
        return self.__alpha

    def GetBeta(self):
        return self.__beta

    def GetGamma(self):
        return self.__gamma

    def SetAlpha(self, alpha):
        self.__alpha = max(min(alpha, 80.0), -80.0)

    def SetBeta(self, beta):
        self.__beta = max(min(beta, 80.0), -80.0)

    def SetGamma(self, gamma):
        self.__gamma = max(min(gamma, 80.0), -80.0)

    def Control(self):
        angles = []
        for axisNum, target in enumerate([self.__alpha, self.__beta, self.__gamma]):
            target = target * np.pi / 180.0
            kp = 10.0
            fmax = 100.0
            angle = odepy.dJointGetAMotorAngle(self.__motor, axisNum)
            u = kp * (target - angle)

            if axisNum == 0:
                odepy.dJointSetAMotorParam(self.__motor, odepy.dParamVel, u)
                odepy.dJointSetAMotorParam(self.__motor, odepy.dParamFMax, fmax)
            elif axisNum == 1:
                odepy.dJointSetAMotorParam(self.__motor, odepy.dParamVel2, u)
                odepy.dJointSetAMotorParam(self.__motor, odepy.dParamFMax2, fmax)
            elif axisNum == 2:
                odepy.dJointSetAMotorParam(self.__motor, odepy.dParamVel3, u)
                odepy.dJointSetAMotorParam(self.__motor, odepy.dParamFMax3, fmax)

            angles.append(angle * 180.0 / np.pi)

        print('{}\t{}\t{}'.format(angles[0], angles[1], angles[2]))

def RunDrawStuff(stepCallback, command, pathToTextures='/usr/local/share/ode-0.16.1-drawstuff-textures', w=500, h=500, cameraXyz=[-0.7, 0.0, 0.4], cameraHpr=[0.0, 0.0, 0.0]):
    def __StartCallback():
        xyz = (ctypes.c_float * 3)()
        hpr = (ctypes.c_float * 3)()
        for i, v in enumerate(cameraXyz):
            xyz[i] = v
        for i, v in enumerate(cameraHpr):
            hpr[i] = v
        ds.dsSetViewpoint(xyz, hpr)
    fn = ds.dsFunctions()
    fn.version = ds.DS_VERSION
    fn.start = ds.dsStartCallback(__StartCallback)
    fn.step = ds.dsStepCallback(stepCallback)
    fn.path_to_textures = ctypes.create_string_buffer(pathToTextures.encode('utf-8'))
    fn.command = ds.dsCommandCallback(command)
    ds.dsSimulationLoop(0, ctypes.byref(ctypes.POINTER(ctypes.c_char)()), w, h, fn)

def CreateWorld():
    odepy.dInitODE()
    world = odepy.dWorldCreate()
    odepy.dWorldSetGravity(world, 0.0, 0.0, -9.8)
    space = odepy.dHashSpaceCreate(0)
    ground = odepy.dCreatePlane(space, 0, 0, 1, 0)
    contactgroup = odepy.dJointGroupCreate(0)
    return world, space, ground, contactgroup

def DestroyWorld(world, space):
    odepy.dSpaceDestroy(space)
    odepy.dWorldDestroy(world)
    odepy.dCloseODE()

if __name__ == '__main__':
    Main()
