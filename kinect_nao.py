# -*- coding: utf-8 -*-
#修改elbow_roll计算方式
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
# from first import main as R_knee_pitch
import ctypes
import _ctypes
import pygame
import sys
import math
import time
from naoqi import ALProxy
import utils
import numpy as np

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread
global pi
pi = 3.1415926
global i
i=0
# colors for drawing different bodies
SKELETON_COLORS = [pygame.color.THECOLORS["red"],
                   pygame.color.THECOLORS["blue"],
                   pygame.color.THECOLORS["green"],
                   pygame.color.THECOLORS["orange"],
                   pygame.color.THECOLORS["purple"],
                   pygame.color.THECOLORS["yellow"],
                   pygame.color.THECOLORS["violet"]]


class BodyGameRuntime(object):
    def __init__(self):
        pygame.init()
        robotIp = "127.0.0.1"
        PORT = 50755
        self.motionProxy = ALProxy("ALMotion", robotIp, PORT)
        self.postureProxy = ALProxy("ALRobotPosture", robotIp, PORT)
        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1),
                                               pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

        pygame.display.set_caption("Kinect for Windows v2 Body Game")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(
            PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface(
            (self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)

        # here we will store skeleton data
        self._bodies = None




    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
        joint0State = joints[joint0].TrackingState
        joint1State = joints[joint1].TrackingState

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked):
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)
        if joint0 == PyKinectV2.JointType_Head:
            # print start, end
            theta = (start[0] - end[0]) / (end[1] - start[1])
            import math
            # print 180*math.atan(theta)/math.pi

        try:
            pygame.draw.line(self._frame_surface, color, start, end, 8)
        except:  # need to catch it due to possible invalid positions (with inf)
            pass

    def draw_body(self, joints, jointPoints, color):
        # Torso
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_SpineMid)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_ShoulderRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder,
                            PyKinectV2.JointType_ShoulderLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft)

        # Right Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight,
                            PyKinectV2.JointType_ElbowRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight,
                            PyKinectV2.JointType_WristRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight,
                            PyKinectV2.JointType_HandTipRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight,
                            PyKinectV2.JointType_ThumbRight)

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft,
                            PyKinectV2.JointType_ElbowLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_ThumbLeft)

        # Right Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipRight, PyKinectV2.JointType_KneeRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeRight, PyKinectV2.JointType_AnkleRight)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleRight, PyKinectV2.JointType_FootRight)

        # Left Leg
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HipLeft, PyKinectV2.JointType_KneeLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_KneeLeft, PyKinectV2.JointType_AnkleLeft)
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_AnkleLeft, PyKinectV2.JointType_FootLeft)

    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def motion(self, rshoulder_pitch, rshoulder_roll, relbow_roll,lshoulder_pitch, lshoulder_roll, lelbow_roll,):
        self.motionProxy.post.angleInterpolationWithSpeed("RShoulderPitch", rshoulder_pitch, 0.4)
        self.motionProxy.post.angleInterpolationWithSpeed("RShoulderRoll", rshoulder_roll, 0.4)
        self.motionProxy.post.angleInterpolationWithSpeed("RElbowRoll", relbow_roll, 0.4)
        self.motionProxy.post.angleInterpolationWithSpeed("LShoulderPitch", lshoulder_pitch, 0.4)
        self.motionProxy.post.angleInterpolationWithSpeed("LShoulderRoll", lshoulder_roll, 0.4)
        self.motionProxy.post.angleInterpolationWithSpeed("LElbowRoll", lelbow_roll, 0.4)

        # self.motionProxy.post.angleInterpolationWithSpeed("RHipPitch", radhippitch, 0.4)
        # self.motionProxy.post.angleInterpolationWithSpeed("RHipRoll", radhiproll, 0.4)
        # self.motionProxy.post.angleInterpolationWithSpeed("RKneePitch", radkneepitch, 0.4)

    def Build_Coor(self, joints, joint0, joint1, joint2, joint3):

        ShoulderRight = [joints[joint0].Position.x, joints[joint0].Position.y, joints[joint0].Position.z]
        ShoulderLeft = [joints[joint1].Position.x, joints[joint1].Position.y, joints[joint1].Position.z]
        SpineBase = [joints[joint2].Position.x, joints[joint2].Position.y, joints[joint2].Position.z]
        SpineShoulder = [joints[joint3].Position.x, joints[joint3].Position.y, joints[joint3].Position.z]

        # confirm z axis: increasing z direction is from SpineBase to SpineShoulder
        axix_z = [SpineShoulder[0] - SpineBase[0], SpineShoulder[1] - SpineBase[1],
                  SpineShoulder[2] - SpineBase[2]]

        # confirm y axis:increasing y direction is from ShoulderReft to ShoulderLeft
        axix_y = [ShoulderLeft[0] - ShoulderRight[0], ShoulderLeft[1] - ShoulderRight[1],
                                      ShoulderLeft[2] - ShoulderRight[2]]

        # confirm x axis
        axix_x = utils.normalized_cross(axix_y, axix_z)
        coor = [axix_x, axix_y, axix_z]
        return coor


    def Cal_Joint_angel_RightArm(self,joints,joint0,joint1,joint2, coor):
        coor_x = coor[0]
        coor_y = coor[1]
        coor_z = coor[2]

        Shoulder = [joints[joint0].Position.x, joints[joint0].Position.y, joints[joint0].Position.z]
        Elbow = [joints[joint1].Position.x, joints[joint1].Position.y, joints[joint1].Position.z]
        Wrist = [joints[joint2].Position.x, joints[joint2].Position.y, joints[joint2].Position.z]

        #计算relbow_roll
        Vector_Shoulder_Elbow = utils.get_vector(Elbow, Shoulder)
        Vector_Elbow_Wrist = utils.get_vector(Wrist, Elbow)
        relbow_roll = np.arccos(utils.normalized_dot(Vector_Shoulder_Elbow,Vector_Elbow_Wrist))
        relbow_roll = min(relbow_roll, 1.5446)
        relbow_roll = max(relbow_roll, 0.0349)


        # #计算RShoulder_Pitch
        # rshoulder_pitch = np.arccos(utils.normalized_dot(coor_x, Vector_Shoulder_Elbow))  #与x轴夹角
        # if(Shoulder[1] - Elbow[1] < 0):
        #      rshoulder_pitch = -rshoulder_pitch
        # rshoulder_pitch = min(rshoulder_pitch, 2.0857)
        #  rshoulder_pitch = max(rshoulder_pitch, -2.0857)

        # 计算RShoulder_Pitch2
        rshoulder_pitch = 0
        [x, y, z] = utils.get_vector(Shoulder, Elbow)
        if (y > 0 and z > 0):
            rshoulder_pitch = np.arctan(y / z)
        if (y > 0 and z < 0):
            rshoulder_pitch = pi - np.arctan(-y / z)
        if (y < 0 and z < 0):
            rshoulder_pitch = np.arctan(y / z) - pi
        if (y > 0 and z > 0):
            rshoulder_pitch = -np.arctan(-y / z)
        rshoulder_pitch = min(rshoulder_pitch, 2.0857)
        rshoulder_pitch = max(rshoulder_pitch, -2.0857)


        # #计算RShoulder_Pitch3
        # coor_z1 = [-1 * z for z in coor_z]
        # rshoulder_pitch = np.arccos(utils.normalized_dot(coor_z1, Vector_Shoulder_Elbow))  #与负z轴夹角
        # rshoulder_pitch = rshoulder_pitch - math.pi / 2
        # rshoulder_pitch = min(rshoulder_pitch, 2.0857)
        # rshoulder_pitch = max(rshoulder_pitch, -2.0857)






        #计算RShoulder_roll
        coor_y1 = [-1*y for y in coor_y]   # 取反
        rshoulder_roll = np.arccos(utils.normalized_dot(coor_y1, Vector_Shoulder_Elbow)) - math.pi/2
        rshoulder_roll = min(rshoulder_roll, 0.3142)
        rshoulder_roll= max(rshoulder_roll, -1.3265)

        angle_rshoulder_roll = rshoulder_roll / math.pi * 180
        angle_rshoulder_pitch = rshoulder_pitch / math.pi * 180
        angle_relbow_roll = relbow_roll / math.pi * 180
        if (i % 30 == 0):
            #print("rshoulder_roll:", angle_rshoulder_roll)
            print("rshoulder_pitch:", angle_rshoulder_pitch)
            #print("relbow_roll:", angle_relbow_roll)

        return rshoulder_pitch, rshoulder_roll, relbow_roll

    def Cal_Joint_angle_LeftArm(self,joints,joint0,joint1,joint2,coor):
        coor_x = coor[0]
        coor_y = coor[1]
        coor_z = coor[2]

        Shoulder = [joints[joint0].Position.x, joints[joint0].Position.y, joints[joint0].Position.z]
        Elbow = [joints[joint1].Position.x, joints[joint1].Position.y, joints[joint1].Position.z]
        Wrist = [joints[joint2].Position.x, joints[joint2].Position.y, joints[joint2].Position.z]

        # 计算lelbow_roll
        Vector_Shoulder_Elbow = utils.get_vector(Elbow, Shoulder)
        Vector_Elbow_Wrist = utils.get_vector(Wrist, Elbow)
        lelbow_roll = -np.arccos(utils.normalized_dot(Vector_Shoulder_Elbow, Vector_Elbow_Wrist))

        lelbow_roll = min(lelbow_roll, -0.0349)
        lelbow_roll = max(lelbow_roll, -1.5446)

        # 计算lShoulder_Pitch
        # lshoulder_pitch = np.arccos(utils.normalized_dot(coor_x, Vector_Shoulder_Elbow))  # 与x轴夹角
        # if (Shoulder[1] - Elbow[1] < 0):
        #     lshoulder_pitch = -lshoulder_pitch
        # lshoulder_pitch = min(lshoulder_pitch, 2.0857)
        # lshoulder_pitch = max(lshoulder_pitch, -2.0857)

        lshoulder_pitch = 0
        [x, y, z] = utils.get_vector(Shoulder, Elbow)
        if (y > 0 and z > 0):
            lshoulder_pitch = np.arctan(y / z)
        if (y > 0 and z < 0):
            lshoulder_pitch = pi - np.arctan(-y / z)
        if (y < 0 and z < 0):
            lshoulder_pitch = np.arctan(y / z) - pi
        if (y > 0 and z > 0):
            lshoulder_pitch = -np.arctan(-y / z)
        lshoulder_pitch = min(lshoulder_pitch, 2.0857)
        lshoulder_pitch = max(lshoulder_pitch, -2.0857)

        # 计算lShoulder_roll
        coor_y1 = [-1 * y for y in coor_y]  # 取反
        lshoulder_roll = np.arccos(utils.normalized_dot(coor_y1, Vector_Shoulder_Elbow)) - math.pi / 2
        lshoulder_roll = min(lshoulder_roll, 1.3265)
        lshoulder_roll = max(lshoulder_roll, -0.3142)


        angle_lshoulder_roll = lshoulder_roll / math.pi * 180
        angle_lshoulder_pitch = lshoulder_pitch/ math.pi * 180
        angle_lelbow_roll = lelbow_roll / math.pi * 180

        # if(i % 30 ==0):
        #     print("lshoulder_roll:", angle_lshoulder_roll)
        #     print("lshoulder_pitch:", angle_lshoulder_pitch)
        #     print("lelbow_roll:", angle_lelbow_roll)

        return lshoulder_pitch, lshoulder_roll, lelbow_roll

    def run(self):
        PORT = 9559
        ip = '192.168.1.101'
        contsi = 0
        replayarray = []
        cnt = 0

        # -------- Main Program Loop -----------
        while (1):
            for event in pygame.event.get():  # User did something
                if event.type == pygame.QUIT:  # If user clicked close
                    self._done = True  # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE:  # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'],
                                                           pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.RESIZABLE, 32)

            # --- Game logic should go here

            # --- Getting frames and drawing
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame():
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            #

            if self._bodies is not None:
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]

                    if not body.is_tracked:
                        continue

                    joints = body.joints

                    i = i+1
                    coor = self.Build_Coor(joints, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineBase)
                    rshoulder_pitch, rshoulder_roll, relbow_roll = self.Cal_Joint_angel_RightArm(joints, PyKinectV2.JointType_ShoulderRight,
                                                                                                             PyKinectV2.JointType_ElbowRight,
                                                                                                             PyKinectV2.JointType_HandRight,
                                                                                                             coor)
                    lshoulder_pitch, lshoulder_roll, lelbow_roll = self.Cal_Joint_angle_LeftArm(joints,  PyKinectV2.JointType_ShoulderLeft,
                                                                                                            PyKinectV2.JointType_ElbowLeft,
                                                                                                            PyKinectV2.JointType_HandLeft,
                                                                                                            coor)

                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    self.draw_body(joints, joint_points, SKELETON_COLORS[i])

                    self.motion(rshoulder_pitch, rshoulder_roll, relbow_roll,lshoulder_pitch, lshoulder_roll, lelbow_roll)

                    # joint_points = numpy.ndarray((PyKinectV2.JointType_Count), dtype=numpy.object)
                    # print joints[6].Position.x,joints[6].Position.y,joints[6].Position.z

                    # print Camera_distance

                    # convert joint coordinates to color space
                    # joint_points = self._kinect.body_joints_to_color_space(joints)

                    # self.draw_body(joints, joint_points, SKELETON_COLORS[i])

            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size)
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height))
            self._screen.blit(surface_to_draw, (0, 0))
            surface_to_draw = None
            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to 60 frames per second
            self._clock.tick(60)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()
        time.sleep(0.5)
        memorytouch = ALProxy("ALMemory", ip, PORT)
        touched = 0
        while True:
            if memorytouch.getData("FrontTactilTouched") > 0:
                touched = 1
                break
        time.sleep(0.5)
        self.motionProxy.rest()


__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime()
game.run()

