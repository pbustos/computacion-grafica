#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide2.QtCore import QTimer
from PySide2.QtCore import(Property, QObject, QPropertyAnimation, Signal)
from PySide2.QtGui import (QGuiApplication, QMatrix4x4, QQuaternion, QVector3D)
from PySide2.Qt3DCore import (Qt3DCore)
from PySide2.Qt3DExtras import (Qt3DExtras)
from genericworker import *
import cv2
import numpy as np
import time

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from matplotlib import style
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager
from PIL import Image

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
sys.path.append('/opt/robocomp/lib')

COCO_IDS = ["nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow",
            "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee",
            "left_ankle", "right_ankle"]
SKELETON_CONNECTIONS = [("left_ankle", "left_knee"),
                        ("left_knee", "left_hip"),
                        ("right_ankle", "right_knee"),
                        ("right_knee", "right_hip"),
                        ("left_hip", "right_hip"),
                        ("left_shoulder", "left_hip"),
                        ("right_shoulder", "right_hip"),
                        ("left_shoulder", "right_shoulder"),
                        ("left_shoulder", "left_elbow"),
                        ("right_shoulder", "right_elbow"),
                        ("left_elbow", "left_wrist"),
                        ("right_elbow", "right_wrist"),
                        ("left_eye", "right_eye"),
                        ("nose", "left_eye"),
                        ("nose", "right_eye"),
                        ("left_eye", "left_ear"),
                        ("right_eye", "right_ear"),
                        ("left_ear", "left_shoulder"),
                        ("right_ear", "right_shoulder")]

class Window(Qt3DExtras.Qt3DWindow):
    def __init__(self):
        super(Window, self).__init__()

        # Camera
        self.camera().lens().setPerspectiveProjection(45, 16 / 9, 0.1, 1000)
        self.camera().setPosition(QVector3D(0, 0, 40))
        self.camera().setViewCenter(QVector3D(0, 0, 0))

        # For camera controls
        self.createScene()
        self.camController = Qt3DExtras.QOrbitCameraController(self.rootEntity)
        self.camController.setLinearSpeed(50)
        self.camController.setLookSpeed(180)
        self.camController.setCamera(self.camera())

        self.setRootEntity(self.rootEntity)

    def createScene(self):
        # Root entity
        self.rootEntity = Qt3DCore.QEntity()

        # Material
        self.material = Qt3DExtras.QPhongMaterial(self.rootEntity)

        # Sphere
        self.sphereEntity = Qt3DCore.QEntity(self.rootEntity)
        self.sphereMesh = Qt3DExtras.QSphereMesh()
        self.sphereMesh.setRadius(3)
        self.sphereTransform = Qt3DCore.QTransform()

        self.sphereEntity.addComponent(self.sphereMesh)
        self.sphereEntity.addComponent(self.sphereTransform)
        self.sphereEntity.addComponent(self.material)


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)
        self.new_people = False
        self.new_image = False
        self.contPersonas = 0
        self.xMano = 0
        self.yMano = 0
        self.Period = 50

        self.cameras = {}

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        self.params = params

        self.viewimage = "true" in self.params["viewimage"]

        self.count = 0
        self.total = 0
        self.begin = time.time()

        object_to_cam1 = pt.transform_from(
            pr.active_matrix_from_intrinsic_euler_xyz(np.array([1.25, -0.028, -0.06])),
            np.array([381, 761, -3173]))

        object_to_cam3 = pt.transform_from(
            pr.active_matrix_from_intrinsic_euler_xyz(np.array([-2.04, -3.13, 0.09])),
            np.array([-192, 254, -3408]))

        self.tm = TransformManager()
        self.tm.add_transform("object", "camera_1", object_to_cam1)
        self.tm.add_transform("object", "camera_3", object_to_cam3)

        self.view = Window()
        self.view.show()

        return True


    @QtCore.Slot()
    def compute(self):
        self.count += 1

        if self.new_image:
            self.cameras[self.imgCruda.cameraID] = np.frombuffer(self.imgCruda.image, np.uint8).reshape(self.imgCruda.height, self.imgCruda.width, self.imgCruda.depth)
            cv2.putText(self.cameras[self.imgCruda.cameraID], str(self.total) + ' fps', (10, 450),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(self.cameras[self.imgCruda.cameraID], str(len(self.people.peoplelist)) + ' bodies', (500, 450),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1, cv2.LINE_AA)
            self.new_image = False

            if self.new_people:
                self.transform_to_world(self.people)
                self.cameras[self.imgCruda.cameraID] = self.draw_body(self.people, self.cameras[self.imgCruda.cameraID])

            self.cameras[self.imgCruda.cameraID] = cv2.cvtColor(self.cameras[self.imgCruda.cameraID], cv2.COLOR_BGR2RGB)
            pix = QPixmap.fromImage(QImage(self.cameras[self.imgCruda.cameraID], self.cameras[self.imgCruda.cameraID].shape[1], self.cameras[self.imgCruda.cameraID].shape[0], QImage.Format_RGB888))
            if self.imgCruda.cameraID == 1:
                self.ui.label_img_left.setPixmap(pix)
            if self.imgCruda.cameraID == 3:
                self.ui.label_img_right.setPixmap(pix)

        if time.time() - self.begin > 1:
            self.total = self.count
            self.count = 0
            self.begin = time.time()


    def transform_to_world(self, people):
        for person in people.peoplelist:
            for name1, name2 in SKELETON_CONNECTIONS:
                try:
                    joint1 = person.joints[name1]
                    joint2 = person.joints[name2]
                    if joint1.score > 0.5 and joint2.score > 0.5:
                        p = pt.transform(self.tm.get_transform("camera_1", "object"), np.array([joint1.x, joint1.y, joint1.z, 1]))

                except:
                    pass

    # call this to obtain the coordinates of the april tag
    def calibrate_with_apriltags(self):
        frame = RoboCompAprilTagsServer.Image()
        frame.data = self.imgCruda.image
        frame.frmt.width = self.imgCruda.width
        frame.frmt.height = self.imgCruda.height
        frame.frmt.size = self.imgCruda.depth
        frame.frmt.modeImage = RoboCompAprilTagsServer.Mode.RGB888Packet
        try:
            lista_tags = self.apriltagsserver_proxy.getAprilTags(frame, 300, 617.648, 616.812);
        except:
            print("Error sending image to apriltagsserver")
        print(lista_tags)

    # Draw body parts on image
    def draw_body(self, people, image):
        for person in people.peoplelist:
            for name1, name2 in SKELETON_CONNECTIONS:
                try:
                    joint1 = person.joints[name1]
                    joint2 = person.joints[name2]
                    if joint1.score > 0.5:
                        cv2.circle(image, (joint1.i, joint1.j), 10, (0, 0, 255))
                    if joint2.score > 0.5:
                        cv2.circle(image, (joint2.i, joint2.j), 10, (0, 0, 255))
                    if joint1.score > 0.5 and joint2.score > 0.5:
                        cv2.line(image, (joint1.i, joint1.j), (joint2.i, joint2.j), (0, 255, 0), 2)
                except:
                    pass
        return image

    #################################################################################################
    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)
    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    #

    # SUBSCRIPTION to pushRGBD method from CameraRGBDSimplePub interface
    #
    def CameraRGBDSimplePub_pushRGBD(self, im, dep):

        #print(im.image.cameraID)
        self.imgCruda=im
        self.new_image=True

    def HumanCameraBody_newPeopleData(self, people):
        #print(people)
        self.people = people
        self.new_people=True


