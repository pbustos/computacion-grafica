# !/usr/bin/python3
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
from genericworker import *
import traceback


import pygame
from PySide2.QtCore import QTimer, Signal
from PySide2.QtWidgets import QApplication

from DualControl import DualControl
from HUD import HUD
from Logger import Logger
from SensorManager import CameraManager, GNSSSensor, IMUSensor
import numpy as np

from math import sin, cos, sqrt, atan2, radians
import numpy as np
import cv2
from PIL import Image
from yolov4 import Detector
import matplotlib.pyplot as plt
import time
import matplotlib.pyplot as plt



class SpecificWorker(GenericWorker):
    logger_signal = Signal(str, str, str)
    latitude = 0
    longitude = 0
    timestamp = 0

    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        self.d = Detector(gpu_id=0, config_path='/home/salabeta/software/darknet/cfg/yolov4-tiny.cfg',
                     weights_path='/home/salabeta/software/darknet/yolov4-tiny.weights')

        self.width = 1280
        self.height = 720
        self.vel=0.0008081557472921903

        pygame.init()
        pygame.font.init()

        self.display = pygame.display.set_mode(
            (self.width, self.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        self.camera_manager = CameraManager(self.width, self.height)
        self.gnss_sensor = GNSSSensor()
        self.imu_sensor = IMUSensor()
        self.hud = HUD(self.width, self.height, self.gnss_sensor, self.imu_sensor)
        self.controller = None
        self.clock = pygame.time.Clock()

        self.data_to_save = {
            'control': ['Time', 'Throttle', 'Steer', 'Brake', 'Gear', 'Handbrake', 'Reverse', 'Manualgear'],
            'communication': ['Time', 'CommunicationTime']
        }
        self.logger = Logger(self.melexlogger_proxy, 'carlaRemoteControl', self.data_to_save)
        self.logger_signal.connect(self.logger.publish_to_logger)

        self.Period = 0

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        try:
            wheel_config = params["wheel"]
            self.controller = DualControl(self.camera_manager, self.hud, wheel_config,
                                          self.carlavehiclecontrol_proxy)

        except:
            traceback.print_exc()
            print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute(self):
        self.clock.tick_busy_loop(60)
        if self.controller and self.controller.parse_events(self.clock):
            exit(-1)

        if self.controller.car_moved():
            control, elapsed_time = self.controller.publish_vehicle_control()
            control_array = [control.throttle, control.steer, control.brake, control.gear, control.handbrake,
                             control.reverse,
                             control.manualgear]
            data = ';'.join(map(str, control_array))
            self.logger_signal.emit('control', 'compute', data)
            self.logger_signal.emit('communication', 'compute', str(elapsed_time))

            self.hud.tick(self, self.clock, control)

        #print(self.controller._control_steer)
        self.camera_manager.render(self.display)
        self.hud.render(self.display)

        # Calculo de velocidad. Se obtendra el timestamp actual en formato unix (segundos), latitud y longitud.
        # Obtenidos los valores se calcula la velocidad
        aux_latitude = self.latitude
        aux_longitude = self.longitude
        self.latitude = self.gnss_sensor.latitude
        self.longitude = self.gnss_sensor.longitude
        km = self.getDistanceFromLatLonInKm(aux_latitude, aux_longitude, self.latitude,
                                            self.longitude)  # Distancia en km

        aux_timestamp = self.timestamp
        self.timestamp = self.gnss_sensor.timestamp
        speed = self.calc_velocity(km, aux_timestamp, self.timestamp)

        surf = pygame.Surface((self.d.network_width(), self.d.network_height()))  # I'm going to use 100x200 in examples
        data = pygame.image.tostring(surf, 'RGBA')

        #img = Image.open('/home/salabeta/software/darknet/data/dog.jpg')

        img_arr = self.camera_manager.get_image_for_yolo(0)
        if img_arr is not None:
            #cv2.imshow('title', img_arr)
            img_arr = cv2.cvtColor(img_arr, cv2.COLOR_BGRA2BGR)
            img = Image.fromarray(img_arr)
            img_yolo = np.array(img.resize((self.d.network_width(), self.d.network_height())))
            detections = self.d.perform_detect(image_path_or_buf=img_yolo, show_image=True)
            print(len(detections))
            for detection in detections:
                i=detection.left_x, detection.top_y, detection.width, detection.height
                x,y,w,h = detection.left_x, detection.top_y, detection.width, detection.height
                cv2.line(img_yolo, (x, y), (x, y + h), (0, 255, 0), thickness=4)
                cv2.line(img_yolo, (x, y), (x + w, y), (0, 255, 0), thickness=4)
                cv2.line(img_yolo, (x, y + h), (x + w, y + h), (0, 255, 0), thickness=4)
                cv2.line(img_yolo, (x + w, y), (x + w, y + h), (0, 255, 0), thickness=4)
                x=int(x*1280/416)
                h=int(h*720/416)
                y=int(y*720/416)
                w=int(w*1280/416)
                #imgs.append(box)
                print(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} % | {i}')
                pygame.draw.line(self.display, (0,0,255), (x,y),(x,y+h))
                pygame.draw.line(self.display, (0, 0, 255), (x, y), (x + w, y))
                pygame.draw.line(self.display, (0, 0, 255), (x, y + h), (x + w, y+h))
                pygame.draw.line(self.display, (0, 0, 255), (x + w, y), (x + w, y + h))



                """"""
                cv2.imshow('title', img_yolo)
                """pygame.draw.line(self.display, (0, 0, 255), [(y + w, x + h), (y, x), (y, x + h), (y + w, x)])
                pygame.draw.line(self.display, (0, 0, 255), [(y + w, x + h), (y, x), (y, x + h), (y + w, x)])"""


               # [i[1]: i[1] + i[3], i[0]: i[0] + i[2]]















        pygame.display.flip()

        # print('Der',jDer, iDer)
        # print('Izq',jIzq, iIzq)

        """
        pygame.draw.circle(self.display, (0,0,255), [int(iDer), int(jDer)], 5)
        pygame.draw.circle(self.display, (0, 0, 255), [int(iIzq), int(jIzq)], 5)

        pygame.draw.circle(self.display, (0, 0, 255), [int(iFaroDer), int(jFaroDer)], 5)
        pygame.draw.circle(self.display, (0, 0, 255), [int(iFaroIzq), int(jFaroIzq)], 5)
        """


        """
        #Dibujo del poligono representado por lineas para una mejor visualizacion 
        pygame.draw.line(self.display, (0, 0, 255), (iFaroIzq, jFaroIzq), (iIzq, jIzq))
        pygame.draw.line(self.display, (0, 0, 255), (iIzq, jIzq), (iDer, jDer))
        pygame.draw.line(self.display, (0, 0, 255), (iDer, jDer), (iFaroDer, jFaroDer))"""

        pygame.display.flip()

        return True

    def getDistanceFromLatLonInKm(self, lat1, lon1, lat2, lon2):
        R = 6373.0  # Radius of the earth in km
        dLat = radians(lat2 - lat1)
        dLon = radians(lon2 - lon1)
        rLat1 = radians(lat1)
        rLat2 = radians(lat2)
        a = sin(dLat / 2) * sin(dLat / 2) + cos(rLat1) * cos(rLat2) * sin(dLon / 2) * sin(dLon / 2)
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        d = R * c * 1000  # Distance in m
        return d

    def calc_velocity(self, dist_km, time_start, time_end):
        """Return 0 if time_start == time_end, avoid dividing by 0"""
        dt = (time_end - time_start)
        if dt == 0:
            return 0

        return (dist_km / dt) * 3.6 if time_end > time_start else 0

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    #
    # SUBSCRIPTION to pushRGBD method from CameraRGBDSimplePub interface
    #
    def CarCameraRGBD_pushRGBD(self, im, dep):
        self.camera_manager.images_received[im.cameraID] = im

    #
    # SUBSCRIPTION to updateSensorGNSS method from CarlaSensors interface
    #
    def CarlaSensors_updateSensorGNSS(self, gnssData):
        # print(gnssData.latitude, gnssData.longitude)
        self.gnss_sensor.update(gnssData.latitude, gnssData.longitude, gnssData.altitude, gnssData.frame,
                                gnssData.timestamp)

    #
    # SUBSCRIPTION to updateSensorIMU method from CarlaSensors interface
    #
    def CarlaSensors_updateSensorIMU(self, imuData):
        self.imu_sensor.update(imuData.accelerometer, imuData.gyroscope, imuData.compass, imuData.frame,
                               imuData.timestamp)

    # ===================================================================
    # ===================================================================

    ######################
    # From the RoboCompCarlaVehicleControl you can call this methods:
    # self.carlavehiclecontrol_proxy.updateVehicleControl(...)

    ######################
    # From the RoboCompCarlaVehicleControl you can use this types:
    # RoboCompCarlaVehicleControl.VehicleControl

    ######################
    # From the RoboCompMelexLogger you can publish calling this methods:
    # self.melexlogger_proxy.createNamespace(...)
    # self.melexlogger_proxy.sendMessage(...)

    ######################
    # From the RoboCompMelexLogger you can use this types:
    # RoboCompMelexLogger.LogMessage
    # RoboCompMelexLogger.LogNamespace

    ######################
    # From the RoboCompCarlaSensors you can use this types:
    # RoboCompCarlaSensors.IMU
    # RoboCompCarlaSensors.GNSS
