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
import time


class SpecificWorker(GenericWorker):
    logger_signal = Signal(str, str, str)
    latitude = 0
    longitude = 0
    timestamp = 0

    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

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
        if speed != 0 or self.vel>25:
            self.vel=speed
        print(self.vel)
        if(self.controller._control_reverse):
            y1=1
        else:
            y1 = 1 + (self.vel * 3 / 25)
        y2 = 1
        y3 = -1

        if self.controller._control_steer > 0.2 or  self.controller._control_steer < -0.2:
            ster=self.controller._control_steer
            if ster >0.9:
                ster=0.9
            elif ster < -0.9:
                ster=-0.9
            if ster < 0:
                y2 = y2 * ster-2
                y3 = y3 * ster-2
                aux=y3
                y3=y2
                y2=aux
            else:
                y2 = y2 * ster + 2
                y3 = y3 * ster + 2

        print(y2, '---', y3)




        # Obtencion puntos
        puntoDer = np.array([y1, y2, 0.1])
        puntoIzq = np.array([y1, y3, 0.1])
        puntoDerB = np.array([1, 1, 0.1])
        puntoIzqB = np.array([1, -1, 0.1])

        camara = np.array([0.0, -0.25, 1.0])

        puntoPintarDer = puntoDer - camara
        puntoPintarIzq = puntoIzq - camara

        # semejanza de triangulos
        f = 320.0 / np.tan(np.radians(110.0 / 2.0))
        iDer = (f * puntoPintarDer[1]) / puntoPintarDer[0] + self.width / 2
        jDer = (f * puntoPintarDer[2]) / puntoPintarDer[0] + self.height / 2
        jDer = self.height - jDer

        # print('-----------',puntoPintarDer[1], puntoPintarIzq[1] )

        iIzq = (f * puntoPintarIzq[1]) / puntoPintarIzq[0] + self.width / 2
        jIzq = (f * puntoPintarIzq[2]) / puntoPintarIzq[0] + self.height / 2
        jIzq = self.height - jIzq;

        # -------------------------------------------------------------------------------------

        camara = np.array([0.0, -0.25, 1.0])

        puntoPintarDer = puntoDerB - camara
        puntoPintarIzq = puntoIzqB - camara

        # semejanza de triangulos
        f = 320.0 / np.tan(np.radians(110.0 / 2.0))
        iDerB = (f * puntoPintarDer[1]) / puntoPintarDer[0] + self.width / 2
        jDerB = (f * puntoPintarDer[2]) / puntoPintarDer[0] + self.height / 2
        jDerB = self.height - jDerB

        # print('-----------', puntoPintarDer[1], puntoPintarIzq[1])

        iIzqB = (f * puntoPintarIzq[1]) / puntoPintarIzq[0] + self.width / 2
        jIzqB = (f * puntoPintarIzq[2]) / puntoPintarIzq[0] + self.height / 2
        jIzqB = self.height - jIzqB;

        # print('Der',jDerB, iDerB)
        # print('Izq',jIzqB, iIzqB)

        pygame.draw.line(self.display, (0, 0, 255), [iDer, jDer], [iIzq, jIzq], width=5)
        """pygame.draw.line(self.display, (0, 0, 255), [iDer, jDer], [iDerB, jDerB], width=5)
        pygame.draw.line(self.display, (0, 0, 255), [iIzq, jIzq], [iIzqB, jIzqB], width=5)"""


        y = 1
        x2 = -1
        x3 = 1

        dif = abs(y2) - abs(x2)
        print('dif ',dif)

        i=0
        while i<1000 and y1>y:
            puntoDer = np.array([y, x2, 0.1])
            puntoIzq = np.array([y, x3, 0.1])
            camara = np.array([0.0, -0.25, 1.0])

            puntoPintarDer = puntoDer - camara
            puntoPintarIzq = puntoIzq - camara
            f = 320.0 / np.tan(np.radians(110.0 / 2.0))
            iDer = (f * puntoPintarDer[1]) / puntoPintarDer[0] + self.width / 2
            jDer = (f * puntoPintarDer[2]) / puntoPintarDer[0] + self.height / 2
            jDer = self.height - jDer

            # print('-----------',puntoPintarDer[1], puntoPintarIzq[1] )

            iIzq = (f * puntoPintarIzq[1]) / puntoPintarIzq[0] + self.width / 2
            jIzq = (f * puntoPintarIzq[2]) / puntoPintarIzq[0] + self.height / 2
            jIzq = self.height - jIzq;

            pygame.draw.circle(self.display, (0, 0, 255), [iDer, jDer], 5, width=5)
            pygame.draw.circle(self.display, (0, 0, 255), [iIzq, jIzq], 5, width=5)
            #pygame.draw.line(self.display, (0, 0, 255), [iDer, jDer], [iDerB, jDerB], width=5)
            #pygame.draw.line(self.display, (0, 0, 255), [iIzq, jIzq], [iIzqB, jIzqB], width=5)

            iDerB=iDer
            jDerB=jDer
            iIzqB= iIzq
            jIzqB=jIzq

            p=0.01
            y=y+(y1 * p)

            if self.controller._control_steer > 0:
                x2 = x2+(dif * y/4)
                x3 = x3+(dif * y/4)
            elif self.controller._control_steer < 0:
                x2 = x2 - (dif * y/4)
                x3 = x3 - (dif * y/4)

            i=i+1







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
