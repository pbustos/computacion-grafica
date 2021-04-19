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
from genericworker import *
import traceback
import pygame
import pygame.gfxdraw
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
        self.camera_manager.render(self.display)
        self.hud.render(self.display)

        # Calculo del ángulo de  giro
        print('giro---',self.controller._control_steer)
        if self.controller._control_steer == 0.0:
            alfa = (np.pi/2) * 0.0000001
        else:
            alfa = (np.pi/2) * self.controller._control_steer

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


        #Obtencion puntos

        # Si va hacia delante, calculo poligono
        if (not self.controller._control_reverse):
            # Si está en movimiento, calculo puntos con velocidad y angulo de giro
            if (speed > 0 ):
                puntoDer = np.array([3*(speed/24)+1,0.5 + alfa,0.1])
                puntoIzq = np.array([3*(speed/24)+1,-0.5 + alfa,0.1])
            else: # Si no está en movimiento, no se tiene en cuenta el giro
                puntoDer = np.array([3 * (speed / 24) + 1, 0.5, 0.1])
                puntoIzq = np.array([3 * (speed / 24) + 1, -0.5, 0.1])
        else: # Si va hacia detrás
            puntoDer = np.array([3 * (0 / 24) + 1, 0.5, 0.1])
            puntoIzq = np.array([3 * (0 / 24) + 1, -0.5, 0.1])

        faroDer = np.array([1, 1, 0.1])
        faroIzq = np.array([1, -1, 0.1])

        camara = np.array([0.0,-0.25,1.0])

        puntoPintarDer = puntoDer-camara
        puntoPintarIzq = puntoIzq-camara

        pintarFaroDer = faroDer - camara
        pintarFaroIzq = faroIzq - camara


        #semejanza de triangulos
        f = 320.0/np.tan(np.radians(110.0/2.0))
        iDer = (f*puntoPintarDer[1])/puntoPintarDer[0] + self.width/2
        jDer = (f*puntoPintarDer[2])/puntoPintarDer[0] + self.height/2
        jDer = self.height - jDer

        iFaroDer = (f * pintarFaroDer[1]) / pintarFaroDer[0] + self.width / 2
        jFaroDer = (f * pintarFaroDer[2]) / pintarFaroDer[0] + self.height / 2
        jFaroDer = self.height - jFaroDer

        #print('-----------',puntoPintarDer[1], puntoPintarIzq[1] )

        iIzq = (f * puntoPintarIzq[1]) / puntoPintarIzq[0] + self.width / 2
        jIzq = (f * puntoPintarIzq[2]) / puntoPintarIzq[0] + self.height / 2
        jIzq = self.height - jIzq;

        iFaroIzq = (f * pintarFaroIzq[1]) / pintarFaroIzq[0] + self.width / 2
        jFaroIzq = (f * pintarFaroIzq[2]) / pintarFaroIzq[0] + self.height / 2
        jFaroIzq = self.height - jFaroIzq;

        #print('Der',jDer, iDer)
        #print('Izq',jIzq, iIzq)

        """
        pygame.draw.circle(self.display, (0,0,255), [int(iDer), int(jDer)], 5)
        pygame.draw.circle(self.display, (0, 0, 255), [int(iIzq), int(jIzq)], 5)

        pygame.draw.circle(self.display, (0, 0, 255), [int(iFaroDer), int(jFaroDer)], 5)
        pygame.draw.circle(self.display, (0, 0, 255), [int(iFaroIzq), int(jFaroIzq)], 5)
        """
        
        pygame.gfxdraw.polygon(self.display,
                            [(iFaroIzq, jFaroIzq), (iFaroDer, jFaroDer), (iDer, jDer), (iIzq, jIzq)],
                               (0, 0, 255))
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
        #print(gnssData.latitude, gnssData.longitude)
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

