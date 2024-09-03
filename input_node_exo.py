#LIBRERIAS PARA ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

#LIBRERÍAS DE ODRIVE
import odrive
from odrive.enums import *

#LIBRERÍAS GENERALES DEL SISTEMA
import sys
import os
import csv
import threading
import time
import math
import pdb
import numpy as np
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
from scipy.interpolate import CubicSpline

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node_exo')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'input_numbers', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'posiciones', self.listener_callback, 10)


        self.i = 0
        self.i_max = 200
        self.numbers_received = None





        self.odrv0=0
        self.odrv1=0
        self.t=0
        self.ecuacionGrad0=0
        self.ecuacionGrad1=0
        self.cont_vueltas=0
        self.posiciones1=[]
        self.posiciones2=[]
        self.grados1=[]
        self.grados2=[]
        self.torques1=[]
        self.torques2=[]
        self.velocidades1=[]
        self.velocidades2=[]
        self.reader1=[]
        self.reader2=[]


        self.posInicio1=0
        self.posInicio2=0

        #MEDICIONES
        self.intensidades1=[]
        self.intensidadesMOT1=[]
        self.voltajes1=[]
        self.intensidades2=[]

        #RECIBIR DATOS de matlab
        self.array_coefs=[]




        self.puntos1=np.empty((1))
        self.puntos2=np.empty((1))


        self.config()



        self.numbers_to_send=self.get_user_input()
        self.send_numbers()






    def config(self):

        print("Looking for Odrives...")
  

        odrive_serial_number_0 = "365833653432"	#Odrive arriba
        self.odrv0=odrive.find_any(serial_number=odrive_serial_number_0)
        print("ODRV0 found")



        odrive_serial_number_1 = "3464365C3330" #Odrive abajo
        self.odrv1=odrive.find_any(serial_number=odrive_serial_number_1)
        print("ODRV1 found")

        self.ser_odrive0 = serial.Serial('/dev/ttyACM0')
        self.ser_odrive1 = serial.Serial('/dev/ttyACM1')





        #ODRV0:
        self.odrv0.axis0.controller.config.vel_limit=2
        self.odrv0.axis0.controller.config.vel_limit_tolerance=5

        self.odrv0.axis0.controller.config.pos_gain=190.273
        self.odrv0.axis0.controller.config.vel_gain=0.449
        self.odrv0.axis0.controller.config.vel_integrator_gain=20.159

        self.odrv0.config.dc_bus_overvoltage_trip_level=27
        self.odrv0.config.dc_bus_undervoltage_trip_level=15
        self.odrv0.config.dc_max_positive_current=14
        self.odrv0.config.dc_max_negative_current=-0.5


        self.odrv0.axis0.config.motor.current_soft_max=70

        self.odrv0.axis0.config.motor.torque_constant=8.27/150
        self.odrv0.axis0.config.motor.current_control_bandwidth=1000


        self.odrv0.axis0.config.torque_soft_max=7


        #self.odrv0.axis0.config.I_bus_hard_max=70
        self.odrv0.axis0.motor.motor_thermistor.config.enabled=False


        self.odrv0.axis0.requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.control_mode=CONTROL_MODE_POSITION_CONTROL



        if self.odrv0.axis0.controller.input_pos >= 0:
            if self.odrv0.axis0.controller.input_pos >= 0.5 or self.odrv0.encoder_estimator0.pos_estimate >= 0.5:
                self.posInicio1=self.odrv0.encoder_estimator0.pos_estimate-1-self.odrv0.axis0.controller.input_pos
            else:
                self.posInicio1=self.odrv0.encoder_estimator0.pos_estimate-self.odrv0.axis0.controller.input_pos


        if self.odrv0.axis0.controller.input_pos < 0:
            if self.odrv0.axis0.controller.input_pos <= -0.5:
                self.posInicio1=self.odrv0.encoder_estimator0.pos_estimate-1+self.odrv0.axis0.controller.input_pos

            elif self.odrv0.encoder_estimator0.pos_estimate >= 0.5:
                self.posInicio1=self.odrv0.encoder_estimator0.pos_estimate-1-self.odrv0.axis0.controller.input_pos
            else:
                self.posInicio1=self.odrv0.encoder_estimator0.pos_estimate+self.odrv0.axis0.controller.input_pos








        #ODRV1:

        self.odrv1.axis0.controller.config.vel_limit=2
        self.odrv1.axis0.controller.config.vel_limit_tolerance=5

        self.odrv1.axis0.controller.config.pos_gain=190.273
        self.odrv1.axis0.controller.config.vel_gain=0.449
        self.odrv1.axis0.controller.config.vel_integrator_gain=20.159

        self.odrv1.config.dc_bus_overvoltage_trip_level=27
        self.odrv1.config.dc_bus_undervoltage_trip_level=15
        self.odrv1.config.dc_max_positive_current=14
        self.odrv1.config.dc_max_negative_current=-0.5

        self.odrv1.axis0.config.motor.current_soft_max=70

        self.odrv1.axis0.config.motor.torque_constant=8.27/150
        self.odrv1.axis0.config.motor.current_control_bandwidth=1000


        self.odrv1.axis0.config.torque_soft_max=7

        self.odrv1.axis0.motor.motor_thermistor.config.enabled=False



        self.odrv1.axis0.requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv1.axis0.controller.config.control_mode=CONTROL_MODE_POSITION_CONTROL



        if self.odrv1.axis0.controller.input_pos >= 0:
            if self.odrv1.axis0.controller.input_pos >= 0.5 or self.odrv1.encoder_estimator0.pos_estimate >= 0.5:
                self.posInicio2=self.odrv1.encoder_estimator0.pos_estimate-1-self.odrv1.axis0.controller.input_pos
            else:
                self.posInicio2=self.odrv1.encoder_estimator0.pos_estimate-self.odrv1.axis0.controller.input_pos


        if self.odrv1.axis0.controller.input_pos < 0:
            if self.odrv1.axis0.controller.input_pos <= -0.5:
                self.posInicio2=self.odrv1.encoder_estimator0.pos_estimate-1+self.odrv1.axis0.controller.input_pos

            elif self.odrv1.encoder_estimator0.pos_estimate >= 0.5:
                self.posInicio2=self.odrv1.encoder_estimator0.pos_estimate-1-self.odrv1.axis0.controller.input_pos
            else:
                self.posInicio2=self.odrv1.encoder_estimator0.pos_estimate+self.odrv1.axis0.controller.input_pos

        print("Configuración realizada")

        time.sleep(1)


    def get_user_input(self):
        print("¿Quiere elegir una trayectoria manualmente o enviarla desde otra terminal?")
        print("1.Elegir manualmente")
        print("2.Enviarla")

        tipo_envio=int(input("Seleccione una opción (1-2): "))
        print("\n")

        if tipo_envio==1:
            print("Elija un tipo de rehabilitación")
            print("1.Círculo")
            print("2.Cuadrado")
            print("3.Marcha")
            trayectoria =int(input("Seleccione una opción (1-3): "))

        print("¿Cuantas vueltas quieres que dé?")
        cantidad_vueltas=int(input("Indique un numero: "))

        return tipo_envio, trayectoria, cantidad_vueltas


    def send_numbers(self):
        msg = Int32MultiArray()
        msg.data = self.numbers_to_send
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sending numbers: {self.numbers_to_send}')



    def listener_callback(self, msg):
        pos1, pos2 = msg.data
        pos_resta1=pos1-self.posInicio1
        pos_resta2=pos2-self.posInicio2

        self.move_motors(pos_resta1, pos_resta2)



    def move_motors(self, pos1, pos2):
        self.odrv0.axis0.controller.input_pos=pos1
        self.odrv1.axis0.controller.input_pos=pos2
        self.get_logger().info(f'MOTOR0: {pos1};  MOTOR1: {pos2}')
    




def main(args=None):

    rclpy.init(args=args)
    node = InputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
