#LIBRERÍAS PARA ROS2
import rclpy

#LIIBRERÍAS PARA RECIBIR DATOS DE MATLAB
import socket

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


from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3


#from submodules.odrive_controller import OdriveController
from submodules.pruebaMotor import PruebaOdriveController


#LIBRERÍAS DE ODRIVE
import odrive
from odrive.enums import *

import datetime

odrv0=0
odrv1=0
t=0
ecuacionGrad0=0
ecuacionGrad1=0
cont_vueltas=0
posiciones1=[]
posiciones2=[]
grados1=[]
grados2=[]
torques1=[]
torques2=[]
velocidades1=[]
velocidades2=[]
reader1=[]
reader2=[]



#MEDICIONES
intensidades1=[]

voltajes1=[]
intensidades2=[]
voltajes2=[]

#RECIBIR DATOS de matlab
array_coefs=[]




puntos1=np.empty((1))
puntos2=np.empty((1))


#OPCIONES DE LECTURA:

    #DEBUGGER
    #pdb.set_trace()

    #INTENSIDAD DE ODRIVE
    #odrv0.ibus

    #CALIBRADO DEL MOTOR
    #odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE




def config():
    global odrv0, odrv1

    
    #ODRV0:
    odrv0.axis0.controller.config.vel_limit=2
    odrv0.axis0.controller.config.vel_limit_tolerance=5

    odrv0.axis0.controller.config.pos_gain=190.273
    odrv0.axis0.controller.config.vel_gain=0.449
    odrv0.axis0.controller.config.vel_integrator_gain=20.159

    odrv0.config.dc_bus_overvoltage_trip_level=27
    odrv0.config.dc_bus_undervoltage_trip_level=15
    odrv0.config.dc_max_positive_current=14
    odrv0.config.dc_max_negative_current=-0.5


    odrv0.axis0.config.motor.current_soft_max=70

    odrv0.axis0.config.motor.torque_constant=8.27/150
    odrv0.axis0.config.motor.current_control_bandwidth=1000


    odrv0.axis0.config.torque_soft_max=7


    #odrv0.axis0.config.I_bus_hard_max=70
    odrv0.axis0.motor.motor_thermistor.config.enabled=True



    #ODRV1:

    odrv1.axis0.controller.config.vel_limit=2
    odrv1.axis0.controller.config.vel_limit_tolerance=5

    odrv1.axis0.controller.config.pos_gain=190.273
    odrv1.axis0.controller.config.vel_gain=0.449
    odrv1.axis0.controller.config.vel_integrator_gain=10#20.159

    odrv1.config.dc_bus_overvoltage_trip_level=27
    odrv1.config.dc_bus_undervoltage_trip_level=15
    odrv1.config.dc_max_positive_current=14
    odrv1.config.dc_max_negative_current=-0.5

    odrv1.axis0.config.motor.current_soft_max=70

    odrv1.axis0.config.motor.torque_constant=8.27/150
    odrv1.axis0.config.motor.current_control_bandwidth=1000


    odrv1.axis0.config.torque_soft_max=7

    odrv1.axis0.motor.motor_thermistor.config.enabled=True






def recibir_datos():

    global array_coefs

    server=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

    server_ip = "0.0.0.0"  #EScuchar en todas las interfaces
    port = 8080  #Puerto que está escuchando

    #Vincular el socket a una dirección y puerto específicos
    server.bind((server_ip, port))

    #Escuchar conexiones entrantes
    server.listen(1)
    print(f"Escuchando en {server_ip}:{port}")
    print("Esperando conexion...")

    #Aceptar conexiones entrantes
    client_socket, client_address = server.accept()
    print(f"Aceptada conexion de {client_address[0]}:{client_address[1]}")

    while True:
        #Recibir datos del cliente
        data=client_socket.recv(1024)
        data=data.decode("utf-8")  #Convertir bytes a cadena

        if not data:
            break

        print(f"Datos recibidos: {data}")

        #Divido el menssaje
        string_recibido=data.split(';')

        for cadena in string_recibido:
            array_coef=np.fromstring(cadena.strip(), sep='     ')
            array_coefs.append(array_coef)

        print(f"Multiples arrays recibidos: {array_coefs}")

    #Cerrar conexión con el cliente
    client_socket.close()
    print("Conexion con el cliente cerrada")
    #Cerrar el socket del servidor
    server.close()
    print("Servidor cerrado")



    #pdb.set_trace()


    
def Motor0():

    global odrv0, t, ecuacionGrad0, cont_vueltas, t_final, posiciones1, grados1, torques1, velocidades1, reader1, voltajes1, intensidades1, puntos1
    
    
    odrv0.axis0.requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis0.controller.config.control_mode=CONTROL_MODE_POSITION_CONTROL


   # if odrv0.encoder_estimator0.pos_estimate >= 0.5:
    #    posInicio1=odrv0.encoder_estimator0.pos_estimate-1-odrv0.axis0.controller.input_pos
    #elif odrv0.encoder_estimator0.pos_estimate <= -0.5:
     #   posInicio1=odrv0.encoder_estimator0.pos_estimate+1-odrv0.axis0.controller.input_pos
    #else :
     #   posInicio1=odrv0.encoder_estimator0.pos_estimate-odrv0.axis0.controller.input_pos



    if odrv0.axis0.controller.input_pos >= 0:
        if odrv0.axis0.controller.input_pos >= 0.5 or odrv0.encoder_estimator0.pos_estimate >= 0.5:
            posInicio1=odrv0.encoder_estimator0.pos_estimate-1-odrv0.axis0.controller.input_pos
        else:
            posInicio1=odrv0.encoder_estimator0.pos_estimate-odrv0.axis0.controller.input_pos


    if odrv0.axis0.controller.input_pos < 0:
        if odrv0.axis0.controller.input_pos <= -0.5:
            posInicio1=odrv0.encoder_estimator0.pos_estimate-1+odrv0.axis0.controller.input_pos

        elif odrv0.encoder_estimator0.pos_estimate >= 0.5:

            posInicio1=odrv0.encoder_estimator0.pos_estimate-1-odrv0.axis0.controller.input_pos
        else:
            posInicio1=odrv0.encoder_estimator0.pos_estimate+odrv0.axis0.controller.input_pos


    print("Pos Inicio1: " + str(posInicio1))
    print("input_pos1: " + str(odrv0.axis0.controller.input_pos))
    print("pos_estimate1: " + str(odrv0.encoder_estimator0.pos_estimate))


    #SOLO PARA EL PUNTO INICIAL
    #x_points1=np.array([0,1])
    #y_points1=np.array([posInicio1,ecuacionGrad0(t[0])])
    #cs=CubicSpline(x_points1,y_points1)
    #time.sleep(1)



    #pdb.set_trace()

    #odrv0.axis0.controller.input_pos=ecuacionGrad0(t[0])/360-posInicio1
    time.sleep(1)

    for i in range(len(t)):
        pos1=(-1)*ecuacionGrad0(t[i])/360
        #pos1=reader1[0][i]/360


        odrv0.axis0.controller.input_pos=pos1-posInicio1

        time.sleep(0.02)

        posicion1=odrv0.encoder_estimator0.pos_estimate
        torq1=odrv0.axis0.motor.torque_estimate
        vel1=odrv0.encoder_estimator1.vel_estimate
        int1=odrv0.ibus
        intMOT1=odrv0.axis0.motor.alpha_beta_controller.I_bus

        volt1=odrv0.vbus_voltage

        posicion1=odrv0.encoder_estimator1.pos_estimate*(-1)
        posiciones1.append(posicion1)        
        posGrad1=posicion1*360
        grados1.append(posGrad1)
        torques1.append(torq1)
        velocidades1.append(vel1)
        intensidades1.append(int1)
        voltajes1.append(volt1)
        time.sleep(0.01)

    

def Motor1():

    global odrv1, t, ecuacionGrad1, cont_vueltas, t_final, posiciones2, grados2, torques2, velocidades2, reader2, voltajes2, puntos2, intensidades2
    
    
    odrv1.axis0.requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv1.axis0.controller.config.control_mode=CONTROL_MODE_POSITION_CONTROL


    if odrv1.axis0.controller.input_pos >= 0:
        if odrv1.axis0.controller.input_pos >= 0.5 or odrv1.encoder_estimator0.pos_estimate >= 0.5:
            posInicio2=odrv1.encoder_estimator0.pos_estimate-1-odrv1.axis0.controller.input_pos
        else:
            posInicio2=odrv1.encoder_estimator0.pos_estimate-odrv1.axis0.controller.input_pos


    if odrv1.axis0.controller.input_pos < 0:
        if odrv1.axis0.controller.input_pos <= -0.5:
            posInicio2=odrv1.encoder_estimator0.pos_estimate-1+odrv1.axis0.controller.input_pos

        elif odrv1.encoder_estimator0.pos_estimate >= 0.5:

            posInicio2=odrv1.encoder_estimator0.pos_estimate-1-odrv1.axis0.controller.input_pos
        else:
            posInicio2=odrv1.encoder_estimator0.pos_estimate+odrv1.axis0.controller.input_pos





   # elif odrv1.axis0.controller.input_pos <= -0.5:
    #    posInicio2=odrv1.encoder_estimator0.pos_estimate+1-odrv1.axis0.controller.input_pos
    #else :
     #   posInicio2=odrv1.encoder_estimator0.pos_estimate-odrv1.axis0.controller.input_pos

    print("Pos Inicio: " + str(posInicio2))
    print("input_pos: " + str(odrv1.axis0.controller.input_pos))
    print("pos_estimate: " + str(odrv1.encoder_estimator0.pos_estimate))


    #odrv1.axis0.controller.input_pos=ecuacionGrad1(t[0])/360-posInicio2
    time.sleep(1)
    
   # for i in range(len(puntos2)):
    #    pos2=(-1)*puntos2[i]/360
    for i in range(len(t)):
        pos2=(-1)*ecuacionGrad1(t[i])/360
        #pos2=reader2[0][i]/360

        odrv1.axis0.controller.input_pos=pos2-posInicio2
        #print("input_pos: " + str(odrv1.axis0.controller.input_pos))
        torq2=odrv1.axis0.motor.torque_estimate
        vel2=odrv1.encoder_estimator1.vel_estimate
        int2=odrv1.ibus

        volt2=odrv1.vbus_voltage
        time.sleep(0.02)

        posicion2=odrv1.encoder_estimator1.pos_estimate*(-1)
        posGrad2=posicion2*360
        grados2.append(posGrad2)
        posiciones2.append(posicion2)
        intensidades2.append(int2)
        torques2.append(torq2)
        voltajes2.append(volt2)
        velocidades2.append(vel2)
        time.sleep(0.01)






def main(args=None):

    global odrv0, odrv1, t, ecuacionGrad0, ecuacionGrad1, cont_vueltas, t_final, reader1, reader2, puntos1, puntos2

    #DATOS GUARDADOS
    global posiciones1, posiciones2, grados1, grados2, torques1, velocidades1, torques2, velocidades2

    #MEDICIONES
    global voltajes1, intensidades1, intensidades2, voltajes2

    #RECIBIR DATOS DE MATLAB
    global array_coefs




    archivoT="datosTIEMPOScircSIN.txt"
    archivo1="datosGRADOS1circSIN.txt"
    archivo2="datosGRADOS2circSIN.txt"
    archivo1="datosTORQUES1circSIN.txt"
    archivo2="datosTORQUES2circSIN.txt"
    archivo1="datosVELOCIDADES1circSIN.txt"
    archivo2="datosVELOCIDADES2circSIN.txt"
    archivo2="datosINTENSIDADES1circSIN.txt"
    archivo2="datosINTENSIDADES2circSIN.txt"


    df=pd.read_csv('coord_circulo_coef1B.csv')

    puntos1=df.values

    df=pd.read_csv('coord_circulo_coef2B.csv')

    puntos2=df.values

  #  with open('coord_circulo_coef1B.csv', 'r') as file:
   #     lector1=csv.reader(file)
   #     for row in lector1:
    #        coor1= row
     #       puntos1=np.append(puntos1, coor1)


 #   with open('coord_circulo_coef2B.csv', 'r') as file:
  #      lector2=csv.reader(file)
   #     for row in lector2:
    #        coor2= row
     #       puntos2=np.append(puntos2, coor2)


    
    sumPos=0


    odrive_serial_number_0 = "3464365C3330"   #"345036583330"	#Odrive abajo
    odrive_serial_number_1 = "365833653432"	#Odrive arriba


    


    odrv0=odrive.find_any(serial_number=odrive_serial_number_0)
    odrv1=odrive.find_any(serial_number=odrive_serial_number_1)


    
    config()


    print("¿Quiere elegir una trayectoria manualmente o enviarla desde otra terminal?")
    print("1.Elegir manualmente")
    print("2.Enviarla")

    tray=input("Seleccione una opción (1-2): ")
    print("\n")


    if tray=='1':


        print("Elija un tipo de rehabilitación")
        print("1.Círculo")
        print("2.Cuadrado")
        print("3.Marcha")
        opcion=input("Seleccione una opción (1-3): ")

        if opcion=='1':
            #CIRCULO
            #coef1=[1.5246, -17.9064, 75.0663, -128.7078, 73.9772, -15.5865, 14.7962]
            #coef2=[0.7942, -11.1785, 58.5693, -133.1539, 109.2037, -3.6494, 94.7886]
            coef1=[0.137577211719669, -2.48972564334658, 18.6672057569009, -74.3470136010840, 167.361333241395, -210.210102317778, 140.285367036520, -43.3859821322072, -18.7767439377094, 13.8023028467493, 12.2929700006286]
            coef2=[1.21318322602377, -24.3092902965526, 208.101039734157, -991.075662273051, 2863.95320859601, -5139.98492986922, 5652.65454614971, -3666.91789826732, 1319.74593743198, -193.830628824605, 122.705430633119]
            #Principio, fin y cada cuanto tiempo se pasan datos
            start_time=0
            end_time=3.7
            num_points=200

        elif opcion=='2':
            #CUADRADO
            #coef1=[-4.14957958399955e-05, 0.00214888718435162, -0.0472933702267780, 0.575766163547228, -4.22900521485169, 19.1888887521228, -52.9723483877474, 84.2254947645377, -70.0065704656505, 36.3393961144970, -7.31951161181147]
            #coef2=[-2.60461191614922e-07, -0.000294153795122175 ,0.0134233694149718, -0.249881713352434, 2.48184847950910, -14.2218017604551, 47.2238188985222, -86.1069089022367, 78.4927780954936, -41.4756393688662, 135.352251729471]
            coef1=[-4.31126322645301e-05, 0.00222479590688393, -0.0487985046271944, 0.592173687014159, -4.33625670793448, 19.6191002076906, -54.0129301000008, 85.6552584939307, -70.9916316309844, 36.6027770872656, -7.33440515767211]
            coef2=[-8.73532990274809e-06, 0.000289779830869871, -0.00256247381090580, -0.0187007654717237, 0.534165219946363, -4.36854269335279, 17.6215680360584, -36.0188825747978, 34.5407717114723, -13.5629691306618, 97.2477477569441]

            reader1=pd.read_csv('angle_motor1.csv', header=None)

            reader2=pd.read_csv('angle_motor2.csv', header=None)

            #SPLINE
    
            #Principio, fin y cada cuanto tiempo se pasan datos
            start_time=0
            end_time=10
            num_points=200

        elif opcion=='3':
            #MARCHA
            ####coef1=[2.03229697301878e-05, -0.00129122992977727, 0.0341793914221324, -0.491564801406733, 4.19812766921453, -21.8677582396702, 68.6946567565940, -123.278355571656, 111.537303655602, -47.0649434008433, 1.55861308759037]
            ####coef2=[-8.85918055748844e-06, 0.000486513209553566, -0.0117494683613398, 0.161335405541034, -1.36281689588270, 7.20351406087949, -23.3534932957313, 43.5231987485680, -40.1832167375279, 16.9281943569151, 101.669417680610]
            coef1=[2.032296972945924e-05, -0.001291229929745, 0.034179391421539, -0.491564801400911, 4.198127669182287, -21.867758239576820, 68.694656756522060, -1.232783555720056e02, 1.115373036566728e02, -47.064943401983115, 1.558613087999853]
            coef2=[1.579371173058048e-05, -9.944829372789605e-04, 0.025756866455590, -0.359451017141363, 2.961914286905127, -14.824645245923930, 44.628234958723354, -76.870143563584660, 67.853987566024370, -28.131684680777090, 73.747385582571010]
            #Principio, fin y cada cuanto tiempo se pasan datos
            start_time=0
            end_time=10.1
            num_points=200
        else:
            print("NO es una opción válida")
            start_time=0
            end_time=3.7
            num_points=200
            pdb.set_trace()
            #sys.exit()


    elif tray=='2':
        recibir_datos()

        coef1=array_coefs[0]
        coef2=array_coefs[1]

        start_time=0
        end_time=array_coefs[2]
        num_points=200
        


    else:
        print("NO es una opción válida")
        sys.exit()


    
    #Lo pongo en una ecuación
    ecuacionGrad0=np.poly1d(coef1)
    ecuacionGrad1=np.poly1d(coef2)

    
    
    #Vector de tiempo
    t = np.linspace(start_time, end_time, num_points)


    
    
    #Creo los hilos
    hilo_Motor0=threading.Thread(target=Motor0)
    hilo_Motor1=threading.Thread(target=Motor1)
    
    #Inicio los hilos
    hilo_Motor0.start()
    hilo_Motor1.start()


    
    
    #Espero a que ambos hilos terminen
    hilo_Motor0.join()
    hilo_Motor1.join()
    #pdb.set_trace()


    valoresX1=ecuacionGrad0(t)/360
    valoresX2=ecuacionGrad1(t)/360
    #valoresX1=puntos1/360
    #valoresX2=puntos2/360
  #  sumX=(ecuacionGrad0(t)+ecuacionGrad1(t))/360


    sumPos=[a+b for a,b in zip(posiciones1,posiciones2)]

    #VISUALIZACIÓN DE GRÁFICAS
    arrayMod1=[]
    arrayMod2=[]


    if tray=='1':
        if opcion=='1':
            for f in range(len(posiciones1)):
                if posiciones1[f]<-0.5:
                    posmod=posiciones1[f]+1
                    arrayMod1.append(posmod)
                else:
                    posmod=posiciones1[f]
                    arrayMod1.append(posmod)
            for g in range(len(posiciones2)):
                posMod2=posiciones2[g]+1
                arrayMod2.append(posMod2)

        elif opcion=='2':
            for f in range(len(posiciones1)):
                if posiciones1[f]<-0.5:
                    posmod=posiciones1[f]+1.0025
                    arrayMod1.append(posmod)
                else:
                    posmod=posiciones1[f]
                    arrayMod1.append(posmod)
            for g in range(len(posiciones2)):
                posMod2=posiciones2[g]+1
                arrayMod2.append(posMod2)


        elif opcion=='3':
            for f in range(len(posiciones1)):
                if posiciones1[f]<-0.5:
                    posmod=posiciones1[f]+1
                    arrayMod1.append(posmod)
                else:
                    posmod=posiciones1[f]
                    arrayMod1.append(posmod)
            for g in range(len(posiciones2)):
                posMod2=posiciones2[g]+1
                arrayMod2.append(posMod2)






    fig, ax=plt.subplots()

    ax.plot(t, arrayMod1, 'b-')
    ax.plot(t, valoresX1, 'r--')

    plt.show()


    fig, ax=plt.subplots()

    ax.plot(t, arrayMod2, 'b-')
    ax.plot(t, valoresX2, 'r--')

    plt.show()

    plt.plot(t, voltajes1)
    plt.show()

    plt.plot(t, intensidades1)
    plt.show()


    plt.plot(t, voltajes2)
    plt.show()

    plt.plot(t, intensidades2)
    plt.show()

    plt.plot(t, torques1)
    plt.show()

    plt.plot(t, torques2)
    plt.show()

    plt.plot(t, velocidades1)
    plt.show()

    plt.plot(t, velocidades2)
    plt.show()

    array_pos1=np.array(grados1)
    array_pos2=np.array(grados2)
    array_torq1=np.array(torques1)
    array_torq2=np.array(torques2)
    array_vel1=np.array(velocidades1)
    array_vel2=np.array(velocidades2)
    array_time=np.array(t)
    array_int1=np.array(intensidades1)
    array_int2=np.array(intensidades2)
    array_volt1=np.array(voltajes1)
    array_volt2=np.array(voltajes2)



    with open("archivoTmarCON.txt", 'w', encoding='utf-8') as arch:
        arch.write("Tiempo\n")
        arch.write("," .join(map(str, array_time)) + '\n')

    with open("archivoM1marCON.txt", 'w', encoding='utf-8') as arch:
        arch.write("Angulos 1\n")
        arch.write("," .join(map(str, array_pos1)) + '\n')

    with open("archivoM2marCON.txt", 'w', encoding='utf-8') as arch:
        arch.write("Angulos 2\n")
        arch.write("," .join(map(str, array_pos2)) + '\n')

    with open("archivoTor1marCON.txt", 'w', encoding='utf-8') as arch:
        arch.write("Torques Motor 1\n")
        arch.write("," .join(map(str, array_torq1)) + '\n')

    with open("archivoTor2marCON.txt", 'w', encoding='utf-8') as arch:
        arch.write("Torques Motor 2\n")
        arch.write("," .join(map(str, array_torq2)) + '\n')

    with open("archivoVel1marCON.txt", 'w', encoding='utf-8') as arch:
        arch.write("Velocidades Motor 1\n")
        arch.write("," .join(map(str, array_vel1)) + '\n')

    with open("archivoVel2marCON.txt", 'w', encoding='utf-8') as arch:
        arch.write("Velocidades Motor 2\n")
        arch.write("," .join(map(str, array_vel2)) + '\n')

    with open("archivoVolt1marCON.txt", 'w', encoding='utf-8') as arch:
        arch.write("Voltajes Motor 1\n")
        arch.write("," .join(map(str, array_volt1)) + '\n')

    with open("archivoVolt2marCON.txt", 'w', encoding='utf-8') as arch:
        arch.write("Voltajes Motor 2\n")
        arch.write("," .join(map(str, array_volt2)) + '\n')

    with open("archivoInt1marCON.txt", 'w', encoding='utf-8') as arch:
        arch.write("Intensidades Motor 1\n")
        arch.write("," .join(map(str, array_int1)) + '\n')

    with open("archivoInt2marCON.txt", 'w', encoding='utf-8') as arch:
        arch.write("Intensidades Motor 2\n")
        arch.write("," .join(map(str, array_int2)) + '\n')




    

if __name__ == '__main__':
    main()    

