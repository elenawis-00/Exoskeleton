#LIBRERIAS PARA ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
from std_msgs.msg import Float32MultiArray

#LIBRERÍAS GENERALES DEL SISTEMA
import numpy as np
import time
import pdb
from scipy.interpolate import CubicSpline

class OutputNode(Node):
    def __init__(self):
        super().__init__('output_node_exo')

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'input_numbers',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'posiciones', 10)
        self.timer=None
        self.numbers_received=None

        self.initial_wait_time =0.1
        self.regular_wait_time=0.03
        self.inactivity_timeout=1 #Tiempo de espera oir inactividad


        self.regular_timer=None
        self.inactivity_timer=None

        self.ecuacionGrad0=0
        self.coef1 = 0
        self.ecuacionGrad1=0
        self.coef2 = 0
        self.t=0
        self.start_time = 0
        self.end_time = 0
        self.num_points = 0
        self.i1 = 0
        self.i2 = 0
        self.i_max = 201
        self.vueltas_dadas=1

        self.spline_motor1=0
        self.spline_motor2=0


        self.valores_motor1=[]
        self.valores_motor2=[]
        self.cantidad_vueltas=0
 

    def listener_callback(self, msg):

        self.numbers_received=msg.data
        self.get_logger().info(f'Received numbers: {self.numbers_received}')
        self.i=1
        self.i1 = 1
        self.i2 = 1

        tipo_envio, trayectoria_seguida, self.cantidad_vueltas = self.numbers_received
        if self.numbers_received is not None and self.i <= self.i_max:

            if tipo_envio==1:
                if trayectoria_seguida==1:
                    #CIRCULO
                    self.coef1=[0.137577211719669, -2.48972564334658, 18.6672057569009, -74.3470136010840, 167.361333241395, -210.210102317778, 140.285367036520, -43.3859821322072, -18.7767439377094, 13.8023028467493, 12.2929700006286]
                    self.coef2=[1.21318322602377, -24.3092902965526, 208.101039734157, -991.075662273051, 2863.95320859601, -5139.98492986922, 5652.65454614971, -3666.91789826732, 1319.74593743198, -193.830628824605, 122.705430633119]
                    #Principio, fin y cada cuanto tiempo se pasan datos
                    self.start_time=0
                    self.end_time=3.7
                    self.num_points=200  
                    self.i_max = 199

                elif trayectoria_seguida==2:
                    self.coef1=[-4.31126322645301e-05, 0.00222479590688393, -0.0487985046271944, 0.592173687014159, -4.33625670793448, 19.6191002076906, -54.0129301000008, 85.6552584939307, -70.9916316309844, 36.6027770872656, -7.33440515767211]
                    self.coef2=[-8.73532990274809e-06, 0.000289779830869871, -0.00256247381090580, -0.0187007654717237, 0.534165219946363, -4.36854269335279, 17.6215680360584, -36.0188825747978, 34.5407717114723, -13.5629691306618, 97.2477477569441]
                    self.start_time=0
                    self.end_time=10
                    self.num_points=201
                    self.i_max = self.num_points

                elif trayectoria_seguida==3:
                    self.coef1=[2.032296972945924e-05, -0.001291229929745, 0.034179391421539, -0.491564801400911, 4.198127669182287, -21.867758239576820, 68.694656756522060, -1.232783555720056e02, 1.115373036566728e02, -47.064943401983115, 1.558613087999853]
                    self.coef2=[1.579371173058048e-05, -9.944829372789605e-04, 0.025756866455590, -0.359451017141363, 2.961914286905127, -14.824645245923930, 44.628234958723354, -76.870143563584660, 67.853987566024370, -28.131684680777090, 73.747385582571010]
                    self.start_time=0
                    self.end_time=10.1
                    self.num_points=201
                    self.i_max = self.num_points 

            #elif tipo_envio==2:

            

            #Lo pongo en una ecuación
            self.ecuacionGrad0=np.poly1d(self.coef1)
            self.ecuacionGrad1=np.poly1d(self.coef2)

            #Vector de tiempo
            self.t = np.linspace(self.start_time, self.end_time, self.num_points)

            for x in range(len(self.t)):
                pos1=(-1)*self.ecuacionGrad0(self.t[x])/360
                pos2=(-1)*self.ecuacionGrad1(self.t[x])/360

                self.valores_motor1.append(pos1)
                self.valores_motor2.append(pos2)

            self.spline_motor1 = CubicSpline(self.t, self.valores_motor1)
            self.spline_motor2 = CubicSpline(self.t, self.valores_motor2)


        self.timer=self.create_timer(0.03, self.timer_callback)






    def timer_callback(self):
 
            #time_target = np.linspace(0, 1, len(self.ecuacionGrad0))

        result1=float(self.spline_motor1(self.t[self.i1]))
        result2=float(self.spline_motor2(self.t[self.i2]))
            

            #result1=self.posiciones_Motor0(self.ecuacionGrad0, self.t, self.i1)
        #result2=self.posiciones_Motor1(self.ecuacionGrad1, self.t, self.i2) 
           
            
            
        msg = Float32MultiArray()
        msg.data = [result1, result2]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: pos1={result1}, pos2={result2}, i={self.i}')
        self.i1+=1
        self.i2+=1
        self.i+=1
          
        if self.i > self.i_max and self.vueltas_dadas<self.cantidad_vueltas:
            self.i1=0
            self.i2=0
            self.i=0
            self.vueltas_dadas+=1
            #pdb.set_trace()



        elif self.i > self.i_max and self.vueltas_dadas>=self.cantidad_vueltas:
            self.get_logger().info('Reached maximum count, stopping publication')
            self.timer.cancel()
            self.numbers_received=None
            #pdb.set_trace()




    def posiciones_Motor0(self, ecuacionGrad0, t, iteracion1):
        pos1 = (-1)*ecuacionGrad0(t[iteracion1])/360
        return pos1


    def posiciones_Motor1(self, ecuacionGrad1, t, iteracion2):
        pos2 = (-1)*ecuacionGrad1(t[iteracion2])/360
        return pos2




    def stop_publication(self):
        self.get_logger().info('No new numbers received for a while. Stopping publication.')
        if self.regular_timer is not None:
            self.regular_timer.cancel()
        if self.inactivity_timer is not None:
            self.inactivity_timer.cancel()

        self.destroy_node()	#Destruye el nodo y detiene la ejecución
        rclpy.shutdown()	#Finaliza ROS2

        

def main(args=None):
    rclpy.init(args=args)
    node = OutputNode()
    rclpy.spin(node)
   # node.destroy_node()
   # rclpy.shutdown()

if __name__ == '__main__':
    main()
