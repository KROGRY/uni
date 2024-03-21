#!/usr/bin/env python

import rospy
import math
import time
from sensor_msgs.msg import LaserScan


def lidar_callback(msg):
    # Obtener datos del LIDAR
    ranges = msg.ranges
    '''
    angleMin=round(math.degrees(msg.angle_min),2)
    angleMax=round(math.degrees(msg.angle_max),2)
    angleInc=round(math.degrees(msg.angle_increment),2)
    intervalo=msg.time_increment
    distMin=msg.range_min
    distMax=msg.range_max
    '''
    # Definir umbral de detección de obstáculos
    #umbral_obstaculo = 1.0  # Ajusta según sea necesario

    #dibujamos el valor, para saber que respuesta se nos da
    #print(ranges,end='\r')

    #definimos la detección
    #Obstaculo=round(ranges[360],2)
    

    #print("se mide desde "+str(angleMin)+" a "+str(angleMax)+", cada "+str(angleInc))
    #print("medicion cada "+str(intervalo)+" seg")
    #print("Obstaculos a "+str(Obstaculo)+"m",end='\r')

    #print(" a "+str(angulo_deseado)+" tenemos el obstaculo a "+str(round(ranges[(89-angulo_deseado)*4],2)),end='\r')
    print("Minimo: "+str(round(min(ranges[300:420]),2)),end='\r')

    '''
    # Verificar si hay obstáculos
    for distancia in ranges:
        if distancia < umbral_obstaculo:
            print("¡Posible choque detectado!")
    '''

def obstacle_detection_node():
    # Inicializar nodo de ROS
    rospy.init_node('Nodo_Grupo6', anonymous=True)

    # Suscribirse al tópico del LIDAR
    rospy.Subscriber('/scan', LaserScan, lidar_callback)

    #se lanza un timer para hacer lecturas periodicas
    rospy.Timer(rospy.Duration(1,0), periodic_callback)

    # Mantener el nodo en ejecución
    rospy.spin()


def periodic_callback(event):
    pass

if __name__ == '__main__':
    try:
        
        angulo_deseado = 200
        while ((angulo_deseado<=-90)or(angulo_deseado >= 90)):
            angulo_deseado = int(input("Define un ángulo entre -90 y 90 grados. ¿que ángulo quieres?:"))
            

        obstacle_detection_node()
    except rospy.ROSInterruptException:
        pass
