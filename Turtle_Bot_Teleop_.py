# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from pynput import keyboard
from pyfiglet import Figlet
import time


class Turtle_Bot_Teleop(Node):

    def __init__(self):
        super().__init__('turle_bot_teleop')
        self.cmd_velPublisher = self.create_publisher(Twist,'/turtlebot_cmdVel' ,10)
        #Crear el mensaje de la velocidad
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0
        self.list_tics = []
        # Print cool
        self.fuente=Figlet(font="graffiti")
        print(self.fuente.renderText("Bienvenido!"))
        print(self.fuente.renderText("Hincha de millos!\n"))
        #Pedir al usuario las velocidades
        self.angular_vel=float(input("Ingrese la velocidad angular a la quequiere que el robot se mueva: "))
        self.lineal_vel=float(input("Ingrese la velocidad lineal a la que quiere que el robot se mueva: "))
        self.ruta = input("Ingrese la ruta en la que se va a guardar el recorrido: ")
        with open(self.ruta, 'w') as self.f:
            self.f.write(str(self.lineal_vel)+'\n')
        
        with open(self.ruta, 'a') as self.f:
            self.f.write(str(self.angular_vel)+'\n')
    
        #Inicializar el listener del teclado
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()
    def adelante(self):
        print("El robot se esta moviendo adelante")
        self.vel.linear.x=self.lineal_vel
        self.cmd_velPublisher.publish(self.vel)
    def atras(self):
        print("El robot se esta moviendo para atras")
        self.vel.linear.x=-self.lineal_vel
        self.cmd_velPublisher.publish(self.vel)    
    def girar_izquierda(self):
        print("El robot esta girando a la izquierda")
        self.vel.angular.z=self.angular_vel
        self.cmd_velPublisher.publish(self.vel)
    def girar_derecha(self):
        print("El robot esta girando a la derecha")
        self.vel.angular.z=-self.angular_vel
        self.cmd_velPublisher.publish(self.vel)
    def on_press(self,key):
        try:
            #Todo: asignar las teclas respectivas a una orden que permita al robot realizar una accion
            print('Buena Bro! Estas oprimiendo la tecla {0}'.format(
                key.char))
            if key.char=='w':
                self.adelante()
            if key.char=='s':
                self.atras()
            if key.char=='e':
                self.girar_derecha()
            if key.char=='q':
                self.girar_izquierda()
        except AttributeError:
            print('A que juegas brother? Deja de oprimir teclas raras, oprimiste la tecla {0}'.format(
                key))
    def on_release(self,key):
        print('Que lastimaa bro, dejaste de oprimir la tecla {0} '.format(
            key))
        self.tic = time.perf_counter()
        self.list_tics.append(self.tic)
        print(self.list_tics)
        try:
            if key.char=='w' or key.char=='s' or key.char=='e' or key.char=='q':
                self.vel.linear.x = 0.0
                self.vel.linear.y = 0.0
                self.vel.linear.z = 0.0
                self.vel.angular.x = 0.0
                self.vel.angular.y = 0.0
                self.vel.angular.z = 0.0
                self.cmd_velPublisher.publish(self.vel)
                self.tiempo = round(self.list_tics[-1]-self.list_tics[-2],2)
                print(self.tiempo)
                with open(self.ruta, 'a') as self.f:
                    self.f.write(str(key.char)+','+str(self.tiempo)+'\n')
                
            if key == keyboard.Key.esc:
                # Stop listener
                return False
        except AttributeError:
            print('Que lastimaa bro, dejaste de oprimir la tecla {0} '.format(
            key))

def main(args=None):
    rclpy.init(args=args)

    turle_bot_teleop = Turtle_Bot_Teleop()

    rclpy.spin(turle_bot_teleop)



if __name__ == '__main__':
    print("Node intialized")
    main()