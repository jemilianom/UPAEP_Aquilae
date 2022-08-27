#!/usr/bin/env python3

import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist , PoseStamped
import smach

import ros_numpy
from utils_evasion import *
import tf2_ros


########## Functions for takeshi states ##########


def get_coords ():
    for i in range(10):   ###TF might be late, try 10 times
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            return trans
        except:
            print ('waiting for tf')
            trans=0
            





def move_base_vel(vx, vy, vw):
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw 
    base_vel_pub.publish(twist)

def move_base(x,y,yaw,timeout=5):
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < timeout:  
        move_base_vel(x, y, yaw) 


def move_forward():
    move_base(0.15,0,0,2.5)
def move_backward():
    move_base(-0.15,0,0,1.5)
def turn_left():
    move_base(0.0,0,0.12*np.pi,2)
def turn_right():
    move_base(0.0,0,-0.12*np.pi,2)

def get_lectura_cuant():
    try:
        lectura=np.asarray(laser.get_data().ranges)
        lectura=np.where(lectura>20,20,lectura) #remove infinito

        right_scan=lectura[:300]
        left_scan=lectura[300:]
        ront_scan=lectura[300:360]

        sd,si,sf=0,0
        if np.mean(left_scan)< 3: si=1
        if np.mean(right_scan)< 3: sd=1
        if np.mean(ront_scan)< 3: sf=1

    except:
        sd,si,sf=0,0    

    return si,sd,sf




##### Define state INITIAL #####

class Inicio (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ','fail']) #shor for success
        
        


    def execute(self,userdata):
    	# Aqui va lo que se desea ejecutar en el estado
    	
    	#posicionar robot

        print('inicializando')
        ########
        
        rospy.sleep(1)#### dar tiempo al arbol de tfs de publicarse
        
       
        ##### agregue código para leer la meta en el tópico adecuado
        
        #meta_leida = PoseStamped()
        #meta_leida = meta_publisher()
        
        
        ############


        ######################################################################
        meta_leida = PoseStamped() 
         ##### EJEMPLO los equipos deben leer la meta del topico, comentar esta linea
        ####################################################################
        punto_inicial = get_coords()
        print ( 'tiempo = '+ str(punto_inicial.header.stamp.to_sec()) , punto_inicial.transform )
        print ('meta leida', meta_leida)
        print('arrancando')
        return 'succ'

class S0(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
    	# Aqui va lo que se desea ejecutar en el estado

        print('robot Estado S_0')
        #####Accion
        
        if (Dx==0 and sf==0): 
            #####Accion
        	move_forward() #est0
        	return 'outcome1'
        if (Dx==0 and sf==1 and sd==0): return 'outcome2' #est1
        if (Dx==0 and sf==1 and sd==1 and si==0 ): return 'outcome4' #est7
        if (Dx==0 and sf==1 and sd==1 and si==1 ): return 'outcome5' #est5
        if (Dx==1 and Dy==0 and mDy==0 ): return 'outcome6' #est12
        if (Dx==1 and Dy==0 and mDy==1 ): return 'outcome7' #est11
        if (Dx==1 and Dy==1): return 'outcome8' #est13
        

class S1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
    	# Aqui va lo que se desea ejecutar en el estado

        print('robot Estado S_1')
        #####Accion
        turn_right()

        return 'outcome1'


class S2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
      
        


    def execute(self,userdata):
        print('robot Estado S_2')
        #####Accion
        move_forward()        
        
        if (si==0): return 'outcome1'
        if (si==1): return 'outcome2'

	return 'outcome1'
        
class S3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_3')
        #####Accion
	move_forward()
	
        if (si==0): return 'outcome1'
        if (si==1): return 'outcome2'

        return 'outcome1'

class S4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_4')
        #####Accion
	turn_left()
        return 'outcome1'

class S5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_5')
        #####Accion
	move_backward()
	
        return 'outcome1'

class S6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self,userdata):
        print('robot Estado S_6')
        #####Accion
	turn_left
	
        return 'outcome1'
    
class S7(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
    	# Aqui va lo que se desea ejecutar en el estado

        print('robot Estado S_7')
        #####Accion
	turn_left()
        return 'outcome1'


class S8(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
      
        


    def execute(self,userdata):
        print('robot Estado S_8')
        #####Accion
        move_forward()
        
        if (sd==0): return 'outcome1'
        if (sd==1): return 'outcome2'



        
class S9(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_9')
        #####Accion
        move_forward()
        
        if (sd==0): return 'outcome1'
        if (sd==1): return 'outcome2'

class S10(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_10')
        #####Accion
	turn_right()
        return 'outcome1'

class S11(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_11')
        #####Accion
        turn_right():

        return 'outcome1'

class S12(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_12')
        #####Accion
	turn_left()
        return 'outcome1'

class S13(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_13')
        #####Accion

        return 'outcome1'


def init(node_name):
    global laser, base_vel_pub
    rospy.init_node(node_name)
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    meta_leida = rospy.Subscriber('/meta_competencia', PoseStamped, queue_size=10)
    laser = Laser()  


#Entry point
if __name__== '__main__':

    print("STATE MACHINE...")
    init("smach_node")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    with sm:
        #State machine for evasion
        smach.StateMachine.add("INICIO",   Inicio(),  transitions = {'fail':'END', 'succ':'s_0'})
        smach.StateMachine.add("s_0",   S0(),  transitions = {'outcome1':'s_0','outcome2':'s_1','outcome4':'s_7','outcome5':'s_5','outcome6':'s_12','outcome7':'s_11','outcome8':'s_13','outcome9':'END'})
        smach.StateMachine.add("s_1",   S1(),  transitions = {'outcome1':'s_2','outcome2':'END'})
        smach.StateMachine.add("s_2",   S2(),  transitions = {'outcome1':'s_4','outcome2':'s_3','outcome3':'END'})
        smach.StateMachine.add("s_3",   S3(),  transitions = {'outcome1':'s_4','outcome2':'s_3','outcome3':'END'})
        smach.StateMachine.add("s_4",   S4(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_5",   S5(),  transitions = {'outcome1':'s_6','outcome2':'END'})
        smach.StateMachine.add("s_6",   S6(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_7",   S7(),  transitions = {'outcome1':'s_8','outcome2':'END'})
        smach.StateMachine.add("s_8",   S8(),  transitions = {'outcome1':'s_9','outcome2':'END'})
        smach.StateMachine.add("s_9",   S9(),  transitions = {'outcome1':'s_9','outcome2':'s_8','outcome3':'END'})
        smach.StateMachine.add("s_10",   S10(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_11",   S11(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_12",   S12(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_13",   S13(),  transitions = {'outcome1':'END'})


outcome = sm.execute()


























