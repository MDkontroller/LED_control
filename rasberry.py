#!/usr/bin/env python       

import rospy                                                                 #ROS-library
import numpy as np                                                           #zur Arrayerstellung
from std_msgs import msg                                                              
from std_msgs.msg import Int16                                              
from sensor_msgs.msg import NavSatFix                                      #imports the data type of the GPS-Sensor's topic / message
from geometry_msgs.msg import Twist 
#from test.msg import signals 


#status= signals()

#mode = 0
pub = rospy.Publisher('steuerung', Int16, queue_size = 10)                  #topic called "steuerung" has been created here. The queue is being held in case the publishing frequency is higher than the loop on the subscriber's side
mode = Int16()



def fahrtrichtung(msg):
    global mode

    rospy.loginfo("Fahrtichtung fkt called")
    if msg.angular.z > 0:               
        #status.Fahrtrichtung.data = 1                                           #rechts
        mode = 0
        #pub.publish(mode) 
    elif msg.angular.z < 0:
        #status.Fahrtrichtung.data = 2                                           #links
        mode = 1
        #pub.publish(mode)
    elif msg.linear.x < 0:
        
        #status.Fahrtrichtung.data = 3                                           #rückwärts
        mode = 2
    
    rospy.loginfo(mode)

def gps_signal(msg):          
    rospy.loginfo("gps_signal fkt called")                                            #Aufruffunktion des Publishers
    if msg.latitude != 0 and  msg.longitude != 0 and msg.altitude != 0:
        #status.GPS.data = 1                                                 # Wert der Message wird festgelegt
        global mode 
        rospy.loginfo("Mode " + mode) 
        if(mode == 0): mode = 10
        elif(mode == 1): mode = 11
        elif(mode == 2): mode = 12
        pub.publish(mode)

        #pub.publish(status)                                                #der Message wird gepublished
    else:
        #status.GPS.data = 0                                                # Wert der Message wird festgelegt
        if(mode == 0): mode = 20
        elif(mode == 1): mode = 21
        elif(mode == 2): mode = 22
        pub.publish(mode)                                                #der Message wird gepublished
   
def main():                                                               #Aufruffunktion des Suscribers
    rospy.init_node('status_signal', anonymous=True)                        
                                            
    sub = rospy.Subscriber('/twist_mux/cmd_vel', Twist, fahrtrichtung)       #Suscribt zu Rostopic fix
    sub = rospy.Subscriber('/ublox_gps/fix', NavSatFix, gps_signal)       #Suscribt zu Rostopic fix
    rate = rospy.Rate(15)

    rospy.loginfo("Sucessfully started node")  

    while not rospy.is_shutdown():
        rate.sleep()
main()





