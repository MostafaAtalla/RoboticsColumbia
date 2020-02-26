#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts

x=0
y=0
suma=0

def callback(msg):
    global x,y,suma
    x=msg.a
    y=msg.b
    suma=x+y
    


def main():
    global x,y,suma
    rospy.init_node('solution')
    rospy.Subscriber('two_ints',TwoInts,callback)
    pub=rospy.Publisher('sum',Int16,queue_size=1)
    pub.publish(suma)
    rospy.loginfo('{}'.format(suma))
    rospy.spin()

if __name__== '__main__':
    main()
   