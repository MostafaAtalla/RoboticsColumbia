#!/usr/bin/env python  
import rospy

from std_msgs.msg import Int16
from project1_solution.msg import TwoInts



class Solution(object):
    def __init__(self,pub):
        self.x=0
        self.y=0
        self.sum=0
        self.pub=pub

    def callback(self,msg):

        self.x=msg.a
        self.y=msg.b
        self.sum=self.x+self.y
        self.publisher()
        
    def publisher(self):
        self.pub.publish(self.sum)
    


def main():
    while not rospy.is_shutdown():
        rospy.init_node('solution')
        pub=rospy.Publisher('sum',Int16,queue_size=10)
        solution_in=Solution(pub)
        rospy.Subscriber('two_ints',TwoInts,solution_in.callback)
        


        rospy.spin()

if __name__== '__main__':
    main()
