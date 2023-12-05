#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class CartBackend:

    def __init__(self):

        rospy.init_node('cart_backend', anonymous=True)
    
        # Robot
        self.odom_reader()
        self.drive_controller()
        
        # App
        self.app_reader()
        self.app_reporter()
        
    
    # Backend to Robot
        
    def odom_reader(self):
    
        #rospy.init_node('cart_odom_subscriber', anonymous=True)
        rospy.Subscriber("cart_diff_drive_controller/odom", Odometry, self.odom_callback)
        
        
    def odom_callback(self, data):
    
        #rospy.loginfo(rospy.get_caller_id() + "Linear Velocity X: %f", data.twist.twist.linear.x)
        #rospy.loginfo(rospy.get_caller_id() + "Linear Velocity Y: %f", data.twist.twist.linear.y)
        #rospy.loginfo(rospy.get_caller_id() + "Linear Velocity Z: %f", data.twist.twist.linear.z)
        #rospy.loginfo(rospy.get_caller_id() + "Angular Velocity X: %f", data.twist.twist.angular.x)
        #rospy.loginfo(rospy.get_caller_id() + "Angular Velocity Y: %f", data.twist.twist.angular.y)
        #rospy.loginfo(rospy.get_caller_id() + "Angular Velocity Z: %f", data.twist.twist.angular.z)
        #rospy.loginfo(rospy.get_caller_id() + "Position X: %f", data.pose.pose.position.x)
        #rospy.loginfo(rospy.get_caller_id() + "Position Y: %f", data.pose.pose.position.y)
        #rospy.loginfo(rospy.get_caller_id() + "Position Z: %f", data.pose.pose.position.z)
        self.odom_data = data
      
        
    def drive_controller(self):
    
        #rospy.init_node('cart_drive_publisher', anonymous=True)
        self.drive_pub = rospy.Publisher('cart_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.drive_control = Twist();
    	
    	
    # Backend to App
    
    def app_reader(self):
    
        #rospy.init_node('cart_backend_listener', anonymous=True)
        rospy.Subscriber("cart_app2backend", String, self.app_callback)
        
        
    def app_callback(self, data):
    
        self.app_data = data
      
        
    def app_reporter(self):
    
        #rospy.init_node('cart_backend_speaker', anonymous=True)
        self.app_pub = rospy.Publisher('cart_backend2app', String, queue_size=10)
    	
    	
    # Backend Cycle
    
    def cycle(self):
    	
        rate = rospy.Rate(2) # 10hz
        self.drive_control.linear.x = 0.05;
        self.drive_control.angular.z = 0.15;

        while not rospy.is_shutdown():

            # Odometry and Drive Controller
            self.drive_control.linear.x += 0.20;
            self.drive_control.angular.z += 0.60;


            # App communication
            app_msg = "On the way"

        
            # Publish data
            self.drive_pub.publish(self.drive_control)
            self.app_pub.publish(app_msg)
            rate.sleep()
	
		
if __name__ == '__main__':

    cart_bcknd = CartBackend()
    cart_bcknd.cycle()
    

