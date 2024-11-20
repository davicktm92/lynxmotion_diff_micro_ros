#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.parameter import Parameter

rosRate=10

class teleop_cdpr(Node):
	def __init__(self):
		super().__init__("teleop_pcdpr")
		
		#Parameters
		self.declare_parameters(
            namespace='',
            parameters=[
				('axis_linear.x',Parameter.Type.INTEGER),
				('scale_linear.x',Parameter.Type.DOUBLE),
				('scale_linear_turbo.x',Parameter.Type.DOUBLE),
				('axis_angular.yaw',Parameter.Type.INTEGER),
				('scale_angular.yaw',Parameter.Type.DOUBLE),
				('enable_button',Parameter.Type.INTEGER),
				('enable_turbo_button',Parameter.Type.INTEGER),
            ])

		#Subscriptors and publishers declarations
		self.create_subscription(Joy,"joy",self.callback_joy,1)
		self.m_pub=self.create_publisher(Twist,"/cmd_vel",1)

		self.create_timer(1/rosRate,self.motor_pub)

		self.m_speed=Twist()

		self.m_pub.publish(self.m_speed)
		self.mode=None

		#Parameters read
		self.axis_linear=self.get_parameter("axis_linear.x").get_parameter_value().integer_value
		self.scale_linear=self.get_parameter("scale_linear.x").get_parameter_value().double_value
		self.scale_linear_turbo=self.get_parameter("scale_linear_turbo.x").get_parameter_value().double_value
		self.axis_angular=self.get_parameter("axis_angular.yaw").get_parameter_value().integer_value
		self.scale_angular=self.get_parameter("scale_angular.yaw").get_parameter_value().double_value
		self.enable_button=self.get_parameter("enable_button").get_parameter_value().integer_value
		self.enable_turbo_button=self.get_parameter("enable_turbo_button").get_parameter_value().integer_value

	def callback_joy(self,joy_msg):
		self.enable=joy_msg.buttons[self.enable_button]
		self.enable_turbo=joy_msg.buttons[self.enable_turbo_button]

		if self.enable==1 and self.enable_turbo==0:
			self.m_speed.linear.x=self.scale_linear*joy_msg.axes[self.axis_linear]
			self.m_speed.angular.z=self.scale_angular*joy_msg.axes[self.axis_angular]
		elif self.enable==0 and self.enable_turbo==1:
			self.m_speed.linear.x=self.scale_linear_turbo*joy_msg.axes[self.axis_linear]
			self.m_speed.angular.z=self.scale_angular*joy_msg.axes[self.axis_angular]
		else:
			self.m_speed.linear.x=0.0
			self.m_speed.angular.z=0.0

	def motor_pub(self, event=None):
		self.m_pub.publish(self.m_speed)

def main(args=None):
    rclpy.init()
    teleop_diff=teleop_cdpr()
    try:
        rclpy.spin(teleop_diff)
    except KeyboardInterrupt:
        print("Terminating Teleop Node..")
        teleop_diff.destroy_node()

if __name__=='__main__':
    main()