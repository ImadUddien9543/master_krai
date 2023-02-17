#!/usr/bin/env python

import rospy
from master_krai.msg import PS4_cmd
from master_krai.msg import Wheel
import numpy as np
from scipy.special import expit as ep
import math


class Kinematik():
	R_robot = 1.0
	matrix_A = np.zeros(shape=(4, 3))
	matrix_B = np.zeros(shape=(3, 1))
	matrix_C = np.zeros(shape=(3, 3))
	matrix_D = np.zeros(shape=(3, 1))	
	result_matrix = np.zeros(shape=(4, 1))	
	matrix_pwm = np.arange(0, 1000, 0.01)
	
	wheel_value = Wheel()
	joy_val = PS4_cmd()		
	theta : float = 0.0
	theta_dot : float = 0.0
	vel : float = 0.0
	vel_x : float = 0.0
	vel_y : float = 0.0
	s_quintic = np.poly1d([.0246, .0585, .06550, .0445, .042, .042])
	v_quintic = np.polyder(s_quintic)
	omni_in = [-5000, 5000]
	omni_pwm = [-1000, 1000]
	avel_l : float = 0.0
	avel_r : float = 0.0
	nos : float = 0
	nos_plus : float = 0
	nos_minus : float = 0
	decay : float = 0.5
	bldc_cal = [0, 7920]
	bldc_pwm = [1800, 7200]
	bldc_in: float = 0
	pwm_esc_in : int = 0
	vertical : float = 0
	horizontal : float = 0
	chain_lift : float = 0

	def __init__(self, data):
		self.Kinematik_sub = rospy.Subscriber('/Gajah/Command', PS4_cmd, self.Command_CB)
		self.Result_kinematik_pub = rospy.Publisher('/Gajah/Four_OWD_Kinematics', Wheel, queue_size = 100)
		self.Command_CB(data)

	@classmethod
	def Command_CB(cls, data):
		cls.msg = data
		cls.theta = math.degrees(math.atan2(cls.msg.LY, cls.msg.LX))
		if cls.theta < 0.0 : cls.theta += 360.0
		
		cls.avel_r = cls.msg.R2A
		cls.avel_l = cls.msg.L2A

		if cls.msg.R2 == True : cls.theta_dot = cls.avel_r		
		if cls.msg.L2 == True : cls.theta_dot = cls.avel_l
		elif cls.msg.L2 or cls.msg.R2 == False: cls.theta_dot = 0
		
		cls.vel_x = cls.msg.LX
		cls.vel_y = cls.msg.LY

		if cls.msg.R3_B == True:			
			cls.nos_plus += 1
			cls.nos_minus = 0.0
			cls.nos = np.clip(cls.v_quintic(cls.nos_plus), 0, 5)
			
		elif cls.msg.R3_B == False:
			cls.nos_plus = 0.0
			cls.nos_minus += 0.75
			cls.nos = np.clip(cls.nos * math.pow(1.0 - cls.decay, cls.nos_minus), 1, 10)

		#bldc control
		if cls.msg.share == True: cls.bldc_in += 100								
		if cls.msg.options == True: cls.bldc_in -= 100			
		#shooter control
		if cls.msg.atas == True: 
			cls.vertical_plus += 0.5
			cls.vertical = 7000 * math.exp(-cls.vertical_plus)
		if cls.msg.bawah == True: 
			cls.vertical_minus += 0.5
			cls.vertical = 7000 * math.exp(-cls.vertical_minus)
		if cls.msg.kanan == True: 
			cls.horizontal_plus += 0.5
			cls.horizontal = 7000 * math.exp(-cls.horizontal_plus)
		if cls.msg.kiri == True: 
			cls.horizontal_minus += 0.5
			cls.horizontal = 7000 * math.exp(-cls.horizontal_minus)
		#chain lift control
		if cls.msg.R1 == True :
			cls.chain_lift_plus += 0.1			
			cls.chain_lift = 700 * math.exp(-cls.chain_lift_plus)
		if cls.msg.L1 == True :
			cls.chain_lift_minus += 0.1
			cls.chain_lift = -(700 * math.exp(-cls.chain_lift_minus))
		elif cls.msg.kiri == False and cls.msg.kanan == False and cls.msg.atas == False and cls.msg.bawah == False and cls.msg.R1 == False and cls.msg.L1 == False:
			cls.horizontal = 0
			cls.vertical = 0
			cls.chain_lift = 0
			cls.chain_lift_minus = 0
			cls.chain_lift_plus = 0
			cls.horizontal_plus = 0
			cls.horizontal_minus = 0
			cls.vertical_plus = 0
			cls.vertical_minus = 0

		cls.vertical = math.ceil(np.clip(cls.vertical, -7000, 7000))
		cls.horizontal = math.ceil(np.clip(cls.horizontal, -7000, 7000))
		cls.chain_lift = math.ceil(np.clip(cls.chain_lift, -500, 500))
		cls.bldc_in = np.clip(cls.bldc_in, cls.bldc_cal[0], cls.bldc_cal[1])

	def Calculate(self):
		#inverse kinematics
		self.wheel_value.sudut = self.theta
		self.wheel_value.ang_vel = self.theta_dot
		self.wheel_value.lin_vel = self.nos
		
		self.matrix_A[0, 0] = -math.sin(math.radians(45.0  + self.wheel_value.sudut))
		self.matrix_A[1, 0] = -math.sin(math.radians(135.0 + self.wheel_value.sudut))
		self.matrix_A[2, 0] = -math.sin(math.radians(225.0 + self.wheel_value.sudut))
		self.matrix_A[3, 0] = -math.sin(math.radians(315.0 + self.wheel_value.sudut))

		self.matrix_A[0, 1] = math.cos(math.radians(45.0  + self.wheel_value.sudut))
		self.matrix_A[1, 1] = math.cos(math.radians(135.0 + self.wheel_value.sudut))
		self.matrix_A[2, 1] = math.cos(math.radians(225.0 + self.wheel_value.sudut))
		self.matrix_A[3, 1] = math.cos(math.radians(315.0 + self.wheel_value.sudut))

		self.matrix_A[0, 2] = self.matrix_A[1, 2] = self.matrix_A[2, 2] = self.matrix_A[3, 2] = self.R_robot
		
		self.matrix_C[0, 0] = math.cos(math.radians(self.wheel_value.sudut))
		self.matrix_C[1, 0] = math.sin(math.radians(self.wheel_value.sudut))		
	
		self.matrix_C[0, 1] = -math.sin(math.radians(self.wheel_value.sudut))
		self.matrix_C[1, 1] = math.cos(math.radians(self.wheel_value.sudut))

		self.matrix_C[0, 2] = self.matrix_C[1, 2] = self.matrix_C[2, 0] = self.matrix_C[2, 1] = 0
		self.matrix_C[2, 2] = 1

		self.matrix_B[0] = self.vel_x 
		self.matrix_B[1] = self.vel_y 
		self.matrix_B[2] = self.wheel_value.ang_vel

		self.result_matrix = np.matmul(self.matrix_A, np.matmul(self.matrix_C, self.matrix_B))	
		self.result_matrix = np.interp(np.clip(np.dot(self.result_matrix, self.nos), -5000, 5000), self.omni_in, self.omni_pwm)
		self.wheel_value.R_Front = math.ceil(self.result_matrix[0]) 
		self.wheel_value.L_Front = math.ceil(self.result_matrix[1]) 
		self.wheel_value.L_Back = math.ceil(self.result_matrix[2]) 
		self.wheel_value.R_Back = math.ceil(self.result_matrix[3]) 

		#roller speed command
		self.pwm_esc_in = math.ceil(np.interp(self.bldc_in, self.bldc_cal, self.bldc_pwm))
		self.wheel_value.roller_spd_1 = self.pwm_esc_in
		self.wheel_value.roller_spd_2 = self.pwm_esc_in
		self.wheel_value.x_penembak = self.horizontal
		self.wheel_value.y_penembak = self.vertical
		self.wheel_value.lift_gripper = self.chain_lift
		self.Result_kinematik_pub.publish(self.wheel_value)

	
if __name__ == '__main__':
	try:
		rospy.init_node('Kinematik', anonymous = True)
		rospy.loginfo("Kinematik Node is Running...")
		Kinematik_Gajah = Kinematik(Kinematik.joy_val)
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():	
			Kinematik_Gajah.Calculate()
			#rospy.loginfo(Kinematik_Gajah.chain_lift)			
			rate.sleep()
	except rospy.ROSInterruptException:
		print("Kinematik program interrupted before completion")