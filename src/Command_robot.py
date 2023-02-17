#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from master_krai.msg import PS4_cmd


class PS4_command():		
	prev_joy_msg = Joy() 
	joy_msg = Joy()
	cmd = PS4_cmd()
	#joy_msg.axes = {0}

	def __init__(self):
		self.joy_sub = rospy.Subscriber('/joy', Joy, self.JoyCB)							
		self.stick_pub = rospy.Publisher('/Gajah/Command', PS4_cmd, queue_size = 100)

	def JoyCB(self, msg):
		if msg.buttons != self.prev_joy_msg.buttons:
			self.prev_joy_msg = msg
			if msg.buttons[15]:	self.cmd.kotak = not self.cmd.kotak					
			
			if msg.buttons[14] == 1: self.cmd.silang = not self.cmd.silang		

			if msg.buttons[13] == 1: self.cmd.bulat = not self.cmd.bulat		

			if msg.buttons[12] == 1: self.cmd.segitiga = not self.cmd.segitiga

		if msg.buttons[4] == 1: self.cmd.atas = True
		elif msg.buttons[4] == 0: self.cmd.atas = False

		if msg.buttons[5] == 1: self.cmd.kanan = True
		elif msg.buttons[5] == 0: self.cmd.kanan = False

		if msg.buttons[6] == 1: self.cmd.bawah = True
		elif msg.buttons[6] == 0: self.cmd.bawah = False
		
		if msg.buttons[7] == 1: self.cmd.kiri = True
		elif msg.buttons[7] == 0: self.cmd.kiri = False

		if msg.buttons[11] == 1: self.cmd.R1 = True
		elif msg.buttons[11] == 0: self.cmd.R1 = False

		if msg.buttons[10] == 1: self.cmd.L1 = True
		elif msg.buttons[10] == 0: self.cmd.L1 = False
				
		if msg.buttons[0] == 1: self.cmd.share = True				
		elif msg.buttons[0] == 0: self.cmd.share = False

		if msg.buttons[3] == 1: self.cmd.options = True
		elif msg.buttons[3] == 0: self.cmd.options = False

		if msg.buttons[8] == 1: self.cmd.L2 = True
		elif msg.buttons[8] == 0: self.cmd.L2 = False

		if msg.buttons[9] == 1: self.cmd.R2 = True
		elif msg.buttons[9] == 0: self.cmd.R2 = False

		if msg.buttons[2] == 1: self.cmd.R3_B = True
		elif msg.buttons[2] == 0: self.cmd.R3_B = False
		
		if msg.buttons[1] == 1: self.cmd.L3_B = True
		elif msg.buttons[1] == 0: self.cmd.L3_B = False

		self.cmd.LX = msg.axes[0] * -1000.0
		self.cmd.LY = msg.axes[1] * 1000.0

		self.cmd.RX = msg.axes[2] * -100.0
		self.cmd.RY = msg.axes[3] * 100.0

		self.cmd.L2A = msg.axes[12] * 1000.0
		self.cmd.R2A = msg.axes[13] * 1000.0

	def publisher_PS4(self):
		self.stick_pub.publish(self.cmd)


if __name__ == '__main__':
	try:
		rospy.init_node('Gajah', anonymous = True)
		rospy.loginfo("Gajah Node is Running...")
		Stick_PS4 = PS4_command()
		rate = rospy.Rate(10)

		while not rospy.is_shutdown():	
			Stick_PS4.publisher_PS4()

			rate.sleep()
	except rospy.ROSInterruptException:
		print("Gajah program interrupted before completion")