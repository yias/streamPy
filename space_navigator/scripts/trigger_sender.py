#!/usr/bin/env python
"""
	developer: Iason Batzianoulis
	maintaner: Iason Batzianoulis
	email: iasonbatz@gmail.com

	description: 
	The ros-node listens to a topic and streams the data through a UDP/IP connection

"""


# import ros-related modules
import rospy
from std_msgs.msg import Int8

# import modules which will be used in the socket communication
import socket
import time
import json
import hashlib
import sys

import numpy as np

# import files for parsing arguments
import argparse
import random
import string

class msg_sender():
	"""

		The class msg_sender handles all the communication with ROS and the UDP/IP communication

	"""

	def __init__(self, IPaddress='localhost', port=9134, nodeName = 'trigger_sender', topicName = 'trigger_debug'):

		# initialize ros node
		self.node_name = nodeName
		rospy.init_node(self.node_name, anonymous=True)

		# initialize subscriber
		self.topic_Name = topicName
		self.listener = rospy.Subscriber(self.topic_Name, Int8, self.rosMsgCallback)

		# Create a UDP/IP socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

		# Bind the socket to the port
		self.server_address = (IPaddress, port)

		# define a header size of the message
		self.HEADERSIZE=8

		# new message identifier
		self.msg_idf="!&?5"

		# end-of-message identifier
		self.endMSG="!3tt"

		# end-connection identifier
		self.ec_id="\ne@c"

		self.is_connected=False

		self.msg_new=False

		self.rate = rospy.Rate(100)

		# initialize hash key encryption
		self.dcdr=hashlib.md5()


	def rosMsgCallback(self, rosmsg):
		# callback function for listening to the topic and store the data to local variables
		
		self.msg_info = rosmsg.data;
		print(self.msg_info)
		self.time_received = rospy.get_rostime()

		if(self.is_connected):
			self.send_msg(rosmsg.data)

	def send_msg(self, msgT):

		# create a json object and serialized it		
		data=json.dumps({"trigger": msgT, "time": [self.time_received.secs, self.time_received.nsecs]})

		self.sock.sendall(self.msg_idf.encode('utf-8')+(data).encode('utf-8')+self.endMSG.encode('utf-8'))


	def close_communication(self):

		print('closing socket')
		self.sock.sendall(self.ec_id.encode('utf-8'))
		self.sock.close()
		print('connection terminated')

	def run(self):

		print('starting up a TCP/IP connection on %s port %s' % self.server_address)

		self.sock.connect(self.server_address)

		# check communication robustness with a hand-shake protocol
		self.is_connected=self.handShake(self.sock,10)

		counter=0;
		while not rospy.is_shutdown():
			rospy.spin()


	def randomString(self, strlength=10):
		letters=string.ascii_lowercase
		return ''.join(random.choice(letters) for i in range(strlength))


	def handShake(self, ssock, strlength):

		ping_times=np.empty([10,1], dtype=np.float64)
		endMSG="!3tt"
		HEADERSIZE=4

		# echo server 10 times
		for i in range(10):
			dcdr=hashlib.md5()
			test_msg=self.randomString(strlength)
			dcdr.update(test_msg)
			# print(test_msg)
			
			chSum=dcdr.hexdigest()
			msg_len=('{:<'+str(HEADERSIZE)+'}').format(str(sys.getsizeof(test_msg)))
			ssock.sendall(msg_len.encode('utf-8')+(test_msg).encode('utf-8')+chSum.encode('utf-8')+endMSG.encode('utf-8'))
			t_time=time.time()
			msg_full=''
			while(True):
				dataT=ssock.recv(4)
				msg_full+=dataT
				if msg_full[-4:].decode('utf-8')==endMSG:
					break
			# print(msg_full.decode('utf-8'))


		return True


if __name__ == '__main__':

	__version__= '0.6.1'

	parser = argparse.ArgumentParser(description='UDP server for receiving trigers from the remote robot')

	parser.add_argument('--host', type=str, help= 'the IP of the server', default='localhost')

	parser.add_argument('--port', type=int, help= 'the port on which the server is listening', default=9135)

	parser.add_argument('--nodeName', type=str, help= 'the name of the ros node', default='udp_sender')

	parser.add_argument('--topicName', type=str, help= 'the name of the ros-topic to listen', default='trigger_debug')

	parser.add_argument('--version', '-V', help='show program version', action='store_true')

	args=parser.parse_args()

	if args.version:
		print('program verions: %s' % __version__)
		exit()

	msgHandler = msg_sender(IPaddress = args.host , port = args.port, nodeName = args.nodeName, topicName = args.topicName)

	msgHandler.run()

	msgHandler.close_communication()
