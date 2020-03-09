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

		# Create a TDP/IP socket
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

		# bind the socket to a specific port of an IP address
		self.sock.bind(self.server_address)
		print('------------- waiting to receive message -------------')


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

		# listen for new connections
		self.sock.listen(6)
		connection_exist=False

		# print('starting up a TCP/IP connection on %s port %s' % self.server_address)

		# self.sock.connect(self.server_address)

		connection_exist = False

		while(True):
			try:
				connection, client_address = self.sock.accept()

				connection_exist=True;

				# conCheck=self.handShake(connection,10)
				if(connection_exist):
					break
			except KeyboardInterrupt:
				if(connection_exist):
					connection.close()
				self.connection_exist=False
				break
			finally:
				print('Waiting for new clients ....')

		# check communication robustness with a hand-shake protocol
		# self.is_connected=self.handShake(self.sock,10)
		print("connection establised")
		counter=0;
		if(connection_exist):
			while not rospy.is_shutdown():
				rospy.spin()


	def randomString(self, strlength=10):
		letters=string.ascii_lowercase
		return ''.join(random.choice(letters) for i in range(strlength))


	def handShake(self, ssock, strlength):

		# ping_times=np.empty([10,1], dtype=np.float64)
		# endMSG="!3tt"
		# HEADERSIZE=4

		# # echo server 10 times
		# for i in range(10):
		# 	dcdr=hashlib.md5()
		# 	test_msg=self.randomString(strlength)
		# 	dcdr.update(test_msg)
		# 	# print(test_msg)
			
		# 	chSum=dcdr.hexdigest()
		# 	msg_len=('{:<'+str(HEADERSIZE)+'}').format(str(sys.getsizeof(test_msg)))
		# 	ssock.sendall(msg_len.encode('utf-8')+(test_msg).encode('utf-8')+chSum.encode('utf-8')+endMSG.encode('utf-8'))
		# 	t_time=time.time()
		# 	msg_full=''
		# 	while(True):
		# 		dataT=ssock.recv(4)
		# 		msg_full+=dataT
		# 		if msg_full[-4:].decode('utf-8')==endMSG:
		# 			break
		# 	# print(msg_full.decode('utf-8'))
		ping_times=np.empty([10,1], dtype=np.float64)
		compute_times=np.empty([10,1], dtype=np.float64)
		validity_counter=np.empty([10,1], dtype=bool)
		endMSG="!3tt"
		HEADERSIZE=4
		t_time0=0.0

		# receive and send random msgs to the client 10 times
		for i in range(10):
			msg_full=''
			while (True):
				dataT=ssock.recv(4)
				msg_full+=dataT
				if msg_full[-4:].decode('utf-8')==endMSG:
					break

			if t_time0!=0.0:
				ping_times[i-1]=time.time()-t_time0
			t_time=time.time()
			msg_validity, tr_msg = self.msgExtractor(msg_full,HEADERSIZE,endMSG)
			validity_counter[i]=msg_validity

			dcdr=hashlib.md5()
			test_msg=self.randomString(strlength)
			dcdr.update(test_msg)
			chSum=dcdr.hexdigest()
			msg_len=('{:<'+str(HEADERSIZE)+'}').format(str(sys.getsizeof(test_msg)))
			compute_times[i]=time.time()-t_time
			ssock.sendall(msg_len.encode('utf-8')+(test_msg).encode('utf-8')+chSum.encode('utf-8')+endMSG.encode('utf-8'))
			t_time0=time.time()

		if((1*(validity_counter)).mean()>0.8):
			print('valid communication established')
			print('compute times: %s %s %s s' %(compute_times.mean(), u'\u00b1', compute_times.std()))
			print('ping times:  %s %s %s s' %(ping_times[:9].mean(), u'\u00b1', ping_times[:9].std()))
			return True
		else:
			print('communication is not valid')
			return False

		return True

	def msgExtractor(self, msg, hdrSize, endMsgID):
		msgSize=int(msg[:hdrSize].decode('utf-8'))

		tmp_msg=msg[hdrSize:hdrSize+msgSize-sys.getsizeof('')]

		hashcode=msg[hdrSize+msgSize-sys.getsizeof(''):-len(endMsgID)]

		hc_check=hashlib.md5()
		hc_check.update(tmp_msg)
		if hc_check.hexdigest()==hashcode:
			return True, tmp_msg
		else:
			return False, ''


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
