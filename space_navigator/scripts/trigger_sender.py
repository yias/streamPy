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

# import files for parsing arguments
import argparse

class msg_sender():
	"""

		The class msg_sender handles all the communication with ROS and the UDP/IP communication

	"""

	def __init__(self, IPaddress='localhost', port=9134, nodeName = 'udp_sender', topicName = 'trigger_debug'):

		# initialize ros node
		self.node_name = nodeName
		rospy.init_node(self.node_name, anonymous=True)

		# initialize subscriber
		self.topic_Name = topicName
		self.listener = rospy.Subscriber(self.topic_Name, Int8, self.rosMsgCallback)

		# Create a UDP/IP socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

		# Bind the socket to the port
		self.server_address = (IPaddress, port)

		print('starting up a UDP/IP connection on %s port %s' % self.server_address)

		# self.sock.bind(self.server_address)

	def rosMsgCallback(self, rosmsg):
		# callback function for listening to the topic and store the data to local variables
		self.msg_info = rosmsg.data;
		print(self.msg_info)
		self.time_received = rospy.get_rostime()
		self.send_msg()

	def send_msg(self):
		
		# create a json object and serialized it		
		data=json.dumps({"trigger": self.msg_info, "time": [self.time_received.secs, self.time_received.nsecs]})

		self.sock.sendto(data.encode('utf-8'), self.server_address)

	def close_communication(self):
		self.sock.close()
		print('connection terminated')

	def run(self):

		while not rospy.is_shutdown():
			rospy.spin()


if __name__ == '__main__':

	__version__= '0.6.1'

	parser = argparse.ArgumentParser(description='UDP server for receiving trigers from the remote robot')

	parser.add_argument('--host', type=str, help= 'the IP of the server', default='localhost')

	parser.add_argument('--port', type=int, help= 'the port on which the server is listening', default=9134)

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
