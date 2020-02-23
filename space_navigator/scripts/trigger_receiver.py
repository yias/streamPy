#!/usr/bin/env python
"""
	developer: Iason Batzianoulis
	maintaner: Iason Batzianoulis
	email: iasonbatz@gmail.com

	description: 
	The script receive messages trhough a UDP/IP connection and saves the data to a txt file

"""

# import modules which will be used in the socket communication
import socket
import time
from datetime import datetime
import json

class msg_receiver():
	"""

		The class msg_receiver handles all the communication of the UDP/IP communication

	"""

	def __init__(self, IPaddress='localhost', port=9134, bufferSize = 4096, fileName = 'logFile'):

		# Create a UDP/IP socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.server_address = (IPaddress, port)  #Fixed IP of PC

		self.buffer_size=bufferSize

		nowT=datetime.now()

		# open a txt file for storing the message
		self.logfile = open("logfiles/" + fileName + "_" + nowT.strftime("%y%m%d_%H%M%S") + ".txt", "w")

		self.start_Time = time.time()

		# bind the socket to a specific port of an IP address
		self.sock.bind(self.server_address)
		print('------------- waiting to receive message -------------')

	def close_communication(self):
		self.sock.close()
		print('connection terminated')

	def run(self):

		while (True):
			try:
				# receive data of a buffer size
				data, address = self.sock.recvfrom(self.buffer_size)

				# retrieve message information 
				msg_info = json.loads(data.decode('utf-8'))

				tgr = msg_info.get("trigger")
				tgr_time = msg_info.get("time")

				now = time.time() - self.start_Time

				print('%f %d %d.%d' % (now, tgr, tgr_time[0], tgr_time[1]))

				self.logfile.write('%f %d %d.%d\n' % (now, tgr, tgr_time[0], tgr_time[1]))

				# print('trigger: %s' %(msg_info.get("trigger")))
				# print('trigger time: %s' %(msg_info.get("time")))
			
			except KeyboardInterrupt:
				# with Cntl+C, close any socket communication and lodfiles
				self.logfile.close()
				self.close_communication()
				break


if __name__ == '__main__':

	msgHandler = msg_receiver()

	msgHandler.run()

	# msgHandler.close_communication()