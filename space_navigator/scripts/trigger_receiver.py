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

# import modules for creating a directory
import os
import errno

# import files for parsing arguments
import argparse

class msg_receiver():
	"""

		The class msg_receiver handles all the communication of the UDP/IP communication

	"""

	def __init__(self, IPaddress='localhost', port=9134, bufferSize = 4096, fileName = 'logFile', directoryName = 'logfiles'):

		# Create a UDP/IP socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.server_address = (IPaddress, port)  #Fixed IP of PC

		self.buffer_size=bufferSize

		# set currect date and time for the name of the logfile
		nowT=datetime.now()

		# create a folder to store the logfiles if doesn't exists
		try:
			os.makedirs(directoryName)
		except OSError as e:
			if e.errno != errno.EEXIST:
				raise

		# open a txt file for storing the message
		self.logfile = open(directoryName + "/" + fileName + "_" + nowT.strftime("%y%m%d_%H%M%S") + ".txt", "w")

		# set program starting time
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

				# print message data with timestamp
				print('%f %d %d.%d' % (now, tgr, tgr_time[0], tgr_time[1]))

				# store message data to the log file, including timestamp
				self.logfile.write('%f %d %d.%d\n' % (now, tgr, tgr_time[0], tgr_time[1]))
			
			except KeyboardInterrupt:
				# with Cntl+C, close any socket communication and lodfiles
				self.logfile.close()
				self.close_communication()
				break


if __name__ == '__main__':

	__version__= '0.6.1'

	parser = argparse.ArgumentParser(description='UDP server for receiving trigers from the remote robot')

	parser.add_argument('--host', type=str, help= 'the IP of the server', default='localhost')

	parser.add_argument('--port', type=int, help= 'the port on which the server is listening', default=9134)

	parser.add_argument('--bufferSize', type=int, help= 'the buffer size of the message', default=4096)

	parser.add_argument('--fileName', type=str, help= 'the name of the file to store the logs', default='logFile')

	parser.add_argument('--directoryName', type=str, help= 'the name of the directory to save the logfile', default='logfiles')

	parser.add_argument('--version', '-V', help='show program version', action='store_true')

	args=parser.parse_args()

	if args.version:
		print('program verions: %s' % __version__)
		exit()

	msgHandler = msg_receiver(IPaddress = args.host, port = args.port, bufferSize = args.bufferSize, fileName = args.fileName, directoryName = args.directoryName)

	msgHandler.run()