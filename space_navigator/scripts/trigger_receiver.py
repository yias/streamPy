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
import hashlib
import sys

# import modules for creating a directory
import os
import errno

# import files for parsing arguments
import argparse
import random
import string

import numpy as np


class msg_receiver():
	"""

		The class msg_receiver handles all the communication of the UDP/IP communication

	"""

	def __init__(self, IPaddress='localhost', port=9135, bufferSize = 4, fileName = 'logFile', directoryName = 'logfiles'):

		# Create a UDP/IP socket
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.server_address = (IPaddress, port)  #Fixed IP of PC

		self.buffer_size=bufferSize

		# set currect date and time for the name of the logfile
		nowT=datetime.now()

		# define a header size of the message
		self.HEADERSIZE=8

		# define message identifier
		self.msg_idf="!&?5"

		# end-of-message identifier
		self.endMSG="!3tt"

		# end-connection identifier
		self.ec_id="\ne@c"

		

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

		# listen for new connections
		self.sock.listen(6)
		connection_exist=False

		while (True):
			try:
				connection, client_address = self.sock.accept()
				connection_exist=True;

				conCheck=self.handShake(connection,10)
				while(conCheck):

					# receive data of a buffer size
					print(self.buffer_size)
					data, address = connection.recv(self.buffer_size)

					if data.decode('utf-8')==self.msg_idf:
						# receive bytes until the full message is received
						full_msg='';
						while (True):
							dataT=connection.recv(1)
							full_msg+=dataT
							if full_msg[-4:].decode('utf-8')==self.endMSG:
								break

						# extract message
						msg_validity, tr_msg = self.msgExtractor(full_msg,self.HEADERSIZE,self.endMSG)

						if msg_validity:

							# retrieve message information 
							msg_info = json.loads(tr_msg.decode('utf-8'))

							tgr = msg_info.get("trigger")
							tgr_time = msg_info.get("time")

							now = time.time() - self.start_Time

							# print message data with timestamp
							print('%f %d %d.%d' % (now, tgr, tgr_time[0], tgr_time[1]))

							# store message data to the log file, including timestamp
							self.logfile.write('%f %d %d.%d\n' % (now, tgr, tgr_time[0], tgr_time[1]))

					if  data.decode('utf-8')==self.ec_id:
						# if end-of-communication identifier received, terminate the connection
						print('Connection terminated by client ', client_address)
						connection.close()
						break

			
			except KeyboardInterrupt:
				# with Cntl+C, close any socket communication and lodfiles
				self.logfile.close()
				if(connection_exist):
					connection.close()
				self.close_communication()
				break

	# def calc_checksum(s):
	# return '%2X' % (-(sum(ord(c) for c in s) % 256) & 0xFF)

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
	# 


	def randomString(self, strlength=10):
		letters=string.ascii_lowercase
		return ''.join(random.choice(letters) for i in range(strlength))


	def handShake(self, conn, strlength):

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
				dataT=conn.recv(4)
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
			conn.sendall(msg_len.encode('utf-8')+(test_msg).encode('utf-8')+chSum.encode('utf-8')+endMSG.encode('utf-8'))
			t_time0=time.time()

		if((1*(validity_counter)).mean()>0.8):
			print('valid communication established')
			print('compute times: %s %s %s s' %(compute_times.mean(), u'\u00b1', compute_times.std()))
			print('ping times:  %s %s %s s' %(ping_times[:9].mean(), u'\u00b1', ping_times[:9].std()))
			return True
		else:
			print('communication is not valid')
			return False


if __name__ == '__main__':

	__version__= '0.6.1'

	parser = argparse.ArgumentParser(description='UDP server for receiving trigers from the remote robot')

	parser.add_argument('--host', type=str, help= 'the IP of the server', default='localhost')

	parser.add_argument('--port', type=int, help= 'the port on which the server is listening', default=9135)

	parser.add_argument('--bufferSize', type=int, help= 'the buffer size of the message', default=4)

	parser.add_argument('--fileName', type=str, help= 'the name of the file to store the logs', default='logFile')

	parser.add_argument('--directoryName', type=str, help= 'the name of the directory to save the logfile', default='logfiles')

	parser.add_argument('--version', '-V', help='show program version', action='store_true')

	args=parser.parse_args()

	if args.version:
		print('program verions: %s' % __version__)
		exit()

	msgHandler = msg_receiver(IPaddress = args.host, port = args.port, bufferSize = args.bufferSize, fileName = args.fileName, directoryName = args.directoryName)

	msgHandler.run()