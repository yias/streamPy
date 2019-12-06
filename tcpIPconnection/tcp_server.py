# import modules 

import numpy as np

# import modules for socket programming (TCP/IP connection)
import socket
import sys
import json

# import modules for theading programming
import threading
import time



def main():

	connection_counter=0

	write = sys.stdout.write
	keep_running=True

	# Create a TCP/IP socket
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

	# Bind the socket to the port
	server_address = ('localhost', 10353)

	print('starting up on %s port %s' % server_address)

	sock.bind(server_address)

	# initialize client and adresses lists
	connections_list=list() #np.empty([1,])
	clients_address=list() #np.empty([1,])
	
	# define a header size of the message
	HEADERSIZE=10

	# define message identifier
	msg_idf="!&?5"

	# listen for new connections
	sock.settimeout(3)
	sock.listen(6)
	

	#Wait for a connection
	connection_exist=False
	print('waiting for a connections ... ')

	while(True):
		
		try:
			connection, client_address = sock.accept()
			connection_exist=True;
			print('connection from ', client_address)
			while(True):
				data=connection.recv(16)
				print('received: %s' %data)
				if(sock.fileno()==-1):
					break;
		except KeyboardInterrupt:
			if connection_exist:
				connection.close()
			break
		finally:
			print('.')
	
	sock.close()
	print('all connections killed')


		


if __name__ == '__main__':
	main()