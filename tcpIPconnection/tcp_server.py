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
	server_address = ('localhost', 10351)

	print('starting up on %s port %s' % server_address)

	sock.bind(server_address)

	# initialize client and adresses lists
	connections_list=list() #np.empty([1,])
	clients_address=list() #np.empty([1,])

	# define buffer size
	BUFFER_SIZE=4
	
	# define a header size of the message
	HEADERSIZE=8

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
				data=connection.recv(BUFFER_SIZE)
				# print(type(data))
				if data.decode('utf-8')==msg_idf:
					print('new msg rcvd')
					data=connection.recv(HEADERSIZE)
					msg_len=int(data.decode('utf-8'))
					print('expecting nb bytes: ', msg_len)
					data=connection.recv(HEADERSIZE)
					msg_id=int(data.decode('utf-8'))
					print('msg id: ',msg_id)
					
					full_msg='';
					while sys.getsizeof(full_msg)<msg_len:
						data=connection.recv(BUFFER_SIZE)
						full_msg=full_msg+data
					msg_data=json.loads(full_msg.decode('utf-8'))
					print('%s: %s' %(msg_data.get("a"),msg_data.get("b")))

				if(sock.fileno()==-1):
					break
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