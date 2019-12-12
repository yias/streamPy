# import standard modules 
import numpy as np
import argparse

# import modules for socket programming (TCP/IP connection)
import socket
import sys
import json

# import modules for theading programming
import threading
import time


def main(args):

	connection_counter=0

	write = sys.stdout.write
	keep_running=True

	# Create a TCP/IP socket
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

	# Bind the socket to the port
	# server_address = ('localhost', 10351)
	server_address = (args.host, args.port)

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

	# end-connection identifier
	ec_id="\ne@c"

	# listen for new connections
	# sock.settimeout(3)
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
					# print('expecting nb bytes: ', msg_len)
					data=connection.recv(HEADERSIZE)
					msg_id=int(data.decode('utf-8'))
					print('msg id: ',msg_id)
					
					full_msg='';
					while sys.getsizeof(full_msg)<msg_len:
						data=connection.recv(BUFFER_SIZE)
						full_msg=full_msg+data
					msg_data=json.loads(full_msg.decode('utf-8'))
					print('%s: %s' %(msg_data.get("a"),msg_data.get("b")))
				if  data.decode('utf-8')==ec_id:
					print('Connection terminated by client ', client_address)
					connection.close()
					break
				# if(sock.fileno()==-1):
				# 	break
		except KeyboardInterrupt:
			if connection_exist:
				connection.close()
			break
		finally:
			print('Waiting for new clients ....')
	
	sock.close()
	print('all connections killed')


		


if __name__ == '__main__':
	__version__='0.1.3'

	parser = argparse.ArgumentParser(description='TCP server for receiving inputs from 3D mouse client')

	parser.add_argument('--host', type=str, help= 'the IP of the server', default='localhost')

	parser.add_argument('--port', type=int, help= 'the port on which the server is listening', default=10351)

	parser.add_argument('--roosnode','--rn', type=bool, help= 'if needed to publish the data to a ros topic', default=False)

	parser.add_argument('--version', '-V', help='show program version', action='store_true')

	args=parser.parse_args()

	if args.version:
		print('program verions: %s' % __version__)
		exit()
	main(args)	