# import standard modules 
import numpy as np
import argparse
import hashlib
import random
import string

# import modules for socket programming (TCP/IP connection)
import socket
import sys
import json

# import modules for theading programming
import threading
import time


def calc_checksum(s):
	return '%2X' % (-(sum(ord(c) for c in s) % 256) & 0xFF)

def msgExtractor(msg, hdrSize, endMsgID):
	msgSize=int(msg[:hdrSize].decode('utf-8'))
	# print('msg size: ', msgSize)
	tmp_msg=msg[hdrSize:hdrSize+msgSize-sys.getsizeof('')]
	# print(tmp_msg)
	hashcode=msg[hdrSize+msgSize-sys.getsizeof(''):-len(endMsgID)]
	# print(hashcode)
	hc_check=hashlib.md5()
	hc_check.update(tmp_msg)
	if hc_check.hexdigest()==hashcode:
		return True, tmp_msg
	else:
		return False, ''
# 


def randomString(strlength=10):
	letters=string.ascii_lowercase
	return ''.join(random.choice(letters) for i in range(strlength))


def handShake(conn, strlength):

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
		
		# print(msg_full.decode('utf-8'))
		if t_time0!=0.0:
			ping_times[i-1]=time.time()-t_time0
		t_time=time.time()
		msg_validity, tr_msg = msgExtractor(msg_full,HEADERSIZE,endMSG)
		validity_counter[i]=msg_validity

		dcdr=hashlib.md5()
		test_msg=randomString(strlength)
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

	# end-of-message identifier
	endMSG="!3tt"

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
			conCheck=handShake(connection,10)
			while(True):
				data=connection.recv(BUFFER_SIZE)
				# print(data)
				if data.decode('utf-8')==msg_idf:
					# print('new msg rcvd')
					# data=connection.recv(HEADERSIZE)
					# msg_len=int(data.decode('utf-8'))
					# print('expecting nb bytes: ', msg_len)
					# data=connection.recv(HEADERSIZE)
					# msg_id=int(data.decode('utf-8'))
					# print('msg id: ',msg_id)
					
					full_msg='';
					while (True):
						dataT=connection.recv(1)
						# print(dataT)
						full_msg+=dataT
						if full_msg[-4:].decode('utf-8')==endMSG:
							break

					# while sys.getsizeof(full_msg)<msg_len:
					# 	data=connection.recv(BUFFER_SIZE)
					# 	print(data)
					# 	full_msg=full_msg+data
					# print(full_msg)
					msg_validity, tr_msg = msgExtractor(full_msg,HEADERSIZE,endMSG)
					# msg_data=json.loads(full_msg.decode('utf-8'))
					msg_data=json.loads(tr_msg.decode('utf-8'))
					# print('%s: %s' %(msg_data.get("a"),msg_data.get("b")))
					# print('%s: %s' %(msg_data.get("c"),msg_data.get("d")))
					print('translation: %s' %(msg_data.get("translation")))
					print('rotation: %s' %(msg_data.get("rotation")))
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