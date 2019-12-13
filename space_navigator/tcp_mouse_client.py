# import standard modules 
import numpy as np
import argparse
import hashlib

# import modules for socket programming (TCP/IP connection)
import socket
import sys
import json

# import modules for theading programming
import threading
import time

# import module for receiving data from the 3D mouse
import spnav

def handShake(ssock):

	test_msg="tW#"

	ping_times=np.empty([10,1], dtype=np.float64)

	# echo server 10 times
	for i in range(10):
		dcdr=haslib.md5()
		dcdr.update(test_msg)
		ckSum=sum(bytearray(dcdr.digest()))
		chSum_msg=('{:<8}').format(str(ckSum))
		ssock.sendall((test_msg+i).encode('utf-8')+chSum_msg.encode('utf-8'))
		t_time=time.time()
		t_data=ssock.recv(5)


	return True



def main(args):
	

	# Create a TCP/IP socket
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


	# Connect the socket to the port where the server is listening
	server_address = (args.host, args.port)

	print('connecting to %s port %s', server_address)

	sock.connect(server_address)

	is_connected=handShake(sock)

	# define a header size of the message
	HEADERSIZE=8

	# new message identifier
	msg_idf="!&?5"

	# end-connection identifier
	ec_id="\ne@c"

	spnav.spnav_open()

	startTime=time.time()

	counter=0

	while(True):
		try: 
			event=spnav.spnav_poll_event()
			if event is not None:
				msg=[event.translation[0],event.translation[1],event.translation[2]]
				data=json.dumps({"a": 'translation', "b": msg})
				msg_len=('{:<'+str(HEADERSIZE)+'}').format(str(sys.getsizeof(data)))
				msg_id=('{:<'+str(HEADERSIZE)+'}').format(str(counter))
				sock.sendall(msg_idf.encode('utf-8')+msg_len.encode('utf-8')+msg_id.encode('utf-8')+data.encode('utf-8'))

				time_passed=time.time()-startTime
				print('time ', time_passed, ', translation: ', msg, 'msgID', counter)
				startTime=time.time()
				counter+=1
		except KeyboardInterrupt:
			print('closing socket')
			sock.sendall(ec_id.encode('utf-8'))
			sock.close()
			break
		# finally:


	print('closing 3D mouse communication')
	spnav.spnav_close()
	# print('closing socket')
	# sock.close()

if __name__=="__main__":
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
