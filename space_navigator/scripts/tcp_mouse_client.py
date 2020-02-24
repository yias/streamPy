#!/usr/bin/env python
"""
	developer: Iason Batzianoulis
	maintaner: Iason Batzianoulis
	email: iasonbatz@gmail.com

	description: 
	This scripts creates a TCP/IP client for streaming the inputs of a space navigator

"""

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

# import module for receiving data from the 3D mouse
import spnav

def randomString(strlength=10):
	letters=string.ascii_lowercase
	return ''.join(random.choice(letters) for i in range(strlength))


def handShake(ssock, strlength):

	ping_times=np.empty([10,1], dtype=np.float64)
	endMSG="!3tt"
	HEADERSIZE=4

	# echo server 10 times
	for i in range(10):
		dcdr=hashlib.md5()
		test_msg=randomString(strlength)
		dcdr.update(test_msg)
		print(test_msg)
		
		chSum=dcdr.hexdigest()
		msg_len=('{:<'+str(HEADERSIZE)+'}').format(str(sys.getsizeof(test_msg)))
		ssock.sendall(msg_len.encode('utf-8')+(test_msg).encode('utf-8')+chSum.encode('utf-8')+endMSG.encode('utf-8'))
		t_time=time.time()
		msg_full=''
		while(True):
			dataT=ssock.recv(4)
			msg_full+=dataT
			if msg_full[-4:].decode('utf-8')==endMSG:
				break
		print(msg_full.decode('utf-8'))


	return True



def main(args):
	

	# Create a TCP/IP socket
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


	# Connect the socket to the port where the server is listening
	server_address = (args.host, args.port)

	print('connecting to %s port %s', server_address)

	sock.connect(server_address)

	# check communication robustness with a hand-shake protocol
	is_connected=handShake(sock,10)

	# define a header size of the message
	HEADERSIZE=8

	# new message identifier
	msg_idf="!&?5"

	# end-of-message identifier
	endMSG="!3tt"

	# end-connection identifier
	ec_id="\ne@c"

	# normalized factor for the inputs of the spacenav in order for the values to be [-1,1]
	full_scale=512.0

	# The number of polls needed to be done before the device is considered "static"
	static_count_threshold=30

	# launch space navigator
	spnav.spnav_open()

	counter=0
	publish=False
	filter_counter=0
	valid_event_received=False
	zero_counter=0

	nb_msg_wailt=4

	tr_msg_group=np.empty([nb_msg_wailt,3], dtype=float)


	raw_input("Press Enter to continue ... ")

	startTime=time.time()

	msg_btn = [0, 0]
	bnt_flags = [False, False]

	while(True):
		try:
			# set an event object for handling the events of space navigator
			event=spnav.spnav_poll_event()

			# initialize hash key encryption
			dcdr=hashlib.md5()

			#initialize the messages
			msg_tr = [0.0,0.0,0.0]
			msg_rot = [0.0,0.0,0.0]
			
			# bnt2_flag = 

			
			if event is not None:
				if event.ev_type == spnav.SPNAV_EVENT_MOTION:
					# print(event.ev_type)
					# if there is an event (motion occurs), retrieve the data, normalize them and set the publish flag to True
					msg_tr=[event.translation[2]/full_scale, -event.translation[0]/full_scale, event.translation[1]/full_scale]
					msg_rot=[event.rotation[2]/full_scale, -event.rotation[0]/full_scale, event.rotation[1]/full_scale]
					valid_event_received=True
					# publish=True
				if event.ev_type == spnav.SPNAV_EVENT_BUTTON:
					print(event.press)

					if event.press: # right button
						bnt_flags[event.bnum] = True
					if event.press == False:
						bnt_flags[event.bnum] = False
					msg_btn = [int(bnt_flags[0]), int(bnt_flags[1])]
					publish = True
			else:
				# if there is no event (no motion), wait until a threshold to publish and sleep for 1 ms
				if zero_counter>=static_count_threshold:
					# publish=True
					valid_event_received=True
					zero_counter=0
				else:
					zero_counter+=1
				time.sleep(0.001)
			if valid_event_received:
				tr_msg_group[filter_counter,0]=msg_tr[0]
				tr_msg_group[filter_counter,1]=msg_tr[1]
				tr_msg_group[filter_counter,2]=msg_tr[2]
				valid_event_received=False
				filter_counter+=1
			if filter_counter>nb_msg_wailt-1:
				tmp=np.mean(tr_msg_group, axis=0)
				msg_tr=[tmp[0],tmp[1],tmp[2]]
				publish=True
				filter_counter=0
			if publish:
				# if the flag publish is True, compose and publish the message

				# throwing the data in a json object
				data=json.dumps({"translation": msg_tr, "rotation": msg_rot, "button": msg_btn})

				# create a hash key based on the message to be sent
				dcdr.update(data)
				chSum=dcdr.hexdigest()

				# create header with the size of the message (it will be introduced before the message)
				msg_len=('{:<'+str(HEADERSIZE)+'}').format(str(sys.getsizeof(data)))
				#msg_id=('{:<'+str(HEADERSIZE)+'}').format(str(counter))

				# encode and send message
				sock.sendall(msg_idf.encode('utf-8')+msg_len.encode('utf-8')+(data).encode('utf-8')+chSum.encode('utf-8')+endMSG.encode('utf-8'))

				# time of the loop
				time_passed=time.time()-startTime

				# print message info
				# print('time ', time_passed, ', translation: ', msg_tr, 'msgID', counter)

				# set the starting time of the next loop
				startTime=time.time()

				# increase counter and set publish flag to false (to avoid sending overpublishing)
				counter+=1
				publish=False

		except KeyboardInterrupt:
			# if Ctrl+C is pressed in the keyboard, send end-connection message and close the connection
			print('closing socket')
			sock.sendall(ec_id.encode('utf-8'))
			sock.close()
			break
		# finally:


	# shut down the space navigator
	print('closing 3D mouse communication')
	spnav.spnav_close()


if __name__=="__main__":
	__version__='0.7.5'

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
6