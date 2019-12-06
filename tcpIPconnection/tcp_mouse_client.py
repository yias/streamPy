# import modules for socket programming (TCP/IP connection)
import socket
import sys
import json

# import modules for theading programming
import threading
import time

# import module for receiving data from the 3D mouse
import spnav


def main():
	

	# Create a TCP/IP socket
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


	# Connect the socket to the port where the server is listening
	server_address = ('localhost', 10353)

	print('connecting to %s port %s', server_address)

	sock.connect(server_address)

	msg=[]

	tmp_msg='test_msg'

	spnav.spnav_open()

	startTime=time.time()

	while(True):
		try: 
			event=spnav.spnav_poll_event()
			if event is not None:
				msg=[event.translation[0],event.translation[1],event.translation[2]]
				data=json.dumps({"a": 'translation', "b": msg})
				sock.sendall(data.encode())
				time_passed=time.time()-startTime
				print('time ', time_passed, ', translation: ', msg, type(msg))
				startTime=time.time()
		except KeyboardInterrupt:
			print('closing socket')
			sock.close()
			break
		# finally:


	print('closing 3D mouse communication')
	spnav.spnav_close()
	# print('closing socket')
	# sock.close()

if __name__=="__main__":
	main()
