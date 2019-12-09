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
	server_address = ('localhost', 10351)

	print('connecting to %s port %s', server_address)

	sock.connect(server_address)

	# define a header size of the message
	HEADERSIZE=8

	# define message identifier
	msg_idf="!&?5"

	msg=[]

	tmp_msg='test_msg'

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
				msg_id='{:<8}'.format(str(counter))
				sock.sendall(msg_idf.encode('utf-8')+msg_len.encode('utf-8')+msg_id.encode()+data.encode())

				time_passed=time.time()-startTime
				print('time ', time_passed, ', translation: ', msg, 'msgID', counter)
				startTime=time.time()
				counter+=1
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
