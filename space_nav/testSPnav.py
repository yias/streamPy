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

	spnav.spnav_open()
	
	while(True):
		try: 
			event=spnav.spnav_poll_event()
			if event is not None:
				print(event.translation[0])
		except KeyboardInterrupt:
			break

		if 0xFF==ord('q'):
			break

	spnav_close()

if __name__=="__main__":
	main()