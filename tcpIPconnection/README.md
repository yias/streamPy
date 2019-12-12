# tcpIPconnection
python scripts for receiving data from a 3D space-navigator through a TCP/IP communication.
The "Object_Recognition" program reads captures pictures from a camera and classifies the pictures according to the user's preferencies. 
It has been created by the Learning Algorithms and Systems Laboratory (LASA) of EPFL

It has been developed in python 2.7 on Ubuntu 18.04

The program uses spnav module to receive events from a 3D space-navigator. See below for installation and running instructions


Please send your feedback or questions to:      iason.batzianoulis@epfl.ch


----------------------------------
       spnav set-up
----------------------------------

1) install the spacenavd and libspanv0 modules (pre-requisites for spnav)

$ sudo apt install spacenavd libspnav0 libspnav-dev

2) install the spnav module

$ pip install spnav

or clone and install spnav from source from this link
https://spnav.readthedocs.io/en/latest/


----------------------------------
     running the scripts
----------------------------------
- running the server

$ python tcp_server.py --host "the IP of the host"

- running the mouse client

$ python tcp_mouse_client.py --host "the server IP"
