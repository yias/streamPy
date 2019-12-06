
import evdev
from select import select
# import asyncio
# import sys

# @asyncio.coroutine 
# def print_events(device):
# 	while(True):
# 		events= yield from devices.async_read()
# 		for event in events:
# 			print(device.fn, evdev.categorize(event), sep=': ')
		


def main():
	devices=[evdev.InputDevice(fn) for fn in evdev.list_devices()]
	input_device=[]
	print('start')

	for device in devices:
		if(device.name=='3Dconnexion Space Navigator'):
			input_device=device.fn
		print(device.fn, device.name, device.phys)

	print('selected device is: %s' % input_device)

	dev=evdev.InputDevice(input_device)

	devices=map(evdev.InputDevice,('/dev/input/event2','/dev/input/event1'))
	devices={dev.fd: dev for dev in devices}

	for dev in devices.values(): print(dev)

	# print(dev.capabilities(verbose=True,absinfo=False))

	# dev.grab()

	print('begin reading:')

	while(True):
		r, w, x = select(devices,[],[])
		for fd in r:
			for event in dev[fd].read():
				print(event)

	# print(dev)

	# asyncio.ensure_future(print_events(dev))

	# loop = asyncio.get_event_loop()
	# loop.run_forever()

if __name__=="__main__":
	main()

# while(True):
# 	for event in dev.read():
# 		print(repr(event))
# 	# if event.type==ecodes.EV_KEY:
# 	# 	print(evdev.categorize(event))