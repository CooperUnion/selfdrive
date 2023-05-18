#!/usr/bin/python3
# mostly stolen from Mike's/Jeannette's code
# adapted for Pico communications

from queue import Queue
from threading import Thread
import serial
import time
import traceback

from pico_bridge.comms.packet import *
from pico_bridge.comms.messages import *

class CommsController():
	def __init__(self, port, baudrate=115200, timeout=10):
		self.outbound_thread = Thread(
			target=self.handle_outbound, daemon=True
		)
		self.inbound_thread = Thread(
			target=self.handle_inbound, daemon=True
		)
		self.outbound = Queue()
		self.inbound = Queue()

		self.ser = serial.Serial(
			port=port, baudrate=baudrate,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			bytesize=serial.EIGHTBITS,
			timeout=timeout
		)
		self.connected = True # sort of vestigial

		self.ser.write('switch-modes\n'.encode('utf-8'))
		self.outbound_thread.start()
		self.inbound_thread.start()

	def handle_outbound(self):
		while True:
			try:
				if not self.outbound.empty():
					p = self.outbound.get()
					p.write_to(self.ser)
			except:
				print('exception in outbound loop:')
				print(traceback.format_exc())

	def handle_inbound(self):
		while True:
			try:
				if self.ser.inWaiting() > 0:
					p = Packet.read_from(self.ser)
					self.inbound.put(p)
			except:
				print('exception in inbound loop:')
				print(traceback.format_exc())

	def send(self, p):
		self.outbound.put(p)

	def has_packet(self):
		return not self.inbound.empty()

	def get_packet(self):
		return self.inbound.get()

if __name__ == "__main__":
	c = CommsController('/dev/ttyACM0')

	f1 = 0.0
	f2 = 1.0

	loop_time = 1
	while True:
		time.sleep(loop_time/2)

		tout = Test_Outbound(f1, f2)
		pout = tout.pack()
		c.send(pout)

		print(f'sending:    {tout}')
		print(f'packet out: {pout}')

		time.sleep(loop_time/2)
		while c.has_packet():
			pin = c.get_packet()
			tin = Test_Inbound(pin)

			print(f'packet in: {pin}')
			print(f'received:  {tin}')
			f1 = tin.field_1
			f2 = tin.field_2
			print()
