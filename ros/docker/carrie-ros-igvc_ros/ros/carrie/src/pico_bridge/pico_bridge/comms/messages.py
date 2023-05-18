from pico_bridge.comms.packet import *
from pico_bridge.comms.serialize import *

class Test_Outbound:
	def __init__(self, field_1, field_2):
		self.field_1 = field_1
		self.field_2 = field_2

	def pack(self):
		return Packet(377, serialize(
			(Float, Float),
			[self.field_1, self.field_2]
		))

	def __repr__(self):
		return f'Test_Outbound<{self.field_1}, {self.field_2}>'

class Test_Inbound:
	def __init__(self, p):
		[
			self.field_1, self.field_2, self.field_3
		], _ = deserialize(
			(Float, Float, Float),
			p.data()
		)

	def __repr__(self):
		return f'Test_Inbound<{self.field_1}, {self.field_2}, {self.field_3}>'
class SwitchToConsole:
	def __init__(self):
		self.id_ = 0

	def pack(self):
		return Packet(self.id_, b' ')

class SimpleMove:
	def __init__(self, msg, trans_id=0):
		self.id_ = 66

		self.packet_spec = (
			Float, # distance
			Float, # curvature
			Float, # velocity
			Int32, # trans_id
		)

		if type(msg) is tuple:
			self.fields = [
				msg[0],
				msg[1],
				msg[2],
				trans_id
			]
		else:
			self.fields = [
				msg.distance,
				msg.curvature,
				msg.velocity,
				trans_id
			]

	def pack(self):
		return Packet(self.id_, serialize(
			self.packet_spec,
			self.fields
		))

class MoveFeedback:
	def __init__(self, p):
		self.id_ = MoveFeedback.id()

		self.packet_spec = (
			Float, # distance
			Float, # curvature
			Float, # velocity
			Int32, # trans_id
		)

		[
			self.distance, self.curvature, self.velocity,
			self.trans_id,
		], _ = deserialize(
			self.packet_spec,
			p.data()
		)

	@classmethod
	def id(cls):
		return 42

	def __repr__(self):
		return f'MoveFeedback<{self.distance}, {self.curvature}, {self.velocity}, {self.trans_id}>'

class GetInfo:
	def __init__(self):
		self.id_ = 32

	def pack(self):
		return Packet(self.id_, b' ')

class Stop:
	def __init__(self):
		self.id_ = 666

	def pack(self):
		return Packet(self.id_, b' ')
