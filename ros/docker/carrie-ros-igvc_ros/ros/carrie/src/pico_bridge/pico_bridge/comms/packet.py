from pico_bridge.comms.serialize import *
import base64
import hashlib

start_tx =   b'\x01' # SOH (start of heading)
start_data = b'\x02' # STX (start of text)
end_data =   b'\x03' # ETX (end of text)
end_tx =     b'\x04' # EOT (end of transmission)

class Packet:
	def __init__(self, p_id = None, data = None):
		self.id_ = p_id or 0
		self.data_ = data or ''

	def checksum(self, cksum = None):
		ser_id = serialize((Int32, ), [self.id_])
		self_sum = hashlib.sha1(ser_id + self.data_).hexdigest().encode('utf-8')

		if cksum:
			return cksum == self_sum
		else:
			return self_sum

	# sort of for interface completeness, but... sure
	def id(self):
		return self.id_

	def data(self):
		return self.data_

	def to_bytes(self):
		return \
			start_tx + \
			base64.b64encode(serialize((Int32, ), [self.id_])) + \
			start_data + \
			base64.b64encode(self.data_) + \
			end_data + \
			self.checksum() + \
			end_tx

	@classmethod
	def from_bytes(cls, b):
		if b[0] == start_tx:
			b = b[1:]

		b64_id, remainder = b.split(start_data, 1)
		b64_data, cksum = remainder.split(end_data, 1)
		if cksum[-1:] == end_tx:
			cksum = cksum[:-1]

		p_id, _ = deserialize((Int32, ), base64.b64decode(b64_id))
		p = cls(p_id[0], base64.b64decode(b64_data))

		if not p.checksum(cksum):
			raise Exception('checksum failed!')

		return p

	def write_to(self, ser):
		ser.write(self.to_bytes())
		ser.flush()

	# will block until a verified packet gets through!
	@classmethod
	def read_from(cls, ser):
		while True:
			try:
				ser.read_until(expected=start_tx)
				return cls.from_bytes(ser.read_until(expected=end_tx))
			except:
				pass

	def __repr__(self):
		return f'Packet<id_={self.id_}, data_={self.data_} ; {self.checksum()}>'
