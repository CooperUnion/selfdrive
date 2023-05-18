import struct

class Char:
	@staticmethod
	def insert(c):
		assert len(c) == 1, 'don\'t pass more than one character, yo!'
		return c.encode('utf-8')[:1] # ascii-only lol

	@staticmethod
	def extract(b):
		c = b[:1].decode('utf-8')
		return c, b[1:]

class Int32:
	@staticmethod
	def insert(i):
		return i.to_bytes(4, byteorder='big')

	@staticmethod
	def extract(b):
		return int.from_bytes(b[:4], byteorder='big'), b[4:]

class Float:
	@staticmethod
	def insert(f):
		# XXX: should maybe change eventually
		return struct.pack('>f', f)

	@staticmethod
	def extract(b):
		# note: endianness specifier required for inter-communication
		# with the Pico -- empirically determined, hence terrible.
		return struct.unpack('>f', b[:4])[0], b[4:]

def serialize(packet_spec, packet_vals):
	assert len(packet_spec) == len(packet_vals), 'packet specification does not match the packet (in length, that is)'

	packet = []
	for i in range(len(packet_spec)):
		next_type = packet_spec[i]
		next_val = packet_vals[i]
		packet.append(next_type.insert(next_val))
	return b''.join(packet)

def deserialize(packet_spec, raw_packet):
	packet_vals = []
	for next_type in packet_spec:
		next_val, raw_packet = next_type.extract(raw_packet)
		packet_vals.append(next_val)
	return packet_vals, raw_packet

if __name__ == "__main__":
	c1 = 'u'
	c2 = 'e'
	i1 = 377
	f1 = 2.71828

	spec = (Char, Char, Int32, Float)

	packet = serialize(spec, [c1, c2, i1, f1])
	print(f'encoded: {packet}')

	packet_values = deserialize(spec, packet)
	print(f'decoded: {packet_values}')
