'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

from . CRC16.CRC16 import crc_calculate

'''
 @brieaf Class for implementing a storm32bgc protocol
  protocol:
  	startsign: 0xFA for incoming message, and 0xFB for outgoing message
  	length: length of payload, number of bytes of the data packet, excluding start sign, length byte, command byte and crc 
  	Command: the command byte
  	Payload: as many bytes as expected by the command
  	checksum: x25 16-bit crc excluding start byte as used by Mavlink.
'''
class Storm32bgcProtocol():
	def __init__(self):
		'''CONSTRUCTOR'''
		self.START_SIGN_IN		= 0xFA #Public const variable
		self.START_SIGN_OUT 	= 0xFB #Public const variable
		self.length 			= 0x00 #Public variable
		self.command 			= 0x00 #Public variable - set command value
		self.payload 			= [] #Public variable - Fill with payload bytes
		self.__crc16 			= 0xFFFF #Private variable

	'''
	 @brief Reset the protocol values
	'''
	def reset_protocol(self):
		self.length 	= 0x00
		self.command 	= 0x00
		self.payload 	= []
		self.__crc16 	= 0xFFFF

	'''
	 @brief Set the command
	'''
	def set_command(self, command):
		self.command = command & 0x00ff

	'''
	 @brief Set the payload.

	 @param payload The payload as a number (int)
	 @param n_bytes Number of bytes that the payload consist of.
	'''
	def set_payload(self, payload, n_bytes):
		if n_bytes <= 0xff:
			byte = 0xff #First byte in payload is lowest byte
			for i in range(0, n_bytes):
				tmp = (payload & byte) >> (8*i)
				self.payload.append(tmp)
				byte = (byte << 8)
			self.length = n_bytes
		else:
			print('Error:Storm32bgcProtocol:set_payload:Payload exceed max length')

	'''
	 @brief Set the checksum 
	'''
	def set_crc16(self):
		tmpbuffer		= [self.length, self.command] + self.payload
		self.__crc16 	= crc_calculate(tmpbuffer)

	'''
	 @brief Get the checksum
	'''
	def get_crc16(self):
		self.set_crc16()
		return self.__crc16

	'''
	 @brief Check an incoming checksum with the protocols checksum
	'''
	def check_crc16(self, crc16_check):
		ok = False
		if crc16_check == self.__crc16:
			ok = True
		return ok

	'''
	 @brief Pack protocol for sending on serial port

	 @return Array of bytes
	'''
	def get_packed_protocol(self):
		self.set_crc16()
		#buff = bytearray([self.START_SIGN_IN, self.length, self.command] + self.payload + [self.__crc16 & 0x00ff, (self.__crc16 & 0xff00) >> 8])
		return [self.START_SIGN_IN, self.length, self.command] + self.payload + [self.__crc16 & 0x00ff, (self.__crc16 & 0xff00) >> 8]

	'''
	 @brief Print protocol in hexa values
	'''
	def print_protocol(self):
		hex_payload = range(len(self.payload))
		for i in range(len(hex_payload)):
			hex_payload[i] = hex(self.payload[i])
		print '|---GIMBAL32BGC PROTOCOL---|'
		print 'Length: ' + hex(self.length)
		print 'Command: ' + hex(self.command)
		print 'payload: {}'.format(hex_payload)
		print 'CRC16: ' + hex(self.__crc16)