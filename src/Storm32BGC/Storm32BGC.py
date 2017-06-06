'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

import serial, math, threading, time, timeit

from . Storm32bgcProtocol.Storm32bgcProtocol import Storm32bgcProtocol

'''
 @brieaf The STorm32BGC class implements serial communication to the storm32bgc gimbal module.
  See: http://www.olliw.eu/storm32bgc-wiki/Serial_Communication

 @param Port serial port name
 @param baudrate of serial port
'''
class Storm32BGC():
	def __init__(self, port, baudrate):
		'''CONSTRUCTOR'''
		self.port 						= port
		self.baudrate 					= baudrate
		self.out_protocol_obj 			= Storm32bgcProtocol() # Protocol to storm32
		self.__in_protocol_obj 			= Storm32bgcProtocol() # Protocol from storm32 (private)
		self.__temp_in_protocol_obj 	= Storm32bgcProtocol() # Protocol temp from storm32 (private)
		self.__new_in_protocol			= False #(Private)
		self.__MAX_INPUT_VAL 			= 2300 #Max input value to gimbal (private const)
		self.__MIN_INPUT_VAL 			= 700 # Min input value to gimbal (private const)
		self.__ADAPT_VAL				= (self.__MAX_INPUT_VAL-self.__MIN_INPUT_VAL)/(2*math.pi)

		# Map of commands:
		self.COMMANDS = {
		'CMD_GETVERSION': 1,
		'CMD_GETVERSIONSTR': 2,
		'CMD_GETPARAMETER': 3,
		'CMD_SETPARAMETER': 4,
		'CMD_GETDATA': 5,
		'CMD_GETDATAFIELDS': 6,
		'CMD_SETPITCH': 10, 
		'CMD_SETROLL': 11,
		'CMD_SETYAW': 12,
		'CMD_SETPANMODE': 13,
		'CMD_SETSTANDBY': 14,
		'CMD_DOCAMERA': 15,
		'CMD_SETSCRIPTCONTROL': 16,
		'CMD_SETANGLE': 17,
		'CMD_SETPITCHROLLYAW': 18,
		'CMD_SETPWMOUT': 19,
		'CMD_RESTOREPARAMETER': 20,
		'CMD_RESTOREALLPARAMETER': 21,
		'CMD_ACTIVEPANMODESETTING': 100,
		'CMD_ACK': 150
		}

		# Map of data fields (used in GETDATAFIELDS command)
		self.DATAFIELDS = {
		'LIVEDATA_STATUS': 0x0001,
		'LIVEDATA_TIMES': 0x0001,
		'LIVEDATA_IMU1GYRO': 0x0004,
		'LIVEDATA_IMU1ACC': 0x0008,
		'LIVEDATA_IMU1R': 0x0010,
		'LIVEDATA_IMU1ANGLES': 0x0020,
		'LIVEDATA_PIDCNTRL': 0x0040,
		'LIVEDATA_INPUTS': 0x0080,
		'LIVEDATA_IMU2ANGLES': 0x0100,
		'LIVEDATA_MAGANGLES': 0x0200,
		'LIVEDATA_STORM32LINK': 0x0400,
		'LIVEDATA_IMUACCCONFIDENCE': 0x0800
		}

		# Map of possible ACK command responses sent due to error
		self.ACK_RESPONSE = {
		0: 'SERIALRCCMD_ACK_OK',
		1: 'SERIALRCCMD_ACK_ERR_FAIL',
		2: 'SERIALRCCMD_ACK_ERR_ACCESS_DENIED',
		3: 'SERIALRCCMD_ACK_ERR_NOT_SUPPORTED',
		150: 'SERIALRCCMD_ACK_ERR_TIMEOUT',
		151: 'SERIALRCCMD_ACK_ERR_CRC ',
		152: 'SERIALRCCMD_ACK_ERR_PAYLOADLEN'
		}

		#Initialize serial port and open.
		self.serial_obj = serial.Serial(
			port 		= self.port,
			baudrate 	= self.baudrate,
			bytesize 	= serial.EIGHTBITS,
			parity 		= serial.PARITY_NONE,
			stopbits 	= serial.STOPBITS_ONE,
			timeout 	= 0.5
		)

		'''
		 Threading objects
		 serial_mutex lock used for reading and writing.
		 protocol_mutex used for reading of latest incoming protocol.
		 Reading of incoming procotols is read continously in separate thread.
		'''
		self.serial_mutex 	= threading.Lock()
		self.protocol_mutex = threading.Lock() 
		t = threading.Thread(target=self.continously_recv_protocol)
		t.daemon = True
		t.start()

	'''
	 @brief Send any command with payload.

	 @param command Command to send.
	 @param payload Payload (int) to send.
	'''
	def send_command(self, command, payload):
		self.out_protocol_obj.reset_protocol()
		cp_payload = payload
		n_bytes = 0
		while cp_payload > 0:
			cp_payload = (cp_payload >> 8)
			n_bytes += 1
		self.out_protocol_obj.set_payload(payload, n_bytes)
		self.out_protocol_obj.set_command(command)
		self.send_protocol()

	'''
	 @brief Convert radians to input values for storm32bgc gimbal rotation
	  Input value to storm32bgc is between 700...2300 (uint16), 
	  and if input value is 0 then the gimbal will be recentered (which will occur at radians = 0).

	 @param radians Radian degree between 0..2pi
	 @return storm32bgc input value for gimbal rotation
	'''
	def radians_to_stormVal(self, radians):
		input_val = int(round(self.__ADAPT_VAL*radians + self.__MIN_INPUT_VAL))
		return input_val

	'''
	 @brief Convert input values for storm32bgc gimbal rotation to radians
	  Input value to storm32bgc is between 700...2300 (uint16), 
	  and if input value is 0 then the gimbal will be recentered (which will occur at radians = 0).

	 @param storm32bgc input value for gimbal rotation
	 @return return Radian degree between 0..2pi
	'''
	def stormVal_to_radians(self, stormVal):
		radians = (stormVal - self.__MIN_INPUT_VAL)/self.__ADAPT_VAL
		return radians

	'''
	 @brief Set the pitch/roll or yaw radian degree between 0..2pi.

	 @param radians Radian degree between 0..2pi
	 @param command Command as given by the COMMANDS map
	'''
	def set_rotation(self, radians, command):
		self.out_protocol_obj.reset_protocol()
		self.out_protocol_obj.set_payload(self.radians_to_stormVal(radians), 2)
		self.out_protocol_obj.set_command(command)
		self.send_protocol()

	'''
	 @brief Send the outgoing protocol. 
	'''
	def send_protocol(self):
		with self.serial_mutex:
			self.serial_obj.write(serial.to_bytes(self.out_protocol_obj.get_packed_protocol()))

	'''
	 @brief Receive an incoming protocol.
	  The incoming protocol is stored in the object - __in_protocol_obj.
	  ACK responses are not collected as new protocols, but the reply is printed to screen.

	 @return True/False on a successfully received protocol. 
	'''
	def recv_protocol(self):
		ok = False
		if self.serial_obj.inWaiting() > 3:
			resp = self.recv_bytes_serial(1)
			if resp == self.__temp_in_protocol_obj.START_SIGN_OUT:
				with self.protocol_mutex:
					resp = self.recv_bytes_serial(2)
					self.__temp_in_protocol_obj.length 		= resp[0]
					self.__temp_in_protocol_obj.command 	= resp[1]
					resp = self.recv_bytes_serial(self.__temp_in_protocol_obj.length+2)
					self.__temp_in_protocol_obj.payload 	= resp[:-2]
					self.__temp_in_protocol_obj.set_crc16()
					recv_crc16 = resp[-2:][0]|(resp[-1:][0] << 8)
					crc_16 = self.__temp_in_protocol_obj.get_crc16()
					print 'received crc: {}, calc crc: {}'.format(recv_crc16, crc_16)
					print 'received crc: {} {}, calc crc: {} {}'.format(resp[-2:][0], resp[-1:][0], crc_16 & 0x00ff, (crc_16 & 0xff00) >> 8)
					if self.__temp_in_protocol_obj.check_crc16(recv_crc16):
						ok = True
						self.__in_protocol_obj = copy.deepcopy(self.__temp_in_protocol_obj)	#Copy temp protocol when the new protocol is received and verified
						self.__new_in_protocol = True #Signal that a new protocol is received, so it can be collected
					else:
						self.__temp_in_protocol_obj.print_protocol()
						self.check_ack_response(self.__temp_in_protocol_obj)
						self.__temp_in_protocol_obj.reset_protocol()
						print('Error:Storm32BGC:recv_protocol:Checksum error')
		return ok

	'''
	 @brief Receive a string on the serial port and convert it to int list

	 @param n_bytes Number of bytes to read
	 @return Ordinary int list
	'''
	def recv_bytes_serial(self, n_bytes):
		resp_tmp = self.serial_obj.read(n_bytes)
		if n_bytes > 1:
			resp = range(n_bytes)
			for i in range(0, n_bytes):
				resp[i] = ord(resp_tmp[i])
		else:
			resp = ord(resp_tmp)
		return resp

	'''
	 @brief Read serial port to find an incoming protocol.
	  Open as long as the serial port is open.
	'''
	def continously_recv_protocol(self):
		while self.serial_obj.isOpen():
			with self.serial_mutex:
				self.recv_protocol()
			time.sleep(0.05)

	'''
	 @brief Get the last successfully received protocol.

	 @return Storm32bgcProtocol object or None
	'''
	def get_latest_incoming_protocol(self):
		with self.protocol_mutex:
			if self.__new_in_protocol:
				self.__new_in_protocol = False #Signal that the new protocol is collected
				return self.__in_protocol_obj
		return None

	'''
	 @brief Wait until a successfully received protocol is received.

	 @param timelimit Max timelimit in seconds
	 @return Storm32bgcProtocol object or None
	'''
	def wait_for_new_protocol(self, timelimit):
		new_protocol_obj 	= None
		start_time 			= timeit.default_timer()
		elapsed 			= 0
		while new_protocol_obj == None and elapsed < timelimit:
			new_protocol_obj 	= self.get_latest_incoming_protocol()
			elapsed 			= timeit.default_timer() - start_time
		return new_protocol_obj

	'''
	 @brief Check if protocol is an ack protocol, and print response to screen.
	  
	 @protocol Protocol to check
	 @return True/False
	'''
	def check_ack_response(self, protocol):
		ok = False
		if protocol.command == self.COMMANDS['CMD_ACK']:
			ok = True
			print self.ACK_RESPONSE[protocol.payload[0]]
		return ok

	'''
	 @brief Executed on deleting this class. 
	  Closes the serialport.
	'''
	def __del__(self):
		self.serial_obj.close()
