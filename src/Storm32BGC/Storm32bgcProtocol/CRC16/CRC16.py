'''
	MAVlin x25 CRC16 calculations.

	Translated from C to python.
	Original C code made by:
		Author: Michael Shimniok
		URL: https://developer.mbed.org/users/shimniok/code/AVC_2012/file/826c6171fc1b/MAVlink/include/checksum.h

'''

'''GLOBAL CONST'''
X25_INIT_CRC 		= 0xffff
X25_VALIDATE_CRC 	= 0xf0b8


'''
 @brief Accumulate the X.25 CRC by adding one char at a time.
 
 The checksum function adds the hash of one char at a time to the
 16 bit checksum (uint16_t).
 
 @param data new char to hash
 @param crcAccum the already accumulated checksum
 @return updated crcAccum
'''
def crc_accumulate(data, crcAccum):
	tmp  = data ^ (crcAccum & 0xff)
	tmp ^= (tmp << 4)
	crcAccum = (crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
	return int(hex(crcAccum), 16) #Convert to 16 bit value

'''
 @brief Initiliaze the buffer for the X.25 CRC
'''
def crc_init():
	global X25_INIT_CRC
	return X25_INIT_CRC

'''
 @brief Calculates the X.25 checksum on a byte buffer
 
 @param  Buffer buffer containing the byte array to hash
 @return the checksum over the buffer bytes
'''
def crc_calculate(Buffer):
	length = len(Buffer)
	crcTmp = crc_init()

	for i in range(0, length):
		crcTmp = crc_accumulate(Buffer[i], crcTmp)

	return crcTmp