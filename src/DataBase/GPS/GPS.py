'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

'''
 @brief GPS class for receiving GPS position.
'''
class GPS():
	def __init__(self):
		'''CONSTRUCTOR'''
		self.InitGPS()

	def InitGPS(self):
		'''
		 @brief Initialize GPS
		'''
		pass

	def GetGPSPosition(self):
		'''
		 @brief Get current GPS position.

		 @return gps_position (Return: gps_position = {'longitude': longitude, 'langitude': langitude, 'latitude': latitude})
		'''
		##################
		# TODO
		longitude 	= 0.0
		langitude 	= 0.0
		latitude 	= 0.0
		##################
		return {'longitude': longitude, 'langitude':langitude, 'latitude':latitude}