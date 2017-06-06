'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

import time
from Settings.Exceptions import PtGreyError

'''
 @brief Pin control class.
 	Control a pin on the microcontroller

 @param pin (uint number)
 @param out_pin (True = pin control OUT, False = pin control IN (default=True))
'''
class PinControl():
	def __init__(self, pin, out_pin=True):
		'''CONSTRUCTOR'''
		try:
			import wiringpi2 as wpi
			self.__wpi = wpi
		except ImportError:
			raise PtGreyError('wiringpi_not_available_error_msg')

		self.__wpi.wiringPiSetup()
		self.SetPin(pin)
		if out_pin:
			self.SetPinOut()
		else:
			self.SetPinIn()

	def CheckPinControlAVailable(self):
		'''
		 @brief Check if pin control is available

		 @return True/False
		'''
		available = True
		if self.__wpi == None:
			available = False
		return available

	def AssertPinControlAvailable(self):
		'''
		 @brief Assert that the pin control is available
		'''
		if not(self.CheckPinControlAVailable()):
			raise Exception(self.wiringpi_not_available_error_msg)

	def GetPin(self):
		'''
		 @brief Get the pin in control
		'''
		self.AssertPinControlAvailable()
		return self.__pin

	def GetWiringControl(self):
		'''
		 @brief Get wiring control class instance

		 @return wpi
		'''
		self.AssertPinControlAvailable()
		return self.__wpi

	def SetPin(self, pin):
		'''
		 @brief Set which pin to control

		 @param pin (uint number)
		'''
		self.__pin = pin

	def SetPinOut(self):
		'''
		 @brief Set pin control out
		 	Default value out is low (0)
		'''
		self.AssertPinControlAvailable()
		self.__wpi.pinMode(self.__pin, 1)
		self.__out_pin = True
		self.SetPinLow()

	def SetPinIn(self):
		'''
		 @brief Set pin control in
		'''
		self.AssertPinControlAvailable()
		self.__wpi.pinMode(self.__pin, 0)
		self.__out_pin = False

	def CheckOutPin(self):
		'''
		 @brief Check if pin control is set to OUT (return True) or IN (return False)

		 @return True/False
		'''
		self.AssertPinControlAvailable()
		return self.__out_pin

	def AssertOutPin(self):
		'''
		 @brief Assert that pin control is set to OUT
		'''
		if not(self.CheckOutPin()):
			raise Exception('Set pin to OUT before writing!')

	def AssertInPin(self):
		'''
		 @brief Assert that pin control is set to OUT
		'''
		if self.CheckOutPin():
			raise Exception('Set pin to IN before reading!')

	def TogglePin(self, pause=0.0):
		'''
		 @brief Toggle pin

		 @param pause (insert delay between High->low (default=0))
		'''
		self.AssertOutPin()
		self.__wpi.digitalWrite(self.__pin, 1)
		if pause > 0:
			time.sleep(pause)
		self.__wpi.digitalWrite(self.__pin, 0)

	def SetPinHigh(self):
		'''	
		 @brief Set pin high
		'''
		self.AssertOutPin()
		self.__wpi.digitalWrite(self.__pin, 1)

	def SetPinLow(self):
		'''	
		 @brief Set pin low
		'''
		self.AssertOutPin()
		self.__wpi.digitalWrite(self.__pin, 0)

	def ReadPin(self):
		'''
		 @brief Read high/low value of pin

		 @return 1/0
		'''
		self.AssertInPin()
		return self.__wpi.digitalRead(self.__pin)

	def __del__(self):
		'''DESTRUCTOR'''
		if self.CheckPinControlAVailable():
			if self.CheckOutPin():
				self.SetPinLow()