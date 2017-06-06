'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

import numpy as np
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape
from MsgParserRecv.keypointsConverter import keypoints_to_list, list_to_keypoints

'''
 @brief Class for handling requests between master and slave.

 @param master_or_slave (True/False - True = this is master, False = this is slave)
'''
class Requests():
	def __init__(self, master_or_slave):
		'''CONSTRUCTOR'''
		self.__master_or_slave = master_or_slave

	def GetContentRequestFrame(self, content, error=False):
		'''
		 @brief Get unpacked/packet content for frame request.

		 @param content
		 @param error (True/False)

		 if master:
		 	@return frame_content, valid (Return: frame_content = frame data as returned from GetProcessedFrame(), valid = True/False)
		 else: (slave)
		 	@return content
		'''
		frame_content = None
		if self.__master_or_slave: # True = master 
			valid = content['valid']
			error = content['error']
			if valid:
				original_frame, original_sl_frame, frame_un, dtype, delta_frame, keypoints, descriptors = content['frame_content']
				dtype 				= np.dtype(dtype)
				original_frame 		= np.array(original_frame, dtype=dtype)
				original_sl_frame 	= np.array(original_sl_frame, dtype=dtype)
				frame_un       		= np.array(frame_un, dtype=dtype)
				delta_frame   		= np.array(delta_frame, dtype=dtype)
				keypoints			= list_to_keypoints(keypoints)
				descriptors 		= np.array(descriptors)
				frame_content  		= (original_frame, original_sl_frame, frame_un, delta_frame, keypoints, descriptors)
			return frame_content, valid, error
		else:
			valid = False
			if not(isinstance(content, type(None))):
				valid = True
				original_frame, original_sl_frame, frame_un, delta_frame, keypoints, descriptors = content
				dtype 				= str(original_frame.dtype)
				original_frame 		= original_frame.tolist()
				original_sl_frame 	= original_sl_frame.tolist()
				frame_un 			= frame_un.tolist()
				delta_frame 		= delta_frame.tolist()
				keypoints			= keypoints_to_list(keypoints)
				descriptors 		= descriptors.tolist()
				frame_content 		= (original_frame, original_sl_frame, frame_un, dtype, delta_frame, keypoints, descriptors)
			content = {'frame_content': frame_content, 'valid': valid, 'error': error}
			return content

	def GetContentRequestOriginalFrame(self, content, error=False):
		'''
		 @brief Get unpacked/packet content for original frame request.

		 @param content
		 @param error (True/False)

		 if master:
		 	@return frame_content, valid (Return: frame_content = frame data as returned from GetProcessedFrame(), valid = True/False)
		 else: (slave)
		 	@return content
		'''
		frame_content = None
		if self.__master_or_slave: # True = master 
			valid = content['valid']
			error = content['error']
			if valid:
				if content['frame_content'] == 2:
					original_frame, dtype = content['frame_content']
					dtype 				= np.dtype(dtype)
					original_frame 		= np.array(original_frame, dtype=dtype)
					frame_content  		= (original_frame)
				else:
					original_frame, original_sl_frame, dtype = content['frame_content']
					dtype 				= np.dtype(dtype)
					original_frame 		= np.array(original_frame, dtype=dtype)
					original_sl_frame 	= np.array(original_sl_frame, dtype=dtype)
					frame_content  		= (original_frame, original_sl_frame)
			return frame_content, valid, error
		else:
			valid = False
			if not(isinstance(content, type(None))):
				valid = True
				if len(content) == 1:
					original_frame = content
					dtype 				= str(original_frame.dtype)
					original_frame 		= original_frame.tolist()
					frame_content 		= (original_frame, dtype)
				else:
					original_frame, original_sl_frame = content
					dtype 				= str(original_frame.dtype)
					original_frame 		= original_frame.tolist()
					original_sl_frame 	= original_sl_frame.tolist()
					frame_content 		= (original_frame, original_sl_frame, dtype)
			content = {'frame_content': frame_content, 'valid': valid, 'error': error}
			return content

	def GetRequestTradeFrame(self, content, recv_content=None, error=False):
		'''
		 @brief Trade frame between master and slave

		 @oaram content
		 @param recv_content (recv_content from master (default=None))
		 @param error (default=False)
		'''
		frame_content = None
		if self.__master_or_slave: # True = master 
			valid = content['valid']
			error = content['error']
			if valid:
				original_frame, dtype 	= content['frame_content']
				dtype 					= np.dtype(dtype)
				original_frame 			= np.array(original_frame, dtype=dtype)
				frame_content  			= (original_frame)
			return frame_content, valid, error
		else:
			valid = False
			if not(isinstance(content, type(None))):
				valid = True
				new_original_frame, dtype 	= recv_content['frame_content']
				dtype 						= np.dtype(dtype)
				new_original_frame 			= np.array(new_original_frame, dtype=dtype)
				original_frame 				= content
				dtype 						= str(original_frame.dtype)
				original_frame 				= original_frame.tolist()
				frame_content 				= (original_frame, dtype)
			content = {'frame_content': frame_content, 'valid': valid, 'error': error}
			return content, new_original_frame

	def GetContentRequestPointList(self, content, error=False):
		'''
		 @brief Get unpacked/packet content for point list request.

		 @param content
		 @param error (True/False)

		 if master:
		 	@return keypoints, descriptors, valid (Return: keypoints = point keys data, descriptors = point description data, valid = True/False)
		 else: (slave)
		 	@return content
		'''
		keypoints 	= None
		descriptors = None
		und_shape 	= None
		if self.__master_or_slave: # True = master 
			valid = content['valid']
			error = content['error']
			if valid:
				und_shape 	= content['und_shape']
				keypoints 	= content['keypoints']
				descriptors = content['descriptors']
				keypoints 	= list_to_keypoints(keypoints)
				descriptors = np.array(descriptors)
			return und_shape, keypoints, descriptors, valid, error
		else:
			valid = False
			if not(isinstance(content, type(None))):
				valid = True
				original_frame, original_sl_frame, frame_un, delta_frame, keypoints, descriptors = content
				und_shape 	= GetShape(frame_un)
				keypoints 	= keypoints_to_list(keypoints)
				descriptors = descriptors.tolist()
			content = {'und_shape': und_shape, 'keypoints': keypoints, 'descriptors': descriptors, 'valid': valid, 'error': error}
			return content

	def GetContentRequestSetTimestamp(self, content):
		'''
		 @brief Get unpacked/packet content for timestamp.

	 	 @param content (content/timestamp)

	 	 if master:
	 	 	@return content (dictionary with timestamp)
	 	 else: (slave)
	 	 	@return timestamp
		'''
		if self.__master_or_slave: # True = master 
			content = {'timestamp': content}
			return content
		else:
			timestamp = content['timestamp']
			return timestamp

	def GetContentRequestSlaveReady(self, content):
		'''
		 @brief Get unpacked/packet content for slave ready.

	 	 @param content (content/ready)

	 	 if master:
	 	 	@return ready (bool)
	 	 else: (slave)
	 	 	@return content (dictionary with ready bool)
		'''
		if self.__master_or_slave: # True = master 
			ready = content['ready']
			return ready
		else:
			content = {'ready': content}
			return content

	def GetContentSendFlagToSlave(self, content):
		'''
		 @brief Get unpacked/packet content for sending flag to slave.

	 	 @param content (content/flag)

	 	 if master:
	 	 	return content (dictionary with flag)
	 	 else: (slave)
	 	 	@return flag
		'''
		if self.__master_or_slave: # True = master 
			content = {'flag': content}
			return content
		else:
			flag = content['flag']
			return flag