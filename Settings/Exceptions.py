'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

class PtGreyError(Exception):
	def __init__(self, error_key=None):
		'''CONSTRUCTOR'''
		self.__error_messages = {	'import_fc2_error_msg'					: 'flycapture2 library is not installed - see HOWTO.txt to install.', \
									'camera_not_connected_error_msg'		: 'Camera is not connected!', \
									'camera_not_capturing_error_msg'		: 'Camera has not been started capturing frames!', \
									'timeout_capturing_frame_error_msg'		: 'Timeout capturing frame!', \
									'wiringpi_not_available_error_msg' 		: 'wiringPi2 is not awailable, cannot use pin control.', \
									'failed_capturing_frame_error_msg' 		: 'Failed capturing frame from PtGray (API error)'}

		self.msg = None
		if error_key != None:
			self.msg = self.__error_messages[error_key]

	def __str__(self):
		return repr(self.msg)

	def GetErrorMessage(self, error_key):
		'''
		 @brief Return all possible error messages
		'''
		return self.__error_messages[error_key]

class DroneVisionError(Exception):
	def __init__(self, error_key=None):
		'''CONSTRUCTOR'''

		self.__error_messages = {	'no_blobs_error_msg'								: 'No blobs detected!', \
									'blob_distance_is_not_calibrated_error_msg'			: 'Blob distance is not calibrated!', \
									'feature_descriptor_not_available_error_msg'		: 'Feature descriptor not avaiable - Use block matching!', \
									'find_line_limits_unexpected_angle'					: 'Unexpected angle: provide vertical or horizontal hough lines only',  \
									'error_msg_no_heading_angle_detected' 				: 'No heading angles detected', \
									'error_msg_no_backup_heading_available'				: 'No edge heading detected, and no backup heading available.', \
									'find_line_limits_no_hor_or_vert_found' 			: 'Only vertical or horizontal lines found - need both horizontal and vertical lines to process.', \
									'detect_boundary_edge_not_found_all_edge_lines'		: 'Could not compute all edges lines - need all boundary edge lines to process', \
									'no_3D_point_matches'								: 'Could not compute 3D points - no matching points.', \
									'could_not_get_point_list_from_slave'				: 'Timeout thread - could not get right point list from slave within timeout!', \
									'triangulation_error_msg'							: 'Triangulation failed!', \
									'3D_point_filtration_failed'						: 'Filtration of 3D points failed!'}

		self.msg = None
		if error_key != None:
			self.msg = self.__error_messages[error_key]

	def __str__(self):
		return repr(self.msg)

	def GetErrorMessage(self, error_key):
		'''
		 @brief Return all possible error messages
		'''
		return self.__error_messages[error_key]