'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

import cv2, math, warnings
import numpy as np
from Settings.Exceptions import DroneVisionError
from EdgeHeading import EdgeHeading
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape, CheckColor

'''
 @brief Heading class. Compute heading direction (rho, theta).
 	- rho 	= Distance to next heading position (in image frame)
 	- theta = heading angle towards next position (in image frame)
 	The heading (rho, theta) is in regard to the image center.

 @param rho_step_distance #If None = then each step distance (how far the drone should fly during each step) is set to a quarter the diagonal frame size (default=None)
 @param rho_min_diag_perc (Default percent of the diagonal length from center to image edge, used to calculate rho_min to navigate along a singel edge. Given as a percent between 0-1 (default=0.25))
 @param default_following_horizontal_edges (False for following vertical edges, True for following horizontal edges from beginning. None results in automatic detection of which direction to follow. (default=None))
 @param default_moving_towards_tip (True for moving towards the blade tip, False for moving back to the blade root from beginning. (default=True))
'''
class Heading(EdgeHeading):
	def __init__(self, rho_step_distance=None, rho_min_diag_perc=1/4.0, default_following_horizontal_edges=None, default_moving_towards_tip=True):
		'''CONSTRUCTOR'''
		self.__current_blade_heading 		= (None, None)
		self.__current_tip_heading			= (None, None)
		self.__rho_step_distance 			= rho_step_distance
		self.__rho_min_diag_perc 			= rho_min_diag_perc
		self.__following_horizontal_edges 	= default_following_horizontal_edges # False for following vertical edges, True for following horizontal edges. (init=None)
		self.__moving_towards_tip 			= default_moving_towards_tip # True for moving towards the blade tip, False for moving back to the blade root. (init=True)
		EdgeHeading.__init__(self)

	def SetMovingTowardsTip(self):
		'''
		 @brief Set moving direction towards the tip.
		'''
		self.__moving_towards_tip = True

	def SetMovingTowardsRoot(self):
		'''
		 @brief Set moving direction towards the root.
		'''
		self.__moving_towards_tip = False

	def GetMovingTowardsTipOrRoot(self):
		'''
		 @brief Get True/False for moving towards the tip or root of the blade.
		 	True 	= moving towards the tip
		 	False 	= moving towards the root

		 @return True/False
		'''
		return self.__moving_towards_tip

	def GetCurrentHeading(self):
		'''
		 @brief Get current heading (rho, theta) along blade and to tip.
		 	Returns (None, None) if the current heading has not been set.

		 @return current_blade_heading, current_tip_heading (rho, theta)
		'''
		return self.__current_blade_heading, self.__current_tip_heading

	def SetCurrentHeading(self, blade_heading, tip_heading):
		'''
		 @brief Set current blade and tip heading

		 @param blade_heading (rho, theta)
		 @param tip_heading (rho, theta)
		'''
		self.__current_blade_heading 	= blade_heading
		self.__current_tip_heading 		= tip_heading

	def ComputeHeading(self, frame, boundary_hough_lines, draw_possible_edge_headings=False, draw_headings=False, draw_elliptical_error_arrow=True, draw_min_rho=True):
		'''
		 @brief Compute heading according to the boundary_hough_lines.

		 	All headings are measured from image center, meaning returned rho and theta are in regard to the image center.
		 	Set draw_possible_edge_headings=True to return a colored frame showing all edge headings.
		 	Set draw_headings=True to return a colored frame showing the heading.
		 	The function considers the current heading to find the most probable next heading.
		 	Use SetCurrentHeading() to update the current heading, or GetCurrentHeading() to get it.

		 @param frame (undistorted frame)
		 @param boundary_hough_lines (See PointDetection.GetBoundaryHoughLines)
		 	boundary_hough_lines = [max_hor_hough_line, min_hor_hough_line, max_vert_hough_line, min_vert_hough_line],
		 	with hough line as (rho, theta)
		 @param draw_possible_edge_headings (default=False)
		 @param draw_headings (YELLOW = selected edge heading, ORANGE = selected heading, RED = selected tip/root heading (default=False))
		 @param draw_elliptical_error_arrow (DARK RED = drawn elliptical error arrow (True/False - default=True))
		 @param draw_min_rho (Draw minium rho of draw_elliptical_error_arrow == True (True/False - default=True))

	 	 @return selected_blade_heading, selected_tip_or_root_heading, tip_or_root_detected, frame
	 		 (Return 
	 		 	selected_blade_heading 			= (rho, theta) (Will be (None, None) if not detected), 
	 		 	selected_tip_or_root_heading 	= (rho, theta) (Will be (None, None) if not detected), 
	 		 	tip_or_root_detected 			= True/False
	 		 	frame
	 		rho = distance to edge, theta = heading direction in radian degrees.
	 		frame (if draw_possible_edge_headings or draw_headings = True, then it is converted to color frame))
		'''
		try:
			edge_heading_returns = self.ComputeEdgeHeading(frame, boundary_hough_lines, draw=draw_possible_edge_headings)
		except DroneVisionError, err:
			if str(err) == str(DroneVisionError('error_msg_no_heading_angle_detected')):
				warnings.simplefilter('always')
				warnings.warn(str(err), Warning)
				warnings.simplefilter('default')
				return self.HandleErrorNoEdgeHeadingDetected()
			raise

		selected_hor_edge_heading, selected_vert_edge_heading, possible_hor_edge_headings, possible_vert_edge_headings = edge_heading_returns[:4]
		if draw_possible_edge_headings:
			frame = edge_heading_returns[4]

		selected_blade_edge_headings, selected_tip_heading = self.GetHorizontalOrVerticalEdgeFollowing(selected_hor_edge_heading, selected_vert_edge_heading, possible_hor_edge_headings, possible_vert_edge_headings)
		if self.GetMovingTowardsTipOrRoot(): #True for moving towards the tip
			tip_or_root_detected, selected_tip_or_root_heading = self.GetTipDetected(frame, selected_tip_heading)
			if tip_or_root_detected: # Simple turn around - should not be a final solution
				self.SetMovingTowardsRoot()
		else:
			tip_or_root_detected, selected_tip_or_root_heading = self.GetRootDetected(frame)
		selected_blade_heading = self.FindHeading(frame, selected_blade_edge_headings, rho_step_distance=self.__rho_step_distance, rho_min_diag_perc=self.__rho_min_diag_perc)

		if draw_headings:
			if len(selected_blade_edge_headings) == 1 and draw_elliptical_error_arrow: # Following a single edge
				frame = self.DrawErrorArrow(frame, selected_blade_edge_headings[0], draw_min_rho=draw_min_rho, rho_min_diag_perc=self.__rho_min_diag_perc, color=(153,0,0), min_rho_color=(102,0,0)) # DARK RED
			if tip_or_root_detected:
				frame = self.DrawHeading(frame, selected_tip_or_root_heading, color=(255,0,0), line_thick=10)	#RED
			for edge_heading in selected_blade_edge_headings:
				frame = self.DrawHeading(frame, edge_heading, color=(255,255,0), line_thick=7) 					#YELLOW
			frame = self.DrawHeading(frame, selected_blade_heading, color=(255,128,0))							#ORANGE

		return selected_blade_heading, selected_tip_or_root_heading, tip_or_root_detected, frame

	def ResetHeading(self):
		'''
		 @brief Reset heading
		'''
		self.__current_heading 				= (None, None)
		self.__following_horizontal_edges 	= None

	def FindHeading(self, frame, selected_blade_edge_headings, rho_step_distance=None, default_rho_step_distance_diag_perc=1/4.0, rho_min_diag_perc=1/4.0, flip_heading=False):
		'''
		 @brief Find the next heading based on selected edge heading and current heading.
		 	Computed using equations (6.14 to 6.17) in MSc final thesis - Hans Erik Heggem
		 	Reset the heading by using ResetHeading()

		 @param frame
		 @param selected_blade_edge_headings
		 @param rho_step_distance (step distance in pixels for the drone to fly during each iteration. If None = then each step distance is set to a quarter the diagonal frame size (default=None))
		 @param default_rho_step_distance_diag_perc (Default step distance if rho_step_distance==None. Given as a percent of the diagonal frame size (default=0.25))
		 @param rho_min_diag_perc (Default percent of the diagonal length from center to image edge, used to calculate rho_min to navigate along a singel edge. Given as a percent between 0-1 (default=0.25))
		 @param flip_heading (Flip heading in opposite direction (True/False))

		 @return selected_heading (rho = distance to next heading position (in image frame), theta = heading angle in radian degrees)
		'''
		selected_heading = (0.0,0.0)
		if len(selected_blade_edge_headings) > 0:
			I_l, I_w 	= GetShape(frame)
			diag_size 	= np.sqrt(I_l*I_l + I_w*I_w)
			if rho_step_distance == None:
				rho_step_distance = diag_size*default_rho_step_distance_diag_perc

			if len(selected_blade_edge_headings) > 1:
				if self.VerifyEdgeQuadrant(selected_blade_edge_headings[0]) and self.VerifyEdgeQuadrant(selected_blade_edge_headings[1]):
					omega = (selected_blade_edge_headings[0][1] + selected_blade_edge_headings[1][1])/2.0 + np.pi
					if not(self.GetMovingTowardsTipOrRoot()): # True for moving towards tip.
						omega -= np.pi
					if flip_heading:
						omega += np.pi
				else: # Either one of the edges are in the incorrect quadrant - estimate heading based on closest edge
					if selected_blade_edge_headings[0][0] < selected_blade_edge_headings[1][0]:
						selected_blade_edge_headings = selected_blade_edge_headings[0:1]
					else:
						selected_blade_edge_headings = selected_blade_edge_headings[1:2]
					return self.FindHeading(frame, selected_blade_edge_headings, rho_step_distance, default_rho_step_distance_diag_perc, rho_min_diag_perc, flip_heading)
			else:
				sign 		= 1.0
				flip_rot 	= 0.0
				if not(self.VerifyEdgeQuadrant(selected_blade_edge_headings[0])):
					sign  		= -1.0 	# Everything must be flipped if the edge is in the wrong quadrant
					flip_rot 	= np.pi # Edge is in the wrong quadrant, so find rho_f in the correct direction.

				edge_rho, edge_theta, max_line, hor_or_vert = selected_blade_edge_headings[0]
				I_cos 		= I_w*np.cos(edge_theta + flip_rot)
				I_sin 		= I_l*np.sin(edge_theta + flip_rot)
				diag_rot_l 	= 0.5*np.sqrt(I_cos*I_cos + I_sin*I_sin)
				rho_f 		= diag_rot_l - sign*edge_rho
				rho_min 	= diag_rot_l*rho_min_diag_perc
				
				if flip_rot != 0.0: # Flip situations if the edge is in wrong quadrant
					rho_f_cp 	= rho_f
					rho_f 		= rho_min
					rho_min 	= rho_f_cp

				if rho_min <= rho_f:
					W_f 	= 2 - rho_min/rho_f
				else:
					W_f 	= rho_f/rho_min

				if max_line: # Mirrored direction depending on the side of the blade to follow.
					sign *= -1.0

				if not(self.GetMovingTowardsTipOrRoot()): # True for moving towards tip.
					sign *= -1.0
				if flip_heading:
					sign *= -1.0

				omega = edge_theta + sign*(np.pi/2)*W_f

			selected_heading = (rho_step_distance, omega)
		return selected_heading #else

	def GetTipDetected(self, frame, selected_tip_heading, diag_percent=1/3.0):
		'''
		 @brief Detect if tip is detected within a given percent of the diagonal distance of the frame.

		 @param frame
		 @param selected_tip_heading
		 @param diag_percent (default=25%)

		 @return tip_detected, selected_tip_heading (Return: tip_detected = True/False, selected_tip_heading = (rho, theta))
		'''
		tip_detected = False
		if not(selected_tip_heading[0] == None):
			width, height = GetShape(frame)
			tip_threshold_distance = math.sqrt(width*width + height*height)*diag_percent
			if selected_tip_heading[0] <= tip_threshold_distance:
				tip_detected = True
		return tip_detected, selected_tip_heading

	def GetRootDetected(self, frame):
		'''
		 @brief Detect if root is detected using external range sensor (radar or similar distance/obstacle detector).

		 @param frame

		 @return root_detected, selected_root_heading (Return: root_detected = True/False, selected_root_heading = (rho, theta))
		'''
		#############

		# TODO 

		############
		root_detected = False
		selected_root_heading = (0.0,0.0)
		return root_detected, selected_root_heading

	def GetHorizontalOrVerticalEdgeFollowing(self, selected_hor_edge_heading, selected_vert_edge_heading, possible_hor_edge_headings, possible_vert_edge_headings):
		'''
		 @brief Get the edge heading to follow the vertical or horizontal lines until the end.
		 	selected_tip_heading will be (None, None) until the end of blade edge is detected.
		 	Reset vertical or horizontal edge following by using ResetHeading().

		 @param selected_hor_edge_heading
		 @param selected_vert_edge_heading
		 @param possible_hor_edge_headings
		 @param possible_vert_edge_headings

		 @return selected_blade_edge_headings, selected_tip_heading
		 	(Return
		 		selected_blade_edge_headings 	= List of headings towards the leading/trailing blade edge (Will be empty list ([]) if no blade edges where detected)
		 		selected_tip_heading 			= Heading towards the tip blade edge (Will be (None, None) if no tip where detected)
		 		)
		'''
		if self.__following_horizontal_edges == None:
			if selected_hor_edge_heading[0] == None:
				self.__following_horizontal_edges = False 	# Follow vertical edges until end
			elif selected_vert_edge_heading[0] == None:
				self.__following_horizontal_edges = True 	# Follow horizontal edges until end
			else:	# Cannot decide, follow the side that is closest
				if selected_hor_edge_heading[0] < selected_vert_edge_heading[0]:
					self.__following_horizontal_edges = True
				else:
					self.__following_horizontal_edges = False

		if self.__following_horizontal_edges: #Select to follow horizontal or vertical edge 
			selected_blade_edge_headings 	= possible_hor_edge_headings
			selected_tip_heading 			= selected_vert_edge_heading
		else:
			selected_blade_edge_headings 	= possible_vert_edge_headings
			selected_tip_heading 			= selected_hor_edge_heading
		return selected_blade_edge_headings, selected_tip_heading

	def HandleErrorNoEdgeHeadingDetected(self):
		'''
		 @brief Handle exception of no edge heading detected.
		 	Raises warning wit error_msg_no_backup_heading_available message.
		 	Will return backup heading, based in previous headings.
		 	Will raise error_msg_no_backup_heading_available exception if current_heading == None (not available)

		 @return selected_heading (rho, theta)
		'''
		blade_heading, tip_heading = self.GetCurrentHeading()
		if blade_heading[0] == None:
			raise DroneVisionError('error_msg_no_backup_heading_available')
		return blade_heading, tip_heading





