'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

import cv2, math
import numpy as np
from Settings.Exceptions import DroneVisionError
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape, CheckColor

'''
 @brief EdgeHeading class. Compute heading direction (rho, theta) towards the most significant edge.
 	- rho 	= Distance to edge (in image frame)
 	- theta = heading angle towards the edge (in image frame)
 	The heading (rho, theta) is in regard to the image center.
'''
class EdgeHeading():
	def __init__(self):
		'''CONSTRUCTOR'''
		self.__current_hor_edge_heading 	= (None,None) #(rho, theta)
		self.__current_vert_edge_heading 	= (None,None) #(rho, theta)

	def GetCurrentEdgeHeadings(self):
		'''
		 @brief Get current edge headings (rho, theta) for horizontal and vertical lines
		 	Returns (None, None) if the current edge heading has not been set.

		 @return (rho, theta)
		'''
		return self.__current_hor_edge_heading, self.__current_vert_edge_heading

	def SetCurrentEdgeHeadings(self, hor_edge_heading, vert_edge_heading):
		'''
		 @brief Set current edge headings

		 @param hor_edge_heading (rho, theta)
		 @param vert_edge_heading (rho, theta)
		'''
		self.__current_hor_edge_heading 	= hor_edge_heading
		self.__current_vert_edge_heading 	= vert_edge_heading

	def ComputeEdgeHeading(self, frame, boundary_hough_lines, draw=False):
		'''
		 @brief Compute heading to the most significant edge based on the boundary hough lines.
		 	Raises self.error_msg_no_heading_angle_detected if no heading angles are detected.

		 	All edge headings are measured from image center, meaning returned rho and theta are in regard to the image center.
		 	The edge heading is perpendicular to the corresponding edge.
		 	Set draw=True to return a colored frame showing the edge headings
		 	The function considers the current edge heading to find the most probable next edge heading.
		 	Use SetCurrentEdgeHeading() to update the current edge heading, or GetCurrentEdgeHeading() to get it.

		 	rho is in range between <0, frame_size>
		 	theta is in range between <0, 2pi>

		 @param frame (undistorted frame)
		 @param boundary_hough_lines (See PointDetection.GetBoundaryHoughLines)
		 	boundary_hough_lines = [max_hor_hough_line, min_hor_hough_line, max_vert_hough_line, min_vert_hough_line],
		 	with hough line as (rho, theta)
		 @param draw (default=False)

		 if draw == True:
		 	@return selected_hor_edge_heading, selected_vert_edge_heading, possible_hor_edge_headings, possible_vert_edge_headings, frame (frame = updated color frame)
		 else:
		 	@return selected_hor_edge_heading, selected_vert_edge_heading, possible_hor_edge_headings, possible_vert_edge_headings 
		 	(Return
		 		selected_hor_edge_heading 	= closest horizontal edge heading to previous horizontal edge heading ((None, None, None, None), if there were no detected horizontal edges)
		 		selected_vert_edge_heading 	= closest vertical edge heading to previous vertical edge heading ((None, None, None, None), if there were no detected vertical edges)
		 		possible_hor_edge_headings 	= [(rho, theta, max/min, hor/vert),..] in horizontal direction
		 		possible_vert_edge_headings = [(rho, theta, max/min, hor/vert),..] in vertical direction
		 		(rho = distance to edge, theta = heading direction in radian degrees., max/min = True/False according to if it is the max or min boundary line, hor/vert = True/False - True for horizontal, False for vertical)
		'''
		possible_hor_edge_headings 	= []
		possible_vert_edge_headings = []

		# for drawing: max_hor = DARK BLUE, min_hor = LIGHT BLUE, max_vert = PURPLE, min_vert = GREEN
		colors = [(0,0,255), (0,255,255), (204,0,204), (0,255,0)]
		n_boundary_hough_lines = len(boundary_hough_lines)
		for i in range(n_boundary_hough_lines):
			if i % 2 == 0:
				max_line = True
			else:
				max_line = False

			proc_returns = self.ProcessBoundaryHoughLine(frame, boundary_hough_lines[i], draw=draw, color=colors[i])
			edge_line, rho, theta = proc_returns[:3]
			if draw:
				frame = proc_returns[3]
			if edge_line:
				if i < n_boundary_hough_lines/2: # horizontal lines
					possible_hor_edge_headings.append((rho, theta, max_line, True))
				else: # vertical lines
					possible_vert_edge_headings.append((rho, theta, max_line, False))

		if (len(possible_hor_edge_headings) + len(possible_vert_edge_headings)) == 0:
			raise DroneVisionError('error_msg_no_heading_angle_detected')

		current_hor_edge_heading, current_vert_edge_heading = self.GetCurrentEdgeHeadings()
		selected_hor_edge_heading 	= self.FindMostSignificantEdgeHeading(possible_hor_edge_headings, current_hor_edge_heading)
		selected_vert_edge_heading 	= self.FindMostSignificantEdgeHeading(possible_vert_edge_headings, current_vert_edge_heading)

		if draw:
			return selected_hor_edge_heading, selected_vert_edge_heading, possible_hor_edge_headings, possible_vert_edge_headings, frame
		return selected_hor_edge_heading, selected_vert_edge_heading, possible_hor_edge_headings, possible_vert_edge_headings #else

	def FindMostSignificantEdgeHeading(self, possible_edge_headings, ref_edge_heading):
		'''
		 @brief Find the most desirable edge heading angle out of the detected edge heading angles.
		 	Considers the referance edge heading to find the most probable next edge heading.

		 @param possible_edge_headings
		 @param ref_edge_heading

		 @return (rho, theta, max/min, hor/vert) (selected edge_heading - Returns (None, None, None) if there are no possible edge headings)
		'''
		edge_heading = (None, None, None)
		if len(possible_edge_headings) > 0:
			closest_edge_heading_index = 0
			if ref_edge_heading[0] == None: # At start, choose the closest edge
				for i in range(len(possible_edge_headings)):
					if possible_edge_headings[i][0] < possible_edge_headings[closest_edge_heading_index][0]:
						closest_edge_heading_index = i
			else: # Choose the edge that is closest to the previously detected edge
				min_delta_rho = np.abs(possible_edge_headings[closest_edge_heading_index][0] - ref_edge_heading[0])
				for i in range(len(possible_edge_headings)):
					temp_min_delta_rho = np.abs(possible_edge_headings[i][0] - ref_edge_heading[0])
					if temp_min_delta_rho < min_delta_rho:
						min_delta_rho = temp_min_delta_rho
						closest_edge_heading_index = i
			edge_heading = possible_edge_headings[closest_edge_heading_index]
		return edge_heading #(rho, theta, max/min)
			
	def ProcessBoundaryHoughLine(self, frame, boundary_hough_line, bound_thres=10, draw=False, color=(255,255,0)):
		'''
		 @brief Process boundary hough line.
		 	Check if the hough line indicates the blade edge, 
		 	and compute the heading angle according to the edge.

		 @param frame (original undistorted frame)
		 @param boundary_hough_line (Touple as (rho, theta))
		 @param bound_thres (Threshold in pixels for the line to be detected as a blade edge)
		 @param draw (default=False)
		 @param color (draw color if draw=True (default=(255,255,0)))

		 if draw:
		 	@return edge_line, rho, theta, frame (color frame with circles of hough line center points)
		 else:
			 @return edge_line, rho, theta
			 	(edge_line = True/False in regard to if the line is following a blade edge
			 	rho  = distance to edge (normalized)
			 	theta = heading angle along the detected edge (normalized.)
		'''
		width, height 	= GetShape(frame)
		rho, theta 		= boundary_hough_line
		rho, theta 		= self.NormalizeEdgeHeading(frame, rho, theta)

		x_center = height/2.0 + np.cos(theta)*rho
		y_center = width/2.0 + np.sin(theta)*rho

		edge_line = False
		if (np.abs(x_center - (height-1)) > bound_thres and x_center > bound_thres) and (np.abs(y_center - (width-1)) > bound_thres and y_center > bound_thres):
			edge_line = True

		if draw:
			cv2.circle(frame,(int(round(x_center)),int(round(y_center))), 20, color, 3)
			if edge_line:
				frame = self.DrawHeading(frame, (rho, theta), color=color)

		if draw:
			return edge_line, rho, theta, frame
		return edge_line, rho, theta

	def NormalizeEdgeHeading(self, frame, rho, theta):
		'''
		 @brief Normalize the edge heading (rho, theta) according to image center.
		 	See theory about the hough transform.
		 	Short: rho is originally considered from top left image corner, 
		 		we want it from image center.
		 		theta is ranged from 0 -> pi, with rho as positive or negative,
		 		we want theta to be ranged from 0-> 2pi and rho as positive.
		 		From image center:
		 			if rho < 0:
		 				theta += pi 
		 			rho = abs(rho), where rho is distance to line from image center.

		 @param frame
		 @param rho
		 @param theta
		 @param draw (draw center hough line points. (default=False))
		 @param color (draw color if draw=True (default=(255,255,0)))

		 @return rho, theta (normalized)
		'''
		if rho < 0:
			theta 	+= np.pi
			rho 	*= -1.0
		width, height 	= GetShape(frame)
		x0, y0 			= height/2.0, width/2.0
		x1, y1, x2, y2 	= self.GetHoughLineSegment(frame, rho, theta)
		theta 			= self.GetAngleOfLineBetweenTwoPoints(x1, y1, x2, y2, x0, y0)
		rho 			= self.GetDistanceToLine(x1, y1, x2, y2, x0, y0)
		return rho, theta

	def GetDistanceToLine(self, x1, y1, x2, y2, x0, y0):
		'''
		 @brief Get distance from a point (x0,y0) to the line defined by two points ((x1,y1), (x2,y2)).
		 	See theory about line defined by two points: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line

		 @param x1
		 @param y1
		 @param x2
		 @param y2
		 @param x0
		 @param y0

		 @return rho (absolute distance to line from (x0,y0))
		'''
		rho = 0
		a = y2-y1
		b = -(x2-x1)
		c = x2*y1 - y2*x1
		if not(a == 0 and b == 0):
			rho = np.abs(a*x0 + b*y0 + c)/np.sqrt(a*a + b*b)
		return rho

	def GetClosestPointOnLine(self, x1, y1, x2, y2, x0, y0):
		'''
		 @brief Get closest point on line from point (x0,y0)
		 	See theory: http://paulbourke.net/geometry/pointlineplane/

		 @param x1
		 @param y1
		 @param x2
		 @param y2
		 @param x0
		 @param y0

		 @return (x,y) (closest point on line)
		'''
		vec_x = x2 - x1 
		vec_y = y2 - y1
		u = ((x0 - x1)*vec_x + (y0 - y1)*vec_y)/np.abs(vec_x*vec_x - vec_y*vec_y)
		x = x1 + u*vec_x
		y = y1 + u*vec_y
		return (x,y)

	def GetAngleOfLineBetweenTwoPoints(self, x1, y1, x2, y2, x0, y0):
		'''
		 @brief Get angle of line defined by two points ((x1,y1), (x2,y2)) from point (x0,y0).

		 @param x1
		 @param y1
		 @param x2
		 @param y2
		 @param x0
		 @param y0

		 @return theta (angle to line)
		'''
		c_x, c_y 	= self.GetClosestPointOnLine(x1, y1, x2, y2, x0, y0)
		c_theta 	= np.arctan2(c_y - y0, c_x - x0) # Angle of perpendicular line from center to closest point on line (not accurate).
		theta 		= np.arctan2(y2 - y1, x2 - x1) - np.pi/2 # Perpendicular angle + slope angle creates a perpendicular triangle
		n_theta 	= theta % (np.pi*2) # Normalized angles for finding correct quadrant
		rot_n_theta = (theta + np.pi) % (np.pi*2)
		n_c_theta 	= c_theta % (np.pi*2)
		if np.abs(n_theta - n_c_theta) >= np.pi/2 and not(np.abs(rot_n_theta - n_c_theta) > np.pi/2): # Assert that the hough angle is in the right quadrant. (Angle to the closest point is not accurate, but is in the right quadrant.)
			theta += np.pi
		return theta

	def GetHoughLineSegment(self, frame, rho, theta):
		'''
		 @brief Get start and end position of hough line.
		 	The hough line is normally positioned within the frame, 
			but for theta > pi/2 will give positiones outside of the frame.
			The algorithm uses sin/cos/tan to compute the correct start and end position
			of the hough transform inside the frame. 
		 
		 @param frame
		 @param rho
		 @param theta

		 @return x_start, y_start, x_end, y_end
		'''
		width, height = GetShape(frame)
		size = math.sqrt(math.pow(width, 2.0) + math.pow(height, 2.0))
		x_start = np.cos(theta)*rho - size*(-np.sin(theta))
		y_start = np.sin(theta)*rho - size*np.cos(theta)
		x_end = np.cos(theta)*rho + size*(-np.sin(theta))
		y_end = np.sin(theta)*rho + size*np.cos(theta)
		return x_start, y_start, x_end, y_end

	def VerifyEdgeQuadrant(self, edge_heading):
		'''
		 @brief Verify that the edge heading is in the correct half quadrant given by the max/min flag.

		 @param edge_heading (tuple as (rho, theta, max/min, hor/vert))

		 @return True/False
		'''
		edge_rho, edge_theta, max_line, hor_or_vert = edge_heading
		correct 	= True
		edge_theta 	= np.sign(edge_theta)*(edge_theta % (np.pi*2))
		if edge_theta < 0:
			edge_theta += np.pi*2
		if hor_or_vert: # Horizontal line
			if max_line:
				if edge_theta > np.pi and edge_theta < np.pi*2: # Max line is in wrong quadrant
					correct = False
			else:
				if not(edge_theta >= np.pi and edge_theta <= np.pi*2): # Min line is in wrong quadrant
					correct = False
		else: # Vertical line
			if max_line:
				if edge_theta > np.pi/2 and edge_theta < np.pi*(3/2.0): # Max line is in wrong quadrant
					correct = False
			else:
				if not(edge_theta >= np.pi/2 and edge_theta <= np.pi*(3/2.0)): # Min line is in wrong quadrant
					correct = False
		return correct

	def DrawHeading(self, frame, heading, color=(255,255,0), line_thick=5, draw_arrow=True):
		'''
		 @brief Draw heading on frame.
		 	Heading is measured from image center.

		 @param frame 
		 @param heading (rho, theta)
		 @param color (default=(255,255,0) (YELLOW))
		 @param line_thick (default=5)
		 @param draw line as arrow

		 @return frame (color)
		'''
		rho, theta = heading[:2]
		if not(rho == None or theta == None):
			frame = CheckColor(frame)
			I_l, I_w = GetShape(frame)
			x0 	= I_w/2.0
			y0	= I_l/2.0
			x1 	= int(round(x0))
			y1 	= int(round(y0))
			x2 	= int(round(x0 + np.cos(theta)*rho))
			y2 	= int(round(y0 + np.sin(theta)*rho))
			if draw_arrow:
				cv2.arrowedLine(frame, (x1,y1), (x2,y2), color, line_thick, tipLength=0.1) 
			else:
				cv2.line(frame, (x1,y1), (x2,y2), color, line_thick) 
		return frame

	def DrawErrorArrow(self, frame, heading, draw_min_rho=True, rho_min_diag_perc=1/4.0, color=(255,0,0), min_rho_color=(153,0,0), line_thick=5, draw_arrow=True):
		'''
		 @brief Draw error line/arrow from point on line to image edge.

		 @param frame 
		 @param heading (rho, theta, max/min, hor/vert)
		 @param draw_min_rho (True/False)
		 @param rho_min_diag_perc (default=1/4.0)
		 @param color (default=(255,0,0) (RED))
		 @param min_rho_color (default=(153,0,0) (DARK RED))
		 @param line_thick (default=5)
		 @param draw line as arrow

		 @return frame (color)
		'''
		rho, theta, max_line, hor_or_vert = heading[:4]
		if not(rho == None or theta == None):
			frame = CheckColor(frame)
			I_l, I_w = GetShape(frame)
			x1 	= int(round(I_w/2.0 + np.cos(theta)*rho))
			y1 	= int(round(I_l/2.0 + np.sin(theta)*rho))

			flip_rot = 0.0
			if not(self.VerifyEdgeQuadrant(heading)):
				flip_rot = np.pi # Edge is in the wrong quadrant, so find rho_f in the correct direction.
			I_cos 		= I_w*np.cos(theta + flip_rot)
			I_sin 		= I_l*np.sin(theta + flip_rot)
			diag_rot_l 	= 0.5*np.sqrt(I_cos*I_cos + I_sin*I_sin)
			x2 = int(round(I_w/2.0 + np.cos(theta + flip_rot)*diag_rot_l))
			y2 = int(round(I_l/2.0 + np.sin(theta + flip_rot)*diag_rot_l))
			x_rho_min = int(round(I_w/2.0 + np.cos(theta + flip_rot)*diag_rot_l*(1.0 - rho_min_diag_perc)))
			y_rho_min = int(round(I_l/2.0 + np.sin(theta + flip_rot)*diag_rot_l*(1.0 - rho_min_diag_perc)))
			if draw_arrow:
				cv2.arrowedLine(frame, (x1,y1), (x2,y2), color, line_thick, tipLength=0.1) 
				if draw_min_rho:
					cv2.arrowedLine(frame, (x1,y1), (x_rho_min,y_rho_min), min_rho_color, line_thick, tipLength=0.1) 
			else:
				cv2.line(frame, (x1,y1), (x2,y2), color, line_thick) 
				if draw_min_rho:
					cv2.line(frame, (x1,y1), (x_rho_min,y_rho_min), min_rho_color, line_thick)
		return frame