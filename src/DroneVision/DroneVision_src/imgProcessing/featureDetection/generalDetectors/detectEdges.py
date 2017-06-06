'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

import cv2
import numpy as np
from Settings.Exceptions import DroneVisionError
from detectLines import HoughLineEdgePoints, DrawHoughLine
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape

def FiltrateEdgePoints(x_list, y_list, std_threshold=10):
	'''
	 @brief Filtrate points that deviate outside of standard deviation + std_threshold.

	 @x_list (x positions of points)
	 @y_list (y positions of points)
	 @std_threshold (standard deviation additional threshold)

	 @param x_list, y_list (Filtrated)
	'''
	# Consider only edge points within the standard deviation.
	if np.std(x_list) > std_threshold and np.std(y_list) > std_threshold:
		valid_indices_x = np.argwhere(np.abs(np.array(x_list)-np.mean(x_list)) <= np.std(x_list) + std_threshold).T[0]
		valid_indices_y = np.argwhere(np.abs(np.array(y_list)-np.mean(y_list)) <= np.std(y_list) + std_threshold).T[0]
		x_m_y 			= np.in1d(valid_indices_x, valid_indices_y)
		y_m_x 			= np.in1d(valid_indices_y, valid_indices_x)
		x_m_y_in 		= np.argwhere(x_m_y == True).T[0]
		y_m_x_in 		= np.argwhere(y_m_x == True).T[0]
		x_m_y_temp		= np.zeros(len(x_m_y_in) + len(y_m_x_in))
		len_xy 			= len(x_m_y_in)
		for i in range(len(x_m_y_temp)):
			if i < len_xy:
				x_m_y_temp[i] = valid_indices_x[x_m_y_in[i]]
			else:
				x_m_y_temp[i] = valid_indices_y[y_m_x_in[i-len_xy]]
		valid_indices = np.unique(x_m_y_temp).astype(int)

		valid_x 	  = [None]*len(valid_indices)
		valid_y 	  = [None]*len(valid_indices)
		for i in range(len(valid_indices)):
			valid_x[i] 	= x_list[valid_indices[i]]
			valid_y[i] 	= y_list[valid_indices[i]]
		return valid_x, valid_y
	else:
		return x_list, y_list

def DetectBoundaryEdges(origin_frame, bounded_lines, max_min_lines, scale_threshold=1.0, line_perc=2.0/3, filtrate_edge_points=True, draw=False, print_hough_positions=False):
	'''
	 @brief Detect all boundary edges, and compute the corresponding line.
	 	Step 1:
	 		For each line in (max_hor_line, min_hor_line, max_vert_line, min_vert_line):
	 			detect if boundary indicates an open space towards the frame edges, indicating end of the blade.
	 	Step 2:
	 		If any max/min boundary line indicates end of blade (detected end of blade region), then 
	 		find all blade edge points by using DetectLineEdge, starting from each horizontal/vertical max/min point and towards the frame end point.
	 	Step 3:
	 		Use hough transform to derive the longest and most significant line for each detected blade edge.

	 @param origin_frame (Original grayscale frame without laser points)
	 @param bounded_lines (bounded lines from FindLineLimits)
	 @param max_min_lines (List of max min horizontal and vertical lines, in this order:
		 - max_hor_line (max_hor_line from FindLineLimits)
		 - min_hor_line (min_hor_line from FindLineLimits)
		 - max_vert_line (max_vert_line from FindLineLimits)
		 - min_vert_line (min_vert_lines from FindLineLimits))
	 @param scale_threshold (scaling threshold variable for considering one of the limit lines as a possible blade edge - 
	 	set between 0 -> 1 (float), where 1 = 100 percent of with or height frame size. 
	 	100 percent will mean that all boundary lines are considered as blade edges.)
	 @param line_perc (Set how many percent of the line to use for detecting the edge. Float between 0 -> 1. (default=2.0/3))
	 @param filtrate_edge_points (Filtrate detected edge points that deviate outside of standard deviation.)
	 @param draw (draw hough lines and points (used during testing))
	 @param print_hough_positions (print hough line positions (rho, theta) (default=False))


	 @return edgel_frame, hough_lines, edge_points 
	 	(Return:
	 		edgel_frame Edge map
	 		hough_lines = [max_hor_hough_line, min_hor_hough_line, max_vert_hough_line, min_vert_hough_line])
					Each index as line of (rho, theta)
	'''
	width, height 				= GetShape(origin_frame)
	edgel_frame 				= Canny(origin_frame)
	hor_edge_region_threshold 	= width*scale_threshold
	vert_edge_region_threshold 	= height*scale_threshold

	max_hor_line 	= max_min_lines[0]
	min_hor_line 	= max_min_lines[1]
	max_vert_line 	= max_min_lines[2]
	min_vert_line 	= max_min_lines[3]

	max_hor_line_edge_points 	= [[],[]] # [x],[y]
	max_hor_hough_line 			= (None, None)
	average_line_x_pos 			= max_hor_line[0][1] + (max_hor_line[1][1] - max_hor_line[0][1])/2
	if average_line_x_pos < hor_edge_region_threshold: # Edge detected on the bottom
		for vert_line in bounded_lines[1]: # Check all vertical lines (max points)
			len_line 	= int(round((vert_line[1][1] - vert_line[0][1])*line_perc))
			max_point 	= vert_line[1]
			y_edge_ind 	= DetectLineEdge(edgel_frame, False, max_point[0], max_point[1], width-1)
			max_hor_line_edge_points[0].append(max_point[0])
			max_hor_line_edge_points[1].append(y_edge_ind)
		if filtrate_edge_points:
			max_hor_line_edge_points[0], max_hor_line_edge_points[1] = FiltrateEdgePoints(max_hor_line_edge_points[0], max_hor_line_edge_points[1])
		max_hor_hough_line = HoughLineEdgePoints(edgel_frame, max_hor_line_edge_points, False)

	min_hor_line_edge_points 	= [[],[]] # [x],[y]
	min_hor_hough_line 			= (None, None)
	average_line_x_pos 			= min_hor_line[0][1] + (min_hor_line[1][1] - min_hor_line[0][1])/2
	if average_line_x_pos > width - hor_edge_region_threshold: # Edge detected on top
		for vert_line in bounded_lines[1]: # Check all vertical lines (min points)
			len_line 	= int(round((vert_line[1][1] - vert_line[0][1])*line_perc))
			min_point 	= vert_line[0]
			y_edge_ind 	= DetectLineEdge(edgel_frame, False, min_point[0], min_point[1], 0)
			min_hor_line_edge_points[0].append(min_point[0])
			min_hor_line_edge_points[1].append(y_edge_ind)
		if filtrate_edge_points:
			min_hor_line_edge_points[0], min_hor_line_edge_points[1] = FiltrateEdgePoints(min_hor_line_edge_points[0], min_hor_line_edge_points[1])
		min_hor_hough_line = HoughLineEdgePoints(edgel_frame, min_hor_line_edge_points, False)

	max_vert_line_edge_points	= [[],[]] # [x],[y]
	max_vert_hough_line 		= (None, None)
	average_line_y_pos			= max_vert_line[0][0] + (max_vert_line[1][0] - max_vert_line[0][0])/2
	if average_line_y_pos < vert_edge_region_threshold: # Edge detected to the right
		for hor_line in bounded_lines[0]: # Check all horizontal lines (max points)
			len_line 	= int(round((hor_line[1][0] - hor_line[0][0])*line_perc))
			max_point 	= hor_line[1]
			x_edge_ind 	= DetectLineEdge(edgel_frame, True, max_point[1], max_point[0], height-1)
			max_vert_line_edge_points[0].append(x_edge_ind)
			max_vert_line_edge_points[1].append(max_point[1])
		if filtrate_edge_points:
			max_vert_line_edge_points[1], max_vert_line_edge_points[0] = FiltrateEdgePoints(max_vert_line_edge_points[1], max_vert_line_edge_points[0])
		max_vert_hough_line = HoughLineEdgePoints(edgel_frame, max_vert_line_edge_points, True)

	min_vert_line_edge_points 	= [[],[]] # [x],[y]
	min_vert_hough_line 		= (None, None)
	average_line_y_pos 			= min_vert_line[0][0] + (min_vert_line[1][0] - min_vert_line[0][0])/2
	if average_line_y_pos > height - vert_edge_region_threshold: # Edge detected to the left
		for hor_line in bounded_lines[0]: # Check all horizontal lines (min points)
			len_line 	= int(round((hor_line[1][0] - hor_line[0][0])*line_perc))
			min_point 	= hor_line[0]
			x_edge_ind 	= DetectLineEdge(edgel_frame, True, min_point[1], min_point[0], 0)
			min_vert_line_edge_points[0].append(x_edge_ind)
			min_vert_line_edge_points[1].append(min_point[1])
		if filtrate_edge_points:
			min_vert_line_edge_points[1], min_vert_line_edge_points[0] = FiltrateEdgePoints(min_vert_line_edge_points[1], min_vert_line_edge_points[0])
		min_vert_hough_line = HoughLineEdgePoints(edgel_frame, min_vert_line_edge_points, True)

	if max_hor_hough_line[0] == None or min_hor_hough_line[0] == None or max_vert_hough_line[0] == None or min_vert_hough_line[0] == None:
		raise DroneVisionError('detect_boundary_edge_not_found_all_edge_lines')

	hough_lines = [max_hor_hough_line, min_hor_hough_line, max_vert_hough_line, min_vert_hough_line]
	if draw:
		for i in range(len(max_hor_line_edge_points[0])):
			if i == 0:
				color = (0,0,255) #DARK BLUE
				edgel_frame = DrawHoughLine(edgel_frame, max_hor_hough_line, color)
			x = max_hor_line_edge_points[0][i]
			y = max_hor_line_edge_points[1][i]
			cv2.circle(edgel_frame,(x,y), 10, color, -1)
		for i in range(len(min_hor_line_edge_points[0])):
			if i == 0:
				color = (0,255,255) #LIGHT BLUE
				edgel_frame = DrawHoughLine(edgel_frame, min_hor_hough_line, color)
			x = min_hor_line_edge_points[0][i]
			y = min_hor_line_edge_points[1][i]
			cv2.circle(edgel_frame,(x,y), 10, color, -1)
		for i in range(len(max_vert_line_edge_points[0])):
			if i == 0:
				color = (204,0,204) #PURPLE
				edgel_frame = DrawHoughLine(edgel_frame, max_vert_hough_line, color)
			x = max_vert_line_edge_points[0][i]
			y = max_vert_line_edge_points[1][i]
			cv2.circle(edgel_frame,(x,y), 10, color, -1)
		for i in range(len(min_vert_line_edge_points[0])):
			if i == 0:
				color = (0,255,0) #GREEN
				edgel_frame = DrawHoughLine(edgel_frame, min_vert_hough_line, color)
			x = min_vert_line_edge_points[0][i]
			y = min_vert_line_edge_points[1][i]
			cv2.circle(edgel_frame,(x,y), 10, color, -1)

	if print_hough_positions:
		print 'max_hor_hough_line: ', max_hor_hough_line
		print 'min_hor_hough_line: ', min_hor_hough_line
		print 'max_vert_hough_line: ', max_vert_hough_line
		print 'min_vert_hough_line: ', min_vert_hough_line

	# hough_lines = [max_hor_hough_line, min_hor_hough_line, max_vert_hough_line, min_vert_hough_line]
	return edgel_frame, hough_lines 

def DetectLineEdge(edgel_frame, hor_line, static_index, start_index, max_index, blocksize=3):
	'''
	 @brief Detect the first significant edgel on the line (greedy)
	 	The search can be both in negative and positive direction, along horizontal or vertical lines.
	 		Negative direction if max_index < start_index
	 		Positive direction if start_index > max_index

	 @param edgel_frame (edgel map)
	 @param hor_line (horizontal line = True, vertical line = False)
	 @param static_index (Static index along horizontal or vertical axis)
	 @param start_index (Start index on line)
	 @param max_index (Max index on line)
	 @param blocksize (Blocksize for edgel search.)

	 @return highest_delta_index
	'''
	blocksize_radi 		= blocksize//2
	n_rows, n_cols 		= edgel_frame.shape
	start_offset 		= static_index - blocksize_radi
	end_offset 			= static_index + blocksize_radi+1
	highest_delta_index = max_index
	
	if start_offset < 0: # Assert start offset
		start_offset = 0

	if hor_line:
		if end_offset > n_rows: # Assert end offset along rows
			end_offset = n_rows

		# Search along all columns
		if max_index < start_index:
			possible_edgels = np.argwhere(edgel_frame[start_offset:end_offset, max_index:start_index+1] > 0)[:,1]
		else:
			possible_edgels = np.argwhere(edgel_frame[start_offset:end_offset, start_index:max_index+1] > 0)[:,1]
	
	else:
		if end_offset > n_cols: # Assert end offset along columns
			end_offset = n_cols
		
		# Search along all rows
		if max_index < start_index:
			possible_edgels = np.argwhere(edgel_frame[max_index:start_index+1, start_offset:end_offset] > 0)[:,0]
		else:
			possible_edgels = np.argwhere(edgel_frame[start_index:max_index+1, start_offset:end_offset] > 0)[:,0]

	if possible_edgels.size > 0: # Check if any edgels were detected
		possible_edgels.sort()
		if max_index < start_index:
			highest_delta_index = max_index + possible_edgels[-1:][0] # Searching in negative direction - get the latest and add the max index (first in negative direction)
		else:
			highest_delta_index = start_index + possible_edgels[0] # Searching in positive direction - get the first and add the start index
	
	return highest_delta_index

def Canny(frame, lower_threshold=30, upper_threshold=45, apertureSize=3, L2gradient=True):
	'''
	 @brief Compute edgel map using canny method (using Sobel method).
	 	- args in cv2 method:
	 		1. src - Source frame
	 		2. lower_threshold - lower treshold for the hystereis procedure.
	 		3. upper_threshold - upper threshold for the hystereis procedure
	 		4. apertureSize - aperture size for the Sobel operator
	 		5. L2gradient -  flag, indicating whether a more accurate L2 norm =sqrt((dI/dx)^2+(dI/dy)^2) 
	 			should be used to calculate the image gradient magnitude ( L2gradient=true ), 
	 			or whether the default L1 norm =|dI/dx|+|dI/dy| is enough ( L2gradient=false ).
	 			
	 @param frame (grayscale)

	 @return edgel map
	'''
	return cv2.Canny(frame, lower_threshold, upper_threshold, apertureSize=apertureSize, L2gradient=L2gradient)