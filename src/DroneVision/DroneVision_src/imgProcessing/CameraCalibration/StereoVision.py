'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''
import cv2
import warnings
import numpy as np
from StereoCalibration import StereoCalibration
from Settings.Exceptions import DroneVisionError

'''
 @brief Class for calibrating the stereo vision system.

 @param me_master (True if this instance is master, False for slave)
 @param settings_inst (CALIB settings)
 @param reset (True/False)
 @param plot_figure (optional plot figure (default=None))
'''
class StereoVision(StereoCalibration):
	def __init__(self, me_master, settings_inst, reset, plot_figure=None):
		'''CONSTRUCTOR'''
		StereoCalibration.__init__(self, me_master, settings_inst, reset, plot_figure=plot_figure)

	def GetMatchingPoints(self, left_keypoints, right_keypoints, matches, return_as_numpy=False):
		'''
		 @brief Get matching points from left and right keypoints

		 @param left_keypoints (list of cv2 keypoints)
		 @param right_keypoints (list of cv2 keypoints)
		 @param matches	(list of cv2 matches)
		 @param return_as_numpy (return left_pts, right_pts as numpy arrays. (default=False))

		 @return left_pts, right_pts (Returned as numpy array with 2xN points (return_as_numpy=True), or list of matching keypoints (return_as_numpy=False).)
		'''
		left_pts 	= []
		right_pts 	= []
		for match in matches:
			if return_as_numpy:
				left_pts.append(left_keypoints[match.trainIdx].pt)
				right_pts.append(right_keypoints[match.queryIdx].pt) 
			else:
				left_pts.append(left_keypoints[match.trainIdx])
				right_pts.append(right_keypoints[match.queryIdx])
		if return_as_numpy:
			left_pts 	= np.int32(left_pts).T
			right_pts 	= np.int32(right_pts).T
		return left_pts, right_pts

	def Compute3DPointsFromDisparity(self, left_keypoints, right_keypoints, matches):
		'''
		 @brief Compute 3D world coordinates by computing the disparity between matching points

		 @param left_keypoints
		 @param right_keypoints
		 @param matches

		 @return points3D (points3D = list of 3xN array, as [x, y, z].T world coordinates)
		'''
		left_pts, right_pts = self.GetMatchingPoints(left_keypoints, right_keypoints, matches)
		points3D 			= []
		f_x, f_y, f_z 		= self.GetPixelFocalLength()
		baseline_p 			= self.GetPixelBaseline()
		baseline 			= self.GetBaseline()
		for i in range(len(left_pts)):
			disparity = left_pts[i].pt[0] - right_pts[i].pt[0]
			if disparity != 0.0:
				world_z 		= f_z*baseline_p/disparity 		# Z_w = f_z*T/d
				world_x 		= left_pts[i].pt[0]*world_z/f_x	# X_w/x_p = Z_w/f_x -> X_w = x_p*Z_w/f_x
				world_y 		= left_pts[i].pt[1]*world_z/f_y	# Y_w/y_p = Z_w/f_y -> Y_w = y_p*Z_w/f_y
				point3D 		= np.array([[world_x],[world_y],[world_z]])
				point3D 		*= baseline/baseline_p # Transform from pixel units to metric (mm) units. Use the relation between baseline in pixels with baseline in metric units.
				if point3D[2,0] >= 0: # Remove negative depth points (should be impossible)
					points3D.append(point3D)
			else:
				warnings.simplefilter('always')
				warnings.warn('Disparity equals ZERO, cannot reconstruct 3D point.', UserWarning)
				warnings.simplefilter('default')
		return points3D

	def TriangulatePoints(self, left_keypoints, right_keypoints, matches, opencv_triangulation=True, iterative_HZ_triangulation=True):
		'''
		 @brief Triangulate matched feature points
		 	First step is to rectify the images, 
		 	next is to compute the intersection points using triangulation.

		 @param left_keypoints
		 @param right_keypoints
		 @param matches
		 @param opencv_triangulation (Use the opencv implememnted triangulation method (Overrides the Harley & Zisserman method) (default=True))
		 @param iterative_HZ_triangulation (Use iterative Harley & Zisserman triangulation method (Default=True))
	
		 @return points3D (points3D = list of 3xN array, as [x, y, z].T world coordinates)
		'''
		self.AssertStereoCalibrated()
		P1, P2 				= self.GetProjectionMatrices()
		left_pts, right_pts = self.GetMatchingPoints(left_keypoints, right_keypoints, matches)
		points3D 			= []
		for i in range(len(left_pts)):
			if opencv_triangulation:
				point4D = self.LinearSVDTriangulation(left_pts[i], P1, right_pts[i], P2)
			else:
				if iterative_HZ_triangulation:
					point4D = self.IterativeLinearLSTriangulation(left_pts[i], P1, right_pts[i], P2)
				else:
					point4D = self.LinearLSTriangulation(left_pts[i], P1, right_pts[i], P2)
			point3D, success = self.Convert4DPixelTo3DWorldCoordinates(point4D)
			if point3D[2,0] >= 0 and success: # Remove negative depth points (should be impossible)
				points3D.append(point3D)
		return points3D

	def Points3DToMatrix(self, points3D):
		'''
		 @brief Stack list 3D points to matrix for smoother computation
		 	Point3D at position n is found in the matrix as index points3D_m[:,n], with x,y,z as the rows.

		 @return points3D_m (3xN 3D points matrix)
		'''
		points3D_m = np.matrix(np.hstack(points3D))
		return points3D_m

	def MatrixTo3DPoints(self, points3D_m):
		'''
		 @brief Split matrix of 3D points to list of 3D points

		 @return points3D (list 3D points)
		'''
		points3D = np.hsplit(np.array(points3D_m), points3D_m.shape[1])
		return points3D

	def FiltratePoints3D(self, points3D):
		'''
		 @brief Filtrate list of 3D points by removing all points with depth outside of the standard deviation of all point depths.

		 @param points3D

		 @return points3D_f (filtrated)
		'''
		points3D_m 	= self.Points3DToMatrix(points3D)
		std_depth	= np.std(points3D_m, axis=1)[2,0]
		valid_ind 	= np.argwhere(np.abs(points3D_m[2] - np.mean(points3D_m[2])) <= std_depth).T[1]
		points3D_f 	= []
		for ind in valid_ind:
			points3D_f.append(points3D[ind])
		return points3D_f

	def Convert4DPixelTo3DWorldCoordinates(self, point4D):
		'''
		 @brief Convert 3D coordinates in pixel to world coordinates.
		 		[x, y, z].T  = [x'/w, y'/w, z'/w]
			where
				[x', y', z', w'].T =  Q * [x_p, y_p, z_p, 1].T
			and
				w = w if w != 0, or inf if w == 0
			Q = 4x4 perspective transformation (disparity to depth) matrix computed when rectifying the stereo vision system.

		 @param point4D (pixel coordinates, as [x_p, y_p, z_p, 1].T) 

		 @return point3D, success (point3D = world coordinates, success = Successfull transformation)
		'''
		Q = self.GetDisparityToDepthMatrix()
		point4D_world = np.array(np.matrix(Q)*np.matrix(point4D))
		success = False
		point3D = None
		if point4D_world[3,0] != 0: # Checking if w != 0
			success = True
			point3D = point4D_world[:3]/point4D_world[3,0]
		return point3D, success

	def LinearSVDTriangulation(self, key_point1, P1, key_point2, P2):
		'''
		 @brief Implementation of the opencv triangulation algorithm, originally Harley & Zisserman.
		 	See: https://github.com/opencv/opencv/blob/master/modules/calib3d/src/triangulate.cpp
		
		 @param key_point1 (homogenous image point (u,v,1) - cv2 key point)
		 @param P1 (camera 1 matrix (projection matrix 3x4))
		 @param key_point2 (homogenous image point in 2nd camera (u,v,1) - cv2 key point)
		 @param P2 (camera 2 matrix (projection matrix 3x4))

		 @return point4D (4x1 matrix as [x,y,z,1]^T)
		'''
		A 		= np.zeros((4,4))
		point4D = np.zeros((4,1))
		keypoints 		= [key_point1, key_point2]
		proj_matrices  	= [P1, P2]
		for i in range(2):
			x = keypoints[i].pt[0]
			y = keypoints[i].pt[1]
			for j in range(4):
				A[i*2+0,j] = x*proj_matrices[i][2,j] - proj_matrices[i][0,j]
				A[i*2+1,j] = y*proj_matrices[i][2,j] - proj_matrices[i][1,j]
		w, u, vt = cv2.SVDecomp(A)
		point4D[0,0] = vt[0,3] # X
		point4D[1,0] = vt[1,3] # Y
		point4D[2,0] = vt[2,3] # Z
		point4D[3,0] = vt[3,3] # W
		return point4D

	def LinearLSTriangulation(self, key_point1, P1, key_point2, P2, wi1=1.0, wi2=1.0):
		'''
		 @brief Triangulate points in 3D space.
		 	Harley & Zisserman triangulation.
		 	Inspired by: http://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/

		 @param key_point1 (homogenous image point (u,v,1) - cv2 key point)
		 @param P1 (camera 1 matrix (projection matrix 3x4))
		 @param key_point2 (homogenous image point in 2nd camera (u,v,1) - cv2 key point)
		 @param P2 (camera 2 matrix (projection matrix 3x4))
		 @param wi1 (weight for camera 1 - used for the iterative method)
		 @param wi2 (weight for camera 2 - used for the iterative method)

		 @return X (4x1 matrix as [x,y,z,1]^T)
		'''
		#build matrix A for homogenous equation system Ax = 0
		#assume X = (x,y,z,1), for Linear-LS method, 
		#which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1

		A = np.zeros((4,3))
		X = np.zeros((3,1))
		B = np.zeros((4,1))

		# Creating A matrice - each column depends 
		for i in range(3):
			A[0,i] = (key_point1.pt[0]*P1[2,i] - P1[0,i])/wi1
			A[1,i] = (key_point1.pt[1]*P1[2,i] - P1[1,i])/wi1
			A[2,i] = (key_point2.pt[0]*P2[2,i] - P2[0,i])/wi2
			A[3,i] = (key_point2.pt[1]*P2[2,i] - P2[1,i])/wi2

		B[0,0] = -(key_point1.pt[0]*P1[2,3] - P1[0,3])/wi1
		B[1,0] = -(key_point1.pt[1]*P1[2,3] - P1[1,3])/wi1
		B[2,0] = -(key_point2.pt[0]*P2[2,3] - P2[0,3])/wi2
		B[3,0] = -(key_point2.pt[1]*P2[2,3] - P2[1,3])/wi2

		flags = cv2.DECOMP_SVD
		retval, X = cv2.solve(A, B, flags=flags)
		if not(retval):
			raise DroneVisionError('triangulation_error_msg')
		return np.vstack([X, np.ones(1)]) #append row with 1 to create 4x1 vector

	def IterativeLinearLSTriangulation(self, key_point1, P1, key_point2, P2, EPSILON=1.0):
		'''
		 @brief Triangulate points in 3D space - Iterative solution which uses the LinearLSTriangulation function.
		 	Harley & Zisserman triangulation.
		 	Inspired by: http://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/

		 @param key_point1 (homogenous image point (u,v,1) - cv2 key point)
		 @param P1 (camera 1 matrix (projection matrix 3x4))
		 @param key_point2 (homogenous image point in 2nd camera (u,v,1) - cv2 key point)
		 @param P2 (camera 2 matrix (projection matrix 3x4))
		 @param EPSILON (Breaking point value)

		 @return X (4x1 matrix as [x,y,z,1]^T)
		'''
		wi1, wi2 	= 1.0, 1.0
		for i in range(10): #Hartley suggests 10 iterations at most
			X = self.LinearLSTriangulation(key_point1, P1, key_point2, P2, wi1, wi2)

			# recalculate weights
			p2x1 = (np.matrix(P1)[2]*np.matrix(X))[0,0]
			p2x2 = (np.matrix(P2)[2]*np.matrix(X))[0,0]

			# breaking point
			if np.abs(wi1-p2x1) <= EPSILON and np.abs(wi2-p2x2) <= EPSILON:
				break
			# reweight equations and solve
			wi1 = p2x1
			wi2 = p2x2
		return X