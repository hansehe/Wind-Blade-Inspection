import cv2
import numpy as np
from scipy.linalg import block_diag
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

class KalmanObj(object):
	def __init__(self, dt_average, predict_length):
		'''CONSTRUCTOR'''
		self.k_filter 		= None
		self.dt_average 	= dt_average
		self.predict_length = predict_length
		self.X 				= []
		self.Y 				= []
		self.dX 			= []
		self.dY 			= []

	def resetKalman(self):
		'''
			Description 	:  
			Input			:
			Output 			:
		'''
		self.k_filter 	= None
		self.X 			= []
		self.Y 			= []
		self.dX 		= []
		self.dY 		= []

	def __kill__(self):
		del self

class Kalman():
	def __init__(self):
		'''CONSTRUCTOR'''
		super(Kalman, self).__init__()
		'''
			This kalman filter gives a linear prediction to a signal.
		'''

	def runKalman(self, kalmanObj, new_x, new_y):
		'''
			Description 	:  
			Input			:
			Output 			:
		'''
		kalmanObj.X.append(new_x)
		kalmanObj.Y.append(new_y)
		if kalmanObj.k_filter == None:
			if len(kalmanObj.X) >= kalmanObj.predict_length:
				kalmanObj.k_filter = self.initKalman(kalmanObj.X, kalmanObj.Y, kalmanObj.dt_average)
			if len(kalmanObj.X) > 1:
				pre_dX = kalmanObj.X[-1:][0] - kalmanObj.X[-2:][0]
				pre_dY = kalmanObj.Y[-1:][0] - kalmanObj.Y[-2:][0]
			else:
				pre_dX = new_x
				pre_dY = new_y
		else:
			kalmanObj.k_filter = self.processKalman(kalmanObj.k_filter, new_x, new_y)
			pre_x, pre_dX, pre_y, pre_dY = kalmanObj.k_filter.x
			new_x = pre_x + pre_dX
			new_y = pre_y + pre_dY
		kalmanObj.dX.append(pre_dX)
		kalmanObj.dY.append(pre_dY)
		if len(kalmanObj.X) >= kalmanObj.predict_length:
			kalmanObj.X.pop(0)
			kalmanObj.Y.pop(0)
			kalmanObj.dX.pop(0)
			kalmanObj.dY.pop(0)
		return kalmanObj, [new_x, new_y]

	def initKalman(self, X, Y, dt_average):
		'''
			Description 	:  
			Input			:
			Output 			:
		'''
		k_filter = KalmanFilter(dim_x=4, dim_z=2)
		dt = dt_average #time step in seconds
		k_filter.x = np.array([X[0], X[1]-X[0], Y[0], Y[1]-Y[0]]).T       # initial state (x and y position)
		k_filter.F = np.array([[1, dt, 0, 0],
				              [0, 1, 0, 0 ],
				              [0, 0, 1, dt],
				              [0, 0, 0, 1 ]])    # state transition matrix
		q = Q_discrete_white_noise(dim=2, dt=dt, var=np.cov(X,Y)[0][0]) # process uncertainty
		k_filter.Q = block_diag(q, q)
		k_filter.H = np.array([[1, 0, 0, 0],
							   [0, 0, 1, 0]])    # Measurement function
		                    
		k_filter.R = np.cov(X,Y) # state uncertainty
		P = np.eye(4)
		covar = np.cov(X, Y)
		P[0][0] = covar[0][0]
		P[1][1] = covar[1][0]
		P[2][2] = covar[0][1]
		P[3][3] = covar[1][1]
		k_filter.P = P    # covariance matrix
		return k_filter

	def predictKalman(self, k_filter, meas_x, meas_y, meas_dx, meas_dy, predict_length):
		'''
			Description 	:  
			Input			:
			Output 			:
		'''
		length = len(meas_x)-1
		x = meas_x[length] + meas_dx[length]
		y = meas_y[length] + meas_dy[length]
		for i in range(0, predict_length):
			m = self.processKalman(k_filter, x, y)
			meas_x.append(m[0])
			meas_dx.append(m[1])
			meas_y.append(m[2])
			meas_dy.append(m[3])
			x = m[0] + m[1]
			y = m[2] + m[3]
		return meas_x, meas_y, meas_dx, meas_dy

	def measureKalman(self, k_filter, X, Y):
		'''
			Description 	:  
			Input			:
			Output 			:
		'''
		meas_x = []
		meas_y = []
		meas_dx = []
		meas_dy = []
		for i in range(0, len(X)):
			m = self.processKalman(k_filter, X[i], Y[i])
			meas_x.append(m[0])
			meas_dx.append(m[1])
			meas_y.append(m[2])
			meas_dy.append(m[3])
		return meas_x, meas_y, meas_dx, meas_dy

	def processKalman(self, k_filter, new_x, new_y):
		'''
			Description 	:  
			Input			:
			Output 			:
		'''
		k_filter.predict()
		try:
			k_filter.update([new_x, new_y])
		except:
			pass
		return k_filter #k_filter.x => Returns four values: X, dX, Y, dY

	def __kill__(self):
		'''DESTRUCTOR'''
		super(Kalman, self).__kill__()
		del self

#kalman = Kalman_Filter()
#kalman.test()