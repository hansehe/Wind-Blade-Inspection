'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''
import pickle, cv2

def keypoints_to_list(keypoints):
	'''
	 @brief Dump keypoints to list

	 @param keypoints

	 @return keypoint_list
	'''
	keypoint_list = []
	for point in keypoints:
		keypoint = (point.pt, point.size, point.angle, point.response, point.octave, point.class_id)
		keypoint_list.append(keypoint)
	return keypoint_list

def list_to_keypoints(keypoint_list):
	'''
	 @brief Load keypoints from list

	 @param keypoint_list

	 @return keypoints
	'''
	keypoints = []
	for point in keypoint_list:
		keypoint = cv2.KeyPoint(x=point[0][0], y=point[0][1], _size=point[1], _angle=point[2], _response=point[3], _octave=point[4], _class_id=point[5])
		keypoints.append(keypoint)
	return keypoints

def keypoints_to_pickle(keypoints):
	'''
	 @brief Dump keypoints to pickle

	 @param keypoints

	 @return keypoint_pickle
	'''
	keypoint_list 		= keypoints_to_list(keypoints)
	keypoints_pickle 	= pickle.dumps(keypoint_list)
	return keypoints_pickle

def pickle_to_keypoints(keypoints_pickle):
	'''
	 @brief Load keypoints from pickle

	 @param keypoints_pickle

	 @return keypoints
	'''
	keypoint_list 	= pickle.loads(keypoints_pickle)
	keypoints 		= list_to_keypoints(keypoint_list)
	return keypoints