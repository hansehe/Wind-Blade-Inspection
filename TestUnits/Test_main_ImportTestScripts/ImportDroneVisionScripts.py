'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

def ImportDroneVisionScripts():
	'''
	 @brief Import test modules from DroneVision scripts.
	'''
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision import Test_DroneVision
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_hardware.Test_LaserLink import Test_LaserLink
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_hardware.Test_CameraLink import Test_CameraLink
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_hardware.Test_PtGrey.Test_PtGrey import Test_PtGrey
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_hardware.Test_PtGrey.Test_PtGreyJordens import Test_PtGreyJordens
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_hardware.Test_PtGrey.Test_PtGreyFLIR import Test_PtGreyFLIR
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_hardware.Test_ImageLink import Test_ImageLink
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_hardware.Test_imageTools import Test_imageTools
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_hardware.Test_RecordFrames import Test_RecordFrames
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_hardware.Test_VideoLink import Test_VideoLink
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_hardware.Test_PinControl import Test_PinControl
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_hardware.Test_PyQtImage import Test_PyQtImage
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_CameraCalibration.Test_CameraCalibration import Test_CameraCalibration
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_CameraCalibration.Test_StereoCalibration import Test_StereoCalibration
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_CameraCalibration.Test_StereoVision import Test_StereoVision
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_featureDetection.Test_BlobDetector.Test_BlobDetector import Test_BlobDetector
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_featureDetection.Test_generalDetectors.Test_detectCorners import Test_detectCorners
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_featureDetection.Test_generalDetectors.Test_detectEdges import Test_detectEdges
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_featureDetection.Test_generalDetectors.Test_detectLines import Test_detectLines
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_featureDetection.Test_PointDetection.Test_PointDetection import Test_PointDetection
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_featureDetection.Test_BlobScaleDetector.Test_BlobScaleDetector import Test_BlobScaleDetector
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_frameTools.Test_frameTools import Test_frameTools
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_Heading.Test_EdgeHeading import Test_EdgeHeading
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_Heading.Test_Heading import Test_Heading
	from TestUnits.Test_src.Test_DroneVision.Test_DroneVision_src.Test_imgProcessing.Test_stereopsis.Test_FeatureStereopsis import Test_FeatureStereopsis

	DroneVisionSripts = {
		'DroneVision': Test_DroneVision,
		'LaserLink': Test_LaserLink,
		'CameraLink': Test_CameraLink,
		'PtGrey': Test_PtGrey,
		'PtGreyJordens': Test_PtGreyJordens,
		'PtGreyFLIR': Test_PtGreyFLIR,
		'ImageLink': Test_ImageLink,
		'imageTools': Test_imageTools,
		'RecordFrames': Test_RecordFrames,
		'VideoLink': Test_VideoLink,
		'PinControl': Test_PinControl,
		'PyQtImage': Test_PyQtImage,
		'CameraCalibration': Test_CameraCalibration,
		'StereoCalibration': Test_StereoCalibration,
		'StereoVision': Test_StereoVision,
		'BlobDetector': Test_BlobDetector,
		'detectCorners': Test_detectCorners,
		'detectEdges': Test_detectEdges,
		'detectLines': Test_detectLines,
		'PointDetection': Test_PointDetection,
		'BlobScaleDetector': Test_BlobScaleDetector,
		'frameTools': Test_frameTools,
		'EdgeHeading': Test_EdgeHeading,
		'Heading': Test_Heading,
		'FeatureStereopsis': Test_FeatureStereopsis
	}

	return DroneVisionSripts