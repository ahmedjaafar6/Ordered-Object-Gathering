#!/usr/bin/env python3

import rospy
from roboflow import Roboflow
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


bridge = CvBridge()
img = None

#roboflow
def box_det_model():
	server_url = "http://localhost:9001"
	api_key = "eKWE2qQqtdmGmaRLgZCf"
	model_name = "3d-geom-shape-detector"
	rf = Roboflow(server_url, api_key)
	
	pred = rf.predict(img, model=model_name, confidence=0.1)

	return pred
	

#subscriber function
def camera_callback(data):
	global img
	img = bridge.imgmsg_to_cv2(data, "bgr8")




if __name__ == '__main__':
	try:
		rospy.init_node('roboflow', anonymous=True)
		cam_sub = rospy.subsriber("/camera/color/image_raw",Image, camera_callback)
		rospy.sleep(0.5)	
	
		pred = box_det_model()
		print("pred: ", pred)

	except rospy.ROSInterruptException:
		print("ROS Interrupted!")






		
