#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import math
import cv2
from skimage.color import rgb2hsv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np



def convert_depth_image(ros_image):
	bridge = CvBridge()
	global depth_array
	try:
		depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
		depth_array = np.array(depth_image, dtype=np.float32)
	except CvBridgeError:
		pass

def convert_color_image(ros_image):
	bridge = CvBridge()
	global color_array
	try:
		color_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
		hsv_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2HSV)
		color_image = hsv_image
		color_array = np.array(color_image, dtype=np.float32)
	except CvBridgeError:
		pass


def get_angle(dists):
	global depth_array, boxes_seen
	global box_angles
	box_angles = []
	for box in range(len(boxes_seen)):
		center_idx = np.array(depth_array.shape)/2
		center_col = center_idx[1]
		opp = center_col - boxes_seen[box][1]
		print(dists[box])
		ang = np.arcsin(opp/(dists[box]))
		box_angles.append(ang)
	return box_angles

def get_depth(depth_arr):
	global boxes_seen
	global box_depths
	box_depths = []
	for box in boxes_seen:
		box_depths.append(depth_arr[int(box[0]), int(box[1])])
	return box_depths

def get_color_loc(ros_image):	
	global boxes_seen
	boxes_seen = []
	for color in color_order:
		# col_thresh = ros_image.shape[0] * ros_image.shape[1] * 0.01
		col_thresh = ros_image.shape[0] * ros_image.shape[1] * 0.002
		print("Looking for a", color, "box.", "Thresh:", col_thresh)
		# get the blue box
		if color == "blue":
			lower_blue = np.array([85, 100, 20])
			upper_blue = np.array([100, 255,255])
			blue_mask = cv2.inRange(ros_image, lower_blue, upper_blue)
			blue_loc = np.argwhere(blue_mask > 0)
			print("num_blue_pixels:", len(blue_loc))
			if len(blue_loc) > col_thresh:
				print("Found a Blue Box!")
				blue_center = np.mean(blue_loc, axis=0)
				boxes_seen.append(blue_center)
		
		elif color == "yellow":
			lower_yellow = np.array([15,100,20])
			upper_yellow = np.array([45,255, 255])
			yellow_mask = cv2.inRange(ros_image, lower_yellow, upper_yellow)
			yellow_loc = np.argwhere(yellow_mask > 0)
			print("num_yellow_pixels:", len(yellow_loc))
			if len(yellow_loc) > col_thresh:
				print("Found a Yellow Box!")
				yellow_center = np.mean(yellow_loc, axis=0)
				boxes_seen.append(yellow_center)
		
		elif color == "green":
			lower_green = np.array([50, 100,20])
			upper_green = np.array([80, 255,255])
			green_mask =  cv2.inRange(ros_image, lower_green, upper_green)
			green_loc = np.argwhere(green_mask > 0)
			print("num_green_pixels:", len(green_loc))
			if len(green_loc) > col_thresh:
				green_center = np.mean(green_loc, axis=0)
				boxes_seen.append(green_center)
		
		elif color == "purple":
			lower_purple = np.array([140, 100, 20])
			upper_purple = np.array([155, 255, 255])
			purple_mask = cv2.inRange(ros_image, lower_purple, upper_purple)
			purple_loc = np.argwhere(purple_mask > 0)
			print("num_purple_pixels:", len(purple_loc))
			if len(purple_loc) > col_thresh:
				print("Found a Purple Box!")
				purple_center = np.mean(purple_loc, axis=0)
				boxe_seen.append(purple_center)
	
	
	

def get_depth_to(x,y, depth_arr):
	h_max = depth_arr.shape[0]
	v_max = depth_arr.shape[1]

	if x < h_max - 1:
		subset_hmax = x + 1
	else:
		subset_hmax = x
	if y < y_max - 1:
		subset_vmax = y + 1
	else:
		subset_vmax = y
	
	if x > 0:
		subset_hmin = x - 1
	else:
		subset_hmin = x
	if y > 0:
		subset_vmin = y - 1
	else:
		subset_vmin = y


	depth_subset = depth_arr[subset_hmin:subset_hmax, subset_vmin:subset_vmax]



class PoseMsg:
    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

# 5 Hz setting
eps_angular = .01
eps_d = .02
max_linear_velocity = 3
max_angular_velocity = 20
Kpa = 100
Kpv = 30
rospy_rate = 10


#last_pose = None


def poseCallback(message):
    global last_pose
    curr_odom = message
    theta = euler_from_quaternion([message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w])[2]
    last_pose = PoseMsg(curr_odom.pose.position.x, curr_odom.pose.position.y, theta)


def turn(waypoint):
    global rate, publisher, last_pose, eps_angular, Kpa
    # compute target angle based on current coord and next waypoint coord
    curr_pose = last_pose
    target_theta = math.atan2(waypoint[1] - curr_pose.y, waypoint[0] - curr_pose.x)

    # loop until correct angle has been achieved
    while not rospy.is_shutdown():
        # compute angular distance theta_err
        curr_pose = last_pose
        theta = curr_pose.theta
        theta_err = theta - target_theta
        theta_err = (theta_err + math.pi) % math.tau - math.pi    # get shortest angle

        #rospy.loginfo('\t{}, {}, {}'.format(theta_err, theta, target_theta))
        if abs(theta_err) < eps_angular:
            break

        # compute angular_velocity based on Kpa * theta_err
        angular_velocity = -Kpa * theta_err
        angular_velocity = max(-max_angular_velocity, min(max_angular_velocity, angular_velocity))

        # send angular_velocity command to turtle
        cmd = Pose2D()
        cmd.theta = angular_velocity
        publisher.publish(cmd)

        rate.sleep()

    # send stop command to turtle
    cmd = Pose2D()
    cmd.theta = 0
    publisher.publish(cmd)
    return


def move(waypoint):
    global rate, publisher, last_pose, eps_angular, eps_d, Kpa, Kpv
    # loop until correct position and angle has been achieved
    while not rospy.is_shutdown():
        # compute target angle based on current coord and next waypoint coord
        curr_pose = last_pose
        target_theta = math.atan2(waypoint[1] - curr_pose.y, waypoint[0] - curr_pose.x)

        # compute angular distance theta_err and distance error d_err
        curr_pose = last_pose
        theta = curr_pose.theta
        theta_err = theta - target_theta
        theta_err = (theta_err + math.pi) % math.tau - math.pi    # get shortest angle
        d_err = math.sqrt((curr_pose.y - waypoint[1])**2 + (curr_pose.x - waypoint[0])**2)

        # if angle is greater than pi/2, set angle in opposite direction and move backwards instead
        if abs(theta_err) > math.pi/2:
            theta_err = theta_err % math.tau - math.pi
            d_err = -d_err

        if abs(d_err) < eps_d: 
            break

        # compute linear_velocity output based on Kpv * d_err
        linear_velocity = Kpv * d_err
        linear_velocity = max(-max_linear_velocity, min(max_linear_velocity, linear_velocity))
        #rospy.loginfo('\t{}, {}'.format(d_err, linear_velocity))
        # compute angular_velocity based on Kpa * theta_err
        angular_velocity = -Kpa * theta_err
        angular_velocity = max(-max_angular_velocity, min(max_angular_velocity, angular_velocity))

        # send linear_velocity and angular_velocity command to turtle
        cmd = Pose2D()
        cmd.y = linear_velocity
        cmd.theta = angular_velocity
        publisher.publish(cmd)
        
        rate.sleep()

    # send stop command to turtle
    cmd = Pose2D()
    cmd.x = 0
    cmd.theta = 0
    publisher.publish(cmd)
    return


def moveback(waypoint):
    global rate, publisher, last_pose, eps_angular, eps_d, Kpa, Kpv
    # loop until correct position and angle has been achieved
    while not rospy.is_shutdown():
        # compute target angle based on current coord and next waypoint coord
        curr_pose = last_pose
        target_theta = math.atan2(waypoint[1] - curr_pose.y, waypoint[0] - curr_pose.x)

        # compute angular distance theta_err and distance error d_err
        curr_pose = last_pose
        theta = curr_pose.theta
        theta_err = theta - target_theta
        theta_err = (theta_err + math.pi) % math.tau - math.pi    # get shortest angle
        d_err = math.sqrt((curr_pose.y - waypoint[1])**2 + (curr_pose.x - waypoint[0])**2)

        # if angle is greater than pi/2, set angle in opposite direction and move backwards instead
        if abs(theta_err) > math.pi/2:
            theta_err = theta_err % math.tau - math.pi
            d_err = -d_err

        if abs(d_err) < eps_d: 
            break

        # compute linear_velocity output based on Kpv * d_err
        linear_velocity = Kpv * d_err
        linear_velocity = max(-max_linear_velocity, min(max_linear_velocity, linear_velocity))
        #rospy.loginfo('\t{}, {}'.format(d_err, linear_velocity))
        # compute angular_velocity based on Kpa * theta_err
        angular_velocity = -Kpa * theta_err
        angular_velocity = max(-max_angular_velocity, min(max_angular_velocity, angular_velocity))

        # send linear_velocity and angular_velocity command to turtle
        cmd = Pose2D()
        cmd.y = linear_velocity
        cmd.theta = angular_velocity
        publisher.publish(cmd)
        
        rate.sleep()

    # send stop command to turtle
    cmd = Pose2D()
    cmd.x = 0
    cmd.theta = 0
    publisher.publish(cmd)
    return


def push(targets):

    for i in range(len(targets)):
        targetx, targety, direction = targets[i]
        target = (targetx, targety)

        rospy.loginfo('Turn {}: {}'.format(i+1, target))
        if direction == 1:
            turn(target)
        
        
        cmd = Pose2D()
        cmd.x = 0
        cmd.theta = 0
        publisher.publish(cmd)

        rospy.loginfo('Move {}: {}'.format(i+1, target))
        if direction == 1:
            move(target)
        else:
            moveback(target)
                
        cmd = Pose2D()
        cmd.x = 0
        cmd.theta = 0
        publisher.publish(cmd)


def planner(boxes):
    targets = []
    robot_pose = last_pose


    box_idx = 0
    for box in boxes:
        dist, angle = box
        dist /= 1000
        
        print('DIST:',dist)
        print('ANGLE:',angle)
        print('robot_pose.x:',robot_pose.x)
        print('robot_pose.y:',robot_pose.y)
        print('robot_pose.theta:',robot_pose.theta)
        box_pos = PoseMsg()
        
        box_pos.x = dist * math.cos(angle)+math.sqrt(0.1**2-0.02**2)
        box_pos.y = dist * math.sin(angle)+.02
        
        box_dist = math.sqrt(box_pos.x**2+box_pos.y**2)
        box_angle = math.atan2(box_pos.y,box_pos.x)
        
        box_dist -=.03
        
        box_pos.x = box_dist * math.cos(box_angle)
        box_pos.y = box_dist * math.sin(box_angle)+0.01
        
        print('box_pos.x:',box_pos.x)
        print('box_pos.y:',box_pos.y)
        
        targets.append((0.1,0.0,1))

        #targets.append((box_pos.x-0.1,box_pos.y))
        targets.append((box_pos.x,box_pos.y,1))

        targets.append((0.1,0.0+box_idx*0.15,1))
        targets.append((-0.15,0.0+box_idx*0.15,1))
        targets.append((0.0,0.0+box_idx*0.15,-1))

        box_idx += 1
    
    return targets


def get_boxes():
    #global depth_array, color_array
    #boxes = []
    #print(color_array)
    #row, col = get_color_loc(color_array)
    #dist = get_depth(row,col, depth_array)
    #angle = get_angle(col, dist)
    #pos = [dist, angle]
    #boxes.append(pos)
    #print(pos)
    #return(boxes)
    global depth_array, color_array, boxes_seen, box_depths
    boxes = []
    get_color_loc(color_array)
    dists = get_depth(depth_array)
    angles = get_angle(dists)
    pos = [dists, angles]
    boxes = [(dists[i],angles[i]) for i in range(len(dists))]
    return boxes


if __name__ == '__main__':
    global rate, publisher
    global last_pose, depth_array, color_array, boxes_seen, box_depths, box_angles, color_order
    last_pose = None
    depth_array = None
    color_array = None
    color_order = ["blue", "yellow", "purple", "green"]
    try:
        rospy.init_node('triton_box_push', anonymous=True)

        position_topic = '/slam_out_pose'
        subscriber = rospy.Subscriber(position_topic, PoseStamped, poseCallback)
        cmd_vel_topic = '/drive_cmd'
        publisher = rospy.Publisher(cmd_vel_topic, Pose2D, queue_size=10)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback=convert_depth_image, queue_size=10)
        rospy.Subscriber("camera/color/image_raw", Image, callback=convert_color_image, queue_size=10)
        rate = rospy.Rate(rospy_rate)
        
        # targets = [(1.00,-0.5),(1.00,1.50),(1.00,0.0),(0.0,0.0)]

        # get list of box locations from box detection algorithm
        # boxes = [(1.00,-0.5),(1.50,0.00),(0.75,0.3)]
        
        print('before while')
        while last_pose is None or depth_array is None or color_array is None: # wait until initial pose has been set
            print('last_pose type:', type(last_pose))
            print('depth_array type:', type(depth_array))
            print('color_array type:', type(color_array))
            rate.sleep()
        
        print('after while')
        
        boxes = get_boxes()
        # boxes = [(0.5,-0.2),(0.6,0.3)]
        targets = planner(boxes)
        
        push(targets)
    except rospy.ROSInterruptException as e: 
        rospy.logerr(e)
        exit(1)
