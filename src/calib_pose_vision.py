#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import time
import transformations

from threading import Thread


class TimeoutException(Exception):
    pass
 
ThreadStop = Thread._Thread__stop
 
def timelimited(timeout):
    def decorator(function):
        def decorator2(*args,**kwargs):
            class TimeLimited(Thread):
                def __init__(self,_error= None,):
                    Thread.__init__(self)
                    self._error =  _error
 
                def run(self):
                    try:
                        self.result = function(*args,**kwargs)
                    except Exception,e:
                        self._error = str(e)
 
                def _stop(self):
                    if self.isAlive():
                        ThreadStop(self)
 
            t = TimeLimited()
            t.start()
            t.join(timeout)

            if isinstance(t._error,TimeoutException):
                t._stop()
                raise TimeoutException('timeout for %s' % (repr(function)))
 
            if t.isAlive():
                t._stop()
                raise TimeoutException('timeout for %s' % (repr(function)))
 
            if t._error is None:
                return t.result
 
        return decorator2
    return decorator

@timelimited(1)  # set execution time 1s
def findChessboardCorners(image_gray, board_size):
    # return cv2.findChessboardCorners(image_gray, board_size, None)
    return cv2.findCirclesGrid(image_gray, board_size, flags=cv2.CALIB_CB_ASYMMETRIC_GRID)

class CalibPoseVision:
    def __init__(self):
        rospy.init_node("calib_pose_vision", anonymous=False)
        self.bridge = cv_bridge.CvBridge()
        self.board_size = (4, 11)
        cell_size = 30   # mm
        obj_points = []
        for i in range(self.board_size[1]):
            for j in range(self.board_size[0]):
                obj_points.append([(2*j + i % 2)*cell_size, i*cell_size, 0])
        self.object_points = np.array(obj_points, dtype=np.float32)
        # self.object_points = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        # self.object_points[:,:2] = np.mgrid[0:self.board_size[0], 0:self.board_size[1]].T.reshape(-1,2) * cell_size
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.camera_matrix = np.array([[476.7030836014194, 0.0, 400.5], 
                          [0.0, 476.7030836014194, 400.5], 
                          [0.0, 0.0, 1.0]])
        self.dist_coefs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.axis_object_points = np.array([[0.0, 0.0, 0.0], 
                               [cell_size, 0.0, 0.0], 
                               [0.0, cell_size, 0.0],
                               [0.0, 0.0, cell_size]])
        self.board_pose_pub = rospy.Publisher('board/pose', PoseStamped, queue_size=1)
        self.image_sub = rospy.Subscriber('panda/camera/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('panda/camera/image_post', Image, queue_size=1)


    def image_callback(self, msg):
        start_time = time.time()
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret = None
        try:
            ret, corners = findChessboardCorners(image_gray, self.board_size)
        except TimeoutException as e:
            print("findChessboardCorners timed out!")
        if ret:
            ret_pnp, rotation_vector, translation_vector = cv2.solvePnP(self.object_points, corners, self.camera_matrix, self.dist_coefs)
            if ret_pnp:
                axis_image_points, _ = cv2.projectPoints(self.axis_object_points, rotation_vector, translation_vector, self.camera_matrix, self.dist_coefs)
                cv2.line(image, tuple(axis_image_points[0, 0].astype(int)), tuple(axis_image_points[1, 0].astype(int)), (0, 0, 255), 4)  # BGR
                cv2.line(image, tuple(axis_image_points[0, 0].astype(int)), tuple(axis_image_points[2, 0].astype(int)), (0, 255, 0), 4)
                cv2.line(image, tuple(axis_image_points[0, 0].astype(int)), tuple(axis_image_points[3, 0].astype(int)), (255, 0, 0), 4)
                rotation_mat, rotation_jacobian = cv2.Rodrigues(rotation_vector)
                # print("rotation_mat: %s" % rotation_mat)
                board_pose = PoseStamped()
                board_pose.header.stamp = rospy.get_rostime()
                board_pose.header.frame_id = "servo_vision"
                translation_m = [translation_vector[0, 0]/1000, translation_vector[1, 0]/1000, translation_vector[2, 0]/1000]
                board_pose.pose.position.x = translation_m[0]
                board_pose.pose.position.y = translation_m[1]
                board_pose.pose.position.z = translation_m[2]
                rotation_quaternion = transformations.quaternion_from_matrix(rotation_mat)
                board_pose.pose.orientation.w = rotation_quaternion[0]
                board_pose.pose.orientation.x = rotation_quaternion[1]
                board_pose.pose.orientation.y = rotation_quaternion[2]
                board_pose.pose.orientation.z = rotation_quaternion[3]
                self.board_pose_pub.publish(board_pose)
        end_time = time.time()
        # print("image callback time eplase: %s" % (end_time - start_time))
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
        self.image_pub.publish(image_msg)
        

if __name__ == '__main__':
    calib_pose_vision = CalibPoseVision()
    rospy.spin()



