import sys
from openni import openni2, utils

openni2.initialize()

dev = openni2.Device.open_any()
ir = dev.create_ir_stream()

__author__ = 'grant@iowntheinter.net'
import numpy as np
import matplotlib.pyplot as plt
from primesense import openni2

import cv2

from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoRequest, SetCameraInfoResponse


openni2.initialize()  # can also accept the path of the OpenNI redistribution
dev = openni2.Device.open_any()
# print dev.get_sensor_info(openni2.SENSOR_DEPTH)


logging = True


def logger(*data):
    if (logging):
        print(data)


import rospy, yaml
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image





class depth_node:
    def frameLoop(self):
        bridge = CvBridge()
        shot_idx = 0
        depth_stream = dev.create_depth_stream()
        depth_stream.start()
        ir_stream = dev.create_ir_stream()
        ir_stream.start()
        while True:
            logger("frame")
            frame = depth_stream.read_frame()
            frame_data = frame.get_buffer_as_uint16()
            depth_array = np.ndarray((frame.height, frame.width), dtype=np.uint16,
                                     buffer=frame_data) / 10000.  # 0-10000mm to 0.-1.
            # cv2.imshow('Depth',depth_array)
            ir_frame = ir_stream.read_frame()
            ir_frame_data = ir_frame.get_buffer_as_uint16()
            ir_array = np.ndarray((ir_frame.height, ir_frame.width), dtype=np.uint16, buffer=ir_frame_data).astype(
                np.float32)
            # cv2.imshow('IR', ir_array / ir_array.max())
            shot_idx += 1
            cv_depth = None
            cv_ir = None

            try:
                cv_depth = bridge.cv2_to_imgmsg((depth_array * 255).astype(np.uint8))  # , "bgr8")
            except CvBridgeError as e:
                logger(e)

            try:
                cv_ir = bridge.cv2_to_imgmsg((ir_array / ir_array.max()))  # , "bgr8")
            except CvBridgeError as e:
                logger(e)

            if cv_depth is not None:
                self.dpub.publish(cv_depth)
            if cv_ir is not None:
                self.ipub.publish(cv_ir)

    def drive(self):
        self.ipub = rospy.Publisher("/ir/image_raw", Image, queue_size=1)
        self.dpub = rospy.Publisher("/depth/image_raw", Image, queue_size=1)
        self.dsvc = CameraInfoServiceImpl("depth")
        self.isvc = CameraInfoServiceImpl("ir")

        print rospy.get_published_topics()
        try:
            self.frameLoop()
        except KeyboardInterrupt:
            logger("goodbye")

    def __init__(self):
        self.ipub=None
        self.dpub=None
        self.dsvc=None
        self.isvc=None





class CameraInfoServiceImpl:
    def handle_info_req(self, req):
        print("got req " + str(req.camera_info))
        self.data = req.camera_info
        self.infopub = CameraInfoPublisher('camera/' + self.name, self.data)
        return {"success":True}

    def __init__(self, name):
        self.infopub = None
        self.name = name
        self.data = CameraInfo()
        self.s = rospy.Service("/" + name + "/set_camera_info", SetCameraInfo, self.handle_info_req)


class CameraInfoPublisher:
    # Callback of the ROS subscriber.

    def callback(self, data):
        cam_info_org = data
        self.cam_info.header = cam_info_org.header
        self.publish()

    def __init__(self, camera_name, camera_data=None):
        if camera_data is None:
            file_name = '/core/data/calib/depth/' + camera_name + '/ost.yaml'
            self.cam_info = parse_yaml(file_name)
        else:
            self.cam_info = camera_data

        self.left_cam_info_org = 0
        self.right_cam_info_org = 0

        topic = "/" + camera_name + "/camera_info"
        rospy.Subscriber(camera_name + "/camera_info", CameraInfo, self.callback)

        self.pub = rospy.Publisher(topic, CameraInfo)

    def publish(self):
        '''
        now = rospy.Time.now()
        self.left_cam_info.header.stamp = now
        self.right_cam_info.header.stamp = now
        '''
        self.pub.publish(self.cam_info)



def parse_yaml(filename):
    stream = file(filename, 'r')
    calib_data = yaml.load(stream)
    cam_info = CameraInfo()
    cam_info.width = calib_data['image_width']
    cam_info.height = calib_data['image_height']
    cam_info.K = calib_data['camera_matrix']['data']
    cam_info.D = calib_data['distortion_coefficients']['data']
    cam_info.R = calib_data['rectification_matrix']['data']
    cam_info.P = calib_data['projection_matrix']['data']
    cam_info.distortion_model = calib_data['distortion_model']
    return cam_info




if __name__ == '__main__':
    rospy.init_node('depth_driver')
    d = depth_node()
    d.drive()
    while not rospy.is_shutdown():
        rospy.sleep(rospy.Duration(.1))
