
import roslib
import rospy
import math
import tf
import argparse
import cv2
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point
from pyzbar import pyzbar

def callback(msg):
    global center_u,center_v,focal
    center_u = info_msg.P[2]
    center_v = info_msg.P[6]
    focal = info_msg.P[0]


if __name__ == '__main__':
    print("Goodbye, World!")
    double x = 0.0;
    double y = 0.0;
    double w = 0.0;
    double h = 0.0;
    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", required=True,
	help="Desktop/mtrx5700_major")
    args = vars(ap.parse_args())

    rospy.init_node('position_qr_code')

    pub = rospy.Publisher('ben', Odometry, queue_size=50)

    # spin() simply keeps python from exiting until this node is stopped

    rospy.Subscriber('realsense/depth/camera_info',CameraInfo,callback)
    rospy.Subscriber('realsense/depth/image_raw',Image,callback2)
    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        try:
            # load the input image
            image = cv2.imread(args["image"])
            # find the barcodes in the image and decode each of the barcodes
            barcodes = pyzbar.decode(image)

        except rospy.ROSInterruptException:
            # exit if there is any interruption from ROS
            return
        except KeyboardInterrupt:
            # exit if there is any interrupt from the keyboard (e.g. Ctrl+c)
            return
        # loop over the detected barcodes
        for barcode in barcodes:
            # extract the bounding box location of the barcode and draw the
            # bounding box surrounding the barcode on the image
            (x, y, w, h) = barcode.rect
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

        intrinsics_inv = np.array([[focal, 0, center_u, 0],
            [0, focal, center_v, 0],
            [0, 0, 1, 0]])

        extrinsics_inv = np.array([[0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 1]])

        result = [[0],
         [0],
         [0],
         [1]]

        x_index = x/2
        y_index = y/2
        qr_center_x = ((u-center_u)/focal)*depth_x
        qr_center_y = ((v-center_v)/focal)*depth_y

        #z1,z2,x1,x2,y1,y2 are some points within the qr code
        x_orientation = math.atan((z1-z2)/(y1-y2)))
        y_orientation = math.atan((z1-z2)/(x1-x2)))
        z_orientation = math.atan((x1-x2)/(y1-y2)))

        ben = Odometry()
        ben.header.frame_id = "baxter"
        ben.child_frame_id = "qr_code"
        ben_quat = tf.transformations.quaternion_from_euler(x_orientation, y_orientation, z_orientation)
        ben.pose.pose = Pose(Point(qr_center_x, qr_center_y, z_depth), Quaternion(*ben_quat))

        pub.publish(ben)

        rate.sleep()
