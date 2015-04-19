#! /usr/bin/python

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('get_transform')

    tf_listener = tf.TransformListener()
    rate = rospy.Rate(10)
    while True:
        trans, rot = None, None
        try:
            (trans, rot) = tf_listener.lookupTransform('/camera_rgb_optical_frame', '/external_cam', rospy.Time(0))
            print trans
            print tf.transformations.euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Error'

        rate.sleep()

    
