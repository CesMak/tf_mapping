#!/usr/bin/env python  
import roslib
roslib.load_manifest('tf_mapping')
import rospy
import tf
import rospkg
import numpy as numpy

if __name__ == '__main__':
    rospy.init_node('tf_navigate')
    rospy.loginfo("Started tf_navigate %s", '')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    printed_number = 10
    while not rospy.is_shutdown():
        list_frames = listener.getFrameStrings()
        try:
            sum_x = 0.0
            sum_y = 0.0
            sum_z = 0.0
            trans_res = [0]*3
            rot3      = [0]*4
            rot3[3]   = 1
            number_detected_marker_id = 0
            for element in list_frames:
                if "marker_id" in element: #marker was found in current frame
                    id_number = int(element[9:])
                    map_to_str      = "/id_"+str(id_number)+"_map" # e.g.: /id_1_map
                    camera_to_str   = "/"+element # e.g.:/marker_id1

                    time_diff =  abs(  (listener.getLatestCommonTime(camera_to_str, '/usb_cam').to_sec()) - ((rospy.get_time()))  )
                    if(time_diff<0.5): # in secs
                        #listener.waitForTransform(camera_to_str, "/usb_cam", rospy.Time().now(), rospy.Duration(4.0))
                        (trans_id_word, rot_id_world) = listener.lookupTransform('/world', map_to_str, rospy.Time(0))
                        (trans_id_camera, rot_id_camera) = listener.lookupTransform(camera_to_str, '/usb_cam', rospy.Time(0))

                        trans1_mat = tf.transformations.translation_matrix(trans_id_word)
                        rot1_mat   = tf.transformations.quaternion_matrix(rot_id_world)
                        trans2_mat = tf.transformations.translation_matrix(trans_id_camera)
                        rot2_mat   = tf.transformations.quaternion_matrix(rot_id_camera)

                        mat1 = numpy.dot(trans1_mat, rot1_mat) # ist 4x4 H Matrix
                        mat2 = numpy.dot(trans2_mat, rot2_mat)
                        mat3 = numpy.dot(mat1, mat2)

                        trans3 = tf.transformations.translation_from_matrix(mat3)
                        sum_x = sum_x + trans3[0]
                        sum_y = sum_y + trans3[1]
                        sum_z = sum_z + trans3[2]
                        number_detected_marker_id = number_detected_marker_id+1
                        rot3 = tf.transformations.quaternion_from_matrix(mat3) 

                        br = tf.TransformBroadcaster()
                        br.sendTransform(trans3, rot3, rospy.Time.now(), "usb_cam_world_"+str(id_number), "world")
            if number_detected_marker_id>0:
                trans_res[0] = sum_x/number_detected_marker_id
                trans_res[1] = sum_y/number_detected_marker_id
                trans_res[2] = sum_z/number_detected_marker_id
                bb = tf.TransformBroadcaster()
                bb.sendTransform(trans_res, rot3, rospy.Time.now(), "usb_cam_world", "world")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "inside except"
            continue
        rate.sleep()
        # else:
        #     rospy.signal_shutdown("Kill")