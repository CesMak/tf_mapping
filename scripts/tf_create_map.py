#!/usr/bin/env python  

# To use this program start it right after the tf_mapping node
# make sure that the camera does not see a marker at the first image
# If this script is started as well point first the camera to just aruco marker 0
# Then such that the camera sees the aruco marker 
import roslib
roslib.load_manifest('tf_mapping')
import rospy
import tf
import rospkg
from alfons_msgs.msg import ArucoInfo
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

marker_ids  = []
center_x_px = []
center_y_px = []
bridge = CvBridge()

def get_dis_to_closest_corner(center_x, center_y):
    max_x = 640
    max_y = 480
    min_dis_x = center_x
    min_dis_y = center_y
    if (max_x - center_x) < center_x:
        min_dis_x = max_x - center_x
    
    if (max_y - center_y) < center_y:
        min_dis_y = max_y - center_y
    
    if min_dis_x < min_dis_y:
        return min_dis_x
    else:
        return min_dis_y

def store_image(x, y, z, qx, qy, qz, qw, id):
    # use fields image, marker_ids
    print "store id: "+str(id)
    print marker_ids
    j = -1
    for i in xrange(0, len(marker_ids)):
        if marker_ids[i] == id:
            j=i
            break
    if(j<0):
        "caution error!!!!"
    center_x = center_x_px[j]
    center_y = center_y_px[j]

    dis = get_dis_to_closest_corner(center_x, center_y)
    print "I saw id: "+str(id)+" with its center at: "+str(center_x)+", "+str(center_y)+" min dis: "+str(dis)


    roi = cv_image[center_y-dis:center_y+dis, center_x-dis:center_x+dis]
    resized_image = cv2.resize(roi, (256, 256)) 
    rospack = rospkg.RosPack()
    round_factor = 4
    scale_factor = 1
    x = round(x, round_factor)*scale_factor
    y = round(y, round_factor)*scale_factor
    z = round(z, round_factor)*scale_factor
    qx = round(qx, 4)
    qy = round(qy, 4)
    qz = round(qz, 4)
    qw = round(qw, 4)
    img_name = str(x)+"_"+str(y)+"_"+str(z)+"_"+str(qx)+"_"+str(qy)+"_"+str(qz)+"_"+str(qw)+"_id:"+str(id)+".jpg"
    path = rospack.get_path('rviz_pics')+"/map/"+img_name
    cv2.imwrite(path, resized_image)

    print "I wrote the image at:"+path



def img_cb(msg):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8") # type: numpy.ndarray
    except CvBridgeError as e:
        print(e)
    
def aruco_info_cb(msg):
    global marker_ids
    global center_x_px
    global center_y_px
    marker_ids = []
    center_x_px = []
    center_y_px = []
    for i in xrange(0, len(msg.marker_ids)):
        marker_ids.append(msg.marker_ids[i])
    
    for i in xrange(0, len(msg.center_x_px)):
        center_x_px.append(msg.center_x_px[i])
    
    for i in xrange(0, len(msg.center_y_px)):
        center_y_px.append(msg.center_y_px[i])

if __name__ == '__main__':
    rospy.init_node('tf_create_map')
    rospy.loginfo("Started tf_create_map %s", '')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    # Subscriber
    rospy.Subscriber("/aruco_list", ArucoInfo, aruco_info_cb)
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_cb)

    printed_number = 2
    max_number     = 6  # if this number is reached program is closed file is written.
    result_file ="<?xml version=\"1.0\"?> \n \n <launch> \n"

    while not rospy.is_shutdown():
        tf_to_str = '/marker_globe_'+str(printed_number)
        id_nr = ""
        list_frames = listener.getFrameStrings()
        if (printed_number<=max_number):
            #print ('Calculate tf from /marker_globe_1 --> '+tf_to_str+" :")
            try:
                #listener.waitForTransform('/marker_globe_1', tf_to_str, rospy.Time(), rospy.Duration(2.0))

                (trans, rot) = listener.lookupTransform('/marker_globe_1', tf_to_str, rospy.Time(0))

                # get id of tf_to_str
                for p in list_frames:
                    if "id_" in p:
                        (trans_p, rot_p) = listener.lookupTransform('/marker_globe_1', p, rospy.Time(0))
                        if trans_p[0] == trans[0]:
                            id_nr = p
                            print "I detected this id: "+ id_nr+" for "+tf_to_str
                if(len(id_nr)>1):
                    str_output =  (str(trans[0])+", "+str(trans[1])+", "+str(trans[2])+",     "+str(rot[0])+", "+str(rot[1])+", "+str(rot[2])+", "+str(rot[3])+" ")
                    str_output = "<node name=\"initpos_to_globe_"+str(printed_number)+"_map_tf_pub\" pkg=\"tf\" type=\"static_transform_publisher\" args=\""+str_output+" /world  /"+id_nr+"_map 100 \"/>"
                    result_file = result_file+"\n \n"+str_output
                    printed_number = printed_number+1

                    store_image(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3], int(id_nr[3:]))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                #print exception
                continue
            rate.sleep()
        else:
            cv2.destroyAllWindows()
            print "max number reached end!"
            result_file = result_file+"\n \n </launch>"
            rospack = rospkg.RosPack()
            path = rospack.get_path('tf_mapping')+"/launch/use_map.launch"

            f = open(path, "w")
            f.write(result_file)
            print result_file
            print "Finish file is written to: tf_mapping/launch/use_map.launch"
            rospy.signal_shutdown("Finish file is written to: tf_mapping/launch/use_map.launch")