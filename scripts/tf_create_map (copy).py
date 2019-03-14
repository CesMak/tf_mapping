#!/usr/bin/env python  
import roslib
roslib.load_manifest('tf_mapping')
import rospy
import tf
import rospkg

if __name__ == '__main__':
    rospy.init_node('tf_create_map')
    rospy.loginfo("Started tf_create_map %s", '')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
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



            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                #print exception
                continue
            rate.sleep()
        else:
            print "max number reached end!"
            result_file = result_file+"\n \n </launch>"
            rospack = rospkg.RosPack()
            path = rospack.get_path('tf_mapping')+"/launch/use_map.launch"

            f = open(path, "w")
            f.write(result_file)
            print result_file
            print "Finish file is written to: tf_mapping/launch/use_map.launch"
            rospy.signal_shutdown("Finish file is written to: tf_mapping/launch/use_map.launch")