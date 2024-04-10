import rospy
from geometry_msgs.msg import PoseStamped

def topic_to_txt(pose_topic, output_file):
    
    rospy.loginfo(f"Recording messages from topic {pose_topic} to {output_file}")
    rospy.loginfo("When recording is done, clean exit node by pressing ctrl + c")

    msgs_recv = 0 # Counter

    # Open csv-file (throw exception if it exists)
    with open(output_file, 'w') as txt_file:

        txt_file.write("# timestamp tx ty tz qx qy qz qw\n")
        
        def callback_pose(data : PoseStamped):
            x = data.pose.pose.position.x
            y = data.pose.position.y
            z = data.pose.position.z
            qx = data.pose.orientation.x
            qy = data.pose.orientation.y
            qz = data.pose.orientation.z
            qw = data.pose.orientation.w
            timestamp = data.header.stamp.to_time()
            
            txt_file.write(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")

            nonlocal msgs_recv
            msgs_recv += 1
            
            rospy.logdebug(f"Received {msgs_recv} msgs")
            #print(f"Stored {msgs_recv} msgs", end='\r')

        # Specify subscribers
        rospy.Subscriber(pose_topic, PoseStamped, callback_pose, queue_size=500)
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


def main():
    rospy.init_node('orb_slam3_to_txt')
    
    pose_topic = rospy.get_param("/orb_slam3_to_txt/pose_topic")
    output_file = rospy.get_param("/orb_slam3_to_txt/output_file")
    
    topic_to_txt(pose_topic, output_file)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass