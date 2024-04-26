import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path

def topic_to_txt(path_topic, output_file):
    
    rospy.loginfo(f"Recording messages from topic {path_topic} to {output_file}")
    rospy.loginfo("When recording is done, clean exit node by pressing ctrl + c")

    msgs_recv = 0 # Counter

    # Open csv-file (throw exception if it exists)
    with open(output_file, 'w') as txt_file:

        txt_file.write("# timestamp tx ty tz qx qy qz qw\n")
        
        def callback_path(data : Path):
            # Get the newest pose
            pose = data.poses[-1]
            
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            qx = pose.pose.orientation.x
            qy = pose.pose.orientation.y
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w
            timestamp = pose.header.stamp.to_time()
            
            txt_file.write(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")

            nonlocal msgs_recv
            msgs_recv += 1
            
            rospy.logdebug(f"Received {msgs_recv} msgs")
            #print(f"Stored {msgs_recv} msgs", end='\r')

        # Specify subscribers
        rospy.Subscriber(path_topic, Path, callback_path, queue_size=500)
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


def main():
    rospy.init_node('vins_to_txt')
    
    path_topic = rospy.get_param("/vins_to_txt/path_topic")
    output_file = rospy.get_param("/vins_to_txt/output_file")
    
    topic_to_txt(path_topic, output_file)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
