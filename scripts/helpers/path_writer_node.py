import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from typing import Union

def topic_to_txt(pose_topic, msg_type, output_file):
    
    rospy.loginfo(f"Recording messages from topic {pose_topic} to {output_file}")
    rospy.loginfo("When recording is done, clean exit node by pressing ctrl + c")

    msgs_recv = 0 # Counter

    # Open csv-file (throw exception if it exists)
    with open(output_file, 'w') as txt_file:

        txt_file.write("# timestamp tx ty tz qx qy qz qw\n")
        
        def callback_pose(data):
            # Get the pose and header from the message type
            pose, header = get_pose_and_header(data, msg_type).values()
            
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w
            timestamp = header.stamp.to_time()
            
            txt_file.write(f"{timestamp} {x} {y} {z} {qx} {qy} {qz} {qw}\n")

            nonlocal msgs_recv
            msgs_recv += 1
        
        rospy.Subscriber(pose_topic, msg_type, callback_pose, queue_size=500)
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

def get_pose_and_header(data : Union[PoseWithCovarianceStamped, PoseStamped, Path, Odometry], 
                        msg_type : str):
    # Extract the pose and header from the message
    if isinstance(msg_type, PoseWithCovarianceStamped):
        return {"pose": data.pose.pose,
                "header": data.header}
        
    elif isinstance(msg_type, PoseStamped):
        return {"pose": data.pose,
                "header": data.header}
            
    elif isinstance(msg_type, Path):
        return {"pose": data.poses[-1].pose,
                "header": data.poses[-1].header}
        
    elif isinstance(msg_type, Odometry):
        return {"pose": data.pose.pose,
                "header": data.header}

    else:
        rospy.logerr(f"Unknown message type: {msg_type}")
        return


def get_message_type(msg_type_name : str):
    # Map the message type string to the corresponding ROS message class
    message_type_mapping = {
        "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
        "PoseStamped": PoseStamped,
        "Path": Path,
        "Odometry": Odometry,
    }

    # Get the corresponding message class from the mapping
    msg_type = message_type_mapping.get(msg_type_name)

    # Check if the message type is valid
    if msg_type is None:
        raise ValueError(f"Invalid message type: {msg_type_name}")

    return msg_type


def main():
    rospy.init_node('path_writer_node')
    
    pose_topic = rospy.get_param("/path_writer_node/pose_topic")
    msg_type_name = rospy.get_param("/path_writer_node/msg_type")
    output_file = rospy.get_param("/path_writer_node/output_file")
    
    topic_to_txt(pose_topic, get_message_type(msg_type_name), output_file)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass