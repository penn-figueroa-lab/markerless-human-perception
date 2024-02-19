import numpy as np
import rospy
from std_msgs.msg import String


def callback(pose):
    pose = json.loads(pose.data)
    # For each person
    for p in range(len(pose["value"])):
        for kp in labels:
            if pose["value"][p][kp]["acc"] > 0.2:
                x.append(pose["value"][p][kp]["x"])
                x.append(pose["value"][p][kp]["y"])
                x.append(pose["value"][p][kp]["z"])
            else:
                x.append(np.nan)
                x.append(np.nan)
                x.append(np.nan)
        x = np.array(x)
        y = completor(x)
        print(x,y,"\n\n\n\n")
        
    
    res = np.dot(np.array(p3d),Rt[:3,:3].transpose())+Rt[:3,3].transpose()

    out_msg = "TBD" # json.dumps({camera_info.header.stamp.to_sec() : res})
    pub.publish(out_msg)
    
def main():
    global pub, Rt
    rospy.init_node("wrist_tracker")  
    
    # Initiate listener
    rospy.Subscriber("hpe/poses", String, callback)
    
    # Setup publisher
    pub = rospy.Publisher('hpe/ee_goal', String, queue_size=10)
    
    Rt = np.load("/home/rmhri/markerless-human-perception/src/RT.npy")
    
    # Synchronize
    rospy.spin()

if __name__ == "__main__":
    
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node Not Executed")
