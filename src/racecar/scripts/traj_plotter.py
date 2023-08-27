import rospy
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
import threading, time

# Global variable to store the last 100 poses
pose_list = []
folder_path = '/home/racecar/Documents/racecar_ws/src/Racecar/src/racecar/scripts/'
traj_name = 'star50.txt'
with open(folder_path+traj_name, 'rb') as f:
    traj = np.loadtxt(f, delimiter=' ')
    traj = traj[:,1:]*1.5 + np.array([0,-0.5])

def pose_callback(pose_msg):
    global pose_list

    # Append new pose to list
    new_pose = [-pose_msg.pose.position.x, pose_msg.pose.position.z]
    pose_list.append(new_pose)
    print(new_pose)

    # Keep only the last 100 poses
    pose_list = pose_list[-300:]
    time.sleep(0.1)

def plot_trajectory():
    global pose_list
    global traj

    plt.figure()
    while not rospy.is_shutdown():
        # Convert to NumPy array for easier slicing
        local_pose_list = list(pose_list)  # Create a local copy to avoid race conditions
        pose_array = np.array(local_pose_list)

        # Plotting
        plt.clf()  # Clear the previous plot
        plt.plot(pose_array[:, 0], pose_array[:, 1], 'b-')
        plt.plot(-traj[:,0],traj[:,1],'r')
        plt.axis('equal')
        plt.xlim((-2,2))
        plt.ylim((-2.5,1.5))
        plt.grid()
        plt.pause(0.1)  # Pause for a short period to allow the plot to update

if __name__ == '__main__':
    rospy.init_node('pose_plotter', anonymous=True)

    # Create a separate thread for plotting
    plot_thread = threading.Thread(target=plot_trajectory)
    plot_thread.start()

    # Subscribe to the pose topic
    rospy.Subscriber("/vrpn_client_node/Racecar/pose", PoseStamped, pose_callback, queue_size=1)

    # Keep the node running
    rospy.spin()
