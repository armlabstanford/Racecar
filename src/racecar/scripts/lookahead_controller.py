import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import numpy as np


class LookAheadController:
    def __init__(self, lookahead_distance, traj_points):
        # Initialize ROS node
        rospy.init_node('lookahead_controller', anonymous=True)

        # Publishers and Subscribers
        self.steer_pub = rospy.Publisher('steer', Float32, queue_size=1)
        rospy.Subscriber("vrpn_client_node/Racecar/pose", PoseStamped, self.pose_callback)

        # Lookahead distance and trajectory points
        self.lookahead_distance = lookahead_distance
        self.traj_points = np.array(traj_points)

        # Initialize variables
        self.current_pose = None

    def pose_callback(self, data):
        # Update current pose
        self.current_pose = np.array([data.pose.position.x, data.pose.position.y])

        if self.current_pose is not None:
            self.control()

    def control(self):
        # Find the closest point to the vehicle on the trajectory
        closest_idx = np.argmin(np.linalg.norm(self.traj_points - self.current_pose, axis=1))

        # Search for the lookahead target point along the trajectory
        for i in range(closest_idx, len(self.traj_points)):
            if np.linalg.norm(self.traj_points[i] - self.current_pose) >= self.lookahead_distance:
                target_point = self.traj_points[i]
                break

        # Calculate the steering angle based on the target point
        steer_angle = np.arctan2(target_point[1] - self.current_pose[1], target_point[0] - self.current_pose[0])

        # Publish the steering angle
        self.steer_pub.publish(steer_angle)

if __name__ == '__main__':
    try:
        # Sample trajectory points
        traj_points = [(0, 0), (1, 0), (2, 1), (3, 1), (4, 0), (5, 0)]
        # Lookahead distance
        lookahead_distance = 0.5

        controller = LookAheadController(lookahead_distance, traj_points)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
