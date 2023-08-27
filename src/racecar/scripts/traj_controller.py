#!/home/racecar/mambaforge/envs/racecar/bin/python3
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class PDController:
    def __init__(self, Kp, Kd, lookahead_distance=0.5):
        # Initialize
        rospy.init_node('pd_controller')
        rospy.Subscriber("Position", PoseStamped, self.pose_callback)
        rospy.Subscriber("Enable_controller", Float32, self.ena_callback)
        self.steer_pub    = rospy.Publisher('steer', Float32, queue_size=1)
        self.throttle_pub = rospy.Publisher('throttle', Float32, queue_size=1)

        self.Kp = Kp
        self.Kd = Kd
        self.lookahead_distance = lookahead_distance  # Added lookahead distance

        self.enable = False
        self.enable_time = 0.0
        self.enable_threshold = 0.1  # maximum 100ms without enable signal

        self.error_prev = 0.0
        self.error_dot  = 0.0

        # Define line segments for the trajectory
        self.line_segments = [((0, 0), (1, 1)), 
                              ((1, 1), (2, 0)), 
                              ((2, 0), (3, 1))]

    def find_lookahead_point(self, current_position):
        # Implement logic to find the lookahead point
        # that the robot should aim for.
        
        lookahead_point = None  # Replace with your logic to find lookahead point
        
        return lookahead_point

    def calculate_error(self, current_position, lookahead_point):
        # Calculate the lateral error between the robot's position
        # and the lookahead point
        error = np.linalg.norm(np.array(current_position) - np.array(lookahead_point))
        return error

    def pose_callback(self, data):
        current_position = (data.pose.position.x, data.pose.position.y)

        # Find the lookahead point
        lookahead_point = self.find_lookahead_point(current_position)
        
        # If no more segments are left or no lookahead point is found, you might want to stop the robot.
        if lookahead_point is None:
            # Implement stopping logic here
            return

        # Calculate error based on the lookahead point
        error = self.calculate_error(current_position, lookahead_point)
        
        self.error_dot  = error - self.error_prev
        self.error_prev = error
        steer_ctrl      = self.Kp * error + self.Kd * self.error_dot

        # Publish control input as a steering command
        self.steer_pub.publish(steer_ctrl)

    def ena_callback(self, data):
        self.enable = data.data > 0.5
        self.enable_time = data.header.stamp

if __name__ == '__main__':
    controller = PDController(Kp=1.0, Kd=0.1, lookahead_distance=0.5)
    rospy.spin()
