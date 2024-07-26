import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from camera_data.msg import ReferencePoses
import numpy as np
import cvxpy as cp

class MPCController:
    def __init__(self, horizon=10):
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/reference_pos', ReferencePoses, sself.waypoints_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.waypoints = []
        self.horizon = horizon
        self.dt = 0.1 # time step


    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist


    def waypoints_callback(self, data):
        self.waypoints = [(point.x, point.y) for point in data.points]
        self.run_mpc()


    def run_mpc(self):
        if len(self.waypoints) < self.horizon:
            return
        
        # initial state
        x0, y0, theta0 = 0.0, 0.0, 0.0 # TODO: change

        # define variables
        x = cp.Variable(self.horizon)
        y = cp.Variable(self.horizon)
        theta = cp.Variable(self.horizon)
        v = cp.Variable(self.horizon - 1) # linear velocity
        omega = cp.Variable(self.horizon - 1) # angular velocity

        # define the cost function
        cost = 0
        for t in range(self.horizon):
            cost += cp.square(x[t] - self.waypoints[t][0]) + cp.square(y[t] - self.waypoints[t][1])
        for t in range(self.horizon - 2):
            cost += cp.square(v[t] - v[t-1]) + cp.square(omega[t] - omega[t-1]) # control effort
        
        # define the constraints
        constraints = []
        constraints += [x[0] == x0, y[0]==y0, theta[0]==theta0]
        for t in range(self.horizon - 1):
            constraints += [
                x[t + 1] == x[t] + self.dt * v[t] * cp.cos(theta[t]),
                y[t + 1] == y[t] + self.dt * v[t] * cp.sin(theta[t]),
                theta[t + 1] == theta[t] + self.dt * omega[t]
            ]

        # solve the optimization problem
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve()

        # get the contorl input and publish
        if prob.status == cp.OPTIMAL:
            control_cmd = Twist()
            control_cmd.linear.x = v.value[0]
            control_cmd.angular.z = omega.value[0]
            self.pub.publish(control_cmd)



if __name__ == "__main__":
    rospy.init_node("MPCController")
    node = MPCController()
    rate = rospy.Rate(50)   # 50 Hz
    while not rospy.is_shutdown():
        rate.sleep()
