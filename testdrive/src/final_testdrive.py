import rospy
import numpy as np
import cvxpy as cp

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from camera_data.msg import ReferencePoses

class MPCController:
    def __init__(self, horizon=10):
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/ref_pos', ReferencePoses, self.waypoints_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.waypoints = []
        self.horizon = horizon
        self.dt = 0.1               # time step
        self.odom_pose = None       # position x, y, z, orientation x, y, z, w
        self.odom_twist = None      # linear x, y, z, angular x, y, z
        self.epsilon = 1e-6

    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist

    def waypoints_callback(self, data):
        # if self.odom_pose is None:
        #     rospy.logwarn("Odometry data has not been received yet.")
        #     return
        
        self.waypoints = [(point.x, point.y) for point in data.points]
        self.run_mpc()

    def run_mpc(self):
        if len(self.waypoints) < self.horizon:
            return

        # initial state
        x0, y0, theta0 = self.odom_pose.position.x, self.odom_pose.position.y, self.get_yaw_from_quaternion(self.odom_pose.orientation)
        v0, omega0 = self.odom_twist.linear.x, self.odom_twist.angular.z
        # define variables
        x = cp.Variable(self.horizon)
        y = cp.Variable(self.horizon)
        theta = cp.Variable(self.horizon)
        v = cp.Variable(self.horizon) # linear velocity
        omega = cp.Variable(self.horizon) # angular velocity

        # Precompute cosines and sines
        cos_theta = np.cos(np.linspace(theta0, theta0 + (self.horizon - 1) * self.dt * (omega.value[0] if omega.value is not None else 0), self.horizon))
        sin_theta = np.sin(np.linspace(theta0, theta0 + (self.horizon - 1) * self.dt * (omega.value[0] if omega.value is not None else 0), self.horizon))

        # define the cost function
        cost = 0
        for t in range(self.horizon):
            cost += cp.square(x[t] - self.waypoints[t][0]) + cp.square(y[t] - self.waypoints[t][1])
        for t in range(1, self.horizon - 1):
            cost +=  cp.square(v[t] - v[t-1]) + cp.square(omega[t] - omega[t-1]) # control effort

        # define the constraints
        constraints = []
        constraints += [x[0] == x0, y[0] == y0, theta[0] == theta0, v[0] == v0, omega[0] == omega0]
        for t in range(self.horizon - 1):
            constraints += [
                x[t + 1] == x[t] + self.dt * v[t] * cos_theta[t],
                y[t + 1] == y[t] + self.dt * v[t] * sin_theta[t],
                theta[t + 1] == theta[t] + self.dt * omega[t],
                v[t] >= -self.epsilon,
                v[t] <= 10,
                omega[t] >= -0.1,
                omega[t] <= 0.1
            ]

        # solve the optimization problem
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve()


        # get the control input and publish
        if prob.status == cp.OPTIMAL:
            control_cmd = Twist()
            control_cmd.linear.x = v.value[1]
            control_cmd.angular.z = omega.value[1]
            print("cost: ", prob.value)
            print("vel, ang vel: ", v.value[1] ,omega.value[1])
            # self.pub.publish(control_cmd)
            # print("waypoints\n", self.waypoints)
            # print("x: ", x.value)
            # print("y: ", y.value)

    def get_yaw_from_quaternion(self, q):
        """
        Convert a quaternion into yaw angle (in radians)
        """
        import tf.transformations
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]

if __name__ == "__main__":
    rospy.init_node("MPCController")
    node = MPCController()
    rate = rospy.Rate(50)   # 50 Hz
    while not rospy.is_shutdown():
        rate.sleep()
