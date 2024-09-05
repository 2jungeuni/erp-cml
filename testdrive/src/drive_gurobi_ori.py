import os
import sys
import rospy
import numpy as np
import gurobipy as gp
from gurobipy import *
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from camera_data.msg import ReferencePoses
from visualization_msgs.msg import Marker
from collections import deque

# status dictionary
status_dict = {1: "loaded",
               2: "optimal",
               3: "infeasible",
               4: "infeasible and unbounded",
               5: "unbounded",
               6: "cut off",
               7: "iteration limit",
               8: "node limit",
               9: "time limit",
               10: "solution limit",
               11: "interrupted",
               12: "numeric",
               13: "suboptimal",
               14: "in progress",
               15: "user objective limit",
               16: "work limit",
               17: "memory limit"}


class MPCController:
    def __init__(self, hz=50, horizon=10):
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/ref_pos', ReferencePoses, self.waypoints_callback)
        rospy.Subscriber('/command', Int8, self.command_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/transformed_points_marker', Marker, queue_size=10)  # Marker publisher
        self.local_points = None
        self.global_points = None
        self.horizon = horizon
        self.hz = hz
        self.dt = 1/hz              # time step = 1/50 = 20ms
        self.odom_pose = None       # position x, y, z, orientation x, y, z, w
        self.odom_twist = None      # linear x, y, z, angular x, y, z
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0
        self.marker_id = 0        
        self.angles = np.linspace(0, np.pi, 100)
        self.our_points = [(0.01*i, 0.01*i) for i in range(500)] + [(5 - 0.01*i, 5 + 0.01*i) for i in range(500)]
        self.start = False

    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist


    def waypoints_callback(self, data):
        self.x0, self.y0, self.theta0 = self.odom_pose.position.x, self.odom_pose.position.y, \
                                        self.get_yaw_from_quaternion(self.odom_pose.orientation)  # world
        # print("current position:",self.x0, self.y0, self.theta0)

        # self.global_points = [(point.x, point.y) for point in data.points]
        self.local_points = [(point.x, point.y) for point in data.points]
        # self.local_points = self.local_points(self.global_points, self.x0, self.y0, self.theta0)


        # if self.marker_id < 50:
        #     self.local_points = [(0.1 * h, 0) for h in range(self.horizon)]
        # else:
        #     self.local_points = [(0.1 * h, 0) for h in range(self.horizon)]
        # self.local_points = [(point.x, point.y) for point in data.points]
        # self.global_points = self.transform_points(self.local_points, self.x0, self.y0, self.theta0)
        # self.global_points = self.our_points[self.marker_id: self.marker_id + self.horizon]
        # self.local_points = self.global2local(self.global_points, self.x0, self.y0, self.theta0)
        # print(self.global_points)
        self.global_points = self.local2global(self.local_points, self.x0, self.y0, self.theta0)
        self.publish_global_points()
        if self.start:
            print("running")
            self.run_mpc()
        else:
            return

    def command_callback(self, data):
        if data.data == 1:
            self.start = True
        elif data.data == 0:
            self.start = False
        print(self.start)
            

    def run_mpc(self):
        # if len(self.local_points) < self.horizon:
        #     return
        
        # define the optimization model
        m = gp.Model()
        m.Params.outputFlag = False
        
        # x, y variables
        x_vars = m.addVars(np.arange(self.horizon), lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="x")
        y_vars = m.addVars(np.arange(self.horizon), lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="y")
        # v variables
        vx_vars = m.addVars(np.arange(self.horizon-1), lb=0.0, ub=1.0,  vtype=GRB.CONTINUOUS, name="v_x")
        vy_vars = m.addVars(np.arange(self.horizon-1), lb=-2.5, ub=2.5,  vtype=GRB.CONTINUOUS, name="v_y")
        # omega variables
        omgx_vars = m.addVars(np.arange(self.horizon), lb=-GRB.INFINITY, ub=GRB.INFINITY,  vtype=GRB.CONTINUOUS, name="omg_x")
        omgy_vars = m.addVars(np.arange(self.horizon), lb=-GRB.INFINITY, ub=GRB.INFINITY,  vtype=GRB.CONTINUOUS, name="omg_y")

        # define the constraints
        # Constraint 1: set initial points
        cons1_1 = m.addConstr(x_vars[0] == 0.0)
        cons1_2 = m.addConstr(y_vars[0] == 0.0)         # for local planning
        # cons1_1 = m.addConstr(x_vars[0] == self.x0)
        # cons1_2 = m.addConstr(y_vars[0] == self.y0)   # for global planning


        # Constraint 2: dynamics
        cons2_1 = m.addConstrs(x_vars[h+1] == x_vars[h] + self.dt * vx_vars[h] for h in range(self.horizon - 1))
        cons2_2 = m.addConstrs(y_vars[h+1] == y_vars[h] + self.dt * vy_vars[h] for h in range(self.horizon - 1))
        cons2_3 = m.addConstrs(vx_vars[h+1] == vx_vars[h] + self.dt * omgx_vars[h] for h in range(self.horizon - 2))
        cons2_4 = m.addConstrs(vy_vars[h+1] == vy_vars[h] + self.dt * omgy_vars[h] for h in range(self.horizon - 2))
        
        # set objective function
        m.setObjective(gp.quicksum((self.local_points[h][0] - x_vars[h])**2 for h in range(self.horizon)) 
                    + gp.quicksum((self.local_points[h][1] - y_vars[h])**2 for h in range(self.horizon)) 
                    + gp.quicksum((vx_vars[h+1]- vx_vars[h])**2 for h in range(self.horizon - 2)) 
                    + gp.quicksum((vy_vars[h+1]- vy_vars[h])**2 for h in range(self.horizon - 2)) 
                    + gp.quicksum((omgx_vars[h+1]- omgx_vars[h])**2 for h in range(self.horizon - 2)) 
                    + gp.quicksum((omgy_vars[h+1]- omgy_vars[h])**2 for h in range(self.horizon - 2)), GRB.MINIMIZE)
        # m.setObjective(gp.quicksum((self.transformed_points[h][0] - x_vars[h])**2 for h in range(self.horizon)) 
        #                + gp.quicksum((self.transformed_points[h][1] - y_vars[h])**2 for h in range(self.horizon)), GRB.MINIMIZE)

        m._xvars = x_vars
        m._yvars = y_vars
        m._vxvars = vx_vars
        m._vyvars = vy_vars
        m._omgxvars = omgx_vars
        m._omgyvars = omgy_vars
        m.optimize()

        # status
        # print(m.getObjective())
        print("Solved (%s)" % status_dict[m.status])

        if m.status == 2:
            print("Objective value: ", m.ObjVal)
            x_vals = m.getAttr('x', x_vars)
            y_vals = m.getAttr('x', y_vars)
            vx_vals = m.getAttr('x', vx_vars)
            vy_vals = m.getAttr('x', vy_vars)
            omgx_vals = m.getAttr('x', omgx_vars)
            omgy_vals = m.getAttr('x', omgy_vars)
            
            theta = 0.0 if np.round(vx_vals[0], 5) == 0 else np.tanh(np.round(vy_vals[0], 5) / np.round(vx_vals[0], 5))

            print("Waypoints:  ", self.local_points)
            print(vx_vals, vy_vals)
            print("speed = ", np.sqrt(vx_vals[0]**2 + vy_vals[0]**2))
            print("theta = ", theta)
            # print("vx")
            # print(vx_vals)
            # print("vy")
            # print(vy_vals)
            # print("omgx")
            # print(omgx_vals)
            # print("omgy")
            # print(omgy_vals)

            control_cmd = Twist()
            control_cmd.linear.x = np.round(vx_vals[0], 5)
            control_cmd.angular.z = theta
            self.pub.publish(control_cmd)
        else:
            sys.exit()


    def publish_global_points(self):
        """
        Publish the transformed points to RViz as a Marker
        """
        for point in self.global_points:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "transformed_points"
            marker.id = self.marker_id  # Unique ID for each point
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            # marker.pose.position.x = (point[0] - self.x0) * 0.01 + self.x0
            # marker.pose.position.y = (point[1] - self.y0) * 0.01 + self.y0
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.marker_pub.publish(marker)
            self.marker_id += 1  # Increment the marker ID for the next point


    def global2local(self, points, x0, y0, theta0):
        local_points = []
        cos_theta = np.cos(theta0)
        sin_theta = np.sin(theta0)
        
        for point in points:
            x_prime = point[0] - x0
            y_prime = point[1] - y0
            x_local = cos_theta * x_prime + sin_theta * y_prime
            y_local = -sin_theta * x_prime + cos_theta * y_prime
            local_points.append((x_local, y_local))

        return local_points

    def local2global(self, points, x0, y0, theta0):
        global_points = []
        cos_theta = np.cos(-theta0)
        sin_theta = np.sin(-theta0)
        
        for point in points:
            x_prime = cos_theta * point[0] + sin_theta * point[1]
            y_prime = -sin_theta * point[0] + cos_theta * point[1]
            global_points.append((x0 + x_prime, y0 + y_prime))
            
        return global_points


    def get_yaw_from_quaternion(self, q):
        """
        Convert a quaternion into yaw angle (in radians)
        """
        import tf.transformations
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return euler[2]


if __name__ == "__main__":
    hz = 50
    rospy.init_node("MPCController")
    node = MPCController(hz)
    rate = rospy.Rate(hz)   # 50 Hz
    while not rospy.is_shutdown():
        rate.sleep()