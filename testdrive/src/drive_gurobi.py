import os
import sys
import math
import rospy
import numpy as np
import gurobipy as gp
from gurobipy import *
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from camera_data.msg import ReferencePoses
from visualization_msgs.msg import Marker, MarkerArray
from scipy.interpolate import interp1d

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
    def __init__(self, hz=50, horizon=30):
        rospy.Subscriber("/odom", Odometry, self.odom_update)
        rospy.Subscriber('/ref_pos', ReferencePoses, self.waypoints_callback)
        rospy.Subscriber('/command', Int8, self.command_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/transformed_points_marker', MarkerArray, queue_size=10)  # Marker publisher
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
        self.start = True

    def odom_update(self, data):
        self.odom_pose = data.pose.pose
        self.odom_twist = data.twist.twist


    def command_callback(self, data):
        if data.data == 1:
            self.start = True
        elif data.data == 0:
            self.start = False


    def waypoints_callback(self, data):
        self.x0, self.y0, self.theta0 = self.odom_pose.position.x, self.odom_pose.position.y, \
                                        self.get_yaw_from_quaternion(self.odom_pose.orientation)  # world
        # self.local_points = [(point.x, point.y) for point in data.points] # change unit from cm to m
        self.local_points = [(0.01 * point.x, 0.01 * point.y) for point in data.points] # change unit from cm to m
        self.ref = self.fit_quadratic_through_origin(self.local_points)
        # print("ref:  ", self.ref)
        # print("current Position:", self.x0, self.y0, self.theta0)
        # print('local_points:', self.local_points)
        self.global_points = self.local2global(self.ref)
        # print("global:", self.global_points)
        self.publish_points(self.global_points)
        self.run_mpc() if self.start else None

    def spline(self, control_points):
        control_points = np.array(control_points)
        # Extracting x and y coordinates
        x = control_points[:, 0]
        y = control_points[:, 1]
        # Fit a quadratic polynomial (ax^2) with c = 0 and slope = 0 at x = 0
        # We perform the fit on y/x^2 to get the best fit for y = ax^2.
        A = np.vstack([x**2]).T
        a = np.linalg.lstsq(A, y, rcond=None)[0][0]
        # Create the quadratic polynomial function
        def quadratic_poly(x):
            return a * x**2
        # Generate x values for plotting the fitted curve
        x_fit = np.linspace(0, max(x), 500)
        # Generate y values based on the fitted quadratic polynomial
        y_fit = quadratic_poly(x_fit)
        # Combine x_fit and y_fit into a list of [x, y] pairs
        xy_fit = [[x, y] for x, y in zip(x_fit, y_fit)]
        return xy_fit[:50]

    def fit_quadratic_through_origin(self, control_points):
        # Ensure control_points is a numpy array
        control_points = np.array(control_points)
        # Extracting x and y coordinates
        x = control_points[:, 0]
        y = control_points[:, 1]
        # Fit a quadratic polynomial (ax^2 + bx) with c = 0 at (0,0)
        # We solve for coefficients a and b in y = ax^2 + bx
        A = np.vstack([x**2, x]).T
        a = np.linalg.lstsq(A, y, rcond=None)[0][0]
        # Create the quadratic polynomial function
        def quadratic_poly(x):
            return a * x**2 # b * x
        # Generate x values for plotting the fitted curve, starting from 0
        x_fit = np.linspace(0, max(x), 90)
        # Generate y values based on the fitted quadratic polynomial
        y_fit = quadratic_poly(x_fit)
        # Combine x_fit and y_fit into a list of [x, y] pairs
        xy_fit = [[x, y] for x, y in zip(x_fit, y_fit)]
        return xy_fit[::3]
            
    def interpolation(self, reference_points):
        reference_points = [(0.0, 0.0)] + reference_points
        x_coords, y_coords = zip(*reference_points)
        # 각 구간의 거리 계산
        distances = np.sqrt(np.diff(x_coords)**2 + np.diff(y_coords)**2)
        cumulative_distances = np.insert(np.cumsum(distances), 0, 0)  # 누적 거리 계산
        # 첫 번째 reference point 이전까지의 거리를 계산
        first_reference_distance = cumulative_distances[1]
        # 동일한 간격으로 분포된 거리 생성 (첫 번째 reference point 이전까지)
        num_points = len(reference_points)
        equal_spaced_distances = np.linspace(0, first_reference_distance, num_points)
        # 보간 함수 생성 (cumulative distance를 기준으로 x와 y를 각각 보간)
        interp_func_x = interp1d(cumulative_distances, x_coords)
        interp_func_y = interp1d(cumulative_distances, y_coords)
        # 동일한 간격으로 분포된 interpolation points 계산
        interpolated_x = interp_func_x(equal_spaced_distances)
        interpolated_y = interp_func_y(equal_spaced_distances)
        interpolated_points = list(zip(interpolated_x, interpolated_y))
        # 첫 번째 reference point 이전까지만 포함하도록 조정
        interpolated_points = [(x, y) for x, y in interpolated_points if x <= x_coords[1]]
        # 최종 결과 리스트 생성 (interpolated points와 reference points 결합)
        result = interpolated_points# + reference_points[1:]
        return result
    

    def run_mpc(self):
        # define the optimization model
        m = gp.Model()
        m.Params.outputFlag = False
        
        # x, y variables
        x_vars = m.addVars(np.arange(self.horizon), lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="x")
        y_vars = m.addVars(np.arange(self.horizon), lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS, name="y")
        # v variables
        vx_vars = m.addVars(np.arange(self.horizon-1), lb=0.0, ub=1.0,  vtype=GRB.CONTINUOUS, name="v_x")
        vy_vars = m.addVars(np.arange(self.horizon-1), lb=-1.0, ub=1.0,  vtype=GRB.CONTINUOUS, name="v_y")
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
        m.setObjective(gp.quicksum((self.ref[h][0] - x_vars[h])**2 for h in range(self.horizon)) 
                    + gp.quicksum((self.ref[h][1] - y_vars[h])**2 for h in range(self.horizon)) 
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
        # print("Solved (%s)" % status_dict[m.status])

        if m.status == 2:
            print("Objective value: ", m.ObjVal)
            x_vals = m.getAttr('x', x_vars)
            y_vals = m.getAttr('x', y_vars)
            vx_vals = m.getAttr('x', vx_vars)
            vy_vals = m.getAttr('x', vy_vars)
            omgx_vals = m.getAttr('x', omgx_vars)
            omgy_vals = m.getAttr('x', omgy_vars)
            
            theta = 0.0 if np.round(vx_vals[0], 5) == 0 else np.tanh(np.round(vy_vals[0], 5) / np.round(vx_vals[0], 5))

            # print(vx_vals, vy_vals)
            print("vx: ", vx_vals[0], "vy: ", vy_vals[0])
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
    

    def find_closest_reference_set(self):
        # 현재 위치
        current_position = (self.x0, self.y0)

        # 초기화: 가장 작은 거리와 그에 해당하는 리스트를 설정
        min_distance = float('inf')
        closest_set = None

        # 각 reference 세트에 대해 첫 번째 점과의 거리 계산
        for ref_set in self.reference:
            first_point = ref_set[0]
            distance = math.sqrt((first_point[0] - current_position[0]) ** 2 + (first_point[1] - current_position[1]) ** 2)
            
            # 가장 작은 거리를 가진 세트를 찾기
            if distance < min_distance:
                min_distance = distance
                closest_set = ref_set
        
        # 가장 가까운 세트를 반환
        return closest_set


    def publish_points(self, points):
        """
        Publish the transformed points to RViz as a MarkerArray
        """
        marker_array = MarkerArray()
        marker_array.markers = []
        
        for i, point in enumerate(points):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "transformed_points"
            marker.id = i  # Unique ID for each point
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
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
            marker.lifetime = rospy.Duration(20)

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


    def global2local(self, points):
        local_points = []
        cos_theta = np.cos(self.theta0)
        sin_theta = np.sin(self.theta0)
        
        for point in points:
            x_prime = point[0] - self.x0
            y_prime = point[1] - self.y0
            x_local = cos_theta * x_prime + sin_theta * y_prime
            y_local = -sin_theta * x_prime + cos_theta * y_prime
            local_points.append((x_local, y_local))

        return local_points

    def local2global(self, points):
        global_points = []
        cos_theta = np.cos(-self.theta0)
        sin_theta = np.sin(-self.theta0)
        
        for point in points:
            x_prime = cos_theta * point[0] + sin_theta * point[1]
            y_prime = -sin_theta * point[0] + cos_theta * point[1]
            global_points.append((self.x0 + x_prime, self.y0 + y_prime))
            
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