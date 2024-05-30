import rospy
from std_msgs.msg import UInt8, Bool, Int32
from geometry_msgs.msg import PointStamped
import numpy as np


CAR_LENGTH = 105  # 차량의 뒷바퀴 중간부터 앞바퀴 중간까지의 거리 (cm)


class ERPtestdrive:
    def __init__(self):
        rospy.Subscriber("/reference_pos", PointStamped, self.testdrive)
        rospy.Subscriber("/state_e_stop", Bool, self.e_stop_update)
        rospy.Subscriber("/state_encoder", Int32, self.encoder_update)
        self.speed_pub = rospy.Publisher("/speed", UInt8, queue_size=3)
        self.brake_pub = rospy.Publisher("/brake", UInt8, queue_size=3)
        self.steer_pub = rospy.Publisher("/steer", Int32, queue_size=3)
        self.e_stop = 0
        self.encoder = 0
        self.initial_flag = True


    def e_stop_update(self, data):
        self.e_stop = data.data


    def encoder_update(self, data):  # testing without camera
        self.encoder = data.data
        if self.initial_flag:
            self.initial_encoder = data.data
            self.initial_flag = False

        

    def testdrive(self, data):
        brake = 100
        speed = 0
        steer = 0
        if self.e_stop == 0: # if emergency stop is not working
            # if self.encoder <= self.initial_encoder + 4000: # while car drives about 2.4m
            brake = 0
            speed = 20
            target_x = data.point.x  # 차량의 원점 기준 얼마나 앞의 지점을 기준으로 삼을지
            delta_y = -1 * data.point.y
            Ld_power = target_x**2 + delta_y**2
            steer = np.rad2deg(np.arctan(2 * CAR_LENGTH * delta_y / Ld_power))
            print("steer(deg): ", steer)

        speed_msg = UInt8()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)
        steer_msg = Int32()
        steer_conv = int(max(-2000, min(2000, round(steer * 71))))
        # change degree to integer and clipping
        steer_msg.data = steer_conv
        self.steer_pub.publish(steer_msg)
        brake_msg = UInt8()
        brake_msg.data = brake
        self.brake_pub.publish(brake_msg)

        

if __name__ == "__main__":
    rospy.init_node("testdrive")
    node = ERPtestdrive()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()