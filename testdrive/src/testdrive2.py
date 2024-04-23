# ros
import rospy
from std_msgs.msg import UInt8, Bool, Int32
from geometry_msgs.msg import PointStamped


class ERPtestdrive:
    def __init__(self):
        rospy.Subscriber("/current_pos", PointStamped, self.testdrive)
        rospy.Subscriber("/state_e_stop", Bool, self.e_stop_update)
        rospy.Subscriber("/state_encoder", Int32, self.encoder_update)
        # rospy.Subscriber("/state_gear", UInt8, self.gear_callback)
        # rospy.Subscriber("/state_speed", UInt8, self.speed_callback)
        # rospy.Subscriber("/state_steer", Int32, self.steer_callback)
        # rospy.Subscriber("/state_brake", UInt8, self.brake_callback)  # 일단 안써서 뺌
        self.speed_pub = rospy.Publisher("/speed", UInt8, queue_size=3)
        self.brake_pub = rospy.Publisher("/brake", UInt8, queue_size=3)
        self.steer_pub = rospy.Publisher("/steer", Int32, queue_size=3)
        self.e_stop = 0
        self.encoder = 0


    def e_stop_update(self, data):
        self.e_stop = data.data


    def encoder_update(self, data):  # testing without camera
        # print("a")
        self.encoder = data.data
        brake = 100
        speed = 0
        steer = 0
        print("encoder: ", self.encoder)
        if self.e_stop == 0: # if emergency stop is not working
            if self.encoder <= -200: # while car drives about 1.2m
                brake = 0
                speed = 20
                steer = 10

        speed_msg = UInt8()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)
        steer_msg = Int32()
        steer_conv = max(-2000, min(2000, round(steer * 71)))
        # change degree to integer and clipping
        steer_msg.data = steer_conv
        self.steer_pub.publish(steer_msg)
        brake_msg = UInt8()
        brake_msg.data = brake
        self.brake_pub.publish(brake_msg)

        

    def testdrive(self, data):
        brake = 100
        speed = 0
        steer = 0
        if self.e_stop == 0: # if emergency stop is not working
            if self.encoder <= 100: # while car drives about 1.2m
                error = data.point.y
                brake = 0
                speed = 20
                steer = 0
        speed_msg = UInt8()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)
        steer_msg = Int32()
        steer_conv = max(-2000, min(2000, round(steer * 71)))
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