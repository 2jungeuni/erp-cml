# ros
import rospy
import serial
from std_msgs.msg import UInt8, Bool, Int32

# built-in
import struct
import numpy as np

PORT = '/dev/ttyUSB0'                   # port number
# PORT = '/dev/erp42'                   # port number
BAUDRATE = 115200                       # baudrate
STAR_BITS = "535458"

class ERPHandler:
    def __init__(self):
        rospy.loginfo("erp handler port: %s", PORT)
        rospy.loginfo("erp handler baudrate: %s", BAUDRATE)
        self.serial = serial.Serial(PORT, BAUDRATE)
        rospy.loginfo("serial %s connected", PORT)
        

        self.e_stop_pub = rospy.Publisher("/state_e_stop", Bool, queue_size=3)
        self.gear_pub = rospy.Publisher("/state_gear", UInt8, queue_size=3)
        self.speed_pub = rospy.Publisher("/state_speed", UInt8, queue_size=3)
        self.steer_pub = rospy.Publisher("/state_steer", Int32, queue_size=3)
        self.brake_pub = rospy.Publisher("/state_brake", UInt8, queue_size=3)
        self.encoder_pub = rospy.Publisher("/state_encoder", Int32, queue_size=3)
        self.flag = True

        
    def erp_to_pc(self):
        packet = self.serial.read(18)
        if self.flag:
            print("[RECEIVE] first packet = {0}".format(packet.hex()))
            self.flag = False
        if packet.hex().find(STAR_BITS) != 0:
            end, data = packet.hex().split(STAR_BITS)
            packet = bytes.fromhex(STAR_BITS + data + end)
        packet = struct.unpack('<BBBBBBhhBiBBB', packet)

        control_mode = packet[3]  # 필요하면 사용하고..? 근데 굳이 필요할까 싶음
        e_stop = bool(packet[4])
        gear = packet[5]
        speed = packet[6]
        steer = -packet[7]
        brake = packet[8]
        encoder = -np.int32(packet[9])
        
        print("[RECEIVE] steer: ", str(round(steer / 71, 2)).ljust(5),
              "degree | speed: ", str(speed).ljust(3),
              "kph | brake: ", str(brake).ljust(3),
              " | gear: ", str(gear).ljust(1),
              " | encoder: ", str(encoder))
        
        # Publish messages
        e_stop_msg = Bool()
        e_stop_msg.data = e_stop
        self.e_stop_pub.publish(e_stop_msg)
        gear_msg = UInt8()
        gear_msg.data = gear
        self.gear_pub.publish(gear_msg)
        speed_msg = UInt8()
        speed_msg.data = speed
        self.speed_pub.publish(speed_msg)
        steer_msg = Int32()
        steer_msg.data = steer
        self.steer_pub.publish(steer_msg)
        brake_msg = UInt8()
        brake_msg.data = brake
        self.brake_pub.publish(brake_msg)
        encoder_msg = Int32()
        encoder_msg.data = encoder
        self.encoder_pub.publish(encoder_msg)



if __name__ == "__main__":
    rospy.init_node("receiver")
    node = ERPHandler()
    rate = rospy.Rate(50)   # Publish for 50 Hz
    while not rospy.is_shutdown():
        node.erp_to_pc()
        rate.sleep()
