# ros
import rospy
import serial
from erp_com.msg import Status, Cmd

# built-in
import struct
import numpy as np

PORT = '/dev/ttyUSB0'                   # port number
BAUDRATE = 115200                       # baudrate
ERPS = serial.Serial(PORT, BAUDRATE)
STAR_BITS = "535458"

class ERPHandler:
    def __init__(self):
        rospy.loginfo("erp handler port: %s", PORT)
        rospy.loginfo("erp handler baudrate: %s", BAUDRATE)
        self.serial = serial.Serial(PORT, BAUDRATE)
        rospy.loginfo("serial %s connected", PORT)

        self.msg_pub = rospy.Publisher("/erp42_status", Status, queue_size=3)
        self.flag = True

        
    def erp_to_pc(self):
        packet = self.serial.read(18)
        if self.flag:
            print("[RECEIVE] first packet = {0}".format(packet.hex()))
            self.flag = False
        if packet.hex().find(STAR_BITS) != 0:
            end, data = packet.hex().split(STAR_BITS)
            packet = bytes.fromhex(STAR_BITS + data + end)
        msg = self.Packet2ErpMsg(packet)

        print("[RECEIVE] steer: ", str(round(msg.steer / 71, 2)).ljust(5),
              "degree | speed: ", str(msg.speed).ljust(3),
              "kph | brake: ", str(msg.brake).ljust(3),
              " | gear: ", str(msg.gear).ljust(1),
              " | encoder: ", str(msg.encoder))
        self.msg_pub.publish(msg)

    
    def Packet2ErpMsg(self, byte):
        packet = struct.unpack('<BBBBBBhhBiBBB', byte)
        msg = Status()
        msg.control_mode = packet[3]
        msg.e_stop = bool(packet[4])
        msg.gear = packet[5]
        msg.speed = packet[6]
        msg.steer = -packet[7]
        msg.brake = packet[8]
        msg.encoder = -np.int32(packet[9])
        msg.alive = packet[10]
        return msg


if __name__ == "__main__":
    rospy.init_node("erp_to_pc", anonymous=False)
    node = ERPHandler()
    rate = rospy.Rate(50)   # 50 Hz
    while not rospy.is_shutdown():
        node.erp_to_pc()
        rate.sleep()
