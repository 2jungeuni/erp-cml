# ros
import rospy
import serial
from std_msgs.msg import Int32
from erp_com.msg import Status, Cmd

# built-in
import time
import struct
import numpy as np

PORT = '/dev/ttyUSB0'  # port number
BAUDRATE = 115200   # baudrate
ERPS = serial.Serial(PORT, BAUDRATE)

STAR_BITS = "535458"

def Packet2ErpMsg(byte):
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


class ERPHandler:
    def __init__(self):
        rospy.init_node("erp_to_pc", anonymous=True)

        rospy.loginfo("erp handler port: %s", PORT)
        rospy.loginfo("erp handler baudrate: %s", BAUDRATE)
        self.serial = serial.Serial(PORT, BAUDRATE)

        rospy.loginfo("serial %s connected", PORT)
        self.alive = 0
        self.packet = Cmd()
        self.packet.e_stop = False
        self.flag = True

        self.msg_pub = rospy.Publisher("/erp42_status",
                                       Status,
                                       queue_size=3)

    def receive_packet(self):
        packet = self.serial.read(18)
        if self.flag:
            print("[RECEIVE] first packet = {0}".format(packet.hex()))
            self.flag = False
        if packet.hex().find(STAR_BITS) != 0:
            end, data = packet.hex().split(STAR_BITS)
            packet = bytes.fromhex(STAR_BITS + data + end)
        msg = Packet2ErpMsg(packet)
        print("[RECEIVE] steer: ", msg.steer, " | speed: ", msg.speed, " | brake: ", msg.brake, " | gear: ", msg.gear)
        self.msg_pub.publish(msg)


if __name__ == "__main__":
    handler = ERPHandler()
    rate = rospy.Rate(50)   # 50 hz
    while not rospy.is_shutdown():
        handler.receive_packet()